#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from robocon_interfaces.msg import ZoneState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.goal_sub = self.create_subscription(String, '/planner/goal_location', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/controller/waypoints', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.extrusion_pub = self.create_publisher(JointTrajectory, '/extrusion_controller/joint_trajectory', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        self.zone_sub = self.create_subscription(ZoneState, '/current_zone', self.zone_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_zone = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.stored_kfs_count = 0
        
        self.cb_group = ReentrantCallbackGroup()
        self.climb_srv = self.create_service(Trigger, '/trigger_climb_block', self.climb_trigger_callback, callback_group=self.cb_group)
        self.pick_srv = self.create_service(Trigger, '/trigger_pick_kfs', self.pick_trigger_callback, callback_group=self.cb_group)
        self.explore_srv = self.create_service(Trigger, '/trigger_explore_next_block', self.explore_trigger_callback, callback_group=self.cb_group)
        self.approach_srv = self.create_service(Trigger, '/trigger_approach_adjacent', self.approach_adjacent_callback, callback_group=self.cb_group)
        
        # Prefer the src file for live editing without rebuilds
        ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/locations.yaml"
        if os.path.exists(ws_src_path):
            self.yaml_path = ws_src_path
        else:
            self.yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'locations.yaml')
            
        self.get_logger().info(f"Planner Node started. Using locations from: {self.yaml_path}")

    def zone_callback(self, msg):
        self.current_zone = msg

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def goal_callback(self, msg):
        self.publish_location_path(msg.data)
        
    def publish_location_path(self, location):
        self.get_logger().info(f"Generating path for location: {location}...")
        
        # Reload YAML dynamically so live edits apply immediately!
        try:
            with open(self.yaml_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load locations.yaml: {e}")
            return
            
        if location not in config.get('locations', {}):
            self.get_logger().warn(f"Unknown location: {location}")
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        # Setup evaluation context (allows evaluating math.pi, etc)
        eval_context = {"math": math}
        eval_context.update(config.get('constants', {}))
        
        waypoints_cfg = config['locations'][location].get('waypoints', [])
        waypoints = []
        
        for wp in waypoints_cfg:
            try:
                x = float(eval(str(wp['x']), eval_context))
                y = float(eval(str(wp['y']), eval_context))
                yaw = float(eval(str(wp['yaw']), eval_context))
                waypoints.append((x, y, yaw))
            except Exception as e:
                self.get_logger().error(f"Error evaluating waypoint for {location}: {e}")
                return

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(waypoints)} waypoints.")
        
    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
        
    def publish_relative_path(self, distance, target_yaw=None):
        if target_yaw is None:
            target_yaw = self.current_yaw
            
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = self.current_x + distance * math.cos(target_yaw)
        pose.pose.position.y = self.current_y + distance * math.sin(target_yaw)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(target_yaw)
        
        path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published relative path: dist={distance}, yaw={target_yaw}")

    def get_kfs_detection(self):
        client = self.create_client(Trigger, '/detect_center_kfs', callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=2.0):
            return "none"
        req = Trigger.Request()
        future = client.call_async(req)
        import time
        while not future.done():
            time.sleep(0.1)
        res = future.result()
        return res.message if res else "none"

    def set_extrusions(self, front_val, back_val):
        traj = JointTrajectory()
        traj.joint_names = [
            'left_front_extrusion_joint', 
            'right_front_extrusion_joint', 
            'left_back_extrusion_joint', 
            'right_back_extrusion_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [front_val, front_val, back_val, back_val]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        self.extrusion_pub.publish(traj)
        self.get_logger().info(f"Extrusions set to {front_val}, {back_val}")

    def set_arm(self, angle):
        traj = JointTrajectory()
        traj.joint_names = ['left_arm_joint', 'right_arm_joint']
        point = JointTrajectoryPoint()
        point.positions = [angle, angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        self.arm_pub.publish(traj)
        
    def set_gripper(self, width):
        traj = JointTrajectory()
        traj.joint_names = ['left_gripper_joint', 'right_gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        self.gripper_pub.publish(traj)

    def climb_trigger_callback(self, request, response):
        self.get_logger().info("Climb sequence triggered!")
        
        self.get_logger().info("1. Pulling front extrusions UP...")
        self.set_extrusions(front_val=0.0, back_val=0.2)
        self.get_clock().sleep_for(Duration(seconds=1.5))
        
        self.get_logger().info("2. Moving forward onto the block...")
        self.publish_relative_path(0.35)
        self.get_clock().sleep_for(Duration(seconds=4.0))
        
        self.get_logger().info("3. Pulling back extrusions UP...")
        self.set_extrusions(front_val=0.0, back_val=0.0)
        self.get_clock().sleep_for(Duration(seconds=1.5))
        
        self.get_logger().info("4. Going to target block center...")
        self.publish_relative_path(0.35)
        self.get_clock().sleep_for(Duration(seconds=3.0))
        
        self.get_logger().info("Climb sequence complete! Acknowledging.")
        response.success = True
        response.message = "Successfully climbed the block"
        return response

    def pick_trigger_callback(self, request, response):
        self.get_logger().info("Pick sequence triggered!")
        
        # We are already approached by 0.5m, so the block is ~0.7m away
        target_x = self.current_x + 0.7 * math.cos(self.current_yaw)
        target_y = self.current_y + 0.7 * math.sin(self.current_yaw)
        
        target_height = 0.0
        map_path = self.yaml_path.replace('locations.yaml', 'game_field_map.yaml')
        if os.path.exists(map_path):
            with open(map_path, 'r') as f:
                map_data = yaml.safe_load(f)
                def find_block_height(data):
                    if isinstance(data, dict):
                        for k, v in data.items():
                            if k.startswith('block_') and isinstance(v, dict):
                                if v.get('x_min', 0) <= target_x <= v.get('x_max', 0) and v.get('y_min', 0) <= target_y <= v.get('y_max', 0):
                                    return v.get('height', 0.0)
                            res = find_block_height(v)
                            if res is not None:
                                return res
                    return None
                    
                found_height = find_block_height(map_data)
                if found_height is not None:
                    target_height = found_height
                            
        current_height = self.current_zone.expected_height if self.current_zone else 0.0
        
        # Add tolerance for float comparison
        if target_height > current_height + 0.05:
            arm_angle = 1.9 # higher block
        elif target_height < current_height - 0.05:
            arm_angle = -0.5 # lower block
        else:
            arm_angle = 0.2 # same height
            
        self.get_logger().info(f"Target height: {target_height}, Current height: {current_height}. Using arm angle: {arm_angle}")
        
        self.get_logger().info("1. Moving arm to target angle...")
        self.set_arm(arm_angle)
        self.get_clock().sleep_for(Duration(seconds=2.5))
        
        self.get_logger().info("2. Closing gripper...")
        self.set_gripper(0.2) # closed
        self.get_clock().sleep_for(Duration(seconds=1.5))
        
        self.get_logger().info("3. Moving arm behind...")
        self.set_arm(-2.0) # behind
        self.get_clock().sleep_for(Duration(seconds=2.5))
        
        self.get_logger().info("4. Releasing gripper...")
        self.set_gripper(0.0) # open
        self.get_clock().sleep_for(Duration(seconds=1.5))
        
        self.stored_kfs_count += 1
        
        self.get_logger().info(f"Pick sequence complete! Total KFS stored: {self.stored_kfs_count}. Acknowledging.")
        response.success = True
        response.message = "Successfully picked KFS"
        return response

    def approach_adjacent_callback(self, request, response):
        self.get_logger().info("Approaching adjacent block...")
        self.publish_relative_path(0.5)
        self.get_clock().sleep_for(Duration(seconds=3.0))
        response.success = True
        return response

    def explore_trigger_callback(self, request, response):
        self.get_logger().info("Exploring adjacent blocks to find KFS...")
        
        # 1. Look front (+Y -> pi/2)
        self.publish_relative_path(0.0, target_yaw=math.pi/2.0)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        res_front = self.get_kfs_detection()
        if res_front == "r2_kfs_real":
            response.success = True
            return response
            
        # 2. Look left (-X -> pi)
        self.publish_relative_path(0.0, target_yaw=math.pi)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        res_left = self.get_kfs_detection()
        if res_left == "r2_kfs_real":
            response.success = True
            return response
            
        # 3. Look right (+X -> 0.0)
        self.publish_relative_path(0.0, target_yaw=0.0)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        res_right = self.get_kfs_detection()
        if res_right == "r2_kfs_real":
            response.success = True
            return response
            
        # 4. If none found, fallback to whichever was "none" (empty)
        if res_front == "none":
            self.publish_relative_path(0.0, target_yaw=math.pi/2.0)
        elif res_left == "none":
            self.publish_relative_path(0.0, target_yaw=math.pi)
        elif res_right == "none":
            self.publish_relative_path(0.0, target_yaw=0.0)
            
        self.get_clock().sleep_for(Duration(seconds=2.0))
        self.get_logger().info("Finished scanning, facing optimal block.")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
