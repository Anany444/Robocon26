#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from robocon_interfaces.srv import PickKFS, FaceDirection
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.extrusion_pub = self.create_publisher(JointTrajectory, '/extrusion_controller/joint_trajectory', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.path_pub = self.create_publisher(Path, '/controller/waypoints', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.cb_group = ReentrantCallbackGroup()
        self.pick_srv = self.create_service(PickKFS, '/pick_kfs', self.pick_kfs_callback, callback_group=self.cb_group)
        self.store_srv = self.create_service(Trigger, '/trigger_store_kfs', self.store_kfs_callback, callback_group=self.cb_group)
        self.face_srv = self.create_service(FaceDirection, '/face_direction', self.face_direction_callback, callback_group=self.cb_group)
        
        # Load map
        self.yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'game_field_map.yaml')
        ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/game_field_map.yaml"
        if os.path.exists(ws_src_path):
            self.yaml_path = ws_src_path
            
        self.get_logger().info("Controller Node initialized and ready.")

        # tuning parameters
        self.dist_block_center_picking_point = 0.35

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
        
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = self.current_x + distance * math.cos(target_yaw)
        pose.pose.position.y = self.current_y + distance * math.sin(target_yaw)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(target_yaw)
        
        path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published relative path: dist={distance}, yaw={target_yaw}")

    def publish_absolute_path(self, x, y, yaw):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published absolute path to: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # self.get_logger().info(f"Current Position: x={self.current_x:.2f}, y={self.current_y:.2f}")

    def set_arm(self, angle):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['left_arm_joint', 'right_arm_joint']
        point = JointTrajectoryPoint()
        point.positions = [angle, angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        self.arm_pub.publish(traj)
        
    def set_gripper(self, width):
        msg = Float64MultiArray()
        msg.data = [float(width), float(width)]
        self.gripper_pub.publish(msg)

    def pick_kfs_callback(self, request, response):
        self.get_logger().info(f"Pick sequence triggered for current: {request.current_block}, target: {request.target_block}")
        
        current_height = 0.0
        target_height = 0.0
        
        # 1. Check the height of current and target directly from map
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                
                def find_block_height(data, block_name):
                    if isinstance(data, dict):
                        if block_name in data:
                            return data[block_name].get('height', 0.0)
                        for k, v in data.items():
                            h = find_block_height(v, block_name)
                            if h is not None:
                                return h
                    return None
                
                red_team_data = map_data.get('red_team', map_data)
                
                c_h = find_block_height(red_team_data, f'block_{request.current_block}')
                if c_h is not None:
                    current_height = c_h
                    
                t_h = find_block_height(red_team_data, f'block_{request.target_block}')
                if t_h is not None:
                    target_height = t_h
                    
        # Calculate arm angle
        if target_height > current_height + 0.05:
            arm_angle = 1.9 # higher block
        elif target_height < current_height - 0.05:
            arm_angle = -0.5 # lower block
        else:
            arm_angle = 0.2 # same height
            
        self.get_logger().info(f"Target height: {target_height}, Current height: {current_height}. Using arm angle: {arm_angle}")
        
        # 2. Extrusion and picking sequence
        if target_height > current_height + 0.05:
            self.get_logger().info("Target is higher: Putting all extrusions DOWN first!")
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ['left_front_extrusion_joint', 'right_front_extrusion_joint', 'left_back_extrusion_joint', 'right_back_extrusion_joint']
            point = JointTrajectoryPoint()
            point.positions = [0.2, 0.2, 0.2, 0.2]
            point.time_from_start.sec = 1
            traj.points.append(point)
            self.extrusion_pub.publish(traj)
            self.get_clock().sleep_for(Duration(seconds=3.0))
            
        # 2.5 Align with Target
        self.get_logger().info("Aligning X, Y, and Yaw towards target block...")
        # Get target and current centers to find absolute angle
        c_x, c_y = 0.0, 0.0
        t_x, t_y = 0.0, 0.0
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                def find_block_center(data, block_name):
                    if isinstance(data, dict):
                        if block_name in data:
                            cx = (data[block_name].get('x_min', 0) + data[block_name].get('x_max', 0)) / 2.0
                            cy = (data[block_name].get('y_min', 0) + data[block_name].get('y_max', 0)) / 2.0
                            return cx, cy
                        for k, v in data.items():
                            res = find_block_center(v, block_name)
                            if res: return res
                    return None
                
                red_team_data = map_data.get('red_team', map_data)
                
                c_center = find_block_center(red_team_data, f'block_{request.current_block}')
                if c_center: c_x, c_y = c_center
                
                t_center = find_block_center(red_team_data, f'block_{request.target_block}')
                if t_center: t_x, t_y = t_center
                
        if c_x != 0.0 or c_y != 0.0 or t_x != 0.0 or t_y != 0.0:
            target_yaw = math.atan2(t_y - c_y, t_x - c_x)
            self.get_logger().info(f"target_yaw: {target_yaw}")
            
            # The absolute goal is distance added from the CURRENT block center
            abs_goal_x = c_x + self.dist_block_center_picking_point * math.cos(target_yaw)
            abs_goal_y = c_y + self.dist_block_center_picking_point * math.sin(target_yaw)
            
            self.publish_absolute_path(abs_goal_x, abs_goal_y, target_yaw)
            self.get_clock().sleep_for(Duration(seconds=4.0))
            
        self.get_logger().info("1. Moving arm to target angle...")
        self.set_arm(arm_angle)
        self.get_clock().sleep_for(Duration(seconds=3.5))
        
        self.get_logger().info("2. Closing gripper...")
        self.set_gripper(0.2) # closed
        self.get_clock().sleep_for(Duration(seconds=2.5))
        
        self.get_logger().info("3. Moving arm to hold vertically...")
        self.set_arm(0.0) # Vertical hold
        self.get_clock().sleep_for(Duration(seconds=2.5))
        
        self.get_logger().info("Pick sequence complete! Holding vertically.")
        response.success = True
        response.message = "Successfully picked and held KFS"
        return response

    def store_kfs_callback(self, request, response):
        self.get_logger().info("Store sequence triggered in Controller Node!")
        
        self.get_logger().info("1. Moving arm behind to store...")
        self.set_arm(-1.5)
        self.get_clock().sleep_for(Duration(seconds=2.5))
        
        self.get_logger().info("2. Releasing gripper...")
        self.set_gripper(0.0) # open
        self.get_clock().sleep_for(Duration(seconds=1.5))
        
        # Optionally reset arm slightly so it doesn't get stuck
        self.get_logger().info("3. Resetting arm to neutral...")
        self.set_arm(0.0)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        
        self.get_logger().info("Store sequence complete!")
        response.success = True
        response.message = "Successfully stored KFS on bot"
        return response

    def face_direction_callback(self, request, response):
        self.get_logger().info(f"Face direction triggered: {request.direction} from block {request.current_block_id}")
        
        c_x, c_y = 0.0, 0.0
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                def find_block_center(data, block_name):
                    if isinstance(data, dict):
                        if block_name in data:
                            cx = (data[block_name].get('x_min', 0) + data[block_name].get('x_max', 0)) / 2.0
                            cy = (data[block_name].get('y_min', 0) + data[block_name].get('y_max', 0)) / 2.0
                            return cx, cy
                        for k, v in data.items():
                            res = find_block_center(v, block_name)
                            if res: return res
                    return None
                
                red_team_data = map_data.get('red_team', map_data)
                c_center = find_block_center(red_team_data, f'block_{request.current_block_id}')
                if c_center: c_x, c_y = c_center
                
        target_yaw = self.current_yaw
        if request.direction == "left":
            target_yaw += math.pi / 2.0
        elif request.direction == "right":
            target_yaw -= math.pi / 2.0
        elif request.direction == "back":
            target_yaw += math.pi
            
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
        
        if c_x != 0.0 or c_y != 0.0:
            self.publish_absolute_path(c_x, c_y, target_yaw)
            self.get_clock().sleep_for(Duration(seconds=3.0))
            
        response.success = True
        response.message = f"Faced {request.direction}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
