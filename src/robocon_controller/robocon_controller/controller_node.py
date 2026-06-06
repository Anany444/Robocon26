#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from robocon_interfaces.srv import PickKFS, PlaceKFS, FaceDirection, MoveToBlock
from nav_msgs.msg import Odometry, Path
from robocon_interfaces.msg import ZoneState
from geometry_msgs.msg import Twist, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.declare_parameter('team', 'red_team')
        self.team = self.get_parameter('team').get_parameter_value().string_value
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.extrusion_pub = self.create_publisher(Float64MultiArray, '/extrusion_controller/commands', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.path_pub = self.create_publisher(Path, '/controller/waypoints', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_arm_angle = 0.0
        self.current_extrusion_positions = [0.0, 0.0, 0.0, 0.0]  # LF, RF, LB, RB
        self.current_gripper_positions = [0.0, 0.0]
        
        self.cb_group = ReentrantCallbackGroup()
        self.pick_srv = self.create_service(PickKFS, '/pick_kfs', self.pick_kfs_callback, callback_group=self.cb_group)
        self.store_srv = self.create_service(Trigger, '/trigger_store_kfs', self.store_kfs_callback, callback_group=self.cb_group)
        self.face_srv = self.create_service(FaceDirection, '/face_direction', self.face_direction_callback, callback_group=self.cb_group)
        self.place_srv = self.create_service(PlaceKFS, '/place_kfs', self.place_kfs_callback, callback_group=self.cb_group)
        
        # Load map
        self.yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'game_field_map.yaml')
        ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/game_field_map.yaml"
        if os.path.exists(ws_src_path):
            self.yaml_path = ws_src_path
            
        self.get_logger().info("Controller Node initialized and ready.")

        # tuning parameters
        self.dist_block_center_picking_point = 0.35
        self.dist_block_center_back_extr_pulling_point = 0.8
        self.dist_high_to_low_front_aligning = 0.5
        self.dist_high_to_low_back_aligning = 1.05


        from std_msgs.msg import String, Bool
        self.zone_sub = self.create_subscription(ZoneState, '/current_zone', self.zone_callback, 10)
        self.goal_sub = self.create_subscription(String, '/planner/goal_location', self.goal_callback, 10)
        self.goal_reached_sub = self.create_subscription(Bool, '/controller/status', self.goal_reached_callback, 10)
        
        self.climb_srv = self.create_service(MoveToBlock, '/move_to_block', self.move_to_block_callback, callback_group=self.cb_group)
        self.approach_srv = self.create_service(Trigger, '/trigger_approach_adjacent', self.approach_adjacent_callback, callback_group=self.cb_group)
        self.generic_trigger_srv = self.create_service(Trigger, '/hardware_trigger', self.generic_trigger_callback, callback_group=self.cb_group)
        self.all_extr_down_srv = self.create_service(Trigger, '/controller/all_extrusions_down', self.all_extrusions_down_callback, callback_group=self.cb_group)
        self.current_zone = None
        self.stored_kfs_count = 0
        self.goal_reached = False

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

    def goal_reached_callback(self, msg):
        if msg.data:
            self.goal_reached = True

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
    def joint_state_callback(self, msg):
        extrusion_names = ['left_front_extrusion_joint', 'right_front_extrusion_joint',
                           'left_back_extrusion_joint', 'right_back_extrusion_joint']
        gripper_names = ['left_gripper_joint', 'right_gripper_joint']
        for i, name in enumerate(extrusion_names):
            if name in msg.name:
                self.current_extrusion_positions[i] = msg.position[msg.name.index(name)]
        for i, name in enumerate(gripper_names):
            if name in msg.name:
                self.current_gripper_positions[i] = msg.position[msg.name.index(name)]
        if 'left_arm_joint' in msg.name:
            self.current_arm_angle = msg.position[msg.name.index('left_arm_joint')]

    def wait_for_arm(self, angle, timeout=7.0):
        start_time = self.get_clock().now()
        timeout_dur = Duration(seconds=timeout)
        while self.get_clock().now() - start_time < timeout_dur:
            if abs(self.current_arm_angle - angle) < 0.1:
                return True
            self.get_clock().sleep_for(Duration(seconds=0.1))
        self.get_logger().warn(f"Arm did not reach target {angle:.2f} in {timeout}s (current: {self.current_arm_angle:.2f})")
        return False

    def wait_for_extrusions(self, front_val, back_val, timeout=5.0, downward_vel=0.0):
        targets = [front_val, front_val, back_val, back_val]
        start_time = self.get_clock().now()
        timeout_dur = Duration(seconds=timeout)
        
        drop_msg = None
        if downward_vel < 0.0:
            drop_msg = Twist()
            drop_msg.linear.z = downward_vel
            
        while self.get_clock().now() - start_time < timeout_dur:
            if drop_msg:
                self.cmd_vel_pub.publish(drop_msg)
            if all(abs(self.current_extrusion_positions[i] - targets[i]) < 0.005 for i in range(4)):
                if drop_msg:
                    self.cmd_vel_pub.publish(Twist()) # Stop down force
                return True
            self.get_clock().sleep_for(Duration(seconds=0.1))
            
        if drop_msg:
            self.cmd_vel_pub.publish(Twist()) # Stop down force on timeout
        self.get_logger().warn(f"Extrusions did not reach target in {timeout}s")
        return False

    def wait_for_goal_reached(self, timeout=10.0):
        start_time = self.get_clock().now()
        timeout_dur = Duration(seconds=timeout)
        while rclpy.ok() and not self.goal_reached:
            if self.get_clock().now() - start_time > timeout_dur:
                self.get_logger().warn(f"Goal wait timeout after {timeout}s")
                break
            self.get_clock().sleep_for(Duration(seconds=0.1))
        return False

    def wait_for_gripper(self, width, timeout=1.0):
        start_time = self.get_clock().now()
        timeout_dur = Duration(seconds=timeout)
        while self.get_clock().now() - start_time < timeout_dur:
            if abs(self.current_gripper_positions[0] - width) < 0.05:
                return True
            self.get_clock().sleep_for(Duration(seconds=0.1))
        self.get_logger().warn(f"Gripper did not reach target {width:.2f} in {timeout}s (current: {self.current_gripper_positions[0]:.2f})")
        return False

    def set_arm(self, angle):
        traj = JointTrajectory()
        # traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['left_arm_joint', 'right_arm_joint']
        point = JointTrajectoryPoint()
        point.positions = [angle, angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1
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
                
                team_data = map_data.get(self.team, map_data)
                
                c_h = find_block_height(team_data, f'block_{request.current_block}')
                if c_h is not None:
                    current_height = c_h
                    
                t_h = find_block_height(team_data, f'block_{request.target_block}')
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
            self.set_extrusions(0.2, 0.2)
            self.wait_for_extrusions(0.2, 0.2)
            
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
                
                team_data = map_data.get(self.team, map_data)
                
                c_center = find_block_center(team_data, f'block_{request.current_block}')
                if c_center: c_x, c_y = c_center
                
                t_center = find_block_center(team_data, f'block_{request.target_block}')
                if t_center: t_x, t_y = t_center
                
        if c_x != 0.0 or c_y != 0.0 or t_x != 0.0 or t_y != 0.0:
            target_yaw = math.atan2(t_y - c_y, t_x - c_x)
            self.get_logger().info(f"target_yaw: {target_yaw}")
            
            # The absolute goal is distance added from the CURRENT block center
            abs_goal_x = c_x + self.dist_block_center_picking_point * math.cos(target_yaw)
            abs_goal_y = c_y + self.dist_block_center_picking_point * math.sin(target_yaw)
            
            self.goal_reached = False
            self.publish_absolute_path(abs_goal_x, abs_goal_y, target_yaw)
            self.wait_for_goal_reached(20.0)
            
        self.get_logger().info("1. Moving arm to target angle...")
        self.set_arm(arm_angle)
        self.wait_for_arm(arm_angle)
        
        self.get_logger().info("2. Closing gripper...")
        self.set_gripper(0.2) # closed
        self.wait_for_gripper(0.2)
        
        self.get_logger().info("3. Moving arm to hold vertically...")
        self.set_arm(0.0) # Vertical hold
        self.wait_for_arm(0.0)
        
        if request.current_block == 0:
            self.get_logger().info("Current block is 0! Auto-storing the picked KFS...")
            self.set_arm(-1.5)
            self.wait_for_arm(-1.5)
            
            self.set_gripper(0.0)
            self.wait_for_gripper(0.0)
            
            self.stored_kfs_count += 1
            self.get_logger().info(f"Auto-store sequence complete! Total KFS stored: {self.stored_kfs_count}.")
            response.message = "Successfully picked and AUTO STORED KFS"
        else:
            self.get_logger().info("Pick sequence complete! Holding vertically.")
            response.message = "Successfully picked and held KFS"
            
        response.success = True
        return response

    def store_kfs_callback(self, request, response):
        self.get_logger().info("Store sequence triggered in Controller Node!")
        
        self.get_logger().info("1. Moving arm behind to store...")
        self.set_arm(-1.0)
        self.wait_for_arm(-1.0)
        
        self.get_logger().info("2. Releasing gripper...")
        self.set_gripper(0.0) # open
        self.wait_for_gripper(0.0, 0.5)
        
        # Optionally reset arm slightly so it doesn't get stuck
        self.get_logger().info("3. Resetting arm to neutral...")
        self.set_arm(0.0)
        self.wait_for_arm(0.0)
        
        self.get_logger().info("Store sequence complete!")
        response.success = True
        response.message = "Successfully stored KFS on bot"
        return response

    def place_kfs_callback(self, request, response):
        self.get_logger().info(f"Place KFS sequence triggered: from block {request.current_block_id} onto block {request.target_block_id}")
        
        # 1. Tilt arm forward to place
        self.get_logger().info("1. Tilting arm forward to place...")
        self.set_arm(0.8)
        self.wait_for_arm(0.8)
        
        # 2. Release gripper
        self.get_logger().info("2. Releasing gripper...")
        self.set_gripper(0.0)
        self.wait_for_gripper(0.0)
        
        # 3. Reset arm to neutral
        self.get_logger().info("3. Resetting arm to neutral...")
        self.set_arm(0.0)
        self.wait_for_arm(0.0)
        
        self.get_logger().info("Place KFS sequence complete!")
        response.success = True
        response.message = "Successfully placed KFS on block"
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
                
                team_data = map_data.get(self.team, map_data)
                c_center = find_block_center(team_data, f'block_{request.current_block_id}')
                if c_center: c_x, c_y = c_center
        target_yaw = self.current_yaw
        if request.direction == "left":
            target_yaw += math.pi / 2.0
        elif request.direction == "right":
            target_yaw -= math.pi / 2.0
        elif request.direction == "back":
            target_yaw += math.pi
            
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Step 1: Move to block center first (at current yaw) to avoid collisions
        self.goal_reached = False
        self.get_logger().info(f"1. Centering on block at ({c_x:.2f}, {c_y:.2f})...")
        self.publish_absolute_path(c_x, c_y, self.current_yaw)
        self.wait_for_goal_reached(5.0)
        
        # Step 2: Rotate to target yaw at block center
        self.goal_reached = False
        self.get_logger().info(f"2. Rotating to yaw={target_yaw:.2f}...")
        self.publish_absolute_path(c_x, c_y, target_yaw)
        self.wait_for_goal_reached(10.0)
            
        response.success = True
        response.message = f"Faced {request.direction}"
        return response

    def zone_callback(self, msg):
        self.current_zone = msg

    def goal_callback(self, msg):
        self.publish_location_path(msg.data)
        
    def publish_location_path(self, location):
        self.get_logger().info(f"Generating path for location: {location}...")
        
        # Reload YAML dynamically so live edits apply immediately!
        try:
            loc_yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'locations.yaml')
            ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/locations.yaml"
            if os.path.exists(ws_src_path):
                loc_yaml_path = ws_src_path
                
            with open(loc_yaml_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load locations.yaml: {e}")
            return
            
        team_config = config.get(self.team, config)
        if location not in team_config.get('locations', {}):
            self.get_logger().warn(f"Unknown location: {location}")
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        # Setup evaluation context (allows evaluating math.pi, etc)
        eval_context = {"math": math}
        eval_context.update(config.get('constants', {}))
        
        waypoints_cfg = team_config['locations'][location].get('waypoints', [])
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
        

        
    def set_extrusions(self, front_val, back_val):
        msg = Float64MultiArray()
        msg.data = [float(front_val), float(front_val), float(back_val), float(back_val)]
        self.extrusion_pub.publish(msg)
        self.get_logger().info(f"Extrusions set to {front_val}, {back_val}")

    def find_faced_block_center(self):
        """Find the center of the block we're currently facing by looking ~1m ahead."""
        if not os.path.exists(self.yaml_path):
            return None
        with open(self.yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
        team_data = map_data.get(self.team, map_data)
        
        look_x = self.current_x + 1.0 * math.cos(self.current_yaw)
        look_y = self.current_y + 1.0 * math.sin(self.current_yaw)
        
        best_center = None
        min_dist = 999.0
        
        def search(data):
            nonlocal best_center, min_dist
            if isinstance(data, dict):
                for k, v in data.items():
                    if k.startswith('block_') and isinstance(v, dict) and 'x_min' in v:
                        cx = (v['x_min'] + v['x_max']) / 2.0
                        cy = (v['y_min'] + v['y_max']) / 2.0
                        dist = math.hypot(look_x - cx, look_y - cy)
                        if dist < min_dist:
                            min_dist = dist
                            best_center = (cx, cy)
                    search(v)
        
        search(team_data)
        return best_center

    def move_to_block_callback(self, request, response):
        self.get_logger().info(f"MoveToBlock triggered: current={request.current_block_id}, target={request.target_block_id}")
        
        # Look up current and target block centers from the map
        c_x, c_y = 0.0, 0.0
        t_x, t_y = 0.0, 0.0
        current_height = 0.0
        target_height = 0.0
        
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                
                def find_block_center_and_height(data, block_name):
                    if isinstance(data, dict):
                        if block_name in data:
                            cx = (data[block_name].get('x_min', 0) + data[block_name].get('x_max', 0)) / 2.0
                            cy = (data[block_name].get('y_min', 0) + data[block_name].get('y_max', 0)) / 2.0
                            h = data[block_name].get('height', 0.0)
                            return cx, cy, h
                        for k, v in data.items():
                            res = find_block_center_and_height(v, block_name)
                            if res: return res
                    return None
                
                team_data = map_data.get(self.team, map_data)
                
                c_data = find_block_center_and_height(team_data, f'block_{request.current_block_id}')
                if c_data: c_x, c_y, current_height = c_data
                
                t_data = find_block_center_and_height(team_data, f'block_{request.target_block_id}')
                if t_data: t_x, t_y, target_height = t_data
        
        if t_x == 0.0 and t_y == 0.0:
            self.get_logger().error("Cannot find target block center from map!")
            response.success = False
            response.message = "Failed to find target block"
            return response
        
        self.get_logger().info(f"Target block center: ({t_x:.2f}, {t_y:.2f}), target_height: {target_height}, current_height: {current_height}")
        
        # Compute yaw from current block center to target block center
        climb_yaw = math.atan2(t_y - c_y, t_x - c_x)
        
        if target_height > current_height + 0.05:
            # --- LOW TO HIGH CLIMBING SEQUENCE ---
            self.get_logger().info("0. Initial alignment to edge of current block...")
            align_x = c_x + self.dist_block_center_picking_point * math.cos(climb_yaw)
            align_y = c_y + self.dist_block_center_picking_point * math.sin(climb_yaw)
            
            self.goal_reached = False
            self.publish_absolute_path(align_x, align_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
            # Compute the back-extrusion pulling point: move forward 'param' distance from CURRENT center
            pull_x = c_x + self.dist_block_center_back_extr_pulling_point * math.cos(climb_yaw)
            pull_y = c_y + self.dist_block_center_back_extr_pulling_point * math.sin(climb_yaw)
            
            self.get_logger().info("1. Pulling front extrusions UP...")
            self.set_extrusions(front_val=0.0, back_val=0.2)
            self.wait_for_extrusions(0.0, 0.2)
            
            self.get_logger().info(f"2. Moving forward to back-extr pull point ({pull_x:.2f}, {pull_y:.2f})...")
            self.goal_reached = False
            self.publish_absolute_path(pull_x, pull_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
            self.get_logger().info("3. Pulling back extrusions UP...")
            self.set_extrusions(front_val=0.0, back_val=0.0)
            self.wait_for_extrusions(0.0, 0.0)
            
            self.get_logger().info(f"4. Going to target block center ({t_x:.2f}, {t_y:.2f})...")
            self.goal_reached = False
            self.publish_absolute_path(t_x, t_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
        elif target_height < current_height - 0.05:
            # --- HIGH TO LOW CLIMBING SEQUENCE ---
            self.get_logger().info("1. All extrusions UP...")
            self.set_extrusions(front_val=0.0, back_val=0.0)
            self.wait_for_extrusions(0.0, 0.0)
            
            self.get_logger().info("2. Align till front align dist...")
            front_align_x = c_x + self.dist_high_to_low_front_aligning * math.cos(climb_yaw)
            front_align_y = c_y + self.dist_high_to_low_front_aligning * math.sin(climb_yaw)
            self.goal_reached = False
            self.publish_absolute_path(front_align_x, front_align_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
            self.get_logger().info("3. Front extrusions DOWN...")
            self.set_extrusions(front_val=0.2, back_val=0.0)
            self.wait_for_extrusions(0.2, 0.0)
            
            self.get_logger().info("4. Align till back align dist...")
            back_align_x = c_x + self.dist_high_to_low_back_aligning * math.cos(climb_yaw)
            back_align_y = c_y + self.dist_high_to_low_back_aligning * math.sin(climb_yaw)
            self.goal_reached = False
            self.publish_absolute_path(back_align_x, back_align_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
            self.get_logger().info("5. Back extrusions DOWN...")
            self.set_extrusions(front_val=0.2, back_val=0.2)
            self.wait_for_extrusions(0.2, 0.2)
            
            self.get_logger().info(f"6. Going to target block center ({t_x:.2f}, {t_y:.2f})...")
            self.goal_reached = False
            self.publish_absolute_path(t_x, t_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
            self.get_logger().info("7. All extrusions UP (resting on new block)...")
            self.set_extrusions(front_val=0.0, back_val=0.0)
            self.wait_for_extrusions(0.0, 0.0, downward_vel=-2.0)
            
        else:
            # SAME HEIGHT (FLAT)
            self.get_logger().info(f"Going to target block center ({t_x:.2f}, {t_y:.2f})... (flat)")
            self.goal_reached = False
            self.publish_absolute_path(t_x, t_y, climb_yaw)
            self.wait_for_goal_reached(20.0)
            
        self.get_logger().info("MoveToBlock complete!")
        response.success = True
        response.message = "Successfully moved to block"
        return response


    def all_extrusions_down_callback(self, request, response):
        self.get_logger().info("Received command to pull all extrusions DOWN (0.2)...")
        self.set_extrusions(front_val=0.2, back_val=0.2)
        self.wait_for_extrusions(0.2, 0.2)
        response.success = True
        response.message = "All extrusions are down"
        return response

    def approach_adjacent_callback(self, request, response):
        self.get_logger().info("Approaching adjacent block...")
        target = self.find_faced_block_center()
        if target:
            t_x, t_y = target
            self.get_logger().info(f"Moving to faced block center: ({t_x:.2f}, {t_y:.2f})")
            self.publish_absolute_path(t_x, t_y, self.current_yaw)
        else:
            self.get_logger().warn("Could not find faced block, falling back to relative path")
            self.publish_relative_path(0.5)
        self.get_clock().sleep_for(Duration(seconds=3.0))
        response.success = True
        return response

    def generic_trigger_callback(self, request, response):
        self.get_logger().info("Received generic hardware trigger from BT. Mocking 1-second execution...")
        self.get_clock().sleep_for(Duration(seconds=1.0))
        response.success = True
        response.message = "Hardware step complete"
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
