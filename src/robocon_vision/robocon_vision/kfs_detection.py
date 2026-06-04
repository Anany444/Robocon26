#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
import time
import math
import yaml
import os

class KFSDetectionNode(Node):
    def __init__(self):
        super().__init__('kfs_detection')
        
        # Hardcoded match from user screenshot
        self.KFS_MAP = {
            "block_1": "none",
            "block_2": "r2_kfs_real",
            "block_3": "r1_kfs",
            "block_4": "none",
            "block_5": "r2_kfs_real",
            "block_6": "r2_kfs_real",
            "block_7": "r1_kfs",
            "block_8": "r2_kfs_real",
            "block_9": "none",
            "block_10": "r1_kfs",
            "block_11": "r2_kfs_fake",
            "block_12": "r2_kfs_real",
        }
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Odom subscriber
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        
        # Load map
        self.yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'game_field_map.yaml')
        ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/game_field_map.yaml"
        if os.path.exists(ws_src_path):
            self.yaml_path = ws_src_path
            
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Trigger, '/detect_center_kfs', self.detection_callback, callback_group=self.cb_group)
        
        self.get_logger().info("KFS Detection Node started (Dynamic Mock).")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def detection_callback(self, request, response):
        self.get_logger().info("Detection requested. Simulating 1s scan...")
        time.sleep(1.0) # Mock delay
        
        target_x = self.current_x + 0.7 * math.cos(self.current_yaw)
        target_y = self.current_y + 0.7 * math.sin(self.current_yaw)
        
        best_block = ""
        min_dist = 999.0
        
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                
                def find_closest_block(data):
                    nonlocal min_dist, best_block
                    if isinstance(data, dict):
                        for k, v in data.items():
                            if k.startswith('block_') and isinstance(v, dict):
                                cx = (v.get('x_min', 0) + v.get('x_max', 0)) / 2.0
                                cy = (v.get('y_min', 0) + v.get('y_max', 0)) / 2.0
                                dist = math.hypot(target_x - cx, target_y - cy)
                                if dist < min_dist and dist < 2.0:
                                    min_dist = dist
                                    best_block = k
                            find_closest_block(v)
                            
                find_closest_block(map_data)
                
        if best_block in self.KFS_MAP:
            detected_type = self.KFS_MAP[best_block]
            self.get_logger().info(f"Target point (x={target_x:.2f}, y={target_y:.2f}) closest to {best_block} ({min_dist:.2f}m). Detected: {detected_type}")
        else:
            detected_type = "none"
            self.get_logger().info(f"Target point (x={target_x:.2f}, y={target_y:.2f}) is too far from any block! Detected: none")
            
        response.success = True
        response.message = detected_type
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KFSDetectionNode()
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
