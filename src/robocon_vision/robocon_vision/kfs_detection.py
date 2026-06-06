#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robocon_interfaces.srv import ScanBlock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
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
            "block_3": "none",
            "block_4": "none",
            "block_5": "r2_kfs_real",
            "block_6": "r2_kfs_real",
            "block_7": "none",
            "block_8": "r2_kfs_real",
            "block_9": "none",
            "block_10": "none",
            "block_11": "r2_kfs_fake",
            "block_12": "r2_kfs_real",
        }
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Odom subscriber
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        
        from robocon_interfaces.msg import ZoneState
        from std_msgs.msg import String
        self.zone_sub = self.create_subscription(ZoneState, '/current_zone', self.zone_callback, 10)
        self.mem_pub = self.create_publisher(String, '/planner/update_memory', 10)
        self.mem_sub = self.create_subscription(String, '/planner/update_memory', self.memory_callback, 10)
        
        self.COORDS_TO_BLOCK = {
            (0, 0): 12, (0, 1): 11, (0, 2): 10,
            (1, 0): 9,  (1, 1): 8,  (1, 2): 7,
            (2, 0): 6,  (2, 1): 5,  (2, 2): 4,
            (3, 0): 3,  (3, 1): 2,  (3, 2): 1
        }
        self.BLOCK_TO_COORDS = {v: k for k, v in self.COORDS_TO_BLOCK.items()}
        
        # Load map
        self.yaml_path = os.path.join(get_package_share_directory('robocon_bringup'), 'config', 'game_field_map.yaml')
        ws_src_path = "/home/robot/robocon_ws/src/robocon_bringup/config/game_field_map.yaml"
        if os.path.exists(ws_src_path):
            self.yaml_path = ws_src_path
            
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(ScanBlock, '/scan_block', self.detection_callback, callback_group=self.cb_group)
        
        self.get_logger().info("KFS Detection Node started (Continuous Vision).")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def zone_callback(self, msg):
        if not msg.local_zone.startswith('block_'):
            return
            
        try:
            block_id = int(msg.local_zone.replace('block_', ''))
        except:
            return
            
        if block_id not in self.BLOCK_TO_COORDS:
            return
            
        r, c = self.BLOCK_TO_COORDS[block_id]
        
        # Determine orientation dr, dc
        yaw = (self.current_yaw + 2 * math.pi) % (2 * math.pi)
        if yaw >= 7 * math.pi / 4 or yaw < math.pi / 4:
            dr, dc = 0, 1 # East
        elif math.pi / 4 <= yaw < 3 * math.pi / 4:
            dr, dc = -1, 0 # North
        elif 3 * math.pi / 4 <= yaw < 5 * math.pi / 4:
            dr, dc = 0, -1 # West
        else:
            dr, dc = 1, 0 # South
            
        vision_front = (r + dr, c + dc)
        if dr != 0:
            vision_left = (vision_front[0], vision_front[1] + dr)
            vision_right = (vision_front[0], vision_front[1] - dr)
        else:
            vision_left = (vision_front[0] - dc, vision_front[1])
            vision_right = (vision_front[0] + dc, vision_front[1])
            
        for vr, vc in [(r, c), vision_front, vision_left, vision_right]:
            if (vr, vc) in self.COORDS_TO_BLOCK:
                v_id = self.COORDS_TO_BLOCK[(vr, vc)]
                kfs_type = self.KFS_MAP.get(f"block_{v_id}", "none")
                
                # Publish memory update only if it changed
                if getattr(self, 'published_memory', None) is None:
                    self.published_memory = {}
                    
                if self.published_memory.get(v_id) != kfs_type:
                    self.published_memory[v_id] = kfs_type
                    pub_msg = String()
                    pub_msg.data = f"{v_id}:{kfs_type}"
                    self.mem_pub.publish(pub_msg)

    def detection_callback(self, request, response):
        self.get_logger().info(f"Detection requested for block_{request.target_block_id}. Simulating 1s scan...")
        time.sleep(1.0) # Mock delay
        
        target_id = f"block_{request.target_block_id}"
        if target_id in self.KFS_MAP:
            detected_type = self.KFS_MAP[target_id]
            self.get_logger().info(f"Target block {target_id} detected: {detected_type}")
            response.kfs_detected = detected_type
            response.status = "success"
        else:
            self.get_logger().warn(f"Target block {target_id} not found in map!")
            response.kfs_detected = "none"
            response.status = "failed"
            
        return response

    def memory_callback(self, msg):
        try:
            b_id_str, k_type = msg.data.split(':')
            target_id = f"block_{b_id_str}"
            if self.KFS_MAP.get(target_id) != k_type:
                self.KFS_MAP[target_id] = k_type
                self.get_logger().info(f"Vision Map Updated: {target_id} is now {k_type}")
        except Exception as e:
            pass

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
