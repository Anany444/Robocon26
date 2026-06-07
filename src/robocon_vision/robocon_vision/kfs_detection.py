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
import torch
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image

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
        
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(ScanBlock, '/scan_block', self.detection_callback, callback_group=self.cb_group)
        
        # Live Inferencing Setup
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device for YOLO: {self.device}")
        
        model_path = os.path.join(get_package_share_directory('robocon_vision'), 'models', 'rbcn5.pt')
        if os.path.exists(model_path):
            self.model = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model from {model_path}")
        else:
            self.model = None
            self.get_logger().warn(f"Model not found at {model_path}!")
            
        self.latest_image = None
        self.new_image = False
        self.latest_kfs = "none"
        self.image_sub = self.create_subscription(Image, '/camera/rgbd/image', self.image_callback, 10)
        self.annotated_pub = self.create_publisher(Image, '/vision/annotated_image', 10)
        
        # Inference loop timer
        self.inference_timer = self.create_timer(0.1, self.inference_loop)
        
        self.get_logger().info("KFS Detection Node started (Live Vision Active).")

    def image_callback(self, msg):
        self.latest_image = msg
        self.new_image = True
        
    def inference_loop(self):
        if self.latest_image is None or self.model is None or not getattr(self, 'new_image', False):
            return
            
        self.new_image = False
        try:
            img_msg = self.latest_image
            img_arr = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))
            
            if img_msg.encoding == "rgb8":
                img_arr = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)
            elif img_msg.encoding == "rgba8":
                img_arr = cv2.cvtColor(img_arr, cv2.COLOR_RGBA2BGR)
                
            results = self.model(img_arr, device=self.device, half=True, verbose=False)
            
            detected_class = "none"
            if len(results) > 0 and len(results[0].boxes) > 0:
                best_box = results[0].boxes[0]
                for box in results[0].boxes:
                    if box.conf[0] > best_box.conf[0]:
                        best_box = box
                
                class_id = int(best_box.cls[0].item())
                detected_class = self.model.names[class_id]
                
                # Map standard YOLO classes to KFS classes if names differ
                if detected_class.lower() == "r2_kfs_real" or "real" in detected_class.lower():
                    detected_class = "r2_kfs_real"
                elif detected_class.lower() == "r2_kfs_fake" or "fake" in detected_class.lower():
                    detected_class = "r2_kfs_fake"
                elif detected_class.lower() == "r1_kfs" or "r1" in detected_class.lower():
                    detected_class = "r1_kfs"
            
            self.latest_kfs = detected_class
            
            # Publish annotated image
            res_plotted = results[0].plot()
            if img_msg.encoding == "rgb8":
                res_plotted = cv2.cvtColor(res_plotted, cv2.COLOR_BGR2RGB)
                
            annotated_msg = Image()
            annotated_msg.header = img_msg.header
            annotated_msg.height = res_plotted.shape[0]
            annotated_msg.width = res_plotted.shape[1]
            annotated_msg.encoding = img_msg.encoding
            annotated_msg.is_bigendian = img_msg.is_bigendian
            annotated_msg.step = res_plotted.shape[1] * res_plotted.shape[2]
            annotated_msg.data = res_plotted.tobytes()
            
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")

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
        self.get_logger().info(f"Detection requested for block_{request.target_block_id}. Using live YOLO...")
        time.sleep(0.5) # Allow half a second to capture any new frames
        
        target_id = f"block_{request.target_block_id}"
        detected_type = self.latest_kfs
        
        # Fallback to map if no valid detection
        if detected_type not in ["r1_kfs", "r2_kfs_real", "r2_kfs_fake"]:
            self.get_logger().warn(f"Live detection '{detected_type}' invalid or none. Falling back to mock map.")
            if target_id in self.KFS_MAP:
                detected_type = self.KFS_MAP[target_id]
            else:
                detected_type = "none"
                
        self.get_logger().info(f"Target block {target_id} finalized detection: {detected_type}")
        response.kfs_detected = detected_type
        response.status = "success"
            
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
