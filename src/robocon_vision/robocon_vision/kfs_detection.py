#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class KFSDetectionNode(Node):
    def __init__(self):
        super().__init__('kfs_detection')
        
        # Default status
        self.current_status = "none"
        
        # Service to return the detection result
        self.srv = self.create_service(Trigger, '/detect_center_kfs', self.detection_callback)
        
        # Topic to allow the GUI to mock the detection result
        self.mock_sub = self.create_subscription(String, '/vision/mock_kfs_status', self.mock_callback, 10)
        
        self.get_logger().info("KFS Detection Node started. Mock status is 'none'.")
        
    def mock_callback(self, msg):
        self.current_status = msg.data
        self.get_logger().info(f"Mock KFS status updated to: {self.current_status}")

    def detection_callback(self, request, response):
        self.get_logger().info(f"Detection requested. Returning: {self.current_status}")
        response.success = True
        response.message = self.current_status
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KFSDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
