#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class FakeImu(Node):
    def __init__(self):
        super().__init__('fake_imu_node')
        self.publisher_ = self.create_publisher(Imu, '/unilidar/imu', 50)
        # Publish at 500 Hz to mimic the real IMU
        self.timer = self.create_timer(1.0 / 500.0, self.timer_callback)
        self.get_logger().info("Publishing Fake IMU data at 500Hz to trick LIO-SAM...")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'unilidar_imu'
        
        # Perfect upright orientation
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Zero rotation
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # Exact gravity vector we calculated yesterday
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 10.057865
        
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = FakeImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
