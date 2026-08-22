#!/usr/bin/env python3
"""
VL53L0X 1D LiDAR Serial Reader Node
===================================
- Reads distance data streamed from Arduino over serial
- Serial format: DIST:<distance_in_meters>
- Publishes: std_msgs/Float32 on '/dist/weapon_rack'
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import serial
import threading
import time


class SerialRead(Node):
    def __init__(self):
        super().__init__('serial_read')

        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)

        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.dist_pub = self.create_publisher(Float32, '/dist/weapon_rack', 10)

        self.ser = None
        try:
            self.ser = serial.Serial(port_name, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f"Opened LiDAR serial port: {port_name} @ {baudrate}")
        except serial.SerialException:
            fallbacks = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB1']
            for fb in fallbacks:
                if fb == port_name:
                    continue
                try:
                    self.ser = serial.Serial(fb, baudrate=baudrate, timeout=0.1)
                    self.get_logger().info(f"Opened fallback LiDAR serial port: {fb} @ {baudrate}")
                    break
                except serial.SerialException:
                    pass
            if not self.ser:
                self.get_logger().error(f"Failed to open LiDAR serial port ({port_name} or fallbacks)")

        self.running = True
        self.serial_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.serial_thread.start()

        self.get_logger().info("Serial Read node initialized. Publishing float distances to '/dist/weapon_rack'.")

    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('DIST:'):
                    val_str = line[5:].strip()
                    dist_val = float(val_str)
                    msg = Float32()
                    msg.data = dist_val
                    self.dist_pub.publish(msg)
            except Exception:
                time.sleep(0.01)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialRead()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
