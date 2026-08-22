import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import std_msgs.msg
from geometry_msgs.msg import TwistStamped

from std_srvs.srv import SetBool
import serial
import time

class SerialPrint(Node):
    def __init__(self):
        super().__init__('serial_print')
        self.get_logger().info('SerialPrint node has been started.')
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)
            self.get_logger().info('Serial port /dev/ttyUSB0 opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None
        self.subscription = self.create_subscription(
            std_msgs.msg.String,
            'serial_topic',
            self.serial_callback,
            10)
        
        self.arm_gripper_srv = self.create_service(SetBool, 'toggle_gripper', self.arm_gripper_callback)
        self.override_srv = self.create_service(SetBool, 'toggle_override', self.override_callback)
        self.weapon_gripper_srv = self.create_service(SetBool, 'toggle_weapon_gripper', self.weapon_gripper_callback)
        self.weapon_servo_srv = self.create_service(SetBool, 'toggle_weapon_servo', self.weapon_servo_callback)
        
        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.declare_parameter('lf_pwm_factor', 1.0)
        self.declare_parameter('rf_pwm_factor', 1.0)
        self.declare_parameter('rr_pwm_factor', 1.0)
        self.declare_parameter('lr_pwm_factor', 1.0)

        self.lf_factor = float(self.get_parameter('lf_pwm_factor').value)
        self.rf_factor = float(self.get_parameter('rf_pwm_factor').value)
        self.rr_factor = float(self.get_parameter('rr_pwm_factor').value)
        self.lr_factor = float(self.get_parameter('lr_pwm_factor').value)

        if self.ser:
            time.sleep(2.0)
            command = f"PWM_FACTORS:{self.lf_factor:.2f},{self.rf_factor:.2f},{self.rr_factor:.2f},{self.lr_factor:.2f}\n"
            try:
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info(f'[SERIAL_TX] {command.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def cmd_vel_callback(self, msg):
        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y
        angular_z = msg.twist.angular.z
        
        command = f"CMD_VEL:{linear_x:.2f},{linear_y:.2f},{angular_z:.2f}\n"
        if self.ser:
            try:
                self.ser.write(command.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
        # Changed to INFO so you can see it in terminal during testing!
        if abs(linear_x) < 0.001 and abs(linear_y) < 0.001 and abs(angular_z)  < 0.01:
            return
        self.get_logger().info(f'[SERIAL_TX] {command.strip()}')

    def serial_callback(self, msg):
        self.print_message(msg.data)

    def print_message(self, message):
        self.get_logger().info(f'[SERIAL_TX] {message}')
        if self.ser:
            try:
                self.ser.write((message + '\n').encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def arm_gripper_callback(self, request, response):
        if request.data:
            command = "GRIPPER_ON\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().info('[SERIAL_TX] GRIPPER_ON')
            response.message = "Gripper activated"
        else:
            command = "GRIPPER_OFF\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().info('[SERIAL_TX] GRIPPER_OFF')
            response.message = "Gripper deactivated"
            
        response.success = True
        return response
    
    def override_callback(self, request, response):
        if request.data:
            command = "OVERRIDE_ON\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().error('[SERIAL_TX] OVERRIDE_ON')
            response.message = "Override activated"
        else:
            command = "OVERRIDE_OFF\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().error('[SERIAL_TX] OVERRIDE_OFF')
            response.message = "Override deactivated"
            
        response.success = True
        return response
    
    def weapon_gripper_callback(self, request, response):
        if request.data:
            command = "WEAPON_GRIPPER_CLOSED\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().warn('[SERIAL_TX] WEAPON_GRIPPER_CLOSED')
            response.message = "Weapon gripper closed"
        else:
            command = "WEAPON_GRIPPER_OPEN\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().warn('[SERIAL_TX] WEAPON_GRIPPER_OPEN')
            response.message = "Weapon gripper open"
            
        response.success = True
        return response

    def weapon_servo_callback(self, request, response):
        if request.data:
            command = "WEAPON_SERVO_ANGLE:90.0\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().info('[SERIAL_TX] WEAPON_SERVO_ANGLE:90.0')
            response.message = "Weapon servo set to 90"
        else:
            command = "WEAPON_SERVO_ANGLE:0.0\n"
            if self.ser:
                try:
                    self.ser.write(command.encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
            self.get_logger().info('[SERIAL_TX] WEAPON_SERVO_ANGLE:0.0')
            response.message = "Weapon servo set to 0"
            
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    serial_print_node = SerialPrint()

    try:
        rclpy.spin(serial_print_node)
    except ExternalShutdownException:
        pass
    finally:
        serial_print_node.destroy_node()
        rclpy.shutdown()
