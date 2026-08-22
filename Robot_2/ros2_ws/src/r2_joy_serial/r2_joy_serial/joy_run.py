import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import std_msgs.msg
import time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool

class JoyRun(Node):
    def __init__(self):
        super().__init__('joy_run')
        self.get_logger().info('R2 JoyRun node has been started.')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            std_msgs.msg.String,
            'serial_topic',
            10
        )

        self.twistpub_ = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10
        )

        self.gripper_client = self.create_client(SetBool, 'toggle_gripper')
        self.override_client = self.create_client(SetBool, 'toggle_override')
        self.front_client = self.create_client(SetBool, 'toggle_front')
        self.back_client = self.create_client(SetBool, 'toggle_back')

        self.max_linear_speed = 1.0
        self.max_angular_speed = 0.5

        self.initialized = False

        self.gripper_on = False
        self.override_on = False
        self.front_on = False
        self.back_on = False
        self.prev_arm_step = 0

    def joy_callback(self, joy_msg):

        if not self.initialized:
            self.buttons = list(joy_msg.buttons)
            self.axes = list(joy_msg.axes)
            self.prev_buttons = list(joy_msg.buttons)
            self.initialized = True
            return 

        self.buttons = list(joy_msg.buttons)
        self.axes = list(joy_msg.axes)
        
        # --- SERVICES (Toggles) ---
        # Override (L2: 6) - Processed FIRST
        if self.buttons[6] == 1 and self.prev_buttons[6] == 0:
            self.toggle_override()

        # If override is active, block all other commands immediately
        if self.override_on:
            self.prev_buttons = list(self.buttons)
            self.axes = [0.0] * len(self.axes)
            self.buttons = [0] * len(self.buttons)
            self.cmd_vel_publish()
            return

        # Arm Gripper Relay (Options: 9)
        if self.buttons[9] == 1 and self.prev_buttons[9] == 0:
            self.toggle_arm_gripper()

        # Front Pneumatic Actuator Relay (Y: 3)
        if self.buttons[3] == 1 and self.prev_buttons[3] == 0:
            self.toggle_front()

        # Back Pneumatic Actuator Relay (A: 1)
        if self.buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.toggle_back()
        
        # --- TOPICS (Continuous / Discrete Strings) ---
        msg = std_msgs.msg.String()

        # Proportional Servo Arm Step (right_stick.ud: 4)
        if len(self.axes) > 4:
            if abs(self.axes[4]) < 0.08:
                speed = 0
            elif speed > 0:
                speed = 1
            else:
                speed = -1
            if speed != self.prev_arm_step:
                self.prev_arm_step = speed
                msg.data = f"SERVO_ARM_STEP:{speed}"
                self.publisher_.publish(msg)

        self.cmd_vel_publish()
        
        # Save previous buttons for toggle debounce
        self.prev_buttons = list(self.buttons)
        return

    def toggle_arm_gripper(self):
        self.gripper_on = not self.gripper_on
        self.get_logger().info(f'Arm Gripper State: {self.gripper_on}')
        self.send_service_request(self.gripper_client, self.gripper_on)

    def toggle_override(self):
        self.override_on = not self.override_on
        self.get_logger().error(f'Override State: {self.override_on}')
        self.send_service_request(self.override_client, self.override_on)

    def toggle_front(self):
        self.front_on = not self.front_on
        self.get_logger().info(f'Front Pneumatic State: {self.front_on}')
        self.send_service_request(self.front_client, self.front_on)

    def toggle_back(self):
        self.back_on = not self.back_on
        self.get_logger().info(f'Back Pneumatic State: {self.back_on}')
        self.send_service_request(self.back_client, self.back_on)

    def send_service_request(self, client, state):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {client.srv_name} not available!')
            return
        
        request = SetBool.Request()
        request.data = state

        future = client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Node Response: {response.message} (Success: {response.success})')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        return

    def cmd_vel_publish(self):
        twist_msg = TwistStamped()

        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        twist_msg.twist.linear.x = self.axes[1] * self.max_linear_speed if len(self.axes) > 1 else 0.0
        twist_msg.twist.linear.y = self.axes[0] * self.max_linear_speed if len(self.axes) > 0 else 0.0
        twist_msg.twist.linear.z = 0.0

        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        if self.buttons[4] == 1 and self.buttons[5] == 1:
            pass
        elif self.buttons[4] == 1:
            twist_msg.twist.angular.z = self.max_angular_speed
        elif self.buttons[5] == 1:
            twist_msg.twist.angular.z = -self.max_angular_speed
        
        self.twistpub_.publish(twist_msg)
        return

def main(args=None):
    rclpy.init(args=args)
    joy_run_node = JoyRun()

    try:
        rclpy.spin(joy_run_node)
    except ExternalShutdownException:
        pass
    finally:
        joy_run_node.destroy_node()
        rclpy.shutdown()


