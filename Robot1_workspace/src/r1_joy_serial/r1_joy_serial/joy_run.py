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
        self.get_logger().info('JoyRun node has been started.')
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
        self.weapon_gripper_client = self.create_client(SetBool, 'toggle_weapon_gripper')
        self.weapon_servo_client = self.create_client(SetBool, 'toggle_weapon_servo')

        self.max_linear_speed = 1.0
        self.max_angular_speed = 0.5

        self.initialized = False

        self.gripper_on = False
        self.override_on = False
        self.weap_gripper = False
        self.weapon_servo_on = False
        self.arm_stepper_pos = 0
        self.prev_arm_step = 0
        self.weapon_stepper_pos = 0
        self.weapon_gripper_angle = 0.0
        self.wea_stp = 1
        self.arm_stp = 1

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
        # We check prev_buttons == 0 to ensure it only toggles once per press (prevents rapid bouncing)

        # Override (L2: 6) - Must be processed FIRST
        if self.buttons[6] == 1 and self.prev_buttons[6] == 0:
            self.toggle_override()

        # If override is active, block all other commands immediately
        if self.override_on:
            self.prev_buttons = list(self.buttons)
            # Ensure the robot stops moving by sending zero velocity
            self.axes = [0.0] * len(self.axes)
            self.buttons = [0] * len(self.buttons)
            self.cmd_vel_publish()
            return

        # Arm Gripper (Options: 9)
        if self.buttons[7] == 1 and self.prev_buttons[7] == 0:
            self.toggle_arm_gripper()

        # Weapon Gripper  (X: 3)
        if self.buttons[3] == 1 and self.prev_buttons[3] == 0:
            self.toggle_weapon_gripper()

        # Weapon Servo 90deg (Share: 8)
        if self.buttons[8] == 1 and self.prev_buttons[8] == 0:
            self.toggle_weapon_servo()

        # --- TOPICS (Continuous / Discrete Strings) ---
        msg = std_msgs.msg.String()

        # Arm Up/Down Stepper moved to right_stick.ud (axis 5)
        if len(self.axes) > 5:
            arm = "ARM_STOP"
            if self.axes[4] > 0.2:
                # stepper_step = 5
                msg.data = f"ARM_HIGH"
                self.publisher_.publish(msg)
                self.arm_stp = 1
            elif self.axes[4] < -0.2:
                # stepper_step = -5
                msg.data = f"ARM_LOW"
                self.publisher_.publish(msg)
                self.arm_stp = 1
            elif self.arm_stp <= 2:
                # stepper_step = 0
                msg.data = f"ARM_STOP"
                self.publisher_.publish(msg)
                self.arm_stp += 1

            # if stepper_step != getattr(self, 'prev_stepper_step', None):
            #     self.prev_stepper_step = stepper_step
            #     msg.data = arm
            #     self.publisher_.publish(msg)

        # Weapon Gripper +5 deg (B: 1)
        if self.buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.weapon_gripper_angle += 5.0
            # Clamp angle between 0 and 180 degrees
            self.weapon_gripper_angle = max(0.0, min(180.0, self.weapon_gripper_angle))
            msg.data = f"WEAPON_GRIPPER_ANGLE:{self.weapon_gripper_angle:.1f}"
            self.publisher_.publish(msg)
        # Servo Arm Step of +3 / -3 / 0 moved to Y (2) and A (0) buttons
        if self.buttons[2] == 1:
            step = 3
        elif self.buttons[0] == 1:
            step = -3
        else:
            step = 0

        if step != getattr(self, 'prev_arm_step', None):
            self.prev_arm_step = step
            msg.data = f"SERVO_ARM_STEP:{step}"
            self.publisher_.publish(msg)
        # Weapon Up/Down Stepper (dpad.up/down: 13)
        if len(self.axes) > 7:
            if self.axes[7] > 0.5:
                increment = 10
                # msg.data = f"WEAPON_STEP:{increment}"
                msg.data = f"WEAPON_HIGH"
                self.publisher_.publish(msg)
                self.wea_stp = 1
            elif self.axes[7] < -0.5:
                increment = -10
                # msg.data = f"WEAPON_STEP:{increment}"
                msg.data = f"WEAPON_LOW"
                self.publisher_.publish(msg)
                self.wea_stp = 1
            elif self.wea_stp <= 2:
                increment = 0
                # msg.data = f"WEAPON_STEP:{increment}"
                msg.data = f"WEAPON_STOP"
                self.publisher_.publish(msg)
                self.wea_stp += 1


        # Weapon Servo CW/CCW (dpad.left/right: 12)
        if len(self.axes) > 6:
            if self.axes[6] > 0.5:
                msg.data = "WEAPON_CCW"
                self.publisher_.publish(msg)
            elif self.axes[6] < -0.5:
                msg.data = "WEAPON_CW"
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
        # disable/enable all the buttons
        self.override_on = not self.override_on
        self.get_logger().error(f'Override State: {self.override_on}')
        self.send_service_request(self.override_client, self.override_on)

    def toggle_weapon_gripper(self):
        self.weap_gripper = not self.weap_gripper
        self.get_logger().warn(f'Weapon Gripper State: {self.weap_gripper}')
        self.weapon_gripper_angle = 0.0
        self.send_service_request(self.weapon_gripper_client, self.weap_gripper)

    def toggle_weapon_servo(self):
        self.weapon_servo_on = not self.weapon_servo_on
        self.get_logger().info(f'Weapon Servo State: {self.weapon_servo_on}')
        self.send_service_request(self.weapon_servo_client, self.weapon_servo_on)

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

        twist_msg.twist.linear.x = self.axes[1] * self.max_linear_speed
        twist_msg.twist.linear.y = self.axes[0] * self.max_linear_speed
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


