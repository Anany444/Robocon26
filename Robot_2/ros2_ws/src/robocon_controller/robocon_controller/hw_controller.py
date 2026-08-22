#!/usr/bin/env python3

"""Sequential Two-Stage Test Drive Controller Node for R2 (XY First, Then Slow Graceful Yaw)
===========================================================================================
- Uses LIO odometry closed-loop feedback (/aft_mapped_to_init)
- Operates in two distinct sequential phases:
    1. Phase 1 ('XY'): Drives smoothly to target (X, Y) coordinates while holding angular.z = 0.0.
       Ramps down gently upon reaching XY tolerance box.
    2. Phase 2 ('YAW'): Only initiates after XY goal is reached. Slowly and gracefully accelerates
       rotational speed to align with target yaw angle.
- Inner velocity loop is purely Proportional + Feedforward (D removed for zero vibration).
- Continuously publishes 0 cmd_vel when idle / goal reached so motors remain safely locked.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped, PointStamped, Point
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


def quaternion_to_yaw(q):
    """Extract yaw angle (in radians) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PositionPController:
    """Outer loop proportional controller mapping position error to target velocity."""
    def __init__(self, kp=1.0, max_velocity=0.3, deadband=0.02):
        self.kp = kp
        self.max_vel = max_velocity
        self.deadband = deadband

    def compute(self, pos_error):
        if abs(pos_error) < self.deadband:
            return 0.0
        target_vel = self.kp * pos_error
        return max(-self.max_vel, min(self.max_vel, target_vel))


class VelocityPDController:
    """Inner loop P+FF controller mapping velocity tracking error to cmd_vel output (D removed)."""
    def __init__(self, kp=1.0, kd=0.0, ff_weight=1.0, max_output=0.5):
        self.kp = kp
        self.kd = 0.0  # D component disabled
        self.ff = ff_weight
        self.max_out = max_output

    def reset(self):
        pass

    def compute(self, target_vel, curr_vel):
        vel_error = target_vel - curr_vel
        output = self.ff * target_vel + self.kp * vel_error
        return max(-self.max_out, min(self.max_out, output))


class TestDriveController(Node):
    def __init__(self):
        super().__init__('test_drive_controller')

        # --- Parameters ---
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('invert_linear_y', True)  # True: body right (+Y error) outputs -linear.y
        self.declare_parameter('invert_yaw', True)       # True: negates angular.z command if motor wiring requires

        # --- Acceleration / Deceleration Ramp Limits ---
        self.declare_parameter('max_accel_x', 0.1)       # m/s^2
        self.declare_parameter('max_accel_y', 0.1)       # m/s^2
        self.declare_parameter('max_accel_yaw', 0.2)     # rad/s^2 (Slow and graceful rotation ramp)

        # --- Outer Loop: Position P Gains ---
        self.declare_parameter('kp_pos_x', 1.0)
        self.declare_parameter('max_vel_x', 0.2)
        self.declare_parameter('deadband_pos_x', 0.03)

        self.declare_parameter('kp_pos_y', 1.0)
        self.declare_parameter('max_vel_y', 0.2)
        self.declare_parameter('deadband_pos_y', 0.03)

        self.declare_parameter('kp_pos_yaw', 1.5)
        self.declare_parameter('max_vel_yaw', 1.0)       # Slow rotation target velocity
        self.declare_parameter('deadband_pos_yaw_deg', 1.5)

        # --- Inner Loop: Velocity P+FF Gains ---
        self.declare_parameter('kp_vel_x', 0.8)
        self.declare_parameter('ff_vel_x', 0.3)
        self.declare_parameter('max_cmd_x', 0.4)

        self.declare_parameter('kp_vel_y', 0.8)
        self.declare_parameter('ff_vel_y', 0.3)
        self.declare_parameter('max_cmd_y', 0.4)

        self.declare_parameter('kp_vel_yaw', 0.8)
        self.declare_parameter('ff_vel_yaw', 0.3)
        self.declare_parameter('max_cmd_yaw', 1.0)

        # Goal tolerances
        self.declare_parameter('xy_tolerance', 0.05)       # meters
        self.declare_parameter('yaw_tolerance_deg', 5.0)   # degrees

        # --- Initial Auto Goal Parameters ---
        self.declare_parameter('team', 'red')              # 'red', 'blue', or 'none'
        self.declare_parameter('enable_initial_goal', True)
        self.declare_parameter('red_initial_x', 0.40)      # 40 cm
        self.declare_parameter('red_initial_y', 0.87)      # 87 cm
        self.declare_parameter('blue_initial_x', -0.75)    # -75 cm
        self.declare_parameter('blue_initial_y', 0.87)     # 87 cm

        # --- Second Auto Goal (After Spear Alignment) Parameters ---
        self.declare_parameter('red_second_x', 120.00)       # placeholder meters
        self.declare_parameter('red_second_y', 87.00)       # placeholder meters
        self.declare_parameter('red_second_yaw_deg', -90.0)  # placeholder degrees
        self.declare_parameter('blue_second_x', 0.00)     # placeholder meters
        self.declare_parameter('blue_second_y', -87.5)      # placeholder meters
        self.declare_parameter('blue_second_yaw_deg', +90.0) # placeholder degrees
        self.declare_parameter('post_alignment_delay_sec', 3.0)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_yaw = self.get_parameter('invert_yaw').value

        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = math.radians(self.get_parameter('yaw_tolerance_deg').value)

        self.max_accel_x = self.get_parameter('max_accel_x').value
        self.max_accel_y = self.get_parameter('max_accel_y').value
        self.max_accel_yaw = self.get_parameter('max_accel_yaw').value

        self.team = self.get_parameter('team').value.lower()
        self.enable_initial_goal = self.get_parameter('enable_initial_goal').value
        self.red_init_x = self.get_parameter('red_initial_x').value
        self.red_init_y = self.get_parameter('red_initial_y').value
        self.blue_init_x = self.get_parameter('blue_initial_x').value
        self.blue_init_y = self.get_parameter('blue_initial_y').value
        self.initial_goal_triggered = False

        self.red_sec_x = self.get_parameter('red_second_x').value
        self.red_sec_y = self.get_parameter('red_second_y').value
        self.red_sec_yaw = math.radians(self.get_parameter('red_second_yaw_deg').value)
        self.blue_sec_x = self.get_parameter('blue_second_x').value
        self.blue_sec_y = self.get_parameter('blue_second_y').value
        self.blue_sec_yaw = math.radians(self.get_parameter('blue_second_yaw_deg').value)
        self.post_align_delay = self.get_parameter('post_alignment_delay_sec').value
        self.second_goal_triggered = False
        self.second_goal_timer = None

        # --- Initialize Controllers ---
        self.pos_p_x = PositionPController(
            kp=self.get_parameter('kp_pos_x').value,
            max_velocity=self.get_parameter('max_vel_x').value,
            deadband=self.get_parameter('deadband_pos_x').value
        )
        self.pos_p_y = PositionPController(
            kp=self.get_parameter('kp_pos_y').value,
            max_velocity=self.get_parameter('max_vel_y').value,
            deadband=self.get_parameter('deadband_pos_y').value
        )
        self.pos_p_yaw = PositionPController(
            kp=self.get_parameter('kp_pos_yaw').value,
            max_velocity=self.get_parameter('max_vel_yaw').value,
            deadband=math.radians(self.get_parameter('deadband_pos_yaw_deg').value)
        )

        self.vel_pd_x = VelocityPDController(
            kp=self.get_parameter('kp_vel_x').value,
            ff_weight=self.get_parameter('ff_vel_x').value,
            max_output=self.get_parameter('max_cmd_x').value
        )
        self.vel_pd_y = VelocityPDController(
            kp=self.get_parameter('kp_vel_y').value,
            ff_weight=self.get_parameter('ff_vel_y').value,
            max_output=self.get_parameter('max_cmd_y').value
        )
        self.vel_pd_yaw = VelocityPDController(
            kp=self.get_parameter('kp_vel_yaw').value,
            ff_weight=self.get_parameter('ff_vel_yaw').value,
            max_output=self.get_parameter('max_cmd_yaw').value
        )

        # --- State Variables ---
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_vyaw = 0.0
        self.odom_received = False

        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_yaw = 0.0

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.goal_active = False
        self.phase = 'XY'  # 'XY' or 'YAW'
        self.controller_enabled = True

        # --- Subscribers & Publishers ---
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.goal_point_sub = self.create_subscription(PointStamped, '/goal_point', self.goal_point_callback, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.pos_err_pub = self.create_publisher(Point, '/pid_debug/pos_errors', 10)
        self.target_vel_pub = self.create_publisher(Point, '/pid_debug/target_vels', 10)
        self.vel_err_pub = self.create_publisher(Point, '/pid_debug/vel_errors', 10)
        self.status_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.fully_aligned_sub = self.create_subscription(Bool, '/vision/spear_alignment/fully_aligned', self.fully_aligned_callback, 10)

        # --- Services ---
        self.enable_srv = self.create_service(SetBool, 'enable_drive_pid', self.enable_service_callback)
        self.spear_client = self.create_client(SetBool, 'enable_spear_detection')
        self.spear_service_called = False
        self.weapon_servo_client = self.create_client(SetBool, 'rotate_weapon_servo')
        self.weapon_servo_called = False

        # --- Control Loop Timer ---
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"TestDriveController started (Sequential XY -> Yaw). Listening to LIO odom on '{odom_topic}'.")

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        self.curr_vx = msg.twist.twist.linear.x
        self.curr_vy = -msg.twist.twist.linear.y if self.invert_linear_y else msg.twist.twist.linear.y
        self.curr_vyaw = msg.twist.twist.angular.z

        if not self.odom_received:
            self.odom_received = True
            if self.enable_initial_goal and not self.initial_goal_triggered:
                if self.team == 'red':
                    self.goal_x = self.red_init_x
                    self.goal_y = self.red_init_y
                    self.goal_yaw = self.curr_yaw  # Maintain initial spawned angle
                    self.phase = 'XY'
                    self.goal_active = True
                    self.initial_goal_triggered = True
                    self.reset_inner_loops()
                    self.get_logger().info(f"[Team RED] Auto Initial Goal triggered: X={self.goal_x:.2f}m, Y={self.goal_y:.2f}m, Yaw={math.degrees(self.goal_yaw):.1f}°")
                elif self.team == 'blue':
                    self.goal_x = self.blue_init_x
                    self.goal_y = self.blue_init_y
                    self.goal_yaw = self.curr_yaw  # Maintain initial spawned angle
                    self.phase = 'XY'
                    self.goal_active = True
                    self.initial_goal_triggered = True
                    self.reset_inner_loops()
                    self.get_logger().info(f"[Team BLUE] Auto Initial Goal triggered: X={self.goal_x:.2f}m, Y={self.goal_y:.2f}m, Yaw={math.degrees(self.goal_yaw):.1f}°")
        else:
            self.odom_received = True

    def reset_inner_loops(self):
        self.vel_pd_x.reset()
        self.vel_pd_y.reset()
        self.vel_pd_yaw.reset()
        self.last_cmd_x = self.curr_vx
        self.last_cmd_y = self.curr_vy
        self.last_cmd_yaw = self.curr_vyaw

    def fully_aligned_callback(self, msg):
        if msg.data and not self.second_goal_triggered:
            self.second_goal_triggered = True
            self.get_logger().info(f"Fully aligned signal received! Waiting {self.post_align_delay:.1f}s before launching second goal...")
            self.second_goal_timer = self.create_timer(self.post_align_delay, self.launch_second_goal)

    def launch_second_goal(self):
        if self.second_goal_timer is not None:
            self.second_goal_timer.cancel()
            self.second_goal_timer = None

        if self.spear_client.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self.spear_client.call_async(req)
            self.get_logger().info("Sent request to disable spear detection node before moving to second goal.")

        if self.team == 'red':
            self.goal_x = self.red_sec_x
            self.goal_y = self.red_sec_y
            self.goal_yaw = self.red_sec_yaw
        elif self.team == 'blue':
            self.goal_x = self.blue_sec_x
            self.goal_y = self.blue_sec_y
            self.goal_yaw = self.blue_sec_yaw
        else:
            self.get_logger().warn(f"Team '{self.team}' not recognized for second goal.")
            return

        self.phase = 'XY'
        self.goal_active = True
        self.reset_inner_loops()
        self.get_logger().info(f"[Team {self.team.upper()}] Second Goal triggered: Phase 1 (XY) to X={self.goal_x:.2f}m, Y={self.goal_y:.2f}m -> Phase 2 (Yaw) to {math.degrees(self.goal_yaw):.1f}°")

    def goal_pose_callback(self, msg):
        if not self.odom_received:
            self.get_logger().warn("Cannot accept goal: No odometry received yet!")
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)
        self.phase = 'XY'
        self.goal_active = True

        self.reset_inner_loops()
        self.get_logger().info(f"New Sequential Goal: Phase 1 (XY) to X={self.goal_x:.2f}, Y={self.goal_y:.2f} -> Phase 2 (Yaw) to {math.degrees(self.goal_yaw):.1f}°")

    def goal_point_callback(self, msg):
        if not self.odom_received:
            self.get_logger().warn("Cannot accept goal: No odometry received yet!")
            return

        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.goal_yaw = self.curr_yaw  # Maintain initial yaw when only point is provided
        self.phase = 'XY'
        self.goal_active = True

        self.reset_inner_loops()
        self.get_logger().info(f"New Point Goal: Phase 1 (XY) to X={self.goal_x:.2f}, Y={self.goal_y:.2f} (Holding Yaw at {math.degrees(self.goal_yaw):.1f}°)")

    def enable_service_callback(self, request, response):
        self.controller_enabled = request.data
        if not self.controller_enabled:
            self.stop_robot()
            self.goal_active = False
            response.message = "Drive PID Controller disabled. Robot stopped."
        else:
            response.message = "Drive PID Controller enabled."
        response.success = True
        self.get_logger().info(response.message)
        return response

    def call_spear_enable_service(self):
        if not self.spear_client.service_is_ready():
            self.get_logger().info("Waiting for 'enable_spear_detection' service...")
        req = SetBool.Request()
        req.data = True
        future = self.spear_client.call_async(req)
        future.add_done_callback(self.spear_service_response_callback)

    def spear_service_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Successfully started spear detection operation: {res.message}")
            else:
                self.get_logger().warn(f"Spear detection service returned failure: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to call spear detection service: {e}")

    def call_weapon_servo_service(self):
        if not self.weapon_servo_client.service_is_ready():
            self.get_logger().info("Waiting for 'rotate_weapon_servo' service...")
        req = SetBool.Request()
        req.data = True
        future = self.weapon_servo_client.call_async(req)
        future.add_done_callback(self.weapon_servo_response_callback)

    def weapon_servo_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Successfully rotated weapon servo: {res.message}")
            else:
                self.get_logger().warn(f"Weapon servo service returned failure: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to call weapon servo service: {e}")

    def stop_robot(self):
        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_yaw = 0.0
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        if not self.controller_enabled or not self.odom_received:
            self.stop_robot()
            return

        if not self.goal_active:
            # Continuously publish 0 cmd_vel when idle / goal reached
            self.stop_robot()
            return

        dt = 1.0 / self.control_rate
        max_dv_x = self.max_accel_x * dt
        max_dv_y = self.max_accel_y * dt
        max_dv_yaw = self.max_accel_yaw * dt

        # ==========================================
        # PHASE 1: XY TRANSLATION (Yaw Held at 0.0)
        # ==========================================
        if self.phase == 'XY':
            dx_world = self.goal_x - self.curr_x
            dy_world = self.goal_y - self.curr_y
            dist_error = math.hypot(dx_world, dy_world)

            cos_th = math.cos(self.curr_yaw)
            sin_th = math.sin(self.curr_yaw)
            e_front = dx_world * cos_th + dy_world * sin_th
            e_right = dx_world * sin_th - dy_world * cos_th

            pos_err_msg = Point(x=float(e_front), y=float(e_right), z=0.0)
            self.pos_err_pub.publish(pos_err_msg)

            # Check if XY target reached
            if dist_error < self.xy_tol:
                if abs(self.last_cmd_x) > 0.02 or abs(self.last_cmd_y) > 0.02:
                    # Ramp down smoothly inside tolerance box before transitioning
                    self.last_cmd_x += max(-max_dv_x, min(max_dv_x, 0.0 - self.last_cmd_x))
                    self.last_cmd_y += max(-max_dv_y, min(max_dv_y, 0.0 - self.last_cmd_y))
                    twist = TwistStamped()
                    twist.header.stamp = self.get_clock().now().to_msg()
                    twist.header.frame_id = 'base_link'
                    twist.twist.linear.x = float(self.last_cmd_x)
                    twist.twist.linear.y = float(-self.last_cmd_y if self.invert_linear_y else self.last_cmd_y)
                    self.cmd_vel_pub.publish(twist)
                    return
                else:
                    self.last_cmd_x = 0.0
                    self.last_cmd_y = 0.0
                    self.phase = 'YAW'
                    self.get_logger().info(f"Phase 1 (XY) Reached! Final Dist Error: {dist_error*100:.1f} cm. Initiating Phase 2 (Yaw)...")
                    return

            v_target_x = self.pos_p_x.compute(e_front)
            v_target_y = self.pos_p_y.compute(e_right)
            self.target_vel_pub.publish(Point(x=float(v_target_x), y=float(v_target_y), z=0.0))

            vel_err_x = v_target_x - self.curr_vx
            vel_err_y = v_target_y - self.curr_vy
            self.vel_err_pub.publish(Point(x=float(vel_err_x), y=float(vel_err_y), z=0.0))

            raw_cmd_x = self.vel_pd_x.compute(v_target_x, self.curr_vx)
            raw_cmd_y = self.vel_pd_y.compute(v_target_y, self.curr_vy)

            cmd_x = self.last_cmd_x + max(-max_dv_x, min(max_dv_x, raw_cmd_x - self.last_cmd_x))
            cmd_y = self.last_cmd_y + max(-max_dv_y, min(max_dv_y, raw_cmd_y - self.last_cmd_y))

            self.last_cmd_x = cmd_x
            self.last_cmd_y = cmd_y

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = float(cmd_x)
            twist.twist.linear.y = float(-cmd_y if self.invert_linear_y else cmd_y)
            twist.twist.angular.z = 0.0  # Strictly 0 during XY phase
            self.cmd_vel_pub.publish(twist)

        # ==========================================
        # PHASE 2: YAW ROTATION (XY Held at 0.0)
        # ==========================================
        elif self.phase == 'YAW':
            e_yaw = normalize_angle(self.goal_yaw - self.curr_yaw)
            self.pos_err_pub.publish(Point(x=0.0, y=0.0, z=float(e_yaw)))

            if abs(e_yaw) < self.yaw_tol:
                if abs(self.last_cmd_yaw) > 0.02:
                    self.last_cmd_yaw += max(-max_dv_yaw, min(max_dv_yaw, 0.0 - self.last_cmd_yaw))
                    twist = TwistStamped()
                    twist.header.stamp = self.get_clock().now().to_msg()
                    twist.header.frame_id = 'base_link'
                    cmd_yaw_out = -self.last_cmd_yaw if self.invert_yaw else self.last_cmd_yaw
                    twist.twist.angular.z = float(cmd_yaw_out)
                    self.cmd_vel_pub.publish(twist)
                    return
                else:
                    self.stop_robot()
                    self.goal_active = False
                    reached_msg = Bool(data=True)
                    self.status_pub.publish(reached_msg)
                    self.get_logger().info(f"Goal completely reached (XY + Yaw)! Final Yaw Error: {math.degrees(e_yaw):.1f}°. Robot stopped.")
                    if self.initial_goal_triggered and not self.spear_service_called and not self.second_goal_triggered:
                        self.spear_service_called = True
                        self.get_logger().info("Initial Auto-Goal completed! Sending start request to spearhead node...")
                        self.call_spear_enable_service()
                    elif self.second_goal_triggered and not self.weapon_servo_called:
                        self.weapon_servo_called = True
                        self.get_logger().info("Second Auto-Goal completed! Sending service request to rotate weapon servo...")
                        self.call_weapon_servo_service()
                    return

            v_target_yaw = self.pos_p_yaw.compute(e_yaw)
            self.target_vel_pub.publish(Point(x=0.0, y=0.0, z=float(v_target_yaw)))

            vel_err_yaw = v_target_yaw - self.curr_vyaw
            self.vel_err_pub.publish(Point(x=0.0, y=0.0, z=float(vel_err_yaw)))

            raw_cmd_yaw = self.vel_pd_yaw.compute(v_target_yaw, self.curr_vyaw)
            cmd_yaw = self.last_cmd_yaw + max(-max_dv_yaw, min(max_dv_yaw, raw_cmd_yaw - self.last_cmd_yaw))
            self.last_cmd_yaw = cmd_yaw
            print(math.degrees(e_yaw), v_target_yaw, cmd_yaw, self.curr_vyaw)

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            cmd_yaw_out = -cmd_yaw if self.invert_yaw else cmd_yaw
            twist.twist.angular.z = float(cmd_yaw_out)
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TestDriveController()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
