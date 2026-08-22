#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import math

class GoalPoseController(Node):
    def __init__(self):
        super().__init__('goal_pose_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/controller/status', 10)
        
        self.path_sub = self.create_subscription(Path, '/controller/waypoints', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.waypoints = []
        
        # Run at 20Hz for smoother control
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Controller gains
        self.kp_linear = 1.0
        self.kp_angular = 20.0
        self.max_linear = 1.0
        self.max_angular = 2.0
        
        self.final_distance_tolerance = 0.1
        self.intermediate_distance_tolerance = 0.3
        self.yaw_tolerance = 0.05 # ~4.5 degrees to prevent micro-oscillations
        self.active = False
        
        self.get_logger().info("Goal Pose Controller started.")

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def path_callback(self, msg):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y, self.euler_from_quaternion(pose.pose.orientation)) for pose in msg.poses]
        if self.waypoints:
            self.active = True
            self.get_logger().info(f"Received new path with {len(self.waypoints)} waypoints.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        if not self.active or not self.waypoints:
            return
            
        goal_x, goal_y, goal_yaw = self.waypoints[0]
        
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        
        distance = math.sqrt(dx**2 + dy**2)
        twist = Twist()
        
        # Calculate yaw error for alignment
        align_yaw_error = goal_yaw - self.current_yaw
        while align_yaw_error > math.pi: align_yaw_error -= 2.0 * math.pi
        while align_yaw_error < -math.pi: align_yaw_error += 2.0 * math.pi
        
        is_final_waypoint = len(self.waypoints) == 1
        current_tolerance = self.final_distance_tolerance if is_final_waypoint else self.intermediate_distance_tolerance
        
        if distance < current_tolerance:
            
            if is_final_waypoint and abs(align_yaw_error) > self.yaw_tolerance:
                # Reached final position, but need to align to final yaw
                twist.linear.x = 0.0
                # Use a lower kp and max_speed for the final delicate alignment
                kp_align = 0.8
                max_align_speed = 0.5
                twist.angular.z = max(min(kp_align * align_yaw_error, max_align_speed), -max_align_speed)
            else:
                # Fully reached (and aligned if it's the final waypoint)
                self.waypoints.pop(0)
                if not self.waypoints:
                    self.get_logger().info("Final waypoint reached and aligned! Path complete.")
                    self.active = False
                    self.cmd_vel_pub.publish(Twist()) # Stop robot
                    
                    status_msg = Bool()
                    status_msg.data = True
                    self.status_pub.publish(status_msg)
                else:
                    self.get_logger().info("Intermediate waypoint reached, moving to next.")
                return
        else:
            # Full Omni-directional control: Independent translation and rotation
            
            # Transform global error to robot's local frame
            # The robot is rotated by self.current_yaw
            local_dx = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_dy = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            # Adaptive velocity profiling
            if len(self.waypoints) == 1:
                # Decelerate smoothly using sqrt profile for final waypoint
                speed = self.kp_linear * math.sqrt(distance)
                speed = max(speed, 0.1) # Min speed
            else:
                # Coast through intermediate waypoints
                speed = self.kp_linear * distance
                speed = max(speed, 0.6)
                
            # Scale local_dx and local_dy to match the desired speed
            if distance > 0:
                scale = min(speed, self.max_linear) / distance
            else:
                scale = 0.0
                
            twist.linear.x = local_dx * scale
            twist.linear.y = local_dy * scale
            
            # Independent angular control
            # Align smoothly to target yaw while translating
            twist.angular.z = max(min(self.kp_angular * align_yaw_error, self.max_angular), -self.max_angular)
            
            

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
