#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from builtin_interfaces.msg import Duration
import threading

class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/extrusion_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # States
        self.front_pos = 0.0
        self.back_pos = 0.0
        self.gripper_pos = 0.0
        self.arm_pos = 0.0
        
        # Velocity State
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Timer for continuous velocity publishing
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Setup GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.start()

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'left_front_extrusion_joint',
            'right_front_extrusion_joint',
            'left_back_extrusion_joint',
            'right_back_extrusion_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [self.front_pos, self.front_pos, self.back_pos, self.back_pos]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points.append(point)
        self.traj_pub.publish(msg)

    # --- GUI Methods ---
    def set_vel(self, linear, angular):
        self.linear_x = float(linear)
        self.angular_z = float(angular)

    def set_front(self, pos):
        self.front_pos = float(pos)
        self.publish_trajectory()
        self.get_logger().info(f"Front Extrusions set to {self.front_pos}")

    def set_back(self, pos):
        self.back_pos = float(pos)
        self.publish_trajectory()
        self.get_logger().info(f"Back Extrusions set to {self.back_pos}")

    def set_all(self, pos):
        self.front_pos = float(pos)
        self.back_pos = float(pos)
        self.publish_trajectory()
        self.get_logger().info(f"All Extrusions set to {pos}")

    def publish_gripper_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['left_gripper_joint', 'right_gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.gripper_pos, self.gripper_pos]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        self.gripper_pub.publish(msg)

    def set_gripper(self, pos):
        self.gripper_pos = float(pos)
        self.publish_gripper_trajectory()
        self.get_logger().info(f"Gripper set to {self.gripper_pos}")

    def publish_arm_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['left_arm_joint', 'right_arm_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.arm_pos, self.arm_pos]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def set_arm(self, pos):
        self.arm_pos = float(pos)
        self.publish_arm_trajectory()
        self.get_logger().info(f"Arm set to {self.arm_pos}")

    def run_gui(self):
        root = tk.Tk()
        root.title("Robocon Control Panel")
        root.geometry("420x580")
        root.resizable(False, False)

        # Title
        tk.Label(root, text="Robot Control GUI", font=("Helvetica", 16, "bold")).pack(pady=10)

        # Drive Controls Frame
        drive_frame = tk.LabelFrame(root, text="Drive Controls (Press & Hold)", padx=10, pady=10)
        drive_frame.pack(fill="x", padx=20, pady=10)

        btn_fwd = tk.Button(drive_frame, text="Forward", width=10, bg="lightgreen")
        btn_back = tk.Button(drive_frame, text="Backward", width=10, bg="lightpink")
        btn_left = tk.Button(drive_frame, text="Left", width=10, bg="lightblue")
        btn_right = tk.Button(drive_frame, text="Right", width=10, bg="lightblue")
        btn_stop = tk.Button(drive_frame, text="STOP", width=10, bg="red", fg="white", command=lambda: self.set_vel(0, 0))

        # Grid Layout for Drive
        btn_fwd.grid(row=0, column=1, pady=5)
        btn_left.grid(row=1, column=0, padx=5)
        btn_stop.grid(row=1, column=1, padx=5)
        btn_right.grid(row=1, column=2, padx=5)
        btn_back.grid(row=2, column=1, pady=5)

        # Bindings for Drive
        vel_linear = 1.0
        vel_angular = 1.0

        btn_fwd.bind("<ButtonPress-1>", lambda e: self.set_vel(vel_linear, 0))
        btn_fwd.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0))

        btn_back.bind("<ButtonPress-1>", lambda e: self.set_vel(-vel_linear, 0))
        btn_back.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0))

        btn_left.bind("<ButtonPress-1>", lambda e: self.set_vel(0, vel_angular))
        btn_left.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0))

        btn_right.bind("<ButtonPress-1>", lambda e: self.set_vel(0, -vel_angular))
        btn_right.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0))

        # Extrusion Controls Frame
        ext_frame = tk.LabelFrame(root, text="Extrusion Mechanisms", padx=10, pady=10)
        ext_frame.pack(fill="x", padx=20, pady=5)

        tk.Button(ext_frame, text="All Ext DOWN (0.2)", width=20, bg="lightyellow", command=lambda: self.set_all(0.2)).grid(row=0, column=0, padx=5, pady=4)
        tk.Button(ext_frame, text="All Ext UP (0.0)", width=20, bg="lightyellow", command=lambda: self.set_all(0.0)).grid(row=0, column=1, padx=5, pady=4)

        tk.Button(ext_frame, text="Front Ext DOWN (0.2)", width=20, command=lambda: self.set_front(0.2)).grid(row=1, column=0, padx=5, pady=2)
        tk.Button(ext_frame, text="Front Ext UP (0.0)", width=20, command=lambda: self.set_front(0.0)).grid(row=1, column=1, padx=5, pady=2)

        tk.Button(ext_frame, text="Back Ext DOWN (0.2)", width=20, command=lambda: self.set_back(0.2)).grid(row=2, column=0, padx=5, pady=2)
        tk.Button(ext_frame, text="Back Ext UP (0.0)", width=20, command=lambda: self.set_back(0.0)).grid(row=2, column=1, padx=5, pady=2)

        # Arm Controls Frame
        arm_frame = tk.LabelFrame(root, text="Arm Controls (Pitch)", padx=10, pady=10)
        arm_frame.pack(fill="x", padx=20, pady=5)

        tk.Button(arm_frame, text="Arm UP (+)", width=20, bg="lightblue", command=lambda: self.set_arm(max(self.arm_pos + 0.2, -1.57))).grid(row=0, column=0, padx=5, pady=2)
        tk.Button(arm_frame, text="Arm DOWN (-)", width=20, bg="lightblue", command=lambda: self.set_arm(min(self.arm_pos - 0.2, 1.57))).grid(row=0, column=1, padx=5, pady=2)

        # Gripper Controls Frame
        grip_frame = tk.LabelFrame(root, text="Gripper (Scroll Pickup)", padx=10, pady=10)
        grip_frame.pack(fill="x", padx=20, pady=5)

        tk.Button(grip_frame, text="Open Gripper", width=20, bg="lightgreen", command=lambda: self.set_gripper(0.0)).grid(row=0, column=0, padx=5, pady=2)
        tk.Button(grip_frame, text="Close Gripper", width=20, bg="lightcoral", command=lambda: self.set_gripper(0.18)).grid(row=0, column=1, padx=5, pady=2)

        def on_closing():
            root.destroy()
            # In a real node, we'd also shutdown rclpy, but this handles the GUI side
            
        root.protocol("WM_DELETE_WINDOW", on_closing)
        root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
