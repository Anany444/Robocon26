#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from std_msgs.msg import String
import tkinter as tk
from builtin_interfaces.msg import Duration
import threading

class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/extrusion_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.mock_kfs_pub = self.create_publisher(String, '/vision/mock_kfs_status', 10)
        
        # States
        self.front_pos = 0.0
        self.back_pos = 0.0
        self.gripper_pos = 0.0
        self.arm_pos = 0.0
        
        # Service Clients
        self.trigger_move_forest_client = self.create_client(Trigger, '/trigger_move_to_forest')
        
        # Velocity State
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Timer for continuous velocity publishing
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Setup GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.start()

    def publish_velocity(self):
        if self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0:
            return # Don't continuously publish zeroes; let physics (gravity) take over
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
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
    def set_vel(self, linear_x, linear_y, angular_z):
        self.linear_x = float(linear_x)
        self.linear_y = float(linear_y)
        self.angular_z = float(angular_z)
        if self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0:
            # Publish exactly once to ensure it stops
            msg = Twist()
            self.cmd_vel_pub.publish(msg)

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
        msg = Float64MultiArray()
        msg.data = [float(self.gripper_pos), float(self.gripper_pos)]
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

    def publish_mock_kfs(self, status):
        msg = String()
        msg.data = status
        self.mock_kfs_pub.publish(msg)
        self.get_logger().info(f"Published mock KFS status: {status}")

    def call_move_forest_trigger(self):
        if not self.trigger_move_forest_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /trigger_move_to_forest not available')
            return
        
        req = Trigger.Request()
        future = self.trigger_move_forest_client.call_async(req)
        self.get_logger().info("Called /trigger_move_to_forest")

    def run_gui(self):
        root = tk.Tk()
        root.title("Robocon Control Panel")
        root.geometry("450x750")
        root.resizable(False, False)

        # Title
        tk.Label(root, text="Robot Control GUI", font=("Helvetica", 16, "bold")).pack(pady=10)

        # Mission Controls
        mission_frame = tk.LabelFrame(root, text="Mission Triggers", padx=10, pady=10)
        mission_frame.pack(fill="x", padx=20, pady=5)
        
        tk.Button(mission_frame, text="Trigger: Move to Forest", width=25, bg="plum", 
                  command=self.call_move_forest_trigger).pack(pady=5)

        # Drive Controls Frame
        drive_frame = tk.LabelFrame(root, text="Drive Controls (Press & Hold)", padx=10, pady=10)
        drive_frame.pack(fill="x", padx=20, pady=10)

        btn_fwd = tk.Button(drive_frame, text="Forward", width=10, bg="lightgreen")
        btn_back = tk.Button(drive_frame, text="Backward", width=10, bg="lightpink")
        btn_left = tk.Button(drive_frame, text="Left", width=10, bg="lightblue")
        btn_right = tk.Button(drive_frame, text="Right", width=10, bg="lightblue")
        btn_rot_l = tk.Button(drive_frame, text="Rot L", width=10, bg="thistle")
        btn_rot_r = tk.Button(drive_frame, text="Rot R", width=10, bg="thistle")
        btn_stop = tk.Button(drive_frame, text="STOP", width=10, bg="red", fg="white", command=lambda: self.set_vel(0, 0, 0))

        # Grid Layout for Drive
        btn_fwd.grid(row=0, column=1, pady=5)
        btn_left.grid(row=1, column=0, padx=5)
        btn_stop.grid(row=1, column=1, padx=5)
        btn_right.grid(row=1, column=2, padx=5)
        btn_back.grid(row=2, column=1, pady=5)
        btn_rot_l.grid(row=3, column=0, pady=5)
        btn_rot_r.grid(row=3, column=2, pady=5)

        # Bindings for Drive
        vel_linear = 1.0
        vel_angular = 1.0

        btn_fwd.bind("<ButtonPress-1>", lambda e: self.set_vel(vel_linear, 0, 0))
        btn_fwd.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))

        btn_back.bind("<ButtonPress-1>", lambda e: self.set_vel(-vel_linear, 0, 0))
        btn_back.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))

        btn_left.bind("<ButtonPress-1>", lambda e: self.set_vel(0, vel_linear, 0))
        btn_left.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))

        btn_right.bind("<ButtonPress-1>", lambda e: self.set_vel(0, -vel_linear, 0))
        btn_right.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))
        
        btn_rot_l.bind("<ButtonPress-1>", lambda e: self.set_vel(0, 0, vel_angular))
        btn_rot_l.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))
        
        btn_rot_r.bind("<ButtonPress-1>", lambda e: self.set_vel(0, 0, -vel_angular))
        btn_rot_r.bind("<ButtonRelease-1>", lambda e: self.set_vel(0, 0, 0))

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

        # Vision Mock Frame
        vision_frame = tk.LabelFrame(root, text="Vision Mock (KFS)", padx=10, pady=10)
        vision_frame.pack(fill="x", padx=20, pady=5)
        
        tk.Button(vision_frame, text="None", width=10, bg="white", command=lambda: self.publish_mock_kfs("none")).grid(row=0, column=0, padx=5, pady=2)
        tk.Button(vision_frame, text="R1 KFS", width=10, bg="pink", command=lambda: self.publish_mock_kfs("r1_kfs")).grid(row=0, column=1, padx=5, pady=2)
        tk.Button(vision_frame, text="R2 Real", width=10, bg="lightgreen", command=lambda: self.publish_mock_kfs("r2_kfs_real")).grid(row=0, column=2, padx=5, pady=2)
        tk.Button(vision_frame, text="R2 Fake", width=10, bg="yellow", command=lambda: self.publish_mock_kfs("r2_kfs_fake")).grid(row=0, column=3, padx=5, pady=2)

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
