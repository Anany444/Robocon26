#!/usr/bin/env python3

"""Spear Alignment & Detection Node using YOLOv8 TensorRT Engine on Jetson
========================================================================
- Optimized for maximum FPS using TensorRT (.engine) and multithreaded executors.
- Detects 'spear' class, calculates pixel offset from camera frame center.
- Publishes alignment commands ("ALIGNED!", "MOVE RIGHT ->", "<- MOVE LEFT") and exact diff.
- All heavy CPU plotting/GUI disabled by default for zero latency.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
import torch
import threading
import queue
import os
import time


class SpearDetectionNode(Node):
    def __init__(self):
        super().__init__('spear_detection')

        # --- Multithreading Setup ---
        # Prevents the Python GIL from locking inference behind image callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('model_path', "/home/orin/r2_ws/src/robocon_vision/models/SpearHead.engine")
        self.declare_parameter('target_class', "fist")
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('tolerance_pixels', 10)
        # --- Velocity control parameters ---
        self.declare_parameter('min_speed', 0.08)           # minimum PWM speed to physically move the bot
        self.declare_parameter('max_alignment_speed', 0.15) # max lateral (Y) speed at full pixel error
        self.declare_parameter('max_approach_speed', 0.1)  # max forward (X) speed at full distance error
        self.declare_parameter('pixel_error_norm', 320.0)  # pixels representing "max" Y error (half frame width)
        self.declare_parameter('distance_error_norm', 0.1) # metres representing "max" X error for scaling
        self.declare_parameter('distance_min', 0.175)      # m — too close, back up
        self.declare_parameter('distance_max', 0.200)      # m — too far, move forward
        self.declare_parameter('show_gui', False)
        self.declare_parameter('enable_plotting', True)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('aligned_hold_duration', 2.0)  # seconds both X and Y must stay aligned
        # --- Frame saving for dataset collection ---
        self.declare_parameter('save_frames', True)          # enable/disable frame saving
        self.declare_parameter('save_dir', '/home/orin/r2_ws/src/robocon_vision/dataset/raw_frames')  # output directory
        self.declare_parameter('save_every_n_frames', 1)      # save every Nth frame (1 = all frames)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_thresh = self.get_parameter('conf_threshold').value
        self.tolerance = self.get_parameter('tolerance_pixels').value
        self.min_speed           = self.get_parameter('min_speed').value
        self.max_alignment_speed = self.get_parameter('max_alignment_speed').value
        self.max_approach_speed  = self.get_parameter('max_approach_speed').value
        self.pixel_error_norm    = self.get_parameter('pixel_error_norm').value
        self.distance_error_norm = self.get_parameter('distance_error_norm').value
        self.distance_min        = self.get_parameter('distance_min').value
        self.distance_max        = self.get_parameter('distance_max').value
        self.show_gui = self.get_parameter('show_gui').value
        self.enable_plotting = self.get_parameter('enable_plotting').value
        self.aligned_hold_duration = self.get_parameter('aligned_hold_duration').value
        self.save_frames = self.get_parameter('save_frames').value
        self.save_dir    = self.get_parameter('save_dir').value
        self.save_every_n = self.get_parameter('save_every_n_frames').value
        self._frame_counter = 0  # counts total frames processed

        self.declare_parameter('start_enabled', False)
        self.operation_enabled = self.get_parameter('start_enabled').value

        # Distance state — updated by /distancex subscriber
        self.current_distance_x = None  # None = no data received yet

        # Alignment stability state
        self._aligned_since = None  # time.time() when alignment first became stable, None = not aligned

        # ---------------------------------------------------------
        # Background frame saver (queue-based, zero detection latency)
        # Detection thread does: queue.put(frame)  -- ~0.05ms
        # Saver thread does:     cv2.imwrite(...)  -- runs independently
        # ---------------------------------------------------------
        self._save_queue = queue.Queue(maxsize=120)  # buffer up to 120 frames (~4s at 30fps)
        self._save_thread = threading.Thread(
            target=self._frame_saver_worker, daemon=True, name="FrameSaverThread"
        )
        if self.save_frames:
            os.makedirs(self.save_dir, exist_ok=True)
            self._save_thread.start()
            self.get_logger().info(
                f"Frame saving ENABLED → {self.save_dir} "
                f"(every {self.save_every_n} frame(s), buffer={self._save_queue.maxsize})"
            )
        else:
            self.get_logger().info("Frame saving DISABLED. Set save_frames:=true to enable.")

        # Publishers
        self.annotated_pub   = self.create_publisher(Image,       '/vision/spear_alignment/annotated_image', 10)
        self.status_pub      = self.create_publisher(String,      '/vision/spear_alignment/status', 10)
        self.diff_pub        = self.create_publisher(Float32,     '/vision/spear_alignment/diff', 10)
        self.cmd_vel_pub     = self.create_publisher(TwistStamped,'/cmd_vel', 10)
        self.fully_aligned_pub = self.create_publisher(Bool,      '/vision/spear_alignment/fully_aligned', 10)

        # Service to start/stop detection operations
        self.enable_srv = self.create_service(
            SetBool, 'enable_spear_detection', self.enable_service_callback, callback_group=self.callback_group
        )

        # Subscriber — distance from weapon in X (metres), published externally
        self.distance_sub = self.create_subscription(
            Float32, '/dist/weapon_rack', self.distance_callback, 10,
            callback_group=self.callback_group
        )

        # Load YOLO model (TensorRT .engine files already have device baked in)
        self.get_logger().info(f"Loading YOLO model from: {model_path}...")
        if model_path.endswith('.engine'):
            self.model = YOLO(model_path, task='detect')
        else:
            self.model = YOLO(model_path).to(self.device)

        # Run hardware warmup pass to initialize CUDA context & memory pools
        self.get_logger().info("Running hardware CUDA warmup frame...")
        dummy_frame = np.zeros((640, 640, 3), dtype=np.uint8)
        self.is_engine = model_path.endswith('.engine')
        if self.is_engine:
            # TRT engine has device baked in — do NOT pass device= arg
            self.model(dummy_frame, imgsz=640, verbose=False)
        else:
            self.model(dummy_frame, imgsz=640, device=self.device, verbose=False)
        self.get_logger().info("GPU Engine Initialized and Ready for High-Speed Inference!")

        # CvBridge instantiated once at init - not per-frame (saves ~1ms/frame)
        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        except Exception:
            self.bridge = None

        # Bind image subscriber - inference runs directly inside this callback
        # No timer needed: processing is purely event-driven on frame arrival
        img_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, img_topic, self.ros_image_callback, 10, callback_group=self.callback_group
        )
        # No timer - processing is triggered directly by incoming frames

    def distance_callback(self, msg: Float32):
        """Update stored X distance from /distancex topic."""
        self.current_distance_x = msg.data

    def _frame_saver_worker(self):
        """Background thread: drains save queue and writes frames to disk.
        Completely decoupled from the detection loop — no latency impact.
        """
        while True:
            try:
                item = self._save_queue.get(timeout=1.0)
                if item is None:  # sentinel: shutdown signal
                    break
                frame, filepath = item
                cv2.imwrite(filepath, frame)  # blocking disk I/O happens here, not in detection thread
                self._save_queue.task_done()
            except queue.Empty:
                continue  # keep waiting for frames
            except Exception as e:
                self.get_logger().error(f"Frame save error: {e}")

    def enable_service_callback(self, request, response):
        self.operation_enabled = request.data
        if self.operation_enabled:
            response.message = "Spear detection enabled and operational."
            self.get_logger().info(response.message)
        else:
            response.message = "Spear detection disabled."
            self.get_logger().info(response.message)
        response.success = True
        return response

    def ros_image_callback(self, msg):
        """Decode image AND run inference immediately - zero polling latency."""
        if not self.operation_enabled:
            return
        try:
            if self.bridge is not None:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                img_arr = np.frombuffer(msg.data, dtype=np.uint8)
                enc = msg.encoding.lower()
                if enc in ['yuv422', 'yuyv', 'yuyv422', 'yuv422_yuy2']:
                    img_arr = img_arr.reshape((msg.height, msg.width, 2))
                    frame = cv2.cvtColor(img_arr, cv2.COLOR_YUV2BGR_YUYV)
                elif enc in ['uyvy', 'yuv422_uyvy']:
                    img_arr = img_arr.reshape((msg.height, msg.width, 2))
                    frame = cv2.cvtColor(img_arr, cv2.COLOR_YUV2BGR_UYVY)
                else:
                    img_arr = img_arr.reshape((msg.height, msg.width, -1))
                    if enc == "rgb8":
                        frame = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)
                    elif enc == "rgba8":
                        frame = cv2.cvtColor(img_arr, cv2.COLOR_RGBA2BGR)
                    elif img_arr.shape[-1] == 4:
                        frame = cv2.cvtColor(img_arr, cv2.COLOR_BGRA2BGR)
                    else:
                        frame = img_arr
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        self.process_frame(frame)

    def process_frame(self, frame):
        """Run inference with full SpearHead logic: conflict resolution, drawing, alignment & cmd_vel."""

        # ---------------------------------------------------------
        # Enqueue frame for saving BEFORE inference (saves raw input frame)
        # Non-blocking: if queue is full, drop the frame silently
        # Cost to detection thread: ~0.05ms (just a reference copy + queue.put)
        # ---------------------------------------------------------
        self._frame_counter += 1
        if self.save_frames and (self._frame_counter % self.save_every_n == 0):
            timestamp = time.strftime("%Y%m%d_%H%M%S") + f"_{self._frame_counter:06d}"
            filepath = os.path.join(self.save_dir, f"frame_{timestamp}.jpg")
            try:
                # Non-blocking put: drops frame if queue is full (detection never waits for disk)
                self._save_queue.put_nowait((frame.copy(), filepath))
            except queue.Full:
                self.get_logger().warn("Frame save queue full — disk too slow, skipping frame.", throttle_duration_sec=5.0)
        with torch.no_grad():
            if self.is_engine:
                # TRT engine: device baked in, half precision handled internally
                results = self.model(
                    frame,
                    imgsz=640,
                    conf=self.conf_thresh,
                    verbose=False,
                )
            else:
                # PyTorch .pt model: specify device and FP16 explicitly
                results = self.model(
                    frame,
                    imgsz=640,
                    conf=self.conf_thresh,
                    verbose=False,
                    device=self.device,
                    half=(self.device == 'cuda'),
                )

        height, width = frame.shape[:2]
        camera_center_x = width // 2

        # ---------------------------------------------------------
        # Fist vs Palm Conflict Resolution (from SpearHead.py)
        # If multiple fists detected, keep highest confidence as fist,
        # relabel all others as palm
        # ---------------------------------------------------------
        fist_boxes = []
        other_boxes = []

        class_names = {0: "spear", 1: "fist", 2: "palm"}
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = class_names.get(cls_id, str(self.model.names.get(cls_id, cls_id))).lower()
            if label in ("0", "class0", "spear"):
                label = "spear"
            elif label in ("1", "class1", "fist"):
                label = "fist"
            elif label in ("2", "class2", "palm"):
                label = "palm"

            if label == "fist":
                fist_boxes.append(box)
            else:
                other_boxes.append((box, label))

        # Sort fists by confidence descending
        fist_boxes.sort(key=lambda b: float(b.conf[0]), reverse=True)

        final_boxes = list(other_boxes)  # start with non-fist detections
        if fist_boxes:
            final_boxes.append((fist_boxes[0], "fist"))         # highest conf → stays fist
            for extra in fist_boxes[1:]:
                final_boxes.append((extra, "palm"))              # rest → relabeled palm

        # ---------------------------------------------------------
        # Find the target box from resolved labels
        # ---------------------------------------------------------
        target_box = None
        for box, label in final_boxes:
            if label == self.target_class.lower():
                target_box = box
                break  # use first match (highest conf since fists are sorted)

        # ---------------------------------------------------------
        # Compute alignment status
        # ---------------------------------------------------------
        status_text = "NO TARGET"
        diff_val = 0.0
        target_center_x = None

        if target_box is not None:
            x1, y1, x2, y2 = map(int, target_box.xyxy[0])
            target_center_x = (x1 + x2) // 2
            diff_val = float(target_center_x - camera_center_x)

            if abs(diff_val) <= self.tolerance:
                status_text = "ALIGNED!"
            elif diff_val > 0:
                status_text = "MOVE RIGHT ->"
            else:
                status_text = "<- MOVE LEFT"

        # Publish status and pixel diff
        self.status_pub.publish(String(data=status_text))
        self.diff_pub.publish(Float32(data=diff_val))

        # ---------------------------------------------------------
        # Proportional velocity for X (forward/backward from /distancex)
        # ---------------------------------------------------------
        # Formula:
        #   error    = how far outside the [distance_min, distance_max] band we are
        #   raw_vel  = (error / distance_error_norm) * max_approach_speed   [0.0 – max]
        #   clamped  = clamp(raw_vel, min_speed, max_approach_speed)        [min – max]
        #   Then apply direction sign and zero inside the aligned band.
        # ---------------------------------------------------------
        vel_x = 0.0
        x_status = "X: NO DATA"

        if self.current_distance_x is not None:
            d = self.current_distance_x
            if d < self.distance_min:
                error_x = self.distance_min - d                       # positive, how much too close
                raw     = (error_x / self.distance_error_norm) * self.max_approach_speed
                vel_x   = -max(self.min_speed, min(raw, self.max_approach_speed))  # negative = back up
                x_status = f"X: TOO CLOSE ({d:.4f}m) vel={vel_x:.3f}"
            elif d > self.distance_max:
                error_x = d - self.distance_max                       # positive, how much too far
                raw     = (error_x / self.distance_error_norm) * self.max_approach_speed
                vel_x   = max(self.min_speed, min(raw, self.max_approach_speed))   # positive = approach
                x_status = f"X: TOO FAR ({d:.4f}m) vel={vel_x:.3f}"
            else:
                vel_x    = 0.0
                x_status = f"X: ALIGNED ({d:.4f}m)"

        # ---------------------------------------------------------
        # Proportional velocity for Y (lateral from pixel diff)
        # ---------------------------------------------------------
        # Formula:
        #   error    = abs(diff_val) in pixels
        #   raw_vel  = (error / pixel_error_norm) * max_alignment_speed     [0.0 – max]
        #   clamped  = clamp(raw_vel, min_speed, max_alignment_speed)       [min – max]
        #   Then apply direction sign and zero inside tolerance.
        # ---------------------------------------------------------
        vel_y = 0.0
        if status_text in ("<- MOVE LEFT", "MOVE RIGHT ->"):
            error_y = abs(diff_val)
            raw     = (error_y / self.pixel_error_norm) * self.max_alignment_speed
            speed_y = max(self.min_speed, min(raw, self.max_alignment_speed))
            vel_y   = -speed_y if status_text == "<- MOVE LEFT" else speed_y  # LEFT = negative Y

        # ---------------------------------------------------------
        # Publish combined cmd_vel (X approach + Y lateral)
        # ---------------------------------------------------------
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f"[CMD_VEL] x={vel_x:.3f} y={vel_y:.3f} | {x_status} | Y: {status_text}")

        # ---------------------------------------------------------
        # Stable Alignment Confirmation (2-second hold timer)
        # Both X AND Y must be in their aligned zones continuously.
        # Only then publish fully_aligned=True to trigger next process.
        # ---------------------------------------------------------
        y_aligned = (status_text == "ALIGNED!" and target_box is not None)
        x_aligned = (vel_x == 0.0 and self.current_distance_x is not None)
        both_aligned = y_aligned and x_aligned

        fully_aligned = False
        if both_aligned:
            if self._aligned_since is None:
                self._aligned_since = time.time()  # start the hold timer
            elif (time.time() - self._aligned_since) >= self.aligned_hold_duration:
                fully_aligned = True   # held stable long enough!
        else:
            self._aligned_since = None  # alignment broke — reset timer

        self.fully_aligned_pub.publish(Bool(data=fully_aligned))

        if fully_aligned:
            self.get_logger().info(
                f"[FULLY ALIGNED] Stable for {self.aligned_hold_duration}s — ready for next process!"
            )

        # ---------------------------------------------------------
        # Drawing (only when show_gui or enable_plotting is True)
        # ---------------------------------------------------------
        if self.enable_plotting or self.show_gui:
            annotated_frame = frame.copy()

            # Green vertical line = camera center
            cv2.line(annotated_frame, (camera_center_x, 0), (camera_center_x, height), (0, 255, 0), 2)

            # Draw all resolved boxes with color coding
            for box, label in final_boxes:
                bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                if label == "fist":
                    box_color = (0, 0, 255)    # Red
                elif label == "palm":
                    box_color = (255, 0, 0)    # Blue
                else:
                    box_color = (0, 255, 0)    # Green
                cv2.rectangle(annotated_frame, (bx1, by1), (bx2, by2), box_color, 2)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}",
                            (bx1, max(20, by1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)

            # Red vertical line = target center
            if target_center_x is not None:
                cv2.line(annotated_frame, (target_center_x, 0), (target_center_x, height), (0, 0, 255), 2)
                text_color = (0, 255, 0) if status_text == "ALIGNED!" else (0, 255, 255)
                cv2.putText(annotated_frame,
                            f"Target: {self.target_class.upper()} | {status_text}",
                            (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, text_color, 3, cv2.LINE_AA)

            if self.enable_plotting:
                try:
                    img_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                    pub_msg = Image()
                    pub_msg.header.stamp = self.get_clock().now().to_msg()
                    pub_msg.header.frame_id = 'camera_link'
                    pub_msg.height, pub_msg.width = img_rgb.shape[:2]
                    pub_msg.encoding = 'rgb8'
                    pub_msg.step = pub_msg.width * 3
                    pub_msg.data = img_rgb.tobytes()
                    self.annotated_pub.publish(pub_msg)
                except Exception as e:
                    self.get_logger().error(f"Annotated image publish error: {e}")

            if self.show_gui:
                cv2.imshow("Spear Detection", annotated_frame)
                cv2.waitKey(1)

    def destroy_node(self):
        # Gracefully shut down the frame saver thread
        if self.save_frames and self._save_thread.is_alive():
            self.get_logger().info("Waiting for frame saver to finish writing remaining frames...")
            self._save_queue.put(None)  # send sentinel to unblock worker
            self._save_thread.join(timeout=10.0)
            self.get_logger().info(f"Frame saver shut down. Total frames processed: {self._frame_counter}")
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpearDetectionNode()

    # Force ROS2 to spin up a concurrent background thread pool executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()