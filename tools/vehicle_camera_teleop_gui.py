#!/usr/bin/env python3
"""Qt dual-camera viewer with dead-man teleop and local recording."""

import argparse
import json
import math
import os
import queue
import shlex
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

# Select the vehicle DDS domain before rclpy is imported.
_bootstrap_parser = argparse.ArgumentParser(add_help=False)
_bootstrap_parser.add_argument("--vehicle", type=int, choices=(1, 2))
_bootstrap_parser.add_argument("--ros-domain-id", type=int, default=None)
_bootstrap_args, _ = _bootstrap_parser.parse_known_args()
_bootstrap_domain = _bootstrap_args.ros_domain_id
if _bootstrap_domain is None and _bootstrap_args.vehicle in (1, 2):
    _bootstrap_domain = 214 + _bootstrap_args.vehicle
if _bootstrap_domain is not None:
    os.environ["ROS_DOMAIN_ID"] = str(_bootstrap_domain)

# The vehicle container defaults to the POSIX locale, which makes Qt render
# Korean UI text and notes incorrectly even though metadata is written as UTF-8.
for locale_variable in ("LANG", "LC_ALL"):
    if not os.environ.get(locale_variable):
        os.environ[locale_variable] = "C.UTF-8"

# Match the vehicle runtime before importing rclpy, which selects its RMW
# implementation during import on some ROS 2 distributions.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

# Allow this single file to be launched from a plain Ubuntu terminal without
# making the operator source ROS manually first.
if not os.environ.get("VEHICLE_GUI_ROS_READY"):
    ros_setup = next(
        (path for path in (Path("/opt/ros/humble/setup.bash"), Path("/opt/ros/jazzy/setup.bash")) if path.exists()),
        None,
    )
    if ros_setup is not None:
        workspace_setup = Path("/home/ubuntu/ros2_ws/install/setup.bash")
        setup_commands = [f"source {shlex.quote(str(ros_setup))}"]
        if workspace_setup.exists():
            setup_commands.append(f"source {shlex.quote(str(workspace_setup))}")
        environment = os.environ.copy()
        environment["VEHICLE_GUI_ROS_READY"] = "1"
        if _bootstrap_domain is not None:
            environment["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
        # Conda's Python and libstdc++ are incompatible with the system ROS 2
        # extension modules.  Prefer the OS Python whenever the GUI is started
        # from an activated Conda shell.
        python_executable = "/usr/bin/python3" if environment.get("CONDA_PREFIX") else sys.executable
        if python_executable == "/usr/bin/python3":
            environment.pop("LD_LIBRARY_PATH", None)
        os.execve(
            "/bin/bash",
            [
                "bash", "-lc",
                f'{" && ".join(setup_commands)} && exec "$@"',
                "vehicle-gui", python_executable, str(Path(__file__).resolve()), *sys.argv[1:],
            ],
            environment,
        )

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from python_qt_binding.QtCore import QEvent, QPointF, QRect, Qt, QTimer
from python_qt_binding.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDialog, QGridLayout, QGroupBox, QHBoxLayout,
    QDoubleSpinBox, QLabel, QListWidget, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider,
    QVBoxLayout, QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String, UInt16


class DevControlClientNode(Node):
    """ROS I/O client for the optional development GUI/UI."""

    def __init__(self, args):
        super().__init__(getattr(args, "node_name", "dev_control_client"))
        self.viewer_only = args.viewer_only
        self.bridge = CvBridge()
        self.frame = None
        self.frame_sequence = 0
        self.last_frame_monotonic = 0.0
        self.secondary_frame = None
        self.secondary_frame_sequence = 0
        self.secondary_last_frame_monotonic = 0.0
        self.tertiary_frame = None
        self.tertiary_frame_sequence = 0
        self.tertiary_last_frame_monotonic = 0.0
        self.secondary_stream_stop = threading.Event()
        self.external_stream_threads = []
        self.control_address = (args.control_host, args.control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_base_url = args.primary_video_url.rsplit("/stream", 1)[0]
        self.front_range = math.inf
        self.closest_obstacle_angle_rad = None
        self.odom_yaw_unwrapped = None
        self._last_odom_yaw = None
        self.odom_position = None
        self.last_odom_monotonic = 0.0
        self.odom_path_m = 0.0
        self.motor_events = []
        self.last_motor_active_monotonic = 0.0
        self.battery_voltage_mv = None
        self.battery_samples = []
        self.last_battery_monotonic = 0.0
        self.safety_min_valid_range = args.safety_min_valid_range
        self.lidar_self_filter_distance_m = getattr(
            args, "lidar_self_filter_distance_m", 0.25
        )
        self.last_scan_monotonic = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = None
        if getattr(args, "disable_primary_camera", False):
            pass
        elif args.primary_video_url:
            self.start_mjpeg_stream(args.primary_video_url, "primary")
        else:
            self.image_sub = self.create_subscription(Image, args.image_topic, self.on_image, qos)
        self.secondary_image_sub = None
        if args.secondary_image_topic:
            self.secondary_image_sub = self.create_subscription(
                Image, args.secondary_image_topic, self.on_secondary_image, qos
            )
        elif not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_1_video_url, "secondary")
        if not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_2_video_url, "tertiary")
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.motor_sub = self.create_subscription(
            MotorsState, args.motor_command_topic, self.on_motor_command, 50
        )
        self.battery_sub = self.create_subscription(
            UInt16, args.battery_topic, self.on_battery, 10
        )
        self.latest_detection = None
        self.latest_detection_monotonic = 0.0
        self.detection_sub = self.create_subscription(
            String, args.detection_topic, self.on_detection, 10
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.fork_pub = self.create_publisher(String, args.fork_command_topic, 10)
        self.arrival_pub = self.create_publisher(String, args.arrival_topic, 10)
        self.auto_dock_stop_pub = self.create_publisher(
            Empty, args.auto_dock_stop_topic, 10
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.auto_dock_status = {"state": "unknown", "reason": "no_status"}
        self.auto_dock_status_sub = self.create_subscription(
            String, args.auto_dock_status_topic, self.on_auto_dock_status, status_qos
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_image = None
        self.map_metadata = None
        self.map_sequence = 0
        self.entity_map = {"state": "waiting", "entities": []}
        self.entity_map_sequence = 0
        self.map_sub = self.create_subscription(
            OccupancyGrid, args.map_topic, self.on_map, map_qos
        )
        self.entity_map_sub = self.create_subscription(
            String, args.entity_map_topic, self.on_entity_map, map_qos
        )

    def publish_arrival(self, left, right):
        self.arrival_pub.publish(String(data=f"arrived {left} {right}"))

    def publish_auto_dock_stop(self):
        self.auto_dock_stop_pub.publish(Empty())

    def on_auto_dock_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.auto_dock_status = status
        except (TypeError, ValueError):
            self.get_logger().warning("invalid auto_dock status JSON received")

    def on_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return
        occupancy = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
        grayscale = np.full((height, width), 110, dtype=np.uint8)
        known = occupancy >= 0
        grayscale[known] = np.clip(
            254.0 - occupancy[known].astype(np.float32) * 2.34, 20, 254
        ).astype(np.uint8)
        grayscale = np.flipud(grayscale)
        orientation = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.map_image = QImage(
            grayscale.data, width, height, width, QImage.Format_Grayscale8
        ).copy()
        self.map_metadata = {
            "width": width,
            "height": height,
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "origin_yaw": origin_yaw,
            "frame_id": str(msg.header.frame_id),
        }
        self.map_sequence += 1

    def on_entity_map(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.entity_map = payload
                self.entity_map_sequence += 1
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid tag entity map JSON received")

    def on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.latest_detection_monotonic = time.monotonic()
        except (TypeError, ValueError):
            self.get_logger().warning("invalid detection JSON received")

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_sequence += 1
        self.last_frame_monotonic = time.monotonic()

    def on_secondary_image(self, msg):
        self.secondary_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.secondary_frame_sequence += 1
        self.secondary_last_frame_monotonic = time.monotonic()

    def start_mjpeg_stream(self, url, stream_name):
        thread = threading.Thread(target=self.read_mjpeg, args=(url, stream_name), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg(self, url, stream_name):
        frame_attr = "frame" if stream_name == "primary" else f"{stream_name}_frame"
        sequence_attr = "frame_sequence" if stream_name == "primary" else f"{stream_name}_frame_sequence"
        age_attr = "last_frame_monotonic" if stream_name == "primary" else f"{stream_name}_last_frame_monotonic"
        while not self.secondary_stream_stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=5) as response:
                    buffer = bytearray()
                    while not self.secondary_stream_stop.is_set():
                        chunk = response.read(16384)
                        if not chunk:
                            break
                        buffer.extend(chunk)
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                        if start >= 0 and end >= 0:
                            jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                            del buffer[:end + 2]
                            if frame is not None:
                                setattr(self, frame_attr, frame)
                                setattr(self, sequence_attr, getattr(self, sequence_attr) + 1)
                                setattr(self, age_attr, time.monotonic())
                        elif len(buffer) > 2_000_000:
                            del buffer[:-2]
            except Exception as exc:
                self.get_logger().warning(f"external webcam {stream_name} unavailable: {exc}")
                self.secondary_stream_stop.wait(1.0)

    def on_scan(self, msg):
        self.last_scan_monotonic = time.monotonic()
        values = []
        for index, value in enumerate(msg.ranges):
            if math.isfinite(value) and max(
                msg.range_min,
                self.safety_min_valid_range,
                self.lidar_self_filter_distance_m,
            ) <= value <= msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment
                values.append((value, math.atan2(math.sin(angle), math.cos(angle))))
        if values:
            self.front_range, self.closest_obstacle_angle_rad = min(values)
        else:
            self.front_range = math.inf
            self.closest_obstacle_angle_rad = None

    def on_odom(self, msg):
        self.last_odom_monotonic = time.monotonic()
        position = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if self.odom_position is not None:
            self.odom_path_m += math.dist(position, self.odom_position)
        self.odom_position = position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        if self._last_odom_yaw is None:
            self.odom_yaw_unwrapped = yaw
        else:
            delta = math.atan2(
                math.sin(yaw - self._last_odom_yaw),
                math.cos(yaw - self._last_odom_yaw),
            )
            self.odom_yaw_unwrapped += delta
        self._last_odom_yaw = yaw

    def on_motor_command(self, msg):
        if any(abs(float(motor.rps)) > 1e-4 for motor in msg.data):
            self.last_motor_active_monotonic = time.monotonic()
        self.motor_events.append({
            "monotonic": time.monotonic(),
            "motors": [
                {"id": int(motor.id), "rps": float(motor.rps)} for motor in msg.data
            ],
        })
        if len(self.motor_events) > 20000:
            del self.motor_events[:10000]

    def on_battery(self, msg):
        now = time.monotonic()
        self.battery_voltage_mv = int(msg.data)
        self.last_battery_monotonic = now
        self.battery_samples.append((now, self.battery_voltage_mv))
        cutoff = now - 600.0
        while self.battery_samples and self.battery_samples[0][0] < cutoff:
            del self.battery_samples[0]

    def publish(self, linear_x, linear_y, angular_z):
        if self.viewer_only:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop(self, repeats=1):
        if self.viewer_only:
            return
        for _ in range(repeats):
            self.cmd_pub.publish(Twist())

    def publish_fork(self, command):
        if self.viewer_only:
            return
        self.fork_pub.publish(String(data=command))

    def select_target(self, top_left, top_right):
        payload = {"type": "target", "top_left": top_left, "top_right": top_right}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def update_pose_config(
        self, camera_pitch_deg, distance_scale, distance_offset, yaw_bias_deg
    ):
        payload = {
            "type": "pose_config",
            "camera_pitch_deg": camera_pitch_deg,
            "camera_distance_scale_cm_per_pnp_unit": distance_scale,
            "camera_distance_offset_cm": distance_offset,
            "yaw_bias_deg": yaw_bias_deg,
        }
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def start_telemetry(self, session):
        payload = {"type": "recording", "action": "start", "session": session}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def log_telemetry_event(self, event_type, data):
        payload = {"type": "telemetry_event", "event_type": event_type, "data": data}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def finish_telemetry(self, session, destination):
        payload = {"type": "recording", "action": "stop", "session": session}
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)
        for _ in range(10):
            try:
                time.sleep(0.1)
                urllib.request.urlretrieve(
                    f"{self.telemetry_base_url}/telemetry/{session}.jsonl", str(destination)
                )
                return True
            except Exception:
                continue
        return False


# Compatibility for the archived 1.1 headless runner. New GUI/UI code uses the
# role-specific name above.
TeleopNode = DevControlClientNode


class HttpViewerSource:
    """Three MJPEG inputs with the same frame interface as DevControlClientNode."""

    def __init__(self, args):
        self.frame = None
        self.frame_sequence = 0
        self.last_frame_monotonic = 0.0
        self.secondary_frame = None
        self.secondary_frame_sequence = 0
        self.secondary_last_frame_monotonic = 0.0
        self.tertiary_frame = None
        self.tertiary_frame_sequence = 0
        self.tertiary_last_frame_monotonic = 0.0
        self.front_range = math.inf
        self.odom_yaw_unwrapped = None
        self.odom_position = None
        self.last_odom_monotonic = 0.0
        self.odom_path_m = 0.0
        self.last_scan_monotonic = 0.0
        self.motor_events = []
        self.last_motor_active_monotonic = 0.0
        self.battery_voltage_mv = None
        self.battery_samples = []
        self.last_battery_monotonic = 0.0
        self.latest_detection = None
        self.latest_detection_monotonic = 0.0
        self.secondary_stream_stop = threading.Event()
        self.external_stream_threads = []
        self.child_processes = []
        self.control_address = (args.control_host, args.control_port)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_url = args.control_url
        self.control_command = args.control_command
        self.control_process = None
        self.control_queue = queue.Queue(maxsize=32)
        self.telemetry_base_url = args.primary_video_url.rsplit("/stream", 1)[0]
        if args.primary_video_command:
            self.start_mjpeg_command(args.primary_video_command)
        else:
            self.start_mjpeg_stream(args.primary_video_url, "primary")
        if not args.disable_external_webcams:
            self.start_mjpeg_stream(args.webcam_1_video_url, "secondary")
            self.start_mjpeg_stream(args.webcam_2_video_url, "tertiary")
        if self.control_url or self.control_command:
            thread = threading.Thread(target=self.send_control_http, daemon=True)
            thread.start()
            self.external_stream_threads.append(thread)

    def start_mjpeg_command(self, command):
        thread = threading.Thread(target=self.read_mjpeg_command, args=(command,), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg_command(self, command):
        while not self.secondary_stream_stop.is_set():
            process = subprocess.Popen(
                shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            self.child_processes.append(process)
            buffer = bytearray()
            try:
                while not self.secondary_stream_stop.is_set():
                    chunk = process.stdout.read(16384)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                    if start >= 0 and end >= 0:
                        jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                        frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                        del buffer[:end + 2]
                        if frame is not None:
                            self.frame = frame
                            self.frame_sequence += 1
                            self.last_frame_monotonic = time.monotonic()
                    elif len(buffer) > 2_000_000:
                        del buffer[:-2]
            finally:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                if process in self.child_processes:
                    self.child_processes.remove(process)
            self.secondary_stream_stop.wait(1.0)

    def start_mjpeg_stream(self, url, stream_name):
        thread = threading.Thread(target=self.read_mjpeg, args=(url, stream_name), daemon=True)
        thread.start()
        self.external_stream_threads.append(thread)

    def read_mjpeg(self, url, stream_name):
        frame_attr = "frame" if stream_name == "primary" else f"{stream_name}_frame"
        sequence_attr = "frame_sequence" if stream_name == "primary" else f"{stream_name}_frame_sequence"
        age_attr = "last_frame_monotonic" if stream_name == "primary" else f"{stream_name}_last_frame_monotonic"
        while not self.secondary_stream_stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=5) as response:
                    buffer = bytearray()
                    while not self.secondary_stream_stop.is_set():
                        chunk = response.read(16384)
                        if not chunk:
                            break
                        buffer.extend(chunk)
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                        if start >= 0 and end >= 0:
                            jpg = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                            del buffer[:end + 2]
                            if frame is not None:
                                setattr(self, frame_attr, frame)
                                setattr(self, sequence_attr, getattr(self, sequence_attr) + 1)
                                setattr(self, age_attr, time.monotonic())
                        elif len(buffer) > 2_000_000:
                            del buffer[:-2]
            except Exception:
                self.secondary_stream_stop.wait(1.0)

    def send_control(self, payload):
        if not self.control_url and not self.control_command:
            self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)
            return
        try:
            self.control_queue.put_nowait(payload)
        except queue.Full:
            try:
                self.control_queue.get_nowait()
            except queue.Empty:
                pass
            self.control_queue.put_nowait(payload)

    def send_control_http(self):
        while not self.secondary_stream_stop.is_set():
            try:
                payload = self.control_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if self.control_command:
                    if self.control_process is None or self.control_process.poll() is not None:
                        self.control_process = subprocess.Popen(
                            shlex.split(self.control_command),
                            stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        self.child_processes.append(self.control_process)
                    self.control_process.stdin.write(
                        json.dumps(payload).encode("utf-8") + b"\n"
                    )
                    self.control_process.stdin.flush()
                else:
                    request = urllib.request.Request(
                        self.control_url,
                        data=json.dumps(payload).encode("utf-8"),
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    with urllib.request.urlopen(request, timeout=1.0):
                        pass
            except Exception:
                if self.control_process is not None:
                    if self.control_process in self.child_processes:
                        self.child_processes.remove(self.control_process)
                    self.control_process = None
                continue

    def publish(self, linear_x, linear_y, angular_z):
        self.send_control({
            "type": "drive", "linear_x": linear_x,
            "linear_y": linear_y, "angular_z": angular_z,
        })

    def stop(self, repeats=1):
        for _ in range(repeats):
            self.publish(0.0, 0.0, 0.0)

    def publish_fork(self, command):
        self.send_control({"type": "fork", "command": command})

    def select_target(self, top_left, top_right):
        self.send_control({"type": "target", "top_left": top_left, "top_right": top_right})

    def update_pose_config(
        self, camera_pitch_deg, distance_scale, distance_offset, yaw_bias_deg
    ):
        self.send_control({
            "type": "pose_config",
            "camera_pitch_deg": camera_pitch_deg,
            "camera_distance_scale_cm_per_pnp_unit": distance_scale,
            "camera_distance_offset_cm": distance_offset,
            "yaw_bias_deg": yaw_bias_deg,
        })

    def start_telemetry(self, session):
        self.send_control({"type": "recording", "action": "start", "session": session})

    def log_telemetry_event(self, event_type, data):
        self.send_control({"type": "telemetry_event", "event_type": event_type, "data": data})

    def finish_telemetry(self, session, destination):
        self.send_control({"type": "recording", "action": "stop", "session": session})
        for _ in range(10):
            try:
                time.sleep(0.1)
                urllib.request.urlretrieve(
                    f"{self.telemetry_base_url}/telemetry/{session}.jsonl", str(destination)
                )
                return True
            except Exception:
                continue
        return False


class VideoLabel(QLabel):
    """Fixed-size video viewport that does not feed each frame back into layout."""

    def __init__(self, placeholder, width, height):
        super().__init__(placeholder)
        self._pixmap = None
        self._source_size = None
        self._draw_rect = QRect()
        self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(width, height)
        self.setStyleSheet("background: #090909; color: #d9d9d9;")

    def set_frame(self, image):
        self._pixmap = QPixmap.fromImage(image)
        if self._source_size != self._pixmap.size():
            self._source_size = self._pixmap.size()
            self._update_draw_rect()
        self.update()

    def resizeEvent(self, event):
        self._update_draw_rect()
        super().resizeEvent(event)

    def _update_draw_rect(self):
        if self._pixmap is None or self._pixmap.isNull():
            self._draw_rect = QRect()
            return
        source_size = self._pixmap.size()
        target_size = source_size.scaled(self.contentsRect().size(), Qt.KeepAspectRatio)
        self._draw_rect = QRect(
            (self.width() - target_size.width()) // 2,
            (self.height() - target_size.height()) // 2,
            target_size.width(),
            target_size.height(),
        )

    def paintEvent(self, event):
        if self._pixmap is None:
            super().paintEvent(event)
            return
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.palette().window())
        painter.drawPixmap(self._draw_rect, self._pixmap)


class EntityMapView(QWidget):
    """Low-cost OccupancyGrid view with persistent pallet-face markers."""

    SYMBOL_TEXT = {
        "star": "★", "diamond": "◆", "spade": "♠",
        "clover": "♣", "heart": "♥",
    }

    def __init__(self):
        super().__init__()
        self.map_image = None
        self.metadata = None
        self.entity_map = {"state": "waiting", "entities": []}
        self.setFixedSize(560, 220)
        self.setStyleSheet("background: #090909;")

    def set_data(self, map_image, metadata, entity_map):
        self.map_image = map_image
        self.metadata = metadata
        self.entity_map = entity_map or {"state": "waiting", "entities": []}
        self.update()

    def map_pixel(self, pose):
        metadata = self.metadata
        dx = float(pose["x"]) - metadata["origin_x"]
        dy = float(pose["y"]) - metadata["origin_y"]
        c = math.cos(metadata["origin_yaw"])
        s = math.sin(metadata["origin_yaw"])
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (
            local_x / metadata["resolution"],
            metadata["height"] - 1.0 - local_y / metadata["resolution"],
        )

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#090909"))
        if self.map_image is None or self.metadata is None:
            painter.setPen(QColor("#d9d9d9"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Nav2 /map 수신 대기")
            return
        target_size = self.map_image.size().scaled(
            self.contentsRect().size(), Qt.KeepAspectRatio
        )
        draw_rect = QRect(
            (self.width() - target_size.width()) // 2,
            (self.height() - target_size.height()) // 2,
            target_size.width(), target_size.height(),
        )
        painter.drawImage(draw_rect, self.map_image)
        scale_x = draw_rect.width() / max(self.metadata["width"], 1)
        scale_y = draw_rect.height() / max(self.metadata["height"], 1)
        painter.setPen(QPen(QColor("#00e5ff"), 2))
        painter.setBrush(QColor("#00e5ff"))
        for entity in self.entity_map.get("entities", []):
            pose = entity.get("pose") if isinstance(entity, dict) else None
            if not isinstance(pose, dict):
                continue
            try:
                source_x, source_y = self.map_pixel(pose)
                x = draw_rect.left() + source_x * scale_x
                y = draw_rect.top() + source_y * scale_y
                yaw = float(pose.get("yaw", 0.0)) - self.metadata["origin_yaw"]
            except (KeyError, TypeError, ValueError):
                continue
            center = QPointF(x, y)
            heading = QPointF(x + 18.0 * math.cos(yaw), y - 18.0 * math.sin(yaw))
            painter.drawEllipse(center, 5.0, 5.0)
            painter.drawLine(center, heading)
            matrix = entity.get("matrix") or []
            symbols = [self.SYMBOL_TEXT.get(str(item), str(item)[:1]) for item in matrix]
            label = "".join(symbols[:2]) + ("/" + "".join(symbols[2:4]) if len(symbols) >= 4 else "")
            painter.drawText(QPointF(x + 8.0, y - 7.0), f"{entity.get('id', '?')} {label}")
        painter.setPen(QColor("#f2f2f2"))
        state = str(self.entity_map.get("state", "waiting"))
        count = len(self.entity_map.get("entities", []))
        painter.drawText(draw_rect.adjusted(8, 6, -8, -6), Qt.AlignLeft | Qt.AlignTop,
                         f"{self.metadata['frame_id']} | {state} | entities {count}")


class TeleopWindow(QMainWindow):
    MOVEMENT_KEYS = {Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Q, Qt.Key_E}
    FORK_KEYS = {Qt.Key_Up, Qt.Key_Down}
    LATERAL_ACCEL_M_S2 = 0.025
    LATERAL_STOP_TOLERANCE_M = 0.003
    ARC_CYCLE_YAW_THRESHOLD_DEG = 3.0

    def __init__(self, node, args):
        super().__init__()
        self.node = node
        self.args = args
        self.pressed = set()
        self.last_displayed_sequence = (-1, -1, -1)
        self.last_map_sequence = (-1, -1)
        self.writer = None
        self.record_path = None
        self.telemetry_session = None
        self.recorded_frames = 0
        self.camera_calibration_dir = None
        self.camera_calibration_samples = []
        self.camera_distance_scale = None
        self.camera_distance_offset = None
        self.camera_yaw_bias = 0.0
        self.centerline_offset_cm = 0.0
        self.lateral_overrun_cm = 0.0
        self.rotation_overrun_deg = 0.0
        self.small_rotation_threshold_deg = 5.0
        self.small_rotation_speed_rad_s = 0.55
        self.near_center_check_distance_cm = 25.0
        self.load_state = "unloaded"
        self.active_calibration_preset = "unloaded"
        self.calibration_presets = {}
        self.rotation_samples = []
        self.rotation_scale = 1.0
        self.rotation_trial_start = None
        self.rotation_trial_started_at = None
        self.rotation_trial_speed = None
        self.rotation_reference = None
        self.distance_samples = []
        self.distance_scale = 1.0
        self.distance_trial_start = None
        self.distance_trial_started_at = None
        self.distance_trial_speed = None
        self.lateral_samples = []
        self.lateral_scale = 1.0
        self.lateral_trial_start = None
        self.drive_reference_path = None
        self.drive_reference_position = None
        self.drive_reference_yaw = None
        self.updating_truth_yaw = False
        self.truth_yaw_source = "manual"
        self.command_forward_m = 0.0
        self.command_lateral_m = 0.0
        self.command_rotation_rad = 0.0
        self.command_forward_abs_m = 0.0
        self.command_lateral_abs_m = 0.0
        self.command_rotation_abs_rad = 0.0
        self.last_command_integral_at = time.monotonic()
        self.mapping_active = False
        self.mapping_started_at = None
        self.mapping_started_stamp = None
        self.mapping_start_frame = None
        self.mapping_start_integrals = None
        self.mapping_start_detection = None
        self.mapping_input_events = []
        self.mapping_pose = None
        self.mapping_odom_start_yaw = None
        self.mapping_odom_start_position = None
        self.arc_plan = None
        self.arc_active = False
        self.last_arc_stop_reason = None
        self.last_arc_stop_was_docking = False
        self.arc_start_position = None
        self.arc_start_yaw = None
        self.arc_started_at = None
        self.arc_waypoint_index = 0
        self.arc_final_approach_active = False
        self.arc_recovery_active = False
        self.arc_recovery_started_at = None
        self.arc_target_world = None
        self.arc_recovery_count = 0
        self.arc_fusion_streak = 0
        self.arc_last_fusion_monotonic = 0.0
        self.arc_last_fused_detection_id = None
        self.arc_replan_streak = 0
        self.arc_pass_count = 0
        self.arc_reobserve_active = False
        self.arc_reobserve_started_at = None
        self.arc_dock_phase = None
        self.arc_phase_started_at = None
        self.arc_phase_streak = 0
        self.arc_phase_last_detection_id = None
        self.arc_dock_lateral_target_m = None
        self.arc_dock_rotation_target_rad = None
        self.arc_dock_forward_target_m = None
        self.arc_dock_motion_anchor_position = None
        self.arc_dock_motion_anchor_yaw = None
        self.arc_dock_lateral_progress_m = 0.0
        self.arc_dock_last_control_at = None
        self.arc_dock_last_lateral_command = 0.0
        self.arc_dock_yaw_correction_count = 0
        self.arc_dock_yaw_verify_samples = []
        self.arc_dock_yaw_error_before_rotation_deg = None
        self.arc_dock_rotation_was_commanded = False
        self.arc_yaw_direction_multiplier = -1.0
        self.arc_yaw_direction_flip_count = 0
        self.arc_near_center_check_done = False
        self.arc_near_recheck_samples = []
        self.arc_auto_pass = 0
        self.arc_auto_max_passes = 1
        self.arc_auto_enabled = False
        self.arc_auto_internal_start = False
        self.arc_auto_replan_due_at = None
        self.arc_auto_last_result = None
        self.arc_cycle_limit = 0
        self.arc_auto_insert_after_verify = True
        # Set only by the headless integration runner.  GUI actions keep the
        # existing behaviour and never raise the fork implicitly.
        self.auto_lift_after_dock = False
        self.last_auto_lift_monotonic = None
        self.terminal_run_mode = "search"
        self.arc_cycle_index = 0
        self.arc_cycle_advance_m = 0.10
        self.arc_cycle_pause_sec = 0.7
        self.stable_detection_frames = 5
        self.arc_cycle_replan_due_at = None
        self.target_search_active = False
        self.target_search_linear_m_s = 0.08
        self.target_search_angular_rad_s = 0.12
        self.search_circle_diameter_m = 1.34
        self.arc_forward_anchor_position = None
        self.arc_forward_anchor_yaw = None
        self.arc_forward_anchor_remaining_m = None
        self.arc_initial_plan = None
        self.last_arc_result = None
        self.arc_correction_started_at = None
        self.arc_correction_start_integrals = None
        self.arc_correction_input_events = []
        self.arc_correction_start_frame = None
        self.arc_correction_start_detection = None
        self.arc_correction_start_odom = None
        self.arc_result_dir = Path(args.output_dir) / "arc_result_samples"
        self.arc_result_sample_count = 0
        arc_samples_path = self.arc_result_dir / "samples.jsonl"
        if arc_samples_path.exists():
            try:
                self.arc_result_sample_count = sum(
                    1 for line in arc_samples_path.open(encoding="utf-8") if line.strip()
                )
            except OSError:
                self.arc_result_sample_count = 0
        self.charge_state_path = Path(args.output_dir) / "battery_charge_state.json"
        self.charge_state = "unknown"
        self.charge_started_at = None
        self.last_charge_ended_at = None
        self.last_charge_evaluation = 0.0
        self.battery_slope_mv_min = None
        self.load_charge_state()
        self.setWindowTitle(f"Vehicle {args.vehicle} Camera Teleop")
        self.resize(1180, 760)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #151515; color: #f2f2f2; }
            QGroupBox { border: 1px solid #f2f2f2; margin-top: 8px; padding: 12px 10px 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #f2f2f2; }
            QPushButton { background: #222222; border: 1px solid #f2f2f2; padding: 7px 10px; }
            QPushButton:pressed { background: #555555; }
            QPushButton:checked { background: #f2f2f2; color: #151515; }
            QComboBox { background: #222222; border: 1px solid #f2f2f2; padding: 5px; }
            QPlainTextEdit { background: #222222; border: 1px solid #f2f2f2; padding: 5px; }
            QSlider::groove:horizontal { height: 3px; background: #777777; }
            QSlider::sub-page:horizontal { background: #f2f2f2; }
            QSlider::handle:horizontal { background: #f2f2f2; width: 12px; margin: -5px 0; }
            QLabel#status { color: #d4d4d4; font-family: monospace; font-size: 11px; }
        """)

        self.video = VideoLabel("Waiting for vehicle camera…", 560, 420)
        self.entity_map_view = EntityMapView()
        self.status = QLabel("READY")
        self.status.setObjectName("status")
        self.battery_label = QLabel("배터리: 수신 대기 | 충전 상태 판정 중")
        self.battery_label.setObjectName("status")

        self.linear, self.linear_value = self.speed_slider(
            args.linear_speed, 0.01, 0.30, 0.01, " m/s"
        )
        self.angular, self.angular_value = self.speed_slider(
            args.angular_speed, 0.05, 2.0, 0.05, " rad/s"
        )
        self.drive_mode = QComboBox()
        self.drive_mode.addItem("Mecanum Omnidirectional", "mecanum")
        self.drive_mode.addItem("Car-like (Ackermann-style)", "car_like")
        self.drive_mode.currentIndexChanged.connect(self.on_drive_mode_changed)

        self.target_left = QComboBox()
        self.target_right = QComboBox()
        for symbol in ("star", "diamond", "spade", "clover", "heart"):
            self.target_left.addItem(symbol, symbol)
            self.target_right.addItem(symbol, symbol)
        self.target_left.setCurrentIndex(self.target_left.findData("diamond"))
        self.target_right.setCurrentIndex(self.target_right.findData("spade"))
        self.target_left.currentIndexChanged.connect(self.on_target_changed)
        self.target_right.currentIndexChanged.connect(self.on_target_changed)
        self.target_left_group = QButtonGroup(self)
        self.target_right_group = QButtonGroup(self)
        self.target_left_group.setExclusive(True)
        self.target_right_group.setExclusive(True)
        target_selector_row = QHBoxLayout()
        target_selector_row.setContentsMargins(0, 0, 0, 0)
        target_selector_row.setSpacing(3)
        symbol_labels = {
            "star": "★", "diamond": "◆", "spade": "♠", "clover": "♣", "heart": "♥",
        }
        target_selector_row.addWidget(QLabel("좌"))
        for symbol, label in symbol_labels.items():
            left_button = QPushButton(label)
            left_button.setCheckable(True)
            left_button.setChecked(symbol == "diamond")
            left_button.setToolTip(symbol)
            left_button.setFixedWidth(38)
            left_button.clicked.connect(
                lambda _checked, value=symbol: self.select_target_symbol("left", value)
            )
            self.target_left_group.addButton(left_button)
            target_selector_row.addWidget(left_button)
        target_selector_row.addSpacing(8)
        target_selector_row.addWidget(QLabel("우"))
        for symbol, label in symbol_labels.items():
            right_button = QPushButton(label)
            right_button.setCheckable(True)
            right_button.setChecked(symbol == "spade")
            right_button.setToolTip(symbol)
            right_button.setFixedWidth(38)
            right_button.clicked.connect(
                lambda _checked, value=symbol: self.select_target_symbol("right", value)
            )
            self.target_right_group.addButton(right_button)
            target_selector_row.addWidget(right_button)
        target_selector_widget = QWidget()
        target_selector_widget.setLayout(target_selector_row)

        controls = QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(8)
        controls.addWidget(QLabel("Drive mode"), 0, 0)
        controls.addWidget(self.drive_mode, 0, 1, 1, 3)
        controls.addWidget(QLabel("Forward speed"), 1, 0)
        controls.addWidget(self.linear, 1, 1, 1, 2)
        controls.addWidget(self.linear_value, 1, 3)
        controls.addWidget(QLabel("Steering speed"), 2, 0)
        controls.addWidget(self.angular, 2, 1, 1, 2)
        controls.addWidget(self.angular_value, 2, 3)
        controls.addWidget(QLabel("목표 상단"), 3, 0)
        controls.addWidget(target_selector_widget, 3, 1, 1, 3)

        self.record = QPushButton("●  Start recording")
        self.record.setCheckable(True)
        self.record.toggled.connect(self.toggle_recording)
        self.record_label = QLabel("Not recording")
        record_row = QHBoxLayout()
        record_row.addWidget(self.record)
        record_row.addWidget(self.record_label, 1)

        self.truth_forward = QDoubleSpinBox()
        self.truth_forward.setRange(0.0, 1000.0)
        self.truth_forward.setDecimals(1)
        self.truth_forward.setValue(50.0)
        self.truth_forward.setSuffix(" cm")
        self.truth_lateral = QDoubleSpinBox()
        self.truth_lateral.setRange(-1000.0, 1000.0)
        self.truth_lateral.setDecimals(1)
        self.truth_lateral.setSuffix(" cm")
        self.truth_yaw = QDoubleSpinBox()
        self.truth_yaw.setRange(-180.0, 180.0)
        self.truth_yaw.setDecimals(1)
        self.truth_yaw.setSuffix("°")
        self.truth_yaw.valueChanged.connect(self.on_truth_yaw_edited)
        truth_layout = QGridLayout()
        truth_layout.addWidget(QLabel("전후 거리"), 0, 0)
        truth_layout.addWidget(self.truth_forward, 0, 1)
        truth_layout.addWidget(QLabel("좌우 거리 (오른쪽 +)"), 0, 2)
        truth_layout.addWidget(self.truth_lateral, 0, 3)
        truth_layout.addWidget(QLabel("물체 정면 상대각"), 1, 0)
        truth_layout.addWidget(self.truth_yaw, 1, 1)

        self.auto_truth_yaw = QCheckBox("보정 회전각 자동 입력")
        self.auto_truth_yaw.setChecked(True)
        self.rotation_zero = QPushButton("현재 물체 정면 = 0°")
        self.rotation_zero.clicked.connect(self.set_rotation_reference)
        truth_layout.addWidget(self.auto_truth_yaw, 1, 2)
        truth_layout.addWidget(self.rotation_zero, 1, 3)

        self.rotation_actual = QDoubleSpinBox()
        self.rotation_actual.setRange(1.0, 720.0)
        self.rotation_actual.setDecimals(1)
        self.rotation_actual.setValue(360.0)
        self.rotation_actual.setSuffix("°")
        self.rotation_trial = QPushButton("회전 측정 시작")
        self.rotation_trial.clicked.connect(self.start_rotation_trial)
        self.rotation_save = QPushButton("실제각 샘플 추가")
        self.rotation_save.clicked.connect(self.save_rotation_trial)
        self.rotation_label = QLabel("회전 보정 없음 | Q=반시계(+), E=시계(-)")
        rotation_layout = QGridLayout()
        rotation_layout.addWidget(QLabel("실제 회전각"), 0, 0)
        rotation_layout.addWidget(self.rotation_actual, 0, 1)
        rotation_layout.addWidget(self.rotation_trial, 0, 2)
        rotation_layout.addWidget(self.rotation_save, 0, 3)
        rotation_layout.addWidget(self.rotation_label, 1, 0, 1, 4)

        self.distance_actual = QDoubleSpinBox()
        self.distance_actual.setRange(1.0, 2000.0)
        self.distance_actual.setDecimals(1)
        self.distance_actual.setValue(50.0)
        self.distance_actual.setSuffix(" cm")
        self.distance_trial = QPushButton("주행 측정 시작")
        self.distance_trial.clicked.connect(self.start_distance_trial)
        self.distance_save = QPushButton("실제거리 샘플 추가")
        self.distance_save.clicked.connect(self.save_distance_trial)
        self.distance_zero = QPushButton("현재 주행거리 = 0")
        self.distance_zero.clicked.connect(self.set_drive_reference)
        self.distance_label = QLabel("주행 보정 없음 | W/S 전후 주행 기준")
        rotation_layout.addWidget(QLabel("실제 주행거리"), 2, 0)
        rotation_layout.addWidget(self.distance_actual, 2, 1)
        rotation_layout.addWidget(self.distance_trial, 2, 2)
        rotation_layout.addWidget(self.distance_save, 2, 3)
        rotation_layout.addWidget(self.distance_zero, 3, 0, 1, 2)
        rotation_layout.addWidget(self.distance_label, 3, 2, 1, 2)

        self.lateral_actual = QDoubleSpinBox()
        self.lateral_actual.setRange(1.0, 2000.0)
        self.lateral_actual.setDecimals(1)
        self.lateral_actual.setValue(50.0)
        self.lateral_actual.setSuffix(" cm")
        self.lateral_trial = QPushButton("좌우 측정 시작")
        self.lateral_trial.clicked.connect(self.start_lateral_trial)
        self.lateral_save = QPushButton("실제거리 샘플 추가")
        self.lateral_save.clicked.connect(self.save_lateral_trial)
        self.lateral_label = QLabel("좌우 보정 없음 | A/D 횡이동 기준")
        rotation_layout.addWidget(QLabel("실제 좌우거리"), 4, 0)
        rotation_layout.addWidget(self.lateral_actual, 4, 1)
        rotation_layout.addWidget(self.lateral_trial, 4, 2)
        rotation_layout.addWidget(self.lateral_save, 4, 3)
        rotation_layout.addWidget(self.lateral_label, 5, 0, 1, 4)

        self.mapping_start = QPushButton("1. 중앙 목표 매핑 시작")
        self.mapping_start.clicked.connect(self.start_mapping)
        self.mapping_save = QPushButton("2. 전진 완료 후 매핑 저장")
        self.mapping_save.clicked.connect(self.save_mapping)
        self.mapping_save.setEnabled(False)
        self.mapping_label = QLabel("대기: 목표물을 화면 중앙에 놓고 시작")
        mapping_layout = QVBoxLayout()
        mapping_layout.addWidget(QLabel("A/D 횡이동 → Q/E 회전 → W/S 전후이동"))
        mapping_buttons = QHBoxLayout()
        mapping_buttons.addWidget(self.mapping_start)
        mapping_buttons.addWidget(self.mapping_save)
        mapping_layout.addLayout(mapping_buttons)
        mapping_layout.addWidget(self.mapping_label)

        self.arc_standoff = QDoubleSpinBox()
        self.arc_standoff.setRange(3.0, 100.0)
        self.arc_standoff.setValue(8.0)
        self.arc_standoff.setDecimals(1)
        self.arc_standoff.setSuffix(" cm")
        self.arc_insertion_distance = QDoubleSpinBox()
        self.arc_insertion_distance.setRange(0.0, 50.0)
        self.arc_insertion_distance.setValue(12.0)
        self.arc_insertion_distance.setDecimals(1)
        self.arc_insertion_distance.setSuffix(" cm")
        self.approach_mode = QComboBox()
        self.approach_mode.addItem("ARC", "arc")
        self.approach_mode.addItem("\ud6a1이동 → 회전", "visual_alternating")
        self.approach_mode.setCurrentIndex(1)
        self.arc_plan_button = QPushButton("주행 계산")
        self.arc_plan_button.clicked.connect(self.plan_arc_approach)
        self.arc_execute_button = QPushButton("주행 실행")
        self.arc_execute_button.clicked.connect(self.start_arc_approach)
        self.arc_execute_button.setEnabled(False)
        self.run_mode = QComboBox()
        self.run_mode.addItem("원형 탐색 → 자동주행", "search")
        self.run_mode.addItem("무제한 자동주행 → 자동 삽입", "auto")
        self.run_mode.addItem("단일 정렬 (삽입 전 정지)", "single")
        self.run_mode.addItem("3회 정렬 (삽입 전 정지)", "cycle3")
        self.run_mode.setCurrentIndex(self.run_mode.findData("search"))
        self.run_mode_button = QPushButton("선택 모드 실행")
        self.run_mode_button.clicked.connect(self.run_selected_mode)
        self.arc_cancel_button = QPushButton("취소")
        self.arc_cancel_button.clicked.connect(
            lambda: self.cancel_arc_approach("사용자 취소")
        )
        self.arrival_button = QPushButton("1.2 arrival 토픽 발행")
        self.arrival_button.clicked.connect(self.publish_arrival_trigger)
        self.auto_dock_stop_button = QPushButton("1.2 stop 토픽 발행")
        self.auto_dock_stop_button.clicked.connect(self.publish_auto_dock_stop)
        self.auto_dock_status_label = QLabel("1.2 상태: 수신 대기")
        self.arc_label = QLabel("대기: 목표 검출 후 경로 계산")
        self.arc_forward_error = QDoubleSpinBox()
        self.arc_forward_error.setRange(-200.0, 200.0)
        self.arc_forward_error.setDecimals(1)
        self.arc_forward_error.setSuffix(" cm")
        self.arc_lateral_error = QDoubleSpinBox()
        self.arc_lateral_error.setRange(-200.0, 200.0)
        self.arc_lateral_error.setDecimals(1)
        self.arc_lateral_error.setSuffix(" cm")
        self.arc_sample_save = QPushButton("ARC 결과 샘플 저장")
        self.arc_sample_save.clicked.connect(self.save_arc_result_sample)
        self.arc_sample_label = QLabel(
            f"샘플 {self.arc_result_sample_count}개 | ARC 완료 후 실제 오차 입력"
        )
        arc_layout = QGridLayout()
        arc_layout.addWidget(QLabel("정렬 완료 거리"), 0, 0)
        arc_layout.addWidget(self.arc_standoff, 0, 1)
        arc_layout.addWidget(self.arc_plan_button, 0, 2)
        arc_layout.addWidget(self.arc_execute_button, 0, 3)
        arc_layout.addWidget(self.arc_cancel_button, 0, 4)
        arc_layout.addWidget(QLabel("포크 추가 삽입"), 1, 0)
        arc_layout.addWidget(self.arc_insertion_distance, 1, 1)
        arc_layout.addWidget(QLabel("주행 방식"), 1, 2)
        arc_layout.addWidget(self.approach_mode, 1, 3, 1, 2)
        arc_layout.addWidget(QLabel("자동 모드"), 2, 0)
        arc_layout.addWidget(self.run_mode, 2, 1, 1, 3)
        arc_layout.addWidget(self.run_mode_button, 2, 4)
        arc_layout.addWidget(self.arrival_button, 3, 0, 1, 3)
        arc_layout.addWidget(self.auto_dock_stop_button, 3, 3, 1, 2)
        arc_layout.addWidget(self.auto_dock_status_label, 4, 0, 1, 5)
        arc_layout.addWidget(self.arc_label, 5, 0, 1, 5)
        arc_layout.addWidget(QLabel("전후차 (+덜 감)"), 6, 0)
        arc_layout.addWidget(self.arc_forward_error, 6, 1)
        arc_layout.addWidget(QLabel("좌우차 (+왼쪽)"), 6, 2)
        arc_layout.addWidget(self.arc_lateral_error, 6, 3)
        arc_layout.addWidget(self.arc_sample_save, 6, 4)
        arc_layout.addWidget(self.arc_sample_label, 7, 0, 1, 5)

        self.memo = QPlainTextEdit()
        self.memo.setPlaceholderText("캡처 메모 (이미지와 별도 JSON으로 저장)")
        self.memo.setMaximumHeight(70)
        self.capture = QPushButton("현재 차량 화면 캡처")
        self.capture.clicked.connect(self.capture_snapshot)
        self.capture_label = QLabel("캡처 대기")
        capture_row = QHBoxLayout()
        capture_row.addWidget(self.capture)
        capture_row.addWidget(self.capture_label, 1)

        self.camera_calibration_distance = QDoubleSpinBox()
        self.camera_calibration_distance.setRange(10.0, 1000.0)
        self.camera_calibration_distance.setValue(50.0)
        self.camera_calibration_distance.setSuffix(" cm")
        self.camera_calibration_distance.setDecimals(1)
        self.camera_calibration_capture = QPushButton("샘플 1 저장")
        self.camera_calibration_capture.clicked.connect(self.save_camera_calibration_sample)
        self.camera_calibration_reset = QPushButton("두 샘플 초기화")
        self.camera_calibration_reset.clicked.connect(self.reset_camera_calibration)
        self.camera_calibration_label = QLabel(
            "0/2 | 목표물을 화면 중앙·정면에 두고 포크 끝 기준 거리를 입력"
        )
        camera_calibration_layout = QGridLayout()
        camera_calibration_layout.addWidget(QLabel("포크 끝 → 목표물 정면"), 0, 0)
        camera_calibration_layout.addWidget(self.camera_calibration_distance, 0, 1)
        camera_calibration_layout.addWidget(self.camera_calibration_capture, 0, 2)
        camera_calibration_layout.addWidget(self.camera_calibration_reset, 0, 3)
        camera_calibration_layout.addWidget(self.camera_calibration_label, 1, 0, 1, 4)

        self.preset_combo = QComboBox()
        self.preset_combo.setEditable(True)
        self.preset_combo.setInsertPolicy(QComboBox.NoInsert)
        self.preset_combo.setCurrentText("unloaded")
        self.preset_load = QPushButton("불러오기")
        self.preset_load.clicked.connect(self.load_selected_calibration_preset)
        self.preset_save = QPushButton("현재값 저장")
        self.preset_save.clicked.connect(self.save_current_calibration_preset)
        self.preset_label = QLabel("현재 preset: unloaded")
        preset_layout = QGridLayout()
        preset_layout.addWidget(QLabel("Preset 이름"), 0, 0)
        preset_layout.addWidget(self.preset_combo, 0, 1)
        preset_layout.addWidget(self.preset_load, 0, 2)
        preset_layout.addWidget(self.preset_save, 0, 3)
        preset_layout.addWidget(self.preset_label, 1, 0, 1, 4)

        vehicle_column = QVBoxLayout()
        vehicle_column.addWidget(self.video, 1)
        vehicle_column.addWidget(self.entity_map_view)
        vehicle_panel = QGroupBox("차량 카메라 / 태그 엔티티 지도")
        vehicle_panel.setLayout(vehicle_column)

        layout = QVBoxLayout()
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)
        main_row = QHBoxLayout()
        main_row.setSpacing(16)
        main_row.addWidget(vehicle_panel, 1, Qt.AlignTop)
        control_panel = QGroupBox("설정 및 캡처")
        control_layout = QVBoxLayout()
        mapping_panel = QGroupBox("중앙 목표 매핑")
        mapping_panel.setLayout(mapping_layout)
        control_layout.addWidget(mapping_panel)
        arc_panel = QGroupBox("ARC 정면 진입")
        arc_panel.setLayout(arc_layout)
        control_layout.addWidget(arc_panel)
        camera_calibration_panel = QGroupBox("카메라 2점 Calibration")
        camera_calibration_panel.setLayout(camera_calibration_layout)
        preset_panel = QGroupBox("Motion Calibration Preset")
        preset_panel.setLayout(preset_layout)
        rotation_panel = QGroupBox("3축 이동 Calibration")
        rotation_panel.setLayout(rotation_layout)
        self.calibration_window = QDialog(self)
        self.calibration_window.setWindowTitle("Vehicle Calibration")
        self.calibration_window.resize(720, 520)
        calibration_window_layout = QVBoxLayout()
        calibration_window_layout.addWidget(preset_panel)
        calibration_window_layout.addWidget(camera_calibration_panel)
        calibration_window_layout.addWidget(rotation_panel)
        self.calibration_window.setLayout(calibration_window_layout)
        self.open_calibration_button = QPushButton("Calibration 창 열기")
        self.open_calibration_button.clicked.connect(self.open_calibration_window)
        control_layout.addWidget(self.open_calibration_button)
        self.open_pose_config_button = QPushButton("공통 설정 JSON 열기")
        self.open_pose_config_button.clicked.connect(self.open_pose_config_editor)
        control_layout.addWidget(self.open_pose_config_button)
        self.log_filter = QComboBox()
        for label, value in (
            ("전체", "all"), ("녹화", "recording"), ("캡처", "capture"),
            ("중앙매핑", "mapping"), ("ARC 샘플", "arc"),
            ("카메라 보정", "camera_calibration"),
            ("3축 이동 보정", "motion_calibration"),
        ):
            self.log_filter.addItem(label, value)
        self.log_filter.currentIndexChanged.connect(self.refresh_log_records)
        self.log_list = QListWidget()
        self.log_list.currentRowChanged.connect(self.show_selected_log_record)
        self.log_preview = QLabel("기록을 선택하세요")
        self.log_preview.setAlignment(Qt.AlignCenter)
        self.log_preview.setMinimumSize(520, 320)
        self.log_details = QPlainTextEdit()
        self.log_details.setReadOnly(True)
        self.log_details.setMaximumHeight(220)
        self.log_refresh_button = QPushButton("새로고침")
        self.log_refresh_button.clicked.connect(self.refresh_log_records)
        self.log_discard_button = QPushButton("노이즈로 폐기")
        self.log_discard_button.clicked.connect(self.discard_selected_log_record)
        log_top = QHBoxLayout()
        log_top.addWidget(QLabel("종류"))
        log_top.addWidget(self.log_filter, 1)
        log_top.addWidget(self.log_refresh_button)
        log_top.addWidget(self.log_discard_button)
        log_body = QHBoxLayout()
        log_body.addWidget(self.log_list, 1)
        log_visual = QVBoxLayout()
        log_visual.addWidget(self.log_preview, 1)
        log_visual.addWidget(self.log_details)
        log_body.addLayout(log_visual, 2)
        self.log_window = QDialog(self)
        self.log_window.setWindowTitle("기록 검토 / 노이즈 폐기")
        self.log_window.resize(980, 700)
        log_window_layout = QVBoxLayout()
        log_window_layout.addLayout(log_top)
        log_window_layout.addLayout(log_body, 1)
        self.log_window.setLayout(log_window_layout)
        self.open_log_button = QPushButton("기록 검토 / 폐기 창 열기")
        self.open_log_button.clicked.connect(self.open_log_window)
        control_layout.addWidget(self.open_log_button)
        control_layout.addLayout(controls)
        control_layout.addWidget(QLabel(
            "주행: W A S D / 제자리 회전: Q E / 리프트: 방향키 / 비상정지: SPACE"
        ))
        control_layout.addWidget(self.status)
        control_layout.addWidget(self.battery_label)
        control_layout.addLayout(record_row)
        control_layout.addWidget(self.memo)
        control_layout.addLayout(capture_row)
        control_panel.setLayout(control_layout)
        main_row.addWidget(control_panel, 1, Qt.AlignTop)
        layout.addLayout(main_row, 1)
        root = QWidget()
        root.setLayout(layout)
        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        # X11 forwarding becomes unresponsive when three full-resolution
        # streams are repainted at 20 FPS.  Teleop commands remain responsive
        # at 10 FPS while substantially reducing remote paint traffic.
        self.timer.start(100)
        self.on_target_changed()
        self.load_rotation_calibration()

    def on_target_changed(self, _index=None):
        self.arc_plan = None
        if hasattr(self, "arc_execute_button"):
            self.arc_execute_button.setEnabled(False)
        self.node.select_target(
            self.target_left.currentData(), self.target_right.currentData()
        )

    def select_target_symbol(self, side, symbol):
        combo = self.target_left if side == "left" else self.target_right
        index = combo.findData(symbol)
        if index >= 0:
            combo.setCurrentIndex(index)

    def open_calibration_window(self):
        self.calibration_window.show()
        self.calibration_window.raise_()
        self.calibration_window.activateWindow()

    def open_pose_config_editor(self):
        path = self.rotation_calibration_path()
        try:
            content = path.read_text(encoding="utf-8") if path.exists() else "{}\n"
        except OSError as exc:
            QMessageBox.warning(self, "설정 JSON", f"파일을 열 수 없습니다: {exc}")
            return
        dialog = QDialog(self)
        dialog.setWindowTitle(f"공통 설정 JSON — {path}")
        dialog.resize(760, 620)
        editor = QPlainTextEdit()
        editor.setPlainText(content)
        status = QLabel("수정 후 저장하면 GUI와 control_ui가 같은 값을 사용합니다.")
        save_button = QPushButton("저장 후 다시 읽기")
        close_button = QPushButton("닫기")

        def save_config():
            try:
                data = json.loads(editor.toPlainText())
                if not isinstance(data, dict):
                    raise ValueError("최상위 값은 JSON 객체여야 합니다")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(
                    json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                self.load_rotation_calibration()
                status.setText("저장 완료: 현재 GUI 설정을 다시 읽었습니다.")
            except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
                status.setText(f"저장 실패: {exc}")

        save_button.clicked.connect(save_config)
        close_button.clicked.connect(dialog.accept)
        buttons = QHBoxLayout()
        buttons.addWidget(save_button)
        buttons.addWidget(close_button)
        layout = QVBoxLayout()
        layout.addWidget(editor, 1)
        layout.addWidget(status)
        layout.addLayout(buttons)
        dialog.setLayout(layout)
        dialog.exec_()

    def open_log_window(self):
        self.refresh_log_records()
        self.log_window.show()
        self.log_window.raise_()
        self.log_window.activateWindow()

    def collect_log_records(self):
        root = Path(self.args.output_dir)
        records = []

        def add_record(kind, label, files, metadata=None):
            files = list(dict.fromkeys(path for path in files if path.exists()))
            if not files:
                return
            images = [path for path in files if path.suffix.lower() in {".jpg", ".jpeg", ".png"}]
            videos = [path for path in files if path.suffix.lower() in {".mp4", ".avi"}]
            records.append({
                "kind": kind,
                "label": label,
                "files": files,
                "metadata": metadata if metadata is not None and metadata.exists() else None,
                "preview_files": images or videos,
                "mtime": max(path.stat().st_mtime for path in files),
            })

        recording_stems = {
            path.stem for path in root.glob("teleop_*.*")
            if path.suffix.lower() in {".mp4", ".jsonl"}
        }
        for stem in recording_stems:
            files = [root / f"{stem}.mp4", root / f"{stem}.jsonl"]
            add_record("recording", f"녹화 | {stem}", files, root / f"{stem}.jsonl")

        capture_dir = root / "captures"
        capture_stems = {path.stem for path in capture_dir.glob("capture_*.*")}
        for stem in capture_stems:
            files = [capture_dir / f"{stem}.json", capture_dir / f"{stem}.jpg"]
            add_record("capture", f"캡처 | {stem}", files, capture_dir / f"{stem}.json")

        mapping_dir = root / "mappings"
        mapping_stems = set()
        for path in mapping_dir.glob("mapping_*.*"):
            stem = path.stem
            for suffix in ("_initial", "_final"):
                if stem.endswith(suffix):
                    stem = stem[:-len(suffix)]
            mapping_stems.add(stem)
        for stem in mapping_stems:
            files = [
                mapping_dir / f"{stem}.json",
                mapping_dir / f"{stem}_initial.jpg",
                mapping_dir / f"{stem}_final.jpg",
            ]
            add_record("mapping", f"중앙매핑 | {stem}", files, mapping_dir / f"{stem}.json")

        arc_dir = root / "arc_result_samples"
        for metadata in arc_dir.glob("arc_sample_*.json"):
            stem = metadata.stem
            files = [
                metadata,
                arc_dir / f"{stem}_arc_end.jpg",
                arc_dir / f"{stem}_corrected.jpg",
            ]
            add_record("arc", f"ARC | {stem}", files, metadata)

        camera_root = root / "camera_calibration"
        for session_dir in camera_root.glob("camera_calibration_*"):
            if session_dir.is_dir():
                files = [path for path in session_dir.rglob("*") if path.is_file()]
                add_record(
                    "camera_calibration", f"카메라 보정 | {session_dir.name}",
                    files, session_dir / "calibration.json",
                )
        config_mtime = (
            self.rotation_calibration_path().stat().st_mtime
            if self.rotation_calibration_path().exists() else time.time()
        )
        for axis, label, samples in (
            ("rotation", "회전", self.rotation_samples),
            ("forward", "전후", self.distance_samples),
            ("lateral", "좌우", self.lateral_samples),
        ):
            for index, sample in enumerate(samples):
                records.append({
                    "kind": "motion_calibration",
                    "label": f"3축 보정 {label} #{index + 1}",
                    "files": [],
                    "metadata": None,
                    "preview_files": [],
                    "mtime": config_mtime + index * 0.001,
                    "calibration_axis": axis,
                    "calibration_index": index,
                    "inline_metadata": sample,
                })
        return sorted(records, key=lambda record: record["mtime"], reverse=True)

    def refresh_log_records(self, _index=None):
        if not hasattr(self, "log_list"):
            return
        selected_filter = self.log_filter.currentData()
        self.log_records = [
            record for record in self.collect_log_records()
            if selected_filter == "all" or record["kind"] == selected_filter
        ]
        self.log_list.clear()
        for record in self.log_records:
            stamp = time.strftime("%m-%d %H:%M:%S", time.localtime(record["mtime"]))
            self.log_list.addItem(f"{stamp}  {record['label']}")
        self.log_preview.setText(
            f"기록 {len(self.log_records)}개" if self.log_records else "기록 없음"
        )
        self.log_preview.setPixmap(QPixmap())
        self.log_details.clear()

    @staticmethod
    def compact_log_json(value, depth=0):
        if depth >= 5:
            if isinstance(value, (dict, list)):
                return f"<{type(value).__name__} {len(value)}개>"
            return value
        if isinstance(value, dict):
            return {
                key: TeleopWindow.compact_log_json(item, depth + 1)
                for key, item in value.items()
            }
        if isinstance(value, list):
            if len(value) > 12:
                return {
                    "count": len(value),
                    "first": [
                        TeleopWindow.compact_log_json(item, depth + 1)
                        for item in value[:3]
                    ],
                }
            return [TeleopWindow.compact_log_json(item, depth + 1) for item in value]
        return value

    def log_record_summary(self, record):
        if "inline_metadata" in record:
            return self.compact_log_json(record["inline_metadata"])
        metadata = record.get("metadata")
        if metadata is None:
            return {"files": [path.name for path in record["files"]]}
        try:
            if metadata.suffix == ".jsonl":
                counts = {}
                first_time = last_time = None
                line_count = 0
                with metadata.open(encoding="utf-8") as source:
                    for line in source:
                        if not line.strip():
                            continue
                        line_count += 1
                        try:
                            item = json.loads(line)
                        except ValueError:
                            continue
                        event_type = item.get("type", "unknown")
                        counts[event_type] = counts.get(event_type, 0) + 1
                        stamp = item.get("monotonic")
                        if stamp is not None:
                            first_time = stamp if first_time is None else first_time
                            last_time = stamp
                return {
                    "files": [path.name for path in record["files"]],
                    "line_count": line_count,
                    "duration_sec": (
                        None if first_time is None or last_time is None
                        else last_time - first_time
                    ),
                    "event_counts": counts,
                }
            return self.compact_log_json(
                json.loads(metadata.read_text(encoding="utf-8"))
            )
        except (OSError, TypeError, ValueError) as exc:
            return {"files": [path.name for path in record["files"]], "read_error": str(exc)}

    def show_selected_log_record(self, row):
        if row < 0 or row >= len(getattr(self, "log_records", [])):
            return
        record = self.log_records[row]
        frames = []
        for path in record["preview_files"][:2]:
            if path.suffix.lower() in {".mp4", ".avi"}:
                capture = cv2.VideoCapture(str(path))
                frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                if frame_count > 1:
                    capture.set(cv2.CAP_PROP_POS_FRAMES, frame_count // 2)
                ok, frame = capture.read()
                capture.release()
                if ok:
                    frames.append(frame)
            else:
                frame = cv2.imread(str(path))
                if frame is not None:
                    frames.append(frame)
        if frames:
            target_height = min(frame.shape[0] for frame in frames)
            resized = [
                cv2.resize(
                    frame,
                    (max(1, round(frame.shape[1] * target_height / frame.shape[0])), target_height),
                    interpolation=cv2.INTER_AREA,
                )
                for frame in frames
            ]
            combined = np.hstack(resized)
            rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)
            image = QImage(
                rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888
            ).copy()
            pixmap = QPixmap.fromImage(image).scaled(
                self.log_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.log_preview.setPixmap(pixmap)
        else:
            self.log_preview.setPixmap(QPixmap())
            self.log_preview.setText("미리보기 없음")
        self.log_details.setPlainText(
            json.dumps(self.log_record_summary(record), ensure_ascii=False, indent=2)
        )

    def discard_selected_log_record(self):
        row = self.log_list.currentRow()
        if row < 0 or row >= len(getattr(self, "log_records", [])):
            self.log_details.setPlainText("폐기할 기록을 선택하세요")
            return
        record = self.log_records[row]
        if (
            self.record.isChecked() and self.record_path is not None
            and Path(self.record_path) in record["files"]
        ):
            self.log_details.setPlainText("녹화 중인 항목은 먼저 녹화를 종료하세요")
            return
        answer = QMessageBox.question(
            self.log_window, "노이즈 기록 폐기",
            f"{record['label']}\n"
            + (
                "이 보정 샘플을 preset에서 빼고 trash에 보관할까?"
                if record["kind"] == "motion_calibration"
                else f"연결된 파일 {len(record['files'])}개를 trash로 옮길까?"
            ),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        trash_root = Path(self.args.output_dir) / ".trash" / "log_records"
        record_stem = (
            record["files"][0].stem if record["files"]
            else f"{record.get('calibration_axis', 'sample')}_{record.get('calibration_index', 0)}"
        )
        trash_dir = trash_root / (
            time.strftime("%Y%m%d_%H%M%S") + "_" + record["kind"] + "_" +
            record_stem
        )
        trash_dir.mkdir(parents=True, exist_ok=False)
        if record["kind"] == "motion_calibration":
            axis = record["calibration_axis"]
            samples_by_axis = {
                "rotation": self.rotation_samples,
                "forward": self.distance_samples,
                "lateral": self.lateral_samples,
            }
            samples = samples_by_axis[axis]
            index = record["calibration_index"]
            if index >= len(samples):
                self.log_details.setPlainText("샘플 번호가 바뀌었어. 새로고침 후 다시 시도해줘")
                return
            removed_sample = samples.pop(index)
            (trash_dir / "removed_calibration_sample.json").write_text(
                json.dumps({
                    "preset": self.active_calibration_preset,
                    "axis": axis,
                    "sample": removed_sample,
                }, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            remaining_scale = (
                sum(float(sample["scale"]) for sample in samples) / len(samples)
                if samples else 1.0
            )
            if axis == "rotation":
                self.rotation_scale = remaining_scale
                self.refresh_rotation_label("노이즈 샘플 폐기")
            elif axis == "forward":
                self.distance_scale = remaining_scale
                self.refresh_distance_label("노이즈 샘플 폐기")
            else:
                self.lateral_scale = remaining_scale
                self.refresh_lateral_label("노이즈 샘플 폐기")
            self.persist_rotation_calibration()
            self.refresh_log_records()
            self.log_details.setPlainText(f"trash에 보관하고 preset에서 제거함: {trash_dir}")
            return
        removed_arc_sample = None
        if record["kind"] == "arc" and record.get("metadata") is not None:
            try:
                removed_arc_sample = json.loads(
                    record["metadata"].read_text(encoding="utf-8")
                )
            except (OSError, ValueError):
                removed_arc_sample = None
        for path in record["files"]:
            path.replace(trash_dir / path.name)
        if removed_arc_sample is not None:
            aggregate = Path(self.args.output_dir) / "arc_result_samples" / "samples.jsonl"
            if aggregate.exists():
                kept, removed = [], []
                target_image = removed_arc_sample.get("corrected_image")
                for line in aggregate.read_text(encoding="utf-8").splitlines():
                    try:
                        item = json.loads(line)
                    except ValueError:
                        kept.append(line)
                        continue
                    (removed if item.get("corrected_image") == target_image else kept).append(line)
                if removed:
                    (trash_dir / "removed_samples.jsonl").write_text(
                        "\n".join(removed) + "\n", encoding="utf-8"
                    )
                    aggregate.write_text(
                        ("\n".join(kept) + "\n") if kept else "", encoding="utf-8"
                    )
        self.arc_result_sample_count = len(list(
            (Path(self.args.output_dir) / "arc_result_samples").glob("arc_sample_*.json")
        ))
        self.refresh_log_records()
        self.log_details.setPlainText(f"trash로 옮겨짐: {trash_dir}")

    def load_charge_state(self):
        if not self.charge_state_path.exists():
            return
        try:
            data = json.loads(self.charge_state_path.read_text(encoding="utf-8"))
            state = str(data.get("charge_state", "unknown"))
            self.charge_state = state if state in {"unknown", "charging", "not_charging"} else "unknown"
            self.charge_started_at = data.get("charge_started_at")
            self.last_charge_ended_at = data.get("last_charge_ended_at")
        except (OSError, TypeError, ValueError):
            return

    def persist_charge_state(self):
        self.charge_state_path.parent.mkdir(parents=True, exist_ok=True)
        self.charge_state_path.write_text(
            json.dumps({
                "charge_state": self.charge_state,
                "charge_started_at": self.charge_started_at,
                "last_charge_ended_at": self.last_charge_ended_at,
                "updated_at": time.time(),
                "method": "estimated from battery voltage trend while motors are stopped",
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def format_elapsed(seconds):
        seconds = max(0, int(seconds))
        days, remainder = divmod(seconds, 86400)
        hours, remainder = divmod(remainder, 3600)
        minutes, _seconds = divmod(remainder, 60)
        if days:
            return f"{days}일 {hours}시간"
        if hours:
            return f"{hours}시간 {minutes}분"
        return f"{minutes}분"

    def battery_snapshot(self):
        return {
            "voltage_mv": self.node.battery_voltage_mv,
            "charge_state": self.charge_state,
            "charge_state_is_estimated": True,
            "voltage_slope_mv_per_min": self.battery_slope_mv_min,
            "charge_started_at_unix": self.charge_started_at,
            "last_charge_ended_at_unix": self.last_charge_ended_at,
        }

    def update_battery_status(self):
        now = time.monotonic()
        if now - self.last_charge_evaluation < 1.0:
            return
        self.last_charge_evaluation = now
        voltage = self.node.battery_voltage_mv
        if voltage is None or now - self.node.last_battery_monotonic > 5.0:
            self.battery_label.setText("배터리: 수신 없음 | 충전 상태 알 수 없음")
            return
        samples = [sample for sample in self.node.battery_samples if sample[0] >= now - 180.0]
        enough = len(samples) >= 20 and samples[-1][0] - samples[0][0] >= 60.0
        if enough:
            origin = samples[0][0]
            times = [sample[0] - origin for sample in samples]
            values = [float(sample[1]) for sample in samples]
            mean_time = sum(times) / len(times)
            mean_value = sum(values) / len(values)
            denominator = sum((value - mean_time) ** 2 for value in times)
            slope = 0.0 if denominator <= 1e-9 else (
                sum(
                    (sample_time - mean_time) * (sample_value - mean_value)
                    for sample_time, sample_value in zip(times, values)
                ) / denominator * 60.0
            )
            self.battery_slope_mv_min = slope
            stationary = now - self.node.last_motor_active_monotonic >= 20.0
            next_state = self.charge_state
            if stationary and slope >= 8.0:
                next_state = "charging"
            elif not stationary or slope <= 2.0:
                next_state = "not_charging"
            if next_state != self.charge_state:
                wall_time = time.time()
                if next_state == "charging":
                    self.charge_started_at = wall_time
                elif self.charge_state == "charging":
                    self.last_charge_ended_at = wall_time
                    self.charge_started_at = None
                self.charge_state = next_state
                try:
                    self.persist_charge_state()
                except OSError:
                    pass
        voltage_text = f"{voltage / 1000.0:.2f} V"
        slope_text = (
            "판정 중"
            if self.battery_slope_mv_min is None
            else f"{self.battery_slope_mv_min:+.1f} mV/분"
        )
        if self.charge_state == "charging":
            elapsed = ""
            if self.charge_started_at is not None:
                elapsed = f" | 충전 시작 후 {self.format_elapsed(time.time() - self.charge_started_at)}"
            state_text = f"충전 중 추정{elapsed}"
        else:
            if self.last_charge_ended_at is None:
                last_text = "마지막 충전 기록 없음"
            else:
                last_text = (
                    "마지막 충전 종료 후 "
                    f"{self.format_elapsed(time.time() - self.last_charge_ended_at)}"
                )
            state_text = (
                "미충전 추정" if self.charge_state == "not_charging" else "충전 판정 중"
            )
            state_text += f" | {last_text}"
        self.battery_label.setText(
            f"배터리 {voltage_text} | {state_text} | 전압추세 {slope_text}"
        )

    def rotation_calibration_path(self):
        return Path(self.args.pose_config)

    @staticmethod
    def calibration_by_speed(samples, speed_key):
        grouped = {}
        for sample in samples:
            speed = sample.get(speed_key)
            if speed is None:
                continue
            grouped.setdefault(float(speed), []).append(float(sample["scale"]))
        return {
            f"{speed:.3f}".rstrip("0").rstrip("."): {
                "coefficient": sum(scales) / len(scales),
                "sample_count": len(scales),
            }
            for speed, scales in sorted(grouped.items())
        }

    @staticmethod
    def calibration_scale_for_speed(samples, speed_key, speed, fallback):
        grouped = {}
        for sample in samples:
            sample_speed = sample.get(speed_key)
            if sample_speed is None:
                continue
            grouped.setdefault(float(sample_speed), []).append(float(sample["scale"]))
        points = sorted(
            (sample_speed, sum(scales) / len(scales))
            for sample_speed, scales in grouped.items()
        )
        if not points:
            return fallback
        speed = abs(float(speed))
        if speed <= points[0][0]:
            return points[0][1]
        if speed >= points[-1][0]:
            return points[-1][1]
        for (left_speed, left_scale), (right_speed, right_scale) in zip(points, points[1:]):
            if left_speed <= speed <= right_speed:
                ratio = (speed - left_speed) / (right_speed - left_speed)
                return left_scale + ratio * (right_scale - left_scale)
        return fallback

    def odom_rotation_scale(self):
        scales = [
            float(sample["scale"])
            for sample in self.rotation_samples
            if sample.get("odom_deg") is not None
        ]
        return self.rotation_scale if not scales else sum(scales) / len(scales)

    def current_motion_preset(self):
        return {
            "name": self.active_calibration_preset,
            "friction_coefficient": self.rotation_scale,
            "rotation_calibration_samples": self.rotation_samples,
            "rotation_friction_by_speed": self.calibration_by_speed(
                self.rotation_samples, "angular_speed_rad_s"
            ),
            "distance_coefficient": self.distance_scale,
            "distance_calibration_samples": self.distance_samples,
            "forward_friction_by_speed": self.calibration_by_speed(
                self.distance_samples, "linear_speed_m_s"
            ),
            "lateral_coefficient": self.lateral_scale,
            "lateral_calibration_samples": self.lateral_samples,
            "lateral_friction_by_speed": self.calibration_by_speed(
                self.lateral_samples, "linear_speed_m_s"
            ),
        }

    def apply_motion_preset(self, name):
        preset = self.calibration_presets[name]
        self.active_calibration_preset = name
        self.load_state = name
        self.rotation_samples = list(preset.get("rotation_calibration_samples", []))
        self.rotation_scale = float(preset.get("friction_coefficient", 1.0))
        self.distance_samples = list(preset.get("distance_calibration_samples", []))
        self.distance_scale = float(preset.get("distance_coefficient", 1.0))
        self.lateral_samples = list(preset.get("lateral_calibration_samples", []))
        self.lateral_scale = float(preset.get("lateral_coefficient", 1.0))

    def refresh_preset_combo(self):
        self.preset_combo.blockSignals(True)
        self.preset_combo.clear()
        self.preset_combo.addItems(sorted(self.calibration_presets))
        self.preset_combo.setCurrentText(self.active_calibration_preset)
        self.preset_combo.blockSignals(False)
        self.preset_label.setText(f"현재 preset: {self.active_calibration_preset}")

    def load_selected_calibration_preset(self):
        name = self.preset_combo.currentText().strip()
        if name not in self.calibration_presets:
            self.preset_label.setText("없는 preset입니다. 현재값 저장으로 먼저 생성하세요")
            return
        self.apply_motion_preset(name)
        try:
            self.persist_rotation_calibration()
        except OSError as exc:
            self.preset_label.setText(f"불러오기 저장 실패: {exc}")
            return
        self.refresh_preset_combo()
        self.refresh_rotation_label("preset 불러옴")
        self.refresh_distance_label()
        self.refresh_lateral_label()

    def save_current_calibration_preset(self):
        name = self.preset_combo.currentText().strip()
        if not name:
            self.preset_label.setText("preset 이름을 입력하세요")
            return
        self.active_calibration_preset = name[:64]
        self.load_state = self.active_calibration_preset
        self.calibration_presets[self.active_calibration_preset] = self.current_motion_preset()
        try:
            self.persist_rotation_calibration()
        except OSError as exc:
            self.preset_label.setText(f"preset 저장 실패: {exc}")
            return
        self.refresh_preset_combo()
        self.preset_label.setText(f"저장됨: {self.active_calibration_preset}")

    def load_rotation_calibration(self):
        path = self.rotation_calibration_path()
        self.rotation_scale = self.args.friction_coefficient
        migrated = False
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                self.camera_distance_scale = data.get(
                    "camera_distance_scale_cm_per_pnp_unit"
                )
                self.camera_distance_offset = data.get("camera_distance_offset_cm")
                self.camera_yaw_bias = float(data.get("yaw_bias_deg", 0.0))
                self.centerline_offset_cm = float(
                    data.get("centerline_offset_cm", 0.0)
                )
                self.lateral_overrun_cm = float(
                    data.get("lateral_overrun_cm", 0.0)
                )
                self.rotation_overrun_deg = float(
                    data.get("rotation_overrun_deg", 0.0)
                )
                self.small_rotation_threshold_deg = max(
                    0.0, float(data.get("small_rotation_threshold_deg", 5.0))
                )
                self.small_rotation_speed_rad_s = max(
                    0.05, float(data.get("small_rotation_speed_rad_s", 0.55))
                )
                self.arc_insertion_distance.setValue(float(
                    data.get("insertion_distance_cm", 12.0)
                ))
                self.near_center_check_distance_cm = float(
                    data.get("near_center_check_distance_cm", 25.0)
                )
                self.arc_cycle_pause_sec = min(10.0, max(
                    0.1, float(data.get("arc_cycle_pause_sec", 0.7))
                ))
                self.stable_detection_frames = min(30, max(
                    1, int(data.get("stable_detection_frames", 5))
                ))
                self.search_circle_diameter_m = min(10.0, max(
                    0.20, float(data.get("search_circle_diameter_m", 1.34))
                ))
                self.target_search_angular_rad_s = (
                    2.0 * self.target_search_linear_m_s
                    / self.search_circle_diameter_m
                )
                self.args.stop_distance = min(2.0, max(
                    0.05, float(data.get(
                        "lidar_stop_distance_m", self.args.stop_distance
                    ))
                ))
                self.node.lidar_self_filter_distance_m = min(1.0, max(
                    0.05, float(data.get(
                        "lidar_self_filter_distance_m",
                        self.node.lidar_self_filter_distance_m,
                    ))
                ))
                self.camera_calibration_samples = list(
                    data.get("camera_calibration_samples", [])
                )[-2:]
                active = str(data.get(
                    "active_calibration_preset", data.get("load_state", "unloaded")
                ))
                presets = data.get("calibration_presets")
                if not isinstance(presets, dict) or not presets:
                    presets = {active: {
                        "name": active,
                        "friction_coefficient": float(data.get("friction_coefficient", 1.0)),
                        "rotation_calibration_samples": list(
                            data.get("rotation_calibration_samples", [])
                        ),
                        "distance_coefficient": float(data.get("distance_coefficient", 1.0)),
                        "distance_calibration_samples": list(
                            data.get("distance_calibration_samples", [])
                        ),
                        "lateral_coefficient": float(data.get("lateral_coefficient", 1.0)),
                        "lateral_calibration_samples": list(
                            data.get("lateral_calibration_samples", [])
                        ),
                    }}
                    migrated = True
                self.calibration_presets = presets
                if active not in self.calibration_presets:
                    active = sorted(self.calibration_presets)[0]
                self.apply_motion_preset(active)
            except (OSError, TypeError, ValueError, KeyError):
                self.rotation_samples = []
                self.rotation_scale = 1.0
                self.distance_samples = []
                self.distance_scale = 1.0
                self.lateral_samples = []
                self.lateral_scale = 1.0
                self.calibration_presets = {
                    "unloaded": self.current_motion_preset()
                }
        if not self.calibration_presets:
            self.calibration_presets = {
                "unloaded": self.current_motion_preset()
            }
        self.refresh_preset_combo()
        self.refresh_rotation_label()
        self.refresh_distance_label()
        self.refresh_lateral_label()
        self.refresh_camera_calibration_label()
        if migrated:
            try:
                self.persist_rotation_calibration()
            except OSError:
                self.preset_label.setText("기존 config의 preset 이관 저장 실패")

    def persist_rotation_calibration(self):
        path = self.rotation_calibration_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {}
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, TypeError, ValueError):
                data = {}
        self.calibration_presets[self.active_calibration_preset] = self.current_motion_preset()
        data.update({
            "load_state": self.active_calibration_preset,
            "active_calibration_preset": self.active_calibration_preset,
            "calibration_presets": self.calibration_presets,
            "camera_pitch_deg": self.args.camera_pitch_deg,
            "friction_coefficient": self.rotation_scale,
            "rotation_convention": "vehicle CCW positive; target face yaw is opposite vehicle rotation",
            "rotation_calibration_samples": self.rotation_samples,
            "distance_coefficient": self.distance_scale,
            "distance_convention": "calibrated odom path length; W/S forward/reverse",
            "distance_calibration_samples": self.distance_samples,
            "lateral_coefficient": self.lateral_scale,
            "lateral_convention": "vehicle right positive; A is left and D is right",
            "lateral_calibration_samples": self.lateral_samples,
            "camera_distance_scale_cm_per_pnp_unit": self.camera_distance_scale,
            "camera_distance_offset_cm": self.camera_distance_offset,
            "camera_distance_reference": "fork tip to target front face",
            "camera_calibration_samples": self.camera_calibration_samples,
            "yaw_bias_deg": self.camera_yaw_bias,
            "centerline_offset_cm": self.centerline_offset_cm,
            "lateral_overrun_cm": self.lateral_overrun_cm,
            "rotation_overrun_deg": self.rotation_overrun_deg,
            "small_rotation_threshold_deg": self.small_rotation_threshold_deg,
            "small_rotation_speed_rad_s": self.small_rotation_speed_rad_s,
            "insertion_distance_cm": self.arc_insertion_distance.value(),
            "near_center_check_distance_cm": self.near_center_check_distance_cm,
            "arc_cycle_pause_sec": self.arc_cycle_pause_sec,
            "stable_detection_frames": self.stable_detection_frames,
            "search_circle_diameter_m": self.search_circle_diameter_m,
            "lidar_stop_distance_m": self.args.stop_distance,
            "lidar_self_filter_distance_m": self.node.lidar_self_filter_distance_m,
        })
        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def current_vehicle_rotation_deg(self):
        current = self.node.odom_yaw_unwrapped
        if current is None or self.rotation_reference is None:
            return None
        return math.degrees(current - self.rotation_reference) * self.rotation_scale

    @staticmethod
    def add_directional_overrun(value, overrun):
        """Adjust planned magnitude: positive adds travel, negative removes it."""
        if abs(value) < 1e-6 or abs(overrun) < 1e-9:
            return value
        return math.copysign(max(0.0, abs(value) + overrun), value)

    def motor_speed_summary(self, started_at):
        events = [
            event for event in self.node.motor_events
            if started_at is not None and event["monotonic"] >= started_at
        ]
        nonzero = [
            abs(float(motor["rps"]))
            for event in events
            for motor in event["motors"]
            if abs(float(motor["rps"])) > 1e-4
        ]
        return {
            "event_count": len(events),
            "mean_abs_rps": None if not nonzero else sum(nonzero) / len(nonzero),
            "max_abs_rps": None if not nonzero else max(nonzero),
        }

    def refresh_rotation_label(self, prefix=""):
        estimate = self.current_vehicle_rotation_deg()
        estimate_text = "기준 미설정" if estimate is None else f"차체 {estimate:+.1f}° / 물체 {-estimate:+.1f}°"
        sample_text = f"샘플 {len(self.rotation_samples)}회 | 배율 {self.rotation_scale:.4f}"
        self.rotation_label.setText(
            f"{prefix + ' | ' if prefix else ''}{sample_text} | {estimate_text}"
        )

    def start_rotation_trial(self):
        restarted = self.rotation_trial_start is not None
        self.rotation_trial_start = self.command_rotation_rad
        self.rotation_trial_started_at = time.monotonic()
        self.rotation_trial_speed = self.angular.value() / self.angular.speed_scale
        self.rotation_trial.setText("회전 측정 다시 시작")
        prefix = "이전 측정 폐기 후 다시 시작" if restarted else "측정 중"
        self.refresh_rotation_label(f"{prefix}: Q/E로 실제각만큼 회전")

    def save_rotation_trial(self):
        if self.rotation_trial_start is None:
            self.refresh_rotation_label("먼저 회전 측정 시작을 누르세요")
            return
        command_deg = math.degrees(self.command_rotation_rad - self.rotation_trial_start)
        if abs(command_deg) < 1.0:
            self.refresh_rotation_label("실패: 측정된 회전이 너무 작음")
            return
        actual_deg = self.rotation_actual.value()
        sample_scale = actual_deg / abs(command_deg)
        previous_scale = self.rotation_scale if self.rotation_samples else None
        self.rotation_samples.append({
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "load_state": self.load_state,
            "actual_deg": actual_deg,
            "command_deg": command_deg,
            "scale": sample_scale,
            "angular_speed_rad_s": self.rotation_trial_speed,
            "set_motor": self.motor_speed_summary(self.rotation_trial_started_at),
            "difference_vs_previous_percent": (
                None if previous_scale is None
                else (sample_scale / previous_scale - 1.0) * 100.0
            ),
        })
        self.rotation_scale = sum(
            float(sample["scale"]) for sample in self.rotation_samples
        ) / len(self.rotation_samples)
        self.rotation_trial_start = None
        self.rotation_trial_started_at = None
        self.rotation_trial.setText("회전 측정 시작")
        try:
            self.persist_rotation_calibration()
        except OSError as exc:
            self.refresh_rotation_label(f"보정 저장 실패: {exc}")
            return
        difference = self.rotation_samples[-1]["difference_vs_previous_percent"]
        difference_text = "" if difference is None else f" | 이전 대비 {difference:+.1f}%"
        self.refresh_rotation_label(
            f"추가됨: {actual_deg:.1f}° / 명령 {command_deg:+.1f}°{difference_text}"
        )

    def set_rotation_reference(self):
        if self.node.odom_yaw_unwrapped is None:
            self.refresh_rotation_label("실패: odom 없음")
            return
        self.rotation_reference = self.node.odom_yaw_unwrapped
        self.set_truth_yaw(0.0, "calibrated_odom")
        self.refresh_rotation_label("물체 정면 기준 설정")

    def set_truth_yaw(self, value, source):
        wrapped = math.degrees(math.atan2(math.sin(math.radians(value)), math.cos(math.radians(value))))
        self.updating_truth_yaw = True
        self.truth_yaw.setValue(wrapped)
        self.updating_truth_yaw = False
        self.truth_yaw_source = source

    def on_truth_yaw_edited(self, _value):
        if not self.updating_truth_yaw:
            self.truth_yaw_source = "manual"

    def update_rotation_estimate(self):
        estimate = self.current_vehicle_rotation_deg()
        if estimate is not None and self.auto_truth_yaw.isChecked():
            self.set_truth_yaw(-estimate, "calibrated_odom")
        self.refresh_rotation_label()
        self.refresh_distance_label()

    def current_drive_measurement(self):
        current_position = self.node.odom_position
        if (
            current_position is None
            or self.drive_reference_position is None
            or self.drive_reference_path is None
            or self.drive_reference_yaw is None
        ):
            return None
        dx = current_position[0] - self.drive_reference_position[0]
        dy = current_position[1] - self.drive_reference_position[1]
        yaw = self.drive_reference_yaw
        forward_m = math.cos(yaw) * dx + math.sin(yaw) * dy
        left_m = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return {
            "path_cm": (self.node.odom_path_m - self.drive_reference_path) * 100.0 * self.distance_scale,
            "forward_cm": forward_m * 100.0 * self.distance_scale,
            "lateral_right_cm": -left_m * 100.0 * self.distance_scale,
        }

    def refresh_distance_label(self, prefix=""):
        measurement = self.current_drive_measurement()
        current_text = (
            "기준 미설정"
            if measurement is None
            else f"누적 {measurement['path_cm']:.1f} cm | 전후 {measurement['forward_cm']:+.1f} cm"
        )
        sample_text = f"샘플 {len(self.distance_samples)}회 | 배율 {self.distance_scale:.4f}"
        self.distance_label.setText(
            f"{prefix + ' | ' if prefix else ''}{sample_text} | {current_text}"
        )

    def start_distance_trial(self):
        restarted = self.distance_trial_start is not None
        self.distance_trial_start = self.command_forward_m
        self.distance_trial_started_at = time.monotonic()
        self.distance_trial_speed = self.linear.value() / self.linear.speed_scale
        self.distance_trial.setText("주행 측정 다시 시작")
        prefix = "이전 측정 폐기 후 다시 시작" if restarted else "측정 중"
        self.refresh_distance_label(f"{prefix}: W/S로 실제거리만큼 주행")

    def save_distance_trial(self):
        if self.distance_trial_start is None:
            self.refresh_distance_label("먼저 주행 측정 시작을 누르세요")
            return
        command_cm = abs(self.command_forward_m - self.distance_trial_start) * 100.0
        if command_cm < 1.0:
            self.refresh_distance_label("실패: 측정된 주행이 너무 짧음")
            return
        actual_cm = self.distance_actual.value()
        sample_scale = actual_cm / command_cm
        previous_scale = self.distance_scale if self.distance_samples else None
        self.distance_samples.append({
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "load_state": self.load_state,
            "actual_cm": actual_cm,
            "command_cm": command_cm,
            "scale": sample_scale,
            "linear_speed_m_s": self.distance_trial_speed,
            "set_motor": self.motor_speed_summary(self.distance_trial_started_at),
            "difference_vs_previous_percent": (
                None if previous_scale is None
                else (sample_scale / previous_scale - 1.0) * 100.0
            ),
        })
        self.distance_scale = sum(
            float(sample["scale"]) for sample in self.distance_samples
        ) / len(self.distance_samples)
        self.distance_trial_start = None
        self.distance_trial_started_at = None
        self.distance_trial.setText("주행 측정 시작")
        try:
            self.persist_rotation_calibration()
        except OSError as exc:
            self.refresh_distance_label(f"보정 저장 실패: {exc}")
            return
        difference = self.distance_samples[-1]["difference_vs_previous_percent"]
        difference_text = "" if difference is None else f" | 이전 대비 {difference:+.1f}%"
        self.refresh_distance_label(
            f"추가됨: {actual_cm:.1f} cm / 명령 {command_cm:.1f} cm{difference_text}"
        )

    def set_drive_reference(self):
        if self.node.odom_position is None or self.node.odom_yaw_unwrapped is None:
            self.refresh_distance_label("실패: odom 없음")
            return
        self.drive_reference_path = self.node.odom_path_m
        self.drive_reference_position = self.node.odom_position
        self.drive_reference_yaw = self.node.odom_yaw_unwrapped
        self.refresh_distance_label("주행 기준 설정")

    def refresh_lateral_label(self, prefix=""):
        sample_text = f"샘플 {len(self.lateral_samples)}회 | 배율 {self.lateral_scale:.4f}"
        self.lateral_label.setText(
            f"{prefix + ' | ' if prefix else ''}{sample_text} | 오른쪽 + / 왼쪽 -"
        )

    def start_lateral_trial(self):
        restarted = self.lateral_trial_start is not None
        self.lateral_trial_start = self.command_lateral_m
        self.lateral_trial.setText("좌우 측정 다시 시작")
        prefix = "이전 측정 폐기 후 다시 시작" if restarted else "측정 중"
        self.refresh_lateral_label(f"{prefix}: A/D로 실제거리만큼 횡이동")

    def save_lateral_trial(self):
        if self.lateral_trial_start is None:
            self.refresh_lateral_label("먼저 좌우 측정 시작을 누르세요")
            return
        command_cm = abs(self.command_lateral_m - self.lateral_trial_start) * 100.0
        if command_cm < 1.0:
            self.refresh_lateral_label("실패: 측정된 횡이동이 너무 짧음")
            return
        actual_cm = self.lateral_actual.value()
        sample_scale = actual_cm / command_cm
        self.lateral_samples.append({
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "load_state": self.load_state,
            "actual_cm": actual_cm,
            "command_cm": command_cm,
            "scale": sample_scale,
        })
        self.lateral_scale = sum(
            float(sample["scale"]) for sample in self.lateral_samples
        ) / len(self.lateral_samples)
        self.lateral_trial_start = None
        self.lateral_trial.setText("좌우 측정 시작")
        try:
            self.persist_rotation_calibration()
        except OSError as exc:
            self.refresh_lateral_label(f"보정 저장 실패: {exc}")
            return
        self.refresh_lateral_label(f"추가됨: {actual_cm:.1f} cm / 명령 {command_cm:.1f} cm")

    def mapping_motion(self):
        if self.mapping_start_integrals is None:
            return None
        current = (
            self.command_forward_m,
            self.command_lateral_m,
            self.command_rotation_rad,
            self.command_forward_abs_m,
            self.command_lateral_abs_m,
            self.command_rotation_abs_rad,
        )
        delta = [value - start for value, start in zip(current, self.mapping_start_integrals)]
        return {
            "forward_cm": delta[0] * 100.0 * self.distance_scale,
            "lateral_right_cm": -delta[1] * 100.0 * self.lateral_scale,
            "rotation_ccw_deg": math.degrees(delta[2]) * self.rotation_scale,
            "absolute_forward_cm": delta[3] * 100.0 * self.distance_scale,
            "absolute_lateral_cm": delta[4] * 100.0 * self.lateral_scale,
            "absolute_rotation_deg": math.degrees(delta[5]) * self.rotation_scale,
        }

    def solve_mapping_trajectory(self, events, end_t_sec):
        """Integrate every calibrated cmd_vel sample on the initial vehicle x/y plane."""
        x_m = y_m = yaw_rad = path_m = 0.0
        initial_lateral_m = 0.0
        post_turn_forward_m = post_turn_right_m = 0.0
        rotation_started = False
        samples = [{"t_sec": 0.0, "x_forward_cm": 0.0, "y_right_cm": 0.0,
                    "yaw_ccw_deg": 0.0}]
        for index, event in enumerate(events):
            next_t = events[index + 1]["t_sec"] if index + 1 < len(events) else end_t_sec
            dt = min(max(float(next_t) - float(event["t_sec"]), 0.0), 0.2)
            command = event.get("cmd_vel", {})
            raw_forward = float(command.get("linear_x", 0.0))
            raw_left = float(command.get("linear_y", 0.0))
            raw_yaw = float(command.get("angular_z", 0.0))
            forward_scale = self.calibration_scale_for_speed(
                self.distance_samples, "linear_speed_m_s", raw_forward, self.distance_scale
            )
            lateral_scale = self.calibration_scale_for_speed(
                self.lateral_samples, "linear_speed_m_s", raw_left, self.lateral_scale
            )
            rotation_scale = self.calibration_scale_for_speed(
                self.rotation_samples, "angular_speed_rad_s", raw_yaw, self.rotation_scale
            )
            body_forward = raw_forward * forward_scale * dt
            body_left = raw_left * lateral_scale * dt
            delta_yaw = raw_yaw * rotation_scale * dt
            if abs(raw_yaw) > 1e-6:
                rotation_started = True
            if not rotation_started:
                initial_lateral_m += -body_left
            else:
                post_turn_forward_m += body_forward
                post_turn_right_m += -body_left
            middle_yaw = yaw_rad + delta_yaw * 0.5
            dx = body_forward * math.cos(middle_yaw) - body_left * math.sin(middle_yaw)
            dy_left = body_forward * math.sin(middle_yaw) + body_left * math.cos(middle_yaw)
            x_m += dx
            y_m += dy_left
            yaw_rad += delta_yaw
            path_m += math.hypot(body_forward, body_left)
            if (index % 5 == 0 or index + 1 == len(events)) and (
                abs(body_forward) > 1e-9 or abs(body_left) > 1e-9 or abs(delta_yaw) > 1e-9
            ):
                samples.append({
                    "t_sec": float(next_t),
                    "x_forward_cm": x_m * 100.0,
                    "y_right_cm": -y_m * 100.0,
                    "yaw_ccw_deg": math.degrees(yaw_rad),
                })

        correction_rad = 0.0
        if abs(post_turn_forward_m) > 1e-6 or abs(post_turn_right_m) > 1e-6:
            correction_magnitude = math.atan2(
                abs(post_turn_right_m), abs(post_turn_forward_m)
            )
            same_side = initial_lateral_m * post_turn_right_m >= 0.0
            turn_sign = 1.0 if yaw_rad >= 0.0 else -1.0
            correction_rad = turn_sign * correction_magnitude * (1.0 if same_side else -1.0)
        equivalent = {
            "initial_lateral_right_cm": (initial_lateral_m + post_turn_right_m) * 100.0,
            "rotation_ccw_deg": math.degrees(yaw_rad + correction_rad),
            "straight_forward_cm": math.hypot(post_turn_forward_m, post_turn_right_m) * 100.0,
            "later_lateral_correction_right_cm": post_turn_right_m * 100.0,
            "rotation_correction_deg": math.degrees(correction_rad),
            "method": "fold_post_turn_lateral_into_initial_lateral_and_rotation_v1",
        }
        return {
            "coordinate_system": "start x=forward, y=right, yaw=CCW positive",
            "final_pose": {
                "x_forward_cm": x_m * 100.0,
                "y_right_cm": -y_m * 100.0,
                "yaw_ccw_deg": math.degrees(yaw_rad),
            },
            "translation_path_cm": path_m * 100.0,
            "samples": samples,
            "equivalent_ideal_motion": equivalent,
        }

    def start_mapping(self):
        if self.node.frame is None:
            self.mapping_label.setText("실패: 차량 카메라 화면 없음")
            return
        restarted = self.mapping_active
        self.mapping_active = True
        self.mapping_started_at = time.monotonic()
        self.mapping_started_stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.mapping_start_frame = self.node.frame.copy()
        self.mapping_start_integrals = (
            self.command_forward_m,
            self.command_lateral_m,
            self.command_rotation_rad,
            self.command_forward_abs_m,
            self.command_lateral_abs_m,
            self.command_rotation_abs_rad,
        )
        self.mapping_start_detection = json.loads(json.dumps(self.node.latest_detection)) \
            if self.node.latest_detection is not None else None
        self.mapping_input_events = []
        self.mapping_pose = None
        self.mapping_odom_start_yaw = self.node.odom_yaw_unwrapped
        self.mapping_odom_start_position = self.node.odom_position
        self.node.motor_events.clear()
        self.mapping_start.setText("1. 매핑 다시 시작 (기존 폐기)")
        self.mapping_save.setEnabled(True)
        prefix = "기존 기록 폐기 후 다시 시작" if restarted else "기록 시작"
        self.mapping_label.setText(
            f"{prefix}: A/D 횡이동 → Q/E 회전 → W/S 전후이동"
        )
        self.setFocus()

    def update_mapping_label(self):
        if not self.mapping_active:
            return
        motion = self.mapping_motion()
        self.mapping_label.setText(
            f"기록 중 | 좌우 {motion['lateral_right_cm']:+.1f} cm | "
            f"회전 {motion['rotation_ccw_deg']:+.1f}° | 전후 {motion['forward_cm']:+.1f} cm"
        )

    def save_mapping(self):
        if not self.mapping_active or self.mapping_start_frame is None:
            self.mapping_label.setText("먼저 중앙 목표 매핑 시작을 누르세요")
            return
        self.pressed.clear()
        self.node.stop(repeats=3)
        motion = self.mapping_motion()
        trajectory = self.solve_mapping_trajectory(
            self.mapping_input_events, time.monotonic() - self.mapping_started_at
        )
        mapping_dir = Path(self.args.output_dir) / "mappings"
        mapping_dir.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("mapping_%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        initial_path = mapping_dir / f"{stem}_initial.jpg"
        final_path = mapping_dir / f"{stem}_final.jpg"
        metadata_path = mapping_dir / f"{stem}.json"
        final_frame = None if self.node.frame is None else self.node.frame.copy()
        if not cv2.imwrite(str(initial_path), self.mapping_start_frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            self.mapping_label.setText("실패: 시작 이미지 저장 오류")
            return
        if final_frame is not None:
            cv2.imwrite(str(final_path), final_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        metadata = {
            "vehicle": self.args.vehicle,
            "load_state": self.load_state,
            "calibration_preset": self.active_calibration_preset,
            "workflow": "target_centered_then_lateral_then_rotate_then_forward",
            "started_at": self.mapping_started_stamp,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "duration_sec": time.monotonic() - self.mapping_started_at,
            "initial_image": initial_path.name,
            "final_image": final_path.name if final_frame is not None else None,
            "target_top": [self.target_left.currentData(), self.target_right.currentData()],
            "motion_to_final": motion,
            "integrated_xy_trajectory": trajectory,
            "equivalent_ideal_motion": trajectory["equivalent_ideal_motion"],
            "input_timeline": self.mapping_input_events,
            "motor_command_timeline": [
                {
                    "t_sec": event["monotonic"] - self.mapping_started_at,
                    "motors": event["motors"],
                }
                for event in self.node.motor_events
                if event["monotonic"] >= self.mapping_started_at
            ],
            "calibration": {
                "forward_coefficient": self.distance_scale,
                "lateral_coefficient": self.lateral_scale,
                "rotation_coefficient": self.rotation_scale,
                "forward_samples": len(self.distance_samples),
                "lateral_samples": len(self.lateral_samples),
                "rotation_samples": len(self.rotation_samples),
            },
            "camera_pitch_deg": self.args.camera_pitch_deg,
            "battery": self.battery_snapshot(),
            "initial_detection": self.mapping_start_detection,
            "note": self.memo.toPlainText().strip(),
        }
        try:
            metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        except OSError as exc:
            self.mapping_label.setText(f"이미지만 저장됨: {exc}")
            return
        self.mapping_active = False
        self.mapping_start_frame = None
        self.mapping_start_integrals = None
        self.mapping_start_detection = None
        self.mapping_input_events = []
        self.mapping_pose = None
        self.mapping_start.setText("1. 중앙 목표 매핑 시작")
        self.mapping_save.setEnabled(False)
        self.memo.clear()
        ideal = trajectory["equivalent_ideal_motion"]
        self.mapping_label.setText(
            f"저장됨 | 이상적 좌우 {ideal['initial_lateral_right_cm']:+.1f} cm | "
            f"회전 {ideal['rotation_ccw_deg']:+.1f}° | 직진 {ideal['straight_forward_cm']:.1f} cm"
        )

    @staticmethod
    def speed_slider(value, minimum, maximum, step, suffix):
        scale = round(1.0 / step)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(round(minimum * scale), round(maximum * scale))
        slider.setValue(round(value * scale))
        slider.setMinimumWidth(220)
        label = QLabel()
        label.setMinimumWidth(82)
        update_label = lambda raw: label.setText(f"{raw / scale:.2f}{suffix}")
        slider.valueChanged.connect(update_label)
        update_label(slider.value())
        slider.speed_scale = scale
        return slider, label

    def make_move_button(self, text, key):
        button = QPushButton(text)
        button.setMinimumHeight(48)
        button.pressed.connect(lambda: self.set_pressed(key, True))
        button.released.connect(lambda: self.set_pressed(key, False))
        return button

    def set_pressed(self, key, active):
        if active:
            self.pressed.add(key)
        else:
            self.pressed.discard(key)
            self.node.stop()

    def make_fork_button(self, text, key):
        button = QPushButton(text)
        button.setMinimumHeight(48)
        button.pressed.connect(lambda: self.set_fork_pressed(key, True))
        button.released.connect(lambda: self.set_fork_pressed(key, False))
        return button

    def set_fork_pressed(self, key, active):
        if active:
            self.pressed.add(key)
            self.publish_fork_from_keys()
        else:
            self.pressed.discard(key)
            self.publish_fork_from_keys()

    def publish_fork_from_keys(self):
        if Qt.Key_Up in self.pressed and Qt.Key_Down not in self.pressed:
            self.node.publish_fork("UP")
        elif Qt.Key_Down in self.pressed and Qt.Key_Up not in self.pressed:
            self.node.publish_fork("DOWN")
        else:
            self.node.publish_fork("STOP")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.emergency_stop()
            return
        if event.key() == Qt.Key_R and not event.isAutoRepeat():
            self.record.toggle()
            return
        if event.key() in self.MOVEMENT_KEYS:
            self.pressed.add(event.key())
            event.accept()
            return
        if event.key() in self.FORK_KEYS:
            if not event.isAutoRepeat():
                self.pressed.add(event.key())
                self.publish_fork_from_keys()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in self.MOVEMENT_KEYS and not event.isAutoRepeat():
            self.pressed.discard(event.key())
            self.node.stop()
            event.accept()
            return
        if event.key() in self.FORK_KEYS and not event.isAutoRepeat():
            self.pressed.discard(event.key())
            self.publish_fork_from_keys()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def eventFilter(self, watched, event):
        """Capture drive keys even when a checkbox, button, or spinbox has focus."""
        # The memo editor must receive ordinary letters, spaces, and arrow keys
        # instead of treating them as vehicle controls.
        preset_editor = self.preset_combo.lineEdit()
        if self.memo.hasFocus() or (preset_editor is not None and preset_editor.hasFocus()):
            return super().eventFilter(watched, event)
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space and not event.isAutoRepeat():
                self.emergency_stop()
                return True
            if event.key() == Qt.Key_R and not event.isAutoRepeat():
                self.record.toggle()
                return True
            if event.key() in self.MOVEMENT_KEYS:
                self.pressed.add(event.key())
                return True
            if event.key() in self.FORK_KEYS:
                if not event.isAutoRepeat():
                    self.pressed.add(event.key())
                    self.publish_fork_from_keys()
                return True
        elif event.type() == QEvent.KeyRelease:
            if event.key() in self.MOVEMENT_KEYS and not event.isAutoRepeat():
                self.pressed.discard(event.key())
                self.node.stop()
                return True
            if event.key() in self.FORK_KEYS and not event.isAutoRepeat():
                self.pressed.discard(event.key())
                self.publish_fork_from_keys()
                return True
        return super().eventFilter(watched, event)

    def focusOutEvent(self, event):
        self.pressed.clear()
        self.node.stop(repeats=3)
        self.node.publish_fork("STOP")
        super().focusOutEvent(event)

    def on_drive_mode_changed(self, _index):
        self.pressed.difference_update(self.MOVEMENT_KEYS)
        self.node.stop(repeats=3)
        mode = self.drive_mode.currentData()
        if mode == "mecanum":
            self.status.setText("MECANUM: A/D = lateral, Q/E = rotate in place")
        else:
            self.status.setText("CAR-LIKE: A/D = steering, Q/E = rotate in place")
        self.setFocus()

    @staticmethod
    def calibrated_visual_yaw(frontal_error):
        """Monotonic map from tag-row perspective to measured pallet yaw."""
        sign = 1.0 if frontal_error >= 0.0 else -1.0
        error = abs(float(frontal_error))
        measured = ((0.0, 0.0), (0.0324, 8.65), (0.0952, 17.50), (0.1697, 26.82))
        for (left_error, left_yaw), (right_error, right_yaw) in zip(measured, measured[1:]):
            if error <= right_error:
                ratio = (error - left_error) / max(right_error - left_error, 1e-6)
                return sign * (left_yaw + ratio * (right_yaw - left_yaw))
        slope = (measured[-1][1] - measured[-2][1]) / (
            measured[-1][0] - measured[-2][0]
        )
        magnitude = measured[-1][1] + slope * (error - measured[-1][0])
        return sign * min(magnitude, 64.5)

    @staticmethod
    def cubic_arc_waypoints(goal_x, goal_y, goal_yaw, count=41):
        """Solve a forward straight segment followed by an exact terminal arc."""
        denominator = 1.0 - math.cos(goal_yaw)
        if abs(goal_yaw) > math.radians(2.0) and abs(denominator) > 1e-6:
            radius = goal_y / denominator
            straight_length = goal_x - radius * math.sin(goal_yaw)
            arc_length = abs(radius * goal_yaw)
            if straight_length >= 0.0 and abs(radius) >= 0.12:
                total_length = max(straight_length + arc_length, 1e-6)
                waypoints = []
                for index in range(count):
                    distance = total_length * index / (count - 1)
                    if distance <= straight_length or arc_length < 1e-6:
                        x, y, yaw = distance, 0.0, 0.0
                    else:
                        arc_ratio = min((distance - straight_length) / arc_length, 1.0)
                        yaw = goal_yaw * arc_ratio
                        x = straight_length + radius * math.sin(yaw)
                        y = radius * (1.0 - math.cos(yaw))
                    waypoints.append((x, y, yaw))
                waypoints[-1] = (goal_x, goal_y, goal_yaw)
                return waypoints

        # Degenerate poses that cannot be represented by straight-then-arc
        # retain the position-reaching circular fallback.
        distance_sq = max(goal_x * goal_x + goal_y * goal_y, 1e-6)
        curvature = 2.0 * goal_y / distance_sq
        total_yaw = 2.0 * math.atan2(goal_y, goal_x)
        waypoints = []
        for index in range(count):
            t = index / (count - 1)
            yaw = total_yaw * t
            if abs(curvature) < 1e-6:
                x, y = goal_x * t, 0.0
            else:
                x = math.sin(yaw) / curvature
                y = (1.0 - math.cos(yaw)) / curvature
            waypoints.append((x, y, yaw))
        waypoints[-1] = (goal_x, goal_y, total_yaw)
        return waypoints

    def valid_arc_measurement(self):
        """Return a quality-gated target measurement shared by all ARC phases."""
        detection = self.node.latest_detection
        age = time.monotonic() - self.node.latest_detection_monotonic
        candidate = None if detection is None else detection.get("candidate")
        pnp = None if candidate is None else candidate.get("pnp")
        if age > 0.8 or candidate is None or pnp is None:
            return None, None, "선택 목표의 최신 자세 검출 없음"
        if detection.get("target_top") != [
            self.target_left.currentData(), self.target_right.currentData()
        ]:
            return None, None, "GUI와 검출 목표 불일치"
        if int(candidate.get("streak", 0)) < self.stable_detection_frames:
            return None, None, "검출이 아직 안정되지 않음"
        forward_cm = pnp.get("forward_distance_cm")
        if forward_cm is None or not math.isfinite(float(forward_cm)):
            return None, None, "목표 거리값이 유효하지 않음"
        if not 10.0 <= float(forward_cm) <= 300.0:
            return None, None, "목표 거리값이 유효하지 않음"
        if float(pnp.get("reprojection_error_px", 999.0)) > 3.0:
            return None, None, "자세 재투영 오차가 3px 초과"
        if abs(float(candidate.get("frontal_error", 999.0))) > 0.35:
            return None, None, "태그 원근값이 보정 범위 밖"
        return candidate, pnp, None

    def valid_arc_reobserve_measurement(self):
        """Relax only the close-range limits while the vehicle is stopped."""
        detection = self.node.latest_detection
        age = time.monotonic() - self.node.latest_detection_monotonic
        candidate = None if detection is None else detection.get("candidate")
        pnp = None if candidate is None else candidate.get("pnp")
        if age > 0.8 or candidate is None or pnp is None:
            return None, None, "최신 목표 자세 없음"
        if detection.get("target_top") != [
            self.target_left.currentData(), self.target_right.currentData()
        ]:
            return None, None, "목표 불일치"
        if int(candidate.get("streak", 0)) < self.stable_detection_frames:
            return None, None, "검출 안정화 대기"
        forward_cm = pnp.get("forward_distance_cm")
        if forward_cm is None or not math.isfinite(float(forward_cm)):
            return None, None, "거리값 오류"
        if not 4.0 <= float(forward_cm) <= 200.0:
            return None, None, "근거리 거리값 범위 밖"
        if float(pnp.get("reprojection_error_px", 999.0)) > 5.0:
            return None, None, "재투영 오차 5px 초과"
        return candidate, pnp, None

    def replan_arc_from_live_measurement(self, candidate, pnp):
        target_x = float(pnp["forward_distance_cm"]) / 100.0
        target_y = (
            -float(pnp.get("lateral_ratio", 0.0)) * target_x
            + self.centerline_offset_cm / 100.0
        )
        # At close range the tag-height regression is outside its calibrated
        # range.  PnP yaw is used only for this stopped, second observation.
        goal_yaw = max(
            -math.radians(45.0),
            min(math.radians(45.0), math.radians(-float(pnp.get("yaw_deg", 0.0)))),
        )
        requested_standoff = self.arc_standoff.value() / 100.0
        standoff = min(requested_standoff, max(0.03, target_x * 0.5))
        goal_x = target_x - standoff * math.cos(goal_yaw)
        goal_y = target_y - standoff * math.sin(goal_yaw)
        if goal_x < 0.005:
            return False
        odom_position = self.node.odom_position
        odom_yaw = self.node.odom_yaw_unwrapped
        c0, s0 = math.cos(odom_yaw), math.sin(odom_yaw)
        target_world = {
            "x": odom_position[0] + c0 * target_x - s0 * target_y,
            "y": odom_position[1] + s0 * target_x + c0 * target_y,
            "yaw": odom_yaw + goal_yaw,
        }
        self.arc_plan = {
            "created_at": time.monotonic(),
            "source": "stopped_camera_reobserve",
            "controller_mode": "variable_curvature_pose",
            "target_forward_cm": target_x * 100.0,
            "target_lateral_left_cm": target_y * 100.0,
            "target_x_m": target_x,
            "target_y_m": target_y,
            "face_yaw_deg": -float(pnp.get("yaw_deg", 0.0)),
            "goal_x_m": goal_x,
            "goal_y_m": goal_y,
            "goal_yaw_rad": goal_yaw,
            "waypoints": self.cubic_arc_waypoints(goal_x, goal_y, goal_yaw),
            "straight_length_m": None,
            "arc_radius_m": None,
            "alignment_distance_m": standoff,
            "insertion_distance_m": self.arc_insertion_distance.value() / 100.0,
            "target_world": target_world,
            "plan_odom_position": odom_position,
            "plan_odom_yaw": odom_yaw,
        }
        self.arc_start_position = odom_position
        self.arc_start_yaw = odom_yaw
        self.arc_target_world = dict(target_world)
        self.arc_pass_count += 1
        self.arc_reobserve_active = False
        self.arc_reobserve_started_at = None
        self.node.log_telemetry_event("arc_stopped_replan", {
            "pass": self.arc_pass_count + 1,
            "target_forward_cm": target_x * 100.0,
            "target_lateral_left_cm": target_y * 100.0,
            "goal_yaw_ccw_deg": math.degrees(goal_yaw),
            "alignment_distance_cm": standoff * 100.0,
            "pnp": pnp,
        })
        self.arc_label.setText(
            f"2차 재계산 완료 | 전후 {target_x*100:.1f} / "
            f"좌 {target_y*100:+.1f} cm / 각 {math.degrees(goal_yaw):+.1f}°"
        )
        return True

    def plan_arc_approach(self, reobserve=False):
        validator = (
            self.valid_arc_reobserve_measurement
            if reobserve else self.valid_arc_measurement
        )
        candidate, pnp, invalid_reason = validator()
        if invalid_reason is not None:
            self.arc_label.setText(f"실패: {invalid_reason}")
            self.arc_execute_button.setEnabled(False)
            return False
        if self.node.odom_position is None or self.node.odom_yaw_unwrapped is None:
            self.arc_label.setText("실패: odom 없음")
            self.arc_execute_button.setEnabled(False)
            return False
        selected_mode = self.approach_mode.currentData()
        target_x = float(pnp["forward_distance_cm"]) / 100.0
        # PnP/camera x is right-positive; odom/path y is left-positive.
        target_y = (
            -float(pnp.get("lateral_ratio", 0.0)) * target_x
            + self.centerline_offset_cm / 100.0
        )
        face_yaw_deg = self.calibrated_visual_yaw(float(candidate.get("frontal_error", 0.0)))
        # frontal_error describes the pallet face; the vehicle must rotate in
        # the opposite direction to become parallel with that face.
        goal_yaw = math.radians(-face_yaw_deg)
        standoff = self.arc_standoff.value() / 100.0
        goal_x = target_x - standoff * math.cos(goal_yaw)
        goal_y = target_y - standoff * math.sin(goal_yaw)
        if selected_mode == "arc" and goal_x < 0.05:
            self.arc_label.setText("실패: 정지거리보다 목표가 가까움")
            self.arc_execute_button.setEnabled(False)
            return False
        waypoints = self.cubic_arc_waypoints(goal_x, goal_y, goal_yaw)
        # The controller now changes curvature continuously from the live pose
        # error.  These legacy fields remain in logs only for compatibility.
        arc_radius = None
        straight_length = None
        odom_position = self.node.odom_position
        odom_yaw = self.node.odom_yaw_unwrapped
        c0, s0 = math.cos(odom_yaw), math.sin(odom_yaw)
        target_world = {
            "x": odom_position[0] + c0 * target_x - s0 * target_y,
            "y": odom_position[1] + s0 * target_x + c0 * target_y,
            "yaw": odom_yaw + goal_yaw,
        }
        controller_mode = (
            "variable_curvature_pose"
            if selected_mode == "arc"
            else "visual_alternating_dock"
        )
        self.arc_plan = {
            "created_at": time.monotonic(),
            "target_forward_cm": target_x * 100.0,
            "target_lateral_left_cm": target_y * 100.0,
            "centerline_offset_cm": self.centerline_offset_cm,
            "target_x_m": target_x,
            "target_y_m": target_y,
            "face_yaw_deg": face_yaw_deg,
            "goal_x_m": goal_x,
            "goal_y_m": goal_y,
            "goal_yaw_rad": goal_yaw,
            "waypoints": waypoints,
            "straight_length_m": straight_length,
            "arc_radius_m": arc_radius,
            "controller_mode": controller_mode,
            "alignment_distance_m": standoff,
            "insertion_distance_m": self.arc_insertion_distance.value() / 100.0,
            "target_world": target_world if selected_mode == "arc" else None,
            "plan_odom_position": odom_position,
            "plan_odom_yaw": odom_yaw,
        }
        self.node.log_telemetry_event("arc_plan", {
            "target_forward_cm": target_x * 100.0,
            "target_lateral_left_cm": target_y * 100.0,
            "face_yaw_deg": face_yaw_deg,
            "goal_x_cm": goal_x * 100.0,
            "goal_y_left_cm": goal_y * 100.0,
            "goal_yaw_ccw_deg": math.degrees(goal_yaw),
            "straight_length_cm": (
                None if straight_length is None else straight_length * 100.0
            ),
            "arc_radius_cm": None if arc_radius is None else arc_radius * 100.0,
            "standoff_cm": standoff * 100.0,
            "insertion_distance_cm": self.arc_insertion_distance.value(),
            "frontal_error": float(candidate.get("frontal_error", 0.0)),
            "overlap_angle_deg": candidate.get("overlap_angle_deg"),
            "pnp": pnp,
            "path": [
                {"x_cm": point[0] * 100.0, "y_left_cm": point[1] * 100.0,
                 "yaw_ccw_deg": math.degrees(point[2])}
                for point in waypoints[::4]
            ] + [{
                "x_cm": waypoints[-1][0] * 100.0,
                "y_left_cm": waypoints[-1][1] * 100.0,
                "yaw_ccw_deg": math.degrees(waypoints[-1][2]),
            }],
        })
        self.arc_execute_button.setEnabled(True)
        if selected_mode == "arc":
            mode_text = f"ARC | {standoff*100:.1f}cm 이전 정렬"
        else:
            mode_text = "횡이동 1회 → 회전 1회 → odom 직진 | 재계산 없음"
        self.arc_label.setText(
            f"계산됨: 목표 전후 {target_x*100:.1f} / 좌 {target_y*100:+.1f} cm | "
            f"팔레트각 {face_yaw_deg:+.1f}° | {mode_text} → "
            f"{self.arc_insertion_distance.value():.1f}cm 삽입"
        )
        return True

    def virtual_arc_target_relative(self):
        if (
            self.arc_target_world is None
            or self.node.odom_position is None
            or self.node.odom_yaw_unwrapped is None
        ):
            return None
        yaw = self.node.odom_yaw_unwrapped
        dx = self.arc_target_world["x"] - self.node.odom_position[0]
        dy = self.arc_target_world["y"] - self.node.odom_position[1]
        c, s = math.cos(yaw), math.sin(yaw)
        target_x = c * dx + s * dy
        target_y = -s * dx + c * dy
        target_yaw = math.atan2(
            math.sin(self.arc_target_world["yaw"] - yaw),
            math.cos(self.arc_target_world["yaw"] - yaw),
        )
        return target_x, target_y, target_yaw

    def begin_arc_manual_correction(self, result):
        self.last_arc_result = json.loads(json.dumps(result))
        self.arc_correction_started_at = time.monotonic()
        self.arc_correction_start_integrals = (
            self.command_forward_m,
            self.command_lateral_m,
            self.command_rotation_rad,
            self.command_forward_abs_m,
            self.command_lateral_abs_m,
            self.command_rotation_abs_rad,
        )
        self.arc_correction_input_events = []
        self.arc_correction_start_frame = (
            None if self.node.frame is None else self.node.frame.copy()
        )
        self.arc_correction_start_detection = (
            None if self.node.latest_detection is None
            else json.loads(json.dumps(self.node.latest_detection))
        )
        self.arc_correction_start_odom = {
            "position": (
                None if self.node.odom_position is None
                else list(self.node.odom_position)
            ),
            "yaw": self.node.odom_yaw_unwrapped,
        }
        self.arc_sample_label.setText(
            "ARC 종료 기준 기록 중 | A/D 횡이동 + W/S 직진 후 샘플 저장"
        )

    def arc_correction_motion(self):
        if self.arc_correction_start_integrals is None:
            return None
        current = (
            self.command_forward_m,
            self.command_lateral_m,
            self.command_rotation_rad,
            self.command_forward_abs_m,
            self.command_lateral_abs_m,
            self.command_rotation_abs_rad,
        )
        delta = [
            value - start
            for value, start in zip(current, self.arc_correction_start_integrals)
        ]
        return {
            "forward_cm": delta[0] * 100.0 * self.distance_scale,
            "lateral_left_cm": delta[1] * 100.0 * self.lateral_scale,
            "rotation_ccw_deg": math.degrees(delta[2]) * self.rotation_scale,
            "absolute_forward_cm": delta[3] * 100.0 * self.distance_scale,
            "absolute_lateral_cm": delta[4] * 100.0 * self.lateral_scale,
            "absolute_rotation_deg": math.degrees(delta[5]) * self.rotation_scale,
        }

    def save_arc_result_sample(self):
        if self.last_arc_result is None:
            self.arc_sample_label.setText("실패: 먼저 ARC 주행을 완료하세요")
            return
        if self.node.frame is None:
            self.arc_sample_label.setText("실패: 카메라 화면 없음")
            return
        self.arc_result_dir.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("arc_sample_%Y%m%d_%H%M%S") + \
            f"_{time.time_ns() % 1_000_000_000:09d}"
        image_path = self.arc_result_dir / f"{stem}_corrected.jpg"
        arc_end_image_path = self.arc_result_dir / f"{stem}_arc_end.jpg"
        metadata_path = self.arc_result_dir / f"{stem}.json"
        if not cv2.imwrite(
            str(image_path), self.node.frame,
            [cv2.IMWRITE_JPEG_QUALITY, 95],
        ):
            self.arc_sample_label.setText("실패: 이미지 저장 오류")
            return
        if self.arc_correction_start_frame is not None:
            cv2.imwrite(
                str(arc_end_image_path), self.arc_correction_start_frame,
                [cv2.IMWRITE_JPEG_QUALITY, 95],
            )
        correction_duration = (
            0.0 if self.arc_correction_started_at is None
            else time.monotonic() - self.arc_correction_started_at
        )
        correction_motion = self.arc_correction_motion()
        correction_trajectory = self.solve_mapping_trajectory(
            self.arc_correction_input_events, correction_duration
        )
        sample = {
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "vehicle": self.args.vehicle,
            "load_state": self.load_state,
            "calibration_preset": self.active_calibration_preset,
            "target_top": [
                self.target_left.currentData(), self.target_right.currentData()
            ],
            "arc_end_image": (
                arc_end_image_path.name
                if self.arc_correction_start_frame is not None else None
            ),
            "corrected_image": image_path.name,
            "manual_final_error": {
                "forward_short_cm": self.arc_forward_error.value(),
                "lateral_left_cm": self.arc_lateral_error.value(),
                "convention": "forward + means stopped short; lateral + means left",
            },
            "arc_result": self.last_arc_result,
            "manual_correction": {
                "duration_sec": correction_duration,
                "motion": correction_motion,
                "integrated_xy_trajectory": correction_trajectory,
                "input_timeline": self.arc_correction_input_events,
                "motor_command_timeline": [
                    {
                        "t_sec": event["monotonic"] - self.arc_correction_started_at,
                        "motors": event["motors"],
                    }
                    for event in self.node.motor_events
                    if self.arc_correction_started_at is not None
                    and event["monotonic"] >= self.arc_correction_started_at
                ],
                "start_odom": self.arc_correction_start_odom,
                "end_odom": {
                    "position": (
                        None if self.node.odom_position is None
                        else list(self.node.odom_position)
                    ),
                    "yaw": self.node.odom_yaw_unwrapped,
                },
                "start_detection": self.arc_correction_start_detection,
            },
            "final_detection": self.node.latest_detection,
            "recording_session": self.telemetry_session,
            "note": self.memo.toPlainText().strip(),
        }
        try:
            encoded = json.dumps(sample, ensure_ascii=False)
            metadata_path.write_text(
                json.dumps(sample, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            with (self.arc_result_dir / "samples.jsonl").open(
                "a", encoding="utf-8"
            ) as output:
                output.write(encoded + "\n")
        except (OSError, TypeError, ValueError) as exc:
            self.arc_sample_label.setText(f"실패: 샘플 저장 오류 {exc}")
            return
        self.arc_result_sample_count += 1
        self.arc_correction_started_at = None
        self.arc_correction_start_integrals = None
        self.arc_correction_input_events = []
        self.arc_correction_start_frame = None
        self.arc_sample_label.setText(
            f"샘플 {self.arc_result_sample_count}개 | 저장됨: "
            f"전후 {self.arc_forward_error.value():+.1f}, "
            f"좌우 {self.arc_lateral_error.value():+.1f} cm"
        )

    def fuse_live_arc_target(self, candidate, pnp):
        """Correct the odom-held target only after repeated, nearby observations."""
        if self.arc_target_world is None:
            return False
        detection_id = (self.node.latest_detection or {}).get("source_stamp_ns")
        if not detection_id:
            detection_id = self.node.latest_detection_monotonic
        if detection_id == self.arc_last_fused_detection_id:
            return False
        self.arc_last_fused_detection_id = detection_id
        yaw = self.node.odom_yaw_unwrapped
        position = self.node.odom_position
        if yaw is None or position is None:
            return False
        target_x = float(pnp["forward_distance_cm"]) / 100.0
        target_y = -float(pnp.get("lateral_ratio", 0.0)) * target_x
        visual_yaw_deg = self.calibrated_visual_yaw(
            float(candidate.get("frontal_error", 0.0))
        )
        depth_yaw = candidate.get("depth_yaw") or {}
        depth_yaw_deg = depth_yaw.get("yaw_deg")
        if depth_yaw_deg is not None and math.isfinite(float(depth_yaw_deg)):
            # Depth measures the two upper-tag depths directly.  Blend it with
            # the image yaw only when both refer to the same stable target.
            visual_yaw_deg = 0.5 * visual_yaw_deg + 0.5 * float(depth_yaw_deg)
        measured_yaw = yaw + math.radians(visual_yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        measured_x = position[0] + c * target_x - s * target_y
        measured_y = position[1] + s * target_x + c * target_y
        position_residual = math.hypot(
            measured_x - self.arc_target_world["x"],
            measured_y - self.arc_target_world["y"],
        )
        yaw_residual = math.atan2(
            math.sin(measured_yaw - self.arc_target_world["yaw"]),
            math.cos(measured_yaw - self.arc_target_world["yaw"]),
        )
        accepted = position_residual <= 0.12 and abs(yaw_residual) <= math.radians(15.0)
        self.arc_fusion_streak = self.arc_fusion_streak + 1 if accepted else 0
        fused = accepted and self.arc_fusion_streak >= 2
        material_residual = (
            position_residual > 0.05 or abs(yaw_residual) > math.radians(5.0)
        )
        self.arc_replan_streak = (
            self.arc_replan_streak + 1 if accepted and material_residual else 0
        )
        if fused:
            position_alpha, yaw_alpha = 0.20, 0.15
            self.arc_target_world["x"] += position_alpha * (
                measured_x - self.arc_target_world["x"]
            )
            self.arc_target_world["y"] += position_alpha * (
                measured_y - self.arc_target_world["y"]
            )
            self.arc_target_world["yaw"] += yaw_alpha * yaw_residual
            self.arc_last_fusion_monotonic = time.monotonic()
        self.node.log_telemetry_event("arc_target_fusion", {
            "accepted": accepted,
            "fused": fused,
            "streak": self.arc_fusion_streak,
            "replan_streak": self.arc_replan_streak,
            "position_residual_cm": position_residual * 100.0,
            "yaw_residual_deg": math.degrees(yaw_residual),
            "reprojection_error_px": pnp.get("reprojection_error_px"),
            "depth_yaw_deg": depth_yaw_deg,
        })
        return fused and self.arc_replan_streak >= 3

    def replan_arc_from_virtual_target(self):
        relative = self.virtual_arc_target_relative()
        if relative is None:
            return False
        target_x, target_y, goal_yaw = relative
        standoff = self.arc_standoff.value() / 100.0
        goal_x = target_x - standoff * math.cos(goal_yaw)
        goal_y = target_y - standoff * math.sin(goal_yaw)
        if goal_x < 0.03:
            return False
        waypoints = self.cubic_arc_waypoints(goal_x, goal_y, goal_yaw)
        self.arc_plan = {
            "created_at": time.monotonic(),
            "source": "odom_virtual_target",
            "controller_mode": "variable_curvature_pose",
            "target_forward_cm": target_x * 100.0,
            "target_lateral_left_cm": target_y * 100.0,
            "target_x_m": target_x,
            "target_y_m": target_y,
            "face_yaw_deg": math.degrees(goal_yaw),
            "goal_x_m": goal_x,
            "goal_y_m": goal_y,
            "goal_yaw_rad": goal_yaw,
            "waypoints": waypoints,
            "target_world": dict(self.arc_target_world),
            "plan_odom_position": self.node.odom_position,
            "plan_odom_yaw": self.node.odom_yaw_unwrapped,
        }
        self.arc_start_position = self.node.odom_position
        self.arc_start_yaw = self.node.odom_yaw_unwrapped
        self.arc_waypoint_index = 0
        self.arc_final_approach_active = False
        self.node.log_telemetry_event("arc_virtual_replan", {
            "target_relative": {
                "x_cm": target_x * 100.0,
                "y_left_cm": target_y * 100.0,
                "yaw_ccw_deg": math.degrees(goal_yaw),
            },
            "goal_relative": {
                "x_cm": goal_x * 100.0,
                "y_left_cm": goal_y * 100.0,
                "yaw_ccw_deg": math.degrees(goal_yaw),
            },
        })
        return True

    def start_arc_approach(self):
        internal_start = self.arc_auto_internal_start
        self.arc_auto_internal_start = False
        if self.arc_active:
            if self.arc_dock_phase == "await_insertion":
                self.continue_holonomic_insertion(time.monotonic())
                return
            self.arc_label.setText("이미 주행 실행 중")
            return
        if self.arc_plan is None:
            self.arc_label.setText("먼저 ARC 경로 계산을 누르세요")
            return
        if self.node.odom_position is None or self.node.odom_yaw_unwrapped is None:
            self.arc_label.setText("실패: odom 없음")
            return
        if time.monotonic() - self.arc_plan["created_at"] > 10.0:
            self.arc_label.setText("실패: 경로가 오래됨, 다시 계산하세요")
            self.arc_execute_button.setEnabled(False)
            return
        plan_position = self.arc_plan.get("plan_odom_position")
        plan_yaw = self.arc_plan.get("plan_odom_yaw")
        moved_after_plan = (
            plan_position is None
            or plan_yaw is None
            or math.dist(self.node.odom_position, plan_position) > 0.02
            or abs(math.atan2(
                math.sin(self.node.odom_yaw_unwrapped - plan_yaw),
                math.cos(self.node.odom_yaw_unwrapped - plan_yaw),
            )) > math.radians(2.0)
        )
        if moved_after_plan:
            self.arc_label.setText("실패: 경로 계산 후 차량이 움직임, 다시 계산하세요")
            self.arc_execute_button.setEnabled(False)
            return
        self.pressed.difference_update(self.MOVEMENT_KEYS)
        self.arc_start_position = self.node.odom_position
        self.arc_start_yaw = self.node.odom_yaw_unwrapped
        # The target belongs to the odom pose at plan time, not the later start pose.
        target_world = self.arc_plan.get("target_world")
        self.arc_target_world = None if target_world is None else dict(target_world)
        self.arc_initial_plan = json.loads(json.dumps(self.arc_plan))
        self.arc_started_at = time.monotonic()
        self.arc_waypoint_index = 0
        self.last_arc_result = None
        self.arc_correction_started_at = None
        self.arc_correction_start_integrals = None
        self.arc_correction_input_events = []
        self.arc_correction_start_frame = None
        self.arc_recovery_active = False
        self.arc_recovery_started_at = None
        self.arc_recovery_count = 0
        self.arc_fusion_streak = 0
        self.arc_last_fusion_monotonic = 0.0
        self.arc_last_fused_detection_id = None
        self.arc_replan_streak = 0
        self.arc_pass_count = 0
        self.arc_reobserve_active = False
        self.arc_reobserve_started_at = None
        self.arc_dock_lateral_target_m = None
        self.arc_dock_rotation_target_rad = None
        self.arc_dock_forward_target_m = None
        self.arc_dock_motion_anchor_position = None
        self.arc_dock_motion_anchor_yaw = None
        self.arc_dock_lateral_progress_m = 0.0
        self.arc_dock_last_control_at = None
        self.arc_dock_last_lateral_command = 0.0
        self.arc_dock_yaw_correction_count = 0
        self.arc_dock_yaw_verify_samples = []
        self.arc_dock_yaw_error_before_rotation_deg = None
        self.arc_dock_rotation_was_commanded = False
        self.arc_near_center_check_done = False
        self.arc_near_recheck_samples = []
        if self.arc_plan.get("controller_mode") == "visual_alternating_dock":
            target_x = float(self.arc_plan["target_x_m"])
            target_y = float(self.arc_plan["target_y_m"])
            planned_yaw = float(self.arc_plan["goal_yaw_rad"])
            lateral_target = target_y - target_x * math.tan(planned_yaw)
            forward_target = target_x / math.cos(planned_yaw)
            lateral_target = self.add_directional_overrun(
                lateral_target, self.lateral_overrun_cm / 100.0
            )
            target_yaw = self.add_directional_overrun(
                planned_yaw, math.radians(self.rotation_overrun_deg)
            )
            if abs(lateral_target) > 0.60:
                self.arc_label.setText(
                    f"실패: 횡이동 {lateral_target*100:+.1f}cm가 60cm 상한 초과"
                )
                return
            self.arc_dock_lateral_target_m = lateral_target
            self.arc_dock_rotation_target_rad = target_yaw
            self.arc_dock_yaw_error_before_rotation_deg = abs(
                float(self.arc_plan.get("face_yaw_deg", 0.0))
            )
            self.arc_dock_forward_target_m = forward_target
            self.arc_dock_motion_anchor_position = self.node.odom_position
            self.arc_dock_motion_anchor_yaw = self.node.odom_yaw_unwrapped
            self.arc_dock_phase = (
                "lateral_once" if abs(lateral_target) > 0.01 else "rotate_once"
            )
        else:
            self.arc_dock_phase = None
        self.arc_phase_started_at = time.monotonic()
        self.arc_dock_last_control_at = self.arc_phase_started_at
        self.arc_phase_streak = 0
        self.arc_phase_last_detection_id = None
        self.arc_forward_anchor_position = None
        self.arc_forward_anchor_yaw = None
        self.arc_forward_anchor_remaining_m = None
        if not internal_start:
            self.arc_auto_enabled = False
            self.arc_auto_pass = 1
            self.arc_auto_replan_due_at = None
            self.arc_auto_last_result = None
            self.arc_cycle_index = 1
            self.arc_cycle_replan_due_at = None
        self.arc_active = True
        self.arc_execute_button.setText("주행 실행")
        self.arc_execute_button.setEnabled(False)
        self.arc_label.setText("ARC 실행 중 | Space 또는 취소로 정지")
        self.node.log_telemetry_event("arc_start", {
            "odom_position": list(self.arc_start_position),
            "odom_yaw": self.arc_start_yaw,
            "plan_age_sec": time.monotonic() - self.arc_plan["created_at"],
            "virtual_target_world": self.arc_target_world,
            "automatic_pass": self.arc_auto_pass,
            "automatic_max_passes": self.arc_auto_max_passes,
            "lateral_overrun_cm": self.lateral_overrun_cm,
            "rotation_overrun_deg": self.rotation_overrun_deg,
        })

    def schedule_holonomic_yaw_retry(self, now, reason):
        """Retry a stopped calculate/execute cycle only after yaw rejection."""
        self.arc_active = False
        self.arc_dock_phase = None
        self.arc_phase_started_at = None
        self.arc_dock_last_lateral_command = 0.0
        self.node.stop(repeats=5)
        if not self.arc_auto_enabled or self.arc_auto_pass >= self.arc_auto_max_passes:
            self.arc_auto_enabled = False
            self.arc_auto_replan_due_at = None
            self.arc_label.setText(
                f"{reason} | 자동 재시도 {self.arc_auto_pass}회 종료"
            )
            return
        self.arc_auto_replan_due_at = now + self.arc_cycle_pause_sec
        self.arc_label.setText(
            f"{reason} | {self.arc_auto_pass}회 정지, 자동 재계산 대기"
        )
        self.node.log_telemetry_event("arc_auto_yaw_retry", {
            "pass": self.arc_auto_pass,
            "reason": reason,
        })

    def update_holonomic_auto_cycle(self, now):
        due_at = self.arc_auto_replan_due_at
        if due_at is None or now < due_at or not self.arc_auto_enabled:
            return
        self.arc_auto_replan_due_at = None
        candidate, pnp, reason = self.valid_arc_reobserve_measurement()
        if reason is not None:
            self.arc_auto_enabled = False
            if self.arc_auto_last_result is not None:
                self.begin_arc_manual_correction(self.arc_auto_last_result)
            self.arc_label.setText(f"자동 재계산 중단: {reason}")
            self.node.log_telemetry_event("arc_auto_stop", {
                "pass": self.arc_auto_pass,
                "reason": reason,
            })
            return
        if not self.plan_arc_approach(reobserve=True):
            self.arc_auto_enabled = False
            return
        self.arc_auto_pass += 1
        self.arc_auto_internal_start = True
        self.start_arc_approach()
        if not self.arc_active:
            self.arc_auto_enabled = False
            return
        self.arc_label.setText(
            f"yaw 자동 재계산 {self.arc_auto_pass}/{self.arc_auto_max_passes}회 실행"
        )

    def schedule_next_selected_cycle(self, now, reason):
        self.arc_active = False
        self.arc_dock_phase = None
        self.arc_cycle_replan_due_at = now + self.arc_cycle_pause_sec
        self.node.stop(repeats=5)
        limit_text = "∞" if self.arc_cycle_limit <= 0 else str(self.arc_cycle_limit)
        self.arc_label.setText(
            f"사이클 {self.arc_cycle_index}/{limit_text} 완료 | "
            f"{reason} | 재계산 대기"
        )

    def selected_cycle_has_next(self):
        return self.arc_cycle_limit <= 0 or self.arc_cycle_index < self.arc_cycle_limit

    def update_selected_cycle_replan(self, now):
        due_at = self.arc_cycle_replan_due_at
        if due_at is None or now < due_at:
            return
        self.arc_cycle_replan_due_at = None
        if self.arc_cycle_limit > 0 and self.arc_cycle_index >= self.arc_cycle_limit:
            return
        if not self.plan_arc_approach(reobserve=True):
            self.node.stop(repeats=5)
            if self.arc_cycle_limit <= 0:
                self.arc_cycle_replan_due_at = now + self.arc_cycle_pause_sec
                self.arc_label.setText("무제한 자동모드 | 목표 재검출 대기")
                return
            self.arc_label.setText("다음 사이클 계산 실패 | 정지")
            return
        self.arc_cycle_index += 1
        self.arc_auto_internal_start = True
        self.start_arc_approach()
        if self.arc_active:
            limit_text = "∞" if self.arc_cycle_limit <= 0 else str(self.arc_cycle_limit)
            self.arc_label.setText(
                f"사이클 {self.arc_cycle_index}/{limit_text} 실행"
            )

    def set_selected_run_mode(self, mode):
        """Apply the terminal UI's automatic-run settings to the GUI action."""
        settings = {
            "search": (0, True),
            "auto": (0, True),
            "single": (1, False),
            "cycle3": (3, False),
        }
        cycle_limit, auto_insert = settings[mode]
        self.terminal_run_mode = mode
        self.arc_cycle_limit = cycle_limit
        self.arc_auto_insert_after_verify = auto_insert
        if hasattr(self, "run_mode"):
            index = self.run_mode.findData(mode)
            if index >= 0:
                self.run_mode.setCurrentIndex(index)

    def run_selected_mode(self):
        """Start the automatic mode selected in the ARC panel."""
        if (
            self.arc_active
            or self.target_search_active
            or self.arc_cycle_replan_due_at is not None
        ):
            self.arc_label.setText("주행 또는 탐색 중에는 모드 변경 불가")
            return
        mode = self.run_mode.currentData()
        self.set_selected_run_mode(mode)
        if mode == "search":
            self.start_target_search()
            return
        if not self.plan_arc_approach():
            return
        self.start_arc_approach()

    def publish_arrival_trigger(self):
        """Trigger the independent 1.2 node without changing 1.1 GUI state."""
        if self.args.http_viewer_only:
            self.arc_label.setText("HTTP 화면 전용 모드에서는 arrival 발행 불가")
            return
        left = self.target_left.currentData()
        right = self.target_right.currentData()
        self.node.publish_arrival(left, right)
        self.arc_label.setText(f"1.2 arrival 발행: {left} / {right}")

    def publish_auto_dock_stop(self):
        if self.args.http_viewer_only:
            self.arc_label.setText("HTTP 화면 전용 모드에서는 stop 발행 불가")
            return
        self.node.publish_auto_dock_stop()
        self.auto_dock_status_label.setText("1.2 stop 요청 전송")

    def start_target_search(self, auto_lift_after_dock=False):
        self.cancel_arc_approach("원형 목표 탐색 시작")
        self.last_arc_stop_reason = None
        self.auto_lift_after_dock = bool(auto_lift_after_dock)
        self.last_auto_lift_monotonic = None
        self.set_selected_run_mode("search")
        self.target_search_active = True
        self.node.stop(repeats=3)
        radius = self.target_search_linear_m_s / self.target_search_angular_rad_s
        self.arc_label.setText(
            f"원형 탐색 중 | 반지름 약 {radius:.2f}m | SPACE/Z로 정지"
        )

    def update_target_search(self):
        candidate, _pnp, reason = self.valid_arc_measurement()
        if reason is None and candidate is not None:
            self.target_search_active = False
            self.node.stop(repeats=5)
            self.set_selected_run_mode("auto")
            if self.plan_arc_approach():
                self.start_arc_approach()
                self.arc_label.setText("목표 발견 | 무제한 자동모드 전환")
            return
        self.node.publish(
            self.target_search_linear_m_s,
            0.0,
            self.target_search_angular_rad_s,
        )
        radius = self.target_search_linear_m_s / self.target_search_angular_rad_s
        self.arc_label.setText(
            f"목표 탐색 중 | 큰 원 반지름 약 {radius:.2f}m | {reason}"
        )

    def cancel_arc_approach(self, reason="사용자 취소"):
        was_active = self.arc_active or self.target_search_active
        self.last_arc_stop_was_docking = bool(self.arc_active)
        self.arc_active = False
        self.target_search_active = False
        self.arc_auto_enabled = False
        self.arc_auto_internal_start = False
        self.arc_auto_replan_due_at = None
        self.arc_cycle_replan_due_at = None
        self.arc_recovery_active = False
        self.arc_final_approach_active = False
        self.arc_reobserve_active = False
        self.arc_reobserve_started_at = None
        self.arc_dock_phase = None
        self.arc_phase_started_at = None
        self.arc_phase_streak = 0
        self.arc_phase_last_detection_id = None
        self.arc_dock_lateral_target_m = None
        self.arc_dock_rotation_target_rad = None
        self.arc_dock_forward_target_m = None
        self.arc_dock_motion_anchor_position = None
        self.arc_dock_motion_anchor_yaw = None
        self.arc_dock_lateral_progress_m = 0.0
        self.arc_dock_last_control_at = None
        self.arc_dock_last_lateral_command = 0.0
        self.arc_dock_yaw_correction_count = 0
        self.arc_dock_yaw_verify_samples = []
        self.arc_dock_yaw_error_before_rotation_deg = None
        self.arc_dock_rotation_was_commanded = False
        self.arc_near_center_check_done = False
        self.arc_near_recheck_samples = []
        self.arc_forward_anchor_position = None
        self.arc_forward_anchor_yaw = None
        self.arc_forward_anchor_remaining_m = None
        self.auto_lift_after_dock = False
        self.last_arc_stop_reason = str(reason)
        self.node.stop(repeats=3)
        self.arc_execute_button.setText("주행 실행")
        if was_active:
            self.arc_label.setText(f"ARC 정지: {reason}")
            self.node.log_telemetry_event("arc_stop", {
                "reason": str(reason),
                "elapsed_sec": (
                    None if self.arc_started_at is None
                    else time.monotonic() - self.arc_started_at
                ),
                "odom_position": (
                    None if self.node.odom_position is None
                    else list(self.node.odom_position)
                ),
                "odom_yaw": self.node.odom_yaw_unwrapped,
            })
        self.arc_execute_button.setEnabled(self.arc_plan is not None)

    def finish_auto_lift_after_dock(self):
        """Raise once after a successful headless docking run."""
        if not self.auto_lift_after_dock:
            return False
        self.auto_lift_after_dock = False
        self.last_auto_lift_monotonic = time.monotonic()
        self.node.publish_fork("UP")
        self.node.log_telemetry_event("arc_auto_lift", {"command": "UP"})
        return True

    def start_verified_holonomic_forward(self, now, verified_yaw_deg):
        if self.arc_dock_forward_target_m is None:
            self.cancel_arc_approach("직진 목표 없음")
            return
        self.node.stop(repeats=3)
        self.arc_dock_phase = "await_insertion"
        self.arc_phase_started_at = now
        self.arc_forward_anchor_remaining_m = (
            self.arc_dock_forward_target_m
            + self.arc_plan.get(
                "insertion_distance_m",
                self.arc_insertion_distance.value() / 100.0,
            )
        )
        self.node.log_telemetry_event("arc_yaw_verified", {
            "verified_yaw_error_deg": verified_yaw_deg,
            "correction_count": self.arc_dock_yaw_correction_count,
            "next_phase": "await_insertion",
        })
        self.arc_execute_button.setText("직선 삽입 계속")
        self.arc_execute_button.setEnabled(True)
        self.arc_label.setText(
            f"yaw 재확인 {verified_yaw_deg:+.1f}° | 직선 삽입 전 정지"
        )
        if self.arc_auto_insert_after_verify:
            self.continue_holonomic_insertion(now)

    def continue_holonomic_insertion(self, now):
        if self.arc_forward_anchor_remaining_m is None:
            self.cancel_arc_approach("직진 목표 없음")
            return
        self.node.stop(repeats=3)
        self.arc_dock_phase = "forward"
        self.arc_phase_started_at = now
        self.arc_forward_anchor_position = self.node.odom_position
        self.arc_forward_anchor_yaw = self.node.odom_yaw_unwrapped
        self.arc_execute_button.setText("주행 실행")
        self.arc_execute_button.setEnabled(False)
        self.node.log_telemetry_event("arc_insertion_resume", {
            "automatic_pass": self.arc_auto_pass,
            "remaining_cm": self.arc_forward_anchor_remaining_m * 100.0,
        })
        self.arc_label.setText(
            f"직선 삽입 시작 | {self.arc_forward_anchor_remaining_m*100:.1f}cm"
        )

    def update_holonomic_dock(self, now, x, y, relative_yaw):
        """Use one planned strafe, one planned rotation, then one odom drive."""
        if self.arc_dock_phase == "await_insertion":
            self.arc_label.setText("직선 삽입 전 정지 | ENTER로 계속")
            return

        if self.arc_dock_phase == "near_center_recheck":
            if now - self.arc_phase_started_at < 0.4:
                self.arc_label.setText("근거리 정지 안정화 중")
                return
            detection = self.node.latest_detection or {}
            detection_id = detection.get("source_stamp_ns")
            if detection_id is None:
                detection_id = self.node.latest_detection_monotonic
            if detection_id == self.arc_phase_last_detection_id:
                return
            candidate, pnp, invalid_reason = self.valid_arc_reobserve_measurement()
            if invalid_reason is not None:
                self.arc_label.setText(f"근거리 중심 재검출 대기 | {invalid_reason}")
                return
            self.arc_phase_last_detection_id = detection_id
            target_x = float(pnp["forward_distance_cm"]) / 100.0
            target_y = (
                -float(pnp.get("lateral_ratio", 0.0)) * target_x
                + self.centerline_offset_cm / 100.0
            )
            face_yaw_deg = self.calibrated_visual_yaw(
                float(candidate.get("frontal_error", 0.0))
            )
            self.arc_near_recheck_samples.append(
                (target_x, target_y, face_yaw_deg)
            )
            self.arc_near_recheck_samples = self.arc_near_recheck_samples[-5:]
            if len(self.arc_near_recheck_samples) < 5:
                self.arc_label.setText(
                    f"근거리 중심 표본 {len(self.arc_near_recheck_samples)}/5"
                )
                return
            target_x = sorted(v[0] for v in self.arc_near_recheck_samples)[2]
            target_y = sorted(v[1] for v in self.arc_near_recheck_samples)[2]
            face_yaw_deg = sorted(
                v[2] for v in self.arc_near_recheck_samples
            )[2]
            planned_yaw = math.radians(
                self.arc_yaw_direction_multiplier * face_yaw_deg
            )
            lateral_target = target_y - target_x * math.tan(planned_yaw)
            lateral_target = self.add_directional_overrun(
                lateral_target, self.lateral_overrun_cm / 100.0
            )
            target_yaw = self.add_directional_overrun(
                planned_yaw, math.radians(self.rotation_overrun_deg)
            )
            if abs(lateral_target) > 0.35:
                self.cancel_arc_approach(
                    f"근거리 횡이동 {lateral_target*100:+.1f}cm 상한 초과"
                )
                return
            self.arc_near_center_check_done = True
            self.arc_dock_lateral_target_m = lateral_target
            self.arc_dock_rotation_target_rad = target_yaw
            self.arc_dock_forward_target_m = target_x / max(
                math.cos(planned_yaw), 0.2
            )
            self.arc_dock_lateral_progress_m = 0.0
            self.arc_dock_last_control_at = now
            self.arc_dock_last_lateral_command = 0.0
            self.arc_dock_motion_anchor_yaw = self.node.odom_yaw_unwrapped
            self.arc_dock_yaw_error_before_rotation_deg = abs(face_yaw_deg)
            self.arc_dock_rotation_was_commanded = False
            self.arc_dock_yaw_correction_count = 0
            self.arc_dock_yaw_verify_samples = []
            self.arc_dock_phase = (
                "lateral_once" if abs(lateral_target) > 0.01 else "rotate_once"
            )
            self.arc_phase_started_at = now
            self.node.log_telemetry_event("arc_near_center_replan", {
                "target_forward_cm": target_x * 100.0,
                "target_lateral_left_cm": target_y * 100.0,
                "lateral_command_cm": lateral_target * 100.0,
                "visual_yaw_deg": face_yaw_deg,
                "rotation_command_deg": math.degrees(target_yaw),
                "lateral_overrun_cm": self.lateral_overrun_cm,
                "rotation_overrun_deg": self.rotation_overrun_deg,
            })
            self.arc_label.setText(
                f"근거리 재계산 | 좌 {lateral_target*100:+.1f}cm | "
                f"회전 {math.degrees(target_yaw):+.1f}°"
            )
            return

        if self.arc_dock_phase == "lateral_once":
            target = self.arc_dock_lateral_target_m
            if target is None or self.arc_dock_last_control_at is None:
                self.cancel_arc_approach("횡이동 기준 없음")
                return
            dt = min(max(now - self.arc_dock_last_control_at, 0.0), 0.2)
            self.arc_dock_last_control_at = now
            lateral_scale = self.calibration_scale_for_speed(
                self.lateral_samples,
                "linear_speed_m_s",
                self.arc_dock_last_lateral_command,
                self.lateral_scale,
            )
            self.arc_dock_lateral_progress_m += (
                self.arc_dock_last_lateral_command * lateral_scale * dt
            )
            travelled = self.arc_dock_lateral_progress_m
            remaining = target - travelled
            direction = math.copysign(1.0, target)
            if direction * remaining <= self.LATERAL_STOP_TOLERANCE_M:
                self.arc_dock_last_lateral_command = 0.0
                self.node.stop(repeats=3)
                self.arc_dock_phase = "rotate_once"
                self.arc_phase_started_at = now
                self.arc_dock_motion_anchor_yaw = self.node.odom_yaw_unwrapped
                self.node.log_telemetry_event("arc_alignment_complete", {
                    "mode": "lateral_once",
                    "target_lateral_left_cm": target * 100.0,
                    "travelled_lateral_left_cm": travelled * 100.0,
                    "next_phase": "rotate_once",
                })
                self.arc_label.setText("횡이동 완료 | 저장 yaw로 회전")
                return
            timeout = max(2.0, abs(target) / 0.025 + 1.5)
            if now - self.arc_phase_started_at > timeout:
                self.cancel_arc_approach("횡이동 안전시간 초과")
                return
            # Match manual A/D exactly: publish the current drive slider speed
            # immediately.  Distance control only decides when to release/stop.
            linear_y = direction * (
                self.linear.value() / self.linear.speed_scale
            )
            self.arc_dock_last_lateral_command = linear_y
            self.node.publish(0.0, linear_y, 0.0)
            self.node.log_telemetry_event("arc_control", {
                "phase": "lateral_once",
                "target_lateral_left_cm": target * 100.0,
                "travelled_lateral_left_cm": travelled * 100.0,
                "remaining_cm": remaining * 100.0,
                "command_source": "manual_a_d_speed",
                "cmd_vel": {"linear_x": 0.0, "linear_y": linear_y,
                            "angular_z": 0.0},
            })
            self.arc_label.setText(
                f"횡이동 | 목표 {target*100:+.1f}cm | "
                f"이동 {travelled*100:+.1f}cm"
            )
            return

        if self.arc_dock_phase in ("rotate_once", "rotate_correction_once"):
            rotation_phase = self.arc_dock_phase
            anchor_yaw = self.arc_dock_motion_anchor_yaw
            target = self.arc_dock_rotation_target_rad
            if anchor_yaw is None or target is None:
                self.cancel_arc_approach("회전 기준 없음")
                return
            manual_angular_speed = (
                self.angular.value() / self.angular.speed_scale
            )
            command_angular_speed = manual_angular_speed
            if abs(math.degrees(target)) <= self.small_rotation_threshold_deg:
                command_angular_speed = max(
                    manual_angular_speed, self.small_rotation_speed_rad_s
                )
            rotation_scale = self.calibration_scale_for_speed(
                self.rotation_samples,
                "angular_speed_rad_s",
                command_angular_speed,
                self.odom_rotation_scale(),
            )
            turned = (
                self.node.odom_yaw_unwrapped - anchor_yaw
            ) * rotation_scale
            remaining = target - turned
            direction = math.copysign(1.0, target)
            reached = abs(target) <= math.radians(2.0) or (
                direction * remaining <= math.radians(2.0)
            )
            if reached:
                self.node.stop(repeats=3)
                self.arc_dock_phase = "verify_yaw"
                self.arc_phase_started_at = now
                self.arc_dock_yaw_verify_samples = []
                self.arc_phase_last_detection_id = (
                    self.node.latest_detection or {}
                ).get("source_stamp_ns")
                if self.arc_phase_last_detection_id is None:
                    self.arc_phase_last_detection_id = (
                        self.node.latest_detection_monotonic
                    )
                next_phase = "verify_yaw"
                self.node.log_telemetry_event("arc_alignment_complete", {
                    "mode": rotation_phase,
                    "target_rotation_deg": math.degrees(target),
                    "turned_deg": math.degrees(turned),
                    "next_phase": next_phase,
                })
                self.arc_label.setText(
                    "회전 완료 | 정지 후 새 검출로 yaw 재확인"
                )
                return
            timeout = max(
                2.0,
                abs(target) / max(command_angular_speed * 0.5, 0.05) + 1.5,
            )
            if now - self.arc_phase_started_at > timeout:
                self.cancel_arc_approach("회전 안전시간 초과")
                return
            angular_z = direction * command_angular_speed
            self.arc_dock_rotation_was_commanded = True
            self.node.publish(0.0, 0.0, angular_z)
            self.node.log_telemetry_event("arc_control", {
                "phase": rotation_phase,
                "target_rotation_deg": math.degrees(target),
                "turned_deg": math.degrees(turned),
                "remaining_deg": math.degrees(remaining),
                "small_rotation_boost": (
                    command_angular_speed > manual_angular_speed
                ),
                "cmd_vel": {"linear_x": 0.0, "linear_y": 0.0,
                            "angular_z": angular_z},
            })
            self.arc_label.setText(
                f"회전 | 목표 {math.degrees(target):+.1f}° | "
                f"회전 {math.degrees(turned):+.1f}°"
            )
            return

        if self.arc_dock_phase == "verify_yaw":
            if now - self.arc_phase_started_at < 0.4:
                self.arc_label.setText("정지 안정화 후 yaw 재확인 중")
                return
            detection = self.node.latest_detection or {}
            detection_id = detection.get("source_stamp_ns")
            if detection_id is None:
                detection_id = self.node.latest_detection_monotonic
            if detection_id == self.arc_phase_last_detection_id:
                self.arc_label.setText("정지 후 새 목표 검출 대기 중")
                return
            candidate, _pnp, invalid_reason = self.valid_arc_reobserve_measurement()
            if invalid_reason is not None:
                self.arc_label.setText(f"yaw 재확인 대기 | {invalid_reason}")
                return
            self.arc_phase_last_detection_id = detection_id
            visual_yaw_deg = self.calibrated_visual_yaw(
                float(candidate.get("frontal_error", 0.0))
            )
            self.arc_dock_yaw_verify_samples.append(visual_yaw_deg)
            self.arc_dock_yaw_verify_samples = (
                self.arc_dock_yaw_verify_samples[-5:]
            )
            if len(self.arc_dock_yaw_verify_samples) < 5:
                self.arc_label.setText(
                    "정지 yaw 표본 수집 "
                    f"{len(self.arc_dock_yaw_verify_samples)}/5"
                )
                return
            visual_yaw_deg = sorted(self.arc_dock_yaw_verify_samples)[2]
            yaw_correction_deg = -visual_yaw_deg
            if abs(visual_yaw_deg) < self.ARC_CYCLE_YAW_THRESHOLD_DEG:
                if self.selected_cycle_has_next():
                    visible_margin_m = self.near_center_check_distance_cm / 100.0
                    advance_m = min(
                        self.arc_cycle_advance_m,
                        max(0.0, self.arc_dock_forward_target_m - visible_margin_m),
                    )
                    if advance_m > 0.005:
                        self.arc_dock_phase = "cycle_advance"
                        self.arc_phase_started_at = now
                        self.arc_forward_anchor_position = self.node.odom_position
                        self.arc_forward_anchor_yaw = self.node.odom_yaw_unwrapped
                        self.arc_forward_anchor_remaining_m = advance_m
                        self.arc_label.setText(
                            f"yaw {visual_yaw_deg:+.1f}° | "
                            f"다음 사이클 전 {advance_m*100:.1f}cm 전진"
                        )
                        return
                self.start_verified_holonomic_forward(now, yaw_correction_deg)
                return
            if self.selected_cycle_has_next():
                self.schedule_next_selected_cycle(
                    now, f"yaw {visual_yaw_deg:+.1f}° 재정렬 필요"
                )
                return
            self.cancel_arc_approach(
                f"yaw {visual_yaw_deg:+.1f}° 불일치 | 추가 보정·삽입 안 함"
            )
            return

        if self.arc_dock_phase in (
            "forward", "precheck_forward", "cycle_advance"
        ):
            forward_phase = self.arc_dock_phase
            if (
                self.arc_forward_anchor_position is None
                or self.arc_forward_anchor_yaw is None
                or self.arc_forward_anchor_remaining_m is None
            ):
                self.cancel_arc_approach("직진 odom 기준 없음")
                return
            dx = self.node.odom_position[0] - self.arc_forward_anchor_position[0]
            dy = self.node.odom_position[1] - self.arc_forward_anchor_position[1]
            raw_forward = (
                math.cos(self.arc_forward_anchor_yaw) * dx
                + math.sin(self.arc_forward_anchor_yaw) * dy
            )
            forward_scale = self.calibration_scale_for_speed(
                self.distance_samples,
                "linear_speed_m_s",
                0.05,
                self.distance_scale,
            )
            remaining_m = (
                self.arc_forward_anchor_remaining_m
                - max(0.0, raw_forward) * forward_scale
            )
            remaining_source = "calibrated_odom_once"
            forward_timeout = max(
                3.0, self.arc_forward_anchor_remaining_m / 0.025 + 2.0
            )
            if now - self.arc_phase_started_at > forward_timeout:
                self.cancel_arc_approach("직진 안전시간 초과")
                return
            if remaining_m <= 0.005:
                if forward_phase == "cycle_advance":
                    self.schedule_next_selected_cycle(now, "10cm 접근 완료")
                    self.arc_forward_anchor_position = None
                    self.arc_forward_anchor_yaw = None
                    self.arc_forward_anchor_remaining_m = None
                    return
                if forward_phase == "precheck_forward":
                    self.node.stop(repeats=5)
                    self.arc_dock_phase = "near_center_recheck"
                    self.arc_phase_started_at = now
                    self.arc_phase_last_detection_id = (
                        self.node.latest_detection or {}
                    ).get("source_stamp_ns")
                    if self.arc_phase_last_detection_id is None:
                        self.arc_phase_last_detection_id = (
                            self.node.latest_detection_monotonic
                        )
                    self.arc_near_recheck_samples = []
                    self.arc_forward_anchor_position = None
                    self.arc_forward_anchor_yaw = None
                    self.arc_forward_anchor_remaining_m = None
                    self.arc_label.setText("근거리 도착 | 중심·yaw 재검출")
                    return
                self.arc_active = False
                self.arc_dock_phase = None
                self.node.stop(repeats=5)
                result = {
                    "mode": "lateral_rotate_drive_once",
                    "elapsed_sec": now - self.arc_started_at,
                    "relative_pose": {
                        "x_cm": x * 100.0,
                        "y_left_cm": y * 100.0,
                        "yaw_ccw_deg": math.degrees(relative_yaw),
                    },
                    "remaining_cm": remaining_m * 100.0,
                    "remaining_source": remaining_source,
                    "automatic_pass": self.arc_auto_pass,
                    "initial_plan": self.arc_initial_plan,
                    "final_plan": self.arc_plan,
                }
                self.arc_auto_enabled = False
                self.arc_auto_replan_due_at = None
                self.arc_execute_button.setText("주행 실행")
                self.begin_arc_manual_correction(result)
                self.node.log_telemetry_event("arc_complete", result)
                lifted = self.finish_auto_lift_after_dock()
                self.arc_label.setText(
                    f"횡이동 → 회전 → 직선 삽입 완료 | {self.arc_auto_pass}회"
                    + (" | 리프트 상승 명령" if lifted else "")
                )
                return
            max_linear = min(
                self.linear.value() / self.linear.speed_scale, 0.08
            )
            linear_x = min(max_linear, max(0.04, 0.8 * remaining_m))
            self.node.publish(linear_x, 0.0, 0.0)
            self.node.log_telemetry_event("arc_control", {
                "phase": (
                    "cycle_advance"
                    if forward_phase == "cycle_advance"
                    else (
                        "near_center_approach"
                        if forward_phase == "precheck_forward"
                        else "holonomic_forward"
                    )
                ),
                "elapsed_sec": now - self.arc_started_at,
                "remaining_cm": remaining_m * 100.0,
                "remaining_source": remaining_source,
                "cmd_vel": {"linear_x": linear_x, "linear_y": 0.0,
                            "angular_z": 0.0},
            })
            if forward_phase == "cycle_advance":
                self.arc_label.setText(
                    f"다음 사이클 접근 | 남은 {remaining_m*100:.1f}cm"
                )
            elif forward_phase == "precheck_forward":
                self.arc_label.setText(
                    f"근거리 중심 확인까지 {remaining_m*100:.1f}cm"
                )
            else:
                self.arc_label.setText(
                    f"직선 삽입 | 남은 거리 {remaining_m*100:.1f}cm | "
                    f"{remaining_source}"
                )
            return

        self.cancel_arc_approach("정렬 단계 오류")

    def update_arc_approach(self):
        if not self.arc_active:
            return
        now = time.monotonic()
        one_shot_dock = (
            self.arc_plan.get("controller_mode") == "visual_alternating_dock"
        )
        if now - self.node.last_odom_monotonic > 0.5:
            self.cancel_arc_approach("odom 데이터 지연")
            return
        if not one_shot_dock:
            if now - self.node.last_scan_monotonic > 0.75:
                self.cancel_arc_approach("LiDAR 데이터 지연")
                return
            if (
                self.node.front_range < self.args.stop_distance
                and not self.arc_recovery_active
            ):
                self.cancel_arc_approach("전방 장애물")
                return
        if (
            not one_shot_dock
            and now - self.arc_started_at > 30.0
        ):
            self.cancel_arc_approach("30초 시간 초과")
            return
        position = self.node.odom_position
        yaw = self.node.odom_yaw_unwrapped
        if position is None or yaw is None:
            self.cancel_arc_approach("odom 끊김")
            return
        dx = position[0] - self.arc_start_position[0]
        dy = position[1] - self.arc_start_position[1]
        c0, s0 = math.cos(self.arc_start_yaw), math.sin(self.arc_start_yaw)
        x = c0 * dx + s0 * dy
        y = -s0 * dx + c0 * dy
        relative_yaw = yaw - self.arc_start_yaw
        if one_shot_dock:
            self.update_holonomic_dock(now, x, y, relative_yaw)
            return
        # Execute the snapshot made by ARC path calculation exactly once.
        # Live detections are observation-only and never change the active path.
        _candidate, live_pnp, _detection_error = self.valid_arc_measurement()

        if self.arc_reobserve_active:
            self.node.stop(repeats=1)
            wait_time = now - self.arc_reobserve_started_at
            candidate, pnp, reason = self.valid_arc_reobserve_measurement()
            if wait_time >= 0.4 and reason is None:
                if self.replan_arc_from_live_measurement(candidate, pnp):
                    return
            if wait_time > 3.0:
                self.arc_active = False
                self.arc_reobserve_active = False
                result = {
                    "mode": "variable_curvature_reobserve_failed",
                    "elapsed_sec": time.monotonic() - self.arc_started_at,
                    "relative_pose": {
                        "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                        "yaw_ccw_deg": math.degrees(relative_yaw),
                    },
                    "reobserve_failure": reason,
                    "initial_plan": self.arc_initial_plan,
                    "final_plan": self.arc_plan,
                }
                self.begin_arc_manual_correction(result)
                self.node.log_telemetry_event("arc_complete", result)
                self.arc_label.setText(
                    f"1차 정렬 후 정지 | 재검출 실패: {reason}"
                )
                return
            self.arc_label.setText(
                f"1차 정렬 완료 | 정지 재검출 중 {wait_time:.1f}s"
            )
            return

        # Once aligned, keep that heading, cross the target plane (0 cm), and
        # insert the fork by the configured additional distance.  This phase
        # deliberately applies no yaw correction.
        if self.arc_final_approach_active:
            goal_yaw = self.arc_plan["goal_yaw_rad"]
            target_dx = self.arc_plan["target_x_m"] - x
            target_dy = self.arc_plan["target_y_m"] - y
            forward_remaining = (
                math.cos(goal_yaw) * target_dx + math.sin(goal_yaw) * target_dy
            )
            lateral_remaining = (
                -math.sin(goal_yaw) * target_dx + math.cos(goal_yaw) * target_dy
            )
            insertion_distance = self.arc_plan.get(
                "insertion_distance_m", self.arc_insertion_distance.value() / 100.0
            )
            insertion_remaining = forward_remaining + insertion_distance
            if insertion_remaining <= 0.005:
                self.arc_active = False
                self.arc_final_approach_active = False
                self.node.stop(repeats=5)
                self.arc_label.setText(
                    f"0cm 통과 후 {insertion_distance*100:.1f}cm 삽입 완료 | "
                    f"횡오차 {lateral_remaining*100:+.1f} cm"
                )
                result = {
                    "mode": "iterative_variable_curvature",
                    "passes": self.arc_pass_count + 1,
                    "elapsed_sec": time.monotonic() - self.arc_started_at,
                    "relative_pose": {
                        "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                        "yaw_ccw_deg": math.degrees(relative_yaw),
                    },
                    "forward_remaining_cm": forward_remaining * 100.0,
                    "insertion_distance_cm": insertion_distance * 100.0,
                    "insertion_remaining_cm": insertion_remaining * 100.0,
                    "lateral_remaining_cm": lateral_remaining * 100.0,
                    "camera_verified": live_pnp is not None,
                    "initial_plan": self.arc_initial_plan,
                    "final_plan": self.arc_plan,
                }
                self.begin_arc_manual_correction(result)
                self.node.log_telemetry_event("arc_complete", result)
                self.finish_auto_lift_after_dock()
                return
            max_linear = min(
                self.linear.value() / self.linear.speed_scale, 0.05
            )
            linear_x = min(max_linear, max(0.02, 0.8 * insertion_remaining))
            self.node.publish(linear_x, 0.0, 0.0)
            self.node.log_telemetry_event("arc_control", {
                "phase": "fixed_final_approach",
                "elapsed_sec": time.monotonic() - self.arc_started_at,
                "relative_pose": {
                    "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                    "yaw_ccw_deg": math.degrees(relative_yaw),
                },
                "forward_remaining_cm": forward_remaining * 100.0,
                "insertion_distance_cm": insertion_distance * 100.0,
                "insertion_remaining_cm": insertion_remaining * 100.0,
                "lateral_remaining_cm": lateral_remaining * 100.0,
                "cmd_vel": {
                    "linear_x": linear_x, "linear_y": 0.0, "angular_z": 0.0,
                },
                "front_range_m": self.node.front_range,
            })
            self.arc_label.setText(
                f"정렬 유지 삽입 | 목표면 {forward_remaining*100:+.1f} cm | "
                f"최종까지 {insertion_remaining*100:.1f} cm"
            )
            return

        if self.arc_plan.get("controller_mode") == "variable_curvature_pose":
            candidate, pnp, _reason = self.valid_arc_measurement()
            if candidate is not None and pnp is not None:
                self.fuse_live_arc_target(candidate, pnp)
            virtual = self.virtual_arc_target_relative()
            if virtual is None:
                self.cancel_arc_approach("가상 팔레트 좌표 없음")
                return
            target_x, target_y, target_yaw = virtual
            standoff = self.arc_plan.get(
                "alignment_distance_m", self.arc_standoff.value() / 100.0
            )
            error_x = target_x - standoff * math.cos(target_yaw)
            error_y = target_y - standoff * math.sin(target_yaw)
            position_error = math.hypot(error_x, error_y)
            yaw_error = target_yaw
            if position_error <= 0.025 and abs(yaw_error) <= math.radians(3.0):
                self.node.stop(repeats=2)
                if self.arc_pass_count < 1:
                    self.arc_reobserve_active = True
                    self.arc_reobserve_started_at = now
                    self.node.log_telemetry_event("arc_alignment_complete", {
                        "mode": "variable_curvature_pose",
                        "pass": 1,
                        "elapsed_sec": now - self.arc_started_at,
                        "position_error_cm": position_error * 100.0,
                        "yaw_error_deg": math.degrees(yaw_error),
                    })
                    self.arc_label.setText("1차 정렬 완료 | 카메라 재계산 대기")
                else:
                    self.arc_final_approach_active = True
                    self.node.log_telemetry_event("arc_alignment_complete", {
                        "mode": "variable_curvature_pose",
                        "pass": 2,
                        "elapsed_sec": now - self.arc_started_at,
                        "position_error_cm": position_error * 100.0,
                        "yaw_error_deg": math.degrees(yaw_error),
                    })
                    self.arc_label.setText(
                        f"2차 정렬 완료 | 0cm 통과 후 "
                        f"{self.arc_insertion_distance.value():.1f}cm 삽입 시작"
                    )
                return
            max_linear = min(self.linear.value() / self.linear.speed_scale, 0.12)
            max_angular = min(self.angular.value() / self.angular.speed_scale, 0.55)
            if position_error <= 0.025:
                linear_x = 0.0
                angular_z = max(-max_angular, min(max_angular, 1.8 * yaw_error))
            else:
                # Mecanum visual servo: keep approaching while correcting both
                # lateral error and face yaw from the odom-held pallet pose.
                linear_x = min(max_linear, max(0.0, 0.8 * error_x))
                linear_y = max(-max_linear, min(max_linear, 1.0 * error_y))
                angular_z = max(-max_angular, min(max_angular, 1.8 * yaw_error))
                if position_error < 0.08:
                    linear_x = min(linear_x, 0.04)
                    linear_y = min(max(linear_y, -0.04), 0.04)
            if position_error <= 0.025:
                linear_y = 0.0
            self.node.publish(linear_x, linear_y, angular_z)
            self.node.log_telemetry_event("arc_control", {
                "phase": "variable_curvature_pose",
                "pass": self.arc_pass_count + 1,
                "elapsed_sec": now - self.arc_started_at,
                "relative_pose": {
                    "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                    "yaw_ccw_deg": math.degrees(relative_yaw),
                },
                "position_error_cm": position_error * 100.0,
                "yaw_error_deg": math.degrees(yaw_error),
                "cmd_vel": {
                    "linear_x": linear_x, "linear_y": linear_y,
                    "angular_z": angular_z,
                },
                "front_range_m": self.node.front_range,
            })
            self.arc_label.setText(
                f"가상좌표 동시보정 | 전후 {error_x*100:+.1f} / "
                f"좌 {error_y*100:+.1f} cm / 각 {math.degrees(yaw_error):+.1f}°"
            )
            return

        straight_length = self.arc_plan.get("straight_length_m")
        arc_radius = self.arc_plan.get("arc_radius_m")
        if straight_length is not None and arc_radius is not None:
            goal_yaw = self.arc_plan["goal_yaw_rad"]
            final_x = self.arc_plan["goal_x_m"]
            final_y = self.arc_plan["goal_y_m"]
            final_distance = math.hypot(final_x - x, final_y - y)
            yaw_remaining = math.atan2(
                math.sin(goal_yaw - relative_yaw),
                math.cos(goal_yaw - relative_yaw),
            )
            max_linear = min(self.linear.value() / self.linear.speed_scale, 0.12)
            max_angular = min(self.angular.value() / self.angular.speed_scale, 0.55)
            if x < straight_length - 0.005:
                phase = "fixed_straight"
                remaining = max(straight_length - x, 0.0)
                linear_x = min(max_linear, max(0.03, 1.2 * remaining))
                angular_z = max(-0.15, min(0.15, -1.2 * relative_yaw))
            elif abs(yaw_remaining) > math.radians(2.0):
                phase = "fixed_arc"
                angular_speed = min(
                    max_angular, max(0.08, 1.2 * abs(yaw_remaining))
                )
                linear_x = min(max_linear, abs(arc_radius) * angular_speed)
                angular_z = math.copysign(linear_x / abs(arc_radius), arc_radius)
            else:
                self.arc_final_approach_active = True
                self.node.stop(repeats=1)
                self.arc_label.setText(
                    f"정렬 완료 | 위치오차 {final_distance*100:.1f} cm | 0cm 직진 시작"
                )
                self.node.log_telemetry_event("arc_alignment_complete", {
                    "mode": "fixed_straight_arc",
                    "elapsed_sec": time.monotonic() - self.arc_started_at,
                    "relative_pose": {
                        "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                        "yaw_ccw_deg": math.degrees(relative_yaw),
                    },
                    "final_position_error_cm": final_distance * 100.0,
                    "final_yaw_error_deg": math.degrees(yaw_remaining),
                    "alignment_distance_cm": self.arc_plan.get(
                        "alignment_distance_m", self.arc_standoff.value() / 100.0
                    ) * 100.0,
                })
                return
            self.node.publish(linear_x, 0.0, angular_z)
            self.node.log_telemetry_event("arc_control", {
                "phase": phase,
                "elapsed_sec": time.monotonic() - self.arc_started_at,
                "relative_pose": {
                    "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                    "yaw_ccw_deg": math.degrees(relative_yaw),
                },
                "straight_length_cm": straight_length * 100.0,
                "arc_radius_cm": arc_radius * 100.0,
                "yaw_remaining_deg": math.degrees(yaw_remaining),
                "cmd_vel": {
                    "linear_x": linear_x, "linear_y": 0.0,
                    "angular_z": angular_z,
                },
                "final_position_error_cm": final_distance * 100.0,
                "front_range_m": self.node.front_range,
            })
            self.arc_label.setText(
                f"계획 실행: {'직진' if phase == 'fixed_straight' else '원호'} | "
                f"위치오차 {final_distance*100:.1f} cm / "
                f"각 {math.degrees(yaw_remaining):+.1f}°"
            )
            return

        waypoints = self.arc_plan["waypoints"]
        search_start = max(0, self.arc_waypoint_index - 1)
        search_stop = min(len(waypoints), self.arc_waypoint_index + 9)
        nearest = min(
            range(search_start, search_stop),
            key=lambda index: math.hypot(waypoints[index][0] - x, waypoints[index][1] - y),
        )
        self.arc_waypoint_index = max(self.arc_waypoint_index, nearest)
        lookahead_index = self.arc_waypoint_index
        lookahead_distance = 0.0
        while lookahead_index < len(waypoints) - 1 and lookahead_distance < 0.10:
            current = waypoints[lookahead_index]
            following = waypoints[lookahead_index + 1]
            lookahead_distance += math.hypot(
                following[0] - current[0], following[1] - current[1]
            )
            lookahead_index += 1
        target_x, target_y, _target_yaw = waypoints[lookahead_index]
        error_x = target_x - x
        error_y = target_y - y
        final_x, final_y, _path_final_yaw = waypoints[-1]
        final_yaw = self.arc_plan["goal_yaw_rad"]
        final_distance = math.hypot(final_x - x, final_y - y)
        final_yaw_error = math.atan2(
            math.sin(final_yaw - relative_yaw), math.cos(final_yaw - relative_yaw)
        )
        if final_distance < 0.025 and abs(final_yaw_error) < math.radians(4.0):
            self.arc_final_approach_active = True
            self.node.stop(repeats=1)
            self.arc_label.setText("정렬 완료 | 0cm 직진 시작")
            self.node.log_telemetry_event("arc_alignment_complete", {
                "elapsed_sec": time.monotonic() - self.arc_started_at,
                "relative_pose": {
                    "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                    "yaw_ccw_deg": math.degrees(relative_yaw),
                },
                "final_position_error_cm": final_distance * 100.0,
                "final_yaw_error_deg": math.degrees(final_yaw_error),
            })
            return
        c, s = math.cos(relative_yaw), math.sin(relative_yaw)
        body_x = c * error_x + s * error_y
        body_y = -s * error_x + c * error_y
        max_linear = min(self.linear.value() / self.linear.speed_scale, 0.12)
        max_angular = min(self.angular.value() / self.angular.speed_scale, 0.55)
        if final_distance < 0.025:
            linear_x = 0.0
            angular_z = max(-max_angular, min(max_angular, 1.8 * final_yaw_error))
        else:
            lookahead_sq = max(body_x * body_x + body_y * body_y, 0.0025)
            curvature = 2.0 * body_y / lookahead_sq
            linear_x = min(max_linear, max(0.035, 0.9 * final_distance))
            if abs(math.atan2(body_y, max(body_x, 0.01))) > math.radians(55.0):
                linear_x = min(linear_x, 0.04)
            angular_z = linear_x * curvature
            angular_z = max(-max_angular, min(max_angular, angular_z))
        # ARC mode is non-holonomic: it never commands mecanum lateral motion.
        self.node.publish(linear_x, 0.0, angular_z)
        self.node.log_telemetry_event("arc_control", {
            "elapsed_sec": time.monotonic() - self.arc_started_at,
            "waypoint_index": self.arc_waypoint_index,
            "lookahead_index": lookahead_index,
            "relative_pose": {
                "x_cm": x * 100.0, "y_left_cm": y * 100.0,
                "yaw_ccw_deg": math.degrees(relative_yaw),
            },
            "cmd_vel": {"linear_x": linear_x, "linear_y": 0.0, "angular_z": angular_z},
            "final_position_error_cm": final_distance * 100.0,
            "final_yaw_error_deg": math.degrees(final_yaw_error),
            "front_range_m": self.node.front_range,
        })
        self.arc_label.setText(
            f"ARC 실행 중 {lookahead_index}/{len(waypoints)-1} | "
            f"남은 위치오차 {final_distance*100:.1f} cm / 각 {math.degrees(final_yaw_error):+.1f}°"
        )

    def emergency_stop(self):
        self.arc_active = False
        self.target_search_active = False
        self.arc_auto_enabled = False
        self.arc_auto_internal_start = False
        self.arc_auto_replan_due_at = None
        self.arc_cycle_replan_due_at = None
        self.auto_lift_after_dock = False
        self.pressed.clear()
        self.node.stop(repeats=5)
        self.node.publish_fork("STOP")
        if not self.args.http_viewer_only:
            self.node.publish_auto_dock_stop()
        self.status.setText("EMERGENCY STOP")

    def command(self):
        linear_speed = self.linear.value() / self.linear.speed_scale
        angular_speed = self.angular.value() / self.angular.speed_scale
        linear_x = linear_speed * ((Qt.Key_W in self.pressed) - (Qt.Key_S in self.pressed))
        left_right = (Qt.Key_A in self.pressed) - (Qt.Key_D in self.pressed)
        rotate = (Qt.Key_Q in self.pressed) - (Qt.Key_E in self.pressed)
        if self.drive_mode.currentData() == "mecanum":
            return linear_x, linear_speed * left_right, angular_speed * rotate
        angular_z = angular_speed * rotate
        if rotate == 0 and linear_x != 0.0:
            angular_z = angular_speed * left_right
        return linear_x, 0.0, angular_z

    def tick(self):
        if not self.args.http_viewer_only:
            if not rclpy.ok():
                return
            for _ in range(20):
                rclpy.spin_once(self.node, timeout_sec=0.0)
        camera_age = time.monotonic() - self.node.last_frame_monotonic
        camera_ok = self.node.frame is not None and camera_age <= 0.75
        linear_x, linear_y, angular_z = self.command()
        manual_drive_active = bool(self.pressed & self.MOVEMENT_KEYS)
        blocked = linear_x > 0.0 and self.node.front_range < self.args.stop_distance
        autonomous_active = (
            self.arc_active
            or self.target_search_active
            or self.arc_cycle_replan_due_at is not None
            or self.arc_auto_replan_due_at is not None
        )
        if autonomous_active and self.node.front_range < self.args.stop_distance:
            self.cancel_arc_approach("LiDAR interrupt: 안전거리 침범")
        elif manual_drive_active and self.arc_active:
            self.cancel_arc_approach("수동 조작 전환")
        elif manual_drive_active and self.target_search_active:
            self.cancel_arc_approach("수동 조작으로 목표 탐색 취소")
        elif manual_drive_active and self.arc_cycle_replan_due_at is not None:
            self.arc_cycle_replan_due_at = None
            self.node.stop(repeats=3)
            self.arc_label.setText("수동 조작으로 사이클 반복 취소")
        # Manual teleop must remain independent of a delayed/dropped video
        # frame. When no drive key is held, leave the command channel free for
        # the vehicle-side aligner instead of continuously publishing zeros.
        if manual_drive_active and not blocked:
            self.node.publish(linear_x, linear_y, angular_z)
        elif manual_drive_active:
            self.node.stop()
        elif self.arc_active:
            try:
                self.update_arc_approach()
            except Exception as exc:
                self.cancel_arc_approach(f"제어 예외: {exc}")
        elif self.arc_cycle_replan_due_at is not None:
            try:
                self.update_selected_cycle_replan(time.monotonic())
            except Exception as exc:
                self.cancel_arc_approach(f"사이클 재계산 예외: {exc}")
        elif self.target_search_active:
            try:
                self.update_target_search()
            except Exception as exc:
                self.cancel_arc_approach(f"목표 탐색 예외: {exc}")
        elif self.arc_auto_replan_due_at is not None:
            try:
                self.update_holonomic_auto_cycle(time.monotonic())
            except Exception as exc:
                self.cancel_arc_approach(f"자동 재계산 예외: {exc}")
        state = "READY" if camera_ok else "TELEOP READY | CAMERA STALE"
        if not camera_ok:
            state += " (video only)"
        if blocked:
            state += f" | FORWARD BLOCKED: {self.node.front_range:.2f} m"
        else:
            state += f" | front: {self.node.front_range:.2f} m"
        active = manual_drive_active and not blocked
        now = time.monotonic()
        dt = min(max(now - self.last_command_integral_at, 0.0), 0.2)
        self.last_command_integral_at = now
        if active:
            self.command_forward_m += linear_x * dt
            self.command_lateral_m += linear_y * dt
            self.command_rotation_rad += angular_z * dt
            self.command_forward_abs_m += abs(linear_x) * dt
            self.command_lateral_abs_m += abs(linear_y) * dt
            self.command_rotation_abs_rad += abs(angular_z) * dt
        if self.mapping_active:
            key_names = {
                Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S",
                Qt.Key_D: "D", Qt.Key_Q: "Q", Qt.Key_E: "E",
            }
            self.mapping_input_events.append({
                "t_sec": now - self.mapping_started_at,
                "keys": sorted(
                    key_names[key] for key in self.pressed if key in key_names
                ),
                "blocked": blocked,
                "cmd_vel": {
                    "linear_x": linear_x if active else 0.0,
                    "linear_y": linear_y if active else 0.0,
                    "angular_z": angular_z if active else 0.0,
                },
                "calibrated_motion": self.mapping_motion(),
            })
        if self.last_arc_result is not None and self.arc_correction_started_at is not None:
            key_names = {
                Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S",
                Qt.Key_D: "D", Qt.Key_Q: "Q", Qt.Key_E: "E",
            }
            self.arc_correction_input_events.append({
                "t_sec": now - self.arc_correction_started_at,
                "keys": sorted(
                    key_names[key] for key in self.pressed if key in key_names
                ),
                "blocked": blocked,
                "cmd_vel": {
                    "linear_x": linear_x if active else 0.0,
                    "linear_y": linear_y if active else 0.0,
                    "angular_z": angular_z if active else 0.0,
                },
                "calibrated_motion": self.arc_correction_motion(),
            })
            correction = self.arc_correction_motion()
            self.arc_sample_label.setText(
                f"보정 기록 중 | 전후 {correction['forward_cm']:+.1f} cm | "
                f"좌우(왼+) {correction['lateral_left_cm']:+.1f} cm | "
                "목표 도착 후 샘플 저장"
            )
        mode = "MECANUM" if self.drive_mode.currentData() == "mecanum" else "CAR-LIKE"
        state += (f" | {mode} | cmd x={linear_x if active else 0.0:+.2f}, "
                  f"y={linear_y if active else 0.0:+.2f}, z={angular_z if active else 0.0:+.2f}")
        self.status.setText(state)
        if not self.args.http_viewer_only:
            auto_status = self.node.auto_dock_status
            self.auto_dock_status_label.setText(
                f"1.2 상태: {auto_status.get('state', '?')} | "
                f"{auto_status.get('reason', '?')}"
            )
        self.update_battery_status()
        self.update_rotation_estimate()
        self.update_mapping_label()
        self.update_entity_map()
        self.update_frame()

    def update_entity_map(self):
        sequences = (
            getattr(self.node, "map_sequence", 0),
            getattr(self.node, "entity_map_sequence", 0),
        )
        if sequences == self.last_map_sequence:
            return
        self.last_map_sequence = sequences
        self.entity_map_view.set_data(
            getattr(self.node, "map_image", None),
            getattr(self.node, "map_metadata", None),
            getattr(self.node, "entity_map", None),
        )

    def update_frame(self):
        sequences = (
            self.node.frame_sequence,
            self.node.secondary_frame_sequence,
            self.node.tertiary_frame_sequence,
        )
        if sequences == self.last_displayed_sequence:
            return
        frame = None if self.node.frame is None else self.node.frame.copy()
        secondary = None if self.node.secondary_frame is None else self.node.secondary_frame.copy()
        tertiary = None if self.node.tertiary_frame is None else self.node.tertiary_frame.copy()
        self.last_displayed_sequence = sequences
        if frame is not None and self.writer is not None:
            recording_frame = self.compose_recording_frame(frame, secondary, tertiary)
            self.writer.write(recording_frame)
            self.recorded_frames += 1
            self.record_label.setText(f"REC {self.recorded_frames} frames → {self.record_path}")
        self.show_frame(self.render_detection_overlay(frame), self.video)

    def render_detection_overlay(self, frame):
        """Draw the existing detection JSON over the original camera frame."""
        if frame is None:
            return None
        detection = self.node.latest_detection or {}
        if time.monotonic() - self.node.latest_detection_monotonic > 1.0:
            return frame
        for item in detection.get("detections", []):
            box = item.get("box") or []
            if len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            label = str(item.get("class", "?"))
            confidence = float(item.get("confidence", 0.0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(
                frame, f"{label} {confidence:.2f}", (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 0), 2,
            )
        candidate = detection.get("candidate") or {}
        for point in candidate.get("tag_centers", []):
            if len(point) == 2:
                cv2.circle(
                    frame, (int(round(point[0])), int(round(point[1]))),
                    5, (0, 255, 255), -1,
                )
        return frame

    @staticmethod
    def show_frame(frame, label):
        if frame is None:
            return
        height, width = frame.shape[:2]
        scale = min(label.width() / width, label.height() / height, 1.0)
        if scale < 1.0:
            frame = cv2.resize(
                frame,
                (max(1, round(width * scale)), max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format_RGB888).copy()
        label.set_frame(image)

    def compose_recording_frame(self, primary, secondary, tertiary):
        if self.args.disable_external_webcams:
            return primary
        height, width = primary.shape[:2]
        frames = [primary]
        for name, external in (("WEBCAM 1", secondary), ("WEBCAM 2", tertiary)):
            if external is None:
                external = np.zeros_like(primary)
                cv2.putText(external, f"{name} OFFLINE", (20, height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                external = cv2.resize(external, (width, height), interpolation=cv2.INTER_AREA)
            frames.append(external)
        return np.hstack(frames)

    def capture_snapshot(self):
        if self.node.frame is None:
            self.capture_label.setText("실패: 차량 카메라 화면 없음")
            return

        capture_dir = Path(self.args.output_dir) / "captures"
        capture_dir.mkdir(parents=True, exist_ok=True)
        captured_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        stem = time.strftime("capture_%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        image_path = capture_dir / f"{stem}.jpg"
        metadata_path = capture_dir / f"{stem}.json"
        frame = self.node.frame.copy()

        if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            self.capture_label.setText("실패: 이미지 저장 오류")
            return

        metadata = {
            "vehicle": self.args.vehicle,
            "load_state": self.load_state,
            "calibration_preset": self.active_calibration_preset,
            "captured_at": captured_at,
            "image": image_path.name,
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "target_top": [self.target_left.currentData(), self.target_right.currentData()],
            "ground_truth": {
                "forward_cm": self.truth_forward.value(),
                "lateral_right_cm": self.truth_lateral.value(),
                "target_face_yaw_deg": self.truth_yaw.value(),
                "yaw_source": self.truth_yaw_source,
            },
            "rotation_measurement": {
                "vehicle_ccw_deg": self.current_vehicle_rotation_deg(),
                "odom_scale": self.rotation_scale,
                "calibration_samples": len(self.rotation_samples),
            },
            "drive_measurement": {
                **(self.current_drive_measurement() or {
                    "path_cm": None,
                    "forward_cm": None,
                    "lateral_right_cm": None,
                }),
                "distance_coefficient": self.distance_scale,
                "calibration_samples": len(self.distance_samples),
                "source": "calibrated_command_integrated_odom",
            },
            "pose_config": {
                "camera_pitch_deg": self.args.camera_pitch_deg,
                "friction_coefficient": self.rotation_scale,
                "distance_coefficient": self.distance_scale,
            },
            "detection": self.node.latest_detection,
            "visual_yaw_from_tag_heights_deg": (
                None
                if not (self.node.latest_detection or {}).get("candidate")
                else self.calibrated_visual_yaw(
                    self.node.latest_detection["candidate"].get("frontal_error", 0.0)
                )
            ),
            "note": self.memo.toPlainText().strip(),
        }
        try:
            metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self.capture_label.setText(f"이미지만 저장됨: {exc}")
            return

        self.memo.clear()
        self.capture_label.setText(f"저장됨: {image_path.name} + JSON")

    @staticmethod
    def angle_delta_deg(value, reference):
        difference = math.radians(value - reference)
        return math.degrees(math.atan2(math.sin(difference), math.cos(difference)))

    def current_pnp_pose(self):
        if time.monotonic() - self.node.latest_detection_monotonic > 1.0:
            return None
        result = self.node.latest_detection or {}
        candidate = result.get("candidate") or {}
        return candidate.get("pnp")

    def refresh_camera_calibration_label(self, prefix=""):
        count = len(self.camera_calibration_samples)
        if count >= 2 and self.camera_distance_scale is not None:
            detail = (
                f"피치 {self.args.camera_pitch_deg:+.2f}° | "
                f"yaw 0점 {self.camera_yaw_bias:+.2f}° | "
                f"거리={self.camera_distance_scale:.3f}×PnP"
                f"{self.camera_distance_offset:+.1f} cm"
            )
            self.camera_calibration_capture.setText("새 Calibration 시작")
        else:
            detail = "목표물을 화면 중앙·정면에 두고 거리 입력"
            self.camera_calibration_capture.setText(f"샘플 {count + 1} 저장")
        self.camera_calibration_label.setText(
            f"{prefix + ' | ' if prefix else ''}{min(count, 2)}/2 | {detail}"
        )

    def reset_camera_calibration(self):
        self.camera_calibration_dir = None
        self.camera_calibration_samples = []
        self.camera_calibration_distance.setValue(50.0)
        self.refresh_camera_calibration_label("새 2점 교정")

    def save_camera_calibration_sample(self):
        if len(self.camera_calibration_samples) >= 2:
            self.reset_camera_calibration()
        pose = self.current_pnp_pose()
        if self.node.frame is None or pose is None:
            self.refresh_camera_calibration_label("실패: 최신 목표 PnP 없음")
            return
        required = ("translation_y_units", "translation_z_units", "raw_camera_pitch_deg")
        if any(key not in pose for key in required):
            self.refresh_camera_calibration_label("실패: YOLO를 재시작하세요")
            return

        if self.camera_calibration_dir is None:
            root = Path(self.args.output_dir) / "camera_calibration"
            self.camera_calibration_dir = root / time.strftime(
                "camera_calibration_%Y%m%d_%H%M%S"
            )
            self.camera_calibration_dir.mkdir(parents=True, exist_ok=True)

        sample_number = len(self.camera_calibration_samples) + 1
        sample_name = f"sample_{sample_number:03d}"
        image_path = self.camera_calibration_dir / f"{sample_name}.jpg"
        frame = self.node.frame.copy()
        if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            self.refresh_camera_calibration_label("실패: 이미지 저장 오류")
            return

        record = {
            "sample": sample_number,
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "image": image_path.name,
            "fork_tip_to_target_cm": self.camera_calibration_distance.value(),
            "translation_x_units": float(pose.get("translation_x_units", 0.0)),
            "translation_y_units": float(pose["translation_y_units"]),
            "translation_z_units": float(pose["translation_z_units"]),
            "raw_camera_pitch_deg": float(pose["raw_camera_pitch_deg"]),
            "yaw_before_bias_deg": float(
                pose.get("yaw_before_bias_deg", pose["yaw_deg"] + pose.get("yaw_bias_deg", 0.0))
            ),
            "reprojection_error_px": float(pose.get("reprojection_error_px", 0.0)),
            "target_top": [self.target_left.currentData(), self.target_right.currentData()],
            "note": self.memo.toPlainText().strip(),
        }
        self.camera_calibration_samples.append(record)
        with (self.camera_calibration_dir / "samples.jsonl").open(
            "a", encoding="utf-8"
        ) as output:
            output.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.memo.clear()
        if len(self.camera_calibration_samples) == 1:
            self.camera_calibration_distance.setValue(
                min(1000.0, record["fork_tip_to_target_cm"] + 50.0)
            )
            self.refresh_camera_calibration_label(
                f"1번 {record['fork_tip_to_target_cm']:.1f} cm 저장"
            )
            return
        self.finish_camera_calibration()

    def finish_camera_calibration(self):
        first, second = self.camera_calibration_samples
        distance_delta = (
            second["fork_tip_to_target_cm"] - first["fork_tip_to_target_cm"]
        )
        if abs(distance_delta) < 5.0:
            self.camera_calibration_samples.pop()
            self.refresh_camera_calibration_label("실패: 두 거리를 5 cm 이상 다르게 입력")
            return
        pitch = first["raw_camera_pitch_deg"] + 0.5 * self.angle_delta_deg(
            second["raw_camera_pitch_deg"], first["raw_camera_pitch_deg"]
        )
        yaw_bias = first["yaw_before_bias_deg"] + 0.5 * self.angle_delta_deg(
            second["yaw_before_bias_deg"], first["yaw_before_bias_deg"]
        )
        pitch_rad = math.radians(pitch)
        for sample in self.camera_calibration_samples:
            sample["pnp_forward_units"] = (
                math.sin(pitch_rad) * sample["translation_y_units"]
                + math.cos(pitch_rad) * sample["translation_z_units"]
            )
        pnp_delta = (
            second["pnp_forward_units"] - first["pnp_forward_units"]
        )
        if abs(pnp_delta) < 1e-4:
            self.camera_calibration_samples.pop()
            self.refresh_camera_calibration_label("실패: 두 영상의 PnP 거리 차이가 너무 작음")
            return
        scale = distance_delta / pnp_delta
        if scale <= 0.0:
            self.camera_calibration_samples.pop()
            self.refresh_camera_calibration_label("실패: 가까운/먼 거리 순서와 입력값 확인")
            return
        offset = first["fork_tip_to_target_cm"] - scale * first["pnp_forward_units"]
        self.args.camera_pitch_deg = pitch
        self.camera_distance_scale = scale
        self.camera_distance_offset = offset
        self.camera_yaw_bias = yaw_bias
        try:
            self.persist_rotation_calibration()
            summary = {
                "camera_pitch_deg": pitch,
                "camera_distance_scale_cm_per_pnp_unit": scale,
                "camera_distance_offset_cm": offset,
                "yaw_bias_deg": yaw_bias,
                "distance_reference": "fork tip to target front face",
                "samples": self.camera_calibration_samples,
            }
            (self.camera_calibration_dir / "calibration.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            self.refresh_camera_calibration_label(f"저장 실패: {exc}")
            return
        self.node.update_pose_config(pitch, scale, offset, yaw_bias)
        self.refresh_camera_calibration_label("저장 및 YOLO 적용 완료")

    def toggle_recording(self, enabled):
        if not enabled:
            self.finish_recording()
            return
        if self.node.frame is None:
            self.record.setChecked(False)
            self.record_label.setText("Cannot record: no camera frame")
            return
        output = Path(self.args.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        self.record_path = output / time.strftime("teleop_%Y%m%d_%H%M%S.mp4")
        sample = self.compose_recording_frame(
            self.node.frame, self.node.secondary_frame, self.node.tertiary_frame
        )
        height, width = sample.shape[:2]
        self.writer = cv2.VideoWriter(
            str(self.record_path), cv2.VideoWriter_fourcc(*"mp4v"), self.args.record_fps, (width, height)
        )
        if not self.writer.isOpened():
            self.writer = None
            self.record.setChecked(False)
            self.record_label.setText("Failed to open video writer")
            return
        self.recorded_frames = 0
        self.telemetry_session = self.record_path.stem
        self.node.start_telemetry(self.telemetry_session)
        self.record.setText("Stop recording (R)")

    def finish_recording(self):
        telemetry_session = self.telemetry_session
        telemetry_path = None if self.record_path is None else self.record_path.with_suffix(".jsonl")
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        telemetry_ok = False
        if telemetry_session is not None and telemetry_path is not None:
            telemetry_ok = self.node.finish_telemetry(telemetry_session, telemetry_path)
            self.telemetry_session = None
        self.record.setText("Start recording (R)")
        if self.record_path is not None:
            suffix = f" + {telemetry_path.name}" if telemetry_ok else " (telemetry download failed)"
            self.record_label.setText(f"Saved: {self.record_path}{suffix}")

    def closeEvent(self, event):
        self.pressed.clear()
        self.node.stop(repeats=5)
        self.node.publish_fork("STOP")
        self.finish_recording()
        event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=int, choices=(1, 2), required=True)
    parser.add_argument(
        "--ros-domain-id", type=int, default=None,
        help="ROS domain to use; defaults to the selected vehicle number.",
    )
    parser.add_argument("--webcam-ip", default="210.220.0.12")
    parser.add_argument("--image-topic", default="")
    parser.add_argument("--secondary-image-topic", default="")
    parser.add_argument("--secondary-video-url", default="")
    parser.add_argument("--primary-video-url", default="")
    parser.add_argument("--primary-video-command", default="")
    parser.add_argument("--control-host", default="")
    parser.add_argument("--control-port", type=int, default=8091)
    parser.add_argument("--control-url", default="")
    parser.add_argument("--control-command", default="")
    parser.add_argument("--webcam-1-video-url", default="")
    parser.add_argument("--webcam-2-video-url", default="")
    parser.add_argument("--scan-topic", default="")
    parser.add_argument("--odom-topic", default="")
    parser.add_argument("--motor-command-topic", default="")
    parser.add_argument("--battery-topic", default="")
    parser.add_argument("--detection-topic", default="")
    parser.add_argument("--cmd-vel-topic", default="")
    parser.add_argument("--fork-command-topic", default="")
    parser.add_argument("--arrival-topic", default="")
    parser.add_argument("--auto-dock-stop-topic", default="")
    parser.add_argument("--auto-dock-status-topic", default="")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--entity-map-topic", default="")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--linear-speed", type=float, default=0.05)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--friction-coefficient", type=float, default=1.0)
    parser.add_argument("--pose-config", default="")
    parser.add_argument("--stop-distance", type=float, default=0.20)
    parser.add_argument("--safety-min-valid-range", type=float, default=0.25)
    parser.add_argument("--record-fps", type=float, default=15.0)
    parser.add_argument("--viewer-only", action="store_true")
    webcam_group = parser.add_mutually_exclusive_group()
    webcam_group.add_argument(
        "--disable-external-webcams",
        action="store_true",
        dest="disable_external_webcams",
    )
    webcam_group.add_argument(
        "--enable-external-webcams",
        action="store_false",
        dest="disable_external_webcams",
    )
    parser.set_defaults(disable_external_webcams=None)
    parser.add_argument("--http-viewer-only", action="store_true")
    args = parser.parse_args()
    explicit_image_topic = bool(args.image_topic)
    robot_namespace = f"/robot_{args.vehicle}"
    if not args.image_topic:
        args.image_topic = f"{robot_namespace}/ascamera/camera_publisher/rgb0/image"
    if not args.scan_topic:
        args.scan_topic = "/scan_raw"
    if not args.odom_topic:
        args.odom_topic = "/odom_raw"
    if not args.motor_command_topic:
        args.motor_command_topic = "/ros_robot_controller/set_motor"
    if not args.battery_topic:
        args.battery_topic = "/ros_robot_controller/battery"
    if not args.detection_topic:
        args.detection_topic = f"{robot_namespace}/symbol_seg/detections"
    if not args.cmd_vel_topic:
        args.cmd_vel_topic = "/controller/cmd_vel"
    if not args.fork_command_topic:
        args.fork_command_topic = "/fork/command"
    if not args.arrival_topic:
        args.arrival_topic = f"{robot_namespace}/nav2/arrival"
    if not args.auto_dock_stop_topic:
        args.auto_dock_stop_topic = f"{robot_namespace}/auto_dock/stop"
    if not args.auto_dock_status_topic:
        args.auto_dock_status_topic = f"{robot_namespace}/auto_dock/status"
    if not args.entity_map_topic:
        args.entity_map_topic = f"{robot_namespace}/tag_entity_map"
    if args.secondary_image_topic and (args.secondary_video_url or args.webcam_1_video_url):
        parser.error("use only one of --secondary-image-topic and --secondary-video-url")
    if args.secondary_video_url and args.webcam_1_video_url:
        parser.error("use only one of --secondary-video-url and --webcam-1-video-url")
    if args.secondary_video_url:
        args.webcam_1_video_url = args.secondary_video_url
    if not args.secondary_image_topic and not args.webcam_1_video_url:
        args.webcam_1_video_url = f"http://{args.webcam_ip}:5000/video/0"
    if not args.webcam_2_video_url:
        args.webcam_2_video_url = f"http://{args.webcam_ip}:5000/video/2"
    if not args.primary_video_url and not explicit_image_topic:
        args.primary_video_url = f"http://210.220.0.{200 + args.vehicle}:8090/stream"
    if not args.control_host:
        args.control_host = f"210.220.0.{200 + args.vehicle}"
    if not args.output_dir:
        args.output_dir = str(Path.home() / "recordings" / f"vehicle{args.vehicle}")
    if not args.pose_config:
        candidates = (
            Path("/shared/vehicle_pose_config.json"),
            Path(__file__).resolve().parent.parent / "config" / "vehicle_pose_config.json",
        )
        args.pose_config = str(next(
            (path for path in candidates if path.exists()),
            Path(args.output_dir) / "vehicle_pose_config.json",
        ))
    pose_config_path = Path(args.pose_config)
    if pose_config_path.exists():
        try:
            pose_config = json.loads(pose_config_path.read_text(encoding="utf-8"))
            args.camera_pitch_deg = float(
                pose_config.get("camera_pitch_deg", args.camera_pitch_deg)
            )
            args.friction_coefficient = float(
                pose_config.get("friction_coefficient", args.friction_coefficient)
            )
            if args.disable_external_webcams is None:
                args.disable_external_webcams = bool(
                    pose_config.get("disable_external_webcams", True)
                )
        except (OSError, TypeError, ValueError):
            parser.error(f"invalid pose config: {pose_config_path}")
    if args.disable_external_webcams is None:
        args.disable_external_webcams = True

    args.ros_domain_id = (
        214 + args.vehicle if args.ros_domain_id is None else args.ros_domain_id
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    # Vehicle bringup currently uses normal Fast DDS discovery. Inheriting an
    # old/dead discovery-server setting isolates this GUI from every topic.
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    os.environ.pop("CYCLONEDDS_URI", None)

    if args.http_viewer_only:
        node = HttpViewerSource(args)
    else:
        rclpy.init()
        node = DevControlClientNode(args)
    app = QApplication([])
    window = TeleopWindow(node, args)
    app.installEventFilter(window)
    window.show()
    try:
        app.exec_()
    finally:
        node.secondary_stream_stop.set()
        for process in getattr(node, "child_processes", []):
            if process.poll() is None:
                process.terminate()
        for thread in node.external_stream_threads:
            thread.join(timeout=1.0)
        node.stop(repeats=5)
        node.publish_fork("STOP")
        if not args.http_viewer_only:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
