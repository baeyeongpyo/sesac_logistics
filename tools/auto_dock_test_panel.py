#!/usr/bin/env python3
"""Manual ROS event panel for real-vehicle auto-dock integration tests.

This tool injects Nav2/Fork events, shows Auto Dock/YOLO state, and provides
hold-to-drive WASD/QE manual control for real-vehicle tests.
"""

import argparse
import json
import math
import os
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String


SYMBOLS = ("spade", "heart", "clover", "diamond", "star")
LOCATIONS = ("DOCK_1", "NORMAL", "FRESH", "Y1", "Y2", "Y3", "Y4")
TUNING_FIELDS = (
    ("stable_detection_frames", "확정 프레임", 2, int, 1, 30),
    ("candidate_stop_delay_sec", "추가 이동(초)", 0.2, float, 0.0, 5.0),
    ("candidate_confirmation_timeout_sec", "확인 제한(초)", 0.8, float, 0.1, 10.0),
    ("candidate_retry_cooldown_sec", "재탐색 이동(초)", 1.0, float, 0.0, 10.0),
    ("tape_guidance_enabled", "주의테이프 추종(1/0)", 1, int, 0, 1),
    ("tape_roi_top_ratio", "테이프 ROI 시작(비율)", 0.55, float, 0.30, 0.90),
    ("tape_min_yellow_pixels", "테이프 최소 노랑픽셀", 600, int, 100, 20000),
    ("tape_max_age_sec", "테이프 유효시간(초)", 0.50, float, 0.10, 3.0),
    ("tape_forward_gain", "테이프 전후보정 이득", 0.10, float, 0.0, 1.0),
    ("tape_max_forward_speed_m_s", "테이프 전후속도 제한", 0.03, float, 0.0, 0.10),
    ("tape_yaw_gain", "테이프 회전보정 이득", 0.80, float, 0.0, 3.0),
    ("tape_max_yaw_speed_rad_s", "테이프 회전속도 제한", 0.20, float, 0.0, 0.50),
    ("max_pnp_reprojection_error_px", "PnP 유효오차(px)", 3.0, float, 0.1, 100.0),
    ("max_frontal_error", "정면 오차", 0.35, float, 0.01, 2.0),
    ("coarse_center_error_max", "중앙 허용(비율)", 0.10, float, 0.01, 0.50),
    ("centerline_offset_cm", "중심 보정(cm, +좌/-우)", 0.0, float, -30.0, 30.0),
    ("depth_fallback_confirmation_frames", "Depth 확정 프레임", 3, int, 1, 30),
    ("coarse_alignment_timeout_sec", "가정렬 제한(초)", 5.0, float, 0.5, 30.0),
    ("insertion_distance_cm", "팔레트 내부 삽입 깊이(cm)", 15.0, float, 1.0, 100.0),
    ("insertion_lateral_gain", "진입 횡보정 이득", 0.50, float, 0.0, 2.0),
    ("insertion_max_lateral_speed_m_s", "진입 횡속도 제한", 0.025, float, 0.0, 0.08),
    ("insertion_yaw_gain", "진입 각도보정 이득", 0.80, float, 0.0, 3.0),
    ("insertion_max_angular_speed_rad_s", "진입 회전속도 제한", 0.12, float, 0.0, 0.35),
    ("motion_transition_pause_sec", "동작 전환 정지(초)", 0.50, float, 0.10, 2.0),
    ("lidar_front_clearance_m", "차체 밖 전방 여유(m)", 0.01, float, 0.0, 1.0),
    ("lidar_rear_clearance_m", "차체 밖 후방 여유(m)", 0.01, float, 0.0, 1.0),
    ("lidar_left_clearance_m", "차체 밖 좌측 여유(m)", 0.01, float, 0.0, 1.0),
    ("lidar_right_clearance_m", "차체 밖 우측 여유(m)", 0.01, float, 0.0, 1.0),
    ("lidar_body_front_extent_m", "LiDAR→차체 전방(m)", 0.35, float, 0.01, 1.0),
    ("lidar_body_rear_extent_m", "LiDAR→차체 후방(m)", 0.05, float, 0.01, 1.0),
    ("lidar_body_half_width_m", "LiDAR→바퀴 측면(m)", 0.085, float, 0.01, 0.50),
    ("lidar_loaded_front_extent_m", "LiDAR→적재물 앞끝(m)", 0.48, float, 0.10, 1.50),
    ("lidar_loaded_half_width_m", "LiDAR→적재물 측면(m)", 0.085, float, 0.01, 0.50),
    ("lidar_sensor_radius_m", "LiDAR 센서 반경(m)", 0.015, float, 0.005, 0.10),
    ("ready_right_turn_speed_rad_s", "Ready 우회전 속도(rad/s)", 0.20, float, 0.05, 0.50),
    ("ready_right_turn_tolerance_deg", "Ready 우회전 오차(°)", 3.0, float, 0.5, 15.0),
    ("lidar_self_mask_front_half_angle_deg", "차체마스크 반각(°)", 20.0, float, 0.0, 90.0),
    ("lidar_self_mask_front_max_range_m", "차체마스크 거리(m)", 0.18, float, 0.0, 1.0),
)


class TestPanelNode(Node):
    def __init__(
        self, vehicle, image_topic, log_callback, yolo_callback, status_callback,
        lidar_callback, motion_watchdog_callback,
    ):
        super().__init__("auto_dock_test_panel")
        robot = f"/robot_{vehicle}"
        self.log_callback = log_callback
        self.yolo_callback = yolo_callback
        self.status_callback = status_callback
        self.lidar_callback = lidar_callback
        self.motion_watchdog_callback = motion_watchdog_callback
        self.bridge = CvBridge()
        self.motion_watchdog_enabled = False
        self.motion_watchdog_triggered = False
        self.auto_dock_state = "IDLE"
        self.auto_dock_cmd_gid = None
        self.auto_dock_gid_checked_at = 0.0
        self.last_auto_command_at = 0.0
        self.auto_command_started_at = None
        self.low_flow_started_at = None
        self.last_flow_frame_at = 0.0
        self.previous_flow_gray = None
        self.last_watchdog_report_at = 0.0

        self.arrival_pub = self.create_publisher(String, f"{robot}/nav2/arrival", 10)
        self.fork_command_pub = self.create_publisher(String, "/fork/command", 10)
        self.fork_state_pub = self.create_publisher(String, f"{robot}/fork/state", 10)
        self.stop_pub = self.create_publisher(Empty, f"{robot}/auto_dock/stop", 10)
        self.load_state_pub = self.create_publisher(
            String, f"{robot}/auto_dock/test/load_state", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/controller/cmd_vel", 10)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, f"{robot}/auto_dock/status",
            lambda msg: self.received_status(f"{robot}/auto_dock/status", msg.data),
            status_qos,
        )
        self.create_subscription(
            String, "/fork/command",
            lambda msg: self.received("/fork/command", msg.data), 10,
        )
        self.create_subscription(
            String, f"{robot}/fork/state",
            lambda msg: self.received(f"{robot}/fork/state", msg.data), 10,
        )
        self.create_subscription(
            Empty, f"{robot}/auto_dock/drive_ready",
            lambda _msg: self.received(f"{robot}/auto_dock/drive_ready", "<Empty>"), 10,
        )
        self.create_subscription(
            String, f"{robot}/symbol_seg/detections", self.received_yolo, 10,
        )
        self.create_subscription(LaserScan, "/scan_raw", self.received_lidar, 10)
        self.create_subscription(
            Twist, "/controller/cmd_vel", self.received_cmd_vel, 20
        )
        camera_qos = QoSProfile(depth=1)
        camera_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Image, image_topic, self.received_camera, camera_qos)

    def received(self, topic, value):
        if topic.endswith("/auto_dock/status"):
            try:
                payload = json.loads(value)
                value = (
                    f"state={payload.get('state')} reason={payload.get('reason')} "
                    f"operation={payload.get('operation')} "
                    f"load_state={payload.get('load_state')}"
                )
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
        self.log_callback("SUB", topic, value)

    def received_yolo(self, message):
        self.yolo_callback(message.data)

    def received_lidar(self, message):
        nearest = {
            direction: math.inf
            for direction in ("front", "rear", "left", "right")
        }
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or distance < max(message.range_min, 0.03):
                continue
            angle = message.angle_min + index * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= math.radians(20.0) and distance <= 0.18:
                continue
            x, y = math.cos(angle), math.sin(angle)
            if abs(x) >= abs(y):
                direction = "front" if x >= 0.0 else "rear"
            else:
                direction = "left" if y >= 0.0 else "right"
            nearest[direction] = min(nearest[direction], float(distance))
        self.lidar_callback(nearest)

    def received_status(self, topic, value):
        try:
            payload = json.loads(value)
            if isinstance(payload, dict):
                self.auto_dock_state = str(payload.get("state") or "UNKNOWN").upper()
                if self.auto_dock_state in {"IDLE", "READY", "ERROR"}:
                    self.reset_motion_watchdog_tracking()
                self.status_callback(payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
        self.received(topic, value)

    def set_motion_watchdog_enabled(self, enabled):
        self.motion_watchdog_enabled = bool(enabled)
        self.motion_watchdog_triggered = False
        self.reset_motion_watchdog_tracking(clear_frame=True)
        state = "대기 중" if enabled else "꺼짐"
        self.motion_watchdog_callback(state)
        self.log_callback("CFG", "camera_motion_watchdog", str(bool(enabled)))

    def reset_motion_watchdog_tracking(self, clear_frame=False):
        self.last_auto_command_at = 0.0
        self.auto_command_started_at = None
        self.low_flow_started_at = None
        if clear_frame:
            self.previous_flow_gray = None

    def refresh_auto_dock_cmd_gid(self, now):
        if self.auto_dock_cmd_gid is not None and now - self.auto_dock_gid_checked_at < 2.0:
            return
        self.auto_dock_gid_checked_at = now
        self.auto_dock_cmd_gid = None
        for endpoint in self.get_publishers_info_by_topic("/controller/cmd_vel"):
            if endpoint.node_name == "auto_dock":
                self.auto_dock_cmd_gid = bytes(endpoint.endpoint_gid)
                break

    def received_cmd_vel(self, message, message_info):
        if not self.motion_watchdog_enabled or self.motion_watchdog_triggered:
            return
        now = time.monotonic()
        self.refresh_auto_dock_cmd_gid(now)
        if (
            self.auto_dock_cmd_gid is None
            or bytes(message_info.publisher_gid) != self.auto_dock_cmd_gid
        ):
            return
        moving = (
            math.hypot(float(message.linear.x), float(message.linear.y)) >= 0.025
            or abs(float(message.angular.z)) >= 0.06
        )
        if not moving:
            self.reset_motion_watchdog_tracking()
            return
        self.last_auto_command_at = now
        if self.auto_command_started_at is None:
            self.auto_command_started_at = now

    def received_camera(self, message):
        if not self.motion_watchdog_enabled or self.motion_watchdog_triggered:
            return
        now = time.monotonic()
        if now - self.last_flow_frame_at < 0.10:
            return
        self.last_flow_frame_at = now
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # CvBridge exception types vary by ROS build.
            if now - self.last_watchdog_report_at >= 1.0:
                self.last_watchdog_report_at = now
                self.motion_watchdog_callback(f"영상 변환 실패: {exc}")
            return
        height, width = frame.shape[:2]
        scale = min(1.0, 320.0 / max(width, 1))
        gray = cv2.cvtColor(
            cv2.resize(frame, (max(1, int(width * scale)), max(1, int(height * scale)))),
            cv2.COLOR_BGR2GRAY,
        )
        previous = self.previous_flow_gray
        previous_at = getattr(self, "previous_flow_frame_at", None)
        self.previous_flow_gray = gray
        self.previous_flow_frame_at = now
        if previous is None or previous.shape != gray.shape or previous_at is None:
            return
        points = cv2.goodFeaturesToTrack(
            previous, maxCorners=80, qualityLevel=0.02, minDistance=8, blockSize=7
        )
        if points is None or len(points) < 15:
            self.low_flow_started_at = None
            self.motion_watchdog_callback("특징점 부족 · 판정 보류")
            return
        moved, status, _error = cv2.calcOpticalFlowPyrLK(
            previous, gray, points, None,
            winSize=(15, 15), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )
        if moved is None or status is None:
            self.low_flow_started_at = None
            return
        valid = status.reshape(-1).astype(bool)
        if int(np.count_nonzero(valid)) < 15:
            self.low_flow_started_at = None
            return
        displacement = np.linalg.norm(
            moved.reshape(-1, 2)[valid] - points.reshape(-1, 2)[valid], axis=1
        )
        dt = max(0.001, now - previous_at)
        flow_px_s = float(np.median(displacement) / dt)
        command_active = (
            self.auto_dock_state in {
                "SEARCHING", "ALIGNING", "INSERTING", "REVERSING", "TURNING"
            }
            and now - self.last_auto_command_at <= 0.25
            and self.auto_command_started_at is not None
            and now - self.auto_command_started_at >= 0.40
        )
        if not command_active:
            self.low_flow_started_at = None
        elif flow_px_s < 3.0:
            if self.low_flow_started_at is None:
                self.low_flow_started_at = now
            elif now - self.low_flow_started_at >= 0.70:
                self.motion_watchdog_triggered = True
                self.stop_pub.publish(Empty())
                detail = f"STOP · 명령 중 영상 이동 {flow_px_s:.1f}px/s"
                self.motion_watchdog_callback(detail)
                self.log_callback("SAFE", "camera_motion_watchdog", detail)
                return
        else:
            self.low_flow_started_at = None
        if now - self.last_watchdog_report_at >= 0.50:
            self.last_watchdog_report_at = now
            prefix = "감시 중" if command_active else "대기 중"
            self.motion_watchdog_callback(f"{prefix} · {flow_px_s:.1f}px/s")

    def publish_arrival(self, payload):
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.arrival_pub.publish(String(data=data))
        self.log_callback("PUB", self.arrival_pub.topic_name, data)

    def publish_fork_state(self, state, error=""):
        data = json.dumps({"state": state, "error": error}, separators=(",", ":"))
        self.fork_state_pub.publish(String(data=data))
        self.log_callback("PUB", self.fork_state_pub.topic_name, data)

    def publish_fork_command(self, command):
        command = command.strip().upper()
        self.fork_command_pub.publish(String(data=command))
        self.log_callback("PUB", self.fork_command_pub.topic_name, command)

    def publish_stop(self):
        self.stop_pub.publish(Empty())
        self.log_callback("PUB", self.stop_pub.topic_name, "<Empty>")

    def publish_load_state(self, state):
        state = state.strip().upper()
        self.load_state_pub.publish(String(data=state))
        self.log_callback("PUB", self.load_state_pub.topic_name, state)

    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(message)


class TestPanel:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.enabled = tk.BooleanVar(value=False)
        self.arrival_status = tk.StringVar(value="SUCCEEDED")
        self.arrival_reason = tk.StringVar(value="manual_test_failure")
        self.location = tk.StringVar(value="DOCK_1")
        self.operation = tk.StringVar(value="PICK")
        self.product_type = tk.StringVar(value="NORMAL")
        self.target_type = tk.StringVar(value="SYMBOLS")
        self.left_symbol = tk.StringVar(value="spade")
        self.right_symbol = tk.StringVar(value="spade")
        self.slot_id = tk.StringVar(value="AUTO")
        self.error_text = tk.StringVar(value="test failure")
        self.fsm_state = tk.StringVar(value="연결 대기")
        self.fsm_detail = tk.StringVar(value="Auto Dock status 수신 전")
        # Match the established vehicle teleop defaults.  The previous
        # 0.06 m/s value can be too weak to overcome drivetrain stiction.
        self.manual_linear_speed = tk.StringVar(value="0.12")
        self.manual_angular_speed = tk.StringVar(value="0.35")
        self.manual_status = tk.StringVar(value="정지")
        self.lidar_ranges = tk.StringVar(value="LiDAR 수신 대기")
        self.motion_watchdog_enabled = tk.BooleanVar(value=False)
        self.motion_watchdog_status = tk.StringVar(value="꺼짐")
        self.last_lidar_log_at = 0.0
        self.manual_keys = set()
        self.manual_engaged = False
        self.manual_takeover_active = False
        self.manual_release_jobs = {}
        self.last_yolo_signature = None
        self.log_entries = deque(maxlen=10)
        self.config_path = Path(args.config)
        self.tuning_vars = {
            key: tk.StringVar(value=str(default))
            for key, _label, default, _kind, _minimum, _maximum in TUNING_FIELDS
        }
        self.tuning_notice = tk.StringVar(value=f"설정 파일: {self.config_path}")
        self.load_tuning()

        root.title(f"AUTO-DOCK REAL TEST PANEL · robot_{args.vehicle}")
        root.geometry("1440x820")
        root.minsize(1080, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.node = TestPanelNode(
            args.vehicle, args.image_topic, self.append_log, self.update_yolo,
            self.update_fsm_status, self.update_lidar_ranges,
            self.update_motion_watchdog_status,
        )
        self.action_buttons = []
        self.build_ui()
        self.root.bind_all("<KeyPress>", self.on_manual_key_press)
        self.root.bind_all("<KeyRelease>", self.on_manual_key_release)
        self.root.bind("<Button-1>", self.on_panel_click, add="+")
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.update_enabled()
        self.root.after(20, self.poll_ros)
        self.root.after(50, self.manual_drive_tick)

    def build_ui(self):
        style = ttk.Style(self.root)
        style.configure("Compact.TButton", padding=(5, 3), font=("DejaVu Sans", 9))
        style.configure("Compact.Toolbutton", padding=(5, 3), font=("DejaVu Sans", 9))
        warning = tk.Label(
            self.root,
            text=(
                "REAL VEHICLE TEST · 이 GUI는 실제 ROS 토픽을 발행합니다.\n"
                "WASD/QE 또는 auto_dock 명령으로 실차가 움직일 수 있습니다."
            ),
            bg="#8b1e1e", fg="white", font=("DejaVu Sans", 11, "bold"), pady=8,
        )
        warning.pack(fill="x")

        enable = tk.Checkbutton(
            self.root, text="실차 이벤트 발행 활성화", variable=self.enabled,
            command=self.update_enabled, fg="#8b1e1e", font=("DejaVu Sans", 10, "bold"),
        )
        enable.pack(anchor="w", padx=12, pady=(8, 2))

        stop = tk.Button(
            self.root, text="■ AUTO-DOCK STOP", command=self.publish_stop,
            bg="#b42318", fg="white", activebackground="#8b1e1e",
            font=("DejaVu Sans", 12, "bold"), pady=5,
        )
        stop.pack(fill="x", padx=12, pady=(2, 4))

        watchdog = ttk.Frame(self.root)
        watchdog.pack(fill="x", padx=12, pady=(0, 4))
        ttk.Checkbutton(
            watchdog,
            text="실험: 카메라 이동불일치 자동 STOP",
            variable=self.motion_watchdog_enabled,
            command=self.toggle_motion_watchdog,
        ).pack(side="left")
        ttk.Label(
            watchdog, textvariable=self.motion_watchdog_status,
            font=("DejaVu Sans Mono", 9),
        ).pack(side="left", padx=(12, 0))

        panes = ttk.Panedwindow(self.root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        controls_host = ttk.Frame(panes)
        monitor = ttk.Frame(panes)
        panes.add(controls_host, weight=3)
        panes.add(monitor, weight=2)

        # The control column is taller than many laptop displays.  Keep the
        # monitor visible and let the entire left column scroll independently.
        controls_canvas = tk.Canvas(
            controls_host, highlightthickness=0, borderwidth=0,
        )
        self.controls_canvas = controls_canvas
        controls_scrollbar = ttk.Scrollbar(
            controls_host, orient="vertical", command=controls_canvas.yview,
        )
        controls_canvas.configure(yscrollcommand=controls_scrollbar.set)
        controls_scrollbar.pack(side="right", fill="y")
        controls_canvas.pack(side="left", fill="both", expand=True)

        controls = ttk.Frame(controls_canvas)
        controls_window = controls_canvas.create_window(
            (0, 0), window=controls, anchor="nw",
        )
        controls.bind(
            "<Configure>",
            lambda _event: controls_canvas.configure(
                scrollregion=controls_canvas.bbox("all")
            ),
        )
        controls_canvas.bind(
            "<Configure>",
            lambda event: controls_canvas.itemconfigure(
                controls_window, width=event.width
            ),
        )
        self.root.bind_all("<MouseWheel>", self.on_controls_mousewheel, add="+")
        self.root.bind_all("<Button-4>", self.on_controls_mousewheel, add="+")
        self.root.bind_all("<Button-5>", self.on_controls_mousewheel, add="+")

        manual = ttk.LabelFrame(
            monitor, text="수동 주행 · 누르는 동안만 동작", padding=8
        )
        manual.pack(fill="x", padx=4, pady=(6, 3))
        ttk.Label(
            manual, text="W/S 전후 · A/D 횡이동 · Q/E 회전",
            font=("DejaVu Sans", 11, "bold"),
        ).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(manual, text="이동 m/s").grid(row=1, column=0, sticky="e", pady=(6, 0))
        ttk.Entry(
            manual, textvariable=self.manual_linear_speed, width=7
        ).grid(row=1, column=1, sticky="w", padx=(4, 12), pady=(6, 0))
        ttk.Label(manual, text="회전 rad/s").grid(row=1, column=2, sticky="e", pady=(6, 0))
        ttk.Entry(
            manual, textvariable=self.manual_angular_speed, width=7
        ).grid(row=1, column=3, sticky="w", padx=4, pady=(6, 0))
        tk.Label(
            manual, textvariable=self.manual_status, bg="#111827", fg="#f8fafc",
            font=("DejaVu Sans", 10, "bold"), padx=8, pady=3,
        ).grid(row=2, column=0, columnspan=4, sticky="ew", pady=(7, 0))
        ttk.Label(
            manual, textvariable=self.lidar_ranges,
            font=("DejaVu Sans Mono", 10),
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(7, 0))
        manual.columnconfigure(3, weight=1)

        fsm = ttk.LabelFrame(controls, text="현재 Auto Dock 상태머신", padding=8)
        fsm.pack(fill="x", padx=12, pady=6)
        self.fsm_state_label = tk.Label(
            fsm, textvariable=self.fsm_state, bg="#334155", fg="white",
            font=("DejaVu Sans", 16, "bold"), padx=12, pady=5,
        )
        self.fsm_state_label.pack(side="left", fill="y")
        ttk.Label(
            fsm, textvariable=self.fsm_detail, justify="left",
            font=("DejaVu Sans", 10),
        ).pack(side="left", fill="x", expand=True, padx=10)

        nav = ttk.LabelFrame(controls, text="Arrival 발행 · 선택 버튼", padding=5)
        nav.pack(fill="x", padx=12, pady=6)
        self.add_choice_group(
            nav, "상태", self.arrival_status,
            (("성공", "SUCCEEDED"), ("실패", "FAILED")), 0, 0, 2,
        )
        self.add_choice_group(
            nav, "작업", self.operation, (("PICK", "PICK"), ("PLACE", "PLACE")),
            0, 2, 2,
        )
        self.add_choice_group(
            nav, "상품", self.product_type,
            (("NORMAL", "NORMAL"), ("FRESH", "FRESH")), 0, 4, 2,
        )
        self.add_choice_group(
            nav, "타깃", self.target_type,
            (("TAG", "SYMBOLS"), ("SLOT", "SLOT"),
             ("AUTO", "AUTO_SLOT"), ("없음", "NONE")),
            0, 6, 4,
        )
        self.add_choice_group(
            nav, "위치", self.location,
            (("D1", "DOCK_1"), ("NORMAL", "NORMAL"), ("FRESH", "FRESH"),
             ("Y1", "Y1"), ("Y2", "Y2"), ("Y3", "Y3"), ("Y4", "Y4")),
            1, 0, 10,
        )
        symbol_buttons = tuple(zip(
            ("♠ spade", "♥ heart", "♣ clover", "♦ diamond", "★ star"),
            SYMBOLS,
        ))
        self.add_choice_group(
            nav, "왼쪽 태그", self.left_symbol, symbol_buttons, 2, 0, 5
        )
        self.add_choice_group(
            nav, "오른쪽 태그", self.right_symbol, symbol_buttons, 2, 5, 5
        )
        arrival_extra = ttk.LabelFrame(nav, text="slot / 실패사유 / 발행", padding=2)
        arrival_extra.grid(
            row=3, column=0, columnspan=10, sticky="ew", padx=2, pady=1
        )
        ttk.Entry(arrival_extra, textvariable=self.slot_id, width=7).grid(
            row=0, column=0, sticky="ew", padx=1
        )
        ttk.Entry(arrival_extra, textvariable=self.arrival_reason, width=10).grid(
            row=0, column=1, sticky="ew", padx=1
        )
        self.add_action_button(
            arrival_extra, "ARRIVAL", self.publish_arrival, row=0, column=2, padx=1
        )
        for column in range(10):
            nav.columnconfigure(column, weight=1)

        slot_grid = ttk.LabelFrame(
            controls,
            text="NORMAL 3×3 지정 하차 · 클릭 즉시 Arrival 발행",
            padding=8,
        )
        slot_grid.pack(fill="x", padx=12, pady=6)
        ttk.Label(
            slot_grid,
            text="안쪽 R3",
            font=("DejaVu Sans", 9, "bold"),
        ).grid(row=0, column=0, sticky="w", padx=(0, 6))
        for display_row, slot_row in enumerate((3, 2, 1), start=0):
            for column in range(1, 4):
                slot_id = f"R{slot_row}C{column}"
                self.add_action_button(
                    slot_grid,
                    slot_id,
                    lambda selected=slot_id: self.publish_normal_slot(selected),
                    row=display_row,
                    column=column,
                    padx=3,
                    pady=3,
                )
                slot_grid.columnconfigure(column, weight=1)
        ttk.Label(slot_grid, text="바깥쪽 R1").grid(
            row=2, column=0, sticky="w", padx=(0, 6)
        )

        quick = ttk.LabelFrame(controls, text="빠른 Arrival", padding=10)
        quick.pack(fill="x", padx=12, pady=6)
        scenarios = (
            ("DOCK PICK", "DOCK_1", "PICK", "NORMAL", "SYMBOLS", "AUTO"),
            ("NORMAL PLACE", "NORMAL", "PLACE", "NORMAL", "AUTO_SLOT", "AUTO"),
            ("FRESH PLACE", "FRESH", "PLACE", "FRESH", "AUTO_SLOT", "AUTO"),
            ("FRESH PICK", "FRESH", "PICK", "FRESH", "AUTO_SLOT", "AUTO"),
            ("Y1 PLACE", "Y1", "PLACE", "FRESH", "SLOT", "Y1"),
            ("Y2 PLACE", "Y2", "PLACE", "FRESH", "SLOT", "Y2"),
            ("Y3 PLACE", "Y3", "PLACE", "FRESH", "SLOT", "Y3"),
            ("Y4 PLACE", "Y4", "PLACE", "FRESH", "SLOT", "Y4"),
        )
        for index, scenario in enumerate(scenarios):
            self.add_action_button(
                quick, scenario[0], lambda values=scenario: self.publish_scenario(values),
                row=index // 4, column=index % 4, padx=3, pady=3,
            )
            quick.columnconfigure(index % 4, weight=1)

        tuning_header = ttk.Frame(controls)
        tuning_header.pack(fill="x", padx=12, pady=4)
        self.tuning_toggle_button = ttk.Button(
            tuning_header, text="▶ Auto Dock 상세 설정 펼치기", command=self.toggle_tuning,
            style="Compact.TButton",
        )
        self.tuning_toggle_button.pack(fill="x")
        self.tuning_frame = ttk.LabelFrame(
            controls,
            text=(
                "Auto Dock 시험 설정 · 다음 Arrival부터 적용 · "
                "LiDAR 대각선은 차체 외곽에서 자동 계산"
            ),
            padding=10,
        )
        tuning = self.tuning_frame
        tuning_columns = 5
        for index, (key, label, _default, _kind, _minimum, _maximum) in enumerate(
            TUNING_FIELDS
        ):
            column = index % tuning_columns
            label_row = (index // tuning_columns) * 2
            ttk.Label(tuning, text=label).grid(
                row=label_row, column=column, sticky="w", padx=3
            )
            ttk.Entry(
                tuning, textvariable=self.tuning_vars[key], width=12
            ).grid(row=label_row + 1, column=column, sticky="ew", padx=3)
            tuning.columnconfigure(column, weight=1)
        action_row = ((len(TUNING_FIELDS) - 1) // tuning_columns + 1) * 2
        self.add_action_button(
            tuning, "설정 저장", self.save_tuning,
            row=action_row, column=0, columnspan=tuning_columns, pady=(8, 3),
        )
        ttk.Label(tuning, textvariable=self.tuning_notice).grid(
            row=action_row + 1, column=0, columnspan=tuning_columns, sticky="w", padx=3
        )

        actions = ttk.LabelFrame(controls, text="적재·포크 시험 버튼", padding=4)
        self.actions_frame = actions
        actions.pack(fill="x", padx=12, pady=4)
        for column, state in enumerate(("UNLOADED", "LOADED")):
            self.add_action_button(
                actions, f"강제 {state}",
                lambda value=state: self.publish_load_state(value),
                row=0, column=column, padx=2, pady=1,
            )
        for column, command in enumerate(("UP", "DOWN", "STOP")):
            self.add_action_button(
                actions, f"FORK {command}",
                lambda value=command: self.publish_fork_command(value),
                row=0, column=column + 2, padx=2, pady=1,
            )
        for column, (text, command) in enumerate((
            ("UP_COMPLETE", lambda: self.publish_fork("UP_COMPLETE")),
            ("DOWN_COMPLETE", lambda: self.publish_fork("DOWN_COMPLETE")),
            ("FAILED", lambda: self.publish_fork("FAILED")),
        )):
            self.add_action_button(
                actions, text, command, row=1, column=column, padx=2, pady=1
            )
        ttk.Entry(actions, textvariable=self.error_text).grid(
            row=1, column=3, columnspan=2, sticky="ew", padx=2, pady=1
        )
        for column in range(5):
            actions.columnconfigure(column, weight=1)

        yolo_frame = ttk.LabelFrame(
            monitor,
            text=f"YOLO 최신 인식 · /robot_{self.args.vehicle}/symbol_seg/detections",
            padding=6,
        )
        yolo_frame.pack(fill="both", expand=True, padx=4, pady=(6, 3))
        self.yolo = tk.Text(
            yolo_frame, height=17, wrap="word", state="disabled",
            font=("DejaVu Sans Mono", 9), bg="#071a12", fg="#d1fae5",
        )
        yolo_scrollbar = ttk.Scrollbar(
            yolo_frame, orient="vertical", command=self.yolo.yview
        )
        self.yolo.configure(yscrollcommand=yolo_scrollbar.set)
        self.yolo.pack(side="left", fill="both", expand=True)
        yolo_scrollbar.pack(side="right", fill="y")

        log_frame = ttk.LabelFrame(
            monitor, text="토픽 송수신 로그 · 최신 10건", padding=6
        )
        log_frame.pack(fill="both", expand=True, padx=4, pady=(3, 6))
        self.log = tk.Text(
            log_frame, height=14, wrap="word", state="disabled",
            font=("DejaVu Sans Mono", 9), bg="#111827", fg="#e5e7eb",
        )
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def add_action_button(self, parent, text, command, **grid):
        button = ttk.Button(
            parent, text=text, command=command, style="Compact.TButton"
        )
        button.grid(sticky="ew", **grid)
        self.action_buttons.append(button)
        return button

    def add_choice_group(
        self, parent, title, variable, choices, row, column, columnspan=1
    ):
        group = ttk.LabelFrame(parent, text=title, padding=2)
        group.grid(
            row=row, column=column, columnspan=columnspan,
            sticky="nsew", padx=2, pady=1,
        )
        for index, (text, value) in enumerate(choices):
            ttk.Radiobutton(
                group, text=text, variable=variable, value=value,
                style="Compact.Toolbutton",
            ).grid(row=0, column=index, sticky="w", padx=1)
        return group

    @staticmethod
    def widget_is_descendant(widget, ancestor):
        current = widget
        while current is not None:
            if current == ancestor:
                return True
            parent_name = current.winfo_parent()
            if not parent_name:
                break
            try:
                current = current.nametowidget(parent_name)
            except KeyError:
                break
        return False

    def on_controls_mousewheel(self, event):
        if not self.widget_is_descendant(event.widget, self.controls_canvas):
            return None
        if getattr(event, "num", None) == 4:
            units = -3
        elif getattr(event, "num", None) == 5:
            units = 3
        else:
            delta = getattr(event, "delta", 0)
            units = -3 if delta > 0 else 3
        self.controls_canvas.yview_scroll(units, "units")
        return "break"

    def toggle_tuning(self):
        if self.tuning_frame.winfo_manager():
            self.tuning_frame.pack_forget()
            self.tuning_toggle_button.configure(text="▶ Auto Dock 상세 설정 펼치기")
        else:
            self.tuning_frame.pack(
                fill="x", padx=12, pady=4, before=self.actions_frame
            )
            self.tuning_toggle_button.configure(text="▼ Auto Dock 상세 설정 접기")

    def update_enabled(self):
        state = "normal" if self.enabled.get() else "disabled"
        for button in self.action_buttons:
            button.configure(state=state)
        if not self.enabled.get() and hasattr(self, "node"):
            self.stop_manual_drive()
            self.manual_takeover_active = False

    def require_enabled(self):
        return self.enabled.get()

    @staticmethod
    def is_text_input(widget):
        return widget.winfo_class() in {"Entry", "TEntry", "TCombobox", "Text"}

    def on_panel_click(self, event):
        if not self.is_text_input(event.widget):
            self.root.focus_set()

    def on_manual_key_press(self, event):
        key = str(event.keysym).lower()
        if key not in {"w", "a", "s", "d", "q", "e"}:
            return None
        if not self.require_enabled() or self.is_text_input(event.widget):
            return None
        release_job = self.manual_release_jobs.pop(key, None)
        if release_job is not None:
            self.root.after_cancel(release_job)
        if not self.manual_takeover_active:
            self.node.publish_stop()
            self.manual_takeover_active = True
        self.manual_engaged = True
        self.manual_keys.add(key)
        self.publish_manual_velocity()
        return "break"

    def on_manual_key_release(self, event):
        key = str(event.keysym).lower()
        if key not in {"w", "a", "s", "d", "q", "e"}:
            return None
        previous_job = self.manual_release_jobs.pop(key, None)
        if previous_job is not None:
            self.root.after_cancel(previous_job)
        # Remote X11 key repeat can deliver the synthetic release/press pair
        # more than 40 ms apart.  Keep the key held through that gap so a
        # repeated key does not inject STOP commands between drive commands.
        self.manual_release_jobs[key] = self.root.after(
            150, lambda held_key=key: self.finish_manual_key_release(held_key)
        )
        return "break"

    def finish_manual_key_release(self, key):
        self.manual_release_jobs.pop(key, None)
        self.manual_keys.discard(key)
        if not self.manual_keys:
            self.manual_engaged = False
        self.publish_manual_velocity()

    def on_focus_out(self, _event):
        self.stop_manual_drive()

    def manual_speeds(self):
        try:
            linear = min(0.20, max(0.01, float(self.manual_linear_speed.get())))
            angular = min(0.50, max(0.05, float(self.manual_angular_speed.get())))
        except ValueError:
            self.manual_status.set("속도 입력 오류")
            return None
        return linear, angular

    def publish_manual_velocity(self):
        if not self.require_enabled() or not self.manual_engaged:
            self.node.publish_cmd_vel()
            self.manual_status.set("정지")
            return
        speeds = self.manual_speeds()
        if speeds is None:
            self.stop_manual_drive()
            return
        linear, angular = speeds
        linear_x = linear * (("w" in self.manual_keys) - ("s" in self.manual_keys))
        linear_y = linear * (("a" in self.manual_keys) - ("d" in self.manual_keys))
        angular_z = angular * (("q" in self.manual_keys) - ("e" in self.manual_keys))
        self.node.publish_cmd_vel(linear_x, linear_y, angular_z)
        keys = "+".join(sorted(self.manual_keys)) or "정지"
        self.manual_status.set(
            f"{keys.upper()} · x={linear_x:+.2f} y={linear_y:+.2f} yaw={angular_z:+.2f}"
        )

    def manual_drive_tick(self):
        if self.manual_engaged and self.manual_keys:
            self.publish_manual_velocity()
        self.root.after(50, self.manual_drive_tick)

    def stop_manual_drive(self):
        for job in self.manual_release_jobs.values():
            self.root.after_cancel(job)
        self.manual_release_jobs.clear()
        self.manual_keys.clear()
        self.manual_engaged = False
        if hasattr(self, "node"):
            self.node.publish_cmd_vel()
        self.manual_status.set("정지")

    def load_tuning(self):
        try:
            payload = json.loads(self.config_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.tuning_notice.set(f"설정 읽기 실패: {exc}")
            return
        for key, _label, default, _kind, _minimum, _maximum in TUNING_FIELDS:
            self.tuning_vars[key].set(str(payload.get(key, default)))

    def save_tuning(self):
        if not self.require_enabled():
            return
        updates = {}
        try:
            for key, label, _default, kind, minimum, maximum in TUNING_FIELDS:
                value = kind(self.tuning_vars[key].get())
                if not minimum <= value <= maximum:
                    raise ValueError(f"{label}: {minimum}~{maximum} 범위")
                updates[key] = value
            payload = {}
            if self.config_path.exists():
                payload = json.loads(self.config_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("설정 JSON은 object여야 합니다")
            payload.update(updates)
            temporary = self.config_path.with_suffix(self.config_path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, self.config_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.tuning_notice.set(f"저장 실패: {exc}")
            return
        self.tuning_notice.set("저장 완료 · STOP 후 다음 Arrival부터 적용")
        self.append_log("CFG", str(self.config_path), json.dumps(updates))

    def target_payload(self):
        kind = self.target_type.get()
        if kind == "SYMBOLS":
            return {"type": kind, "left": self.left_symbol.get(), "right": self.right_symbol.get()}
        if kind == "SLOT":
            return {"type": kind, "slot_id": self.slot_id.get().strip()}
        if kind == "AUTO_SLOT":
            return {"type": kind}
        return None

    def arrival_payload(self):
        payload = {
            "status": self.arrival_status.get(),
            "location": self.location.get(),
            "operation": self.operation.get(),
            "product_type": self.product_type.get(),
            "target": self.target_payload(),
        }
        if self.arrival_status.get() == "FAILED":
            payload["reason"] = self.arrival_reason.get().strip()
        return payload

    def publish_arrival(self):
        if self.require_enabled():
            self.manual_takeover_active = False
            self.node.publish_arrival(self.arrival_payload())

    def publish_scenario(self, values):
        if not self.require_enabled():
            return
        _, location, operation, product, target_type, slot = values
        self.arrival_status.set("SUCCEEDED")
        self.location.set(location)
        self.operation.set(operation)
        self.product_type.set(product)
        self.target_type.set(target_type)
        self.slot_id.set(slot)
        self.manual_takeover_active = False
        self.node.publish_arrival(self.arrival_payload())

    def publish_normal_slot(self, slot_id):
        if not self.require_enabled():
            return
        self.arrival_status.set("SUCCEEDED")
        self.location.set("NORMAL")
        self.operation.set("PLACE")
        self.product_type.set("NORMAL")
        self.target_type.set("SLOT")
        self.slot_id.set(str(slot_id).upper())
        self.manual_takeover_active = False
        self.node.publish_arrival(self.arrival_payload())

    def publish_fork(self, state):
        if self.require_enabled():
            error = self.error_text.get().strip() if state == "FAILED" else ""
            self.node.publish_fork_state(state, error)

    def publish_fork_command(self, command):
        if self.require_enabled():
            self.node.publish_fork_command(command)

    def publish_stop(self):
        self.stop_manual_drive()
        self.manual_takeover_active = False
        self.node.publish_stop()

    def toggle_motion_watchdog(self):
        self.node.set_motion_watchdog_enabled(self.motion_watchdog_enabled.get())

    def update_motion_watchdog_status(self, status):
        self.motion_watchdog_status.set(str(status))

    def publish_load_state(self, state):
        if self.require_enabled():
            self.node.publish_load_state(state)

    def append_log(self, direction, topic, value):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{timestamp} {direction:<3} {topic}\n    {value}\n"
        self.log_entries.append(line)
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.insert("1.0", "".join(self.log_entries))
        self.log.see("end")
        self.log.configure(state="disabled")

    def update_fsm_status(self, payload):
        state = str(payload.get("state") or "UNKNOWN").upper()
        self.fsm_state.set(state)
        self.fsm_detail.set(
            f"reason: {payload.get('reason', '-')}\n"
            f"operation: {payload.get('operation', '-')}   "
            f"load_state: {payload.get('load_state', '-')}   "
            f"location: {payload.get('location', '-')}"
        )
        colors = {
            "IDLE": "#475569", "SEARCHING": "#2563eb",
            "ALIGNING": "#7c3aed", "INSERTING": "#c2410c",
            "WAIT_UP_COMPLETE": "#a16207", "WAIT_DOWN_COMPLETE": "#a16207",
            "REVERSING": "#0369a1", "TURNING": "#0f766e",
            "READY": "#15803d", "ERROR": "#b91c1c",
        }
        self.fsm_state_label.configure(bg=colors.get(state, "#334155"))

    def update_lidar_ranges(self, nearest):
        def shown(direction):
            distance = nearest.get(direction, math.inf)
            return "---" if not math.isfinite(distance) else f"{distance * 100.0:.1f}cm"

        text = (
            f"LiDAR  전 {shown('front')}  후 {shown('rear')}  "
            f"좌 {shown('left')}  우 {shown('right')}"
        )
        self.lidar_ranges.set(text)
        now = time.monotonic()
        if now - self.last_lidar_log_at >= 1.0:
            self.last_lidar_log_at = now
            self.append_log("SUB", "/scan_raw", text)

    def update_yolo(self, raw):
        try:
            payload = json.loads(raw)
            target = payload.get("target_top")
            candidate = payload.get("candidate")
            partial = payload.get("tracked_partial")

            def summarize_target(item):
                if not isinstance(item, dict):
                    return "없음"
                pnp = item.get("pnp") or {}
                depth = item.get("depth_yaw") or {}
                distance = depth.get("forward_distance_cm", pnp.get("forward_distance_cm"))
                yaw = depth.get("yaw_deg", pnp.get("yaw_deg"))
                return (
                    f"matrix={item.get('matrix')} streak={item.get('streak', '-')} "
                    f"center_error={item.get('center_error', '-')} "
                    f"distance_cm={distance} yaw_deg={yaw}"
                )

            detections = payload.get("detections") or []
            seen = [
                f"{item.get('class', '?')} {float(item.get('confidence', 0.0)):.2f}"
                for item in detections
                if isinstance(item, dict)
            ][:10]
            rendered = "\n".join((
                f"수신 시각: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}",
                f"요청 target_top: {target}",
                f"확정 candidate: {summarize_target(candidate)}",
                f"부분 추적: {summarize_target(partial)}",
                f"최근 화면 검출(최대 10개): {', '.join(seen) if seen else '없음'}",
            ))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rendered = f"YOLO JSON 해석 실패: {exc}\n\n{raw}"
        self.yolo.configure(state="normal")
        self.yolo.delete("1.0", "end")
        self.yolo.insert("1.0", rendered)
        self.yolo.configure(state="disabled")

    def poll_ros(self):
        if not rclpy.ok():
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.poll_ros)

    def close(self):
        self.stop_manual_drive()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--config", default="/shared/vehicle_pose_config.json")
    parser.add_argument(
        "--image-topic", default="/ascamera/camera_publisher/rgb0/image"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id or 214 + args.vehicle)
    rclpy.init()
    root = tk.Tk()
    TestPanel(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
