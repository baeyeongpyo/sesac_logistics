#!/usr/bin/env python3
"""Manual ROS event panel for real-vehicle auto-dock integration tests.

This tool injects Nav2/Fork events, shows Auto Dock/YOLO state, and provides
hold-to-drive WASD/QE manual control for real-vehicle tests.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
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
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String


SYMBOLS = ("spade", "heart", "clover", "diamond", "star")
LOCATIONS = ("DOCK_1", "NORMAL", "FRESH", "Y1", "Y2", "Y3", "Y4")
CHECKBOX_FIELDS = (
    ("tag_guided_lateral_search_enabled", "옵션 1 · 정면 YOLO bbox/depth"),
    ("search_rear_lidar_guidance_enabled", "옵션 2 · 후방 LiDAR 30cm"),
    ("lidar_safety_enabled", "LiDAR safety"),
    ("lidar_backoff_enabled", "LiDAR backoff"),
    ("fork_timed_up_complete_enabled", "임시 UP 3초 완료 · 위험"),
    (
        "post_lift_rear_opening_test_enabled",
        "테스트 · 리프트후 우측탐색→후방20cm",
    ),
    ("manual_lateral_yaw_hold_enabled", "수동 A/D 횡이동 yaw hold"),
    ("dock_inventory_scan_enabled", "개발 · DOCK 최근접 슬롯 인식"),
)
CHECKBOX_KEYS = {key for key, _label in CHECKBOX_FIELDS}
TUNING_FIELDS = (
    ("tag_guided_lateral_search_enabled", "정면태그 자세 횡탐색(1/0)", 0, int, 0, 1),
    ("search_rear_lidar_guidance_enabled", "후방LiDAR 횡탐색(1/0)", 0, int, 0, 1),
    ("lidar_safety_enabled", "LiDAR safety(1/0)", 0, int, 0, 1),
    ("lidar_backoff_enabled", "LiDAR backoff(1/0)", 0, int, 0, 1),
    ("lidar_backoff_speed_m_s", "LiDAR backoff 속도(m/s)", 0.08, float, 0.05, 0.30),
    ("lidar_backoff_duration_sec", "LiDAR backoff 1회 시간(초)", 0.30, float, 0.10, 2.00),
    ("lidar_backoff_max_attempts", "LiDAR backoff 최대 횟수", 5, int, 1, 20),
    ("fork_timed_up_complete_enabled", "임시 UP 시간완료(1/0)", 0, int, 0, 1),
    ("post_lift_rear_opening_test_enabled", "리프트후 후방개구 탐색(1/0)", 0, int, 0, 1),
    ("manual_lateral_yaw_hold_enabled", "수동 횡이동 yaw hold(1/0)", 1, int, 0, 1),
    ("manual_lateral_yaw_hold_tolerance_deg", "수동 yaw hold 허용각(°)", 3.0, float, 0.5, 15.0),
    ("manual_lateral_yaw_hold_speed_rad_s", "수동 yaw hold 회전속도(rad/s)", 0.35, float, 0.10, 0.50),
    ("dock_inventory_scan_enabled", "DOCK 최근접 슬롯 인식(1/0)", 0, int, 0, 1),
    ("dock_inventory_scan_interval_sec", "DOCK 인식 주기(초)", 0.50, float, 0.20, 5.0),
    ("dock_inventory_minimum_red_pixels", "DOCK 빨간끝선 최소픽셀", 180, int, 50, 5000),
    ("dock_inventory_first_row_center_ratio", "우측끝→R1 중심 비율", 0.65, float, 0.20, 1.50),
    ("dock_inventory_row_pitch_ratio", "DOCK R간격/팔레트폭", 1.15, float, 0.50, 2.50),
    ("dock_inventory_depth_c1_min_cm", "DOCK C1 최소 depth(cm)", 20.0, float, 5.0, 200.0),
    ("dock_inventory_depth_c1_max_cm", "DOCK C1 최대 depth(cm)", 30.0, float, 5.0, 200.0),
    ("dock_inventory_depth_c2_max_cm", "DOCK C2 최대 depth(cm)", 40.0, float, 5.0, 200.0),
    ("dock_inventory_depth_c3_max_cm", "DOCK C3 최대 depth(cm)", 50.0, float, 5.0, 200.0),
    ("dock_inventory_max_age_sec", "DOCK 관측 유효시간(초)", 3.0, float, 0.5, 30.0),
    ("tape_target_center_y_ratio", "주의선 목표 화면높이 비율", 0.65, float, 0.10, 0.90),
    ("tape_min_forward_speed_m_s", "주의선 전후 최소속도(m/s)", 0.10, float, 0.10, 0.20),
    ("tape_max_forward_speed_m_s", "주의선 전후 최대속도(m/s)", 0.10, float, 0.10, 0.20),
    ("rear_lateral_gain", "뒤 바퀴 횡이동 배율", 1.20, float, 0.50, 2.00),
    ("post_lift_opening_lateral_speed_m_s", "후방개구 우측 횡속도(m/s)", 0.12, float, 0.05, 0.20),
    ("post_lift_opening_jump_cm", "후방개구 급증 판정(cm)", 15.0, float, 5.0, 100.0),
    ("post_lift_opening_confirmation_frames", "후방개구 확인 프레임", 2, int, 1, 10),
    ("post_lift_opening_rear_target_cm", "개구후 후방 목표거리(cm)", 20.0, float, 5.0, 100.0),
    ("post_lift_opening_reverse_speed_m_s", "개구후 후진속도(m/s)", 0.05, float, 0.01, 0.15),
    ("post_lift_opening_scan_max_age_sec", "개구탐색 LiDAR 유효시간(초)", 0.50, float, 0.10, 2.0),
    ("post_lift_opening_search_timeout_sec", "개구탐색 제한시간(초)", 30.0, float, 1.0, 120.0),
    ("post_lift_opening_reverse_timeout_sec", "개구후 후진 제한시간(초)", 10.0, float, 1.0, 60.0),
    ("fork_timed_up_complete_sec", "임시 UP 완료시간(초)", 3.0, float, 0.5, 10.0),
    ("tag_search_max_distance_cm", "탐색 정면태그 최대거리(cm)", 20.0, float, 5.0, 100.0),
    ("tag_search_min_distance_cm", "탐색 정면태그 최소거리(cm)", 15.0, float, 5.0, 100.0),
    ("tag_search_reverse_correction_speed_m_s", "정면태그 과근접 후진속도(m/s)", 0.08, float, 0.03, 0.15),
    ("tag_search_reverse_rear_margin_cm", "과근접 후진 후방여유(cm)", 3.0, float, 0.0, 20.0),
    ("tag_search_noise_max_distance_cm", "탐색 depth 노이즈 상한(cm)", 30.0, float, 10.0, 100.0),
    ("tag_search_yaw_tolerance_deg", "탐색 태그각도 허용(°)", 3.0, float, 0.5, 15.0),
    ("tag_search_min_angular_speed_rad_s", "탐색 최소 회전속도(rad/s)", 0.35, float, 0.10, 0.50),
    ("tag_search_max_angular_speed_rad_s", "탐색 최대 회전속도(rad/s)", 0.35, float, 0.10, 0.50),
    ("tag_search_forward_correction_speed_m_s", "정면태그 전진보정(m/s)", 0.12, float, 0.08, 0.20),
    ("search_rear_lidar_min_distance_cm", "옵션2 후방LiDAR 최소거리(cm)", 30.0, float, 5.0, 100.0),
    ("search_rear_lidar_max_age_sec", "옵션2 LiDAR 유효시간(초)", 0.50, float, 0.10, 2.0),
    ("search_rear_lidar_forward_speed_m_s", "옵션2 전진보정(m/s)", 0.12, float, 0.05, 0.20),
    ("translation_first_alignment_enabled", "이동우선 정렬(1/0)", 0, int, 0, 1),
    ("alignment_max_trusted_yaw_deg", "신뢰 회전오차 한계(°)", 12.0, float, 3.0, 30.0),
    ("translation_alignment_min_angular_speed_rad_s", "이동정렬 최소회전(rad/s)", 0.10, float, 0.0, 0.20),
    ("translation_alignment_max_angular_speed_rad_s", "이동정렬 최대회전(rad/s)", 0.12, float, 0.0, 0.20),
    ("nav2_scan_approach_enabled", "도착후 회전스캔 접근(1/0)", 0, int, 0, 1),
    ("nav2_scan_angle_deg", "회전스캔 좌우각(°)", 20.0, float, 2.0, 30.0),
    ("nav2_scan_angular_speed_rad_s", "회전스캔 속도(rad/s)", 0.18, float, 0.05, 0.40),
    ("nav2_scan_confirmation_sec", "스캔 검출확인(초)", 0.8, float, 0.1, 3.0),
    ("nav2_forward_search_speed_m_s", "태그기준 전진속도(m/s)", 0.08, float, 0.05, 0.15),
    ("nav2_forward_search_max_distance_m", "태그기준 최대전진(m)", 1.0, float, 0.20, 3.0),
    ("nav2_approach_standoff_m", "접근후 정렬거리(m)", 0.22, float, 0.12, 1.0),
    ("nav2_approach_speed_m_s", "목표 접근속도(m/s)", 0.08, float, 0.05, 0.15),
    ("nav2_approach_max_angular_speed_rad_s", "접근 회전제한(rad/s)", 0.16, float, 0.05, 0.25),
    ("stable_detection_frames", "확정 프레임", 2, int, 1, 30),
    ("candidate_stop_delay_sec", "추가 이동(초)", 0.2, float, 0.0, 5.0),
    ("candidate_confirmation_timeout_sec", "확인 제한(초)", 0.8, float, 0.1, 10.0),
    ("candidate_retry_cooldown_sec", "재탐색 이동(초)", 1.0, float, 0.0, 10.0),
    ("nearest_candidate_probe_enabled", "최근접 확정전 좌우스캔(1/0)", 1, int, 0, 1),
    ("nearest_candidate_probe_angle_deg", "최근접 좌우스캔 각도(°)", 20.0, float, 2.0, 45.0),
    ("nearest_candidate_probe_angular_speed_rad_s", "최근접 스캔 회전속도", 0.18, float, 0.05, 0.40),
    ("nearest_candidate_probe_yaw_tolerance_deg", "최근접 스캔 각도허용(°)", 1.5, float, 0.5, 5.0),
    ("tape_guidance_enabled", "주의테이프 추종(1/0)", 0, int, 0, 1),
    ("tape_guidance_only", "주의테이프 전용탐색(1/0)", 0, int, 0, 1),
    ("tape_min_yellow_pixels", "테이프 최소 노랑픽셀", 600, int, 100, 20000),
    ("tape_max_age_sec", "테이프 유효시간(초)", 0.50, float, 0.10, 3.0),
    ("tape_target_angle_deg", "테이프 평행 기준각(도)", 0.0, float, -35.0, 35.0),
    ("tape_forward_gain", "테이프 전후보정 이득", 0.10, float, 0.0, 1.0),
    ("tape_max_forward_speed_m_s", "테이프 전후속도 제한", 0.03, float, 0.0, 0.10),
    ("tape_yaw_gain", "테이프 회전보정 이득", 0.80, float, 0.0, 3.0),
    ("tape_max_yaw_speed_rad_s", "테이프 회전속도 제한", 0.20, float, 0.0, 0.50),
    ("max_pnp_reprojection_error_px", "PnP 유효오차(px)", 3.0, float, 0.1, 100.0),
    ("max_frontal_error", "정면 오차", 0.35, float, 0.01, 2.0),
    ("minimum_dock_measurement_cm", "도킹 측정 최소거리(cm)", 5.0, float, 3.0, 30.0),
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
    ("lidar_body_front_extent_m", "LiDAR→차체 전방(m)", 0.30, float, 0.01, 1.0),
    ("lidar_body_rear_extent_m", "LiDAR→차체 후방(m)", 0.06, float, 0.01, 1.0),
    ("lidar_body_half_width_m", "LiDAR→바퀴 측면(m)", 0.06, float, 0.01, 0.50),
    ("lidar_loaded_front_extent_m", "LiDAR→적재물 앞끝(m)", 0.48, float, 0.10, 1.50),
    ("lidar_loaded_half_width_m", "LiDAR→적재물 측면(m)", 0.10, float, 0.01, 0.50),
    ("lidar_sensor_radius_m", "LiDAR 센서 반경(m)", 0.015, float, 0.005, 0.10),
    ("ready_right_turn_speed_rad_s", "Ready 우회전 속도(rad/s)", 0.20, float, 0.05, 0.50),
    ("ready_right_turn_tolerance_deg", "Ready 우회전 오차(°)", 3.0, float, 0.5, 15.0),
    ("right_turn_scan_wait_timeout_sec", "우회전 LiDAR 대기(초)", 2.0, float, 0.5, 5.0),
    ("lidar_self_mask_front_half_angle_deg", "차체마스크 반각(°)", 20.0, float, 0.0, 90.0),
    ("lidar_self_mask_front_max_range_m", "차체마스크 거리(m)", 0.20, float, 0.0, 1.0),
    ("lidar_self_mask_fixed_angle_deg", "차체반사 고정각(°)", -1.43, float, -180.0, 180.0),
    ("lidar_self_mask_fixed_half_width_deg", "고정각 마스크 반폭(°)", 1.0, float, 0.0, 10.0),
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
        self.last_auto_command_at = 0.0
        self.auto_command_started_at = None
        self.low_flow_started_at = None
        self.last_flow_frame_at = 0.0
        self.previous_flow_gray = None
        self.last_watchdog_report_at = 0.0
        self.odom_yaw = None
        self.imu_yaw = None

        self.arrival_pub = self.create_publisher(String, f"{robot}/nav2/arrival", 10)
        self.fork_command_pub = self.create_publisher(String, "/fork/command", 10)
        self.stop_pub = self.create_publisher(Empty, f"{robot}/auto_dock/stop", 10)
        self.load_state_pub = self.create_publisher(
            String, f"{robot}/auto_dock/test/load_state", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/controller/cmd_vel", 10)
        self.controller_param_client = self.create_client(
            SetParameters, "/odom_publisher/set_parameters"
        )

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
        self.create_subscription(Odometry, "/odom_raw", self.received_odom, 20)
        self.create_subscription(
            Vector3Stamped, "/imu/rpy/filtered", self.received_imu_rpy, 20
        )
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
        self.lidar_callback(message)

    def received_odom(self, message):
        q = message.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def received_imu_rpy(self, message):
        self.imu_yaw = float(message.vector.z)

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

    def received_cmd_vel(self, message):
        if not self.motion_watchdog_enabled or self.motion_watchdog_triggered:
            return
        now = time.monotonic()
        moving = (
            math.hypot(float(message.linear.x), float(message.linear.y)) >= 0.025
            or abs(float(message.angular.z)) >= 0.06
        )
        if not moving:
            # Several nodes publish zero Twist on this shared topic.  Do not
            # let an unrelated zero erase a recent autonomous drive command;
            # the freshness timeout below disarms it naturally.
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

    def set_rear_lateral_gain(self, gain, result_callback):
        try:
            request = SetParameters.Request()
            request.parameters = [
                Parameter("rear_lateral_gain", value=float(gain)).to_parameter_msg()
            ]
            future = self.controller_param_client.call_async(request)
        except Exception as exc:
            result_callback(False, str(exc))
            return

        def finished(completed):
            try:
                results = completed.result().results
                successful = bool(results) and all(result.successful for result in results)
                reason = "" if successful else "; ".join(
                    result.reason for result in results if result.reason
                )
                result_callback(successful, reason)
            except Exception as exc:
                result_callback(False, str(exc))

        future.add_done_callback(finished)


class TestPanel:
    def __init__(self, root, args):
        self.root = root
        self.closing = False
        self.replacement_command = None
        self.args = args
        self.arrival_status = tk.StringVar(value="SUCCEEDED")
        self.arrival_reason = tk.StringVar(value="manual_test_failure")
        self.location = tk.StringVar(value="DOCK_1")
        self.operation = tk.StringVar(value="PICK")
        self.product_type = tk.StringVar(value="NORMAL")
        self.target_type = tk.StringVar(value="SYMBOLS")
        self.left_symbol = tk.StringVar(value="spade")
        self.right_symbol = tk.StringVar(value="spade")
        self.slot_id = tk.StringVar(value="AUTO")
        self.fsm_state = tk.StringVar(value="연결 대기")
        self.fsm_detail = tk.StringVar(value="Auto Dock status 수신 전")
        # Match the established vehicle teleop defaults.  The previous
        # 0.06 m/s value can be too weak to overcome drivetrain stiction.
        self.manual_linear_speed = tk.StringVar(value="0.12")
        self.manual_angular_speed = tk.StringVar(value="0.35")
        self.manual_status = tk.StringVar(value="정지")
        self.lidar_ranges = tk.StringVar(value="LiDAR 수신 대기")
        self.lidar_record_status = tk.StringVar(value="LiDAR 기록 꺼짐")
        self.motion_watchdog_enabled = tk.BooleanVar(value=False)
        self.motion_watchdog_status = tk.StringVar(value="꺼짐")
        self.last_lidar_log_at = 0.0
        self.manual_keys = set()
        self.manual_engaged = False
        self.manual_takeover_active = False
        self.manual_release_jobs = {}
        self.manual_lateral_yaw_target = None
        self.manual_lateral_yaw_source = None
        self.last_yolo_signature = None
        self.log_entries = deque(maxlen=10)
        self.lidar_record_file = None
        self.lidar_record_path = None
        self.lidar_record_scan_count = 0
        self.lidar_record_cmd_count = 0
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
        self.root.after(20, self.poll_ros)
        self.root.after(50, self.manual_drive_tick)
        self.root.after(1000, self.apply_rear_lateral_gain)

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

        stop = tk.Button(
            self.root, text="■ AUTO-DOCK STOP", command=self.publish_stop,
            bg="#b42318", fg="white", activebackground="#8b1e1e",
            font=("DejaVu Sans", 12, "bold"), pady=5,
        )
        stop.pack(fill="x", padx=12, pady=(2, 4))

        switch = tk.Button(
            self.root,
            text="테스트 패널 종료 → Control GUI 열기",
            command=self.switch_to_control_gui,
            bg="#1d4ed8", fg="white", activebackground="#1e40af",
            font=("DejaVu Sans", 10, "bold"), pady=4,
        )
        switch.pack(fill="x", padx=12, pady=(0, 4))

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
        self.lidar_record_button = ttk.Button(
            manual, text="LiDAR 기록 시작", command=self.toggle_lidar_recording,
            style="Compact.TButton",
        )
        self.lidar_record_button.grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(7, 0), padx=(0, 4)
        )
        ttk.Label(
            manual, textvariable=self.lidar_record_status,
            font=("DejaVu Sans Mono", 9),
        ).grid(row=4, column=2, columnspan=2, sticky="w", pady=(7, 0))
        manual.columnconfigure(3, weight=1)

        search_options = ttk.LabelFrame(
            monitor, text="SEARCHING 방식 · 복수 선택 가능", padding=6
        )
        search_options.pack(fill="x", padx=4, pady=(3, 3))
        for index, (key, label) in enumerate(CHECKBOX_FIELDS):
            ttk.Checkbutton(
                search_options, text=label, variable=self.tuning_vars[key],
                onvalue="1", offvalue="0",
                command=lambda selected_key=key: self.save_checkbox_tuning(
                    selected_key
                ),
            ).grid(
                row=index // 2, column=index % 2, sticky="w", padx=5, pady=2
            )
        ttk.Label(
            search_options,
            text=(
                "둘 다 선택하면 후방 30cm를 먼저 확보한 뒤 YOLO 기준으로 탐색 · "
                "backoff는 safety가 켜져야 동작"
            ),
        ).grid(row=3, column=0, columnspan=2, sticky="w", padx=5, pady=(3, 0))
        self.add_action_button(
            search_options, "체크박스 설정 저장 · 다음 Arrival부터 적용",
            self.save_tuning, row=4, column=0, columnspan=2, padx=5, pady=(5, 2),
        )
        for column in range(2):
            search_options.columnconfigure(column, weight=1)

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
             ("AUTO", "AUTO_SLOT"), ("최근접", "NEAREST"), ("없음", "NONE")),
            0, 6, 5,
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
            ("DOCK 최근접 NORMAL", "DOCK_1", "PICK", "NORMAL", "NEAREST", "AUTO"),
            ("DOCK 최근접 FRESH", "DOCK_1", "PICK", "FRESH", "NEAREST", "AUTO"),
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
        entry_fields = [
            field for field in TUNING_FIELDS if field[0] not in CHECKBOX_KEYS
        ]
        for index, (key, label, _default, _kind, _minimum, _maximum) in enumerate(
            entry_fields
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
        action_row = ((len(entry_fields) - 1) // tuning_columns + 1) * 2
        self.add_action_button(
            tuning, "설정 저장", self.save_tuning,
            row=action_row, column=0, columnspan=tuning_columns, pady=(8, 3),
        )
        ttk.Label(tuning, textvariable=self.tuning_notice).grid(
            row=action_row + 1, column=0, columnspan=tuning_columns, sticky="w", padx=3
        )

        actions = ttk.LabelFrame(controls, text="적재·포크 명령", padding=4)
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
        ttk.Label(
            actions,
            text="완료 상태는 fork_controller 리미트 스위치에서 자동 발행",
        ).grid(
            row=1, column=0, columnspan=5, sticky="w", padx=2, pady=2
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
        if self.is_text_input(event.widget):
            return None
        release_job = self.manual_release_jobs.pop(key, None)
        if release_job is not None:
            self.root.after_cancel(release_job)
        if not self.manual_takeover_active:
            self.node.publish_stop()
            self.manual_takeover_active = True
        self.manual_engaged = True
        if (
            key in {"a", "d"}
            and not ({"a", "d"} & self.manual_keys)
        ):
            if self.node.imu_yaw is not None:
                self.manual_lateral_yaw_target = self.node.imu_yaw
                self.manual_lateral_yaw_source = "imu"
            elif self.node.odom_yaw is not None:
                self.manual_lateral_yaw_target = self.node.odom_yaw
                self.manual_lateral_yaw_source = "odom"
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
        if not ({"a", "d"} & self.manual_keys):
            self.manual_lateral_yaw_target = None
            self.manual_lateral_yaw_source = None
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
        if not self.manual_engaged:
            self.node.publish_cmd_vel()
            self.record_manual_velocity(0.0, 0.0, 0.0)
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
        yaw_hold_active = False
        if (
            linear_y != 0.0
            and angular_z == 0.0
            and self.tuning_vars["manual_lateral_yaw_hold_enabled"].get() == "1"
            and self.manual_lateral_yaw_target is not None
        ):
            current_yaw = (
                self.node.imu_yaw
                if self.manual_lateral_yaw_source == "imu"
                else self.node.odom_yaw
            )
            if current_yaw is None:
                current_yaw = self.manual_lateral_yaw_target
            yaw_error = math.atan2(
                math.sin(self.manual_lateral_yaw_target - current_yaw),
                math.cos(self.manual_lateral_yaw_target - current_yaw),
            )
            try:
                tolerance = math.radians(float(
                    self.tuning_vars[
                        "manual_lateral_yaw_hold_tolerance_deg"
                    ].get()
                ))
                hold_speed = float(
                    self.tuning_vars["manual_lateral_yaw_hold_speed_rad_s"].get()
                )
            except ValueError:
                tolerance = math.radians(3.0)
                hold_speed = 0.35
            if abs(yaw_error) > tolerance:
                linear_y = 0.0
                angular_z = math.copysign(hold_speed, yaw_error)
                yaw_hold_active = True
        self.node.publish_cmd_vel(linear_x, linear_y, angular_z)
        self.record_manual_velocity(linear_x, linear_y, angular_z)
        keys = "+".join(sorted(self.manual_keys)) or "정지"
        hold_text = " · YAW HOLD" if yaw_hold_active else ""
        self.manual_status.set(
            f"{keys.upper()} · x={linear_x:+.2f} y={linear_y:+.2f} "
            f"yaw={angular_z:+.2f}{hold_text}"
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
        self.manual_lateral_yaw_target = None
        self.manual_lateral_yaw_source = None
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
            value = payload.get(key, default)
            if key in CHECKBOX_KEYS:
                if isinstance(value, str):
                    value = value.strip().lower() in {"1", "true", "yes", "on"}
                self.tuning_vars[key].set("1" if bool(value) else "0")
            else:
                self.tuning_vars[key].set(str(value))

    def save_tuning(self):
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
        self.apply_rear_lateral_gain()

    def apply_rear_lateral_gain(self):
        try:
            gain = float(self.tuning_vars["rear_lateral_gain"].get())
        except ValueError:
            return

        def applied(successful, reason):
            if successful:
                self.tuning_notice.set(
                    f"저장 완료 · 뒤 바퀴 횡이동 배율 {gain:.2f} 즉시 적용"
                )
                self.append_log("CFG", "/odom_publisher", f"rear_lateral_gain={gain:.2f}")
            else:
                detail = reason or "파라미터 서비스 응답 없음"
                self.tuning_notice.set(f"배율 적용 실패: {detail}")
                self.append_log("ERR", "/odom_publisher", detail)

        self.node.set_rear_lateral_gain(gain, applied)

    def save_checkbox_tuning(self, key):
        try:
            payload = {}
            if self.config_path.exists():
                payload = json.loads(self.config_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("설정 JSON은 object여야 합니다")
            value = int(self.tuning_vars[key].get())
            payload[key] = value
            temporary = self.config_path.with_suffix(self.config_path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, self.config_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.tuning_notice.set(f"체크박스 저장 실패: {exc}")
            return
        self.tuning_notice.set(
            f"{key}={value} 자동 저장 · STOP 후 다음 Arrival부터 적용"
        )
        self.append_log("CFG", str(self.config_path), f"{key}={value}")

    def target_payload(self):
        kind = self.target_type.get()
        if kind == "SYMBOLS":
            return {"type": kind, "left": self.left_symbol.get(), "right": self.right_symbol.get()}
        if kind == "SLOT":
            return {"type": kind, "slot_id": self.slot_id.get().strip()}
        if kind == "AUTO_SLOT":
            return {"type": kind}
        if kind == "NEAREST":
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
        self.manual_takeover_active = False
        self.node.publish_arrival(self.arrival_payload())

    def publish_scenario(self, values):
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
        self.arrival_status.set("SUCCEEDED")
        self.location.set("NORMAL")
        self.operation.set("PLACE")
        self.product_type.set("NORMAL")
        self.target_type.set("SLOT")
        self.slot_id.set(str(slot_id).upper())
        self.manual_takeover_active = False
        self.node.publish_arrival(self.arrival_payload())

    def publish_fork_command(self, command):
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

    @staticmethod
    def record_timestamp():
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

    def write_lidar_record(self, payload):
        if self.lidar_record_file is None:
            return
        payload = {"recorded_at": self.record_timestamp(), **payload}
        try:
            self.lidar_record_file.write(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
            )
            self.lidar_record_file.flush()
        except OSError as exc:
            self.stop_lidar_recording(f"기록 실패: {exc}")

    def toggle_lidar_recording(self):
        if self.lidar_record_file is None:
            self.start_lidar_recording()
        else:
            self.stop_lidar_recording()

    def start_lidar_recording(self):
        record_dir = Path(self.args.lidar_record_dir).expanduser()
        filename = datetime.now().strftime(
            f"vehicle_{self.args.vehicle}_lidar_%Y%m%d_%H%M%S_%f.jsonl"
        )
        path = record_dir / filename
        try:
            record_dir.mkdir(parents=True, exist_ok=True)
            self.lidar_record_file = path.open("x", encoding="utf-8", buffering=1)
        except OSError as exc:
            self.lidar_record_status.set(f"기록 시작 실패: {exc}")
            self.append_log("ERR", "lidar_record", str(exc))
            return
        self.lidar_record_path = path
        self.lidar_record_scan_count = 0
        self.lidar_record_cmd_count = 0
        self.lidar_record_button.configure(text="LiDAR 기록 종료")
        self.lidar_record_status.set(f"기록 중: {path.name}")
        self.write_lidar_record({
            "event": "recording_started",
            "vehicle": self.args.vehicle,
            "scan_topic": "/scan_raw",
            "cmd_vel_topic": "/controller/cmd_vel",
            "max_range_cm": 30.0,
        })
        self.append_log("REC", "lidar_record", str(path))

    def stop_lidar_recording(self, status=None):
        record_file = self.lidar_record_file
        if record_file is None:
            return
        self.lidar_record_file = None
        if status is None:
            try:
                payload = {
                    "recorded_at": self.record_timestamp(),
                    "event": "recording_stopped",
                    "scan_count": self.lidar_record_scan_count,
                    "manual_cmd_count": self.lidar_record_cmd_count,
                }
                record_file.write(
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                    + "\n"
                )
                record_file.flush()
            except OSError as exc:
                status = f"기록 종료 실패: {exc}"
        try:
            record_file.close()
        except OSError:
            pass
        self.lidar_record_button.configure(text="LiDAR 기록 시작")
        summary = status or (
            f"저장 완료: {self.lidar_record_path} "
            f"(scan {self.lidar_record_scan_count}, cmd {self.lidar_record_cmd_count})"
        )
        self.lidar_record_status.set(summary)
        self.append_log("REC", "lidar_record", summary)

    def record_manual_velocity(self, linear_x, linear_y, angular_z):
        if self.lidar_record_file is None:
            return
        self.lidar_record_cmd_count += 1
        self.write_lidar_record({
            "event": "manual_cmd_vel",
            "keys": sorted(self.manual_keys),
            "linear_x_m_s": round(float(linear_x), 4),
            "linear_y_m_s": round(float(linear_y), 4),
            "angular_z_rad_s": round(float(angular_z), 4),
        })

    def lidar_nearest_by_direction(self, message, apply_self_mask):
        def setting(key, default):
            try:
                return float(self.tuning_vars[key].get())
            except (KeyError, TypeError, ValueError):
                return default

        nearest = {
            direction: math.inf
            for direction in ("front", "rear", "left", "right")
        }
        minimum = max(float(message.range_min), 0.03)
        front_half_angle = math.radians(setting(
            "lidar_self_mask_front_half_angle_deg", 20.0
        ))
        front_max_range = setting("lidar_self_mask_front_max_range_m", 0.20)
        fixed_center = math.radians(setting(
            "lidar_self_mask_fixed_angle_deg", -1.43
        ))
        fixed_half_width = math.radians(setting(
            "lidar_self_mask_fixed_half_width_deg", 1.0
        ))
        for index, raw_distance in enumerate(message.ranges):
            distance = float(raw_distance)
            if (
                not math.isfinite(distance)
                or distance < minimum
                or distance > float(message.range_max)
            ):
                continue
            angle = float(message.angle_min) + index * float(message.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if apply_self_mask:
                fixed_error = math.atan2(
                    math.sin(angle - fixed_center), math.cos(angle - fixed_center)
                )
                if fixed_half_width > 0.0 and abs(fixed_error) <= fixed_half_width:
                    continue
                if abs(angle) <= front_half_angle and distance <= front_max_range:
                    continue
            x, y = math.cos(angle), math.sin(angle)
            if abs(x) >= abs(y):
                direction = "front" if x >= 0.0 else "rear"
            else:
                direction = "left" if y >= 0.0 else "right"
            nearest[direction] = min(nearest[direction], distance)
        return nearest

    def update_lidar_ranges(self, message):
        raw_nearest = self.lidar_nearest_by_direction(message, False)
        filtered_nearest = self.lidar_nearest_by_direction(message, True)

        def shown(nearest, direction):
            distance = nearest.get(direction, math.inf)
            return "---" if not math.isfinite(distance) else f"{distance * 100.0:.1f}cm"

        text = (
            f"원본  전 {shown(raw_nearest, 'front')}  후 {shown(raw_nearest, 'rear')}  "
            f"좌 {shown(raw_nearest, 'left')}  우 {shown(raw_nearest, 'right')}\n"
            f"필터  전 {shown(filtered_nearest, 'front')}  후 {shown(filtered_nearest, 'rear')}  "
            f"좌 {shown(filtered_nearest, 'left')}  우 {shown(filtered_nearest, 'right')}"
        )
        self.lidar_ranges.set(text)
        now = time.monotonic()
        if now - self.last_lidar_log_at >= 1.0:
            self.last_lidar_log_at = now
            self.append_log("SUB", "/scan_raw", text)
        if self.lidar_record_file is not None:
            points = []
            minimum = max(float(message.range_min), 0.03)
            for index, distance in enumerate(message.ranges):
                distance = float(distance)
                if not math.isfinite(distance) or distance < minimum or distance > 0.30:
                    continue
                angle = float(message.angle_min) + index * float(message.angle_increment)
                points.append({
                    "index": index,
                    "angle_deg": round(math.degrees(angle), 3),
                    "range_cm": round(distance * 100.0, 2),
                })
            self.lidar_record_scan_count += 1
            self.write_lidar_record({
                "event": "scan",
                "ros_stamp_sec": int(message.header.stamp.sec),
                "ros_stamp_nanosec": int(message.header.stamp.nanosec),
                "frame_id": message.header.frame_id,
                "raw_nearest_cm": {
                    direction: (
                        None if not math.isfinite(distance)
                        else round(float(distance) * 100.0, 2)
                    )
                    for direction, distance in raw_nearest.items()
                },
                "filtered_nearest_cm": {
                    direction: (
                        None if not math.isfinite(distance)
                        else round(float(distance) * 100.0, 2)
                    )
                    for direction, distance in filtered_nearest.items()
                },
                "points_within_30cm": points,
            })

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

    def switch_to_control_gui(self):
        candidates = (
            Path("/home/ubuntu/ros2_ws/tools/vehicle_camera_teleop_gui.py"),
            Path("/shared/vehicle_camera_teleop_gui.py"),
            Path(__file__).resolve().with_name("vehicle_camera_teleop_gui.py"),
        )
        target = next((path for path in candidates if path.exists()), None)
        if target is None:
            self.tuning_notice.set("Control GUI 파일을 찾지 못했습니다")
            return
        domain = self.args.ros_domain_id or 214 + self.args.vehicle
        self.replacement_command = [
            sys.executable, str(target),
            "--vehicle", str(self.args.vehicle),
            "--ros-domain-id", str(domain),
            "--image-topic", self.args.image_topic,
            "--pose-config", str(self.config_path),
        ]
        self.close()

    @staticmethod
    def launch_replacement(command):
        subprocess.Popen(
            [
                "/bin/bash", "-lc",
                'sleep 0.8; exec "$@"', "gui-switch", *command,
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            close_fds=True,
        )

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop_manual_drive()
        self.stop_lidar_recording()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        replacement = self.replacement_command
        self.root.destroy()
        if replacement is not None:
            self.launch_replacement(replacement)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--config", default="/shared/vehicle_pose_config.json")
    parser.add_argument(
        "--lidar-record-dir", default="/shared/lidar_records",
        help="directory for manual-search LiDAR JSONL recordings",
    )
    parser.add_argument(
        "--image-topic", default="/ascamera/camera_publisher/rgb0/image"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id or 214 + args.vehicle)
    rclpy.init()
    root = tk.Tk()
    panel = TestPanel(root, args)

    def request_close(_signum=None, _frame=None):
        try:
            root.after_idle(panel.close)
        except tk.TclError:
            pass

    signal.signal(signal.SIGINT, request_close)
    signal.signal(signal.SIGTERM, request_close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        panel.close()


if __name__ == "__main__":
    main()
