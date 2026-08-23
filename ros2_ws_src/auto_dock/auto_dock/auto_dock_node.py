#!/usr/bin/env python3
"""GUI-free ROS node for pallet search, alignment, insertion, and handoff.

The 1.1 implementation lived inside ``TeleopWindow`` and a headless runner
instantiated Qt only to reuse that GUI state.  This node owns the autonomous
state machine directly; GUI/UI programs are optional ROS clients.
"""

import json
import math
import os
import socket
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String


SYMBOLS = {"star", "diamond", "spade", "clover", "heart"}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_direction(angle):
    """Return the dominant body direction for a LiDAR bearing."""
    x = math.cos(angle)
    y = math.sin(angle)
    if abs(x) >= abs(y):
        return "front" if x >= 0.0 else "rear"
    return "left" if y >= 0.0 else "right"


class AutoDockNode(Node):
    """ROS-only docking controller; no Qt widget or UI state is used."""

    def __init__(self):
        super().__init__("auto_dock")
        # 0 means infer the vehicle namespace from the DDS domain.
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("pose_config", "/shared/vehicle_pose_config.json")
        self.declare_parameter("config_overrides", "{}")
        self.declare_parameter("search_linear_speed_m_s", 0.0)
        self.declare_parameter("trigger_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("stop_topic", "")
        self.declare_parameter("entry_complete_topic", "")
        self.declare_parameter("lift_up_complete_topic", "")
        self.declare_parameter("drive_ready_topic", "")
        self.declare_parameter("entity_map_topic", "")
        self.declare_parameter("nav_approach_goal_topic", "")
        self.declare_parameter("nav_approach_result_topic", "")
        self.declare_parameter("detection_topic", "")
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("cmd_vel_topic", "/controller/cmd_vel")
        self.declare_parameter("fork_command_topic", "/fork/command")
        self.declare_parameter("control_host", "127.0.0.1")
        self.declare_parameter("control_port", 8091)

        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError(
                "vehicle must be 1/2, or ROS_DOMAIN_ID must be 215 (vehicle 1) "
                "or 216 (vehicle 2) when vehicle=0"
            )
        robot = f"/robot_{self.vehicle}"
        self.pose_config_path = Path(str(self.get_parameter("pose_config").value))
        self.config_overrides = self.parse_overrides(
            str(self.get_parameter("config_overrides").value)
        )
        temporary_search_speed = float(
            self.get_parameter("search_linear_speed_m_s").value
        )
        if temporary_search_speed > 0.0:
            self.config_overrides["search_linear_speed_m_s"] = temporary_search_speed
        self.trigger_topic = self.topic_or_default("trigger_topic", f"{robot}/nav2/arrival")
        self.status_topic = self.topic_or_default("status_topic", f"{robot}/auto_dock/status")
        self.stop_topic = self.topic_or_default("stop_topic", f"{robot}/auto_dock/stop")
        self.entry_complete_topic = self.topic_or_default(
            "entry_complete_topic", f"{robot}/auto_dock/entry_complete"
        )
        self.lift_up_complete_topic = self.topic_or_default(
            "lift_up_complete_topic", f"{robot}/lift/up_complete"
        )
        self.drive_ready_topic = self.topic_or_default(
            "drive_ready_topic", f"{robot}/auto_dock/drive_ready"
        )
        self.entity_map_topic = self.topic_or_default(
            "entity_map_topic", f"{robot}/tag_entity_map"
        )
        self.nav_approach_goal_topic = self.topic_or_default(
            "nav_approach_goal_topic", f"{robot}/nav2/approach_goal"
        )
        self.nav_approach_result_topic = self.topic_or_default(
            "nav_approach_result_topic", f"{robot}/nav2/approach_result"
        )
        self.detection_topic = self.topic_or_default(
            "detection_topic", f"{robot}/symbol_seg/detections"
        )

        self.config = {}
        self.load_config()
        self.state = "idle"
        self.reason = "ready"
        self.status_signature = None
        self.target_left = "diamond"
        self.target_right = "spade"
        self.latest_detection = None
        self.latest_detection_at = 0.0
        self.latest_entity_map = None
        self.candidate_stop_due_at = None
        self.odom_position = None
        self.odom_yaw = None
        self.nearest_range = math.inf
        self.nearest_angle = None
        self.nearest_by_direction = {
            direction: (math.inf, None)
            for direction in ("front", "rear", "left", "right")
        }
        self.target_world = None
        self.insert_start_position = None
        self.post_lift_reverse_start = None
        self.turn_target_yaw = None
        self.nav_approach_completed = False
        self.backoff_until = None
        self.backoff_command = (0.0, 0.0)
        self.was_docking_before_interrupt = False

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.fork_pub = self.create_publisher(
            String, str(self.get_parameter("fork_command_topic").value), 10
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)
        self.entry_complete_pub = self.create_publisher(
            Empty, self.entry_complete_topic, 10
        )
        self.drive_ready_pub = self.create_publisher(
            Empty, self.drive_ready_topic, 10
        )
        self.nav_approach_goal_pub = self.create_publisher(
            PoseStamped, self.nav_approach_goal_topic, 10
        )
        self.create_subscription(String, self.trigger_topic, self.on_trigger, 10)
        self.create_subscription(Empty, self.stop_topic, self.on_stop, 10)
        self.create_subscription(
            Empty, self.lift_up_complete_topic, self.on_lift_up_complete, 10
        )
        self.create_subscription(
            String, self.nav_approach_result_topic, self.on_nav_approach_result, 10
        )
        self.create_subscription(String, self.detection_topic, self.on_detection, 10)
        self.create_subscription(String, self.entity_map_topic, self.on_entity_map, 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10
        )
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_address = (
            str(self.get_parameter("control_host").value),
            int(self.get_parameter("control_port").value),
        )
        self.timer = self.create_timer(0.05, self.tick)
        self.publish_status("idle", "ready")

    def topic_or_default(self, parameter, default):
        value = str(self.get_parameter(parameter).value).strip()
        return value or default

    def parse_overrides(self, raw):
        try:
            value = json.loads(raw)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("config_overrides is not a JSON object; ignoring it")
            return {}

    def load_config(self):
        data = {}
        try:
            if self.pose_config_path.exists():
                data = json.loads(self.pose_config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"pose config read failed: {exc}")
        data.update(self.config_overrides)
        self.config = data

    def number(self, key, default, minimum=None, maximum=None):
        try:
            value = float(self.config.get(key, default))
        except (TypeError, ValueError):
            value = default
        if minimum is not None:
            value = max(minimum, value)
        if maximum is not None:
            value = min(maximum, value)
        return value

    def publish_status(self, state, reason, **extra):
        signature = (state, reason, tuple(sorted(extra.items())))
        if signature == self.status_signature:
            return
        self.status_signature = signature
        self.status_pub.publish(String(data=json.dumps({
            "state": state, "reason": reason,
            "stamp_monotonic": time.monotonic(), **extra,
        }, ensure_ascii=False)))

    def publish_drive(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_drive(self, repeats=5):
        for _ in range(repeats):
            self.publish_drive()

    def send_yolo_target(self):
        payload = {
            "type": "target", "top_left": self.target_left,
            "top_right": self.target_right,
        }
        self.control_socket.sendto(json.dumps(payload).encode("utf-8"), self.control_address)

    def on_trigger(self, msg):
        fields = str(msg.data).strip().split()
        if not fields:
            return
        command = fields[0].lower()
        if command in {"cancel", "stop"}:
            self.cancel("external_cancel")
            return
        if command != "arrived":
            self.publish_status("ignored", "trigger_value_mismatch")
            return
        if len(fields) == 3:
            left, right = fields[1].lower(), fields[2].lower()
            if left not in SYMBOLS or right not in SYMBOLS:
                self.publish_status("rejected", "invalid_target_symbols")
                return
            self.target_left, self.target_right = left, right
        elif len(fields) != 1:
            self.publish_status("rejected", "arrival_format")
            return
        self.load_config()
        self.target_world = None
        self.insert_start_position = None
        self.post_lift_reverse_start = None
        self.turn_target_yaw = None
        self.nav_approach_completed = False
        self.candidate_stop_due_at = None
        self.send_yolo_target()
        self.state = "search"
        self.reason = "search_started"
        self.publish_status("running", self.reason, left=self.target_left, right=self.target_right)

    def on_stop(self, _msg):
        self.cancel("emergency_stop")

    def on_lift_up_complete(self, _msg):
        if self.state != "waiting_lift_up":
            self.get_logger().warning(
                "ignoring lift-up completion outside waiting_lift_up state"
            )
            return
        if self.odom_yaw is None:
            self.cancel("odom_missing_after_lift_up")
            return
        if self.odom_position is None:
            self.cancel("odom_missing_after_lift_up")
            return
        self.post_lift_reverse_start = self.odom_position
        self.turn_target_yaw = None
        self.state = "reversing_after_lift"
        self.publish_status("running", "lift_up_complete_reversing")

    def on_nav_approach_result(self, msg):
        if self.state != "waiting_nav_approach":
            return
        try:
            result = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.cancel("invalid_nav_approach_result")
            return
        if result.get("status") != "succeeded":
            self.cancel(f"nav_approach_{result.get('status', 'failed')}")
            return
        self.nav_approach_completed = True
        self.state = "confirm"
        self.publish_status("running", "nav_approach_complete_confirming")

    def cancel(self, reason):
        self.state = "idle"
        self.reason = reason
        self.backoff_until = None
        self.candidate_stop_due_at = None
        self.post_lift_reverse_start = None
        self.turn_target_yaw = None
        self.nav_approach_completed = False
        self.stop_drive(10)
        self.fork_pub.publish(String(data="STOP"))
        self.publish_status("cancelled", reason)

    def on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.latest_detection_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid detection JSON")

    def on_entity_map(self, msg):
        try:
            payload = json.loads(msg.data)
            if payload.get("frame_id") == "map":
                self.latest_entity_map = payload
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid tag entity map JSON")

    def publish_nav_approach_goal(self, candidate):
        entities = (self.latest_entity_map or {}).get("entities") or []
        matrix = tuple(candidate.get("matrix") or [])
        matches = [
            entity for entity in entities
            if tuple(entity.get("matrix") or []) == matrix
        ]
        if not matches:
            return False
        entity = max(
            matches, key=lambda item: float(item.get("last_seen_unix", 0.0))
        )
        center_pose = entity.get("pose") or {}
        face_pose = entity.get("face_pose")
        try:
            yaw = float(center_pose["yaw"])
            if isinstance(face_pose, dict):
                face_x = float(face_pose["x"])
                face_y = float(face_pose["y"])
            else:
                face_to_center = self.number(
                    "pallet_face_to_center_m", 0.065, 0.01, 0.50
                )
                face_x = float(center_pose["x"]) - face_to_center * math.cos(yaw)
                face_y = float(center_pose["y"]) - face_to_center * math.sin(yaw)
        except (KeyError, TypeError, ValueError):
            return False
        standoff = self.number("nav_approach_standoff_m", 0.45, 0.20, 2.0)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = face_x - standoff * math.cos(yaw)
        goal.pose.position.y = face_y - standoff * math.sin(yaw)
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        self.nav_approach_goal_pub.publish(goal)
        return True

    def on_scan(self, msg):
        self.nearest_range = math.inf
        self.nearest_angle = None
        self.nearest_by_direction = {
            direction: (math.inf, None)
            for direction in ("front", "rear", "left", "right")
        }
        self_filter = self.number("lidar_self_filter_distance_m", 0.25, 0.05, 1.0)
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance < max(msg.range_min, self_filter):
                continue
            if distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = normalize_angle(angle)
            direction = scan_direction(angle)
            if distance < self.nearest_by_direction[direction][0]:
                self.nearest_by_direction[direction] = (distance, angle)
            if distance < self.nearest_range:
                self.nearest_range = distance
                self.nearest_angle = angle

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom_position = (float(pose.position.x), float(pose.position.y))
        q = pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def selected_candidate(self):
        detection = self.latest_detection
        if detection is None or time.monotonic() - self.latest_detection_at > 0.8:
            return None, None
        if detection.get("target_top") != [self.target_left, self.target_right]:
            return None, None
        candidate = detection.get("candidate")
        if not isinstance(candidate, dict):
            return None, None
        pnp = candidate.get("pnp")
        if not isinstance(pnp, dict):
            pnp = None
        return candidate, pnp

    def valid_measurement(self):
        candidate, pnp = self.selected_candidate()
        if candidate is None:
            return None, None, "no_selected_candidate"
        frames = int(self.number("stable_detection_frames", 5, 1, 30))
        if int(candidate.get("streak", 0)) < frames:
            return None, None, "unstable_detection"
        if pnp is None:
            return None, None, "invalid_pnp"
        try:
            forward_cm = float(pnp["forward_distance_cm"])
            reprojection = float(pnp.get("reprojection_error_px", 999.0))
            frontal = abs(float(candidate.get("frontal_error", 999.0)))
        except (KeyError, TypeError, ValueError):
            return None, None, "invalid_pnp"
        if not 10.0 <= forward_cm <= 300.0 or reprojection > 3.0 or frontal > 0.35:
            return None, None, "invalid_pnp"
        return candidate, pnp, None

    def update_world_target(self, candidate, pnp, blend_existing=False):
        if self.odom_position is None or self.odom_yaw is None:
            return False
        forward = float(pnp["forward_distance_cm"]) / 100.0
        lateral = -float(pnp.get("lateral_ratio", 0.0)) * forward
        lateral += self.number("centerline_offset_cm", 0.0) / 100.0
        yaw_deg = -float(pnp.get("yaw_deg", 0.0))
        depth_yaw = candidate.get("depth_yaw") or {}
        if depth_yaw.get("yaw_deg") is not None:
            yaw_deg = 0.5 * yaw_deg + 0.5 * float(depth_yaw["yaw_deg"])
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        observed = {
            "x": self.odom_position[0] + c * forward - s * lateral,
            "y": self.odom_position[1] + s * forward + c * lateral,
            "yaw": normalize_angle(self.odom_yaw + math.radians(yaw_deg)),
        }
        if blend_existing and self.target_world is not None:
            alpha = 0.20
            self.target_world["x"] = (1.0 - alpha) * self.target_world["x"] + alpha * observed["x"]
            self.target_world["y"] = (1.0 - alpha) * self.target_world["y"] + alpha * observed["y"]
            yaw_delta = normalize_angle(observed["yaw"] - self.target_world["yaw"])
            self.target_world["yaw"] = normalize_angle(self.target_world["yaw"] + 0.15 * yaw_delta)
        else:
            self.target_world = observed
        return True

    def target_in_body(self):
        if self.target_world is None or self.odom_position is None or self.odom_yaw is None:
            return None
        dx = self.target_world["x"] - self.odom_position[0]
        dy = self.target_world["y"] - self.odom_position[1]
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        return c * dx + s * dy, -s * dx + c * dy, normalize_angle(self.target_world["yaw"] - self.odom_yaw)

    def interrupt_for_lidar(self):
        if self.state in {
            "idle", "waiting_lift_up", "waiting_nav_approach", "safety_backoff"
        }:
            return False
        legacy_clearance = self.number("lidar_stop_distance_m", 0.35, 0.05, 2.0)
        violations = []
        directions = (
            ("rear", self.nearest_by_direction["rear"]),
        ) if self.state == "reversing_after_lift" else self.nearest_by_direction.items()
        for direction, (distance, angle) in directions:
            clearance = self.number(
                f"lidar_{direction}_clearance_m", legacy_clearance, 0.05, 2.0
            )
            if distance < clearance:
                violations.append(
                    (distance - clearance, direction, distance, angle, clearance)
                )
        if not violations:
            return False
        _margin, direction, distance, angle, clearance = min(violations)
        self.nearest_range = distance
        self.nearest_angle = angle
        if self.state in {"reversing_after_lift", "turning_for_drive"}:
            self.stop_drive()
            self.publish_status(
                "waiting", "post_lift_manoeuvre_blocked", direction=direction,
                range_m=round(distance, 3), clearance_m=round(clearance, 3),
            )
            return True
        self.was_docking_before_interrupt = self.state in {"docking", "inserting"}
        self.candidate_stop_due_at = None
        angle = angle or 0.0
        self.backoff_command = (-0.08 * math.cos(angle), -0.08 * math.sin(angle))
        self.backoff_until = time.monotonic() + 0.7
        self.state = "safety_backoff"
        self.publish_status(
            "recovering", "lidar_backoff", direction=direction,
            range_m=round(distance, 3), clearance_m=round(clearance, 3),
        )
        return True

    def tick(self):
        if self.interrupt_for_lidar():
            return
        if self.state == "idle":
            return
        if self.state == "search":
            self.tick_search()
        elif self.state == "confirm":
            self.tick_confirm()
        elif self.state == "docking":
            self.tick_docking()
        elif self.state == "inserting":
            self.tick_inserting()
        elif self.state == "waiting_lift_up":
            self.stop_drive()
        elif self.state == "waiting_nav_approach":
            return
        elif self.state == "reversing_after_lift":
            self.tick_reversing_after_lift()
        elif self.state == "turning_for_drive":
            self.tick_turning_for_drive()
        elif self.state == "safety_backoff":
            self.tick_backoff()

    def tick_search(self):
        candidate, _pnp = self.selected_candidate()
        now = time.monotonic()
        if candidate is not None and self.candidate_stop_due_at is None:
            delay = self.number("candidate_stop_delay_sec", 0.5, 0.0, 5.0)
            self.candidate_stop_due_at = now + delay
            self.publish_status(
                "running", "candidate_stop_scheduled", delay_sec=delay
            )
        if self.candidate_stop_due_at is not None and now >= self.candidate_stop_due_at:
            self.candidate_stop_due_at = None
            self.stop_drive(5)
            self.state = "confirm"
            self.publish_status("running", "candidate_paused_for_confirmation")
            return
        speed = self.number("search_linear_speed_m_s", 0.03, 0.01, 0.30)
        diameter = self.number("search_circle_diameter_m", 1.34, 0.20, 10.0)
        self.publish_drive(speed, 0.0, 2.0 * speed / diameter)

    def tick_confirm(self):
        candidate, pnp, reason = self.valid_measurement()
        if candidate is None:
            raw, _ = self.selected_candidate()
            if raw is None:
                self.state = "search"
                self.publish_status("running", "candidate_lost_resume_search")
                return
            self.stop_drive()
            self.publish_status(
                "running", "candidate_confirmation", measurement_reason=reason
            )
            return
        if not self.update_world_target(candidate, pnp):
            self.cancel("odom_missing")
            return
        if not self.nav_approach_completed:
            self.stop_drive(5)
            if not self.publish_nav_approach_goal(candidate):
                self.publish_status("waiting", "target_found_waiting_tag_map")
                return
            self.state = "waiting_nav_approach"
            self.publish_status("waiting", "map_goal_sent_waiting_nav2")
            return
        self.nav_approach_completed = False
        self.state = "docking"
        self.publish_status("running", "virtual_target_docking")

    def tick_docking(self):
        candidate, pnp, _ = self.valid_measurement()
        if candidate is not None:
            self.update_world_target(candidate, pnp, blend_existing=True)
        target = self.target_in_body()
        if target is None:
            self.cancel("odom_missing")
            return
        forward, lateral, yaw = target
        standoff = self.number("dock_standoff_m", 0.20, 0.03, 1.0)
        forward_error = forward - standoff
        if abs(forward_error) < 0.035 and abs(lateral) < 0.025 and abs(yaw) < math.radians(3.0):
            self.stop_drive(5)
            self.insert_start_position = self.odom_position
            self.state = "inserting"
            self.publish_status("running", "aligned_inserting")
            return
        self.publish_drive(
            clamp(0.55 * forward_error, -0.10, 0.10),
            clamp(0.80 * lateral, -0.08, 0.08),
            clamp(1.20 * yaw, -0.35, 0.35),
        )

    def tick_inserting(self):
        if self.insert_start_position is None or self.odom_position is None:
            self.cancel("odom_missing")
            return
        insertion_m = self.number("insertion_distance_cm", 12.0, 1.0, 100.0) / 100.0
        travelled = math.dist(self.odom_position, self.insert_start_position)
        if travelled < insertion_m:
            self.publish_drive(0.05, 0.0, 0.0)
            return
        self.stop_drive(10)
        self.state = "waiting_lift_up"
        self.entry_complete_pub.publish(Empty())
        self.publish_status("waiting", "entry_complete_waiting_lift_up")

    def tick_reversing_after_lift(self):
        if self.post_lift_reverse_start is None or self.odom_position is None:
            self.cancel("odom_missing_during_post_lift_reverse")
            return
        reverse_m = self.number(
            "post_lift_reverse_distance_cm", 30.0, 5.0, 200.0
        ) / 100.0
        travelled = math.dist(self.odom_position, self.post_lift_reverse_start)
        if travelled < reverse_m:
            speed = self.number(
                "post_lift_reverse_speed_m_s", 0.05, 0.01, 0.20
            )
            self.publish_drive(-speed, 0.0, 0.0)
            return
        self.stop_drive(10)
        turn_deg = self.number("post_lift_turn_deg", 180.0, -360.0, 360.0)
        self.turn_target_yaw = normalize_angle(
            self.odom_yaw + math.radians(turn_deg)
        )
        self.state = "turning_for_drive"
        self.publish_status("running", "post_lift_reverse_complete_turning", turn_deg=turn_deg)

    def tick_turning_for_drive(self):
        if self.turn_target_yaw is None or self.odom_yaw is None:
            self.cancel("odom_missing_during_post_lift_turn")
            return
        yaw_error = normalize_angle(self.turn_target_yaw - self.odom_yaw)
        tolerance = math.radians(
            self.number("post_lift_turn_tolerance_deg", 3.0, 0.5, 30.0)
        )
        if abs(yaw_error) <= tolerance:
            self.stop_drive(10)
            self.turn_target_yaw = None
            self.state = "idle"
            self.drive_ready_pub.publish(Empty())
            self.publish_status("completed", "drive_ready_after_lift")
            return
        max_speed = self.number(
            "post_lift_turn_speed_rad_s", 0.30, 0.05, 1.0
        )
        self.publish_drive(
            0.0, 0.0, clamp(1.2 * yaw_error, -max_speed, max_speed)
        )

    def tick_backoff(self):
        if self.backoff_until is not None and time.monotonic() < self.backoff_until:
            self.publish_drive(*self.backoff_command, 0.0)
            return
        self.stop_drive(5)
        self.backoff_until = None
        self.state = "docking" if self.was_docking_before_interrupt and self.target_world else "search"
        self.publish_status("running", "lidar_replanned_virtual_dock" if self.state == "docking" else "lidar_recovery_search")

    def destroy_node(self):
        # ROS shutdown may already have invalidated publishers when launch
        # delivers SIGINT.  Do not turn an otherwise clean shutdown into an
        # exception; normal cancel paths still send STOP while ROS is alive.
        if rclpy.ok():
            self.stop_drive(10)
            self.fork_pub.publish(String(data="STOP"))
        self.control_socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoDockNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # launch may deliver a second SIGINT while rclpy destroys the
            # publisher handles; shutdown is already in progress.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
