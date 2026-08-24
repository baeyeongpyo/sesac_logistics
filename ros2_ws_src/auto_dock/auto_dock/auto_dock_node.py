#!/usr/bin/env python3
"""GUI-free ROS node for pallet search, alignment, insertion, and handoff.

The 1.1 implementation lived inside ``TeleopWindow`` and a headless runner
instantiated Qt only to reuse that GUI state.  This node owns the autonomous
state machine directly; GUI/UI programs are optional ROS clients.
"""

import json
import math
import os
import re
import socket
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String


SYMBOLS = {"star", "diamond", "spade", "clover", "heart"}
OPERATIONS = {"PICK", "PLACE"}
PRODUCT_TYPES = {"NORMAL", "FRESH"}
TARGET_TYPES = {"SYMBOLS", "SLOT", "AUTO_SLOT", "NONE"}


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


def parse_arrival(raw):
    """Parse the 1.4 JSON arrival contract or the legacy arrived command."""
    text = str(raw).strip()
    if not text:
        raise ValueError("arrival_empty")
    if not text.startswith("{"):
        fields = text.split()
        if fields[0].lower() in {"cancel", "stop"}:
            return {"cancel": True}
        if fields[0].lower() != "arrived" or len(fields) not in (1, 3):
            raise ValueError("arrival_format")
        target = {"type": "NONE"}
        if len(fields) == 3:
            left, right = fields[1].lower(), fields[2].lower()
            if left not in SYMBOLS or right not in SYMBOLS:
                raise ValueError("invalid_target_symbols")
            target = {"type": "SYMBOLS", "left": left, "right": right}
        return {
            "status": "SUCCEEDED", "location": "DOCK_1",
            "operation": "PICK", "product_type": "NORMAL",
            "target": target, "legacy": True,
        }
    try:
        payload = json.loads(text)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("arrival_json") from exc
    if not isinstance(payload, dict):
        raise ValueError("arrival_json_object")
    status = str(payload.get("status", "")).upper()
    if status != "SUCCEEDED":
        raise ValueError("arrival_not_succeeded")
    location = str(payload.get("location", "")).strip().upper()
    operation = str(payload.get("operation", "")).strip().upper()
    product_type = str(payload.get("product_type", "")).strip().upper()
    if not location:
        raise ValueError("arrival_location")
    if operation not in OPERATIONS:
        raise ValueError("arrival_operation")
    if product_type not in PRODUCT_TYPES:
        raise ValueError("arrival_product_type")
    target = payload.get("target")
    if target is None:
        target = {"type": "NONE"}
    if not isinstance(target, dict):
        raise ValueError("arrival_target")
    target_type = str(target.get("type", "NONE")).strip().upper()
    if target_type not in TARGET_TYPES:
        raise ValueError("arrival_target_type")
    normalized_target = {"type": target_type}
    if target_type == "SYMBOLS":
        left = str(target.get("left", "")).lower()
        right = str(target.get("right", "")).lower()
        if left not in SYMBOLS or right not in SYMBOLS:
            raise ValueError("invalid_target_symbols")
        normalized_target.update(left=left, right=right)
    elif target_type == "SLOT":
        slot_id = str(target.get("slot_id", "")).strip().upper()
        if not slot_id:
            raise ValueError("arrival_slot_id")
        normalized_target["slot_id"] = slot_id
    return {
        "status": status, "location": location, "operation": operation,
        "product_type": product_type, "target": normalized_target,
        "legacy": False,
    }


def normalize_slot_id(zone, slot_id):
    zone = str(zone).upper()
    match = re.fullmatch(
        r"(?:(NORMAL|FRESH)_)?R([1-3])_?C([1-3])",
        str(slot_id).strip().upper(),
    )
    if match is None:
        raise ValueError("arrival_slot_id")
    specified_zone, row, column = match.groups()
    if specified_zone is not None and specified_zone != zone:
        raise ValueError("arrival_slot_zone_mismatch")
    return ZoneOccupancy.slot_id(zone, int(row), int(column))


class ZoneOccupancy:
    """Stabilize per-slot OpenCV observations without treating misses as free."""

    FREE = "FREE"
    OCCUPIED = "OCCUPIED"
    UNKNOWN = "UNKNOWN"
    DIMENSIONS = {
        "NORMAL": (3, 3),
        "FRESH": (3, 3),
        "DOCK": (8, 3),
    }

    def __init__(self, confirmation_frames=3):
        self.confirmation_frames = max(1, int(confirmation_frames))
        self.cells = {}
        self.pending = {}

    @staticmethod
    def slot_id(zone, row, column):
        zone = str(zone).upper()
        if zone not in ZoneOccupancy.DIMENSIONS:
            raise ValueError(f"unsupported storage zone: {zone}")
        rows, columns = ZoneOccupancy.DIMENSIONS[zone]
        if not 1 <= int(row) <= rows or not 1 <= int(column) <= columns:
            raise ValueError(
                f"invalid {zone} {rows}x{columns} slot: "
                f"row={row}, column={column}"
            )
        return f"{zone}_R{int(row)}_C{int(column)}"

    def observe(self, zone, row, column, occupied):
        slot_id = self.slot_id(zone, row, column)
        observed = self.UNKNOWN if occupied is None else (
            self.OCCUPIED if occupied else self.FREE
        )
        previous, count = self.pending.get(slot_id, (None, 0))
        count = count + 1 if previous == observed else 1
        self.pending[slot_id] = (observed, count)
        if observed == self.UNKNOWN:
            self.cells.setdefault(slot_id, self.UNKNOWN)
        elif count >= self.confirmation_frames:
            self.cells[slot_id] = observed
        return self.cells.get(slot_id, self.UNKNOWN)

    def snapshot(self, zone):
        zone = str(zone).upper()
        if zone not in self.DIMENSIONS:
            raise ValueError(f"unsupported storage zone: {zone}")
        rows, columns = self.DIMENSIONS[zone]
        return {
            self.slot_id(zone, row, column): self.cells.get(
                self.slot_id(zone, row, column), self.UNKNOWN
            )
            for row in range(1, rows + 1)
            for column in range(1, columns + 1)
        }


class SlotSelector:
    """Choose a confirmed slot in each zone's configured deep-first order."""

    COLUMN_ORDER = {
        "NORMAL": (1, 2, 3),
        "FRESH": (3, 2, 1),
    }

    def priority(self, zone):
        zone = str(zone).upper()
        if zone not in self.COLUMN_ORDER:
            raise ValueError(f"unsupported storage zone: {zone}")
        return [
            ZoneOccupancy.slot_id(zone, row, column)
            for row in (3, 2, 1)
            for column in self.COLUMN_ORDER[zone]
        ]

    def select(self, zone, operation, occupancy):
        operation = str(operation).upper()
        desired = {
            "PLACE": ZoneOccupancy.FREE,
            "PICK": ZoneOccupancy.OCCUPIED,
        }.get(operation)
        if desired is None:
            raise ValueError(f"unsupported slot operation: {operation}")
        return next(
            (
                slot_id for slot_id in self.priority(zone)
                if occupancy.get(slot_id, ZoneOccupancy.UNKNOWN) == desired
            ),
            None,
        )


class SlotGridVision:
    """Detect the green 3x3 floor grid and classify its cell interiors."""

    def __init__(self, free_ratio=0.12, occupied_ratio=0.28, warp_size=360):
        self.free_ratio = float(free_ratio)
        self.occupied_ratio = float(occupied_ratio)
        self.warp_size = int(warp_size)

    @staticmethod
    def order_corners(points):
        points = np.asarray(points, dtype=np.float32)
        sums = points.sum(axis=1)
        differences = np.diff(points, axis=1).reshape(-1)
        return np.array([
            points[np.argmin(sums)],
            points[np.argmin(differences)],
            points[np.argmax(sums)],
            points[np.argmax(differences)],
        ], dtype=np.float32)

    def analyze(self, frame, zone):
        if frame is None or frame.size == 0:
            return None, "empty_image"
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, (35, 60, 45), (95, 255, 255))
        kernel_size = max(3, int(round(min(height, width) / 300.0)) | 1)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, "green_grid_not_found"
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 0.015 * width * height:
            return None, "green_grid_too_small"
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        corners = cv2.approxPolyDP(hull, 0.04 * perimeter, True).reshape(-1, 2)
        if len(corners) != 4:
            return None, "green_grid_corners"
        margin = max(2, int(round(min(height, width) * 0.003)))
        if any(
            x <= margin or y <= margin or x >= width - margin or y >= height - margin
            for x, y in corners
        ):
            return None, "green_grid_clipped"
        source = self.order_corners(corners)
        size = self.warp_size
        destination = np.array(
            [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
            dtype=np.float32,
        )
        warped = cv2.warpPerspective(
            frame, cv2.getPerspectiveTransform(source, destination), (size, size)
        )
        warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        cell_size = size // 3
        inset = max(4, int(round(cell_size * 0.23)))
        observations = {}
        for image_row in range(3):
            row = 3 - image_row
            for image_column in range(3):
                column = image_column + 1
                y0 = image_row * cell_size + inset
                y1 = (image_row + 1) * cell_size - inset
                x0 = image_column * cell_size + inset
                x1 = (image_column + 1) * cell_size - inset
                cell = warped_hsv[y0:y1, x0:x1]
                floor_pixels = (cell[:, :, 1] < 55) & (cell[:, :, 2] > 100)
                non_floor_ratio = 1.0 - float(np.mean(floor_pixels))
                occupied = None
                if non_floor_ratio <= self.free_ratio:
                    occupied = False
                elif non_floor_ratio >= self.occupied_ratio:
                    occupied = True
                slot_id = ZoneOccupancy.slot_id(zone, row, column)
                observations[slot_id] = {
                    "occupied": occupied,
                    "non_floor_ratio": round(non_floor_ratio, 3),
                }
        return observations, None


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
        self.declare_parameter("fork_state_topic", "")
        self.declare_parameter("drive_ready_topic", "")
        self.declare_parameter("detection_topic", "")
        self.declare_parameter(
            "slot_image_topic", "/ascamera_hp60c/camera_publisher/rgb0/image_upright"
        )
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
        self.fork_state_topic = self.topic_or_default(
            "fork_state_topic", f"{robot}/fork/state"
        )
        self.drive_ready_topic = self.topic_or_default(
            "drive_ready_topic", f"{robot}/auto_dock/drive_ready"
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
        self.operation = "PICK"
        self.location = "DOCK_1"
        self.product_type = "NORMAL"
        self.load_state = "UNLOADED"
        self.arrival_is_legacy = False
        self.selected_slot_id = None
        self.target_type = "NONE"
        self.zone_occupancy = ZoneOccupancy(
            confirmation_frames=int(
                self.number("slot_confirmation_frames", 3, 1, 30)
            )
        )
        self.slot_selector = SlotSelector()
        self.slot_grid_vision = SlotGridVision(
            free_ratio=self.number("slot_free_non_floor_ratio", 0.12, 0.01, 0.50),
            occupied_ratio=self.number(
                "slot_occupied_non_floor_ratio", 0.28, 0.05, 0.90
            ),
        )
        self.cv_bridge = CvBridge()
        self.last_slot_snapshot = None
        self.latest_detection = None
        self.latest_detection_at = 0.0
        self.candidate_stop_due_at = None
        self.odom_position = None
        self.odom_yaw = None
        self.search_heading_yaw = None
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
        self.create_subscription(String, self.trigger_topic, self.on_trigger, 10)
        self.create_subscription(Empty, self.stop_topic, self.on_stop, 10)
        self.create_subscription(
            Empty, self.lift_up_complete_topic, self.on_lift_up_complete, 10
        )
        self.create_subscription(String, self.fork_state_topic, self.on_fork_state, 10)
        self.create_subscription(String, self.detection_topic, self.on_detection, 10)
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Image, str(self.get_parameter("slot_image_topic").value),
            self.on_slot_image, image_qos,
        )
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

    def boolean(self, key, default):
        value = self.config.get(key, default)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def publish_status(self, state, reason, **extra):
        extra = {
            "operation": self.operation,
            "product_type": self.product_type,
            "location": self.location,
            "load_state": self.load_state,
            "slot_id": self.selected_slot_id,
            **extra,
        }
        signature = (state, reason, json.dumps(extra, sort_keys=True, default=str))
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
        try:
            arrival = parse_arrival(msg.data)
        except ValueError as exc:
            self.publish_status("rejected", str(exc))
            return
        if arrival.get("cancel"):
            self.cancel("external_cancel")
            return
        if self.state not in {"idle", "ready"}:
            self.publish_status("rejected", "arrival_while_busy")
            return
        operation = arrival["operation"]
        if operation == "PICK" and self.load_state != "UNLOADED":
            self.publish_status("rejected", "pick_requires_unloaded")
            return
        if operation == "PLACE" and self.load_state != "LOADED":
            self.publish_status("rejected", "place_requires_loaded")
            return
        self.operation = operation
        self.location = arrival["location"]
        self.product_type = arrival["product_type"]
        self.arrival_is_legacy = bool(arrival["legacy"])
        target = arrival["target"]
        self.target_type = target["type"]
        if target["type"] == "SYMBOLS":
            self.target_left = target["left"]
            self.target_right = target["right"]
        self.selected_slot_id = None
        zone = self.location.split("_", 1)[0]
        if target["type"] in {"SLOT", "AUTO_SLOT"}:
            if zone not in {"NORMAL", "FRESH"}:
                self.publish_status("rejected", "slot_target_requires_storage_zone")
                return
            if target["type"] == "SLOT":
                try:
                    self.selected_slot_id = normalize_slot_id(zone, target["slot_id"])
                except ValueError as exc:
                    self.publish_status("rejected", str(exc))
                    return
                self.state = "slot_target_ready"
                self.publish_status("waiting", "slot_execution_not_implemented")
            else:
                self.state = "slot_scanning"
                self.publish_status("waiting", "slot_grid_scanning")
            return
        self.load_config()
        self.target_world = None
        self.insert_start_position = None
        self.post_lift_reverse_start = None
        self.turn_target_yaw = None
        self.candidate_stop_due_at = None
        self.search_heading_yaw = self.odom_yaw
        self.send_yolo_target()
        self.state = "search"
        self.reason = "search_started"
        self.publish_status(
            "running", self.reason, left=self.target_left,
            right=self.target_right, target_type=target["type"],
        )

    def on_stop(self, _msg):
        self.cancel("emergency_stop")

    def on_lift_up_complete(self, _msg):
        if self.state not in {"waiting_lift_up", "waiting_fork"} or self.operation != "PICK":
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
        self.finish_fork_operation("UP_COMPLETE", legacy=True)

    def on_fork_state(self, msg):
        if self.state != "waiting_fork":
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.cancel("invalid_fork_state")
            return
        fork_state = str(payload.get("state", "")).upper()
        if fork_state == "FAILED":
            self.cancel(f"fork_failed:{payload.get('error', '')}")
            return
        expected = "UP_COMPLETE" if self.operation == "PICK" else "DOWN_COMPLETE"
        if fork_state != expected:
            self.get_logger().warning(
                f"ignoring {fork_state or 'empty'} while waiting for {expected}"
            )
            return
        self.finish_fork_operation(fork_state)

    def finish_fork_operation(self, fork_state, legacy=False):
        if self.odom_yaw is None or self.odom_position is None:
            self.cancel("odom_missing_after_fork")
            return
        self.load_state = "LOADED" if fork_state == "UP_COMPLETE" else "UNLOADED"
        self.post_lift_reverse_start = self.odom_position
        self.turn_target_yaw = None
        self.state = "reversing_after_lift"
        self.publish_status(
            "running", "fork_complete_reversing", fork_state=fork_state,
            legacy=legacy,
        )

    def cancel(self, reason):
        self.state = "idle"
        self.reason = reason
        self.backoff_until = None
        self.candidate_stop_due_at = None
        self.post_lift_reverse_start = None
        self.turn_target_yaw = None
        self.stop_drive(10)
        self.fork_pub.publish(String(data="STOP"))
        self.publish_status("cancelled", reason)

    def on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.latest_detection_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid detection JSON")

    def on_slot_image(self, msg):
        if self.state not in {"slot_scanning", "slot_target_ready"}:
            return
        zone = self.location.split("_", 1)[0]
        if zone not in {"NORMAL", "FRESH"}:
            return
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.publish_status("waiting", "slot_image_conversion", error=str(exc))
            return
        observations, error = self.slot_grid_vision.analyze(frame, zone)
        if observations is None:
            self.publish_status("waiting", error)
            return
        ratios = {}
        for slot_id, observation in observations.items():
            self.zone_occupancy.observe(
                zone, *self.slot_row_column(slot_id), observation["occupied"]
            )
            ratios[slot_id] = observation["non_floor_ratio"]
        snapshot = self.zone_occupancy.snapshot(zone)
        if self.target_type == "AUTO_SLOT":
            self.selected_slot_id = self.slot_selector.select(
                zone, self.operation, snapshot
            )
        if snapshot == self.last_slot_snapshot and self.selected_slot_id is None:
            return
        self.last_slot_snapshot = snapshot
        reason = "slot_selected_execution_not_implemented" if self.selected_slot_id else "slot_grid_confirming"
        self.publish_status(
            "waiting", reason, occupancy=snapshot, ratios=ratios,
        )

    @staticmethod
    def slot_row_column(slot_id):
        match = re.search(r"_R([1-3])_C([1-3])$", slot_id)
        return int(match.group(1)), int(match.group(2))

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
        frames = int(self.number("stable_detection_frames", 2, 1, 30))
        if int(candidate.get("streak", 0)) < frames:
            return None, None, "unstable_detection"
        if pnp is None:
            return None, None, "invalid_pnp"
        depth_measurement = candidate.get("depth_yaw")
        if not isinstance(depth_measurement, dict):
            return None, None, "depth_distance_unavailable"
        try:
            forward_cm = float(depth_measurement["forward_distance_cm"])
            reprojection = float(pnp.get("reprojection_error_px", 999.0))
            frontal = abs(float(candidate.get("frontal_error", 999.0)))
        except (KeyError, TypeError, ValueError):
            return None, None, "depth_distance_unavailable"
        if not 10.0 <= forward_cm <= 300.0:
            return None, None, "invalid_depth_distance"
        if reprojection > 3.0 or frontal > 0.35:
            return None, None, "invalid_pnp"
        return candidate, pnp, None

    def update_world_target(self, candidate, pnp, blend_existing=False):
        if self.odom_position is None or self.odom_yaw is None:
            return False
        depth_measurement = candidate.get("depth_yaw") or {}
        forward = float(depth_measurement["forward_distance_cm"]) / 100.0
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
            "idle", "ready", "waiting_lift_up", "waiting_fork",
            "slot_scanning", "slot_target_ready",
            "safety_backoff"
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
        elif self.state in {"waiting_lift_up", "waiting_fork"}:
            self.stop_drive()
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
            delay = self.number("candidate_stop_delay_sec", 0.2, 0.0, 5.0)
            self.candidate_stop_due_at = now + delay
            self.publish_status(
                "running", "candidate_stop_scheduled", delay_sec=delay
            )
        if self.candidate_stop_due_at is not None and now >= self.candidate_stop_due_at:
            self.candidate_stop_due_at = None
            self.stop_drive(5)
            candidate, pnp, reason = self.valid_measurement()
            if candidate is None:
                self.state = "confirm"
                self.publish_status(
                    "running", "candidate_paused_for_confirmation",
                    measurement_reason=reason,
                )
                return
            if not self.update_world_target(candidate, pnp):
                self.cancel("odom_missing")
                return
            self.state = "docking"
            self.publish_status("running", "virtual_target_locked_docking")
            return
        fallback_speed = self.number("search_linear_speed_m_s", 0.03, 0.01, 0.30)
        lateral_speed = self.number(
            "search_lateral_speed_m_s", fallback_speed, 0.01, 0.30
        )
        direction = str(
            self.config.get("search_lateral_direction", "left")
        ).strip().lower()
        direction_sign = -1.0 if direction == "right" else 1.0
        if self.search_heading_yaw is None and self.odom_yaw is not None:
            self.search_heading_yaw = self.odom_yaw
        yaw_error = 0.0
        if self.search_heading_yaw is not None and self.odom_yaw is not None:
            yaw_error = normalize_angle(self.search_heading_yaw - self.odom_yaw)
        angular_correction = clamp(1.2 * yaw_error, -0.20, 0.20)
        self.publish_drive(0.0, direction_sign * lateral_speed, angular_correction)

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
        self.state = "docking"
        self.publish_status("running", "virtual_target_locked_docking")

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
            0.0,
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
            insertion_speed = self.number(
                "insertion_speed_m_s", 0.05, 0.01, 0.20
            )
            self.publish_drive(insertion_speed, 0.0, 0.0)
            return
        self.stop_drive(10)
        command = "UP" if self.operation == "PICK" else "DOWN"
        self.state = "waiting_fork"
        self.fork_pub.publish(String(data=command))
        if self.arrival_is_legacy and command == "UP":
            self.entry_complete_pub.publish(Empty())
        self.publish_status("waiting", "fork_command_sent", fork_command=command)

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
            self.state = "ready"
            self.drive_ready_pub.publish(Empty())
            self.publish_status("completed", "drive_ready_after_fork")
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
