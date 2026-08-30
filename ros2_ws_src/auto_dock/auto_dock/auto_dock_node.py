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
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Empty, String


SYMBOLS = {"star", "diamond", "spade", "clover", "heart"}
OPERATIONS = {"PICK", "PLACE"}
PRODUCT_TYPES = {"NORMAL", "FRESH"}
TARGET_TYPES = {"SYMBOLS", "NEAREST", "SLOT", "AUTO_SLOT", "NONE"}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def public_fsm_state(internal_state, operation, event_state="", was_docking=False):
    """Translate implementation-only phases into the agreed ROS status state."""
    if event_state == "cancelled":
        return "ERROR"
    if internal_state == "waiting_fork":
        return "WAIT_UP_COMPLETE" if operation == "PICK" else "WAIT_DOWN_COMPLETE"
    if internal_state == "safety_backoff":
        return "ALIGNING" if was_docking else "SEARCHING"
    return {
        "idle": "IDLE",
        "ready": "READY",
        "scan_sweep": "SEARCHING",
        "scan_forward_search": "SEARCHING",
        "scan_approach": "ALIGNING",
        "search": "SEARCHING",
        "confirm": "SEARCHING",
        "coarse_align": "ALIGNING",
        "docking": "ALIGNING",
        "inserting": "INSERTING",
        "reversing_after_lift": "REVERSING",
        "post_lift_opening_search": "SEARCHING",
        "post_lift_opening_reverse": "REVERSING",
        "turning_right_for_ready": "TURNING",
        "slot_target_ready": "SEARCHING",
        "slot_scanning": "SEARCHING",
    }.get(internal_state, "ERROR")


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_direction(angle):
    """Return the dominant body direction for a LiDAR bearing."""
    x = math.cos(angle)
    y = math.sin(angle)
    if abs(x) >= abs(y):
        return "front" if x >= 0.0 else "rear"
    return "left" if y >= 0.0 else "right"


def detect_warning_tape(frame, roi_top_ratio=0.55, minimum_yellow_pixels=600):
    """Detect a mostly horizontal yellow/black warning-tape band."""
    if frame is None or frame.size == 0:
        return None
    height, width = frame.shape[:2]
    roi_top = int(clamp(float(roi_top_ratio), 0.0, 0.90) * height)
    roi = frame[roi_top:height]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(
        hsv, np.asarray((15, 90, 70), dtype=np.uint8),
        np.asarray((42, 255, 255), dtype=np.uint8),
    )
    yellow = cv2.morphologyEx(
        yellow, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    component_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        yellow, connectivity=8
    )
    accepted = np.zeros_like(yellow)
    accepted_components = 0
    for label in range(1, component_count):
        _x, _y, component_width, _component_height, area = stats[label]
        if area < 80 or component_width < 12:
            continue
        accepted[labels == label] = 255
        accepted_components += 1
    ys, xs = np.nonzero(accepted)
    if (
        accepted_components < 2
        or len(xs) < int(minimum_yellow_pixels)
        or int(xs.max()) - int(xs.min()) < int(width * 0.25)
    ):
        return None
    points = np.column_stack((xs, ys)).astype(np.float32)
    vx, vy, line_x, line_y = (
        float(value) for value in cv2.fitLine(
            points, cv2.DIST_L2, 0.0, 0.01, 0.01
        ).reshape(-1)
    )
    if abs(vx) < 1e-6:
        return None
    angle_deg = math.degrees(math.atan2(vy, vx))
    if angle_deg >= 90.0:
        angle_deg -= 180.0
    if angle_deg < -90.0:
        angle_deg += 180.0
    if abs(angle_deg) > 35.0:
        return None
    center_y = line_y + (vy / vx) * (width * 0.5 - line_x) + roi_top
    normal_distance = np.abs(-vy * (xs - line_x) + vx * (ys - line_y))
    band_width_px = float(np.percentile(normal_distance, 90)) * 2.0
    if band_width_px > height * 0.18:
        return None
    return {
        "center_y_ratio": float(center_y / height),
        "angle_deg": float(angle_deg),
        "yellow_pixels": int(len(xs)),
        "component_count": int(accepted_components),
        "band_width_px": round(band_width_px, 1),
    }


def detect_dock_end_markers(frame, minimum_red_pixels=180):
    """Return repeating red DOCK end bands that meet the warning tape."""
    if frame is None or frame.size == 0:
        return {}
    height, width = frame.shape[:2]
    tape = detect_warning_tape(frame)
    if tape is None:
        return {}
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 120, 65), (12, 255, 255)),
        cv2.inRange(hsv, (168, 120, 65), (179, 255, 255)),
    )
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        red, connectivity=8
    )
    minimum_segment_area = max(30, int(minimum_red_pixels * 0.15))
    components = []
    for label in range(1, count):
        x, y, component_width, component_height, area = stats[label]
        center_x, center_y = centroids[label]
        if (
            area < minimum_segment_area
            or component_width < 5
            or component_height < 10
        ):
            continue
        components.append({
            "x": int(x), "y": int(y), "width": int(component_width),
            "height": int(component_height), "area": int(area),
            "center": np.asarray((center_x, center_y), dtype=np.float64),
        })
    markers = {}
    for side in ("left", "right"):
        side_components = [
            item for item in components
            if (
                item["center"][0] <= width * 0.45
                if side == "left" else item["center"][0] >= width * 0.55
            )
        ]
        best = None
        for first_index, first in enumerate(side_components):
            for second in side_components[first_index + 1:]:
                delta = second["center"] - first["center"]
                length = float(np.linalg.norm(delta))
                if length < height * 0.08:
                    continue
                direction = delta / length
                angle_deg = abs(math.degrees(math.atan2(
                    direction[1], direction[0]
                )))
                angle_deg = min(angle_deg, 180.0 - angle_deg)
                if angle_deg < 50.0:
                    continue
                inliers = []
                residuals = []
                for item in side_components:
                    offset = item["center"] - first["center"]
                    residual = abs(
                        direction[0] * offset[1]
                        - direction[1] * offset[0]
                    )
                    tolerance = max(10.0, item["width"] * 0.40)
                    if residual <= tolerance:
                        inliers.append(item)
                        residuals.append(residual)
                y_span = max(i["center"][1] for i in inliers) - min(
                    i["center"][1] for i in inliers
                )
                red_pixels = sum(i["area"] for i in inliers)
                if (
                    len(inliers) < 2
                    or y_span < height * 0.15
                    or red_pixels < int(minimum_red_pixels)
                ):
                    continue
                score = (
                    len(inliers), red_pixels,
                    -float(np.mean(residuals) if residuals else math.inf),
                )
                if best is None or score > best[0]:
                    best = (score, inliers)
        if best is None:
            continue
        inliers = best[1]
        points = np.asarray([item["center"] for item in inliers], np.float32)
        vx, vy, line_x, line_y = (
            float(value) for value in cv2.fitLine(
                points, cv2.DIST_L2, 0.0, 0.01, 0.01
            ).reshape(-1)
        )
        tape_slope = math.tan(math.radians(float(tape["angle_deg"])))
        tape_intercept = float(tape["center_y_ratio"]) * height - tape_slope * width * 0.5
        if abs(vx) < 1e-6:
            intersection_x = line_x
        else:
            marker_slope = vy / vx
            denominator = marker_slope - tape_slope
            if abs(denominator) < 1e-6:
                continue
            marker_intercept = line_y - marker_slope * line_x
            intersection_x = (tape_intercept - marker_intercept) / denominator
        intersection_y = tape_slope * intersection_x + tape_intercept
        if (
            not 0.0 <= intersection_x < width
            or (side == "right" and intersection_x < width * 0.50)
            or (side == "left" and intersection_x > width * 0.50)
        ):
            continue
        red_bottom = max(item["y"] + item["height"] for item in inliers)
        contact_gap = max(30, int(round(height * 0.10)))
        tape_endpoint_gap_px = intersection_y - float(red_bottom)
        if not -contact_gap * 0.25 <= tape_endpoint_gap_px <= contact_gap:
            continue
        x1 = min(item["x"] for item in inliers)
        y1 = min(item["y"] for item in inliers)
        x2 = max(item["x"] + item["width"] for item in inliers)
        y2 = max(item["y"] + item["height"] for item in inliers)
        red_pixels = sum(item["area"] for item in inliers)
        confidence = clamp(
            0.5 * min(1.0, len(inliers) / 4.0)
            + 0.5 * min(1.0, red_pixels / max(float(minimum_red_pixels * 5), 1.0)),
            0.0,
            1.0,
        )
        candidate = {
            "x_px": round(float(intersection_x), 1),
            "y_px": round(float(intersection_y), 1),
            "box": [int(x1), int(y1), int(x2), int(y2)],
            "red_pixels": red_pixels,
            "red_segment_count": len(inliers),
            "tape_endpoint_gap_px": round(tape_endpoint_gap_px, 1),
            "confidence": round(float(confidence), 3),
        }
        markers[side] = candidate
    return markers


def pallet_product_type(matrix):
    """Stars denote fresh goods; every other complete tag matrix is normal."""
    symbols = {str(symbol).strip().lower() for symbol in (matrix or [])}
    return "FRESH" if "star" in symbols else "NORMAL"


class DockInventoryTracker:
    """Build a compact nearest-visible DOCK map from the right end marker."""

    def __init__(
        self,
        depth_edges_cm=(20.0, 30.0, 40.0, 50.0),
        first_row_center_ratio=0.65,
        row_pitch_ratio=1.15,
        maximum_age_sec=3.0,
        nearest_tape_only=False,
        tape_gap_min_ratio=-0.10,
        tape_gap_max_ratio=0.35,
    ):
        self.depth_edges_cm = tuple(float(value) for value in depth_edges_cm)
        self.first_row_center_ratio = float(first_row_center_ratio)
        self.row_pitch_ratio = float(row_pitch_ratio)
        self.maximum_age_sec = float(maximum_age_sec)
        self.nearest_tape_only = bool(nearest_tape_only)
        self.tape_gap_min_ratio = float(tape_gap_min_ratio)
        self.tape_gap_max_ratio = float(tape_gap_max_ratio)
        self.revision = 0
        self.observations = {}
        self.entity_rows = {}
        self.right_end_seen = False
        self.last_markers = {}
        self.rescan_reason = "startup"

    def reset(self, reason="rescan_requested"):
        self.revision += 1
        self.observations = {}
        self.entity_rows = {}
        self.right_end_seen = False
        self.last_markers = {}
        self.rescan_reason = str(reason)

    def depth_column(self, distance_cm):
        if len(self.depth_edges_cm) != 4:
            return None
        for column in range(1, 4):
            if self.depth_edges_cm[column - 1] <= distance_cm < self.depth_edges_cm[column]:
                return column
        return None

    @staticmethod
    def entity_distance_cm(entity):
        for measurement in (entity.get("depth_yaw"), entity.get("pnp")):
            if not isinstance(measurement, dict):
                continue
            try:
                distance = float(measurement["forward_distance_cm"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(distance) and distance > 0.0:
                return distance
        return None

    def observe(
        self, entities, markers, now=None, source_stamp_ns=None,
        tape=None, image_shape=None,
    ):
        now = time.monotonic() if now is None else float(now)
        right_marker = (markers or {}).get("right")
        if right_marker is not None:
            self.right_end_seen = True
        self.last_markers = dict(markers or {})
        if right_marker is None and (
            not self.nearest_tape_only or not self.right_end_seen
        ):
            return self.snapshot(now)
        candidates = []
        widths = []
        for entity in entities or []:
            box = entity.get("image_pallet_box")
            matrix = entity.get("matrix")
            distance_cm = self.entity_distance_cm(entity)
            if (
                not isinstance(box, (list, tuple)) or len(box) != 4
                or not isinstance(matrix, (list, tuple)) or len(matrix) < 4
                or (distance_cm is None and not self.nearest_tape_only)
            ):
                continue
            try:
                x1, _y1, x2, y2 = (float(value) for value in box)
            except (TypeError, ValueError):
                continue
            width = x2 - x1
            if width <= 5.0:
                continue
            tape_gap_ratio = None
            if self.nearest_tape_only:
                if not isinstance(tape, dict) or image_shape is None:
                    continue
                image_height, image_width = image_shape[:2]
                angle_rad = math.radians(float(tape.get("angle_deg", 0.0)))
                line_y = (
                    float(tape["center_y_ratio"]) * image_height
                    + math.tan(angle_rad) * (0.5 * (x1 + x2) - image_width * 0.5)
                )
                tape_gap_ratio = (line_y - y2) / width
                if not (
                    self.tape_gap_min_ratio
                    <= tape_gap_ratio
                    <= self.tape_gap_max_ratio
                ):
                    continue
            widths.append(width)
            candidates.append((
                entity, 0.5 * (x1 + x2), width, distance_cm, tape_gap_ratio
            ))
        if not candidates:
            return self.snapshot(now)
        reference_width = float(np.median(widths))
        right_x = (
            None if right_marker is None else float(right_marker["x_px"])
        )
        nearest_by_row = {}
        candidates.sort(key=lambda item: item[1], reverse=True)
        for entity, center_x, _width, distance_cm, tape_gap_ratio in candidates:
            estimated_row = None
            if right_x is not None:
                normalized = (
                    right_x - center_x
                    - self.first_row_center_ratio * reference_width
                ) / max(self.row_pitch_ratio * reference_width, 1.0)
                estimated_row = int(round(normalized)) + 1
            column = 1 if self.nearest_tape_only else self.depth_column(distance_cm)
            if column is None:
                continue
            entity_id = entity.get("entity_id")
            row = estimated_row
            if self.nearest_tape_only:
                row = (
                    self.entity_rows.get(entity_id)
                    if entity_id is not None else None
                )
            if self.nearest_tape_only and row is None:
                occupied_rows = set(self.observations) | set(nearest_by_row)
                row = (
                    estimated_row if estimated_row is not None
                    else max(occupied_rows, default=0) + 1
                )
                if not 1 <= row <= 8:
                    continue
                occupant = self.observations.get(row)
                if (
                    row in occupied_rows
                    and (occupant or {}).get("entity_id") != entity_id
                ):
                    row = next(
                        (
                            candidate_row
                            for candidate_row in (
                                list(range(row + 1, 9)) + list(range(1, row))
                            )
                            if candidate_row not in occupied_rows
                        ),
                        None,
                    )
                if row is None:
                    continue
                if entity_id is not None:
                    self.entity_rows[entity_id] = row
            if row is None or not 1 <= row <= 8:
                continue
            previous = nearest_by_row.get(row)
            metric = (
                abs(tape_gap_ratio) if self.nearest_tape_only
                else distance_cm
            )
            if previous is None or metric < previous[5]:
                nearest_by_row[row] = (
                    entity, center_x, column, distance_cm, tape_gap_ratio, metric
                )
        for row, values in nearest_by_row.items():
            entity, center_x, column, distance_cm, tape_gap_ratio, _metric = values
            slot_id = ZoneOccupancy.slot_id("DOCK", row, column)
            visibility = float(entity.get("visibility_score", 0.0) or 0.0)
            self.observations[row] = {
                "slot_id": slot_id,
                "row": row,
                "column": column,
                "state": ZoneOccupancy.OCCUPIED,
                "accessible": column == 1,
                "blocked_by": None if column == 1 else f"DOCK_R{row}_C{column - 1}",
                "product_type": pallet_product_type(entity.get("matrix")),
                "entity_id": entity.get("entity_id"),
                "matrix": list(entity.get("matrix") or []),
                "distance_cm": (
                    None if distance_cm is None else round(distance_cm, 1)
                ),
                "tape_gap_ratio": (
                    None if tape_gap_ratio is None else round(tape_gap_ratio, 3)
                ),
                "image_center_x_px": round(center_x, 1),
                "confidence": round(clamp(visibility / 10000.0, 0.0, 1.0), 3),
                "reserved_by": None,
                "observed_at_monotonic": now,
                "source_stamp_ns": source_stamp_ns,
            }
        self.rescan_reason = None
        return self.snapshot(now)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else float(now)
        visible = [
            observation for observation in self.observations.values()
            if now - observation["observed_at_monotonic"] <= self.maximum_age_sec
        ]
        visible.sort(key=lambda observation: observation["row"])
        return {
            "revision": self.revision,
            "right_end_detected": "right" in self.last_markers,
            "left_end_detected": "left" in self.last_markers,
            "right_end_anchor_locked": self.right_end_seen,
            "markers": self.last_markers,
            "visible_nearest": visible,
            "unreported_slots": ZoneOccupancy.UNKNOWN,
            "rescan_reason": self.rescan_reason,
        }


def parse_arrival(raw):
    """Parse the structured JSON arrival contract."""
    text = str(raw).strip()
    if not text:
        raise ValueError("arrival_empty")
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
    """Rectify a coloured 55 cm floor zone and classify its virtual 3x3 cells."""

    ZONE_SIZE_M = 0.55

    def __init__(self, free_ratio=0.12, occupied_ratio=0.48, warp_size=360):
        self.free_ratio = float(free_ratio)
        self.occupied_ratio = float(occupied_ratio)
        self.warp_size = int(warp_size)
        self.last_geometry = None

    @staticmethod
    def order_corners(points):
        """Return deep-left, deep-right, front-right, front-left image points.

        The usual sum/difference shortcut fails for the strong perspective in
        the real vehicle image: its front-left corner can have the smallest
        coordinate sum.  The camera remains upright, so split the deep/front
        pairs by image Y and order each pair by X instead.
        """
        points = np.asarray(points, dtype=np.float32)
        if points.shape != (4, 2):
            raise ValueError("grid_requires_four_corners")
        by_y = points[np.argsort(points[:, 1])]
        deep = by_y[:2]
        front = by_y[2:]
        return np.array([
            deep[np.argmin(deep[:, 0])],
            deep[np.argmax(deep[:, 0])],
            front[np.argmax(front[:, 0])],
            front[np.argmin(front[:, 0])],
        ], dtype=np.float32)

    @staticmethod
    def zone_mask(hsv, zone):
        zone = str(zone).upper()
        if zone == "NORMAL":
            return cv2.inRange(hsv, (90, 70, 35), (140, 255, 255))
        if zone == "FRESH":
            return cv2.inRange(hsv, (35, 60, 45), (95, 255, 255))
        return None

    @staticmethod
    def pose_from_corners(corners, camera_matrix, distortion):
        half = SlotGridVision.ZONE_SIZE_M / 2.0
        object_points = np.asarray([
            (-half, half, 0.0),
            (half, half, 0.0),
            (half, -half, 0.0),
            (-half, -half, 0.0),
        ], dtype=np.float64)
        matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        coefficients = np.asarray(distortion, dtype=np.float64).reshape(-1)
        solved, rotation_vector, translation_vector = cv2.solvePnP(
            object_points,
            np.asarray(corners, dtype=np.float64),
            matrix,
            coefficients,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if not solved:
            return None
        rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_position = (-rotation.T @ translation_vector).reshape(-1)
        optical_forward = (rotation.T @ np.asarray((0.0, 0.0, 1.0))).reshape(-1)
        ground_norm = math.hypot(optical_forward[0], optical_forward[1])
        if ground_norm < 1e-6:
            return None
        heading = optical_forward[:2] / ground_norm
        heading_yaw = math.atan2(heading[1], heading[0])
        perpendicular_error = normalize_angle(math.pi / 2.0 - heading_yaw)
        projected, _ = cv2.projectPoints(
            object_points, rotation_vector, translation_vector,
            matrix, coefficients,
        )
        reprojection_error = float(np.mean(np.linalg.norm(
            projected.reshape(-1, 2) - np.asarray(corners, dtype=np.float64),
            axis=1,
        )))
        return {
            "camera_x_m": float(camera_position[0]),
            "camera_y_m": float(camera_position[1]),
            "camera_height_m": float(abs(camera_position[2])),
            "heading_x": float(heading[0]),
            "heading_y": float(heading[1]),
            "perpendicular_error_rad": float(perpendicular_error),
            "reprojection_error_px": reprojection_error,
        }

    @staticmethod
    def target_measurement(geometry, slot_id):
        match = re.search(r"_R([1-3])_C([1-3])$", str(slot_id))
        if match is None or geometry is None or geometry.get("pose") is None:
            return None
        row, column = (int(value) for value in match.groups())
        cell = SlotGridVision.ZONE_SIZE_M / 3.0
        target_x = (column - 2) * cell
        target_y = (row - 2) * cell
        pose = geometry["pose"]
        heading = np.asarray((pose["heading_x"], pose["heading_y"]), dtype=float)
        left = np.asarray((-heading[1], heading[0]), dtype=float)
        delta = np.asarray((
            target_x - pose["camera_x_m"],
            target_y - pose["camera_y_m"],
        ), dtype=float)
        return {
            "forward_m": float(np.dot(delta, heading)),
            "lateral_m": float(np.dot(delta, left)),
            "yaw_error_rad": float(pose["perpendicular_error_rad"]),
            "target_x_m": float(target_x),
            "target_y_m": float(target_y),
        }

    def analyze(self, frame, zone, camera_matrix=None, distortion=None):
        self.last_geometry = None
        if frame is None or frame.size == 0:
            return None, "empty_image"
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        grid = self.zone_mask(hsv, zone)
        if grid is None:
            return None, "unsupported_slot_zone"
        kernel_size = max(3, int(round(min(height, width) / 300.0)) | 1)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, "slot_grid_not_found"
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 0.015 * width * height:
            return None, "slot_grid_too_small"
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        corners = cv2.approxPolyDP(hull, 0.04 * perimeter, True).reshape(-1, 2)
        if len(corners) != 4:
            return None, "slot_grid_corners"
        margin = max(2, int(round(min(height, width) * 0.003)))
        if any(
            x <= margin or y <= margin or x >= width - margin or y >= height - margin
            for x, y in corners
        ):
            return None, "slot_grid_clipped"
        source = self.order_corners(corners)
        size = self.warp_size
        destination = np.array(
            [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
            dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(source, destination)
        warped = cv2.warpPerspective(frame, homography, (size, size))
        warped_grid = cv2.warpPerspective(grid, homography, (size, size)) > 0
        warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        neutral_floor = (
            (warped_hsv[:, :, 1] < 80) & (warped_hsv[:, :, 2] > 65)
        )
        floor_like = warped_grid | neutral_floor
        pose = None
        if camera_matrix is not None and distortion is not None:
            pose = self.pose_from_corners(source, camera_matrix, distortion)
            if pose is not None and pose["reprojection_error_px"] > 8.0:
                return None, "slot_grid_pose_reprojection"
        self.last_geometry = {
            "corners_px": source.tolist(),
            "pose": pose,
        }
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
                cell_floor = floor_like[y0:y1, x0:x1]
                non_floor_ratio = 1.0 - float(np.mean(cell_floor))
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
        self.declare_parameter("fork_state_topic", "")
        self.declare_parameter("drive_ready_topic", "")
        self.declare_parameter("test_load_state_topic", "")
        self.declare_parameter("detection_topic", "")
        self.declare_parameter("dock_inventory_topic", "")
        self.declare_parameter("dock_inventory_reset_topic", "")
        self.declare_parameter(
            "slot_image_topic", "/ascamera/camera_publisher/rgb0/image"
        )
        self.declare_parameter(
            "slot_camera_info_topic",
            "/ascamera/camera_publisher/rgb0/camera_info",
        )
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("imu_rpy_topic", "/imu/rpy/filtered")
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
        self.fork_state_topic = self.topic_or_default(
            "fork_state_topic", f"{robot}/fork/state"
        )
        self.drive_ready_topic = self.topic_or_default(
            "drive_ready_topic", f"{robot}/auto_dock/drive_ready"
        )
        self.test_load_state_topic = self.topic_or_default(
            "test_load_state_topic", f"{robot}/auto_dock/test/load_state"
        )
        self.detection_topic = self.topic_or_default(
            "detection_topic", f"{robot}/symbol_seg/detections"
        )
        self.dock_inventory_topic = self.topic_or_default(
            "dock_inventory_topic", f"{robot}/dock/inventory"
        )
        self.dock_inventory_reset_topic = self.topic_or_default(
            "dock_inventory_reset_topic", f"{robot}/dock/inventory/reset"
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
                "slot_occupied_non_floor_ratio", 0.48, 0.05, 0.90
            ),
        )
        depth_edges = self.config.get(
            "dock_inventory_depth_edges_cm", [
                self.number("dock_inventory_depth_c1_min_cm", 20.0, 5.0, 200.0),
                self.number("dock_inventory_depth_c1_max_cm", 30.0, 5.0, 200.0),
                self.number("dock_inventory_depth_c2_max_cm", 40.0, 5.0, 200.0),
                self.number("dock_inventory_depth_c3_max_cm", 50.0, 5.0, 200.0),
            ]
        )
        if not isinstance(depth_edges, (list, tuple)) or len(depth_edges) != 4:
            depth_edges = [20.0, 30.0, 40.0, 50.0]
        self.dock_inventory_tracker = DockInventoryTracker(
            depth_edges_cm=depth_edges,
            first_row_center_ratio=self.number(
                "dock_inventory_first_row_center_ratio", 0.65, 0.20, 1.50
            ),
            row_pitch_ratio=self.number(
                "dock_inventory_row_pitch_ratio", 1.15, 0.50, 2.50
            ),
            maximum_age_sec=self.number(
                "dock_inventory_max_age_sec", 3.0, 0.5, 30.0
            ),
            nearest_tape_only=AutoDockNode.boolean(
                self, "dock_inventory_nearest_tape_only", True
            ),
            tape_gap_min_ratio=self.number(
                "dock_inventory_tape_gap_min_ratio", -0.10, -1.0, 1.0
            ),
            tape_gap_max_ratio=self.number(
                "dock_inventory_tape_gap_max_ratio", 0.35, -1.0, 2.0
            ),
        )
        self.last_dock_inventory_scan_at = 0.0
        self.cv_bridge = CvBridge()
        self.slot_camera_matrix = None
        self.slot_distortion = None
        self.last_slot_snapshot = None
        self.latest_slot_geometry = None
        self.latest_detection = None
        self.latest_detection_at = 0.0
        self.latest_tape_guidance = None
        self.latest_tape_guidance_at = 0.0
        self.tape_initial_detection_complete = False
        self.tape_reference = None
        self.tape_recovery_start_position = None
        self.tape_recovery_direction = None
        self.tape_recovery_done = False
        self.candidate_stop_due_at = None
        self.candidate_confirmation_started_at = None
        self.candidate_retry_not_before = 0.0
        self.latest_tape_guidance = None
        self.latest_tape_guidance_at = 0.0
        self.tape_reference = None
        self.coarse_alignment_started_at = None
        self.coarse_depth_fallback_frames = 0
        self.coarse_last_counted_stamp = None
        self.alignment_best_pose = None
        self.alignment_best_score = math.inf
        self.alignment_bad_frames = 0
        self.alignment_good_frames = 0
        self.alignment_last_good_stamp = None
        self.alignment_lost_since = None
        self.alignment_recovery_pose = None
        self.odom_position = None
        self.odom_yaw = None
        self.imu_yaw = None
        self.search_heading_yaw = None
        self.search_heading_source = None
        self.scan_sweep_started_yaw = None
        self.scan_sweep_phase = 0
        self.scan_candidate_seen_at = None
        self.scan_leftmost_tag_yaw = None
        self.scan_forward_started_position = None
        self.scan_forward_phase = 0
        self.nearest_range = math.inf
        self.nearest_angle = None
        self.scan_points = []
        self.scan_updated_at = 0.0
        self.nearest_by_direction = {
            direction: (math.inf, None, 0.0)
            for direction in ("front", "rear", "left", "right")
        }
        self.target_world = None
        self.target_entity_id = None
        self.insert_start_position = None
        self.insert_start_yaw = None
        self.insertion_entry_gap_m = None
        self.insertion_start_due_at = None
        self.fork_command_due_at = None
        self.post_lift_reverse_start = None
        self.post_lift_reverse_start_yaw = None
        self.post_lift_reverse_target_m = None
        self.post_lift_opening_started_at = None
        self.post_lift_opening_reference_m = None
        self.post_lift_opening_previous_m = None
        self.post_lift_opening_confirmation_count = 0
        self.post_lift_opening_reverse_started_at = None
        self.post_lift_opening_heading_yaw = None
        self.post_lift_opening_heading_source = None
        self.state_before_lidar_interrupt = None
        self.completed_insertion_distance_m = None
        self.right_turn_target_yaw = None
        self.right_turn_clearance_wait_started_at = None
        self.backoff_until = None
        self.backoff_command = (0.0, 0.0)
        self.backoff_direction = None
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
        self.dock_inventory_pub = self.create_publisher(
            String, self.dock_inventory_topic, status_qos
        )
        self.drive_ready_pub = self.create_publisher(
            Empty, self.drive_ready_topic, 10
        )
        self.create_subscription(String, self.trigger_topic, self.on_trigger, 10)
        self.create_subscription(Empty, self.stop_topic, self.on_stop, 10)
        self.create_subscription(
            String, self.test_load_state_topic, self.on_test_load_state, 10
        )
        self.create_subscription(String, self.fork_state_topic, self.on_fork_state, 10)
        self.create_subscription(String, self.detection_topic, self.on_detection, 10)
        self.create_subscription(
            Empty, self.dock_inventory_reset_topic,
            self.on_dock_inventory_reset, 10,
        )
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Image, str(self.get_parameter("slot_image_topic").value),
            self.on_slot_image, image_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("slot_camera_info_topic").value),
            self.on_slot_camera_info,
            image_qos,
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("imu_rpy_topic").value),
            self.on_imu_rpy,
            20,
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
        public_state = public_fsm_state(
            self.state, self.operation, state, self.was_docking_before_interrupt
        )
        extra = {
            "operation": self.operation,
            "product_type": self.product_type,
            "location": self.location,
            "load_state": self.load_state,
            "slot_id": self.selected_slot_id,
            **extra,
        }
        signature = (public_state, reason, json.dumps(extra, sort_keys=True, default=str))
        if signature == self.status_signature:
            return
        self.status_signature = signature
        self.status_pub.publish(String(data=json.dumps({
            "state": public_state, "reason": reason,
            "stamp_monotonic": time.monotonic(), **extra,
        }, ensure_ascii=False)))

    def publish_dock_inventory(self, snapshot, reason="observation"):
        payload = {
            "vehicle": self.vehicle,
            "zone": "DOCK",
            "reason": reason,
            "updated_at_monotonic": time.monotonic(),
            **snapshot,
        }
        self.dock_inventory_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )))

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
        target = arrival["target"]
        self.target_type = target["type"]
        if self.target_type == "NEAREST" and operation != "PICK":
            self.publish_status("rejected", "nearest_target_requires_pick")
            return
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
        self.target_entity_id = None
        self.insert_start_position = None
        self.insert_start_yaw = None
        self.insertion_entry_gap_m = None
        self.insertion_start_due_at = None
        self.fork_command_due_at = None
        self.post_lift_reverse_start = None
        self.post_lift_reverse_start_yaw = None
        self.post_lift_reverse_target_m = None
        self.post_lift_opening_started_at = None
        self.post_lift_opening_reference_m = None
        self.post_lift_opening_previous_m = None
        self.post_lift_opening_confirmation_count = 0
        self.post_lift_opening_reverse_started_at = None
        self.post_lift_opening_heading_yaw = None
        self.post_lift_opening_heading_source = None
        self.state_before_lidar_interrupt = None
        self.completed_insertion_distance_m = None
        self.right_turn_target_yaw = None
        self.right_turn_clearance_wait_started_at = None
        self.candidate_stop_due_at = None
        self.candidate_confirmation_started_at = None
        self.candidate_retry_not_before = 0.0
        self.latest_tape_guidance = None
        self.latest_tape_guidance_at = 0.0
        self.tape_initial_detection_complete = False
        self.tape_reference = None
        self.tape_recovery_start_position = None
        self.tape_recovery_direction = None
        self.tape_recovery_done = False
        self.reset_coarse_alignment()
        self.latch_search_heading()
        if self.target_type != "NEAREST":
            self.send_yolo_target()
        if self.boolean("nav2_scan_approach_enabled", False):
            if self.odom_yaw is None:
                self.state = "search"
                self.reason = "scan_approach_odom_missing_fallback_search"
            else:
                self.scan_sweep_started_yaw = self.odom_yaw
                self.scan_sweep_phase = 0
                self.scan_candidate_seen_at = None
                self.scan_leftmost_tag_yaw = None
                self.scan_forward_started_position = None
                self.scan_forward_phase = 0
                self.state = "scan_sweep"
                self.reason = "nav2_scan_sweep_started"
        else:
            self.state = "search"
            self.reason = "search_started"
        self.publish_status(
            "running", self.reason, left=self.target_left,
            right=self.target_right, target_type=target["type"],
        )

    def on_stop(self, _msg):
        self.cancel("emergency_stop")

    def on_dock_inventory_reset(self, _msg):
        self.dock_inventory_tracker.reset("manual_reset")
        self.publish_dock_inventory(
            self.dock_inventory_tracker.snapshot(), reason="manual_reset"
        )

    def on_test_load_state(self, msg):
        requested = msg.data.strip().upper()
        if requested not in {"LOADED", "UNLOADED"}:
            self.publish_status("rejected", "invalid_test_load_state")
            return
        if self.state not in {"idle", "ready"}:
            self.publish_status("rejected", "test_load_state_while_busy")
            return
        self.load_state = requested
        self.publish_status("idle", "test_load_state_override")

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

    def finish_fork_operation(self, fork_state):
        if self.odom_yaw is None or self.odom_position is None:
            self.cancel("odom_missing_after_fork")
            return
        reverse_target = getattr(self, "completed_insertion_distance_m", None)
        if reverse_target is None or reverse_target <= 0.0:
            self.cancel("insertion_distance_missing_after_fork")
            return
        self.load_state = "LOADED" if fork_state == "UP_COMPLETE" else "UNLOADED"
        if (
            hasattr(self, "config")
            and AutoDockNode.boolean(self, "dock_inventory_scan_enabled", False)
            and self.location.split("_", 1)[0] == "DOCK"
        ):
            self.dock_inventory_tracker.reset(
                f"{self.operation.lower()}_fork_complete"
            )
            self.publish_dock_inventory(
                self.dock_inventory_tracker.snapshot(),
                reason="rescan_required_after_fork",
            )
        self.post_lift_reverse_start = self.odom_position
        self.post_lift_reverse_start_yaw = self.odom_yaw
        self.post_lift_reverse_target_m = reverse_target
        self.state = "reversing_after_lift"
        self.publish_status(
            "running", "fork_complete_reversing", fork_state=fork_state,
            reverse_target_cm=round(reverse_target * 100.0, 1),
        )

    def cancel(self, reason):
        self.state = "idle"
        self.reason = reason
        self.backoff_until = None
        self.backoff_direction = None
        self.candidate_stop_due_at = None
        self.candidate_confirmation_started_at = None
        self.candidate_retry_not_before = 0.0
        self.scan_sweep_started_yaw = None
        self.scan_sweep_phase = 0
        self.scan_candidate_seen_at = None
        self.scan_leftmost_tag_yaw = None
        self.scan_forward_started_position = None
        self.scan_forward_phase = 0
        self.tape_recovery_start_position = None
        self.tape_recovery_direction = None
        self.tape_recovery_done = False
        self.reset_coarse_alignment()
        self.post_lift_reverse_start = None
        self.post_lift_reverse_start_yaw = None
        self.post_lift_reverse_target_m = None
        self.post_lift_opening_started_at = None
        self.post_lift_opening_reference_m = None
        self.post_lift_opening_previous_m = None
        self.post_lift_opening_confirmation_count = 0
        self.post_lift_opening_reverse_started_at = None
        self.post_lift_opening_heading_yaw = None
        self.post_lift_opening_heading_source = None
        self.state_before_lidar_interrupt = None
        self.completed_insertion_distance_m = None
        self.right_turn_target_yaw = None
        self.right_turn_clearance_wait_started_at = None
        self.insert_start_yaw = None
        self.insertion_entry_gap_m = None
        self.insertion_start_due_at = None
        self.fork_command_due_at = None
        self.stop_drive(10)
        self.fork_pub.publish(String(data="STOP"))
        self.publish_status("cancelled", reason)

    def on_detection(self, msg):
        try:
            self.latest_detection = json.loads(msg.data)
            self.latest_detection_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid detection JSON")

    def on_slot_camera_info(self, msg):
        if len(msg.k) != 9:
            return
        self.slot_camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.slot_distortion = np.asarray(msg.d, dtype=np.float64)

    def warning_tape_roi_top_ratio(self):
        if not getattr(self, "tape_initial_detection_complete", False):
            return 0.0
        return self.number("tape_roi_top_ratio", 0.55, 0.0, 0.90)

    def warning_tape_initial_approach_complete(self, tape):
        if not isinstance(tape, dict):
            return False
        target = self.number(
            "tape_target_center_y_ratio", 0.65, 0.10, 0.90
        )
        tolerance = self.number(
            "tape_vertical_tolerance_ratio", 0.035, 0.002, 0.10
        )
        return float(tape["center_y_ratio"]) >= target - tolerance

    def on_slot_image(self, msg):
        zone = self.location.split("_", 1)[0]
        tape_due = (
            self.state in {"search", "confirm", "coarse_align", "docking"}
            and (
                AutoDockNode.boolean(self, "tape_guidance_enabled", False)
                or AutoDockNode.boolean(self, "tape_guidance_only", False)
            )
        )
        inventory_enabled = AutoDockNode.boolean(
            self, "dock_inventory_scan_enabled", False
        )
        inventory_due = (
            inventory_enabled
            and zone == "DOCK"
            and self.state in {"idle", "ready", "search", "confirm", "coarse_align"}
            and time.monotonic() - self.last_dock_inventory_scan_at
            >= self.number("dock_inventory_scan_interval_sec", 0.50, 0.20, 5.0)
        )
        grid_due = (
            self.state in {"slot_scanning", "slot_target_ready"}
            and zone in {"NORMAL", "FRESH"}
        )
        if not tape_due and not inventory_due and not grid_due:
            return
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.publish_status("waiting", "camera_image_conversion", error=str(exc))
            return
        tape_inventory_due = (
            inventory_due and self.dock_inventory_tracker.nearest_tape_only
        )
        if tape_due or tape_inventory_due:
            self.latest_tape_guidance = detect_warning_tape(
                frame,
                roi_top_ratio=AutoDockNode.warning_tape_roi_top_ratio(self),
                minimum_yellow_pixels=int(self.number(
                    "tape_min_yellow_pixels", 600, 100, 20000
                )),
            )
            self.latest_tape_guidance_at = time.monotonic()
            if self.latest_tape_guidance is not None:
                self.tape_initial_detection_complete = (
                    self.tape_initial_detection_complete
                    or AutoDockNode.warning_tape_initial_approach_complete(
                        self, self.latest_tape_guidance
                    )
                )
                self.tape_recovery_start_position = None
                self.tape_recovery_direction = None
                self.tape_recovery_done = False
        if inventory_due:
            self.last_dock_inventory_scan_at = time.monotonic()
            markers = detect_dock_end_markers(
                frame,
                minimum_red_pixels=int(self.number(
                    "dock_inventory_minimum_red_pixels", 180, 50, 5000
                )),
            )
            detection = self.latest_detection or {}
            detection_age = time.monotonic() - self.latest_detection_at
            entities = (
                detection.get("entities") or []
                if detection_age <= self.number(
                    "dock_inventory_detection_max_age_sec", 0.80, 0.20, 5.0
                ) else []
            )
            snapshot = self.dock_inventory_tracker.observe(
                entities,
                markers,
                source_stamp_ns=detection.get("source_stamp_ns"),
                tape=self.latest_tape_guidance,
                image_shape=frame.shape,
            )
            self.publish_dock_inventory(snapshot)
        if not grid_due:
            return
        observations, error = self.slot_grid_vision.analyze(
            frame,
            zone,
            camera_matrix=self.slot_camera_matrix,
            distortion=self.slot_distortion,
        )
        if observations is None:
            self.publish_status("waiting", error)
            return
        self.latest_slot_geometry = self.slot_grid_vision.last_geometry
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
        target = self.slot_grid_vision.target_measurement(
            self.latest_slot_geometry, self.selected_slot_id
        )
        reason = "slot_target_pose_ready" if target is not None else (
            "slot_selected_waiting_camera_info"
            if self.selected_slot_id else "slot_grid_confirming"
        )
        target_status = {}
        if target is not None:
            target_status = {
                "slot_forward_cm": round(target["forward_m"] * 100.0, 1),
                "slot_lateral_cm": round(target["lateral_m"] * 100.0, 1),
                "slot_yaw_error_deg": round(
                    math.degrees(target["yaw_error_rad"]), 1
                ),
                "slot_pose_reprojection_px": round(
                    self.latest_slot_geometry["pose"]["reprojection_error_px"], 2
                ),
            }
        self.publish_status(
            "waiting", reason, occupancy=snapshot, ratios=ratios, **target_status,
        )

    @staticmethod
    def slot_row_column(slot_id):
        match = re.search(r"_R([1-3])_C([1-3])$", slot_id)
        return int(match.group(1)), int(match.group(2))

    def on_scan(self, msg):
        self.nearest_range = math.inf
        self.nearest_angle = None
        self.scan_points = []
        self.scan_updated_at = time.monotonic()
        self.nearest_by_direction = {
            direction: (math.inf, None, 0.0)
            for direction in ("front", "rear", "left", "right")
        }
        sensor_radius = self.number("lidar_sensor_radius_m", 0.015, 0.005, 0.10)
        self_filter = max(
            sensor_radius,
            self.number("lidar_self_filter_distance_m", 0.03, 0.01, 1.0),
        )
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance < max(msg.range_min, self_filter):
                continue
            if distance > msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = normalize_angle(angle)
            if self.is_lidar_self_return(angle, distance):
                continue
            self.scan_points.append((distance, angle))
            direction, clearance = self.lidar_safety_threshold(angle)
            previous_distance, _previous_angle, previous_clearance = (
                self.nearest_by_direction[direction]
            )
            if distance - clearance < previous_distance - previous_clearance:
                self.nearest_by_direction[direction] = (
                    distance, angle, clearance
                )
            if distance < self.nearest_range:
                self.nearest_range = distance
                self.nearest_angle = angle

    def is_lidar_self_return(self, angle, distance):
        """Mask stable returns from the vehicle body and a carried pallet."""
        half_angle = math.radians(self.number(
            "lidar_self_mask_front_half_angle_deg", 20.0, 0.0, 90.0
        ))
        max_range = self.number(
            "lidar_self_mask_front_max_range_m", 0.20, 0.0, 1.0
        )
        if abs(normalize_angle(angle)) <= half_angle and distance <= max_range:
            return True
        if str(getattr(self, "load_state", "UNLOADED")).upper() == "LOADED":
            loaded_front = self.number(
                "lidar_loaded_front_extent_m", 0.48, 0.10, 1.50
            )
            loaded_half_width = self.number(
                "lidar_loaded_half_width_m", 0.085, 0.01, 0.50
            )
            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)
            if 0.0 <= point_x <= loaded_front and abs(point_y) <= loaded_half_width:
                return True
        return False

    def right_turn_clearance_available(self):
        """Check the actual rectangular footprint swept through a right turn."""
        if time.monotonic() - getattr(self, "scan_updated_at", 0.0) > 0.5:
            return False, None, None
        body_front = self.number("lidar_body_front_extent_m", 0.30, 0.01, 2.0)
        body_rear = self.number("lidar_body_rear_extent_m", 0.06, 0.01, 2.0)
        # Pure angular cmd_vel rotates around base_footprint, not around the
        # rear-mounted LiDAR.  For the 40 cm body that center is 15 cm ahead
        # of the LiDAR origin.
        center_x = (body_front - body_rear) / 2.0
        front = body_front - center_x
        if str(self.load_state).upper() == "LOADED":
            front = self.number(
                "lidar_loaded_front_extent_m", 0.48, body_front, 1.50
            ) - center_x
        rear = body_rear + center_x
        half_width = self.number("lidar_body_half_width_m", 0.06, 0.01, 1.0)
        edge_clearance = max(
            self.number(f"lidar_{direction}_clearance_m", 0.01, 0.0, 2.0)
            for direction in ("front", "rear", "left", "right")
        )
        blocking = []
        for distance, angle in getattr(self, "scan_points", []):
            point_x = distance * math.cos(angle) - center_x
            point_y = distance * math.sin(angle)
            for step in range(19):
                turn_yaw = -math.radians(90.0) * step / 18.0
                c, s = math.cos(turn_yaw), math.sin(turn_yaw)
                local_x = c * point_x + s * point_y
                local_y = -s * point_x + c * point_y
                if (
                    -rear - edge_clearance <= local_x <= front + edge_clearance
                    and abs(local_y) <= half_width + edge_clearance
                ):
                    blocking.append(distance)
                    break
        nearest_blocking = min(blocking, default=math.inf)
        turn_front_extent = front + edge_clearance
        return not blocking, nearest_blocking, turn_front_extent

    def lidar_safety_threshold(self, angle):
        """Return LiDAR range needed to keep the vehicle edge clear."""
        direction = scan_direction(angle)
        defaults = {"front": 0.01, "rear": 0.01, "left": 0.01, "right": 0.01}
        edge_clearance = self.number(
            f"lidar_{direction}_clearance_m", defaults[direction], 0.0, 2.0
        )
        body_distance = AutoDockNode.lidar_body_boundary_distance(self, angle)
        minimum_stop_distance = self.number(
            "lidar_stop_distance_m", 0.20, 0.05, 2.0
        )
        return direction, max(
            body_distance + edge_clearance, minimum_stop_distance
        )

    def lidar_body_boundary_distance(self, angle):
        """Distance from the LiDAR origin to the 36 x 12 cm vehicle outline."""
        front = self.number("lidar_body_front_extent_m", 0.30, 0.01, 2.0)
        rear = self.number("lidar_body_rear_extent_m", 0.06, 0.01, 2.0)
        half_width = self.number(
            "lidar_body_half_width_m", 0.06, 0.01, 1.0
        )
        x_component = math.cos(angle)
        y_component = math.sin(angle)
        intersections = []
        if x_component > 1e-9:
            intersections.append(front / x_component)
        elif x_component < -1e-9:
            intersections.append(rear / -x_component)
        if abs(y_component) > 1e-9:
            intersections.append(half_width / abs(y_component))
        return min(intersections)

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom_position = (float(pose.position.x), float(pose.position.y))
        q = pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def on_imu_rpy(self, msg):
        self.imu_yaw = float(msg.vector.z)

    def latch_search_heading(self):
        if self.imu_yaw is not None:
            self.search_heading_yaw = self.imu_yaw
            self.search_heading_source = "imu"
        else:
            self.search_heading_yaw = self.odom_yaw
            self.search_heading_source = "odom"

    def search_heading_error(self):
        source = getattr(self, "search_heading_source", None) or "odom"
        current_yaw = (
            getattr(self, "imu_yaw", None)
            if source == "imu" else getattr(self, "odom_yaw", None)
        )
        if self.search_heading_yaw is None or current_yaw is None:
            return None
        return normalize_angle(self.search_heading_yaw - current_yaw)

    def search_angular_command(self, yaw_error):
        minimum = self.number(
            "tag_search_min_angular_speed_rad_s", 0.35, 0.10, 0.50
        )
        maximum = max(minimum, self.number(
            "tag_search_max_angular_speed_rad_s", 0.35, 0.02, 0.50
        ))
        magnitude = clamp(abs(yaw_error) * 0.6, minimum, maximum)
        return math.copysign(magnitude, yaw_error)

    def front_search_pair_yaw_error(self):
        if not all(hasattr(self, name) for name in (
            "target_left", "target_right", "target_entity_id"
        )):
            return None
        candidate, _pnp, reason = AutoDockNode.visible_target_top_pair_measurement(
            self
        )
        if reason is not None or candidate is None:
            return None
        try:
            yaw_deg = float(candidate["depth_yaw"]["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return None
        if not math.isfinite(yaw_deg):
            return None
        return math.radians(yaw_deg)

    def selected_candidate(self):
        detection = self.latest_detection
        if detection is None or time.monotonic() - self.latest_detection_at > 0.8:
            return None, None
        if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
            return self.nearest_product_candidate(detection)
        if detection.get("target_top") != [self.target_left, self.target_right]:
            return None, None
        candidate = detection.get("candidate")
        if not isinstance(candidate, dict):
            return None, None
        pnp = candidate.get("pnp")
        if not isinstance(pnp, dict):
            pnp = None
        # The requested identity contains only the upper tag pair because the
        # lower row disappears at close range.  At longer range that shortcut
        # can combine unrelated visible tags into a false target, so do not
        # allow an upper-pair-only identity lock at or beyond this distance.
        distance_cm = None
        for measurement in (candidate.get("depth_yaw"), pnp):
            if not isinstance(measurement, dict):
                continue
            try:
                value = float(measurement["forward_distance_cm"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                distance_cm = value
                break
        maximum_top_only_distance_cm = self.number(
            "target_top_only_max_distance_cm", 30.0, 5.0, 100.0
        )
        if (
            distance_cm is not None
            and distance_cm >= maximum_top_only_distance_cm
        ):
            return None, None
        return candidate, pnp

    def nearest_product_candidate(self, detection):
        """Return the closest accessible C1 entity of the requested product."""
        candidates = []
        image_width = self.number("camera_image_width_px", 640.0, 100.0, 4000.0)
        tracker = getattr(self, "dock_inventory_tracker", None)
        inventory = tracker.snapshot() if tracker is not None else {}
        visible_inventory = inventory.get("visible_nearest") or []
        observed_entity_ids = {
            item.get("entity_id") for item in visible_inventory
            if item.get("entity_id") is not None
        }
        accessible_entity_ids = {
            item.get("entity_id") for item in visible_inventory
            if (
                item.get("accessible") is True
                and item.get("product_type") == self.product_type
                and item.get("entity_id") is not None
            )
        }
        for entity in detection.get("entities") or []:
            if not isinstance(entity, dict):
                continue
            entity_id = entity.get("entity_id")
            if (
                observed_entity_ids
                and entity_id not in accessible_entity_ids
            ):
                continue
            matrix = entity.get("matrix")
            if (
                not isinstance(matrix, list)
                or len(matrix) < 4
                or pallet_product_type(matrix) != self.product_type
            ):
                continue
            pnp = entity.get("pnp")
            depth_yaw = entity.get("depth_yaw")
            distance_cm = DockInventoryTracker.entity_distance_cm(entity)
            box = entity.get("image_pallet_box")
            if (
                distance_cm is None
                or not isinstance(pnp, dict)
                or not isinstance(box, list)
                or len(box) != 4
            ):
                continue
            center_x = 0.5 * (float(box[0]) + float(box[2]))
            candidate = {
                "entity_id": entity_id,
                "matrix": list(matrix),
                "streak": int(entity.get("seen_count", 1)),
                "center_error": (
                    center_x - image_width * 0.5
                ) / max(image_width * 0.5, 1.0),
                "frontal_error": float(entity.get("frontal_error", 0.0)),
                "top_row_error": float(entity.get("top_row_error", 0.0)),
                "bottom_row_error": float(entity.get("bottom_row_error", 0.0)),
                "pallet_box": list(box),
                "pnp": pnp,
                "depth_yaw": depth_yaw,
            }
            candidates.append((distance_cm, abs(candidate["center_error"]), candidate))
        if not candidates:
            return None, None
        _distance, _center_error, candidate = min(
            candidates, key=lambda item: (item[0], item[1])
        )
        return candidate, candidate["pnp"]

    @staticmethod
    def box_iou(left, right):
        if not (
            isinstance(left, (list, tuple)) and len(left) == 4
            and isinstance(right, (list, tuple)) and len(right) == 4
        ):
            return 0.0
        x1 = max(float(left[0]), float(right[0]))
        y1 = max(float(left[1]), float(right[1]))
        x2 = min(float(left[2]), float(right[2]))
        y2 = min(float(left[3]), float(right[3]))
        intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        left_area = max(0.0, float(left[2]) - float(left[0])) * max(
            0.0, float(left[3]) - float(left[1])
        )
        right_area = max(0.0, float(right[2]) - float(right[0])) * max(
            0.0, float(right[3]) - float(right[1])
        )
        union = left_area + right_area - intersection
        return intersection / union if union > 0.0 else 0.0

    def candidate_matches_best_entity(self, candidate):
        detection = self.latest_detection or {}
        entities = detection.get("entities") or []
        candidate_box = candidate.get("pallet_box")
        minimum_iou = self.number("entity_confirmation_iou_min", 0.50, 0.1, 1.0)
        overlapping = [
            entity for entity in entities
            if isinstance(entity, dict)
            and AutoDockNode.box_iou(
                candidate_box, entity.get("image_pallet_box")
            ) >= minimum_iou
        ]
        if not overlapping:
            return False, "entity_confirmation_unavailable"
        best = max(
            overlapping, key=lambda entity: float(entity.get("visibility_score", 0.0))
        )
        matrix = best.get("matrix")
        if not isinstance(matrix, list) or len(matrix) < 2:
            return False, "entity_confirmation_unavailable"
        if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
            if (
                best.get("entity_id") != candidate.get("entity_id")
                or len(matrix) < 4
                or pallet_product_type(matrix) != self.product_type
            ):
                return False, "conflicting_nearest_product_entity"
            return True, None
        if matrix[:2] != [self.target_left, self.target_right]:
            return False, "conflicting_entity_matrix"
        return True, None

    def identity_measurement(self):
        candidate, pnp = self.selected_candidate()
        if candidate is None:
            return None, None, "no_selected_candidate"
        entity_id = candidate.get("entity_id")
        if (
            self.target_entity_id is not None
            and entity_id != self.target_entity_id
        ):
            return None, None, "locked_entity_mismatch"
        require_entity = (
            self.boolean("require_best_entity_confirmation", True)
            if hasattr(self, "boolean") else False
        )
        if require_entity:
            matches, reason = self.candidate_matches_best_entity(candidate)
            if not matches:
                return None, None, reason
        frames = int(self.number("stable_detection_frames", 2, 1, 30))
        if int(candidate.get("streak", 0)) < frames:
            return None, None, "unstable_detection"
        return candidate, pnp, None

    def tracked_partial_measurement(self):
        """Use only the upper tag pair of the already locked pallet entity."""
        detection = self.latest_detection
        if detection is None or time.monotonic() - self.latest_detection_at > 0.8:
            return None, None, "partial_detection_stale"
        if detection.get("target_top") != [self.target_left, self.target_right]:
            return None, None, "partial_target_mismatch"
        partial = detection.get("tracked_partial")
        if not isinstance(partial, dict) or self.target_entity_id is None:
            return None, None, "partial_entity_unavailable"
        if partial.get("entity_id") != self.target_entity_id:
            return None, None, "partial_entity_mismatch"
        try:
            age_sec = float(partial.get("age_sec", math.inf))
            center_error = float(partial["center_error"])
            depth = partial["depth_yaw"]
            forward_cm = float(depth["forward_distance_cm"])
            yaw_deg = float(depth["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return None, None, "partial_pose_unavailable"
        max_age = self.number("partial_entity_max_age_sec", 5.0, 0.2, 20.0)
        if (
            not 0.0 <= age_sec <= max_age
            or not math.isfinite(center_error)
            or abs(center_error) > 1.0
            or not 10.0 <= forward_cm <= 300.0
            or not math.isfinite(yaw_deg)
            or abs(yaw_deg) > 45.0
        ):
            return None, None, "partial_pose_invalid"
        candidate = {
            "entity_id": self.target_entity_id,
            "center_error": center_error,
            "depth_yaw": depth,
        }
        pnp = {
            "depth_fallback": True,
            "distance_source": "depth",
            "lateral_ratio": clamp(0.5 * center_error, -0.5, 0.5),
        }
        return candidate, pnp, None

    def visible_target_top_pair_measurement(self):
        """Recover the requested upper pair without requiring a pallet entity."""
        detection = self.latest_detection
        if detection is None or time.monotonic() - self.latest_detection_at > 0.8:
            return None, None, "top_pair_detection_stale"
        if detection.get("target_top") != [self.target_left, self.target_right]:
            return None, None, "top_pair_target_mismatch"
        symbols = [
            item for item in (detection.get("detections") or [])
            if isinstance(item, dict) and item.get("class") != "pallet"
            and isinstance(item.get("box"), (list, tuple))
            and len(item["box"]) == 4
        ]
        left_tags = [item for item in symbols if item.get("class") == self.target_left]
        right_tags = [item for item in symbols if item.get("class") == self.target_right]
        image_width = self.number("camera_image_width_px", 640.0, 100.0, 4000.0)
        pairs = []
        for left_tag in left_tags:
            for right_tag in right_tags:
                if left_tag is right_tag:
                    continue
                left_box, right_box = left_tag["box"], right_tag["box"]
                left_center = (
                    0.5 * (float(left_box[0]) + float(left_box[2])),
                    0.5 * (float(left_box[1]) + float(left_box[3])),
                )
                right_center = (
                    0.5 * (float(right_box[0]) + float(right_box[2])),
                    0.5 * (float(right_box[1]) + float(right_box[3])),
                )
                if left_center[0] >= right_center[0]:
                    continue
                average_height = max(
                    0.5 * (
                        float(left_box[3]) - float(left_box[1])
                        + float(right_box[3]) - float(right_box[1])
                    ),
                    1.0,
                )
                row_error = abs(left_center[1] - right_center[1]) / average_height
                if row_error > 0.65:
                    continue
                pair_center_x = 0.5 * (left_center[0] + right_center[0])
                center_error = (pair_center_x - image_width * 0.5) / max(
                    image_width * 0.5, 1.0
                )
                depths = []
                for tag in (left_tag, right_tag):
                    depth = tag.get("depth") or {}
                    try:
                        distance_cm = float(depth["forward_distance_cm"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if math.isfinite(distance_cm) and 5.0 <= distance_cm <= 300.0:
                        depths.append(distance_cm)
                if not depths:
                    continue
                score = row_error + abs(center_error)
                pairs.append((score, center_error, left_tag, right_tag, depths))
        if not pairs:
            return None, None, "visible_target_top_pair_unavailable"
        _score, center_error, left_tag, right_tag, depths = min(
            pairs, key=lambda item: item[0]
        )
        yaw_deg = 0.0
        try:
            left_depth = float(left_tag["depth"]["camera_depth_m"])
            right_depth = float(right_tag["depth"]["camera_depth_m"])
            left_bearing = math.radians(float(left_tag["depth"]["bearing_deg"]))
            right_bearing = math.radians(float(right_tag["depth"]["bearing_deg"]))
            baseline = abs(
                right_depth * math.tan(right_bearing)
                - left_depth * math.tan(left_bearing)
            )
            if baseline > 0.02:
                yaw_deg = math.degrees(math.atan2(
                    right_depth - left_depth, baseline
                ))
        except (KeyError, TypeError, ValueError):
            pass
        if not math.isfinite(yaw_deg) or abs(yaw_deg) > 45.0:
            yaw_deg = 0.0
        depth = {
            "forward_distance_cm": float(np.median(depths)),
            "yaw_deg": yaw_deg,
        }
        candidate = {
            "entity_id": self.target_entity_id,
            "center_error": center_error,
            "depth_yaw": depth,
        }
        pnp = {
            "depth_fallback": True,
            "distance_source": "depth",
            "lateral_ratio": clamp(0.5 * center_error, -0.5, 0.5),
        }
        return candidate, pnp, None

    def valid_measurement(self):
        candidate, pnp, reason = self.identity_measurement()
        if candidate is None:
            return None, None, reason
        if pnp is None:
            return None, None, "invalid_pnp"
        depth_measurement = candidate.get("depth_yaw")
        try:
            reprojection = float(pnp.get("reprojection_error_px", 999.0))
            frontal = abs(float(candidate.get("frontal_error", 999.0)))
        except (TypeError, ValueError):
            return None, None, "invalid_pnp"
        max_reprojection = self.number(
            "max_pnp_reprojection_error_px", 3.0, 0.1, 100.0
        )
        max_frontal = self.number("max_frontal_error", 0.35, 0.01, 2.0)
        minimum_distance_cm = self.number(
            "minimum_dock_measurement_cm", 5.0, 3.0, 30.0
        )
        pnp_quality_ok = reprojection <= max_reprojection and frontal <= max_frontal

        if not isinstance(depth_measurement, dict):
            try:
                pnp_forward_cm = float(pnp["forward_distance_cm"])
            except (KeyError, TypeError, ValueError):
                return None, None, "distance_unavailable"
            if (
                not pnp_quality_ok
                or not minimum_distance_cm <= pnp_forward_cm <= 300.0
            ):
                return None, None, "invalid_pnp"
            pnp = dict(pnp)
            pnp["distance_source"] = "pnp"
            return candidate, pnp, None

        try:
            forward_cm = float(depth_measurement["forward_distance_cm"])
            depth_yaw_deg = float(depth_measurement["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return None, None, "depth_distance_unavailable"
        if not minimum_distance_cm <= forward_cm <= 300.0:
            return None, None, "invalid_depth_distance"
        pnp = dict(pnp)
        pnp["distance_source"] = "depth"
        if not pnp_quality_ok:
            try:
                center_error = float(candidate["center_error"])
            except (KeyError, TypeError, ValueError):
                return None, None, "invalid_pnp"
            if (
                not math.isfinite(depth_yaw_deg)
                or abs(depth_yaw_deg) > 45.0
                or not math.isfinite(center_error)
                or abs(center_error) > 1.0
            ):
                return None, None, "invalid_pnp"
            pnp["lateral_ratio"] = clamp(0.5 * center_error, -0.5, 0.5)
            pnp["depth_fallback"] = True
        return candidate, pnp, None

    def reset_coarse_alignment(self):
        self.coarse_alignment_started_at = None
        self.coarse_depth_fallback_frames = 0
        self.coarse_last_counted_stamp = None

    def reset_alignment_recovery(self):
        self.alignment_best_pose = None
        self.alignment_best_score = math.inf
        self.alignment_bad_frames = 0
        self.alignment_good_frames = 0
        self.alignment_last_good_stamp = None
        self.alignment_lost_since = None
        self.alignment_recovery_pose = None

    def yaw_source_disagreement_deg(self, candidate, pnp):
        if pnp.get("depth_fallback"):
            return 0.0
        depth_yaw = (candidate.get("depth_yaw") or {}).get("yaw_deg")
        pnp_yaw = pnp.get("yaw_deg")
        if depth_yaw is None or pnp_yaw is None:
            return 0.0
        return abs(math.degrees(normalize_angle(
            math.radians(-float(pnp_yaw) - float(depth_yaw))
        )))

    def yaw_sources_agree(self, candidate, pnp):
        maximum = self.number("alignment_yaw_source_max_delta_deg", 8.0, 1.0, 30.0)
        return AutoDockNode.yaw_source_disagreement_deg(self, candidate, pnp) <= maximum

    def enter_alignment(self, candidate, pnp):
        entity_id = candidate.get("entity_id")
        if entity_id is not None:
            if self.target_entity_id is None:
                self.target_entity_id = entity_id
            elif entity_id != self.target_entity_id:
                return False
        if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
            matrix = candidate.get("matrix") or []
            if len(matrix) >= 2:
                self.target_left, self.target_right = matrix[:2]
                self.send_yolo_target()
        if pnp.get("depth_fallback"):
            if not self.update_world_target(candidate, pnp):
                return False
            self.enter_coarse_alignment("pnp_quality_fallback")
            return True
        if not self.update_world_target(candidate, pnp):
            return False
        self.state = "docking"
        self.reset_coarse_alignment()
        self.publish_status(
            "running", "virtual_target_locked_docking",
            measurement_source=pnp.get("distance_source", "depth"),
        )
        return True

    def enter_coarse_alignment(self, reason):
        self.state = "coarse_align"
        self.coarse_alignment_started_at = time.monotonic()
        self.coarse_depth_fallback_frames = 0
        self.coarse_last_counted_stamp = None
        self.publish_status(
            "running", "edge_target_coarse_alignment",
            measurement_reason=reason,
        )

    def update_world_target(self, candidate, pnp, blend_existing=False):
        if self.odom_position is None or self.odom_yaw is None:
            return False
        depth_measurement = candidate.get("depth_yaw") or {}
        distance_source = pnp.get("distance_source", "depth")
        if distance_source == "pnp":
            forward_cm = float(pnp["forward_distance_cm"])
        else:
            forward_cm = float(depth_measurement["forward_distance_cm"])
        forward = forward_cm / 100.0
        lateral = -float(pnp.get("lateral_ratio", 0.0)) * forward
        lateral += self.number("centerline_offset_cm", 0.0) / 100.0
        # Camera measurements are physical metres, while odom over/under-counts
        # motion by the experimentally measured coefficients. Store the target
        # in odom units so reaching it produces the requested physical motion.
        forward /= self.number("distance_coefficient", 1.0, 0.10, 2.0)
        lateral /= self.number("lateral_coefficient", 1.0, 0.10, 2.0)
        depth_yaw = candidate.get("depth_yaw") or {}
        if pnp.get("depth_fallback"):
            yaw_deg = float(depth_yaw.get("yaw_deg", 0.0))
        else:
            yaw_deg = -float(pnp.get("yaw_deg", 0.0))
        if not pnp.get("depth_fallback") and depth_yaw.get("yaw_deg") is not None:
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
        lidar_safety_enabled = (
            AutoDockNode.boolean(self, "lidar_safety_enabled", True)
            if hasattr(self, "config") else True
        )
        if not lidar_safety_enabled:
            return False
        if self.state in {
            "idle", "ready", "waiting_fork",
            "slot_scanning", "slot_target_ready",
            "safety_backoff", "turning_right_for_ready",
            "post_lift_opening_reverse",
        }:
            return False
        default_clearance = self.number("lidar_stop_distance_m", 0.35, 0.05, 2.0)
        violations = []
        if self.state in {"search", "scan_forward_search"}:
            monitored = ("front", "rear", "left", "right")
        elif self.state in {
            "scan_approach", "coarse_align", "docking", "inserting"
        }:
            # The selected pallet is intentionally inside the front clearance.
            monitored = ("left", "right", "rear")
        elif self.state == "reversing_after_lift":
            monitored = ("rear",)
        elif self.state == "confirm":
            monitored = ()
        else:
            monitored = ("front", "rear", "left", "right")
        directions = (
            (direction, self.nearest_by_direction[direction])
            for direction in monitored
        )
        for direction, (distance, angle, calculated_clearance) in directions:
            clearance = calculated_clearance or default_clearance
            if direction == "rear" and self.state in {
                "coarse_align", "docking", "inserting"
            }:
                alignment_rear_minimum = self.number(
                    "alignment_rear_lidar_min_distance_cm", 30.0, 5.0, 100.0
                ) / 100.0
                clearance = max(clearance, alignment_rear_minimum)
            if distance < clearance:
                violations.append(
                    (distance - clearance, direction, distance, angle, clearance)
                )
        if not violations:
            return False
        _margin, direction, distance, angle, clearance = min(violations)
        self.nearest_range = distance
        self.nearest_angle = angle
        if self.state == "reversing_after_lift":
            self.stop_drive()
            self.publish_status(
                "waiting", "post_lift_reverse_blocked", direction=direction,
                range_m=round(distance, 3), clearance_m=round(clearance, 3),
            )
            return True
        if not AutoDockNode.boolean(self, "lidar_backoff_enabled", True):
            self.cancel(f"lidar_{direction}_blocked")
            return True
        self.was_docking_before_interrupt = self.state in {
            "coarse_align", "docking", "inserting"
        }
        self.state_before_lidar_interrupt = self.state
        self.candidate_stop_due_at = None
        self.candidate_confirmation_started_at = None
        backoff_speed = self.number("lidar_backoff_speed_m_s", 0.12, 0.05, 0.30)
        self.backoff_command = {
            "front": (-backoff_speed, 0.0),
            "rear": (backoff_speed, 0.0),
            "left": (0.0, -backoff_speed),
            "right": (0.0, backoff_speed),
        }[direction]
        self.backoff_direction = direction
        self.backoff_until = time.monotonic() + self.number(
            "lidar_backoff_duration_sec", 0.7, 0.1, 2.0
        )
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
        if self.state == "scan_sweep":
            self.tick_scan_sweep()
        elif self.state == "scan_forward_search":
            self.tick_scan_forward_search()
        elif self.state == "scan_approach":
            self.tick_scan_approach()
        elif self.state == "search":
            self.tick_search()
        elif self.state == "confirm":
            self.tick_confirm()
        elif self.state == "coarse_align":
            self.tick_coarse_align()
        elif self.state == "docking":
            self.tick_docking()
        elif self.state == "inserting":
            self.tick_inserting()
        elif self.state == "waiting_fork":
            self.stop_drive()
        elif self.state == "reversing_after_lift":
            self.tick_reversing_after_lift()
        elif self.state == "post_lift_opening_search":
            self.tick_post_lift_opening_search()
        elif self.state == "post_lift_opening_reverse":
            self.tick_post_lift_opening_reverse()
        elif self.state == "turning_right_for_ready":
            self.tick_turning_right_for_ready()
        elif self.state == "safety_backoff":
            self.tick_backoff()

    def tick_scan_sweep(self):
        """Sweep a small yaw arc after Nav2 arrival and lock a visible target."""
        now = time.monotonic()
        AutoDockNode.remember_leftmost_scan_tag(self)
        candidate, pnp, reason = self.valid_measurement()
        if candidate is not None:
            self.stop_drive(5)
            if not self.update_world_target(candidate, pnp):
                self.cancel("odom_missing")
                return
            self.target_entity_id = candidate.get("entity_id")
            self.state = "scan_approach"
            self.scan_candidate_seen_at = None
            self.publish_status(
                "running", "scan_target_locked_approaching",
                measurement_source=pnp.get("distance_source", "depth"),
            )
            return

        raw, _raw_pnp = self.selected_candidate()
        if raw is not None:
            if self.scan_candidate_seen_at is None:
                self.scan_candidate_seen_at = now
            self.stop_drive()
            confirmation_wait = self.number(
                "nav2_scan_confirmation_sec", 0.8, 0.1, 3.0
            )
            if now - self.scan_candidate_seen_at < confirmation_wait:
                self.publish_status(
                    "running", "scan_candidate_confirming",
                    measurement_reason=reason,
                )
                return
            self.scan_candidate_seen_at = None
        else:
            self.scan_candidate_seen_at = None

        if self.odom_yaw is None or self.scan_sweep_started_yaw is None:
            self.state = "search"
            self.publish_status("running", "scan_odom_missing_fallback_search")
            return
        angle = math.radians(self.number("nav2_scan_angle_deg", 20.0, 2.0, 30.0))
        targets = (angle, -angle, 0.0)
        phase = min(self.scan_sweep_phase, len(targets) - 1)
        target_offset = targets[phase]
        current_offset = normalize_angle(self.odom_yaw - self.scan_sweep_started_yaw)
        error = normalize_angle(target_offset - current_offset)
        tolerance = math.radians(
            self.number("nav2_scan_tolerance_deg", 1.5, 0.5, 5.0)
        )
        if abs(error) <= tolerance:
            self.stop_drive()
            self.scan_sweep_phase += 1
            if self.scan_sweep_phase >= len(targets):
                if (
                    self.scan_leftmost_tag_yaw is not None
                    and self.odom_position is not None
                ):
                    self.state = "scan_forward_search"
                    self.scan_forward_started_position = self.odom_position
                    self.scan_forward_phase = 0
                    self.publish_status(
                        "running", "scan_complete_leftmost_tag_forward_search",
                        reference_yaw_deg=round(
                            math.degrees(self.scan_leftmost_tag_yaw), 1
                        ),
                    )
                else:
                    self.state = "search"
                    self.latch_search_heading()
                    self.publish_status(
                        "running", "scan_complete_no_tag_fallback_lateral_search"
                    )
                return
            self.publish_status(
                "running", "scan_sweep_endpoint",
                phase=self.scan_sweep_phase,
                yaw_offset_deg=round(math.degrees(current_offset), 1),
            )
            return
        speed = self.number("nav2_scan_angular_speed_rad_s", 0.18, 0.05, 0.40)
        angular = math.copysign(min(speed, max(0.08, abs(error))), error)
        self.publish_drive(0.0, 0.0, angular)
        self.publish_status(
            "running", "nav2_scan_sweeping",
            phase=phase,
            target_yaw_offset_deg=round(math.degrees(target_offset), 1),
            yaw_error_deg=round(math.degrees(error), 1),
        )

    def remember_leftmost_scan_tag(self):
        """Remember the world bearing of the leftmost visible non-pallet tag."""
        detection = getattr(self, "latest_detection", None)
        if (
            detection is None
            or time.monotonic() - getattr(self, "latest_detection_at", 0.0) > 0.8
            or getattr(self, "odom_yaw", None) is None
        ):
            return
        tags = [
            item for item in (detection.get("detections") or [])
            if isinstance(item, dict)
            and item.get("class") in SYMBOLS
            and isinstance(item.get("box"), list)
            and len(item["box"]) == 4
        ]
        if not tags:
            return
        leftmost = min(tags, key=lambda item: 0.5 * (item["box"][0] + item["box"][2]))
        center_x = 0.5 * (float(leftmost["box"][0]) + float(leftmost["box"][2]))
        image_width = self.number("detection_image_width_px", 640.0, 100.0, 4000.0)
        horizontal_fov = math.radians(
            self.number("camera_horizontal_fov_deg", 60.0, 20.0, 120.0)
        )
        # Image-left is positive body yaw.
        image_error = clamp((0.5 * image_width - center_x) / (0.5 * image_width), -1.0, 1.0)
        observed_yaw = normalize_angle(
            self.odom_yaw + image_error * 0.5 * horizontal_fov
        )
        if self.scan_sweep_started_yaw is None:
            self.scan_leftmost_tag_yaw = observed_yaw
            return
        observed_offset = normalize_angle(observed_yaw - self.scan_sweep_started_yaw)
        remembered_offset = (
            -math.inf if self.scan_leftmost_tag_yaw is None
            else normalize_angle(
                self.scan_leftmost_tag_yaw - self.scan_sweep_started_yaw
            )
        )
        if observed_offset > remembered_offset:
            self.scan_leftmost_tag_yaw = observed_yaw

    def tick_scan_forward_search(self):
        """Move forward while sweeping yaw around the leftmost-tag bearing."""
        candidate, pnp, _reason = self.valid_measurement()
        if candidate is not None:
            self.stop_drive(5)
            if not self.update_world_target(candidate, pnp):
                self.cancel("odom_missing")
                return
            self.target_entity_id = candidate.get("entity_id")
            self.state = "scan_approach"
            self.publish_status("running", "forward_search_target_locked")
            return
        if (
            self.odom_yaw is None
            or self.odom_position is None
            or self.scan_leftmost_tag_yaw is None
            or self.scan_forward_started_position is None
        ):
            self.stop_drive()
            self.state = "search"
            self.publish_status("running", "forward_search_pose_missing_fallback")
            return
        travelled = math.hypot(
            self.odom_position[0] - self.scan_forward_started_position[0],
            self.odom_position[1] - self.scan_forward_started_position[1],
        ) * self.number("distance_coefficient", 1.0, 0.10, 2.0)
        maximum_distance = self.number(
            "nav2_forward_search_max_distance_m", 1.0, 0.20, 3.0
        )
        if travelled >= maximum_distance:
            self.stop_drive(5)
            self.state = "search"
            self.latch_search_heading()
            self.publish_status(
                "running", "forward_search_distance_limit_fallback",
                travelled_m=round(travelled, 2),
            )
            return
        angle = math.radians(self.number("nav2_scan_angle_deg", 20.0, 2.0, 30.0))
        offsets = (0.0, angle, -angle)
        target_yaw = normalize_angle(
            self.scan_leftmost_tag_yaw + offsets[self.scan_forward_phase]
        )
        error = normalize_angle(target_yaw - self.odom_yaw)
        tolerance = math.radians(
            self.number("nav2_scan_tolerance_deg", 1.5, 0.5, 5.0)
        )
        if abs(error) <= tolerance:
            self.scan_forward_phase = (self.scan_forward_phase + 1) % len(offsets)
        speed = self.number("nav2_forward_search_speed_m_s", 0.08, 0.05, 0.15)
        max_yaw = self.number(
            "nav2_scan_angular_speed_rad_s", 0.18, 0.05, 0.40
        )
        self.publish_drive(
            speed,
            0.0,
            clamp(1.2 * error, -max_yaw, max_yaw),
        )
        self.publish_status(
            "running", "leftmost_tag_forward_sweep_search",
            travelled_m=round(travelled, 2),
            yaw_error_deg=round(math.degrees(error), 1),
        )

    def tick_scan_approach(self):
        """Approach the locked target, then hand control to existing alignment."""
        candidate, pnp, _reason = self.valid_measurement()
        if candidate is not None:
            entity_id = candidate.get("entity_id")
            if self.target_entity_id is None or entity_id in {None, self.target_entity_id}:
                self.update_world_target(candidate, pnp, blend_existing=True)
        target = self.target_in_body()
        if target is None:
            self.cancel("scan_approach_target_missing")
            return
        forward, lateral, _target_yaw = target
        forward_actual = forward * self.number(
            "distance_coefficient", 1.0, 0.10, 2.0
        )
        standoff = self.number("nav2_approach_standoff_m", 0.22, 0.12, 1.0)
        if forward_actual <= standoff:
            self.stop_drive(5)
            if candidate is not None:
                if not self.enter_alignment(candidate, pnp):
                    self.cancel("scan_approach_entity_changed")
            else:
                self.state = "docking"
                self.publish_status("running", "scan_approach_complete_aligning")
            return
        if forward <= 0.0:
            self.cancel("scan_approach_target_behind")
            return
        bearing = math.atan2(lateral, forward)
        speed = self.number("nav2_approach_speed_m_s", 0.08, 0.05, 0.15)
        max_yaw = self.number(
            "nav2_approach_max_angular_speed_rad_s", 0.16, 0.05, 0.25
        )
        self.publish_drive(
            speed,
            0.0,
            clamp(1.2 * bearing, -max_yaw, max_yaw),
        )
        self.publish_status(
            "running", "scan_target_approaching",
            distance_cm=round(forward_actual * 100.0, 1),
            bearing_deg=round(math.degrees(bearing), 1),
            standoff_cm=round(standoff * 100.0, 1),
        )

    def tick_search(self):
        candidate, _pnp = self.selected_candidate()
        now = time.monotonic()
        if now < getattr(self, "candidate_retry_not_before", 0.0):
            candidate = None
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
                identity, _identity_pnp, identity_reason = self.identity_measurement()
                if identity is not None:
                    self.enter_coarse_alignment(reason or identity_reason)
                    return
                self.state = "confirm"
                self.candidate_confirmation_started_at = now
                self.publish_status(
                    "running", "candidate_paused_for_confirmation",
                    measurement_reason=reason,
                )
                return
            if not self.enter_alignment(candidate, pnp):
                self.cancel("odom_missing")
                return
            self.candidate_confirmation_started_at = None
            self.candidate_retry_not_before = 0.0
            return
        fallback_speed = self.number("search_linear_speed_m_s", 0.03, 0.01, 0.30)
        lateral_speed = self.number(
            "search_lateral_speed_m_s", fallback_speed, 0.01, 0.30
        )
        direction = str(
            self.config.get("search_lateral_direction", "left")
        ).strip().lower()
        direction_sign = -1.0 if direction == "right" else 1.0
        if AutoDockNode.boolean(self, "tape_guidance_only", False):
            if AutoDockNode.command_warning_tape_search(
                self,
                now, lateral_speed, direction_sign
            ):
                return
            AutoDockNode.tick_missing_tape_recovery(
                self, now, preferred_direction="rear"
            )
            return
        front_guidance_enabled = AutoDockNode.boolean(
            self, "tag_guided_lateral_search_enabled", False
        )
        rear_guidance_enabled = AutoDockNode.boolean(
            self, "search_rear_lidar_guidance_enabled", False
        )
        front_observation = (
            AutoDockNode.front_search_entity_observation(self)
            if front_guidance_enabled else None
        )
        minimum_front_distance_cm = self.number(
            "tag_search_min_distance_cm", 15.0, 5.0, 100.0
        )
        maximum_front_distance_cm = self.number(
            "tag_search_max_distance_cm", 20.0,
            minimum_front_distance_cm, 100.0,
        )
        if rear_guidance_enabled:
            maximum_scan_age = self.number(
                "search_rear_lidar_max_age_sec", 0.50, 0.10, 2.0
            )
            scan_age = now - getattr(self, "scan_updated_at", 0.0)
            if scan_age > maximum_scan_age:
                self.stop_drive()
                self.publish_status(
                    "waiting", "rear_lidar_search_scan_stale",
                    scan_age_sec=round(scan_age, 2),
                    maximum_scan_age_sec=round(maximum_scan_age, 2),
                )
                return
            rear_distance, rear_angle, _rear_threshold = getattr(
                self, "nearest_by_direction", {}
            ).get("rear", (math.inf, None, 0.0))
            if not math.isfinite(rear_distance):
                self.stop_drive()
                self.publish_status(
                    "waiting", "rear_lidar_search_distance_missing"
                )
                return
            minimum_distance_cm = self.number(
                "search_rear_lidar_min_distance_cm", 30.0, 5.0, 100.0
            )
            rear_distance_cm = rear_distance * 100.0
            if rear_distance_cm < minimum_distance_cm:
                if (
                    front_observation is not None
                    and front_observation[1] <= maximum_front_distance_cm
                ):
                    self.stop_drive()
                    self.publish_status(
                        "waiting", "search_longitudinal_clearance_conflict",
                        front_distance_cm=round(front_observation[1], 1),
                        front_minimum_cm=round(minimum_front_distance_cm, 1),
                        front_maximum_cm=round(maximum_front_distance_cm, 1),
                        rear_distance_cm=round(rear_distance_cm, 1),
                        rear_minimum_cm=round(minimum_distance_cm, 1),
                    )
                    return
                forward_speed = self.number(
                    "search_rear_lidar_forward_speed_m_s", 0.12, 0.05, 0.20
                )
                self.publish_drive(forward_speed, 0.0, 0.0)
                self.publish_status(
                    "running", "rear_lidar_distance_correction",
                    rear_distance_cm=round(rear_distance_cm, 1),
                    minimum_distance_cm=round(minimum_distance_cm, 1),
                    rear_angle_deg=(
                        None if rear_angle is None
                        else round(math.degrees(rear_angle), 1)
                    ),
                )
                return
            front_observation_available = (
                front_guidance_enabled
                and front_observation is not None
            )
            if front_observation_available:
                # Rear clearance is satisfied.  Continue with option 1 so
                # both checkboxes can be tested as a combined strategy.
                pass
            elif AutoDockNode.command_warning_tape_search(
                self,
                now, lateral_speed, direction_sign
            ):
                return
            elif AutoDockNode.search_heading_error(self) is None:
                self.stop_drive()
                self.publish_status("waiting", "rear_lidar_search_odom_missing")
                return
            else:
                pair_yaw_error = AutoDockNode.front_search_pair_yaw_error(self)
                yaw_error = (
                    pair_yaw_error if pair_yaw_error is not None
                    else AutoDockNode.search_heading_error(self)
                )
                yaw_deg = math.degrees(yaw_error)
                yaw_tolerance = self.number(
                    "tag_search_yaw_tolerance_deg", 3.0, 0.5, 15.0
                )
                if abs(yaw_deg) > yaw_tolerance:
                    angular = AutoDockNode.search_angular_command(self, yaw_error)
                    self.publish_drive(0.0, 0.0, angular)
                    self.publish_status(
                        "running", "rear_lidar_yaw_correction_before_lateral",
                        rear_distance_cm=round(rear_distance_cm, 1),
                        yaw_deg=round(yaw_deg, 1),
                        yaw_source=(
                            "front_tag_pair" if pair_yaw_error is not None else
                            getattr(self, "search_heading_source", None) or "odom"
                        ),
                    )
                    return
                self.publish_drive(0.0, direction_sign * lateral_speed, 0.0)
                self.publish_status(
                    "running", "rear_lidar_clearance_held_lateral_search",
                    rear_distance_cm=round(rear_distance_cm, 1),
                    minimum_distance_cm=round(minimum_distance_cm, 1),
                    yaw_deg=round(yaw_deg, 1),
                    lateral_speed_m_s=round(direction_sign * lateral_speed, 3),
                )
                return
        if front_guidance_enabled:
            observation = front_observation
            if observation is None:
                if AutoDockNode.command_warning_tape_search(
                    self,
                    now, lateral_speed, direction_sign
                ):
                    return
                self.stop_drive()
                self.publish_status("waiting", "tag_guided_search_depth_missing")
                return
            tag_class, distance_cm, bearing_deg = observation
            if distance_cm < minimum_front_distance_cm:
                rear_distance_cm = math.inf
                rear_minimum_cm = self.number(
                    "search_rear_lidar_min_distance_cm", 30.0, 5.0, 100.0
                )
                if rear_guidance_enabled:
                    rear_distance = getattr(
                        self, "nearest_by_direction", {}
                    ).get("rear", (math.inf, None, 0.0))[0]
                    rear_distance_cm = rear_distance * 100.0
                reverse_margin_cm = self.number(
                    "tag_search_reverse_rear_margin_cm", 3.0, 0.0, 20.0
                )
                if (
                    rear_guidance_enabled
                    and rear_distance_cm <= rear_minimum_cm + reverse_margin_cm
                ):
                    self.stop_drive()
                    self.publish_status(
                        "waiting", "search_longitudinal_clearance_conflict",
                        tag_class=tag_class,
                        front_distance_cm=round(distance_cm, 1),
                        front_minimum_cm=round(minimum_front_distance_cm, 1),
                        rear_distance_cm=round(rear_distance_cm, 1),
                        rear_minimum_cm=round(rear_minimum_cm, 1),
                    )
                    return
                reverse_speed = self.number(
                    "tag_search_reverse_correction_speed_m_s", 0.08, 0.03, 0.15
                )
                self.publish_drive(-reverse_speed, 0.0, 0.0)
                self.publish_status(
                    "running", "front_tag_too_close_reversing",
                    tag_class=tag_class,
                    distance_cm=round(distance_cm, 1),
                    minimum_distance_cm=round(minimum_front_distance_cm, 1),
                )
                return
            pair_yaw_error = AutoDockNode.front_search_pair_yaw_error(self)
            yaw_error = (
                pair_yaw_error if pair_yaw_error is not None
                else AutoDockNode.search_heading_error(self)
            )
            if yaw_error is None:
                self.stop_drive()
                self.publish_status("waiting", "tag_guided_search_odom_missing")
                return
            yaw_deg = math.degrees(yaw_error)
            yaw_tolerance = self.number(
                "tag_search_yaw_tolerance_deg", 3.0, 0.5, 15.0
            )
            if abs(yaw_deg) > yaw_tolerance:
                angular = AutoDockNode.search_angular_command(self, yaw_error)
                self.publish_drive(0.0, 0.0, angular)
                self.publish_status(
                    "running", "tag_yaw_correction_before_lateral",
                    tag_class=tag_class,
                    yaw_deg=round(yaw_deg, 1),
                    yaw_source=(
                        "front_tag_pair" if pair_yaw_error is not None else
                        getattr(self, "search_heading_source", None) or "odom"
                    ),
                )
                return
            if distance_cm > maximum_front_distance_cm:
                forward_speed = self.number(
                    "tag_search_forward_correction_speed_m_s", 0.12, 0.08, 0.20
                )
                self.publish_drive(forward_speed, 0.0, 0.0)
                self.publish_status(
                    "running", "front_tag_distance_correction",
                    tag_class=tag_class,
                    distance_cm=round(distance_cm, 1),
                    maximum_distance_cm=round(maximum_front_distance_cm, 1),
                )
                return
            self.publish_drive(0.0, direction_sign * lateral_speed, 0.0)
            self.publish_status(
                "running", "front_tag_pose_held_lateral_search",
                tag_class=tag_class,
                distance_cm=round(distance_cm, 1),
                bearing_deg=round(bearing_deg, 1),
                yaw_deg=round(yaw_deg, 1),
                lateral_speed_m_s=round(direction_sign * lateral_speed, 3),
            )
            return
        if AutoDockNode.command_warning_tape_search(
            self, now, lateral_speed, direction_sign
        ):
            return
        # The real mecanum base drifts rearward during a pure strafe.  Keep the
        # proven manual lateral speed and mix in a small forward feed-forward
        # correction without letting either wheel pair enter the unstable
        # low-speed region.
        minimum_wheel_component = self.number(
            "search_min_wheel_component_m_s", 0.10, 0.01, 0.30
        )
        correction_limit = max(0.0, lateral_speed - minimum_wheel_component)
        forward_compensation = clamp(
            self.number("search_forward_compensation_m_s", 0.02, -0.10, 0.10),
            -correction_limit,
            correction_limit,
        )
        self.publish_status(
            "running", "lateral_target_search",
            forward_compensation_m_s=round(forward_compensation, 3),
            lateral_speed_m_s=round(direction_sign * lateral_speed, 3),
        )
        self.publish_drive(
            forward_compensation, direction_sign * lateral_speed, 0.0
        )

    def command_warning_tape_search(
        self, now, lateral_speed, direction_sign, pose_correction_only=False
    ):
        """Hold tape angle/distance, then optionally issue the lateral command."""
        if not (
            AutoDockNode.boolean(self, "tape_guidance_enabled", False)
            or AutoDockNode.boolean(self, "tape_guidance_only", False)
        ):
            return False
        tape = getattr(self, "latest_tape_guidance", None)
        tape_age = now - getattr(self, "latest_tape_guidance_at", 0.0)
        if (
            tape is None
            or tape_age > self.number("tape_max_age_sec", 0.50, 0.10, 3.0)
        ):
            return False
        if self.tape_reference is None:
            self.tape_reference = {
                "center_y_ratio": self.number(
                    "tape_target_center_y_ratio", 0.65, 0.10, 0.90
                ),
                "angle_deg": self.number(
                    "tape_target_angle_deg", 0.0, -35.0, 35.0
                ),
            }
            self.tape_filtered_center_y_ratio = float(tape["center_y_ratio"])
            self.tape_filtered_angle_deg = float(tape["angle_deg"])
        filter_alpha = self.number(
            "tape_pose_filter_alpha", 0.20, 0.05, 1.0
        )
        previous_center = getattr(
            self, "tape_filtered_center_y_ratio", float(tape["center_y_ratio"])
        )
        previous_angle = getattr(
            self, "tape_filtered_angle_deg", float(tape["angle_deg"])
        )
        self.tape_filtered_center_y_ratio = (
            (1.0 - filter_alpha) * previous_center
            + filter_alpha * float(tape["center_y_ratio"])
        )
        angle_step = math.degrees(normalize_angle(math.radians(
            float(tape["angle_deg"]) - previous_angle
        )))
        self.tape_filtered_angle_deg = previous_angle + filter_alpha * angle_step
        vertical_error = (
            self.tape_filtered_center_y_ratio
            - float(self.tape_reference["center_y_ratio"])
        )
        angle_error_deg = normalize_angle(math.radians(
            self.tape_filtered_angle_deg
            - float(self.tape_reference["angle_deg"])
        ))
        angle_error_deg = math.degrees(angle_error_deg)
        yaw_tolerance = self.number(
            "tape_yaw_tolerance_deg", 2.5, 0.5, 15.0
        )
        if abs(angle_error_deg) > yaw_tolerance:
            minimum_yaw = self.number(
                "tape_min_yaw_speed_rad_s", 0.12, 0.02, 0.50
            )
            maximum_yaw = max(minimum_yaw, self.number(
                "tape_max_yaw_speed_rad_s", 0.20, 0.02, 0.50
            ))
            magnitude = clamp(
                abs(math.radians(angle_error_deg))
                * self.number("tape_yaw_gain", 0.80, 0.0, 3.0),
                minimum_yaw,
                maximum_yaw,
            )
            angular = math.copysign(magnitude, -angle_error_deg)
            self.publish_drive(0.0, 0.0, angular)
            self.publish_status(
                "running", (
                    "alignment_warning_tape_yaw_correction"
                    if pose_correction_only else
                    "warning_tape_yaw_correction_before_lateral"
                ),
                tape_angle_deg=round(float(tape["angle_deg"]), 2),
                reference_angle_deg=round(
                    float(self.tape_reference["angle_deg"]), 2
                ),
                angle_error_deg=round(angle_error_deg, 2),
            )
            return True
        vertical_tolerance = self.number(
            "tape_vertical_tolerance_ratio", 0.035, 0.002, 0.10
        )
        if abs(vertical_error) > vertical_tolerance:
            minimum_speed = self.number(
                "tape_min_forward_speed_m_s", 0.01, 0.005, 0.05
            )
            maximum_speed = max(minimum_speed, self.number(
                "tape_max_forward_speed_m_s", 0.03, 0.005, 0.10
            ))
            speed = clamp(
                abs(vertical_error)
                * self.number("tape_forward_gain", 0.10, 0.0, 1.0),
                minimum_speed,
                maximum_speed,
            )
            forward = math.copysign(speed, -vertical_error)
            self.publish_drive(forward, 0.0, 0.0)
            self.publish_status(
                "running", (
                    "alignment_warning_tape_distance_correction"
                    if pose_correction_only else
                    "warning_tape_distance_correction_before_lateral"
                ),
                tape_center_y_ratio=round(float(tape["center_y_ratio"]), 4),
                reference_center_y_ratio=round(
                    float(self.tape_reference["center_y_ratio"]), 4
                ),
                vertical_error=round(vertical_error, 4),
                forward_speed_m_s=round(forward, 3),
            )
            return True
        if pose_correction_only:
            return False
        self.publish_drive(0.0, direction_sign * lateral_speed, 0.0)
        self.publish_status(
            "running", "warning_tape_pose_held_lateral_search",
            tape_angle_deg=round(float(tape["angle_deg"]), 2),
            tape_center_y_ratio=round(float(tape["center_y_ratio"]), 4),
            lateral_speed_m_s=round(direction_sign * lateral_speed, 3),
        )
        return True

    def front_search_entity_observation(self):
        """Return the image-centered individual symbol with registered depth."""
        detection = getattr(self, "latest_detection", None)
        if (
            detection is None
            or time.monotonic() - getattr(self, "latest_detection_at", 0.0) > 0.8
        ):
            return None
        observations = []
        maximum_noise_distance = self.number(
            "tag_search_noise_max_distance_cm", 30.0, 10.0, 100.0
        )
        for tag in detection.get("detections") or []:
            if not isinstance(tag, dict) or tag.get("class") not in SYMBOLS:
                continue
            box = tag.get("box")
            if not isinstance(box, list) or len(box) != 4:
                continue
            depth = tag.get("depth") or {}
            distance = depth.get("forward_distance_cm")
            bearing = depth.get("bearing_deg")
            try:
                distance = float(distance)
                bearing = float(bearing)
                center_x = 0.5 * (float(box[0]) + float(box[2]))
            except (TypeError, ValueError):
                continue
            if (
                not math.isfinite(distance)
                or not math.isfinite(bearing)
                or not 5.0 <= distance <= maximum_noise_distance
                or abs(bearing) > 90.0
            ):
                continue
            observations.append((tag.get("class"), distance, bearing, center_x))
        if not observations:
            return None
        image_center_x = self.number(
            "detection_image_width_px", 640.0, 100.0, 4000.0
        ) * 0.5
        selected = min(observations, key=lambda item: abs(item[3] - image_center_x))
        return selected[0], selected[1], selected[2]

    def tick_missing_tape_recovery(self, now, preferred_direction=None):
        """Nudge once toward the clearer longitudinal side while searching."""
        if getattr(self, "tape_recovery_done", False):
            self.stop_drive()
            self.publish_status("waiting", "warning_tape_not_detected_after_nudge")
            return
        maximum_scan_age = self.number(
            "tape_recovery_scan_max_age_sec", 0.50, 0.10, 2.0
        )
        if now - getattr(self, "scan_updated_at", 0.0) > maximum_scan_age:
            self.stop_drive()
            self.publish_status("waiting", "warning_tape_recovery_lidar_stale")
            return
        if self.odom_position is None:
            self.stop_drive()
            self.publish_status("waiting", "warning_tape_recovery_odom_missing")
            return

        target_distance = self.number(
            "tape_recovery_distance_m", 0.08, 0.02, 0.30
        )
        extra_clearance = self.number(
            "tape_recovery_extra_clearance_m", 0.05, 0.0, 0.50
        )
        direction = getattr(self, "tape_recovery_direction", None)
        if direction is None:
            margins = {}
            for candidate_direction in ("front", "rear"):
                distance, _angle, clearance = self.nearest_by_direction[
                    candidate_direction
                ]
                margins[candidate_direction] = distance - clearance
            if preferred_direction in {"front", "rear"}:
                direction = preferred_direction
            elif math.isclose(
                margins["front"], margins["rear"], rel_tol=0.0, abs_tol=0.01
            ):
                tie_direction = str(
                    self.config.get("tape_recovery_tie_direction", "rear")
                ).strip().lower()
                direction = tie_direction if tie_direction in {"front", "rear"} else "rear"
            else:
                direction = max(margins, key=margins.get)
            if margins[direction] < target_distance + extra_clearance:
                self.stop_drive()
                self.tape_recovery_done = True
                self.publish_status(
                    "waiting", "warning_tape_recovery_blocked",
                    direction=direction,
                    available_m=round(margins[direction], 3),
                )
                return
            self.tape_recovery_direction = direction
            self.tape_recovery_start_position = self.odom_position

        start = self.tape_recovery_start_position
        travelled = math.hypot(
            self.odom_position[0] - start[0],
            self.odom_position[1] - start[1],
        )
        if travelled >= target_distance:
            self.stop_drive()
            self.tape_recovery_done = True
            self.publish_status(
                "waiting", "warning_tape_not_detected_after_nudge",
                direction=direction, travelled_m=round(travelled, 3),
            )
            return

        distance, _angle, clearance = self.nearest_by_direction[direction]
        remaining_margin = distance - clearance
        if remaining_margin < (target_distance - travelled) + extra_clearance:
            self.stop_drive()
            self.tape_recovery_done = True
            self.publish_status(
                "waiting", "warning_tape_recovery_blocked",
                direction=direction, available_m=round(remaining_margin, 3),
            )
            return
        speed = self.number("tape_recovery_speed_m_s", 0.08, 0.05, 0.20)
        linear_x = speed if direction == "front" else -speed
        self.publish_status(
            "running", "warning_tape_clearance_nudge",
            direction=direction,
            target_distance_m=round(target_distance, 3),
        )
        self.publish_drive(linear_x, 0.0, 0.0)

    def tick_confirm(self):
        candidate, pnp, reason = self.valid_measurement()
        if candidate is None:
            identity, _identity_pnp, identity_reason = self.identity_measurement()
            if identity is not None:
                self.enter_coarse_alignment(reason or identity_reason)
                return
            raw, _ = self.selected_candidate()
            if raw is None:
                self.state = "search"
                self.candidate_confirmation_started_at = None
                self.candidate_retry_not_before = 0.0
                self.publish_status("running", "candidate_lost_resume_search")
                return
            now = time.monotonic()
            if self.candidate_confirmation_started_at is None:
                self.candidate_confirmation_started_at = now
            timeout = self.number(
                "candidate_confirmation_timeout_sec", 0.8, 0.1, 10.0
            )
            if now - self.candidate_confirmation_started_at >= timeout:
                cooldown = self.number(
                    "candidate_retry_cooldown_sec", 1.0, 0.0, 10.0
                )
                self.state = "search"
                self.candidate_confirmation_started_at = None
                self.candidate_retry_not_before = now + cooldown
                self.publish_status(
                    "running", "candidate_invalid_resume_search",
                    measurement_reason=reason, retry_cooldown_sec=cooldown,
                )
                return
            self.stop_drive()
            self.publish_status(
                "running", "candidate_confirmation", measurement_reason=reason
            )
            return
        if not self.enter_alignment(candidate, pnp):
            self.cancel("odom_missing")
            return
        self.candidate_confirmation_started_at = None
        self.candidate_retry_not_before = 0.0

    def tick_coarse_align(self):
        now = time.monotonic()
        candidate, _pnp, identity_reason = self.identity_measurement()
        partial_pnp = None
        if candidate is None:
            candidate, partial_pnp, partial_reason = self.tracked_partial_measurement()
            if candidate is None:
                top_pair_measurement = getattr(
                    self, "visible_target_top_pair_measurement", None
                )
                if callable(top_pair_measurement):
                    candidate, partial_pnp, top_pair_reason = top_pair_measurement()
                    partial_reason = top_pair_reason or partial_reason
            if candidate is None:
                if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
                    lost_entity_id = self.target_entity_id
                    self.stop_drive()
                    self.target_world = None
                    self.target_entity_id = None
                    self.candidate_stop_due_at = None
                    self.candidate_confirmation_started_at = None
                    self.candidate_retry_not_before = 0.0
                    self.state = "search"
                    self.reset_coarse_alignment()
                    self.latch_search_heading()
                    self.publish_status(
                        "running", "nearest_target_lost_resume_search",
                        measurement_reason=partial_reason or identity_reason,
                        lost_entity_id=lost_entity_id,
                    )
                    return
                if self.target_world is not None:
                    self.state = "docking"
                    self.reset_coarse_alignment()
                    self.publish_status(
                        "running", "coarse_target_lost_using_virtual_target",
                        measurement_reason=partial_reason or identity_reason,
                        entity_id=self.target_entity_id,
                    )
                    return
                self.stop_drive()
                self.publish_status(
                    "waiting", "coarse_locked_target_temporarily_lost",
                    measurement_reason=partial_reason or identity_reason,
                    entity_id=self.target_entity_id,
                )
                return
        if self.coarse_alignment_started_at is None:
            self.coarse_alignment_started_at = now
        timeout = self.number("coarse_alignment_timeout_sec", 5.0, 0.5, 30.0)
        if now - self.coarse_alignment_started_at >= timeout:
            # Once alignment has started, keep the locked candidate instead of
            # dropping back to search merely because coarse alignment is slow.
            self.coarse_alignment_started_at = now
            self.publish_status("running", "coarse_alignment_continuing_locked_target")

        measured_candidate, measured_pnp, measurement_reason = self.valid_measurement()
        if measured_candidate is not None and not measured_pnp.get("depth_fallback"):
            if not self.enter_alignment(measured_candidate, measured_pnp):
                self.cancel("odom_missing")
            return
        if measured_candidate is None and partial_pnp is not None:
            measured_candidate = candidate
            measured_pnp = partial_pnp
            measurement_reason = "locked_entity_upper_pair"

        try:
            center_error = float(candidate.get("center_error", 999.0))
        except (TypeError, ValueError):
            center_error = 999.0
        if not math.isfinite(center_error) or abs(center_error) > 1.0:
            self.stop_drive()
            self.publish_status("running", "coarse_center_error_invalid")
            return
        center_tolerance = self.number("coarse_center_error_max", 0.10, 0.01, 0.50)
        if abs(center_error) > center_tolerance:
            self.coarse_depth_fallback_frames = 0
            self.coarse_last_counted_stamp = None
            lateral_gain = self.number("coarse_align_lateral_gain", 0.10, 0.01, 0.50)
            max_lateral = self.number(
                "coarse_align_max_lateral_speed_m_s", 0.05, 0.01, 0.15
            )
            depth = candidate.get("depth_yaw") or {}
            try:
                yaw_error = math.radians(float(depth["yaw_deg"]))
            except (KeyError, TypeError, ValueError):
                yaw_error = 0.0
                if self.search_heading_yaw is not None and self.odom_yaw is not None:
                    yaw_error = normalize_angle(self.search_heading_yaw - self.odom_yaw)
            self.publish_drive(
                0.0,
                clamp(-lateral_gain * center_error, -max_lateral, max_lateral),
                clamp(1.2 * yaw_error, -0.20, 0.20),
            )
            self.publish_status(
                "running", "coarse_centering",
                center_error=round(center_error, 4),
                measurement_reason=measurement_reason,
            )
            return

        self.stop_drive()
        if measured_candidate is not None and measured_pnp.get("depth_fallback"):
            source_stamp = (self.latest_detection or {}).get("source_stamp_ns")
            if source_stamp != self.coarse_last_counted_stamp:
                self.coarse_last_counted_stamp = source_stamp
                self.coarse_depth_fallback_frames += 1
            required = int(self.number(
                "depth_fallback_confirmation_frames", 3, 1, 30
            ))
            if self.coarse_depth_fallback_frames >= required:
                if not self.update_world_target(measured_candidate, measured_pnp):
                    self.cancel("odom_missing")
                    return
                self.state = "docking"
                self.reset_coarse_alignment()
                self.publish_status(
                    "running", "centered_depth_fallback_locked",
                    confirmation_frames=required,
                )
                return
            self.publish_status(
                "running", "centered_depth_confirmation",
                confirmed_frames=self.coarse_depth_fallback_frames,
                required_frames=required,
            )
            return
        self.publish_status(
            "running", "centered_waiting_valid_pose",
            measurement_reason=measurement_reason,
        )

    def tick_docking(self):
        now = time.monotonic()
        insertion_start_due_at = getattr(self, "insertion_start_due_at", None)
        if insertion_start_due_at is not None:
            self.stop_drive()
            if now < insertion_start_due_at:
                return
            self.insertion_start_due_at = None
            self.insert_start_position = self.odom_position
            self.insert_start_yaw = getattr(self, "odom_yaw", 0.0) or 0.0
            self.state = "inserting"
            self.publish_status("running", "aligned_inserting")
            return
        candidate, pnp, _ = self.valid_measurement()
        if candidate is not None:
            self.update_world_target(candidate, pnp, blend_existing=True)
        target = self.target_in_body()
        if target is None:
            self.cancel("odom_missing")
            return
        forward, lateral, yaw = target
        standoff = self.number("dock_standoff_m", 0.20, 0.03, 1.0)
        forward_actual = forward * self.number(
            "distance_coefficient", 1.0, 0.10, 2.0
        )
        lateral_actual = lateral * self.number(
            "lateral_coefficient", 1.0, 0.10, 2.0
        )
        translation_first = (
            AutoDockNode.boolean(self, "translation_first_alignment_enabled", False)
            if hasattr(self, "config") else False
        )
        if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
            translation_first = True
        maximum_trusted_yaw = math.radians(
            self.number("alignment_max_trusted_yaw_deg", 12.0, 3.0, 30.0)
        )
        yaw_trusted = abs(yaw) <= maximum_trusted_yaw
        minimum_entry_gap = self.number(
            "dock_minimum_entry_gap_m", 0.06, 0.03, 0.15
        )
        distance_ready = minimum_entry_gap <= forward_actual <= standoff + 0.035
        yaw_ready = abs(yaw) < math.radians(3.0)
        tape_enabled = (
            (
                AutoDockNode.boolean(self, "tape_guidance_enabled", False)
                or AutoDockNode.boolean(self, "tape_guidance_only", False)
            )
            if hasattr(self, "config") else False
        )
        initial_tape_approach_ready = (
            not tape_enabled
            or getattr(self, "tape_initial_detection_complete", False)
        )
        if (
            distance_ready
            and abs(lateral_actual) < 0.025
            and yaw_ready
            and initial_tape_approach_ready
        ):
            self.stop_drive(5)
            self.insertion_entry_gap_m = max(0.0, forward_actual)
            pause = self.number("motion_transition_pause_sec", 0.10, 0.10, 2.0)
            self.insertion_start_due_at = now + pause
            self.publish_status(
                "running", "aligned_pause_before_insertion", pause_sec=pause
            )
            return
        lateral_alignment_needed = abs(lateral_actual) >= 0.025
        tape_pose_hold_needed = (
            tape_enabled
            and (
                lateral_alignment_needed
                or not getattr(
                    self, "tape_initial_detection_complete", False
                )
            )
        )
        if tape_pose_hold_needed:
            tape_age = now - getattr(self, "latest_tape_guidance_at", 0.0)
            tape_available = (
                getattr(self, "latest_tape_guidance", None) is not None
                and tape_age <= self.number(
                    "tape_max_age_sec", 0.50, 0.10, 3.0
                )
            )
            if not tape_available:
                self.stop_drive()
                self.publish_status(
                    "waiting", "alignment_warning_tape_not_detected"
                )
                return
            if AutoDockNode.command_warning_tape_search(
                self, now, 0.0, 0.0, pose_correction_only=True
            ):
                return
        if translation_first:
            forward_error = max(0.0, forward_actual - standoff)
            forward_gain = self.number(
                "translation_alignment_forward_gain", 0.8, 0.1, 2.0
            )
            max_forward = self.number(
                "translation_alignment_max_forward_speed_m_s", 0.08, 0.03, 0.15
            )
            if getattr(self, "target_type", "SYMBOLS") == "NEAREST":
                max_forward = min(max_forward, self.number(
                    "nearest_alignment_max_forward_speed_m_s",
                    0.05, 0.02, 0.10,
                ))
            max_lateral = self.number(
                "translation_alignment_max_lateral_speed_m_s", 0.08, 0.03, 0.15
            )
            max_angular = self.number(
                "translation_alignment_max_angular_speed_rad_s", 0.12, 0.0, 0.20
            )
            angular = clamp(0.6 * yaw, -max_angular, max_angular)
            min_angular = min(max_angular, self.number(
                "translation_alignment_min_angular_speed_rad_s", 0.10, 0.0, 0.20
            ))
            if not yaw_ready and 0.0 < abs(angular) < min_angular:
                angular = math.copysign(min_angular, angular)
            forward_command = (
                0.0 if tape_pose_hold_needed else
                clamp(forward_gain * forward_error, 0.0, max_forward)
            )
            self.publish_drive(
                forward_command,
                clamp(0.80 * lateral, -max_lateral, max_lateral),
                angular,
            )
            self.publish_status(
                "running", "translation_first_alignment",
                forward_error_cm=round(forward_error * 100.0, 1),
                lateral_error_cm=round(lateral_actual * 100.0, 1),
                yaw_deg=round(math.degrees(yaw), 1),
                yaw_trusted=yaw_trusted,
                tape_distance_hold=tape_pose_hold_needed,
            )
            return
        self.publish_drive(
            0.0,
            clamp(0.80 * lateral, -0.08, 0.08),
            clamp(1.20 * yaw, -0.35, 0.35),
        )


    def tick_alignment_recovery(self):
        pose = self.alignment_recovery_pose
        if pose is None or self.odom_position is None or self.odom_yaw is None:
            self.stop_drive()
            self.publish_status("waiting", "alignment_recovery_pose_unavailable")
            return
        dx = pose[0] - self.odom_position[0]
        dy = pose[1] - self.odom_position[1]
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        forward = c * dx + s * dy
        lateral = -s * dx + c * dy
        yaw = normalize_angle(pose[2] - self.odom_yaw)
        if math.hypot(dx, dy) <= 0.01 and abs(yaw) <= math.radians(1.0):
            self.stop_drive()
            self.alignment_recovery_pose = None
            self.alignment_bad_frames = 0
            self.alignment_good_frames = 0
            self.alignment_last_good_stamp = None
            self.alignment_lost_since = time.monotonic()
            self.publish_status("waiting", "alignment_best_pose_restored")
            return
        max_linear = self.number(
            "alignment_recovery_max_speed_m_s", 0.04, 0.01, 0.08
        )
        max_angular = self.number(
            "alignment_recovery_max_angular_speed_rad_s", 0.08, 0.02, 0.15
        )
        self.publish_drive(
            clamp(0.8 * forward, -max_linear, max_linear),
            clamp(0.8 * lateral, -max_linear, max_linear),
            clamp(0.8 * yaw, -max_angular, max_angular),
        )
        self.publish_status(
            "recovering", "returning_to_alignment_best_pose",
            remaining_cm=round(math.hypot(dx, dy) * 100.0, 1),
            yaw_error_deg=round(math.degrees(yaw), 1),
        )

    def tick_inserting(self):
        if self.insert_start_position is None or self.odom_position is None:
            self.cancel("odom_missing")
            return
        now = time.monotonic()
        fork_command_due_at = getattr(self, "fork_command_due_at", None)
        if fork_command_due_at is not None:
            self.stop_drive()
            if now < fork_command_due_at:
                return
            self.fork_command_due_at = None
            command = "UP" if self.operation == "PICK" else "DOWN"
            self.state = "waiting_fork"
            self.fork_pub.publish(String(data=command))
            self.publish_status("waiting", "fork_command_sent", fork_command=command)
            return
        insertion_depth_m = self.number(
            "insertion_distance_cm", 12.0, 1.0, 100.0
        ) / 100.0
        entry_gap_m = getattr(self, "insertion_entry_gap_m", None)
        if entry_gap_m is None:
            entry_gap_m = self.number("dock_standoff_m", 0.20, 0.03, 1.0)
        start_yaw = getattr(self, "insert_start_yaw", None)
        if start_yaw is None:
            start_yaw = getattr(self, "odom_yaw", 0.0) or 0.0
        dx = self.odom_position[0] - self.insert_start_position[0]
        dy = self.odom_position[1] - self.insert_start_position[1]
        raw_forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        travelled_actual = max(0.0, raw_forward) * self.number(
            "distance_coefficient", 1.0, 0.10, 2.0
        )
        required_actual = entry_gap_m + insertion_depth_m
        if travelled_actual >= required_actual:
            self.stop_drive(10)
            self.completed_insertion_distance_m = travelled_actual
            pause = self.number("motion_transition_pause_sec", 0.10, 0.10, 2.0)
            self.fork_command_due_at = now + pause
            self.publish_status(
                "running", "insertion_complete_pause_before_fork",
                pause_sec=pause,
                travelled_cm=round(travelled_actual * 100.0, 1),
                entry_gap_cm=round(entry_gap_m * 100.0, 1),
                insertion_depth_cm=round(insertion_depth_m * 100.0, 1),
            )
            return
        candidate, pnp, _reason = self.valid_measurement()
        if candidate is not None:
            self.update_world_target(candidate, pnp, blend_existing=True)
        target = self.target_in_body()
        if target is None:
            self.cancel("virtual_target_missing_during_insertion")
            return
        _forward, lateral, yaw = target
        insertion_speed = self.number(
            "insertion_speed_m_s", 0.05, 0.01, 0.20
        )
        lateral_gain = self.number(
            "insertion_lateral_gain", 0.50, 0.0, 2.0
        )
        max_lateral = self.number(
            "insertion_max_lateral_speed_m_s", 0.025, 0.0, 0.08
        )
        yaw_gain = self.number("insertion_yaw_gain", 0.80, 0.0, 3.0)
        max_yaw = self.number(
            "insertion_max_angular_speed_rad_s", 0.12, 0.0, 0.35
        )
        self.publish_drive(
            insertion_speed,
            clamp(lateral_gain * lateral, -max_lateral, max_lateral),
            clamp(yaw_gain * yaw, -max_yaw, max_yaw),
        )

    def tick_reversing_after_lift(self):
        if self.post_lift_reverse_start is None or self.odom_position is None:
            self.cancel("odom_missing_during_post_lift_reverse")
            return
        reverse_m = getattr(self, "post_lift_reverse_target_m", None)
        if reverse_m is None or reverse_m <= 0.0:
            self.cancel("insertion_distance_missing_during_reverse")
            return
        start_yaw = getattr(self, "post_lift_reverse_start_yaw", None)
        if start_yaw is None:
            start_yaw = getattr(self, "odom_yaw", 0.0) or 0.0
        dx = self.odom_position[0] - self.post_lift_reverse_start[0]
        dy = self.odom_position[1] - self.post_lift_reverse_start[1]
        raw_reverse = -(math.cos(start_yaw) * dx + math.sin(start_yaw) * dy)
        travelled_actual = max(0.0, raw_reverse) * self.number(
            "distance_coefficient", 1.0, 0.10, 2.0
        )
        if travelled_actual < reverse_m:
            speed = self.number(
                "post_lift_reverse_speed_m_s", 0.05, 0.01, 0.20
            )
            self.publish_drive(-speed, 0.0, 0.0)
            return
        self.stop_drive(10)
        opening_test_enabled = (
            AutoDockNode.boolean(
                self, "post_lift_rear_opening_test_enabled", False
            )
            if hasattr(self, "config") else False
        )
        if opening_test_enabled:
            self.post_lift_reverse_target_m = None
            self.right_turn_clearance_wait_started_at = None
            self.post_lift_opening_started_at = time.monotonic()
            self.post_lift_opening_reference_m = None
            self.post_lift_opening_previous_m = None
            self.post_lift_opening_confirmation_count = 0
            self.post_lift_opening_reverse_started_at = None
            imu_yaw = getattr(self, "imu_yaw", None)
            if imu_yaw is not None:
                self.post_lift_opening_heading_yaw = imu_yaw
                self.post_lift_opening_heading_source = "imu"
            else:
                self.post_lift_opening_heading_yaw = getattr(self, "odom_yaw", None)
                self.post_lift_opening_heading_source = "odom"
            self.state = "post_lift_opening_search"
            self.publish_status(
                "running", "post_lift_opening_search_started",
                direction="right",
            )
            return
        clear, blocking, turn_front_extent = self.right_turn_clearance_available()
        if turn_front_extent is None:
            now = time.monotonic()
            if self.right_turn_clearance_wait_started_at is None:
                self.right_turn_clearance_wait_started_at = now
            timeout = self.number(
                "right_turn_scan_wait_timeout_sec", 2.0, 0.5, 5.0
            )
            if now - self.right_turn_clearance_wait_started_at < timeout:
                self.publish_status(
                    "waiting", "right_turn_waiting_for_fresh_scan",
                    reversed_cm=round(travelled_actual * 100.0, 1),
                )
                return
            self.post_lift_reverse_target_m = None
            self.right_turn_clearance_wait_started_at = None
            self.state = "ready"
            self.drive_ready_pub.publish(Empty())
            self.publish_status(
                "completed", "drive_ready_right_turn_scan_timeout",
                reversed_cm=round(travelled_actual * 100.0, 1),
            )
            return
        self.post_lift_reverse_target_m = None
        self.right_turn_clearance_wait_started_at = None
        if clear and self.odom_yaw is not None:
            self.right_turn_target_yaw = normalize_angle(
                self.odom_yaw - math.radians(90.0)
            )
            self.state = "turning_right_for_ready"
            self.publish_status(
                "running", "right_turn_90_started",
                turn_front_extent_cm=round(turn_front_extent * 100.0, 1),
            )
            return
        self.state = "ready"
        self.drive_ready_pub.publish(Empty())
        self.publish_status(
            "completed", "drive_ready_right_turn_skipped",
            reversed_cm=round(travelled_actual * 100.0, 1),
            blocking_range_cm=(
                None if blocking is None or not math.isfinite(blocking)
                else round(blocking * 100.0, 1)
            ),
        )

    def tick_post_lift_opening_search(self):
        now = time.monotonic()
        maximum_scan_age = self.number(
            "post_lift_opening_scan_max_age_sec", 0.50, 0.10, 2.0
        )
        scan_age = now - getattr(self, "scan_updated_at", 0.0)
        if scan_age > maximum_scan_age:
            self.stop_drive()
            self.publish_status(
                "waiting", "post_lift_opening_scan_stale",
                scan_age_sec=round(scan_age, 2),
            )
            return
        rear_distance, rear_angle, _clearance = self.nearest_by_direction.get(
            "rear", (math.inf, None, 0.0)
        )
        if not math.isfinite(rear_distance):
            self.stop_drive()
            self.publish_status("waiting", "post_lift_opening_rear_missing")
            return
        started_at = getattr(self, "post_lift_opening_started_at", None)
        if started_at is None:
            started_at = now
            self.post_lift_opening_started_at = now
        timeout = self.number(
            "post_lift_opening_search_timeout_sec", 30.0, 1.0, 120.0
        )
        if now - started_at >= timeout:
            self.cancel("post_lift_opening_search_timeout")
            return
        reference = getattr(self, "post_lift_opening_reference_m", None)
        if reference is None:
            reference = rear_distance
        else:
            reference = min(reference, rear_distance)
        self.post_lift_opening_reference_m = reference
        self.post_lift_opening_previous_m = rear_distance
        jump_m = self.number(
            "post_lift_opening_jump_cm", 15.0, 5.0, 100.0
        ) / 100.0
        if rear_distance - reference >= jump_m:
            self.post_lift_opening_confirmation_count = (
                getattr(self, "post_lift_opening_confirmation_count", 0) + 1
            )
        else:
            self.post_lift_opening_confirmation_count = 0
        required = int(self.number(
            "post_lift_opening_confirmation_frames", 2, 1, 10
        ))
        if self.post_lift_opening_confirmation_count >= required:
            self.stop_drive(5)
            self.post_lift_opening_reverse_started_at = now
            self.state = "post_lift_opening_reverse"
            self.publish_status(
                "running", "post_lift_rear_opening_detected",
                baseline_cm=round(reference * 100.0, 1),
                rear_distance_cm=round(rear_distance * 100.0, 1),
                rear_angle_deg=(
                    None if rear_angle is None
                    else round(math.degrees(rear_angle), 1)
                ),
            )
            return
        yaw_error = 0.0
        start_yaw = getattr(self, "post_lift_opening_heading_yaw", None)
        heading_source = getattr(self, "post_lift_opening_heading_source", None)
        current_yaw = (
            getattr(self, "imu_yaw", None)
            if heading_source == "imu" else self.odom_yaw
        )
        if start_yaw is not None and current_yaw is not None:
            yaw_error = normalize_angle(start_yaw - current_yaw)
        yaw_tolerance = math.radians(self.number(
            "tag_search_yaw_tolerance_deg", 3.0, 0.5, 15.0
        ))
        if abs(yaw_error) > yaw_tolerance:
            max_yaw = self.number(
                "post_lift_opening_max_angular_speed_rad_s", 0.35, 0.10, 0.50
            )
            self.publish_drive(0.0, 0.0, math.copysign(max_yaw, yaw_error))
            self.publish_status(
                "running", "post_lift_opening_yaw_correction",
                yaw_deg=round(math.degrees(yaw_error), 1),
                rear_distance_cm=round(rear_distance * 100.0, 1),
            )
            return
        lateral_speed = self.number(
            "post_lift_opening_lateral_speed_m_s", 0.12, 0.05, 0.20
        )
        self.publish_drive(0.0, -lateral_speed, 0.0)
        self.publish_status(
            "running", "post_lift_opening_search_right",
            rear_distance_cm=round(rear_distance * 100.0, 1),
            baseline_cm=round(reference * 100.0, 1),
        )

    def tick_post_lift_opening_reverse(self):
        now = time.monotonic()
        maximum_scan_age = self.number(
            "post_lift_opening_scan_max_age_sec", 0.50, 0.10, 2.0
        )
        if now - getattr(self, "scan_updated_at", 0.0) > maximum_scan_age:
            self.stop_drive()
            self.publish_status("waiting", "post_lift_opening_reverse_scan_stale")
            return
        rear_distance, _angle, _clearance = self.nearest_by_direction.get(
            "rear", (math.inf, None, 0.0)
        )
        if not math.isfinite(rear_distance):
            self.stop_drive()
            self.publish_status("waiting", "post_lift_opening_reverse_rear_missing")
            return
        started_at = getattr(self, "post_lift_opening_reverse_started_at", None)
        if started_at is None:
            started_at = now
            self.post_lift_opening_reverse_started_at = now
        timeout = self.number(
            "post_lift_opening_reverse_timeout_sec", 10.0, 1.0, 60.0
        )
        if now - started_at >= timeout:
            self.cancel("post_lift_opening_reverse_timeout")
            return
        target_m = self.number(
            "post_lift_opening_rear_target_cm", 20.0, 5.0, 100.0
        ) / 100.0
        if rear_distance <= target_m:
            self.stop_drive(10)
            self.state = "ready"
            self.drive_ready_pub.publish(Empty())
            self.publish_status(
                "completed", "post_lift_opening_rear_target_reached",
                rear_distance_cm=round(rear_distance * 100.0, 1),
                target_cm=round(target_m * 100.0, 1),
            )
            return
        speed = self.number(
            "post_lift_opening_reverse_speed_m_s", 0.05, 0.01, 0.15
        )
        self.publish_drive(-speed, 0.0, 0.0)
        self.publish_status(
            "running", "post_lift_opening_reversing_to_rear_target",
            rear_distance_cm=round(rear_distance * 100.0, 1),
            target_cm=round(target_m * 100.0, 1),
        )

    def tick_turning_right_for_ready(self):
        clear, blocking, turn_front_extent = self.right_turn_clearance_available()
        if not clear or self.right_turn_target_yaw is None or self.odom_yaw is None:
            self.stop_drive(10)
            self.right_turn_target_yaw = None
            self.state = "ready"
            self.drive_ready_pub.publish(Empty())
            self.publish_status(
                "completed", "drive_ready_right_turn_aborted",
                blocking_range_cm=(
                    None if blocking is None or not math.isfinite(blocking)
                    else round(blocking * 100.0, 1)
                ),
                turn_front_extent_cm=(
                    None if turn_front_extent is None
                    else round(turn_front_extent * 100.0, 1)
                ),
            )
            return
        yaw_error = normalize_angle(self.right_turn_target_yaw - self.odom_yaw)
        tolerance = math.radians(
            self.number("ready_right_turn_tolerance_deg", 3.0, 0.5, 15.0)
        )
        if abs(yaw_error) <= tolerance:
            self.stop_drive(10)
            self.right_turn_target_yaw = None
            self.state = "ready"
            self.drive_ready_pub.publish(Empty())
            self.publish_status("completed", "drive_ready_after_right_turn_90")
            return
        speed = self.number("ready_right_turn_speed_rad_s", 0.20, 0.05, 0.50)
        self.publish_drive(0.0, 0.0, -min(speed, abs(yaw_error)))

    def tick_backoff(self):
        if self.backoff_until is not None and time.monotonic() < self.backoff_until:
            self.publish_drive(*self.backoff_command, 0.0)
            return
        self.stop_drive(5)
        self.backoff_until = None
        direction = self.backoff_direction
        self.backoff_direction = None
        if direction in self.nearest_by_direction:
            distance, _angle, clearance = self.nearest_by_direction[direction]
            if distance < clearance:
                self.cancel(f"lidar_{direction}_blocked_after_backoff")
                return
        interrupted_state = getattr(self, "state_before_lidar_interrupt", None)
        self.state_before_lidar_interrupt = None
        if interrupted_state == "post_lift_opening_search":
            self.state = interrupted_state
        else:
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
