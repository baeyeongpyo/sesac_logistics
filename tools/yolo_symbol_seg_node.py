#!/usr/bin/env python3
"""Run YOLO11 detection on a ROS camera and expose an MJPEG viewer."""

import json
import itertools
import math
import os
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Keep this process off Fast DDS shared memory. UDPv4 still interoperates with
# the navigation participants without making them inherit this setting.
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

import cv2
import ncnn
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String


NAMES = ["star", "diamond", "pallet", "spade", "clover", "heart"]
COLORS = [(30, 220, 255), (255, 120, 30), (40, 210, 40), (230, 80, 230), (50, 180, 50), (40, 40, 240)]


def detection_center(detection):
    x1, y1, x2, y2 = detection["box"]
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def overlap_depth(first_box, second_box):
    """Vertical overlap inferred from intersection area, normalized by overlap width."""
    x1 = max(first_box[0], second_box[0])
    y1 = max(first_box[1], second_box[1])
    x2 = min(first_box[2], second_box[2])
    y2 = min(first_box[3], second_box[3])
    overlap_width = max(0, x2 - x1)
    overlap_area = overlap_width * max(0, y2 - y1)
    return overlap_area / max(overlap_width, 1), overlap_area


def box_overlap_ratio(first_box, second_box):
    """Intersection area normalized by the smaller box area."""
    x1 = max(first_box[0], second_box[0])
    y1 = max(first_box[1], second_box[1])
    x2 = min(first_box[2], second_box[2])
    y2 = min(first_box[3], second_box[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    first_area = max(1, first_box[2] - first_box[0]) * max(
        1, first_box[3] - first_box[1]
    )
    second_area = max(1, second_box[2] - second_box[0]) * max(
        1, second_box[3] - second_box[1]
    )
    return intersection / min(first_area, second_area)


def box_iou(first_box, second_box):
    """Intersection over union for short-lived detection association."""
    x1 = max(first_box[0], second_box[0])
    y1 = max(first_box[1], second_box[1])
    x2 = min(first_box[2], second_box[2])
    y2 = min(first_box[3], second_box[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    first_area = max(1, first_box[2] - first_box[0]) * max(
        1, first_box[3] - first_box[1]
    )
    second_area = max(1, second_box[2] - second_box[0]) * max(
        1, second_box[3] - second_box[1]
    )
    return intersection / max(first_area + second_area - intersection, 1)


def assign_frame_pallet_groups(entities, minimum_overlap_ratio=0.02):
    """Group simultaneously visible pallet faces whose image boxes overlap."""
    parents = list(range(len(entities)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    for first in range(len(entities)):
        for second in range(first + 1, len(entities)):
            if box_overlap_ratio(
                entities[first]["pallet"]["box"],
                entities[second]["pallet"]["box"],
            ) >= minimum_overlap_ratio:
                union(first, second)

    group_ids = {}
    for index, entity in enumerate(entities):
        root = find(index)
        if root not in group_ids:
            group_ids[root] = len(group_ids) + 1
        entity["frame_pallet_group"] = group_ids[root]


def matching_frame_pallet_group(entity, grouped_entities, minimum_overlap_ratio=0.02):
    """Attach a target-filtered fallback face to an existing image group."""
    box = entity["pallet"]["box"]
    matches = [
        (box_overlap_ratio(box, item["pallet"]["box"]), item)
        for item in grouped_entities
        if item.get("frame_pallet_group") is not None
    ]
    if not matches:
        return None
    ratio, match = max(matches, key=lambda item: item[0])
    return match["frame_pallet_group"] if ratio >= minimum_overlap_ratio else None


def ordered_grid(tags):
    by_y = sorted(tags, key=lambda item: detection_center(item)[1])
    top = sorted(by_y[:2], key=lambda item: detection_center(item)[0])
    bottom = sorted(by_y[2:], key=lambda item: detection_center(item)[0])
    return top + bottom


def entity_center(entity):
    if entity.get("complete"):
        points = entity["tag_centers"]
        return (sum(point[0] for point in points) / 4.0, sum(point[1] for point in points) / 4.0)
    return detection_center(entity["pallet"])


def pallet_entities(detections, target_top=None):
    """Associate each symbol with the horizontally nearest pallet below it."""
    pallets = [item for item in detections if item["class"] == "pallet"]
    symbols = [item for item in detections if item["class"] != "pallet"]
    grouped = {id(pallet): [] for pallet in pallets}
    for symbol in symbols:
        symbol_x, symbol_y = detection_center(symbol)
        choices = []
        for pallet in pallets:
            x1, y1, x2, y2 = pallet["box"]
            pallet_width = max(1, x2 - x1)
            pallet_height = max(1, y2 - y1)
            margin = pallet_width * 0.12
            vertical_reach = max(4.0 * pallet_height, 0.9 * pallet_width)
            if x1 - margin <= symbol_x <= x2 + margin and y1 - vertical_reach <= symbol_y <= y2:
                pallet_x, _ = detection_center(pallet)
                horizontal_error = abs(symbol_x - pallet_x) / pallet_width
                vertical_error = abs(symbol_y - y1) / vertical_reach
                # Adjacent visible pallet faces overlap in x.  Horizontal
                # distance alone lets the wider face steal the other face's
                # tags, so also prefer the pallet immediately below the tag.
                choices.append((horizontal_error + 0.35 * vertical_error, pallet))
        if choices:
            grouped[id(min(choices, key=lambda item: item[0])[1])].append(symbol)

    entities = []
    for pallet in pallets:
        candidates = grouped[id(pallet)]
        choices = []
        for combination in itertools.combinations(candidates, 4):
            ordered = ordered_grid(combination)
            if target_top is not None and tuple(item["class"] for item in ordered[:2]) != target_top:
                continue
            centers = [detection_center(item) for item in ordered]
            column_error = abs(centers[0][0] - centers[2][0]) + abs(centers[1][0] - centers[3][0])
            confidence_bonus = 20.0 * sum(item["confidence"] for item in ordered)
            choices.append((column_error - confidence_bonus, list(combination)))
        tags = min(choices, key=lambda item: item[0])[1] if choices else []
        entity = {"pallet": pallet, "tags": tags, "complete": len(tags) == 4}
        if entity["complete"]:
            ordered = ordered_grid(tags)
            centers = [detection_center(item) for item in ordered]
            h_left = abs(centers[2][1] - centers[0][1])
            h_right = abs(centers[3][1] - centers[1][1])
            row_scale = max((h_left + h_right) * 0.5, 1.0)
            top_row_error = (centers[1][1] - centers[0][1]) / row_scale
            bottom_row_error = (centers[3][1] - centers[2][1]) / row_scale
            left_overlap_depth, left_overlap_area = overlap_depth(pallet["box"], ordered[2]["box"])
            right_overlap_depth, right_overlap_area = overlap_depth(pallet["box"], ordered[3]["box"])
            bottom_center_dx = max(abs(centers[3][0] - centers[2][0]), 1.0)
            overlap_angle = math.degrees(math.atan2(
                right_overlap_depth - left_overlap_depth, bottom_center_dx
            ))
            entity["ordered_tags"] = ordered
            entity["tag_centers"] = centers
            entity["top_row_error"] = top_row_error
            entity["bottom_row_error"] = bottom_row_error
            entity["left_overlap_area"] = left_overlap_area
            entity["right_overlap_area"] = right_overlap_area
            entity["overlap_angle"] = overlap_angle
            entity["frontal_error"] = (top_row_error + bottom_row_error) * 0.5
        entities.append(entity)
    if target_top is not None and not any(entity["complete"] for entity in entities):
        grid_choices = []
        for combination in itertools.combinations(symbols, 4):
            ordered = ordered_grid(combination)
            if tuple(item["class"] for item in ordered[:2]) != target_top:
                continue
            centers = [detection_center(item) for item in ordered]
            top_width = centers[1][0] - centers[0][0]
            bottom_width = centers[3][0] - centers[2][0]
            row_gap = ((centers[2][1] + centers[3][1]) - (centers[0][1] + centers[1][1])) * 0.5
            if min(top_width, bottom_width, row_gap) < 20.0:
                continue
            score = (
                abs(centers[0][0] - centers[2][0])
                + abs(centers[1][0] - centers[3][0])
                + abs(top_width - bottom_width)
                - 20.0 * sum(item["confidence"] for item in ordered)
            )
            grid_choices.append((score, ordered, centers))
        if grid_choices:
            _, ordered, centers = min(grid_choices, key=lambda item: item[0])
            grid_x = sum(point[0] for point in centers) / 4.0
            bottom_y = max(item["box"][3] for item in ordered[2:])
            pallet = min(
                pallets,
                key=lambda item: abs(detection_center(item)[0] - grid_x) + 0.3 * abs(item["box"][1] - bottom_y),
            ) if pallets else {"class": "pallet", "confidence": 0.0, "box": [0, 0, 0, 0]}
            h_left = abs(centers[2][1] - centers[0][1])
            h_right = abs(centers[3][1] - centers[1][1])
            row_scale = max((h_left + h_right) * 0.5, 1.0)
            left_depth, left_area = overlap_depth(pallet["box"], ordered[2]["box"])
            right_depth, right_area = overlap_depth(pallet["box"], ordered[3]["box"])
            top_error = (centers[1][1] - centers[0][1]) / row_scale
            bottom_error = (centers[3][1] - centers[2][1]) / row_scale
            entities.append({
                "pallet": pallet, "tags": ordered, "complete": True,
                "ordered_tags": ordered, "tag_centers": centers,
                "top_row_error": top_error, "bottom_row_error": bottom_error,
                "frontal_error": (top_error + bottom_error) * 0.5,
                "left_overlap_area": left_area, "right_overlap_area": right_area,
                "overlap_angle": math.degrees(math.atan2(
                    right_depth - left_depth, max(abs(centers[3][0] - centers[2][0]), 1.0)
                )),
            })
    return entities


def letterbox(image, size):
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    left = (size - resized.shape[1]) // 2
    top = (size - resized.shape[0]) // 2
    canvas[top : top + resized.shape[0], left : left + resized.shape[1]] = resized
    tensor = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
    tensor = np.ascontiguousarray(tensor, dtype=np.float32)[None] / 255.0
    return tensor, scale, left, top, resized.shape[1], resized.shape[0]


class YoloSymbolSeg(Node):
    def __init__(self):
        super().__init__("yolo_tag")
        self.declare_parameter("image_topic", "/ascamera/camera_publisher/rgb0/image")
        self.declare_parameter("depth_image_topic", "/ascamera/camera_publisher/depth0/image_raw")
        self.declare_parameter("camera_info_topic", "/ascamera/camera_publisher/rgb0/camera_info")
        self.declare_parameter("result_topic", "/symbol_seg/detections")
        self.declare_parameter("annotated_topic", "/symbol_seg/annotated")
        self.declare_parameter("model", "/shared/best_ncnn_model")
        self.declare_parameter("input_size", 320)
        self.declare_parameter("max_inference_fps", 4.0)
        self.declare_parameter("confidence", 0.40)
        self.declare_parameter("pallet_confidence", 0.30)
        self.declare_parameter("pallet_latch_missed_frames", 1)
        self.declare_parameter("camera_yaw_deg", 0.0)
        self.declare_parameter("yaw_bias_deg", 0.0)
        self.declare_parameter("pose_config", "/shared/vehicle_pose_config.json")
        self.declare_parameter("web_port", 8090)
        self.declare_parameter("control_port", 8091)
        self.declare_parameter("cmd_vel_topic", "/controller/cmd_vel")
        self.declare_parameter("fork_command_topic", "/fork/command")
        self.declare_parameter("target_top_left", "diamond")
        self.declare_parameter("target_top_right", "spade")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("telemetry_dir", "/shared/telemetry")
        model_dir = str(self.get_parameter("model").value)
        self.net = ncnn.Net()
        self.net.opt.num_threads = 1
        param_status = self.net.load_param(os.path.join(model_dir, "model.ncnn.param"))
        model_status = self.net.load_model(os.path.join(model_dir, "model.ncnn.bin"))
        if param_status != 0 or model_status != 0:
            raise RuntimeError(
                f"failed to load NCNN model from {model_dir}: "
                f"param={param_status}, model={model_status}"
            )
        self.input_size = int(self.get_parameter("input_size").value)
        max_inference_fps = float(self.get_parameter("max_inference_fps").value)
        self.inference_interval = 0.0 if max_inference_fps <= 0.0 else 1.0 / max_inference_fps
        self.last_inference_started = 0.0
        self.confidence = float(self.get_parameter("confidence").value)
        self.pallet_confidence = float(
            self.get_parameter("pallet_confidence").value
        )
        self.pallet_latch_missed_frames = max(
            0, int(self.get_parameter("pallet_latch_missed_frames").value)
        )
        pose_config = {}
        pose_config_path = str(self.get_parameter("pose_config").value)
        self.pose_config_path = pose_config_path
        if pose_config_path and os.path.isfile(pose_config_path):
            try:
                with open(pose_config_path, encoding="utf-8") as source:
                    pose_config = json.load(source)
            except (OSError, TypeError, ValueError):
                self.get_logger().warning(f"invalid pose config: {pose_config_path}")
        self.camera_yaw_deg = float(self.get_parameter("camera_yaw_deg").value)
        self.camera_pitch_deg = float(pose_config.get("camera_pitch_deg", 0.0))
        distance_scale = pose_config.get("camera_distance_scale_cm_per_pnp_unit")
        distance_offset = pose_config.get("camera_distance_offset_cm")
        self.camera_distance_scale = (
            None if distance_scale is None else float(distance_scale)
        )
        self.camera_distance_offset = (
            None if distance_offset is None else float(distance_offset)
        )
        self.depth_camera_to_fork_tip_offset_cm = float(
            pose_config.get("depth_camera_to_fork_tip_offset_cm", 14.0)
        )
        self.individual_tag_depth_max_age_sec = float(
            pose_config.get("individual_tag_depth_max_age_sec", 0.35)
        )
        self.friction_coefficient = float(pose_config.get("friction_coefficient", 1.0))
        self.yaw_bias_deg = float(
            pose_config.get("yaw_bias_deg", self.get_parameter("yaw_bias_deg").value)
        )
        self.target_top = (
            str(self.get_parameter("target_top_left").value),
            str(self.get_parameter("target_top_right").value),
        )
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.processed_sequence = 0
        self.latest_jpeg = None
        self.latest_jpeg_sequence = 0
        self.latest_depth = None
        self.latest_depth_stamp_ns = 0
        self.candidate_center = None
        self.candidate_streak = 0
        self.camera_matrix = None
        self.distortion = None
        self.odom_position = None
        self.odom_yaw = None
        self.entity_tracks = []
        self.next_entity_track_id = 1
        self.pallet_detection_memory = []
        self.previous_pnp_yaw = None
        self.telemetry_lock = threading.Lock()
        self.telemetry_file = None
        self.telemetry_session = None
        self.telemetry_last_flush = 0.0
        self.pnp_object_points = np.asarray([
            [-0.5, -0.5, 0.0], [0.5, -0.5, 0.0],
            [0.5, 0.5, 0.0], [-0.5, 0.5, 0.0],
        ], dtype=np.float32)
        self.stop_worker = threading.Event()
        # The ASCamera vendor publisher is RELIABLE. Some vendor DDS builds
        # discover a BEST_EFFORT reader but do not deliver the large RGB data.
        camera_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, str(self.get_parameter("result_topic").value), 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.fork_pub = self.create_publisher(String, str(self.get_parameter("fork_command_topic").value), 10)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.on_image, camera_qos
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_image_topic").value), self.on_depth, camera_qos
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info, camera_qos,
        )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan, 10)
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()
        self.web_server = self.start_web_server(int(self.get_parameter("web_port").value))
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.settimeout(0.1)
        self.control_socket.bind(("0.0.0.0", int(self.get_parameter("control_port").value)))
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.get_logger().info(f"YOLO NCNN detection started: {model_dir}")

    def control_loop(self):
        last_drive = 0.0
        drive_active = False
        while not self.stop_worker.is_set():
            try:
                payload, _address = self.control_socket.recvfrom(65535)
                command = json.loads(payload.decode("utf-8"))
                if command.get("type") == "drive":
                    message = Twist()
                    message.linear.x = float(command.get("linear_x", 0.0))
                    message.linear.y = float(command.get("linear_y", 0.0))
                    message.angular.z = float(command.get("angular_z", 0.0))
                    self.cmd_pub.publish(message)
                    self.log_telemetry("drive", {
                        "linear_x": message.linear.x, "linear_y": message.linear.y,
                        "angular_z": message.angular.z,
                    })
                    last_drive = time.monotonic()
                    drive_active = any(abs(value) > 1e-6 for value in (
                        message.linear.x, message.linear.y, message.angular.z
                    ))
                elif command.get("type") == "fork":
                    self.fork_pub.publish(String(data=str(command.get("command", "STOP"))))
                    self.log_telemetry("fork", {"command": str(command.get("command", "STOP"))})
                elif command.get("type") == "target":
                    top_left = str(command.get("top_left", ""))
                    top_right = str(command.get("top_right", ""))
                    valid_symbols = set(NAMES) - {"pallet"}
                    if top_left in valid_symbols and top_right in valid_symbols:
                        self.target_top = (top_left, top_right)
                        self.candidate_center = None
                        self.candidate_streak = 0
                        self.previous_pnp_yaw = None
                        self.get_logger().info(f"target changed: {top_left}/{top_right}")
                        self.log_telemetry("target", {
                            "top_left": top_left, "top_right": top_right,
                        })
                elif command.get("type") == "pose_config":
                    self.camera_pitch_deg = float(command["camera_pitch_deg"])
                    self.camera_distance_scale = float(
                        command["camera_distance_scale_cm_per_pnp_unit"]
                    )
                    self.camera_distance_offset = float(
                        command["camera_distance_offset_cm"]
                    )
                    self.yaw_bias_deg = float(command["yaw_bias_deg"])
                    self.previous_pnp_yaw = None
                    self.persist_camera_pose_config()
                    self.get_logger().info(
                        f"camera calibration changed: pitch={self.camera_pitch_deg:+.2f} deg"
                    )
                elif command.get("type") == "recording":
                    if command.get("action") == "start":
                        self.start_telemetry(str(command.get("session", "teleop")))
                    elif command.get("action") == "stop":
                        self.stop_telemetry(str(command.get("session", "")))
                elif command.get("type") == "telemetry_event":
                    event_type = re.sub(
                        r"[^A-Za-z0-9_.-]", "_", str(command.get("event_type", "gui"))
                    )
                    data = command.get("data", {})
                    self.log_telemetry(
                        event_type, data if isinstance(data, dict) else {"value": data}
                    )
            except socket.timeout:
                if drive_active and time.monotonic() - last_drive > 0.35:
                    self.cmd_pub.publish(Twist())
                    drive_active = False
            except (OSError, ValueError, TypeError, UnicodeDecodeError):
                continue

    def persist_camera_pose_config(self):
        if not self.pose_config_path:
            return
        path = Path(self.pose_config_path)
        data = {}
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, TypeError, ValueError, json.JSONDecodeError):
                data = {}
        data.update({
            "camera_pitch_deg": self.camera_pitch_deg,
            "camera_distance_scale_cm_per_pnp_unit": self.camera_distance_scale,
            "camera_distance_offset_cm": self.camera_distance_offset,
            "yaw_bias_deg": self.yaw_bias_deg,
        })
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if path.exists():
            os.chmod(temporary, path.stat().st_mode)
        os.replace(temporary, path)

    def start_web_server(self, port):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    body = b"<html><head><title>Vehicle 1 YOLO</title></head><body style='margin:0;background:#111;display:grid;place-items:center'><img src='/stream' style='max-width:100vw;max-height:100vh'></body></html>"
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith("/telemetry/"):
                    session = re.sub(r"[^A-Za-z0-9_.-]", "_", self.path.split("/telemetry/", 1)[1])
                    path = os.path.join(str(node.get_parameter("telemetry_dir").value), session)
                    with node.telemetry_lock:
                        if node.telemetry_file is not None:
                            node.telemetry_file.flush()
                    if not os.path.isfile(path):
                        self.send_error(404)
                        return
                    with open(path, "rb") as source:
                        body = source.read()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/x-ndjson")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                sent_sequence = -1
                try:
                    while not node.stop_worker.is_set():
                        with node.lock:
                            jpeg = node.latest_jpeg
                            jpeg_sequence = node.latest_jpeg_sequence
                        if jpeg is not None and jpeg_sequence != sent_sequence:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                            sent_sequence = jpeg_sequence
                        time.sleep(0.01)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, _format, *_args):
                pass

        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server

    def on_image(self, message):
        with self.lock:
            self.latest = message
            self.sequence += 1

    def on_depth(self, message):
        if message.encoding not in ("16UC1", "32FC1"):
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except Exception:
            return
        with self.lock:
            self.latest_depth = depth.copy()
            self.latest_depth_stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )

    def on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        with self.lock:
            self.camera_matrix = matrix
            self.distortion = np.asarray(message.d, dtype=np.float64)

    def on_odom(self, message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self.lock:
            self.odom_position = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
            self.odom_yaw = yaw
        self.log_telemetry("odom", {
            "position": [message.pose.pose.position.x, message.pose.pose.position.y],
            "orientation_z": message.pose.pose.orientation.z,
            "orientation_w": message.pose.pose.orientation.w,
            "twist": [message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.angular.z],
        })

    def on_scan(self, message):
        front = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(20) and math.isfinite(value):
                if message.range_min <= value <= message.range_max:
                    front.append(float(value))
        self.log_telemetry("scan", {"front_min_m": min(front) if front else None})

    def remember_complete_entity(
        self, entity, pnp, depth_yaw=None, excluded_track_ids=None
    ):
        """Add/update a short-lived target map entry from a full 2x2 detection."""
        if pnp is None or pnp.get("forward_distance_cm") is None:
            return None
        with self.lock:
            position = self.odom_position
            yaw = self.odom_yaw
        if position is None or yaw is None:
            return None
        forward = float(pnp["forward_distance_cm"]) / 100.0
        left = -float(pnp.get("lateral_ratio", 0.0)) * forward
        world_x = position[0] + math.cos(yaw) * forward - math.sin(yaw) * left
        world_y = position[1] + math.sin(yaw) * forward + math.cos(yaw) * left
        relative_face_yaw = (
            float(depth_yaw["yaw_deg"])
            if isinstance(depth_yaw, dict) and depth_yaw.get("yaw_deg") is not None
            else float(pnp.get("yaw_deg", 0.0))
        )
        face_yaw = yaw - math.radians(relative_face_yaw)
        world_yaw = math.atan2(math.sin(face_yaw), math.cos(face_yaw))
        matrix = tuple(item["class"] for item in entity["ordered_tags"])
        now = time.monotonic()
        self.entity_tracks = [
            track for track in self.entity_tracks if now - track["seen_at"] <= 20.0
        ]
        matches = [
            track for track in self.entity_tracks
            if track["matrix"] == matrix
            and track["id"] not in set(excluded_track_ids or ())
            and math.dist((world_x, world_y), (track["world_x"], track["world_y"])) <= 0.45
        ]
        if matches:
            track = min(matches, key=lambda item: math.dist(
                (world_x, world_y), (item["world_x"], item["world_y"])
            ))
            seen_count = (
                int(track.get("seen_count", 1)) + 1
                if now - track["seen_at"] <= 0.50 else 1
            )
            track.update(
                world_x=world_x, world_y=world_y, world_yaw=world_yaw,
                seen_at=now, seen_count=seen_count,
            )
        else:
            track = {
                "id": self.next_entity_track_id,
                "matrix": matrix,
                "world_x": world_x,
                "world_y": world_y,
                "world_yaw": world_yaw,
                "seen_at": now,
                "seen_count": 1,
            }
            self.next_entity_track_id += 1
            self.entity_tracks.append(track)
        return track

    @staticmethod
    def face_visibility_score(entity, pnp, depth_yaw):
        """Rank simultaneously visible faces by size, frontal angle, and PnP fit."""
        x1, y1, x2, y2 = entity["pallet"]["box"]
        area = max(float(x2 - x1), 1.0) * max(float(y2 - y1), 1.0)
        angle = (
            float(depth_yaw["yaw_deg"])
            if isinstance(depth_yaw, dict) and depth_yaw.get("yaw_deg") is not None
            else float(pnp.get("yaw_deg", 0.0))
        )
        frontal = max(0.10, math.cos(math.radians(min(abs(angle), 89.0))))
        reprojection = max(float(pnp.get("reprojection_error_px", 0.0)), 0.0)
        return area * frontal / (1.0 + reprojection / 5.0)

    def match_partial_entity(self, detections):
        """Match only the upper target pair against the local odom-based map."""
        with self.lock:
            position = self.odom_position
            yaw = self.odom_yaw
            matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
        if position is None or yaw is None or matrix is None:
            return None
        now = time.monotonic()
        self.entity_tracks = [
            track for track in self.entity_tracks if now - track["seen_at"] <= 20.0
        ]
        left_name, right_name = self.target_top
        left_tags = [item for item in detections if item["class"] == left_name]
        right_tags = [item for item in detections if item["class"] == right_name]
        scores = []
        for left_tag in left_tags:
            for right_tag in right_tags:
                if left_tag is right_tag:
                    continue
                left_center = detection_center(left_tag)
                right_center = detection_center(right_tag)
                if left_center[0] >= right_center[0]:
                    continue
                average_height = max(
                    (left_tag["box"][3] - left_tag["box"][1]
                     + right_tag["box"][3] - right_tag["box"][1]) * 0.5,
                    1.0,
                )
                if abs(left_center[1] - right_center[1]) > average_height * 0.65:
                    continue
                pair_u = (left_center[0] + right_center[0]) * 0.5
                observed_bearing = -math.atan2(pair_u - matrix[0, 2], matrix[0, 0])
                for track in self.entity_tracks:
                    if track["matrix"][:2] != (left_name, right_name):
                        continue
                    dx = track["world_x"] - position[0]
                    dy = track["world_y"] - position[1]
                    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
                    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    if forward <= 0.10:
                        continue
                    expected_bearing = math.atan2(left, forward)
                    error = abs(math.atan2(
                        math.sin(observed_bearing - expected_bearing),
                        math.cos(observed_bearing - expected_bearing),
                    ))
                    scores.append((error, track, left_tag, right_tag, observed_bearing))
        if not scores:
            return None
        scores.sort(key=lambda item: item[0])
        best = scores[0]
        # Never pick between neighbouring identical-tag pallets without a
        # clear bearing separation; wait for a complete entity instead.
        if best[0] > math.radians(10.0):
            return None
        if len(scores) > 1 and scores[1][0] - best[0] < math.radians(3.0):
            return None
        return {
            "entity_id": best[1]["id"],
            "left_tag": best[2],
            "right_tag": best[3],
            "bearing_error_deg": math.degrees(best[0]),
            "age_sec": now - best[1]["seen_at"],
        }

    def depth_yaw_from_pair(self, left_tag, right_tag, rgb_stamp_ns):
        """Estimate pallet-face yaw from registered depth at the top tag centers."""
        with self.lock:
            depth = None if self.latest_depth is None else self.latest_depth.copy()
            depth_stamp_ns = self.latest_depth_stamp_ns
            matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
        if (
            depth is None or matrix is None or depth_stamp_ns <= 0
            or abs(depth_stamp_ns - rgb_stamp_ns) > 150_000_000
        ):
            return None

        def median_depth_m(tag):
            u, v = (round(value) for value in detection_center(tag))
            y0, y1 = max(0, v - 2), min(depth.shape[0], v + 3)
            x0, x1 = max(0, u - 2), min(depth.shape[1], u + 3)
            if y0 >= y1 or x0 >= x1:
                return None
            values = depth[y0:y1, x0:x1].astype(np.float32).reshape(-1)
            if depth.dtype == np.uint16:
                values *= 0.001  # ASCamera 16UC1 is millimetres.
            values = values[np.isfinite(values) & (values >= 0.15) & (values <= 6.0)]
            return None if values.size < 5 else float(np.median(values))

        left_depth = median_depth_m(left_tag)
        right_depth = median_depth_m(right_tag)
        if left_depth is None or right_depth is None:
            return None
        left_u, _ = detection_center(left_tag)
        right_u, _ = detection_center(right_tag)
        fx, cx = matrix[0, 0], matrix[0, 2]
        left_x = (left_u - cx) * left_depth / fx
        right_x = (right_u - cx) * right_depth / fx
        baseline = math.hypot(right_x - left_x, right_depth - left_depth)
        if baseline < 0.03:
            return None
        camera_depth_m = 0.5 * (left_depth + right_depth)
        ground_forward_m = camera_depth_m * math.cos(
            math.radians(self.camera_pitch_deg)
        )
        forward_distance_cm = (
            ground_forward_m * 100.0 - self.depth_camera_to_fork_tip_offset_cm
        )
        return {
            "yaw_deg": math.degrees(math.atan2(
                right_depth - left_depth, right_x - left_x
            )),
            "left_depth_m": left_depth,
            "right_depth_m": right_depth,
            "baseline_m": baseline,
            "forward_distance_cm": forward_distance_cm,
            "distance_reference": "fork tip to target front face",
        }

    def attach_individual_tag_depth(self, detections, rgb_stamp_ns):
        """Attach registered center depth to every visible symbol detection."""
        with self.lock:
            depth = None if self.latest_depth is None else self.latest_depth.copy()
            depth_stamp_ns = self.latest_depth_stamp_ns
            matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
        maximum_age_ns = int(
            self.individual_tag_depth_max_age_sec * 1_000_000_000
        )
        if (
            depth is None or matrix is None or depth_stamp_ns <= 0
            or abs(depth_stamp_ns - rgb_stamp_ns) > maximum_age_ns
        ):
            return
        fx, cx = float(matrix[0, 0]), float(matrix[0, 2])
        for detection in detections:
            if detection.get("class") == "pallet":
                continue
            u, v = (round(value) for value in detection_center(detection))
            y0, y1 = max(0, v - 2), min(depth.shape[0], v + 3)
            x0, x1 = max(0, u - 2), min(depth.shape[1], u + 3)
            if y0 >= y1 or x0 >= x1:
                continue
            values = depth[y0:y1, x0:x1].astype(np.float32).reshape(-1)
            if depth.dtype == np.uint16:
                values *= 0.001
            values = values[
                np.isfinite(values) & (values >= 0.15) & (values <= 6.0)
            ]
            if values.size < 5:
                continue
            camera_depth_m = float(np.median(values))
            ground_forward_m = camera_depth_m * math.cos(
                math.radians(self.camera_pitch_deg)
            )
            forward_distance_cm = (
                ground_forward_m * 100.0 - self.depth_camera_to_fork_tip_offset_cm
            )
            horizontal_m = (float(u) - cx) * camera_depth_m / fx
            detection["depth"] = {
                "camera_depth_m": camera_depth_m,
                "forward_distance_cm": forward_distance_cm,
                "bearing_deg": math.degrees(
                    math.atan2(horizontal_m, camera_depth_m)
                ),
                "distance_reference": "fork tip to tag face",
            }

    def start_telemetry(self, session):
        safe_session = re.sub(r"[^A-Za-z0-9_.-]", "_", session)
        directory = str(self.get_parameter("telemetry_dir").value)
        os.makedirs(directory, exist_ok=True)
        with self.telemetry_lock:
            if self.telemetry_file is not None:
                self.telemetry_file.close()
            self.telemetry_session = safe_session
            self.telemetry_file = open(os.path.join(directory, safe_session + ".jsonl"), "w", buffering=1)
            self.telemetry_last_flush = time.monotonic()
        self.log_telemetry("recording_start", {"session": safe_session})

    def stop_telemetry(self, session=""):
        with self.telemetry_lock:
            if self.telemetry_file is None:
                return
            self.telemetry_file.write(json.dumps({
                "time": time.time(), "monotonic": time.monotonic(),
                "type": "recording_stop", "data": {"session": session},
            }) + "\n")
            self.telemetry_file.close()
            self.telemetry_file = None
            self.telemetry_session = None

    def log_telemetry(self, event_type, data):
        with self.telemetry_lock:
            if self.telemetry_file is None:
                return
            self.telemetry_file.write(json.dumps({
                "time": time.time(), "monotonic": time.monotonic(),
                "type": event_type, "data": data,
            }, separators=(",", ":")) + "\n")
            now = time.monotonic()
            if now - self.telemetry_last_flush >= 1.0:
                self.telemetry_file.flush()
                self.telemetry_last_flush = now

    def estimate_target_pose(self, entity):
        with self.lock:
            matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
            distortion = None if self.distortion is None else self.distortion.copy()
        if matrix is None:
            return None
        centers = entity["tag_centers"]
        image_points = np.asarray(
            [centers[0], centers[1], centers[3], centers[2]], dtype=np.float32
        )
        try:
            solved, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                self.pnp_object_points, image_points, matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error:
            return None
        choices = []
        if solved:
            for rvec, tvec in zip(rvecs, tvecs):
                translation = np.asarray(tvec, dtype=np.float64).reshape(3)
                if translation[2] <= 0.0:
                    continue
                projected, _ = cv2.projectPoints(
                    self.pnp_object_points, rvec, tvec, matrix, distortion
                )
                reprojection = float(np.mean(np.linalg.norm(
                    projected.reshape(-1, 2) - image_points, axis=1
                )))
                rotation, _ = cv2.Rodrigues(rvec)
                normal = rotation[:, 2]
                if normal[2] > 0.0:
                    normal = -normal
                forward = -normal
                yaw = math.degrees(math.atan2(float(forward[0]), float(forward[2])))
                choices.append((reprojection, translation, yaw, forward))
        if not choices:
            return None
        if self.previous_pnp_yaw is None:
            selected = min(choices, key=lambda item: item[0])
        else:
            selected = min(
                choices,
                key=lambda item: item[0] + 0.08 * abs(
                    math.degrees(math.atan2(
                        math.sin(math.radians(item[2] - self.previous_pnp_yaw)),
                        math.cos(math.radians(item[2] - self.previous_pnp_yaw)),
                    ))
                ),
            )
        reprojection, translation, raw_yaw, face_forward = selected
        self.previous_pnp_yaw = raw_yaw
        camera_yaw = math.radians(self.camera_yaw_deg)
        camera_pitch = math.radians(self.camera_pitch_deg)
        lateral = float(translation[0])
        forward = (
            math.sin(camera_pitch) * float(translation[1])
            + math.cos(camera_pitch) * float(translation[2])
        )
        vehicle_lateral = math.cos(camera_yaw) * lateral + math.sin(camera_yaw) * forward
        vehicle_forward = -math.sin(camera_yaw) * lateral + math.cos(camera_yaw) * forward
        raw_lateral_ratio = lateral / max(float(translation[2]), 1e-6)
        face_lateral = float(face_forward[0])
        face_forward_ground = (
            math.sin(camera_pitch) * float(face_forward[1])
            + math.cos(camera_pitch) * float(face_forward[2])
        )
        vehicle_face_lateral = (
            math.cos(camera_yaw) * face_lateral
            + math.sin(camera_yaw) * face_forward_ground
        )
        vehicle_face_forward = (
            -math.sin(camera_yaw) * face_lateral
            + math.cos(camera_yaw) * face_forward_ground
        )
        yaw_before_bias = math.degrees(math.atan2(
            vehicle_face_lateral, vehicle_face_forward
        ))
        corrected_yaw = yaw_before_bias - self.yaw_bias_deg
        corrected_yaw = math.degrees(math.atan2(
            math.sin(math.radians(corrected_yaw)),
            math.cos(math.radians(corrected_yaw)),
        ))
        if abs(vehicle_forward) < 1e-6:
            vehicle_forward = math.copysign(1e-6, vehicle_forward or 1.0)
        raw_camera_pitch = math.degrees(math.atan2(
            float(face_forward[1]), float(face_forward[2])
        ))
        forward_distance_cm = None
        if self.camera_distance_scale is not None and self.camera_distance_offset is not None:
            forward_distance_cm = (
                self.camera_distance_scale * vehicle_forward
                + self.camera_distance_offset
            )
        return {
            "yaw_deg": corrected_yaw,
            "lateral_ratio": vehicle_lateral / vehicle_forward,
            "raw_yaw_deg": raw_yaw,
            "raw_lateral_ratio": raw_lateral_ratio,
            "camera_yaw_deg": self.camera_yaw_deg,
            "camera_pitch_deg": self.camera_pitch_deg,
            "yaw_bias_deg": self.yaw_bias_deg,
            "yaw_before_bias_deg": yaw_before_bias,
            "reprojection_error_px": reprojection,
            "translation_x_units": float(translation[0]),
            "translation_y_units": float(translation[1]),
            "translation_z_units": float(translation[2]),
            "pnp_forward_units": vehicle_forward,
            "raw_camera_pitch_deg": raw_camera_pitch,
            "forward_distance_cm": forward_distance_cm,
            "distance_reference": "fork tip to target front face",
        }

    def worker_loop(self):
        while not self.stop_worker.is_set():
            delay = self.inference_interval - (time.monotonic() - self.last_inference_started)
            if delay > 0.0:
                self.stop_worker.wait(min(delay, 0.01))
                continue
            with self.lock:
                message = None if self.sequence == self.processed_sequence else self.latest
                self.processed_sequence = self.sequence
            if message is None:
                time.sleep(0.01)
                continue
            try:
                self.last_inference_started = time.monotonic()
                self.infer(message)
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")

    def latch_pallet_detections(self, detections, minimum_iou=0.30):
        """Keep a pallet through one missed inference (one hit in two frames)."""
        symbols = [item for item in detections if item.get("class") != "pallet"]
        current = [dict(item) for item in detections if item.get("class") == "pallet"]
        unused = set(range(len(current)))
        output = []
        next_memory = []
        for previous in self.pallet_detection_memory:
            best_index = None
            best_iou = 0.0
            for index in unused:
                overlap = box_iou(previous["box"], current[index]["box"])
                if overlap > best_iou:
                    best_index, best_iou = index, overlap
            if best_index is not None and best_iou >= minimum_iou:
                item = current[best_index]
                item.pop("latched", None)
                output.append(item)
                next_memory.append({**item, "missed_frames": 0})
                unused.remove(best_index)
                continue
            missed_frames = int(previous.get("missed_frames", 0)) + 1
            if missed_frames <= self.pallet_latch_missed_frames:
                held = {
                    key: value for key, value in previous.items()
                    if key != "missed_frames"
                }
                held["latched"] = True
                output.append(held)
                next_memory.append({**held, "missed_frames": missed_frames})
        for index in sorted(unused):
            item = current[index]
            item.pop("latched", None)
            output.append(item)
            next_memory.append({**item, "missed_frames": 0})
        self.pallet_detection_memory = next_memory
        return symbols + output

    def infer(self, message):
        started = time.perf_counter()
        source_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        tensor, scale, left, top, _resized_width, _resized_height = letterbox(frame, self.input_size)
        with self.net.create_extractor() as extractor:
            input_status = extractor.input("in0", ncnn.Mat(tensor[0]))
            output_status, output = extractor.extract("out0")
        if input_status != 0 or output_status != 0:
            raise RuntimeError(
                f"NCNN inference failed: input={input_status}, output={output_status}"
            )
        predictions = np.array(output).T
        class_scores = predictions[:, 4 : 4 + len(NAMES)]
        class_ids = class_scores.argmax(axis=1)
        scores = class_scores.max(axis=1)
        pallet_class_id = NAMES.index("pallet")
        score_thresholds = np.where(
            class_ids == pallet_class_id, self.pallet_confidence, self.confidence
        )
        selected = np.flatnonzero(scores >= score_thresholds)
        boxes_xywh = predictions[selected, :4]
        boxes = np.empty_like(boxes_xywh)
        boxes[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
        boxes[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
        boxes[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
        boxes[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2
        nms_boxes = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)] for x1, y1, x2, y2 in boxes]
        # Run NMS per class. A large symbol box must not suppress an overlapping
        # pallet whose intentionally lower class-specific threshold admitted it.
        kept = []
        selected_class_ids = class_ids[selected]
        for class_id in range(len(NAMES)):
            class_offsets = np.flatnonzero(selected_class_ids == class_id)
            if not len(class_offsets):
                continue
            class_boxes = [nms_boxes[int(offset)] for offset in class_offsets]
            class_box_scores = [float(scores[int(selected[offset])]) for offset in class_offsets]
            threshold = (
                self.pallet_confidence
                if class_id == pallet_class_id else self.confidence
            )
            class_kept = cv2.dnn.NMSBoxes(
                class_boxes, class_box_scores, threshold, 0.45
            )
            for kept_offset in np.asarray(class_kept).reshape(-1):
                kept.append(int(class_offsets[int(kept_offset)]))
        annotated = frame.copy()
        results = []
        for keep_index in kept:
            source_index = int(selected[keep_index])
            class_id = int(class_ids[source_index])
            score = float(scores[source_index])
            x1, y1, x2, y2 = boxes[keep_index]
            box = [
                int(np.clip((x1 - left) / scale, 0, frame.shape[1] - 1)),
                int(np.clip((y1 - top) / scale, 0, frame.shape[0] - 1)),
                int(np.clip((x2 - left) / scale, 0, frame.shape[1] - 1)),
                int(np.clip((y2 - top) / scale, 0, frame.shape[0] - 1)),
            ]
            color = COLORS[class_id]
            bx1, by1, bx2, by2 = box
            cv2.rectangle(annotated, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(annotated, f"{NAMES[class_id]} {score:.2f}", (bx1, max(20, by1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            results.append({"class": NAMES[class_id], "confidence": score, "box": box})
        results = self.latch_pallet_detections(results)
        for item in results:
            if item.get("class") != "pallet" or not item.get("latched"):
                continue
            x1, y1, x2, y2 = item["box"]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), COLORS[pallet_class_id], 2)
            cv2.putText(
                annotated, f"pallet HOLD {item['confidence']:.2f}",
                (x1, max(20, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, COLORS[pallet_class_id], 2,
            )
        self.attach_individual_tag_depth(results, source_stamp_ns)
        map_entities = [
            entity for entity in pallet_entities(results)
            if entity["complete"]
            and abs(float(entity.get("top_row_error", 999.0))) <= 0.8
            and abs(float(entity.get("bottom_row_error", 999.0))) <= 0.8
        ]
        assign_frame_pallet_groups(map_entities)
        # Build each pallet entity once without knowing the requested target.
        # Filtering combinations by target_top here creates confirmation bias:
        # nearby symbols can be rearranged into the requested answer even when
        # the best pallet entity has a different upper row.
        entities = map_entities
        complete_entities = [
            entity for entity in entities
            if entity["complete"]
            and tuple(item["class"] for item in entity["ordered_tags"][:2]) == self.target_top
            and abs(float(entity.get("top_row_error", 999.0))) <= 0.8
            and abs(float(entity.get("bottom_row_error", 999.0))) <= 0.8
        ]
        candidate = None
        if complete_entities:
            def target_rank(entity):
                x1, y1, x2, y2 = entity["pallet"]["box"]
                area = max(x2 - x1, 1) * max(y2 - y1, 1)
                center = entity_center(entity)
                previous_distance = (
                    0.0 if self.candidate_center is None
                    else math.dist(center, self.candidate_center)
                )
                image_center_distance = abs(center[0] - frame.shape[1] * 0.5)
                # A nearby pallet appears lower in the image.  Continuity and
                # center proximity are only tie breakers, so a distant object
                # near the optical center cannot steal the target.
                return (
                    -2.0 * y2 - 0.002 * area
                    + 0.25 * previous_distance + 0.05 * image_center_distance
                )

            candidate = min(complete_entities, key=target_rank)
        candidate_status = None
        entity_observations = []
        used_entity_track_ids = set()
        entity_tracks_by_object = {}
        for entity in map_entities:
            entity_pnp = self.estimate_target_pose(entity)
            entity_depth_yaw = self.depth_yaw_from_pair(
                entity["ordered_tags"][0], entity["ordered_tags"][1], source_stamp_ns
            )
            track = self.remember_complete_entity(
                entity, entity_pnp, entity_depth_yaw,
                excluded_track_ids=used_entity_track_ids,
            )
            if track is None:
                continue
            used_entity_track_ids.add(track["id"])
            entity_tracks_by_object[id(entity)] = track
            entity_observations.append({
                "entity_id": track["id"],
                "seen_count": track["seen_count"],
                "matrix": list(track["matrix"]),
                "pnp": entity_pnp,
                "depth_yaw": entity_depth_yaw,
                "frontal_error": entity["frontal_error"],
                "top_row_error": entity["top_row_error"],
                "bottom_row_error": entity["bottom_row_error"],
                "angle_source": "depth" if entity_depth_yaw is not None else "pnp",
                "visibility_score": self.face_visibility_score(
                    entity, entity_pnp, entity_depth_yaw
                ),
                "frame_pallet_group": entity.get("frame_pallet_group"),
                "image_pallet_box": list(entity["pallet"]["box"]),
                "odom_pose": {
                    "x": track["world_x"],
                    "y": track["world_y"],
                    "yaw": track["world_yaw"],
                },
            })
        tracked_partial = None
        cv2.line(annotated, (frame.shape[1] // 2, 35), (frame.shape[1] // 2, frame.shape[0] - 1), (255, 255, 255), 1)
        if candidate is None:
            self.candidate_center = None
            self.candidate_streak = 0
            self.previous_pnp_yaw = None
            tracked_partial = self.match_partial_entity(results)
            if tracked_partial is None:
                status_text = f"target {self.target_top[0]}/{self.target_top[1]}: N/A | seen 0"
            else:
                left_center = detection_center(tracked_partial["left_tag"])
                right_center = detection_center(tracked_partial["right_tag"])
                tracked_partial["center_error"] = (
                    ((left_center[0] + right_center[0]) * 0.5)
                    - frame.shape[1] * 0.5
                ) / max(frame.shape[1] * 0.5, 1.0)
                tracked_partial["depth_yaw"] = self.depth_yaw_from_pair(
                    tracked_partial["left_tag"], tracked_partial["right_tag"], source_stamp_ns
                )
                cv2.line(
                    annotated, tuple(map(round, left_center)), tuple(map(round, right_center)),
                    (255, 100, 0), 2,
                )
                for point in (left_center, right_center):
                    cv2.circle(annotated, tuple(map(round, point)), 5, (255, 100, 0), -1)
                status_text = (
                    f"tracked entity #{tracked_partial['entity_id']} | "
                    f"upper pair | map err {tracked_partial['bearing_error_deg']:.1f} deg"
                )
                if tracked_partial["depth_yaw"] is not None:
                    status_text += f" | depth yaw {tracked_partial['depth_yaw']['yaw_deg']:+.1f} deg"
        else:
            next_center = entity_center(candidate)
            px1, py1, px2, py2 = candidate["pallet"]["box"]
            continuity_limit = max(
                25.0, 0.35 * math.hypot(px2 - px1, py2 - py1)
            )
            if (
                self.candidate_center is None
                or math.dist(next_center, self.candidate_center) > continuity_limit
            ):
                self.candidate_streak = 1
                self.previous_pnp_yaw = None
            else:
                self.candidate_streak += 1
            self.candidate_center = next_center
            center_error = (self.candidate_center[0] - frame.shape[1] * 0.5) / max(frame.shape[1] * 0.5, 1.0)
            frontal_error = candidate["frontal_error"]
            top_row_error = candidate["top_row_error"]
            bottom_row_error = candidate["bottom_row_error"]
            overlap_angle = candidate["overlap_angle"]
            pnp_pose = self.estimate_target_pose(candidate)
            depth_yaw = self.depth_yaw_from_pair(
                candidate["ordered_tags"][0], candidate["ordered_tags"][1], source_stamp_ns
            )
            entity_track = entity_tracks_by_object.get(id(candidate))
            if entity_track is None:
                entity_track = self.remember_complete_entity(
                    candidate, pnp_pose, depth_yaw,
                    excluded_track_ids=used_entity_track_ids,
                )
            entity_id = None if entity_track is None else entity_track["id"]
            if entity_track is not None and not any(
                item["entity_id"] == entity_id for item in entity_observations
            ):
                entity_observations.append({
                    "entity_id": entity_id,
                    "seen_count": entity_track["seen_count"],
                    "matrix": list(entity_track["matrix"]),
                    "pnp": pnp_pose,
                    "depth_yaw": depth_yaw,
                    "frontal_error": candidate["frontal_error"],
                    "top_row_error": candidate["top_row_error"],
                    "bottom_row_error": candidate["bottom_row_error"],
                    "angle_source": "depth" if depth_yaw is not None else "pnp",
                    "visibility_score": self.face_visibility_score(
                        candidate, pnp_pose, depth_yaw
                    ),
                    "frame_pallet_group": matching_frame_pallet_group(
                        candidate, map_entities
                    ),
                    "image_pallet_box": list(candidate["pallet"]["box"]),
                    "odom_pose": {
                        "x": entity_track["world_x"],
                        "y": entity_track["world_y"],
                        "yaw": entity_track["world_yaw"],
                    },
                })
            cv2.rectangle(annotated, (px1, py1), (px2, py2), (0, 255, 255), 3)
            for point in candidate["tag_centers"]:
                cv2.circle(annotated, (round(point[0]), round(point[1])), 4, (0, 255, 255), -1)
            if pnp_pose is None:
                status_text = (
                    f"{self.target_top[0]}/{self.target_top[1]} target | "
                    f"seen {self.candidate_streak} | PnP waiting camera_info"
                )
            else:
                distance_text = (
                    ""
                    if pnp_pose["forward_distance_cm"] is None
                    else f" | dist {pnp_pose['forward_distance_cm']:.1f} cm"
                )
                if depth_yaw is not None:
                    distance_text += f" | depth yaw {depth_yaw['yaw_deg']:+.1f} deg"
                status_text = (
                    f"{self.target_top[0]}/{self.target_top[1]} | seen {self.candidate_streak} | "
                    f"lat {pnp_pose['lateral_ratio']:+.3f} | "
                    f"yaw {pnp_pose['yaw_deg']:+.1f} deg{distance_text}"
                )
            candidate_status = {
                "streak": self.candidate_streak,
                "center_error": center_error,
                "frontal_error": frontal_error,
                "top_row_error": top_row_error,
                "bottom_row_error": bottom_row_error,
                "left_overlap_area": candidate["left_overlap_area"],
                "right_overlap_area": candidate["right_overlap_area"],
                "overlap_angle_deg": overlap_angle,
                "pnp": pnp_pose,
                "pallet_box": candidate["pallet"]["box"],
                "matrix": [item["class"] for item in candidate["ordered_tags"]],
                "entity_id": entity_id,
                "depth_yaw": depth_yaw,
            }
        elapsed = time.perf_counter() - started
        cv2.putText(annotated, f"YOLO11n-detect {self.input_size} | {elapsed * 1000:.0f} ms | conf {self.confidence:.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
        cv2.rectangle(annotated, (5, frame.shape[0] - 31), (frame.shape[1] - 5, frame.shape[0] - 5), (0, 0, 0), -1)
        cv2.putText(annotated, status_text, (10, frame.shape[0] - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        result_payload = {
            "inference_sec": elapsed,
            "source_stamp_ns": source_stamp_ns,
            "target_top": list(self.target_top),
            "pose_config": {
                "camera_pitch_deg": self.camera_pitch_deg,
                "camera_distance_scale_cm_per_pnp_unit": self.camera_distance_scale,
                "camera_distance_offset_cm": self.camera_distance_offset,
                "yaw_bias_deg": self.yaw_bias_deg,
                "friction_coefficient": self.friction_coefficient,
            },
            "detections": results,
            "entities": entity_observations,
            "candidate": candidate_status,
            "tracked_partial": (
                None if tracked_partial is None else {
                    "entity_id": tracked_partial["entity_id"],
                    "bearing_error_deg": tracked_partial["bearing_error_deg"],
                    "age_sec": tracked_partial["age_sec"],
                    "center_error": tracked_partial["center_error"],
                    "depth_yaw": tracked_partial.get("depth_yaw"),
                }
            ),
        }
        self.result_pub.publish(String(data=json.dumps(result_payload)))
        self.log_telemetry("inference", result_payload)
        ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.latest_jpeg = encoded.tobytes()
                self.latest_jpeg_sequence += 1

    def close(self):
        self.stop_worker.set()
        self.stop_telemetry()
        self.control_socket.close()
        self.web_server.shutdown()
        self.worker.join(timeout=2.0)
        self.control_thread.join(timeout=1.0)


def main(args=None):
    os.environ.pop("ROS_DOMAIN_ID", None)
    rclpy.init(args=args)
    node = YoloSymbolSeg()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
