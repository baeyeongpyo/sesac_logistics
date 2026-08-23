#!/usr/bin/env python3
"""Persist complete pallet-face detections as lightweight map landmarks."""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_pose_2d(pose, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    transform_yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    c, s = math.cos(transform_yaw), math.sin(transform_yaw)
    x = float(pose["x"])
    y = float(pose["y"])
    return {
        "x": float(translation.x) + c * x - s * y,
        "y": float(translation.y) + s * x + c * y,
        "yaw": normalize_angle(transform_yaw + float(pose.get("yaw", 0.0))),
    }


class TagEntityMapper(Node):
    """Fuse YOLO 2x2 pallet faces into a persistent map-frame JSON topic."""

    def __init__(self):
        super().__init__("tag_entity_mapper")
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("detection_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("storage_path", "/shared/tag_entity_map.json")
        self.declare_parameter("merge_distance_m", 0.30)
        self.declare_parameter("merge_yaw_deg", 35.0)

        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError("vehicle cannot be inferred from vehicle/ROS_DOMAIN_ID")
        robot = f"/robot_{self.vehicle}"
        detection_topic = str(self.get_parameter("detection_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self.detection_topic = detection_topic or f"{robot}/symbol_seg/detections"
        self.output_topic = output_topic or f"{robot}/tag_entity_map"
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.storage_path = Path(str(self.get_parameter("storage_path").value))
        self.merge_distance_m = max(
            0.05, float(self.get_parameter("merge_distance_m").value)
        )
        self.merge_yaw_rad = math.radians(max(
            1.0, float(self.get_parameter("merge_yaw_deg").value)
        ))
        self.entities = []
        self.next_id = 1
        self.last_state = "waiting_for_map_tf"
        self.load_map()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(String, self.output_topic, qos)
        self.create_subscription(String, self.detection_topic, self.on_detection, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publish_map()

    def load_map(self):
        if not self.storage_path.exists():
            return
        try:
            payload = json.loads(self.storage_path.read_text(encoding="utf-8"))
            if payload.get("frame_id") != self.map_frame:
                return
            entities = payload.get("entities")
            if isinstance(entities, list):
                self.entities = [item for item in entities if isinstance(item, dict)]
                numeric_ids = [
                    int(str(item.get("id", "0")).rsplit("_", 1)[-1])
                    for item in self.entities
                    if str(item.get("id", "")).rsplit("_", 1)[-1].isdigit()
                ]
                self.next_id = max(numeric_ids, default=0) + 1
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"entity map load failed: {exc}")

    def on_detection(self, message):
        try:
            detection = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        observations = detection.get("entities")
        if not isinstance(observations, list) or not observations:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time()
            )
        except TransformException:
            if self.last_state != "waiting_for_map_tf":
                self.last_state = "waiting_for_map_tf"
                self.publish_map()
            return

        changed = False
        for observation in observations:
            matrix = observation.get("matrix") if isinstance(observation, dict) else None
            odom_pose = observation.get("odom_pose") if isinstance(observation, dict) else None
            if not isinstance(matrix, list) or len(matrix) != 4:
                continue
            if not isinstance(odom_pose, dict):
                continue
            try:
                map_pose = transform_pose_2d(odom_pose, transform)
            except (KeyError, TypeError, ValueError):
                continue
            self.merge_observation(matrix, map_pose, observation.get("entity_id"))
            changed = True
        if changed:
            self.last_state = "mapping"
            self.save_map()
            self.publish_map()

    def merge_observation(self, matrix, pose, source_id):
        candidates = []
        for entity in self.entities:
            if entity.get("matrix") != matrix:
                continue
            current = entity.get("pose") or {}
            try:
                distance = math.dist(
                    (float(current["x"]), float(current["y"])),
                    (pose["x"], pose["y"]),
                )
                yaw_error = abs(normalize_angle(float(current["yaw"]) - pose["yaw"]))
            except (KeyError, TypeError, ValueError):
                continue
            if distance <= self.merge_distance_m and yaw_error <= self.merge_yaw_rad:
                candidates.append((distance, entity))
        now = time.time()
        if candidates:
            entity = min(candidates, key=lambda item: item[0])[1]
            current = entity["pose"]
            alpha = 0.20
            current["x"] = (1.0 - alpha) * float(current["x"]) + alpha * pose["x"]
            current["y"] = (1.0 - alpha) * float(current["y"]) + alpha * pose["y"]
            yaw_delta = normalize_angle(pose["yaw"] - float(current["yaw"]))
            current["yaw"] = normalize_angle(float(current["yaw"]) + alpha * yaw_delta)
            entity["observations"] = int(entity.get("observations", 1)) + 1
            entity["last_seen_unix"] = now
            entity["source_entity_id"] = source_id
            return
        self.entities.append({
            "id": f"entity_{self.next_id}",
            "matrix": list(matrix),
            "pose": pose,
            "observations": 1,
            "last_seen_unix": now,
            "source_entity_id": source_id,
        })
        self.next_id += 1

    def payload(self):
        return {
            "frame_id": self.map_frame,
            "state": self.last_state,
            "updated_unix": time.time(),
            "entities": self.entities,
        }

    def save_map(self):
        try:
            self.storage_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.storage_path.with_suffix(self.storage_path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(self.payload(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self.storage_path)
        except OSError as exc:
            self.get_logger().warning(f"entity map save failed: {exc}")

    def publish_map(self):
        self.map_pub.publish(String(data=json.dumps(self.payload(), ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TagEntityMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
