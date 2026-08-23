#!/usr/bin/env python3
"""Group detected pallet faces into lightweight persistent map landmarks."""

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


def pallet_center_from_face(pose, face_to_center_m):
    """Project an observed face along its inward normal to the pallet center."""
    yaw = float(pose.get("yaw", 0.0))
    return {
        "x": float(pose["x"]) + face_to_center_m * math.cos(yaw),
        "y": float(pose["y"]) + face_to_center_m * math.sin(yaw),
    }


def same_face_axis_error(first_yaw, second_yaw):
    """Compare one tagged face while tolerating a flipped surface normal."""
    difference = abs(normalize_angle(float(first_yaw) - float(second_yaw)))
    return min(difference, abs(math.pi - difference))


def matrix_signature(matrix):
    """Identify a face despite 2x2 ordering changes caused by perspective."""
    return tuple(sorted(str(item) for item in matrix))


class TagEntityMapper(Node):
    """Fuse YOLO 2x2 faces into pallets and publish one representative face."""

    def __init__(self):
        super().__init__("tag_entity_mapper")
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("detection_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("storage_path", "/shared/tag_entity_map.json")
        self.declare_parameter("pallet_face_to_center_m", 0.55)
        self.declare_parameter("pallet_merge_distance_m", 0.45)
        self.declare_parameter("pallet_face_group_distance_m", 0.35)
        self.declare_parameter("pallet_duplicate_face_distance_m", 0.12)
        self.declare_parameter("pallet_angle_tolerance_deg", 35.0)
        self.declare_parameter("face_merge_yaw_deg", 40.0)
        self.declare_parameter("min_publish_observations", 3)
        self.declare_parameter("provisional_ttl_sec", 5.0)

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
        self.face_to_center_m = max(
            0.05, float(self.get_parameter("pallet_face_to_center_m").value)
        )
        self.pallet_merge_distance_m = max(
            0.05, float(self.get_parameter("pallet_merge_distance_m").value)
        )
        self.face_group_distance_m = max(
            0.05, float(self.get_parameter("pallet_face_group_distance_m").value)
        )
        self.duplicate_face_distance_m = max(
            0.02,
            float(self.get_parameter("pallet_duplicate_face_distance_m").value),
        )
        self.pallet_angle_tolerance_rad = math.radians(max(
            1.0, float(self.get_parameter("pallet_angle_tolerance_deg").value)
        ))
        self.face_merge_yaw_rad = math.radians(max(
            1.0, float(self.get_parameter("face_merge_yaw_deg").value)
        ))
        self.min_publish_observations = max(
            1, int(self.get_parameter("min_publish_observations").value)
        )
        self.provisional_ttl_sec = max(
            1.0, float(self.get_parameter("provisional_ttl_sec").value)
        )
        self.pallets = []
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
            pallets = payload.get("pallets")
            if payload.get("schema_version") == 7 and isinstance(pallets, list):
                self.pallets = [item for item in pallets if isinstance(item, dict)]
                numeric_ids = [
                    int(str(item.get("id", "0")).rsplit("_", 1)[-1])
                    for item in self.pallets
                    if str(item.get("id", "")).rsplit("_", 1)[-1].isdigit()
                ]
                self.next_id = max(numeric_ids, default=0) + 1
            elif isinstance(payload.get("entities"), list):
                self.get_logger().warning(
                    "ignoring incompatible entity map schema; pallet grouping starts clean"
                )
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
        source_stamp_ns = int(detection.get("source_stamp_ns", 0) or 0)
        observed_unix = time.time()
        frame_pallets = {}
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
            try:
                visibility_score = max(
                    0.0, float(observation.get("visibility_score", 0.0))
                )
            except (TypeError, ValueError):
                visibility_score = 0.0
            frame_group = observation.get("frame_pallet_group")
            pallet = self.merge_observation(
                matrix=matrix,
                pose=map_pose,
                source_id=observation.get("entity_id"),
                angle_source=str(observation.get("angle_source", "pnp")),
                visibility_score=visibility_score,
                source_stamp_ns=source_stamp_ns,
                observed_unix=observed_unix,
                preferred_pallet_id=frame_pallets.get(frame_group),
            )
            if frame_group is not None and pallet is not None:
                frame_pallets[frame_group] = pallet["id"]
            changed = True
        if changed:
            self.pallets = [
                pallet for pallet in self.pallets
                if int(pallet.get("observations", 0)) >= self.min_publish_observations
                or observed_unix - float(pallet.get("last_seen_unix", 0.0))
                <= self.provisional_ttl_sec
            ]
            self.last_state = "mapping"
            self.save_map()
            self.publish_map()

    def merge_observation(
        self, matrix, pose, source_id, angle_source="pnp", visibility_score=0.0,
        source_stamp_ns=0, observed_unix=None, preferred_pallet_id=None,
    ):
        center = pallet_center_from_face(pose, self.face_to_center_m)
        candidates = []
        for pallet in self.pallets:
            current = pallet.get("center") or {}
            try:
                center_distance = math.dist(
                    (float(current["x"]), float(current["y"])),
                    (center["x"], center["y"]),
                )
            except (KeyError, TypeError, ValueError):
                continue
            face_matches = []
            all_face_distances = []
            for face in pallet.get("faces") or []:
                face_pose = face.get("pose") or {}
                try:
                    face_distance = math.dist(
                        (float(face_pose["x"]), float(face_pose["y"])),
                        (pose["x"], pose["y"]),
                    )
                    same_matrix = (
                        matrix_signature(face.get("matrix", []))
                        == matrix_signature(matrix)
                    )
                    angle_error = same_face_axis_error(
                        face_pose["yaw"], pose["yaw"]
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                all_face_distances.append(face_distance)
                depth_pair = (
                    angle_source == "depth" and face.get("angle_source") == "depth"
                )
                mixed_angle_sources = angle_source != face.get("angle_source")
                angle_matches = (
                    angle_error <= self.pallet_angle_tolerance_rad
                    or (same_matrix and mixed_angle_sources)
                )
                pnp_duplicate = (
                    same_matrix
                    and not depth_pair
                    and face_distance <= self.duplicate_face_distance_m
                )
                if same_matrix and (angle_matches or pnp_duplicate):
                    face_matches.append((face_distance, angle_error))
            nearest_face_distance = min(all_face_distances, default=math.inf)
            preferred_spatial_match = (
                pallet.get("id") == preferred_pallet_id
                and (
                    center_distance <= self.pallet_merge_distance_m
                    or nearest_face_distance <= self.face_group_distance_m
                )
            )
            if preferred_spatial_match:
                candidates.append((
                    0,
                    min(center_distance, nearest_face_distance),
                    0.0,
                    pallet,
                ))
                continue
            if not face_matches:
                continue
            face_distance, angle_error = min(face_matches)
            spatial_matches = (
                center_distance <= self.pallet_merge_distance_m
                or face_distance <= self.face_group_distance_m
            )
            if spatial_matches:
                candidates.append((
                    1,
                    min(center_distance, face_distance),
                    angle_error,
                    pallet,
                ))

        now = time.time() if observed_unix is None else float(observed_unix)
        if candidates:
            pallet = min(candidates, key=lambda item: (item[0], item[1], item[2]))[3]
            current = pallet["center"]
            alpha = 0.20
            current["x"] = (1.0 - alpha) * float(current["x"]) + alpha * center["x"]
            current["y"] = (1.0 - alpha) * float(current["y"]) + alpha * center["y"]
        else:
            pallet = {
                "id": f"pallet_{self.next_id}",
                "center": center,
                "faces": [],
                "observations": 0,
                "last_seen_unix": now,
                "latest_stamp_ns": 0,
                "representative_face_id": None,
            }
            self.next_id += 1
            self.pallets.append(pallet)

        faces = pallet["faces"]
        face_candidates = []
        for face in faces:
            current = face.get("pose") or {}
            try:
                yaw_error = same_face_axis_error(current["yaw"], pose["yaw"])
                distance = math.dist(
                    (float(current["x"]), float(current["y"])),
                    (pose["x"], pose["y"]),
                )
            except (KeyError, TypeError, ValueError):
                continue
            pnp_duplicate = (
                angle_source != "depth"
                and distance <= self.duplicate_face_distance_m
                and matrix_signature(face.get("matrix", []))
                == matrix_signature(matrix)
            )
            same_matrix = (
                matrix_signature(face.get("matrix", []))
                == matrix_signature(matrix)
            )
            mixed_angle_sources = angle_source != face.get("angle_source")
            if same_matrix and (
                yaw_error <= self.face_merge_yaw_rad
                or mixed_angle_sources
                or pnp_duplicate
            ):
                face_candidates.append((yaw_error, distance, face))

        if not face_candidates and len(faces) >= 4:
            for face in faces:
                current = face.get("pose") or {}
                try:
                    yaw_error = abs(
                        normalize_angle(float(current["yaw"]) - pose["yaw"])
                    )
                    distance = math.dist(
                        (float(current["x"]), float(current["y"])),
                        (pose["x"], pose["y"]),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                face_candidates.append((yaw_error, distance, face))

        if face_candidates:
            face = min(
                face_candidates,
                key=lambda item: (
                    item[1] if angle_source != "depth" else item[0],
                    item[0] if angle_source != "depth" else item[1],
                ),
            )[2]
            current = face["pose"]
            alpha = 0.20
            current["x"] = (1.0 - alpha) * float(current["x"]) + alpha * pose["x"]
            current["y"] = (1.0 - alpha) * float(current["y"]) + alpha * pose["y"]
            yaw_delta = normalize_angle(pose["yaw"] - float(current["yaw"]))
            current["yaw"] = normalize_angle(float(current["yaw"]) + alpha * yaw_delta)
            if visibility_score >= float(face.get("visibility_score", 0.0)):
                face["matrix"] = list(matrix)
            face["visibility_score"] = visibility_score
            face["observations"] = int(face.get("observations", 1)) + 1
        else:
            face = {
                "id": f"{pallet['id']}_face_{len(faces) + 1}",
                "matrix": list(matrix),
                "pose": dict(pose),
                "observations": 1,
                "visibility_score": visibility_score,
            }
            faces.append(face)

        face["last_seen_unix"] = now
        face["last_seen_stamp_ns"] = source_stamp_ns
        face["source_entity_id"] = source_id
        face["angle_source"] = angle_source
        pallet["observations"] = int(pallet.get("observations", 0)) + 1
        pallet["last_seen_unix"] = now
        latest_stamp = int(pallet.get("latest_stamp_ns", 0) or 0)
        representative = next(
            (item for item in faces if item.get("id") == pallet.get("representative_face_id")),
            None,
        )
        if (
            source_stamp_ns > latest_stamp
            or representative is None
            or (
                source_stamp_ns == latest_stamp
                and visibility_score > float(representative.get("visibility_score", 0.0))
            )
        ):
            pallet["latest_stamp_ns"] = source_stamp_ns
            pallet["representative_face_id"] = face["id"]
        return pallet

    def visible_entities(self):
        entities = []
        for pallet in self.pallets:
            if int(pallet.get("observations", 0)) < self.min_publish_observations:
                continue
            faces = pallet.get("faces") or []
            representative = next(
                (face for face in faces if face.get("id") == pallet.get("representative_face_id")),
                None,
            )
            if representative is None and faces:
                representative = max(
                    faces, key=lambda face: float(face.get("visibility_score", 0.0))
                )
            if representative is None:
                continue
            pose = dict(pallet.get("center") or {})
            pose["yaw"] = float((representative.get("pose") or {}).get("yaw", 0.0))
            entities.append({
                "id": pallet.get("id"),
                "matrix": representative.get("matrix", []),
                "pose": pose,
                "face_count": len(faces),
                "representative_face_id": representative.get("id"),
                "angle_source": representative.get("angle_source", "pnp"),
                "observations": pallet.get("observations", 0),
                "last_seen_unix": pallet.get("last_seen_unix"),
            })
        return entities

    def payload(self):
        return {
            "schema_version": 7,
            "frame_id": self.map_frame,
            "state": self.last_state,
            "updated_unix": time.time(),
            "entities": self.visible_entities(),
            "pallets": self.pallets,
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
