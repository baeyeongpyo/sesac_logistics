"""ROS 2 publisher for Foxglove SceneUpdate warehouse topics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Callable

import yaml

from .dynamic_scene import DynamicScene
from .sdf_scene import Cube, Cylinder, Pose, SceneEntity, Sphere, static_scene_from_sdf


@dataclass(frozen=True)
class RosMessageTypes:
    scene_update: type
    scene_entity: type
    scene_entity_deletion: type
    cube: type
    cylinder: type
    sphere: type
    pose: type
    vector3: type
    color: type


def ros_message_types() -> RosMessageTypes:
    from foxglove_msgs.msg import (
        Color,
        CubePrimitive,
        CylinderPrimitive,
        SceneEntity as FoxgloveSceneEntity,
        SceneEntityDeletion,
        SceneUpdate,
        SpherePrimitive,
    )
    from geometry_msgs.msg import Pose as RosPose
    from geometry_msgs.msg import Vector3
    return RosMessageTypes(
        scene_update=SceneUpdate,
        scene_entity=FoxgloveSceneEntity,
        scene_entity_deletion=SceneEntityDeletion,
        cube=CubePrimitive,
        cylinder=CylinderPrimitive,
        sphere=SpherePrimitive,
        pose=RosPose,
        vector3=Vector3,
        color=Color,
    )


def _pose_message(pose: Pose, messages: RosMessageTypes):
    message = messages.pose()
    message.position.x, message.position.y, message.position.z = pose.position
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = pose.orientation
    return message


def _vector3(values: tuple[float, float, float], messages: RosMessageTypes):
    message = messages.vector3()
    message.x, message.y, message.z = values
    return message


def _color(values: tuple[float, float, float, float], messages: RosMessageTypes):
    message = messages.color()
    message.r, message.g, message.b, message.a = values
    return message


def _primitive_message(primitive: Cube | Cylinder | Sphere, messages: RosMessageTypes):
    if isinstance(primitive, Cube):
        message = messages.cube()
        message.pose = _pose_message(primitive.pose, messages)
        message.size = _vector3(primitive.size, messages)
    elif isinstance(primitive, Cylinder):
        message = messages.cylinder()
        message.pose = _pose_message(primitive.pose, messages)
        message.size = _vector3((primitive.radius * 2, primitive.radius * 2, primitive.length), messages)
        message.bottom_scale = 1.0
        message.top_scale = 1.0
    else:
        message = messages.sphere()
        message.pose = _pose_message(primitive.pose, messages)
        diameter = primitive.radius * 2
        message.size = _vector3((diameter, diameter, diameter), messages)
    message.color = _color(primitive.color, messages)
    return message


def scene_update_from_entities(
    entities: tuple[SceneEntity, ...],
    deleted_ids: tuple[str, ...],
    frame_id: str,
    stamp,
    messages: RosMessageTypes,
):
    """Convert scene entities into a Foxglove message without requiring ROS at import time."""
    update = messages.scene_update()
    update.entities = []
    update.deletions = []
    for source in entities:
        entity = messages.scene_entity()
        entity.id = source.id
        entity.frame_id = frame_id
        entity.timestamp = stamp
        entity.frame_locked = False
        entity.cubes, entity.cylinders, entity.spheres = [], [], []
        for primitive in source.primitives:
            target = _primitive_message(primitive, messages)
            if isinstance(primitive, Cube):
                entity.cubes.append(target)
            elif isinstance(primitive, Cylinder):
                entity.cylinders.append(target)
            else:
                entity.spheres.append(target)
        update.entities.append(entity)
    for entity_id in deleted_ids:
        deletion = messages.scene_entity_deletion()
        deletion.timestamp = stamp
        deletion.type = deletion.MATCHING_ID
        deletion.id = entity_id
        update.deletions.append(deletion)
    return update


def _poses_from_tf(message) -> dict[str, Pose]:
    poses: dict[str, Pose] = {}
    for transform in message.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        poses[transform.child_frame_id] = Pose(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
    return poses


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    messages = ros_message_types()

    class SdfScenePublisher(Node):
        def __init__(self) -> None:
            super().__init__('sdf_scene_publisher')
            self.declare_parameter('world_sdf', '')
            self.declare_parameter('models_root', '')
            self.declare_parameter('frame_id', 'warehouse')
            self.declare_parameter('registry_path', '')
            self.declare_parameter('pose_timeout_seconds', 1.0)
            world_sdf = Path(self.get_parameter('world_sdf').value)
            models_root = Path(self.get_parameter('models_root').value)
            self._frame_id = self.get_parameter('frame_id').value
            if not world_sdf.is_file() or not models_root.is_dir():
                raise RuntimeError('world_sdf and models_root must reference installed warehouse assets')
            static_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._static_publisher = self.create_publisher(messages.scene_update, '/warehouse_scene/static', static_qos)
            self._dynamic_publisher = self.create_publisher(messages.scene_update, '/warehouse_scene/dynamic', 10)
            self._registry_path = Path(self.get_parameter('registry_path').value)
            self._registry_mtime = None
            self._vehicle_subscriptions = {}
            self._pose_timeout_seconds = float(self.get_parameter('pose_timeout_seconds').value)
            self._static_entities = static_scene_from_sdf(world_sdf, models_root)
            self._dynamic_scene = DynamicScene()
            self._poses: dict[str, Pose] = {}
            self._pose_seen_at: dict[str, float] = {}
            self._tf_message_type = TFMessage
            self._reload_vehicle_subscriptions()
            self._publish_static()
            self.create_timer(1.0, self._publish_static)
            self.create_timer(0.1, self._publish_dynamic)

        def _update(self, entities, deleted_ids):
            return scene_update_from_entities(
                entities, deleted_ids, self._frame_id, self.get_clock().now().to_msg(), messages)

        def _on_poses(self, message: TFMessage) -> None:
            poses = _poses_from_tf(message)
            self._poses.update(poses)
            now = time.monotonic()
            self._pose_seen_at.update({name: now for name in poses})

        def _reload_vehicle_subscriptions(self) -> None:
            try:
                mtime = self._registry_path.stat().st_mtime_ns
                if mtime == self._registry_mtime:
                    return
                registry = yaml.safe_load(self._registry_path.read_text())
                vehicle_ids = {
                    vehicle['id'] for vehicle in registry.get('vehicles', [])
                    if vehicle.get('enabled') is True
                }
            except (OSError, yaml.YAMLError, AttributeError, KeyError, TypeError):
                return
            for vehicle_id in tuple(self._vehicle_subscriptions):
                if vehicle_id not in vehicle_ids:
                    self.destroy_subscription(self._vehicle_subscriptions.pop(vehicle_id))
            for vehicle_id in vehicle_ids:
                if vehicle_id not in self._vehicle_subscriptions:
                    self._vehicle_subscriptions[vehicle_id] = self.create_subscription(
                        self._tf_message_type,
                        f'/{vehicle_id}/ground_truth/pose',
                        self._on_poses,
                        10,
                    )
            self._registry_mtime = mtime

        def _publish_static(self) -> None:
            self._static_publisher.publish(self._update(self._static_entities, ()))

        def _publish_dynamic(self) -> None:
            self._reload_vehicle_subscriptions()
            deadline = time.monotonic() - self._pose_timeout_seconds
            self._poses = {
                name: pose for name, pose in self._poses.items()
                if self._pose_seen_at.get(name, 0.0) >= deadline
            }
            self._pose_seen_at = {
                name: seen_at for name, seen_at in self._pose_seen_at.items()
                if seen_at >= deadline
            }
            snapshot = self._dynamic_scene.snapshot(self._poses)
            self._dynamic_publisher.publish(self._update(snapshot.entities, snapshot.deleted_ids))

    rclpy.init(args=args)
    node = SdfScenePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
