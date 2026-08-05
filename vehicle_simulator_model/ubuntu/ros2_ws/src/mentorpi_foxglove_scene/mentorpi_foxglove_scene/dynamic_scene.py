"""Build simple Foxglove primitives for moving warehouse entities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .sdf_scene import Cube, DEFAULT_COLOR, Pose, SceneEntity, compose_pose


@dataclass(frozen=True)
class DynamicSnapshot:
    entities: tuple[SceneEntity, ...]
    deleted_ids: tuple[str, ...]


ROBOT_COLOR = (0.16, 0.55, 0.86, 1.0)
PALLET_COLOR = (0.55, 0.32, 0.14, 1.0)
PAYLOAD_COLOR = (0.55, 0.25, 0.75, 1.0)


def _relative_pose(x: float, y: float, z: float) -> Pose:
    return Pose((x, y, z), (0.0, 0.0, 0.0, 1.0))


def _cube(entity_id: str, name: str, pose: Pose, color: tuple[float, float, float, float], size: tuple[float, float, float]) -> Cube:
    return Cube(f'{entity_id}/{name}', pose, color, size)


def _robot_entity(entity_id: str, pose: Pose) -> SceneEntity:
    return SceneEntity(entity_id, (
        _cube(entity_id, 'chassis', compose_pose(pose, _relative_pose(0.0, 0.0, 0.06)), ROBOT_COLOR, (0.30, 0.20, 0.12)),
        _cube(entity_id, 'mast', compose_pose(pose, _relative_pose(-0.10, 0.0, 0.20)), DEFAULT_COLOR, (0.05, 0.16, 0.28)),
        _cube(entity_id, 'fork_left', compose_pose(pose, _relative_pose(0.20, 0.055, 0.025)), DEFAULT_COLOR, (0.16, 0.025, 0.02)),
        _cube(entity_id, 'fork_right', compose_pose(pose, _relative_pose(0.20, -0.055, 0.025)), DEFAULT_COLOR, (0.16, 0.025, 0.02)),
    ))


def _pallet_entity(entity_id: str, pose: Pose) -> SceneEntity:
    return SceneEntity(entity_id, (
        _cube(entity_id, 'deck', compose_pose(pose, _relative_pose(0.0, 0.0, 0.027)), PALLET_COLOR, (0.135, 0.135, 0.006)),
        _cube(entity_id, 'support_left', compose_pose(pose, _relative_pose(0.0, -0.063, 0.012)), PALLET_COLOR, (0.135, 0.009, 0.008)),
        _cube(entity_id, 'support_center', compose_pose(pose, _relative_pose(0.0, 0.0, 0.012)), PALLET_COLOR, (0.135, 0.009, 0.008)),
        _cube(entity_id, 'support_right', compose_pose(pose, _relative_pose(0.0, 0.063, 0.012)), PALLET_COLOR, (0.135, 0.009, 0.008)),
    ))


def _payload_entity(entity_id: str, pose: Pose) -> SceneEntity:
    return SceneEntity(entity_id, (
        _cube(entity_id, 'lower', compose_pose(pose, _relative_pose(0.0, 0.0, 0.025)), PAYLOAD_COLOR, (0.12, 0.12, 0.05)),
        _cube(entity_id, 'upper', compose_pose(pose, _relative_pose(0.0, 0.0, 0.075)), PAYLOAD_COLOR, (0.10, 0.10, 0.05)),
    ))


def _simple_name(name: str) -> str:
    return name.rsplit('::', 1)[-1]


class DynamicScene:
    """Maintains the last dynamic entity set to emit Foxglove deletions."""

    def __init__(self) -> None:
        self._previous_ids: set[str] = set()

    def snapshot(self, poses: Mapping[str, Pose]) -> DynamicSnapshot:
        normalized = {_simple_name(name): pose for name, pose in poses.items()}
        entities: list[SceneEntity] = []
        for entity_id in sorted(name for name in normalized if name in {'robot_1', 'robot_2'}):
            entities.append(_robot_entity(entity_id, normalized[entity_id]))
        for entity_id in sorted(name for name in normalized if name.startswith('pallet_') and not name.endswith('_payload')):
            entities.append(_pallet_entity(entity_id, normalized[entity_id]))
        for entity_id in sorted(name for name in normalized if name.startswith('pallet_') and name.endswith('_payload')):
            entities.append(_payload_entity(entity_id, normalized[entity_id]))
        current_ids = {entity.id for entity in entities}
        deleted_ids = tuple(sorted(self._previous_ids - current_ids))
        self._previous_ids = current_ids
        return DynamicSnapshot(tuple(entities), deleted_ids)
