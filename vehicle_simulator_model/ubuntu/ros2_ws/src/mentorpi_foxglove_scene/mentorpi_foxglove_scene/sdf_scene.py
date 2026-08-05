"""Convert the supported static subset of SDF into scene primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


class SceneParseError(ValueError):
    """Raised when an SDF scene cannot be resolved safely."""


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    @property
    def yaw(self) -> float:
        x, y, z, w = self.orientation
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class Cube:
    id: str
    pose: Pose
    color: tuple[float, float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class Cylinder:
    id: str
    pose: Pose
    color: tuple[float, float, float, float]
    radius: float
    length: float


@dataclass(frozen=True)
class Sphere:
    id: str
    pose: Pose
    color: tuple[float, float, float, float]
    radius: float


Primitive = Cube | Cylinder | Sphere


@dataclass(frozen=True)
class SceneEntity:
    id: str
    primitives: tuple[Primitive, ...]


IDENTITY_POSE = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _multiply_quaternion(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(point: tuple[float, float, float], orientation: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = orientation
    px, py, pz = point
    twice_cross = (2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px))
    cross_again = (
        y * twice_cross[2] - z * twice_cross[1],
        z * twice_cross[0] - x * twice_cross[2],
        x * twice_cross[1] - y * twice_cross[0],
    )
    return (
        px + w * twice_cross[0] + cross_again[0],
        py + w * twice_cross[1] + cross_again[1],
        pz + w * twice_cross[2] + cross_again[2],
    )


def compose_pose(parent: Pose, child: Pose) -> Pose:
    rotated = _rotate(child.position, parent.orientation)
    return Pose(
        tuple(a + b for a, b in zip(parent.position, rotated)),
        _multiply_quaternion(parent.orientation, child.orientation),
    )


def _parse_pose(element: ET.Element | None) -> Pose:
    if element is None or not element.text or not element.text.strip():
        return IDENTITY_POSE
    values = [float(value) for value in element.text.split()]
    if len(values) != 6:
        raise SceneParseError(f'pose must contain six values, got {element.text!r}')
    return Pose(tuple(values[:3]), _quaternion_from_rpy(*values[3:]))


def _color(visual: ET.Element) -> tuple[float, float, float, float]:
    diffuse = visual.findtext('material/diffuse') or visual.findtext('material/ambient')
    if not diffuse:
        return DEFAULT_COLOR
    values = tuple(float(value) for value in diffuse.split())
    if len(values) != 4:
        raise SceneParseError(f'material color must contain four values, got {diffuse!r}')
    return values


def _visual_primitive(visual: ET.Element, primitive_id: str, pose: Pose) -> Primitive | None:
    geometry = visual.find('geometry')
    if geometry is None:
        return None
    color = _color(visual)
    box = geometry.find('box')
    if box is not None:
        size = tuple(float(value) for value in (box.findtext('size') or '').split())
        if len(size) != 3:
            raise SceneParseError(f'box size must contain three values for {primitive_id}')
        return Cube(primitive_id, pose, color, size)
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        return Cylinder(primitive_id, pose, color, float(cylinder.findtext('radius')), float(cylinder.findtext('length')))
    sphere = geometry.find('sphere')
    if sphere is not None:
        return Sphere(primitive_id, pose, color, float(sphere.findtext('radius')))
    plane = geometry.find('plane')
    if plane is not None:
        size = tuple(float(value) for value in (plane.findtext('size') or '').split())
        if len(size) != 2:
            raise SceneParseError(f'plane size must contain two values for {primitive_id}')
        return Cube(primitive_id, pose, color, (size[0], size[1], 0.01))
    return None


def _model_primitives(model: ET.Element, entity_id: str, parent_pose: Pose) -> tuple[Primitive, ...]:
    model_pose = compose_pose(parent_pose, _parse_pose(model.find('pose')))
    primitives: list[Primitive] = []
    for link in model.findall('link'):
        link_id = link.attrib.get('name')
        if not link_id:
            raise SceneParseError(f'model {entity_id!r} has a link without a name')
        link_pose = compose_pose(model_pose, _parse_pose(link.find('pose')))
        for visual in link.findall('visual'):
            visual_id = visual.attrib.get('name')
            if not visual_id:
                raise SceneParseError(f'link {link_id!r} has a visual without a name')
            primitive = _visual_primitive(
                visual,
                f'{entity_id}/{link_id}/{visual_id}',
                compose_pose(link_pose, _parse_pose(visual.find('pose'))),
            )
            if primitive is not None:
                primitives.append(primitive)
    return tuple(primitives)


def _include_model(include: ET.Element, models_root: Path) -> tuple[str, ET.Element, Pose]:
    uri = (include.findtext('uri') or '').strip()
    prefix = 'model://'
    if not uri.startswith(prefix):
        raise SceneParseError(f'only model:// URIs are allowed, got {uri!r}')
    model_name = uri[len(prefix):]
    if not model_name.startswith('warehouse_') or '/' in model_name or model_name in {'.', '..'}:
        raise SceneParseError(f'warehouse model URI is invalid: {uri!r}')
    path = (models_root / model_name / 'model.sdf').resolve()
    resolved_root = models_root.resolve()
    if resolved_root not in path.parents or not path.is_file():
        raise SceneParseError(f'warehouse model is unavailable: {uri!r}')
    model = ET.parse(path).getroot().find('model')
    if model is None:
        raise SceneParseError(f'warehouse model has no model element: {uri!r}')
    entity_id = (include.findtext('name') or model.attrib.get('name') or model_name).strip()
    return entity_id, model, _parse_pose(include.find('pose'))


def static_scene_from_sdf(world_path: Path, models_root: Path) -> tuple[SceneEntity, ...]:
    """Load inline and allowed warehouse SDF visuals as scene entities."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise SceneParseError(f'world SDF has no world element: {world_path}')
    entities: list[SceneEntity] = []
    for model in world.findall('model'):
        entity_id = model.attrib.get('name')
        if not entity_id:
            raise SceneParseError('world model has no name')
        entities.append(SceneEntity(entity_id, _model_primitives(model, entity_id, IDENTITY_POSE)))
    for include in world.findall('include'):
        entity_id, model, pose = _include_model(include, models_root)
        entities.append(SceneEntity(entity_id, _model_primitives(model, entity_id, pose)))
    return tuple(entities)
