"""Loaded floor-plane calibration, independent of pallet-face PnP calibration."""
import json
import math
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np


def project(homography, points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ np.asarray(homography).T
    if not np.isfinite(homogeneous).all() or (np.abs(homogeneous[:, 2]) < 1e-8).any():
        raise ValueError("바닥 좌표를 계산할 수 없는 점입니다")
    return homogeneous[:, :2] / homogeneous[:, 2:]


def ground_corners(width_cm, depth_cm, far_line_cm):
    # Vehicle-aligned floor coordinates: right+, forward+, origin at fork tip centre.
    return np.array([[-width_cm/2, far_line_cm], [width_cm/2, far_line_cm],
                     [width_cm/2, far_line_cm-depth_cm], [-width_cm/2, far_line_cm-depth_cm]])


def validate_top_line(points, image_size):
    p = np.asarray(points, dtype=float)
    w, h = image_size
    if p.shape != (2, 2) or not np.isfinite(p).all():
        raise ValueError("top line의 왼쪽 끝 → 오른쪽 끝 두 점을 찍으세요")
    if (p < 0).any() or (p[:, 0] >= w).any() or (p[:, 1] >= h).any():
        raise ValueError("top line의 양 끝점이 영상 안에 보여야 합니다")
    if p[1, 0] - p[0, 0] < 20:
        raise ValueError("왼쪽부터 오른쪽 순서로, 충분히 보이는 top line 양 끝을 찍으세요")
    return p


def fit_floor_calibration(samples, width_cm, *, vehicle, source_topic):
    if len(samples) != 2:
        raise ValueError("샘플 1과 2를 각각 저장하세요")
    distances = [float(s['far_line_cm']) for s in samples]
    if not all(math.isfinite(d) and d > 0 for d in distances):
        raise ValueError("각 샘플의 실제 거리를 입력하세요")
    if abs(distances[0] - distances[1]) < 1e-6:
        raise ValueError("서로 다른 두 거리에서 촬영하세요")
    if not math.isfinite(width_cm) or width_cm <= 0:
        raise ValueError("top line의 실제 바깥 폭을 입력하세요")
    samples = sorted(samples, key=lambda s: s['far_line_cm'])
    sizes = [s['image_size'] for s in samples]
    if sizes[0] != sizes[1]:
        raise ValueError("두 영상의 해상도가 다릅니다. 다시 캡처하세요")
    pixels = [validate_top_line(s['top_line_px'], s['image_size']) for s in samples]
    if pixels[0][:, 1].mean() - pixels[1][:, 1].mean() < 3:
        raise ValueError("더 먼 샘플의 top line은 가까운 샘플보다 화면 위에 있어야 합니다. 거리와 점을 확인하세요")
    quad = np.array([pixels[1][0], pixels[1][1], pixels[0][1], pixels[0][0]], dtype=np.float32)
    if not cv2.isContourConvex(quad) or abs(cv2.contourArea(quad)) < 100:
        raise ValueError("두 top line의 배치가 교차하거나 간격이 너무 작습니다")
    ground = [np.array([[-width_cm/2, s['far_line_cm']], [width_cm/2, s['far_line_cm']]]) for s in samples]
    # Two endpoints at each known distance provide four non-collinear plane correspondences.
    # Four points determine H exactly; residual is NOT independent accuracy validation.
    h, _ = cv2.findHomography(np.vstack(pixels), np.vstack(ground), 0)
    if h is None or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
        raise ValueError("바닥 변환이 불안정합니다")
    errors = np.linalg.norm(project(h, np.vstack(pixels))-np.vstack(ground), axis=1)
    return {
        'schema_version': 3, 'kind': 'loaded_floor_plane', 'load_state': 'loaded',
        'method': 'top_line_endpoints_at_two_measured_distances',
        'sample_distances_cm': sorted(distances),
        'vehicle': int(vehicle), 'source_topic': source_topic, 'image_size': sizes[0],
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'coordinate_frame': 'fork_tip_center; right_cm positive right, forward_cm positive forward',
        'distance_reference': 'fork tip to FAR outer warning-border line, measured along floor',
        'width_cm': float(width_cm),
        'image_to_floor_h': h.tolist(), 'calibration_pixel_hull': quad.tolist(),
        'accuracy_validation': 'not independently measured; four points exactly determine homography',
        'fit_rmse_cm': float(np.sqrt(np.mean(errors**2))), 'samples': samples,
        'assumptions': ['loaded camera posture remains fixed', 'flat floor',
                        'same top line endpoints at both distances',
                        'fork centre aligned to line centre and perpendicular to line during capture'],
    }


def floor_point(profile, point, image_size, *, vehicle, source_topic):
    if profile.get('kind') != 'loaded_floor_plane' or profile.get('load_state') != 'loaded':
        raise ValueError("적재 바닥 보정값이 아닙니다")
    if profile['image_size'] != list(image_size) or profile['vehicle'] != vehicle or profile['source_topic'] != source_topic:
        raise ValueError("보정 당시 차량·카메라·해상도와 다릅니다")
    if not np.isfinite(point).all():
        raise ValueError("유효하지 않은 영상 좌표입니다")
    hull = np.asarray(profile.get('calibration_pixel_hull', profile.get('validated_pixel_hull')), dtype=np.float32)
    if cv2.pointPolygonTest(hull, tuple(map(float, point)), False) < 0:
        raise ValueError("두 샘플 사이의 보정 영역 밖입니다")
    right, forward = project(profile['image_to_floor_h'], [point])[0]
    return {'right_cm': float(right), 'forward_cm': float(forward),
            'bearing_left_deg': float(math.degrees(math.atan2(-right, forward)))}


def save_floor_profile(path, profile, config_updates=None):
    """Merge calibration into the latest config, preserving unrelated values.

    ``profile=None`` is the depth-only path: it updates only the explicitly
    allow-listed Y-slot depth keys and leaves the homography presets untouched.
    """
    path = Path(path)
    data = json.loads(path.read_text()) if path.exists() else {}
    if not isinstance(data, dict):
        raise ValueError("pose JSON 최상위가 객체가 아닙니다")
    if profile is not None:
        presets = dict(data.get('floor_calibration_presets', {}))
        presets['loaded'] = profile
        data['floor_calibration_presets'] = presets
    if config_updates:
        for key, value in config_updates.items():
            if key not in {
                'y_slot_depth_camera_pitch_deg',
                'y_slot_depth_camera_to_fork_tip_offset_cm',
                'y_slot_depth_extrinsic_calibration',
            }:
                raise ValueError(f"unsupported floor calibration config key: {key}")
            data[key] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        backup = path.with_name(path.name + '.before_floor_calibration_' + str(time.time_ns()))
        backup.write_bytes(path.read_bytes())
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
    try:
        if path.exists():
            os.fchmod(fd, path.stat().st_mode & 0o777)
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n'); f.flush(); os.fsync(f.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
