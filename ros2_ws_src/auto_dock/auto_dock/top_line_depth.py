"""Registered RGB top-line pixels -> measured depth -> fork-frame line pose.

No homography or PnP distance scale is used. Extrinsics follow the existing
detector's camera pitch/yaw convention; translations are in centimetres.
"""
import math
import cv2
import numpy as np


def _line_pixels(line_px, image_size):
    line = np.asarray(line_px, dtype=float).reshape(2, 2)
    width, height = map(int, image_size)
    if (width <= 0 or height <= 0 or not np.isfinite(line).all()
            or (line[:, 0] < 2).any() or (line[:, 0] >= width-2).any()
            or (line[:, 1] < 2).any() or (line[:, 1] >= height-2).any()):
        raise ValueError('y_slot_depth_line_out_of_image')
    if np.linalg.norm(line[1]-line[0]) < 20:
        raise ValueError('y_slot_depth_line_too_short')
    fractions = np.linspace(0., 1., 33)
    return line, fractions, line[0]+fractions[:, None]*(line[1]-line[0])


def sample_top_line_depth_image(line_px, data, width, height, step, encoding,
                                is_bigendian=False):
    """Decode only the 33 3x3 neighborhoods needed by a detected line."""
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('y_slot_depth_unsupported_encoding')
    dtype = np.dtype('uint16' if encoding == '16UC1' else 'float32').newbyteorder(
        '>' if is_bigendian else '<')
    width, height, step = int(width), int(height), int(step)
    line, _, pixels = _line_pixels(line_px, (width, height))
    if step < width*dtype.itemsize:
        raise ValueError('y_slot_depth_invalid_stride')
    try:
        raw = memoryview(data)
        if raw.nbytes < step*height:
            raise ValueError('y_slot_depth_truncated_image')
        # This is a zero-copy view. Only the selected 297 scalar values below
        # are converted to native float32 and retained.
        image = np.ndarray((height, width), dtype=dtype, buffer=raw,
                           strides=(step, dtype.itemsize))
        patches = np.stack([
            image[int(round(v))-1:int(round(v))+2,
                  int(round(u))-1:int(round(u))+2].reshape(-1)
            for u, v in pixels
        ]).astype(np.float32)
    except (TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith('y_slot_depth_'):
            raise
        raise ValueError('y_slot_depth_invalid_image') from exc
    if encoding == '16UC1':
        patches *= .001
    return dict(line_px=line.ravel().tolist(), image_size=[width, height],
                depth_patches_m=patches)


def measure_sampled_top_line_depth(sampled, matrix, distortion, *,
                                   pitch_deg, yaw_deg=0.,
                                   forward_offset_cm=14., right_offset_cm=0.):
    """Measure a line pose from sparse neighborhoods extracted at callback time."""
    line = np.asarray(sampled.get('line_px'), dtype=float).reshape(2, 2)
    width, height = sampled.get('image_size', (0, 0))
    line, fractions, pixels = _line_pixels(line, (width, height))
    patches = np.asarray(sampled.get('depth_patches_m'), dtype=np.float32)
    if patches.shape != (33, 9):
        raise ValueError('y_slot_depth_invalid_sparse_samples')
    return _measure_line_samples(line, fractions, pixels, patches, matrix, distortion,
                                 pitch_deg=pitch_deg, yaw_deg=yaw_deg,
                                 forward_offset_cm=forward_offset_cm,
                                 right_offset_cm=right_offset_cm)


def measure_top_line_depth(line_px, depth_m, matrix, distortion, *,
                           pitch_deg, yaw_deg=0., forward_offset_cm=14.,
                           right_offset_cm=0.):
    if not np.isfinite([pitch_deg, yaw_deg, forward_offset_cm, right_offset_cm]).all():
        raise ValueError('y_slot_depth_invalid_extrinsics')
    depth = np.asarray(depth_m)
    k = np.asarray(matrix, dtype=float)
    if (depth.ndim != 2 or k.shape != (3, 3) or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0):
        raise ValueError('y_slot_depth_invalid_geometry')
    h, w = depth.shape
    line, fractions, pixels = _line_pixels(line_px, (w, h))
    patches = np.stack([
        depth[int(round(v))-1:int(round(v))+2,
              int(round(u))-1:int(round(u))+2].reshape(-1)
        for u, v in pixels
    ])
    return _measure_line_samples(line, fractions, pixels, patches, matrix, distortion,
                                 pitch_deg=pitch_deg, yaw_deg=yaw_deg,
                                 forward_offset_cm=forward_offset_cm,
                                 right_offset_cm=right_offset_cm)


def _measure_line_samples(line, fractions, pixels, patches, matrix, distortion, *,
                          pitch_deg, yaw_deg, forward_offset_cm, right_offset_cm):
    k = np.asarray(matrix, dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0
            or not np.isfinite([pitch_deg, yaw_deg, forward_offset_cm,
                                right_offset_cm]).all()):
        raise ValueError('y_slot_depth_invalid_geometry')
    samples, kept = [], []
    for index, values in enumerate(patches):
        values = values[np.isfinite(values) & (values >= .15) & (values <= 3.)]
        if len(values) >= 5 and np.quantile(values, .9)-np.quantile(values, .1) <= .03:
            samples.append(float(np.median(values)))
            kept.append(index)
    if len(kept) < 20:
        raise ValueError('y_slot_depth_insufficient_line_samples')
    rays = cv2.undistortPoints(pixels[kept].reshape(-1, 1, 2), k,
                              np.asarray(distortion, dtype=float)).reshape(-1, 2)
    z = np.asarray(samples)*100.
    x, y = rays[:, 0]*z, rays[:, 1]*z
    pitch, yaw = math.radians(pitch_deg), math.radians(yaw_deg)
    forward = math.sin(pitch)*y+math.cos(pitch)*z
    points = np.column_stack((math.cos(yaw)*x+math.sin(yaw)*forward+right_offset_cm,
                              -math.sin(yaw)*x+math.cos(yaw)*forward-forward_offset_cm))
    # Consensus over measured points rejects isolated foreground/background
    # returns. Require support at both ends so a short inner fragment cannot
    # masquerade as the whole target line.
    best = np.zeros(len(points), dtype=bool)
    for a in range(0, len(points), 2):
        for b in range(a+4, len(points), 2):
            tangent = points[b]-points[a]
            length = np.linalg.norm(tangent)
            if length < 5.:
                continue
            residual = np.abs(np.cross(tangent/length, points-points[a]))
            mask = residual <= .75
            if mask.sum() > best.sum():
                best = mask
    coverage = fractions[np.asarray(kept)[best]]
    if best.sum() < 20 or not len(coverage) or coverage.min() > .15 or coverage.max() < .85:
        raise ValueError('y_slot_depth_line_consensus_failed')
    selected = points[best]
    center = selected.mean(axis=0)
    _, _, vh = np.linalg.svd(selected-center, full_matrices=False)
    tangent = vh[0]
    if tangent[0] < 0:
        tangent = -tangent
    # Use the measured near-end groups for the midpoint, avoiding bias from
    # uneven missing depth in the interior.
    extent = (selected-center)@tangent
    middle = (np.median(extent[coverage <= .15])+np.median(extent[coverage >= .85]))/2
    center += middle*tangent
    residual = np.abs(np.cross(tangent, selected-center))
    if center[1] <= 0 or np.ptp(extent) < 8.:
        raise ValueError('y_slot_depth_target_geometry_invalid')
    heading = (math.degrees(math.atan2(tangent[1], tangent[0]))+90)%180-90
    return dict(top_center_cm=center.tolist(), right_cm=float(center[0]),
                forward_cm=float(center[1]), heading_left_deg=heading,
                bearing_left_deg=math.degrees(math.atan2(-center[0], center[1])),
                measurement_source='registered_top_line_depth',
                depth_valid_samples=len(kept), depth_inlier_samples=int(best.sum()),
                depth_line_residual_cm=float(np.sqrt(np.mean(residual**2))),
                depth_line_points_cm=selected.tolist(), top_line_px=line.ravel().tolist())
