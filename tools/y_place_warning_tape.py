"""Image helpers adapted from auto_dock_node.py; ROI-relative span, no ROS imports."""
import math
import cv2
import numpy as np

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def detect_warning_tape(
    frame, minimum_yellow_pixels=600, minimum_center_y_ratio=None,
    filter_config=None,
):
    """Detect a mostly horizontal yellow/black warning-tape band."""
    if frame is None or frame.size == 0:
        return None
    height, width = frame.shape[:2]
    values = filter_config if isinstance(filter_config, dict) else {}
    if minimum_center_y_ratio is None:
        minimum_center_y_ratio = values.get("roi_top_ratio")
    roi_top = 0 if minimum_center_y_ratio is None else int(round(
        clamp(float(minimum_center_y_ratio), 0.0, 0.95) * height
    ))
    roi = frame[roi_top:, :]
    roi_height = roi.shape[0]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(
        hsv,
        np.asarray((
            int(values.get("h_min", 15)),
            int(values.get("s_min", 90)),
            int(values.get("v_min", 70)),
        ), dtype=np.uint8),
        np.asarray((
            int(values.get("h_max", 42)),
            int(values.get("s_max", 255)),
            int(values.get("v_max", 255)),
        ), dtype=np.uint8),
    )
    for operation, key, default in (
        (cv2.MORPH_OPEN, "open_kernel", 3),
        (cv2.MORPH_CLOSE, "close_kernel", 1),
    ):
        kernel_size = int(values.get(key, default))
        if kernel_size > 1:
            if kernel_size % 2 == 0:
                kernel_size += 1
            yellow = cv2.morphologyEx(
                yellow, operation,
                np.ones((kernel_size, kernel_size), dtype=np.uint8),
            )
    black = cv2.inRange(
        hsv, np.asarray((0, 0, 0), dtype=np.uint8),
        np.asarray((179, 255, 75), dtype=np.uint8),
    )
    black = cv2.morphologyEx(
        black, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        yellow, connectivity=8
    )
    candidates = []
    minimum_component_pixels = int(values.get(
        "min_component_pixels", 200 if filter_config is not None else 80
    ))
    for label in range(1, component_count):
        x, y, component_width, component_height, area = stats[label]
        if area < minimum_component_pixels or component_width < 12:
            continue
        side_width = max(8, int(round(component_width * 0.75)))
        y0 = max(0, y)
        y1 = min(roi_height, y + component_height)
        left_x0 = max(0, x - side_width)
        left_x1 = min(width, x + 2)
        right_x0 = max(0, x + component_width - 2)
        right_x1 = min(width, x + component_width + side_width)
        side_areas = []
        for x0, x1 in ((left_x0, left_x1), (right_x0, right_x1)):
            if x1 <= x0 or y1 <= y0:
                continue
            region = black[y0:y1, x0:x1]
            side_areas.append(
                cv2.countNonZero(region) / max(float(region.size), 1.0)
            )
        center_x, center_y = centroids[label]
        candidates.append({
            "label": label,
            "center_x": float(center_x),
            "center_y": float(center_y),
            "height": int(component_height),
            "area": int(area),
            "x0": int(x),
            "x1": int(x + component_width),
            "black_adjacent": bool(
                side_areas and max(side_areas) >= 0.15
            ),
        })
    best_group = []
    best_score = -1.0
    for first_index, first in enumerate(candidates):
        for second in candidates[first_index + 1:]:
            dx = second["center_x"] - first["center_x"]
            dy = second["center_y"] - first["center_y"]
            if abs(dx) < 12.0:
                continue
            angle_deg = math.degrees(math.atan2(dy, dx))
            if angle_deg >= 90.0:
                angle_deg -= 180.0
            if angle_deg < -90.0:
                angle_deg += 180.0
            if abs(angle_deg) > 35.0:
                continue
            norm = math.hypot(dx, dy)
            group = []
            for candidate in candidates:
                distance = abs(
                    dy * (candidate["center_x"] - first["center_x"])
                    - dx * (candidate["center_y"] - first["center_y"])
                ) / norm
                tolerance = max(14.0, min(30.0, candidate["height"] * 0.5))
                if distance <= tolerance:
                    group.append(candidate)
            ordered = sorted(group, key=lambda item: item["x0"])
            widths = [item["x1"] - item["x0"] for item in ordered]
            maximum_gap = max(
                (right["x0"] - left["x1"] for left, right in zip(
                    ordered, ordered[1:]
                )),
                default=0,
            )
            allowed_gap = max(100.0, float(np.median(widths)) * 1.5)
            if maximum_gap > allowed_gap:
                continue
            span = max(item["x1"] for item in group) - min(
                item["x0"] for item in group
            )
            area = sum(item["area"] for item in group)
            center_y = float(np.median([
                item["center_y"] for item in group
            ]))
            score = (
                span * (1.0 + 0.25 * max(0, len(group) - 2))
                + min(area, 5000) * 0.01
                + center_y * 3.0
            )
            if score > best_score:
                best_score = score
                best_group = group
    accepted = np.zeros_like(yellow)
    for candidate in best_group:
        accepted[labels == candidate["label"]] = 255
    accepted_components = len(best_group)
    black_adjacent_components = sum(
        int(candidate["black_adjacent"]) for candidate in best_group
    )
    ys, xs = np.nonzero(accepted)
    if (
        accepted_components < 2
        or black_adjacent_components < max(1, math.ceil(accepted_components / 2))
        or len(xs) < int(minimum_yellow_pixels)
        or int(xs.max()) - int(xs.min()) < int(float(values.get('reference_width_px',width)) * 0.25)
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
        "x_min_px": int(xs.min()),
        "x_max_px": int(xs.max()),
        "center_x_px": float(0.5 * (int(xs.min()) + int(xs.max()))),
        "center_x_ratio": float(
            0.5 * (int(xs.min()) + int(xs.max())) / float(width)
        ),
        "image_width_px": int(width),
        "image_height_px": int(height),
        "yellow_pixels": int(len(xs)),
        "component_count": int(accepted_components),
        "black_adjacent_components": int(black_adjacent_components),
        "band_width_px": round(band_width_px, 1),
    }
