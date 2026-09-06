"""Sparse registered-depth evidence for manual floor calibration samples."""

import math
import time

import cv2
import numpy as np

from auto_dock.top_line_depth import (
    measure_sampled_top_line_depth, sample_top_line_depth_image,
)


def _json_depth_patches(patches):
    """JSON-safe sparse depth values; invalid returns remain explicit nulls."""
    values = np.asarray(patches, dtype=float)
    return [[None if not np.isfinite(value) else float(value) for value in row]
            for row in values]


def build_depth_calibration_evidence(
    line_px, depth, camera_info, extrinsics, entered_distance_cm,
    *, rgb_stamp_ns, rgb_frame_id, rgb_topic, depth_topic,
):
    """Create compact, reproducible depth evidence for one frozen RGB sample."""
    if int(depth["stamp_ns"]) != int(rgb_stamp_ns):
        raise ValueError("floor_calibration_depth_stamp_mismatch")
    size = [int(depth["width"]), int(depth["height"])]
    if (str(depth["frame_id"]) != str(rgb_frame_id)
            or camera_info.get("frame_id") != str(rgb_frame_id)
            or list(camera_info.get("image_size", ())) != size):
        raise ValueError("floor_calibration_depth_registration_mismatch")
    sampled = sample_top_line_depth_image(
        line_px, depth["data"], depth["width"], depth["height"],
        depth["step"], depth["encoding"], depth.get("is_bigendian", False),
    )
    pose = measure_sampled_top_line_depth(
        sampled, np.asarray(camera_info["k"], dtype=float).reshape(3, 3),
        camera_info.get("d", []),
        pitch_deg=float(extrinsics["camera_pitch_deg"]),
        yaw_deg=float(extrinsics.get("camera_yaw_deg", 0.0)),
        forward_offset_cm=float(extrinsics["depth_camera_to_fork_tip_offset_cm"]),
        right_offset_cm=float(extrinsics.get("centerline_offset_cm", 0.0)),
    )
    valid = np.isfinite(sampled["depth_patches_m"])
    valid &= sampled["depth_patches_m"] >= .15
    valid &= sampled["depth_patches_m"] <= 3.0
    measured = float(pose["forward_cm"])
    expected = float(entered_distance_cm)
    return {
        "schema_version": 1,
        "line_definition": "selected_square_tl_to_tr",
        "rgb_source_stamp_ns": int(rgb_stamp_ns),
        "depth_source_stamp_ns": int(depth["stamp_ns"]),
        "exact_source_stamp_match": True,
        "rgb_topic": str(rgb_topic),
        "registered_depth_topic": str(depth_topic),
        "frame_id": str(rgb_frame_id),
        "image_size": size,
        "encoding": str(depth["encoding"]),
        "sample_count": 33,
        "neighborhood_size": [3, 3],
        "depth_patches_m": _json_depth_patches(sampled["depth_patches_m"]),
        "valid_value_mask": valid.astype(np.uint8).tolist(),
        "camera_info": {
            "source_stamp_ns": int(camera_info.get("source_stamp_ns", 0)),
            "frame_id": str(camera_info["frame_id"]),
            "image_size": list(camera_info["image_size"]),
            "k": [float(value) for value in camera_info["k"]],
            "d": [float(value) for value in camera_info.get("d", [])],
        },
        "extrinsics": {key: float(extrinsics.get(key, 0.0)) for key in (
            "camera_pitch_deg", "camera_yaw_deg",
            "depth_camera_to_fork_tip_offset_cm", "centerline_offset_cm",
        )},
        "fork_frame_line_pose": pose,
        "entered_forward_distance_cm": expected,
        "forward_error_cm": measured - expected,
        "absolute_forward_error_cm": abs(measured - expected),
        "centerline_error_cm": float(pose["right_cm"]),
        "heading_error_deg": float(pose["heading_left_deg"]),
    }


def _sample_camera_points(sample):
    evidence = sample.get("registered_depth_evidence", {})
    if not evidence.get("exact_source_stamp_match"):
        raise ValueError("floor_calibration_depth_not_exact_stamp")
    if int(evidence.get("rgb_source_stamp_ns", -1)) != int(
            evidence.get("depth_source_stamp_ns", -2)):
        raise ValueError("floor_calibration_depth_stamp_mismatch")
    if evidence.get("line_definition") != "selected_square_tl_to_tr":
        raise ValueError("floor_calibration_depth_wrong_line")
    patches = np.asarray([
        [np.nan if value is None else value for value in row]
        for row in evidence.get("depth_patches_m", [])
    ], dtype=float)
    if patches.shape != (33, 9):
        raise ValueError("floor_calibration_depth_invalid_sparse_samples")
    depths, kept = [], []
    for index, values in enumerate(patches):
        values = values[np.isfinite(values) & (values >= .15) & (values <= 3.)]
        if len(values) >= 5 and np.quantile(values, .9)-np.quantile(values, .1) <= .03:
            depths.append(float(np.median(values)))
            kept.append(index)
    if len(kept) < 20:
        raise ValueError("floor_calibration_depth_insufficient_samples")
    line = np.asarray(sample["top_line_px"], dtype=float).reshape(2, 2)
    pixels = line[0] + np.linspace(0., 1., 33)[:, None] * (line[1]-line[0])
    info = evidence.get("camera_info", {})
    matrix = np.asarray(info.get("k"), dtype=float).reshape(3, 3)
    rays = cv2.undistortPoints(
        pixels[np.asarray(kept)].reshape(-1, 1, 2), matrix,
        np.asarray(info.get("d", []), dtype=float),
    ).reshape(-1, 2)
    z_cm = np.asarray(depths) * 100.
    return {
        "sample_id": sample.get("sample_id"),
        "entered_distance_cm": float(sample["far_line_cm"]),
        "rgb_source_stamp_ns": int(evidence["rgb_source_stamp_ns"]),
        "depth_source_stamp_ns": int(evidence["depth_source_stamp_ns"]),
        "line_px": line.ravel().tolist(),
        "sample_count": len(kept),
        "camera_info": {
            "frame_id": info.get("frame_id"),
            "image_size": list(info.get("image_size", ())),
            "k": np.asarray(info.get("k"), dtype=float).ravel().tolist(),
            "d": np.asarray(info.get("d", []), dtype=float).ravel().tolist(),
        },
        "active_extrinsics": dict(evidence.get("extrinsics", {})),
        "camera_y_cm": rays[:, 1] * z_cm,
        "camera_z_cm": z_cm,
    }


def fit_depth_extrinsics(samples, *, pitch_range_deg=(-30., 30.)):
    """Fit pitch and forward translation from two known fork-to-line distances."""
    if len(samples) != 2:
        raise ValueError("floor_calibration_depth_requires_two_samples")
    rows = [_sample_camera_points(sample) for sample in samples]
    if any(row["camera_info"] != rows[0]["camera_info"] for row in rows[1:]):
        raise ValueError("floor_calibration_depth_camera_info_changed")
    distances = np.asarray([row["entered_distance_cm"] for row in rows])
    if (not np.isfinite(distances).all() or (distances <= 0).any()
            or abs(distances[0]-distances[1]) < 5.):
        raise ValueError("floor_calibration_depth_distances_degenerate")
    y = np.concatenate([row["camera_y_cm"] for row in rows])
    z = np.concatenate([row["camera_z_cm"] for row in rows])
    target = np.concatenate([
        np.full(row["sample_count"], row["entered_distance_cm"])
        for row in rows
    ])

    def evaluate(pitches_deg):
        radians = np.radians(np.asarray(pitches_deg, dtype=float))[:, None]
        projected = np.sin(radians)*y + np.cos(radians)*z
        offsets = np.median(projected-target, axis=1)
        residual = projected-offsets[:, None]-target
        # Median absolute residual makes the fit insensitive to a few mixed pixels.
        return offsets, residual, np.median(np.abs(residual), axis=1)

    lower, upper = map(float, pitch_range_deg)
    coarse = np.linspace(lower, upper, 6001)
    _, _, score = evaluate(coarse)
    coarse_best = float(coarse[int(np.argmin(score))])
    fine = np.linspace(max(lower, coarse_best-.02), min(upper, coarse_best+.02), 401)
    offsets, residuals, score = evaluate(fine)
    best = int(np.argmin(score))
    pitch_deg = float(fine[best])
    offset_cm = float(offsets[best])
    residual = residuals[best]
    if pitch_deg <= lower+.01 or pitch_deg >= upper-.01:
        raise ValueError("floor_calibration_depth_pitch_at_search_boundary")
    pitch = math.radians(pitch_deg)
    derivative = np.cos(pitch)*y-np.sin(pitch)*z
    design = np.column_stack((derivative, -np.ones_like(derivative)))
    singular = np.linalg.svd(design, compute_uv=False)
    if singular[-1] <= 1e-6 or singular[0]/singular[-1] > 200.:
        raise ValueError("floor_calibration_depth_extrinsics_degenerate")
    inliers = np.abs(residual-np.median(residual)) <= max(
        .75, 4.*float(np.median(np.abs(residual-np.median(residual))))
    )
    if inliers.sum() < 40:
        raise ValueError("floor_calibration_depth_fit_insufficient_inliers")
    # Refit once on the robust consensus.
    def inlier_score(pitch_value):
        angle = math.radians(float(pitch_value))
        projected = math.sin(angle)*y[inliers] + math.cos(angle)*z[inliers]
        offset = float(np.median(projected-target[inliers]))
        return float(np.mean((projected-offset-target[inliers])**2)), offset
    candidates = np.linspace(pitch_deg-.02, pitch_deg+.02, 401)
    scored = [inlier_score(value) for value in candidates]
    best = min(range(len(scored)), key=lambda index: scored[index][0])
    pitch_deg = float(candidates[best])
    offset_cm = float(scored[best][1])
    if not 0. <= offset_cm <= 60.:
        raise ValueError("floor_calibration_depth_offset_out_of_range")
    angle = math.radians(pitch_deg)
    fitted = math.sin(angle)*y + math.cos(angle)*z-offset_cm
    residual = fitted-target
    fit_rmse = float(np.sqrt(np.mean(residual[inliers]**2)))
    if fit_rmse > 2.5:
        raise ValueError("floor_calibration_depth_fit_residual_too_large")
    per_sample = []
    cursor = 0
    for row in rows:
        count = row["sample_count"]
        values = fitted[cursor:cursor+count]
        errors = residual[cursor:cursor+count]
        cursor += count
        per_sample.append({
            key: row[key] for key in (
                "sample_id", "entered_distance_cm", "rgb_source_stamp_ns",
                "depth_source_stamp_ns", "line_px", "sample_count",
            )
        })
        per_sample[-1].update(
            fitted_forward_cm=float(np.median(values)),
            forward_error_cm=float(np.median(errors)),
            residual_rmse_cm=float(np.sqrt(np.mean(errors**2))),
        )
    return {
        "schema_version": 1,
        "method": "two_distance_registered_depth_sparse_line_fit",
        "created_at": time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        "fitted_extrinsics": {
            "camera_pitch_deg": pitch_deg,
            "depth_camera_to_fork_tip_offset_cm": offset_cm,
        },
        "not_fitted_extrinsics": {
            "camera_yaw_deg": "not robustly identifiable from two operator-aligned lines",
            "centerline_offset_cm": "not robustly identifiable from two operator-aligned lines",
        },
        "pitch_search_range_deg": [lower, upper],
        "residual_rmse_cm": fit_rmse,
        "residual_median_abs_cm": float(np.median(np.abs(residual[inliers]))),
        "residual_max_abs_cm": float(np.max(np.abs(residual[inliers]))),
        "inlier_count": int(inliers.sum()),
        "total_sample_count": int(len(residual)),
        "samples": per_sample,
        "provenance": {
            "depth_storage": "33x3x3 sparse neighborhoods per selected TL-TR; no full depth frame",
            "distance_reference": "operator-entered fork tip to selected square TL-TR",
            "assumptions": ["vehicle centered", "vehicle perpendicular", "camera mounting fixed"],
            "camera_info": rows[0]["camera_info"],
            "active_extrinsics_at_capture": [row["active_extrinsics"] for row in rows],
        },
    }
