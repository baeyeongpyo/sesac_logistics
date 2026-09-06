import math

import numpy as np
import pytest

from floor_calibration_depth import (
    build_depth_calibration_evidence, fit_depth_extrinsics,
)


def fixture(stamp=123):
    width, height = 80, 60
    image = np.full((height, width), 1000, dtype=np.uint16)
    depth = dict(stamp_ns=stamp, frame_id="camera", width=width, height=height,
                 step=width * 2, encoding="16UC1", is_bigendian=False,
                 data=image.tobytes())
    info = dict(source_stamp_ns=120, frame_id="camera", image_size=[width, height],
                k=[100., 0., 40., 0., 100., 30., 0., 0., 1.], d=[])
    extrinsics = dict(camera_pitch_deg=0., camera_yaw_deg=0.,
                      depth_camera_to_fork_tip_offset_cm=20., centerline_offset_cm=0.)
    return depth, info, extrinsics


def test_persists_only_sparse_exact_stamp_depth_and_pose():
    depth, info, extrinsics = fixture()
    evidence = build_depth_calibration_evidence(
        [20., 30., 60., 30.], depth, info, extrinsics, 80.,
        rgb_stamp_ns=123, rgb_frame_id="camera", rgb_topic="/rgb", depth_topic="/depth",
    )
    assert evidence["exact_source_stamp_match"] is True
    assert evidence["line_definition"] == "selected_square_tl_to_tr"
    assert len(evidence["depth_patches_m"]) == 33
    assert all(len(row) == 9 for row in evidence["depth_patches_m"])
    assert "data" not in evidence
    assert evidence["camera_info"]["k"] == info["k"]
    assert evidence["extrinsics"] == extrinsics
    assert evidence["fork_frame_line_pose"]["forward_cm"] == pytest.approx(80.)
    assert evidence["forward_error_cm"] == pytest.approx(0.)
    assert math.isfinite(evidence["heading_error_deg"])


def test_rejects_nonmatching_depth_source_stamp():
    depth, info, extrinsics = fixture(stamp=124)
    with pytest.raises(ValueError, match="depth_stamp_mismatch"):
        build_depth_calibration_evidence(
            [20., 30., 60., 30.], depth, info, extrinsics, 80.,
            rgb_stamp_ns=123, rgb_frame_id="camera", rgb_topic="/rgb", depth_topic="/depth",
        )


def test_rejects_unregistered_frame_or_size():
    depth, info, extrinsics = fixture()
    info["frame_id"] = "other"
    with pytest.raises(ValueError, match="registration_mismatch"):
        build_depth_calibration_evidence(
            [20., 30., 60., 30.], depth, info, extrinsics, 80.,
            rgb_stamp_ns=123, rgb_frame_id="camera", rgb_topic="/rgb", depth_topic="/depth",
        )


def synthetic_sample(sample_id, distance_cm, row_px, *, pitch_deg=-7., offset_cm=30.):
    width, height = 80, 60
    fy, cy = 100., 30.
    ray_y = (row_px-cy)/fy
    pitch = math.radians(pitch_deg)
    z_cm = (distance_cm+offset_cm)/(math.cos(pitch)+math.sin(pitch)*ray_y)
    stamp = 1000+sample_id
    return {
        "sample_id": sample_id,
        "far_line_cm": distance_cm,
        "top_line_px": [[20., row_px], [60., row_px]],
        "registered_depth_evidence": {
            "exact_source_stamp_match": True,
            "line_definition": "selected_square_tl_to_tr",
            "rgb_source_stamp_ns": stamp,
            "depth_source_stamp_ns": stamp,
            "depth_patches_m": np.full((33, 9), z_cm/100.).tolist(),
            "camera_info": {
                "frame_id": "camera", "image_size": [width, height],
                "k": [100., 0., 40., 0., fy, cy, 0., 0., 1.], "d": [],
            },
        },
    }


def test_two_distance_sparse_depth_recovers_known_pitch_and_offset():
    fit = fit_depth_extrinsics([
        synthetic_sample(1, 50., 40.),
        synthetic_sample(2, 80., 25.),
    ])
    values = fit["fitted_extrinsics"]
    assert values["camera_pitch_deg"] == pytest.approx(-7., abs=.01)
    assert values["depth_camera_to_fork_tip_offset_cm"] == pytest.approx(30., abs=.02)
    assert fit["residual_rmse_cm"] < .01
    assert fit["total_sample_count"] == 66
    assert "camera_yaw_deg" in fit["not_fitted_extrinsics"]


def test_depth_extrinsic_fit_rejects_degenerate_distances():
    with pytest.raises(ValueError, match="distances_degenerate"):
        fit_depth_extrinsics([
            synthetic_sample(1, 50., 40.),
            synthetic_sample(2, 50., 25.),
        ])


def test_depth_extrinsic_fit_rejects_identical_depth_geometry():
    first = synthetic_sample(1, 50., 40.)
    second = synthetic_sample(2, 80., 40.)
    second["registered_depth_evidence"]["depth_patches_m"] = (
        first["registered_depth_evidence"]["depth_patches_m"]
    )
    with pytest.raises(ValueError, match="pitch_at_search_boundary|extrinsics_degenerate"):
        fit_depth_extrinsics([first, second])
