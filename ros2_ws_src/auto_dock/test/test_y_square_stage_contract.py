"""Regression contracts for Y-square pose and the 40 cm insertion gate.

These tests intentionally describe the required behavior.  They should fail
until the production path stops replacing the detected square edge with an
independent Hough line and splits staging from insertion.
"""
import inspect
from types import SimpleNamespace

import numpy as np
import pytest

import auto_dock.auto_dock_node as module
from auto_dock.auto_dock_node import AutoDockNode, YTopLineTracker


def square_frame():
    return np.full((480, 640, 3), 180, dtype=np.uint8)


def detected_square(sequence=1):
    """A square whose outer top edge differs from its inner X endpoints."""
    return {
        "tracking_sequence": sequence,
        "image_width_px": 640,
        "image_height_px": 480,
        "center_x_px": 300.0,
        "center_x_ratio": 300.0 / 640.0,
        "center_y_ratio": 275.0 / 480.0,
        "x_min_px": 210.0,
        "x_max_px": 390.0,
        "x_lines": [
            [210.0, 230.0, 390.0, 330.0],
            [210.0, 330.0, 390.0, 230.0],
        ],
        "square_corners_px": [
            [180.0, 190.0],  # TL
            [420.0, 200.0],  # TR
            [430.0, 360.0],  # BR
            [170.0, 350.0],  # BL
        ],
    }


def test_detected_square_top_edge_is_the_single_pose_line():
    observation = YTopLineTracker().update(square_frame(), detected_square())

    expected = [180.0, 190.0, 420.0, 200.0]
    assert observation["top_line_px"] == expected
    assert observation["square_top_line_px"] == expected
    assert observation["line_definition"] == "detected_square_tl_to_tr"


def test_slot_image_does_not_run_an_independent_top_line_detector():
    source = inspect.getsource(AutoDockNode.on_slot_image)

    assert "detect_y_top_line(" not in source


def depth_pose_fake(rgb_stamp_ns=1_000_000_000, depth_stamp_ns=1_000_000_000):
    square_top = [180.0, 190.0, 420.0, 210.0]
    fake = SimpleNamespace(
        load_state="LOADED",
        odom_yaw=0.0,
        y_slot_odom_source_received_at=100.0,
        latest_tape_guidance_at=100.0,
        latest_tape_guidance={
            "top_line_px": [210.0, 230.0, 390.0, 230.0],
            "square_top_line_px": square_top,
            "image_width_px": 640,
            "image_height_px": 480,
            "rgb_stamp_ns": rgb_stamp_ns,
            "rgb_frame_id": "registered_rgb",
            "rgb_source_at": 100.0,
        },
        slot_rgb_observations=[],
        slot_depth_frames=[{
            "stamp_ns": depth_stamp_ns,
            "rgb_stamp_ns": rgb_stamp_ns,
            "frame_id": "registered_rgb",
            "source_at": 100.0,
            "depth_samples": {
                "image_size": [640, 480],
                "line_px": square_top,
                "depth_patches_m": np.full((33, 9), 0.70),
            },
        }],
        slot_camera_matrix=np.asarray([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]),
        slot_distortion=np.zeros(5),
        slot_camera_frame="registered_rgb",
        slot_camera_size=(640, 480),
        y_slot_cycle_phase="verify",
        y_slot_measure_after=99.0,
        config={
            "camera_pitch_deg": 19.0,
            "camera_yaw_deg": 0.0,
            "depth_camera_to_fork_tip_offset_cm": 14.0,
            "y_slot_depth_camera_pitch_deg": -7.0,
            "y_slot_depth_camera_to_fork_tip_offset_cm": 30.0,
            "centerline_offset_cm": 1.0,
        },
    )
    return fake, square_top


def test_metric_pose_uses_square_identity_but_depth_supplies_yaw_and_distance(
    monkeypatch,
):
    monkeypatch.setattr(module.time, "monotonic", lambda: 100.0)
    fake, square_top = depth_pose_fake()
    measured = []

    def depth_fit(samples, *_args, **kwargs):
        measured.append(list(samples["line_px"]))
        assert kwargs["pitch_deg"] == -7.0
        assert kwargs["forward_offset_cm"] == 30.0
        # Deliberately disagree with the RGB slope.  This is the authoritative
        # registered-depth 3D result, not a homography/image-angle fallback.
        return {
            "top_center_cm": [1.25, 40.4],
            "right_cm": 1.25,
            "forward_cm": 40.4,
            "heading_left_deg": -3.25,
            "bearing_left_deg": -1.77,
            "measurement_source": "registered_top_line_depth",
        }

    monkeypatch.setattr(module, "measure_sampled_top_line_depth", depth_fit)
    pose = AutoDockNode.calibrated_y_slot_pose(fake)

    assert measured == [square_top]
    assert pose["top_center_cm"] == [1.25, 40.4]
    assert pose["heading_left_deg"] == -3.25
    assert pose["measurement_source"] == "registered_top_line_depth"


def test_metric_pose_rejects_nonmatching_rgb_depth_source_stamp(monkeypatch):
    monkeypatch.setattr(module.time, "monotonic", lambda: 100.0)
    fake, _square_top = depth_pose_fake(depth_stamp_ns=1_001_000_000)

    with pytest.raises(ValueError, match="y_slot_depth_unsynchronized"):
        AutoDockNode.calibrated_y_slot_pose(fake)


def stage_executor_fake():
    fake = SimpleNamespace(
        config={},
        state="y_slot_centering",
        y_slot_cycle_phase="feedback_execute",
        y_slot_cycle_segments=[],
        y_slot_frozen_insertion_enabled=True,
        y_slot_frozen_insertion_distance_cm=35.0,
        completed_insertion_distance_m=None,
        drives=[],
        statuses=[],
    )
    fake.number = lambda _key, default, _low=None, _high=None: default
    fake.boolean = lambda _key, default=False: default
    fake.stop_drive = lambda *_args: fake.drives.append((0.0, 0.0, 0.0))
    fake.publish_drive = lambda *drive: fake.drives.append(drive)
    fake.publish_status = lambda *args, **kwargs: fake.statuses.append((args, kwargs))
    return fake


def test_staging_completion_starts_blind_straight_insertion(monkeypatch):
    monkeypatch.setattr(module.time, "monotonic", lambda: 100.0)
    fake = stage_executor_fake()
    fake.odom_position = (1.0, 2.0)
    fake.odom_yaw = 0.25

    AutoDockNode.tick_y_slot_homography_once(fake)

    assert fake.state == "y_slot_inserting"
    assert fake.y_slot_insert_start_position == (1.0, 2.0)
    assert fake.y_slot_insert_start_yaw == pytest.approx(0.25)
    assert fake.completed_insertion_distance_m is None
    assert not any(drive[0] > 0.0 for drive in fake.drives)


def test_verify_rejects_observations_captured_before_staging_stop(monkeypatch):
    monkeypatch.setattr(module.time, "monotonic", lambda: 102.0)
    fake = stage_executor_fake()
    fake.y_slot_cycle_phase = "verify"
    fake.y_slot_measure_after = 101.0
    fake.latest_tape_guidance_at = 100.9
    fake.calibrated_y_slot_pose = lambda: pytest.fail(
        "pre-stop square/depth data must not be reused for 40 cm verification"
    )

    AutoDockNode.tick_y_slot_homography_once(fake)

    assert fake.y_slot_cycle_phase == "verify"
    assert fake.completed_insertion_distance_m is None
    assert not any(drive[0] > 0.0 for drive in fake.drives)


def test_fresh_but_misaligned_40cm_depth_pose_cannot_start_insertion(monkeypatch):
    monkeypatch.setattr(module.time, "monotonic", lambda: 102.0)
    fake = stage_executor_fake()
    fake.config.update({
        "y_slot_pose_source": "depth",
        "y_slot_start_consensus_frames": 3,
        "y_slot_staging_distance_cm": 40.0,
        "y_slot_stage_distance_tolerance_cm": 1.0,
        "y_slot_stage_lateral_tolerance_cm": 1.5,
        "y_slot_stage_yaw_tolerance_deg": 2.0,
    })
    fake.y_slot_cycle_phase = "verify"
    fake.y_slot_measure_after = 101.0
    fake.latest_tape_guidance_at = 102.0
    fake.y_slot_requested_insertion_distance_cm = 35.0
    fake.y_slot_stage_only = False
    measured = {
        "rgb_stamp_ns": 2_000_000_000,
        "top_center_cm": [0.0, 40.0],
        "right_cm": 0.0,
        "forward_cm": 40.0,
        "heading_left_deg": 8.0,
        "measurement_source": "registered_top_line_depth",
    }
    fake.calibrated_y_slot_pose = lambda: dict(measured)
    fake.y_slot_observation_evidence = lambda _pose: {}
    fake.loaded_response_coefficients = lambda: pytest.fail(
        "a failed fresh 40 cm yaw gate must not create an insertion plan"
    )
    monkeypatch.setattr(
        module, "y_slot_metric_pose_consensus", lambda _poses, _minimum: measured
    )

    AutoDockNode.tick_y_slot_homography_once(fake)

    assert fake.y_slot_cycle_phase in {"verify", "reverse"}
    assert fake.completed_insertion_distance_m is None
    assert not any(drive[0] > 0.0 for drive in fake.drives)
