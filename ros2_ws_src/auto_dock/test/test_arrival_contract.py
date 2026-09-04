import json
import math

import cv2
import numpy as np
import pytest

from auto_dock.auto_dock_node import (
    AutoDockNode,
    DockInventoryTracker,
    SlotSelector,
    SlotGridVision,
    ZoneOccupancy,
    detect_dock_end_markers,
    detect_warning_tape,
    normalize_slot_id,
    pallet_product_type,
    parse_arrival,
    public_fsm_state,
)
from std_msgs.msg import Empty, String


def test_warning_tape_detector_fits_repeating_yellow_band():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    for start_x in range(20, 600, 100):
        end_x = min(start_x + 58, 639)
        top_left = int(325 + 0.10 * start_x)
        top_right = int(325 + 0.10 * end_x)
        polygon = np.asarray([
            (start_x, top_left), (end_x, top_right),
            (end_x, top_right + 42), (start_x, top_left + 42),
        ], dtype=np.int32)
        cv2.fillConvexPoly(frame, polygon, (0, 220, 220))
        black_end = min(end_x + 30, 639)
        black_top_right = int(325 + 0.10 * black_end)
        cv2.fillConvexPoly(frame, np.asarray([
            (end_x, top_right), (black_end, black_top_right),
            (black_end, black_top_right + 42), (end_x, top_right + 42),
        ], dtype=np.int32), (5, 5, 5))

    tape = detect_warning_tape(frame)

    assert tape is not None
    assert 4.0 < tape["angle_deg"] < 8.0
    assert tape["component_count"] >= 5


def test_warning_tape_detector_rejects_blank_floor():
    frame = np.full((480, 640, 3), 130, dtype=np.uint8)

    assert detect_warning_tape(frame) is None


def test_warning_tape_first_detection_can_use_the_full_frame():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    for start_x in range(20, 600, 100):
        cv2.rectangle(
            frame, (start_x, 90), (min(start_x + 58, 639), 120),
            (0, 220, 220), -1,
        )
        cv2.rectangle(
            frame, (min(start_x + 58, 639), 90),
            (min(start_x + 88, 639), 120), (5, 5, 5), -1,
        )

    assert detect_warning_tape(frame) is not None


def test_warning_tape_groups_one_band_instead_of_fitting_yellow_clutter():
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    for x, y, width, height in (
        (70, 35, 45, 42),
        (145, 120, 55, 48),
        (245, 195, 75, 65),
        (380, 260, 60, 55),
    ):
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 220, 220), -1)
        cv2.rectangle(
            frame, (x + width, y), (x + width + 25, y + height), (5, 5, 5), -1,
        )
    for start_x in range(10, 610, 150):
        end_x = min(start_x + 82, 639)
        top_left = int(438 - 0.13 * start_x)
        top_right = int(438 - 0.13 * end_x)
        cv2.fillConvexPoly(frame, np.asarray([
            (start_x, top_left), (end_x, top_right),
            (end_x, top_right + 35), (start_x, top_left + 35),
        ], dtype=np.int32), (0, 220, 220))
        black_end = min(end_x + 45, 639)
        black_top_right = int(438 - 0.13 * black_end)
        cv2.fillConvexPoly(frame, np.asarray([
            (end_x, top_right), (black_end, black_top_right),
            (black_end, black_top_right + 35), (end_x, top_right + 35),
        ], dtype=np.int32), (5, 5, 5))

    tape = detect_warning_tape(frame)

    assert tape is not None
    assert -10.0 < tape["angle_deg"] < -5.0
    assert tape["center_y_ratio"] > 0.75


def test_warning_tape_rejects_a_distant_component_from_the_selected_band():
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    cv2.rectangle(frame, (0, 250), (15, 265), (0, 220, 220), -1)
    cv2.rectangle(frame, (16, 250), (30, 265), (5, 5, 5), -1)
    for start_x in (320, 488):
        cv2.rectangle(frame, (start_x, 465), (start_x + 82, 479),
                      (0, 220, 220), -1)
        cv2.rectangle(frame, (start_x + 82, 465), (start_x + 125, 479),
                      (5, 5, 5), -1)

    tape = detect_warning_tape(frame)

    assert tape is not None
    assert abs(tape["angle_deg"]) < 2.0
    assert tape["center_y_ratio"] > 0.95


def test_warning_tape_rejects_repeating_yellow_without_adjacent_black():
    frame = np.full((480, 640, 3), 90, dtype=np.uint8)
    for start_x in range(20, 600, 100):
        cv2.rectangle(
            frame, (start_x, 300), (min(start_x + 58, 639), 335),
            (0, 220, 220), -1,
        )

    assert detect_warning_tape(frame) is None


def test_warning_tape_pose_jump_is_rejected_while_track_is_fresh():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {}
    fake.number = lambda key, default, *_args: fake.config.get(key, default)
    fake.latest_tape_guidance = {"angle_deg": 0.0, "center_y_ratio": 0.99}
    fake.latest_tape_guidance_at = 10.0
    fake.pending_tape_guidance = None
    fake.pending_tape_guidance_count = 0
    diagonal = {"angle_deg": 22.0, "center_y_ratio": 0.81}

    assert AutoDockNode.update_warning_tape_guidance(fake, diagonal, 10.1) is False
    assert AutoDockNode.update_warning_tape_guidance(fake, diagonal, 10.2) is False
    assert fake.latest_tape_guidance["angle_deg"] == 0.0
    assert AutoDockNode.update_warning_tape_guidance(fake, diagonal, 10.3) is False
    assert fake.latest_tape_guidance["angle_deg"] == 0.0
    gradual = {"angle_deg": 4.0, "center_y_ratio": 0.94}
    assert AutoDockNode.update_warning_tape_guidance(fake, gradual, 10.4) is True
    assert fake.latest_tape_guidance["angle_deg"] == 4.0


def test_warning_tape_tracking_roi_ignores_everything_above_previous_band():
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    for start_x in range(20, 600, 100):
        cv2.rectangle(frame, (start_x, 100), (start_x + 58, 130),
                      (0, 220, 220), -1)
        cv2.rectangle(frame, (start_x + 58, 100), (start_x + 88, 130),
                      (5, 5, 5), -1)

    assert detect_warning_tape(frame) is not None
    assert detect_warning_tape(
        frame, minimum_center_y_ratio=0.50
    ) is None


def test_warning_tape_uses_saved_hsv_and_drops_small_components():
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    pale_yellow = cv2.cvtColor(
        np.asarray([[[30, 50, 220]]], dtype=np.uint8), cv2.COLOR_HSV2BGR
    )[0, 0].tolist()
    for start_x in (20, 150, 280):
        cv2.rectangle(frame, (start_x, 330), (start_x + 80, 355),
                      pale_yellow, -1)
        cv2.rectangle(frame, (start_x + 80, 330), (start_x + 120, 355),
                      (5, 5, 5), -1)
    cv2.rectangle(frame, (500, 100), (509, 109), pale_yellow, -1)
    saved = {
        "h_min": 25, "h_max": 40,
        "s_min": 30, "s_max": 71,
        "v_min": 70, "v_max": 255,
        "open_kernel": 1, "close_kernel": 1,
        "min_component_pixels": 200,
    }

    tape = detect_warning_tape(frame, filter_config=saved)

    assert tape is not None
    assert tape["component_count"] == 3
    assert tape["black_adjacent_components"] >= 2
    assert abs(tape["angle_deg"]) < 1.0


def test_warning_tape_initial_approach_finishes_near_target_height():
    fake = type("FakeDock", (), {})()
    fake.config = {
        "tape_target_center_y_ratio": 0.65,
        "tape_vertical_tolerance_ratio": 0.035,
    }
    fake.number = lambda key, default, *_args: fake.config.get(key, default)

    assert AutoDockNode.warning_tape_initial_approach_complete(
        fake, {"center_y_ratio": 0.43}
    ) is False
    assert AutoDockNode.warning_tape_initial_approach_complete(
        fake, {"center_y_ratio": 0.62}
    ) is True


def test_first_tape_detection_commands_forward_toward_target_height():
    fake = type("FakeDock", (), {})()
    fake.config = {
        "tape_guidance_only": True,
        "tape_pose_filter_alpha": 1.0,
        "tape_target_center_y_ratio": 0.65,
    }
    fake.number = lambda key, default, *_args: fake.config.get(key, default)
    fake.latest_tape_guidance = {"center_y_ratio": 0.43, "angle_deg": 0.0}
    fake.latest_tape_guidance_at = 10.0
    fake.tape_reference = None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    fake.publish_status = lambda *_args, **_kwargs: None

    correcting = AutoDockNode.command_warning_tape_search(
        fake, 10.1, 0.12, 1.0
    )

    assert correcting is True
    assert fake.tape_reference["center_y_ratio"] == pytest.approx(0.65)
    assert commands == [(0.10, 0.0, 0.0)]


@pytest.mark.parametrize(("distance_cm", "accepted"), [
    (29.9, True),
    (30.0, False),
    (33.6, False),
])
def test_top_pair_target_lock_is_near_range_only(distance_cm, accepted):
    class FakeNode:
        target_left = "spade"
        target_right = "spade"
        latest_detection_at = math.inf
        latest_detection = {
            "target_top": ["spade", "spade"],
            "candidate": {
                "entity_id": 26,
                "depth_yaw": {"forward_distance_cm": distance_cm},
                "pnp": {"forward_distance_cm": 20.0},
            },
        }

        @staticmethod
        def number(_key, default, _minimum, _maximum):
            return default

    candidate, _pnp = AutoDockNode.selected_candidate(FakeNode())

    assert (candidate is not None) is accepted


def test_dock_end_marker_detects_repeating_red_band_at_right_edge():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for top in (120, 190, 260):
        cv2.rectangle(frame, (595, top), (628, top + 34), (0, 0, 230), -1)
    for left in (80, 190, 300, 410, 550):
        cv2.rectangle(frame, (left, 302), (left + 70, 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (left + 70, 302), (left + 100, 330), (5, 5, 5), -1)
    # A red object away from the edge is not a DOCK endpoint.
    cv2.rectangle(frame, (390, 100), (480, 260), (0, 0, 230), -1)

    markers = detect_dock_end_markers(frame)

    assert "right" in markers
    assert markers["right"]["x_px"] > 590
    assert len(markers["right"]["line"]) == 2
    assert markers["right"]["line"][1][0] == markers["right"]["x_px"]
    assert "left" not in markers


def test_dock_end_uses_default_tape_fallback_when_saved_mask_misses():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for top in (120, 190, 260):
        cv2.rectangle(frame, (595, top), (628, top + 34), (0, 0, 230), -1)
    for left in (80, 190, 300, 410, 550):
        cv2.rectangle(frame, (left, 302), (left + 70, 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (left + 70, 302), (left + 100, 330), (5, 5, 5), -1)
    deliberately_missing_tape = {
        "h_min": 100, "h_max": 120,
        "s_min": 200, "s_max": 255,
        "v_min": 200, "v_max": 255,
    }

    markers = detect_dock_end_markers(
        frame, warning_tape_filter_config=deliberately_missing_tape
    )

    assert "right" in markers


def test_dock_right_end_can_appear_in_left_half_of_camera():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for top in (120, 190, 260):
        cv2.rectangle(frame, (270, top), (300, top + 34), (0, 0, 230), -1)
    for left in (20, 90, 160, 230):
        cv2.rectangle(frame, (left, 302), (left + 50, 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (left + 50, 302), (left + 70, 330), (5, 5, 5), -1)

    markers = detect_dock_end_markers(frame)

    assert "right" in markers
    assert markers["right"]["x_px"] < 320
    assert "left" not in markers


def test_dock_end_accepts_contact_intersection_just_outside_cropped_frame():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for box in (
        ((570, 140), (595, 165)),
        ((590, 185), (625, 210)),
        ((610, 235), (639, 270)),
    ):
        cv2.rectangle(frame, box[0], box[1], (0, 0, 230), -1)
    for left in (40, 150, 260, 370, 480, 590):
        cv2.rectangle(frame, (left, 302), (min(left + 70, 639), 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (min(left + 70, 639), 302), (min(left + 100, 639), 330), (5, 5, 5), -1)

    markers = detect_dock_end_markers(frame)

    assert "right" in markers
    assert markers["right"]["x_px"] >= 640


def test_dock_left_end_can_appear_in_right_half_of_camera():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for top in (120, 190, 260):
        cv2.rectangle(frame, (350, top), (380, top + 34), (0, 0, 230), -1)
    for left in (360, 430, 500, 570):
        cv2.rectangle(frame, (left, 302), (min(left + 50, 639), 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (min(left + 50, 639), 302), (min(left + 70, 639), 330), (5, 5, 5), -1)

    markers = detect_dock_end_markers(frame)

    assert "left" in markers
    assert markers["left"]["x_px"] > 320
    assert "right" not in markers


def test_dock_end_marker_uses_gui_authored_hsv_range():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    blue = cv2.cvtColor(
        np.asarray([[[115, 230, 230]]], dtype=np.uint8), cv2.COLOR_HSV2BGR
    )[0, 0]
    for top in (120, 190, 260):
        cv2.rectangle(
            frame, (595, top), (628, top + 34),
            tuple(int(value) for value in blue), -1,
        )
    for left in (80, 190, 300, 410, 550):
        cv2.rectangle(frame, (left, 302), (left + 70, 330), (0, 230, 230), -1)
        cv2.rectangle(frame, (left + 70, 302), (left + 100, 330), (5, 5, 5), -1)

    assert detect_dock_end_markers(frame) == {}
    markers = detect_dock_end_markers(
        frame,
        dock_end_filter_config={
            "h_min": 105, "h_max": 125,
            "s_min": 120, "s_max": 255,
            "v_min": 65, "v_max": 255,
            "min_component_pixels": 30,
        },
    )

    assert "right" in markers


def test_dock_end_marker_rejects_repeating_red_band_without_yellow_tape():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for top in (120, 190, 260):
        cv2.rectangle(frame, (595, top), (628, top + 34), (0, 0, 230), -1)

    assert detect_dock_end_markers(frame) == {}


def test_dock_end_marker_accepts_red_segments_with_hidden_black_contact_gap():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    for center_x, center_y in ((470, 110), (495, 180), (520, 250), (545, 320)):
        cv2.rectangle(
            frame,
            (center_x - 15, center_y - 17),
            (center_x + 15, center_y + 17),
            (0, 0, 230), -1,
        )
    for left in (40, 150, 260, 370, 480):
        cv2.rectangle(frame, (left, 400), (left + 70, 430), (0, 230, 230), -1)
        cv2.rectangle(frame, (left + 70, 400), (left + 100, 430), (5, 5, 5), -1)

    markers = detect_dock_end_markers(frame)

    assert "right" in markers
    assert markers["right"]["tape_endpoint_gap_px"] > 48.0


def test_dock_end_marker_rejects_single_red_or_low_saturation_edge_object():
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    cv2.rectangle(frame, (600, 20), (639, 140), (0, 0, 230), -1)
    low_saturation_red = cv2.cvtColor(
        np.asarray([[[8, 110, 180]]], dtype=np.uint8), cv2.COLOR_HSV2BGR
    )[0, 0]
    cv2.rectangle(
        frame, (565, 220), (625, 430),
        tuple(int(value) for value in low_saturation_red), -1,
    )

    assert detect_dock_end_markers(frame) == {}


@pytest.mark.parametrize(("matrix", "expected"), [
    (["spade", "heart", "clover", "diamond"], "NORMAL"),
    (["star"], "FRESH"),
    (["star", "star", "star", "star"], "FRESH"),
    (["star", "heart", "clover", "diamond"], None),
    (["heart"], None),
])
def test_pallet_product_type_uses_star_as_fresh(matrix, expected):
    assert pallet_product_type(matrix) == expected


def test_dock_inventory_keeps_only_nearest_visible_entity_per_lateral_row():
    tracker = DockInventoryTracker(
        depth_edges_cm=(20.0, 30.0, 40.0, 50.0),
        first_row_center_ratio=0.5,
        row_pitch_ratio=1.0,
        maximum_age_sec=3.0,
    )
    entities = [
        {
            "entity_id": 1,
            "matrix": ["star", "star", "star", "star"],
            "image_pallet_box": [520, 200, 620, 400],
            "pnp": {"forward_distance_cm": 25.0},
            "visibility_score": 8000,
        },
        {
            "entity_id": 2,
            "matrix": ["spade", "heart", "clover", "diamond"],
            "image_pallet_box": [520, 160, 620, 300],
            "pnp": {"forward_distance_cm": 45.0},
            "visibility_score": 9000,
        },
        {
            "entity_id": 3,
            "matrix": ["spade", "heart", "clover", "diamond"],
            "image_pallet_box": [420, 180, 520, 360],
            "pnp": {"forward_distance_cm": 35.0},
            "visibility_score": 7000,
        },
    ]

    snapshot = tracker.observe(
        entities,
        {"right": {"x_px": 620.0, "confidence": 1.0}},
        now=10.0,
        source_stamp_ns=123,
    )

    assert [item["slot_id"] for item in snapshot["visible_nearest"]] == [
        "DOCK_R1_C1", "DOCK_R2_C2",
    ]
    assert snapshot["visible_nearest"][0]["product_type"] == "FRESH"
    assert snapshot["visible_nearest"][0]["accessible"] is True
    assert snapshot["visible_nearest"][1]["blocked_by"] == "DOCK_R2_C1"
    assert snapshot["unreported_slots"] == "UNKNOWN"


def test_dock_inventory_reset_requires_a_fresh_observation():
    tracker = DockInventoryTracker(first_row_center_ratio=0.5, row_pitch_ratio=1.0)
    tracker.observe([
        {
            "entity_id": 1,
            "matrix": ["star", "heart", "clover", "diamond"],
            "image_pallet_box": [520, 200, 620, 400],
            "pnp": {"forward_distance_cm": 25.0},
        }
    ], {"right": {"x_px": 620.0}}, now=10.0)

    tracker.reset("pick_fork_complete")
    snapshot = tracker.snapshot(now=10.1)

    assert snapshot["visible_nearest"] == []
    assert snapshot["unreported_slots"] == "UNKNOWN"
    assert snapshot["rescan_reason"] == "pick_fork_complete"


def test_dock_inventory_tape_mode_accepts_nearest_row_below_depth_range():
    tracker = DockInventoryTracker(
        first_row_center_ratio=0.5,
        row_pitch_ratio=1.0,
        nearest_tape_only=True,
    )
    entities = [{
        "entity_id": 130,
        "matrix": ["star", "star", "star", "star"],
        "image_pallet_box": [320, 370, 570, 419],
        "pnp": {"forward_distance_cm": 11.9},
        "visibility_score": 4805,
    }]

    snapshot = tracker.observe(
        entities,
        {"right": {"x_px": 695.0}},
        now=10.0,
        tape={"center_y_ratio": 0.897, "angle_deg": 2.47},
        image_shape=(480, 640, 3),
    )

    assert [item["slot_id"] for item in snapshot["visible_nearest"]] == [
        "DOCK_R1_C1"
    ]
    assert snapshot["visible_nearest"][0]["tape_gap_ratio"] == pytest.approx(
        0.067, abs=0.002
    )


def test_dock_inventory_keeps_entity_row_and_allocates_next_free_row():
    tracker = DockInventoryTracker(
        first_row_center_ratio=0.5,
        row_pitch_ratio=1.0,
        nearest_tape_only=True,
    )
    marker = {"right": {"x_px": 620.0}}
    tape = {"center_y_ratio": 0.90, "angle_deg": 0.0}
    shape = (480, 640, 3)

    tracker.observe([{
        "entity_id": 10,
        "matrix": ["star"] * 4,
        "image_pallet_box": [470, 350, 570, 420],
    }], marker, now=1.0, tape=tape, image_shape=shape)
    snapshot = tracker.observe([{
        "entity_id": 10,
        "matrix": ["star"] * 4,
        "image_pallet_box": [370, 350, 470, 420],
    }, {
        "entity_id": 11,
        "matrix": ["star"] * 4,
        "image_pallet_box": [480, 350, 580, 420],
    }], marker, now=1.1, tape=tape, image_shape=shape)

    slots = {item["entity_id"]: item["slot_id"] for item in snapshot["visible_nearest"]}
    assert slots == {10: "DOCK_R1_C1", 11: "DOCK_R2_C1"}


def test_dock_inventory_continues_after_right_end_leaves_view():
    tracker = DockInventoryTracker(
        first_row_center_ratio=0.5,
        row_pitch_ratio=1.0,
        nearest_tape_only=True,
    )
    tape = {"center_y_ratio": 0.90, "angle_deg": 0.0}
    shape = (480, 640, 3)
    tracker.observe([{
        "entity_id": 10,
        "matrix": ["star"] * 4,
        "image_pallet_box": [470, 350, 570, 420],
    }], {"right": {"x_px": 620.0}}, now=1.0, tape=tape, image_shape=shape)

    snapshot = tracker.observe([{
        "entity_id": 11,
        "matrix": ["star"] * 4,
        "image_pallet_box": [470, 350, 570, 420],
    }], {}, now=1.1, tape=tape, image_shape=shape)

    slots = {item["entity_id"]: item["slot_id"] for item in snapshot["visible_nearest"]}
    assert slots == {10: "DOCK_R1_C1", 11: "DOCK_R2_C1"}
    assert snapshot["right_end_detected"] is True
    assert snapshot["right_end_anchor_locked"] is True

    expired = tracker.observe([], {}, now=2.6, tape=tape, image_shape=shape)
    assert expired["right_end_detected"] is False
    assert expired["right_end_anchor_locked"] is True


@pytest.mark.parametrize(("internal", "operation", "expected"), [
    ("idle", "PICK", "IDLE"),
    ("scan_sweep", "PICK", "SEARCHING"),
    ("scan_forward_search", "PICK", "SEARCHING"),
    ("scan_approach", "PICK", "ALIGNING"),
    ("search", "PICK", "SEARCHING"),
    ("confirm", "PICK", "SEARCHING"),
    ("docking", "PICK", "ALIGNING"),
    ("inserting", "PICK", "INSERTING"),
    ("waiting_fork", "PICK", "WAIT_UP_COMPLETE"),
    ("waiting_fork", "PLACE", "WAIT_DOWN_COMPLETE"),
    ("reversing_after_lift", "PICK", "REVERSING"),
    ("ready", "PICK", "READY"),
])
def test_internal_phases_publish_agreed_fsm_states(internal, operation, expected):
    assert public_fsm_state(internal, operation) == expected


def test_cancelled_event_publishes_error_state():
    assert public_fsm_state("idle", "PICK", "cancelled") == "ERROR"


def test_test_panel_can_force_unloaded_only_while_idle():
    fake = type("FakeDock", (), {})()
    fake.state = "idle"
    fake.load_state = "LOADED"
    published = []
    fake.publish_status = lambda state, reason: published.append((state, reason))

    AutoDockNode.on_test_load_state(fake, String(data="UNLOADED"))

    assert fake.load_state == "UNLOADED"
    assert published == [("idle", "test_load_state_override")]


def test_manual_dock_inventory_reset_clears_anchor_and_observations():
    fake = type("FakeDock", (), {})()
    fake.dock_inventory_tracker = DockInventoryTracker(nearest_tape_only=True)
    fake.dock_inventory_tracker.right_end_seen = True
    fake.dock_inventory_tracker.entity_rows = {10: 1}
    fake.dock_inventory_tracker.observations = {1: {"entity_id": 10}}
    published = []
    fake.publish_dock_inventory = (
        lambda snapshot, reason="observation": published.append((snapshot, reason))
    )

    AutoDockNode.on_dock_inventory_reset(fake, Empty())

    assert fake.dock_inventory_tracker.right_end_seen is False
    assert fake.dock_inventory_tracker.entity_rows == {}
    assert fake.dock_inventory_tracker.observations == {}
    assert published[-1][1] == "manual_reset"


def test_test_load_state_is_rejected_while_busy():
    fake = type("FakeDock", (), {})()
    fake.state = "inserting"
    fake.load_state = "LOADED"
    published = []
    fake.publish_status = lambda state, reason: published.append((state, reason))

    AutoDockNode.on_test_load_state(fake, String(data="UNLOADED"))

    assert fake.load_state == "LOADED"
    assert published == [("rejected", "test_load_state_while_busy")]


@pytest.mark.parametrize(("angle_deg", "direction", "distance"), [
    (0.0, "front", 0.31),
    (180.0, "rear", 0.20),
    (90.0, "left", 0.20),
    (-90.0, "right", 0.20),
    (135.0, "left", 0.20),
    (45.0, "front", 0.20),
])
def test_lidar_threshold_respects_global_stop_distance_and_body_clearance(
    angle_deg, direction, distance
):
    fake = type("FakeDock", (), {})()
    fake.number = lambda key, default, *_args: default

    got_direction, got_distance = AutoDockNode.lidar_safety_threshold(
        fake, math.radians(angle_deg)
    )

    assert got_direction == direction
    assert got_distance == pytest.approx(distance)


def test_selected_pallet_front_return_is_allowed_during_insertion():
    fake = type("FakeDock", (), {})()
    fake.state = "inserting"
    fake.number = lambda key, default, *_args: default
    fake.nearest_by_direction = {
        "front": (0.10, 0.0, 0.20),
        "rear": (math.inf, None, 0.08),
        "left": (math.inf, None, 0.08),
        "right": (math.inf, None, 0.08),
    }

    assert AutoDockNode.interrupt_for_lidar(fake) is False


def test_alignment_enforces_thirty_centimetre_rear_clearance():
    fake = type("FakeDock", (), {})()
    fake.state = "docking"
    fake.config = {"lidar_backoff_enabled": False}
    fake.number = lambda key, default, *_args: default
    fake.nearest_by_direction = {
        "front": (0.10, 0.0, 0.20),
        "rear": (0.25, math.pi, 0.20),
        "left": (math.inf, None, 0.20),
        "right": (math.inf, None, 0.20),
    }
    cancelled = []
    fake.cancel = lambda reason: cancelled.append(reason)

    assert AutoDockNode.interrupt_for_lidar(fake) is True
    assert cancelled == ["lidar_rear_blocked"]


def test_disabled_lidar_does_not_force_reverse_during_lateral_search():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {
        "search_lateral_direction": "left",
        "lidar_safety_enabled": False,
    }
    fake.number = lambda key, default, *_args: default
    fake.nearest_by_direction = {
        "front": (0.13, math.radians(-16.5), 0.20),
        "rear": (0.66, math.radians(-165.0), 0.08),
        "left": (0.53, math.radians(106.0), 0.08),
        "right": (0.20, math.radians(-59.0), 0.08),
    }

    assert AutoDockNode.interrupt_for_lidar(fake) is False


def test_known_front_chassis_return_is_self_masked():
    fake = type("FakeDock", (), {})()
    fake.number = lambda key, default, *_args: default

    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-16.5), 0.23
    ) is True
    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-16.5), 0.231
    ) is False


def test_loaded_pallet_edge_return_is_self_masked_with_width_margin():
    fake = type("FakeDock", (), {})()
    fake.load_state = "LOADED"
    fake.number = lambda key, default, *_args: default
    distance = math.hypot(0.208, 0.085)
    angle = math.atan2(0.085, 0.208)

    assert AutoDockNode.is_lidar_self_return(fake, angle, distance) is True


def test_fixed_angle_chassis_return_is_masked_without_widening_front_mask():
    fake = type("FakeDock", (), {})()
    values = {
        "lidar_self_mask_fixed_angle_deg": -1.43,
        "lidar_self_mask_fixed_half_width_deg": 1.0,
    }
    fake.number = lambda key, default, *_args: values.get(key, default)

    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-1.43), 0.232
    ) is True
    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-0.72), 0.229
    ) is True
    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-4.0), 0.232
    ) is False


def test_left_diagonal_lidar_backoff_is_pure_right_strafe(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {"tape_guidance_enabled": False}
    fake.number = lambda key, default, *_args: default
    fake.nearest_by_direction = {
        "front": (math.inf, None, 0.20),
        "rear": (math.inf, None, 0.20),
        "left": (0.12, math.radians(45.0), 0.20),
        "right": (math.inf, None, 0.20),
    }
    fake.candidate_stop_due_at = None
    fake.candidate_confirmation_started_at = None
    fake.publish_status = lambda *_args, **_kwargs: None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    assert AutoDockNode.interrupt_for_lidar(fake) is True
    assert fake.backoff_direction == "left"
    assert fake.backoff_command == (0.0, -0.08)


def test_disabled_lidar_backoff_stops_without_recovery_motion():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {
        "tape_guidance_enabled": False,
        "lidar_backoff_enabled": False,
    }
    fake.number = lambda key, default, *_args: default
    fake.nearest_by_direction = {
        "front": (math.inf, None, 0.20),
        "rear": (math.inf, None, 0.20),
        "left": (0.12, math.pi / 2.0, 0.20),
        "right": (math.inf, None, 0.20),
    }
    cancelled = []
    fake.cancel = lambda reason: cancelled.append(reason)

    assert AutoDockNode.interrupt_for_lidar(fake) is True
    assert cancelled == ["lidar_left_blocked"]
    assert not hasattr(fake, "backoff_command")


def test_disabled_lidar_safety_does_not_interrupt_driving():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {"lidar_safety_enabled": False}
    fake.nearest_by_direction = {
        direction: (0.05, 0.0, 0.20)
        for direction in ("front", "rear", "left", "right")
    }

    assert AutoDockNode.interrupt_for_lidar(fake) is False


def test_search_visible_target_is_not_treated_as_front_obstacle():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {"lidar_safety_enabled": True}
    fake.number = lambda key, default, *_args: default
    fake.selected_candidate = lambda: ({"entity_id": 7}, {})
    fake.nearest_by_direction = {
        "front": (0.20, 0.0, 0.36),
        "rear": (math.inf, None, 0.20),
        "left": (math.inf, None, 0.20),
        "right": (math.inf, None, 0.20),
    }

    assert AutoDockNode.interrupt_for_lidar(fake) is False


def test_search_without_visible_target_still_monitors_front_lidar():
    fake = type("FakeDock", (), {})()
    fake.state = "search"
    fake.config = {
        "lidar_safety_enabled": True,
        "lidar_backoff_enabled": False,
    }
    fake.number = lambda key, default, *_args: default
    fake.selected_candidate = lambda: (None, None)
    fake.nearest_by_direction = {
        "front": (0.20, 0.0, 0.36),
        "rear": (math.inf, None, 0.20),
        "left": (math.inf, None, 0.20),
        "right": (math.inf, None, 0.20),
    }
    cancelled = []
    fake.cancel = lambda reason: cancelled.append(reason)

    assert AutoDockNode.interrupt_for_lidar(fake) is True
    assert cancelled == ["lidar_front_blocked"]


def test_persistent_lidar_obstacle_retries_until_configured_limit(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.backoff_until = 10.0
    fake.backoff_direction = "left"
    fake.backoff_attempt_count = 1
    fake.backoff_command = (0.0, -0.08)
    fake.number = lambda key, default, *_args: {
        "lidar_backoff_max_attempts": 5,
        "lidar_backoff_duration_sec": 0.3,
    }.get(key, default)
    fake.nearest_by_direction = {"left": (0.12, math.pi / 2.0, 0.20)}
    fake.stop_drive = lambda *_args: None
    statuses = []
    fake.publish_status = lambda *args, **kwargs: statuses.append((args, kwargs))
    cancelled = []
    fake.cancel = lambda reason: cancelled.append(reason)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_backoff(fake)

    assert cancelled == []
    assert fake.backoff_attempt_count == 2
    assert fake.backoff_direction == "left"
    assert fake.backoff_until == pytest.approx(10.4)
    assert statuses[-1][0] == ("recovering", "lidar_backoff_retry")

    fake.backoff_until = 10.0
    fake.backoff_attempt_count = 5
    AutoDockNode.tick_backoff(fake)

    assert cancelled == ["lidar_left_blocked_after_backoff"]


def test_structured_dock_pick_allows_duplicate_symbols():
    arrival = parse_arrival(json.dumps({
        "status": "SUCCEEDED",
        "location": "DOCK_1",
        "operation": "PICK",
        "product_type": "NORMAL",
        "target": {"type": "SYMBOLS", "left": "heart", "right": "heart"},
    }))
    assert arrival["operation"] == "PICK"
    assert arrival["target"] == {
        "type": "SYMBOLS", "left": "heart", "right": "heart"
    }


def test_structured_dock_pick_accepts_nearest_product_target():
    arrival = parse_arrival(json.dumps({
        "status": "SUCCEEDED",
        "location": "DOCK_1",
        "operation": "PICK",
        "product_type": "FRESH",
        "target": {"type": "NEAREST"},
    }))

    assert arrival["target"] == {
        "type": "NEAREST", "recognition_mode": "CURRENT"
    }


def test_structured_dock_pick_accepts_legacy_nearest_recognition():
    arrival = parse_arrival(json.dumps({
        "status": "SUCCEEDED",
        "location": "DOCK_1",
        "operation": "PICK",
        "product_type": "NORMAL",
        "target": {"type": "NEAREST", "recognition_mode": "LEGACY"},
    }))

    assert arrival["target"] == {
        "type": "NEAREST", "recognition_mode": "LEGACY"
    }


def test_nearest_product_candidate_filters_product_and_uses_depth_distance():
    fake = type("FakeDock", (), {})()
    fake.product_type = "NORMAL"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {"entities": [
        {
            "entity_id": 10,
            "seen_count": 3,
            "matrix": ["heart", "spade", "diamond", "clover"],
            "image_pallet_box": [300, 200, 400, 260],
            "pnp": {"forward_distance_cm": 24.0},
            "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 1.0},
        },
        {
            "entity_id": 11,
            "seen_count": 5,
            "matrix": ["star", "star", "star", "star"],
            "image_pallet_box": [200, 200, 300, 260],
            "pnp": {"forward_distance_cm": 12.0},
            "depth_yaw": {"forward_distance_cm": 12.0, "yaw_deg": 0.0},
        },
        {
            "entity_id": 12,
            "seen_count": 4,
            "matrix": ["diamond", "spade", "heart", "clover"],
            "image_pallet_box": [400, 200, 500, 260],
            "pnp": {"forward_distance_cm": 21.0},
            "depth_yaw": {"forward_distance_cm": 21.0, "yaw_deg": 0.0},
        },
    ]}

    candidate, pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["entity_id"] == 10
    assert candidate["streak"] == 3
    assert pnp["lateral_source"] == "pallet_box"
    assert pnp["tag_lateral_ratio"] is None
    assert pnp["lateral_ratio"] == pytest.approx(
        math.tan(math.radians((350.0 - 320.0) / 320.0 * 30.0))
    )


def test_nearest_locked_candidate_uses_continuous_tracking_streak():
    fake = type("FakeDock", (), {})()
    fake.product_type = "NORMAL"
    fake.target_entity_id = 10
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "candidate": {"entity_id": 10, "streak": 8},
        "entities": [{
            "entity_id": 10,
            "seen_count": 1,
            "matrix": ["heart", "spade", "diamond", "clover"],
            "image_pallet_box": [300, 200, 400, 260],
            "pnp": {"forward_distance_cm": 24.0},
            "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 1.0},
        }],
    }

    candidate, _pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["entity_id"] == 10
    assert candidate["streak"] == 8


def test_legacy_nearest_uses_original_entity_seen_count():
    fake = type("FakeDock", (), {})()
    fake.product_type = "NORMAL"
    fake.target_entity_id = 10
    fake.nearest_recognition_mode = "LEGACY"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "candidate": {"entity_id": 10, "streak": 8},
        "entities": [{
            "entity_id": 10,
            "seen_count": 1,
            "matrix": ["heart", "spade", "diamond", "clover"],
            "image_pallet_box": [300, 200, 400, 260],
            "pnp": {"forward_distance_cm": 24.0},
            "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 1.0},
        }],
    }

    candidate, _pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["streak"] == 1


def test_nearest_product_candidate_skips_inventory_blocked_entity():
    fake = type("FakeDock", (), {})()
    fake.product_type = "NORMAL"
    fake.number = lambda _key, default, _minimum, _maximum: default
    fake.dock_inventory_tracker = type("Tracker", (), {
        "snapshot": lambda _self: {"visible_nearest": [
            {
                "entity_id": 10, "accessible": False,
                "product_type": "NORMAL",
            },
            {
                "entity_id": 12, "accessible": True,
                "product_type": "NORMAL",
            },
        ]}
    })()
    detection = {"entities": [
        {
            "entity_id": 10,
            "matrix": ["heart", "spade", "diamond", "clover"],
            "image_pallet_box": [300, 200, 400, 260],
            "pnp": {"forward_distance_cm": 18.0},
            "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 0.0},
        },
        {
            "entity_id": 12,
            "matrix": ["diamond", "spade", "heart", "clover"],
            "image_pallet_box": [400, 200, 500, 260],
            "pnp": {"forward_distance_cm": 21.0},
            "depth_yaw": {"forward_distance_cm": 21.0, "yaw_deg": 0.0},
        },
    ]}

    candidate, _pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["entity_id"] == 12


def test_nearest_fresh_accepts_one_star_attached_to_a_pallet_face():
    fake = type("FakeDock", (), {})()
    fake.product_type = "FRESH"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "entities": [],
        "detections": [
            {
                "class": "star", "confidence": 0.90,
                "box": [193, 288, 303, 370],
                "depth": {
                    "forward_distance_cm": 17.6,
                    "bearing_deg": -8.6,
                },
            },
            {
                "class": "pallet", "confidence": 0.62,
                "box": [101, 381, 329, 406],
            },
        ],
    }

    candidate, pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["pallet_tag_candidate"] is True
    assert candidate["matrix"] == ["star"]
    assert candidate["pallet_box"] == [101, 381, 329, 406]
    assert candidate["depth_yaw"]["forward_distance_cm"] == pytest.approx(17.6)
    assert pnp["depth_fallback"] is True
    assert AutoDockNode.candidate_matches_best_entity(fake, candidate) == (True, None)


@pytest.mark.parametrize(
    ("product_type", "classes"),
    [
        ("NORMAL", ["heart", "clover", "diamond", "spade"]),
        ("FRESH", ["star"]),
        ("FRESH", ["star", "star"]),
        ("FRESH", ["star", "star", "star"]),
        ("FRESH", ["star", "star", "star", "star"]),
    ],
)
def test_nearest_uses_one_to_four_tags_attached_to_pallet(product_type, classes):
    fake = type("FakeDock", (), {})()
    fake.product_type = product_type
    fake.nearest_recognition_mode = "LEGACY"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detections = [{
        "class": name,
        "confidence": 0.9,
        "box": [150 + index * 50, 220, 190 + index * 50, 300],
        "depth": {"forward_distance_cm": 25.0 + index},
    } for index, name in enumerate(classes)]
    detections.append({
        "class": "pallet", "confidence": 0.8,
        "box": [120, 310, 360, 370],
    })

    candidate, pnp = AutoDockNode.nearest_product_candidate(
        fake, {"entities": [], "detections": detections}
    )

    assert candidate["pallet_tag_candidate"] is True
    assert candidate["pallet_tag_count"] == len(classes)
    assert candidate["matrix"] == classes
    assert pnp["lateral_source"] == "pallet_box"
    assert AutoDockNode.candidate_matches_best_entity(fake, candidate) == (True, None)


@pytest.mark.parametrize(
    ("product_type", "classes"),
    [
        ("NORMAL", ["heart"]),
        ("NORMAL", ["heart", "clover", "diamond"]),
        ("NORMAL", ["heart", "clover", "diamond", "star"]),
        ("FRESH", ["heart", "star"]),
    ],
)
def test_nearest_rejects_physically_invalid_partial_tag_layouts(
    product_type, classes
):
    fake = type("FakeDock", (), {})()
    fake.product_type = product_type
    fake.nearest_recognition_mode = "LEGACY"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detections = [{
        "class": name,
        "confidence": 0.9,
        "box": [150 + index * 50, 220, 190 + index * 50, 300],
        "depth": {"forward_distance_cm": 25.0 + index},
    } for index, name in enumerate(classes)]
    detections.append({
        "class": "pallet", "confidence": 0.8,
        "box": [120, 310, 360, 370],
    })

    candidate, pnp = AutoDockNode.nearest_product_candidate(
        fake, {"entities": [], "detections": detections}
    )

    assert candidate is None
    assert pnp is None


def test_nearest_does_not_lock_partial_tag_pallet_cropped_at_image_edge():
    fake = type("FakeDock", (), {})()
    fake.product_type = "NORMAL"
    fake.nearest_recognition_mode = "LEGACY"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "entities": [],
        "detections": [
            {
                "class": "clover",
                "confidence": 0.9,
                "box": [560, 300, 610, 370],
                "depth": {"forward_distance_cm": 20.0},
            },
            {
                "class": "pallet",
                "confidence": 0.8,
                "box": [548, 390, 639, 440],
            },
        ],
    }

    candidate, pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate is None
    assert pnp is None


def test_nearest_fresh_keeps_visual_candidate_while_star_depth_is_missing():
    fake = type("FakeDock", (), {})()
    fake.product_type = "FRESH"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "entities": [],
        "detections": [
            {
                "class": "star", "confidence": 0.78,
                "box": [228, 162, 308, 238],
            },
            {
                "class": "pallet", "confidence": 0.52,
                "box": [149, 234, 328, 280],
            },
        ],
    }

    candidate, pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["fresh_single_star"] is True
    assert candidate["fresh_pose_pending"] is True
    assert candidate["pallet_box"] == [149, 234, 328, 280]
    assert candidate["depth_yaw"] is None
    assert pnp is None


@pytest.mark.parametrize("star_box", [
    [215, 170, 295, 250],
    [345, 170, 425, 250],
    [215, 285, 295, 365],
    [345, 285, 425, 365],
])
def test_nearest_fresh_accepts_one_star_in_any_face_quadrant(star_box):
    fake = type("FakeDock", (), {})()
    fake.product_type = "FRESH"
    fake.number = lambda _key, default, _minimum, _maximum: default
    detection = {
        "entities": [],
        "detections": [
            {
                "class": "star", "confidence": 0.80,
                "box": star_box,
                "depth": {
                    "forward_distance_cm": 180.0,
                    "bearing_deg": 0.0,
                },
            },
            {
                "class": "pallet", "confidence": 0.65,
                "box": [200, 380, 440, 410],
            },
        ],
    }

    candidate, _pnp = AutoDockNode.nearest_product_candidate(fake, detection)

    assert candidate["pallet_tag_candidate"] is True
    assert candidate["depth_yaw"]["forward_distance_cm"] == pytest.approx(180.0)


def test_tape_pose_hold_never_reverses_when_tape_is_already_close():
    fake = type("FakeDock", (), {})()
    fake.config = {
        "tape_guidance_enabled": True,
        "tape_pose_filter_alpha": 1.0,
    }
    fake.number = lambda key, default, *_args: fake.config.get(key, default)
    fake.latest_tape_guidance = {"center_y_ratio": 0.80, "angle_deg": 0.0}
    fake.latest_tape_guidance_at = 10.0
    fake.tape_reference = {"center_y_ratio": 0.70, "angle_deg": 0.0}
    fake.tape_filtered_center_y_ratio = 0.70
    fake.tape_filtered_angle_deg = 0.0
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    fake.publish_status = lambda *_args, **_kwargs: None

    correcting = AutoDockNode.command_warning_tape_search(
        fake, 10.1, 0.0, 0.0, pose_correction_only=True
    )

    assert correcting is False
    assert commands == []


def test_structured_slot_is_normalized():
    arrival = parse_arrival(json.dumps({
        "status": "SUCCEEDED",
        "location": "fresh",
        "operation": "place",
        "product_type": "fresh",
        "target": {"type": "slot", "slot_id": "r3c3"},
    }))
    assert arrival["location"] == "FRESH"
    assert arrival["target"]["slot_id"] == "R3C3"


@pytest.mark.parametrize("status", ["FAILED", "", "RUNNING"])
def test_only_successful_arrival_starts(status):
    with pytest.raises(ValueError, match="arrival_not_succeeded"):
        parse_arrival(json.dumps({
            "status": status, "location": "DOCK_1", "operation": "PICK",
            "product_type": "NORMAL", "target": {"type": "NONE"},
        }))


def test_non_json_arrival_is_rejected():
    with pytest.raises(ValueError, match="arrival_json"):
        parse_arrival("arrived heart heart")


def test_fresh_auto_slot_priority_starts_at_r3c3():
    occupancy = ZoneOccupancy(confirmation_frames=1)
    occupancy.observe("FRESH", 3, 3, False)
    selected = SlotSelector().select("FRESH", "PLACE", occupancy.snapshot("FRESH"))
    assert selected == "FRESH_R3_C3"


def test_slot_id_uses_arrival_zone():
    assert normalize_slot_id("NORMAL", "r2c1") == "NORMAL_R2_C1"
    with pytest.raises(ValueError, match="arrival_slot_zone_mismatch"):
        normalize_slot_id("NORMAL", "FRESH_R2_C1")


def test_blue_normal_grid_marks_brown_cell_occupied():
    frame = np.full((600, 600, 3), 210, dtype=np.uint8)
    blue = (180, 80, 15)
    cv2.rectangle(frame, (60, 60), (540, 540), blue, 18)
    for coordinate in (220, 380):
        cv2.line(frame, (coordinate, 60), (coordinate, 540), blue, 18)
        cv2.line(frame, (60, coordinate), (540, coordinate), blue, 18)
    cv2.rectangle(frame, (90, 90), (190, 190), (35, 90, 145), -1)

    observations, error = SlotGridVision().analyze(frame, "NORMAL")

    assert error is None
    assert observations["NORMAL_R3_C1"]["occupied"] is True
    assert observations["NORMAL_R3_C2"]["occupied"] is False
    assert observations["NORMAL_R1_C3"]["occupied"] is False


def test_blue_grid_corner_order_survives_strong_perspective():
    points = np.asarray([
        (20, 163), (222, 21), (555, 38), (599, 228),
    ], dtype=np.float32)

    ordered = SlotGridVision.order_corners(points)

    assert ordered.tolist() == [
        [222.0, 21.0], [555.0, 38.0],
        [599.0, 228.0], [20.0, 163.0],
    ]


def test_grid_pose_reports_perpendicular_alignment_error():
    matrix = np.asarray([
        [583.0, 0.0, 320.0],
        [0.0, 583.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    half = SlotGridVision.ZONE_SIZE_M / 2.0
    object_points = np.asarray([
        (-half, half, 0.0), (half, half, 0.0),
        (half, -half, 0.0), (-half, -half, 0.0),
    ])
    rotation_vector = np.asarray((math.radians(50.0), 0.0, 0.0))
    translation_vector = np.asarray((0.0, 0.0, 0.9))
    corners, _ = cv2.projectPoints(
        object_points, rotation_vector, translation_vector,
        matrix, np.zeros(5),
    )

    pose = SlotGridVision.pose_from_corners(
        corners.reshape(-1, 2), matrix, np.zeros(5)
    )

    assert pose is not None
    assert pose["reprojection_error_px"] < 0.1
    assert abs(pose["perpendicular_error_rad"]) < math.radians(0.5)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.mark.parametrize("operation,expected", [("PICK", "UP"), ("PLACE", "DOWN")])
def test_insertion_completion_sends_operation_command(monkeypatch, operation, expected):
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (1.0, 0.0)
    fake.operation = operation
    fake.fork_command_due_at = None
    fake.fork_pub = CapturePublisher()
    fake.stop_drive = lambda *_args: None
    fake.number = lambda key, default, *_args: {
        "insertion_distance_cm": 12.0,
        "motion_transition_pause_sec": 0.10,
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    now = [10.0]
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: now[0])

    AutoDockNode.tick_inserting(fake)
    assert fake.fork_pub.messages == []
    now[0] = 10.1
    AutoDockNode.tick_inserting(fake)

    assert fake.state == "waiting_fork"
    assert isinstance(fake.fork_pub.messages[0], String)
    assert fake.fork_pub.messages[0].data == expected


@pytest.mark.parametrize(
    "fork_state,expected_load",
    [("UP_COMPLETE", "LOADED"), ("DOWN_COMPLETE", "UNLOADED")],
)
def test_fork_completion_starts_reverse(fork_state, expected_load):
    fake = type("FakeDock", (), {})()
    fake.operation = "PICK" if fork_state == "UP_COMPLETE" else "PLACE"
    fake.location = "DOCK_1"
    fake.odom_yaw = 0.0
    fake.odom_position = (1.0, 2.0)
    fake.completed_insertion_distance_m = 0.27
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.publish_status = lambda *_args, **_kwargs: None

    AutoDockNode.finish_fork_operation(fake, fork_state)

    assert fake.load_state == expected_load
    assert fake.post_lift_reverse_start == (1.0, 2.0)
    assert fake.post_lift_reverse_target_m == pytest.approx(0.27)
    assert fake.state == "reversing_after_lift"


def test_reverse_returns_insertion_distance_then_publishes_ready():
    fake = type("FakeDock", (), {})()
    fake.post_lift_reverse_start = (0.0, 0.0)
    fake.post_lift_reverse_start_yaw = 0.0
    fake.post_lift_reverse_target_m = 0.27
    fake.odom_position = (-0.28, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.drive_ready_pub = CapturePublisher()
    fake.publish_status = lambda *_args, **_kwargs: None

    AutoDockNode.tick_reversing_after_lift(fake)

    assert fake.state == "ready"
    assert fake.post_lift_reverse_target_m is None
    assert len(fake.drive_ready_pub.messages) == 1
    assert isinstance(fake.drive_ready_pub.messages[0], Empty)


def test_search_locks_virtual_target_before_stopping():
    fake = type("FakeDock", (), {})()
    candidate = {"tag_id": 7}
    pnp = {"forward": 0.6, "lateral": 0.1, "yaw": 0.0}
    fake.candidate_stop_due_at = 0.0
    fake.selected_candidate = lambda: (candidate, pnp)
    fake.valid_measurement = lambda: (candidate, pnp, "ok")
    fake.update_world_target = lambda got_candidate, got_pnp: (
        got_candidate == candidate and got_pnp == pnp
    )
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None
    def enter_alignment(got_candidate, got_pnp):
        assert got_candidate == candidate
        assert got_pnp == pnp
        fake.state = "docking"
        return True
    fake.enter_alignment = enter_alignment

    AutoDockNode.tick_search(fake)

    assert fake.state == "docking"


def test_first_matching_frame_schedules_stop_after_point_two_seconds(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {"tag_id": 7, "streak": 1}
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (candidate, {})
    fake.number = lambda key, default, *_args: {
        "candidate_stop_delay_sec": 0.2,
        "search_linear_speed_m_s": 0.08,
        "search_lateral_speed_m_s": 0.12,
        "search_forward_compensation_m_s": 0.02,
        "search_min_wheel_component_m_s": 0.10,
        "tape_target_angle_deg": fake.config.get("tape_target_angle_deg", default),
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.config = {
        "search_lateral_direction": "left",
        "tape_guidance_enabled": False,
        "tape_target_angle_deg": 0.0,
    }
    fake.search_heading_yaw = None
    fake.odom_yaw = 0.0
    commands = []
    stops = []
    fake.stop_drive = lambda *_args: stops.append(True)
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_search(fake)

    assert fake.candidate_stop_due_at == pytest.approx(10.2)
    assert stops == [True]
    assert commands == []


def test_first_stable_nearest_candidate_stops_before_alignment(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {"entity_id": 22, "streak": 6}
    pnp = {"distance_source": "depth"}
    fake.target_type = "NEAREST"
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (candidate, pnp)
    fake.valid_measurement = lambda: (candidate, pnp, None)
    stops = []
    fake.stop_drive = lambda *_args: stops.append(True)
    fake.number = lambda key, default, *_args: {
        "candidate_stop_delay_sec": 0.2,
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.enter_alignment = lambda got_candidate, got_pnp: (
        got_candidate == candidate and got_pnp == pnp
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_search(fake)

    assert fake.candidate_stop_due_at == pytest.approx(10.2)
    assert stops == [True]


def test_nearest_complete_entity_requires_three_stable_frames():
    fake = type("FakeDock", (), {})()
    candidate = {
        "entity_id": 22,
        "matrix": ["spade", "spade", "spade", "spade"],
        "streak": 1,
        "pallet_box": [100, 100, 300, 300],
    }
    fake.target_type = "NEAREST"
    fake.target_entity_id = None
    fake.selected_candidate = lambda: (candidate, {})
    fake.boolean = lambda _key, default: False
    fake.number = lambda _key, default, *_args: default

    got_candidate, _pnp, reason = AutoDockNode.identity_measurement(fake)

    assert got_candidate is None
    assert reason == "unstable_detection"

    candidate["streak"] = 3
    got_candidate, _pnp, reason = AutoDockNode.identity_measurement(fake)

    assert got_candidate == candidate
    assert reason is None


def test_experimental_scan_locks_valid_target_before_approach(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {"entity_id": 4}
    pnp = {"distance_source": "depth"}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.stop_drive = lambda *_args: None
    fake.update_world_target = lambda got_candidate, got_pnp: (
        got_candidate == candidate and got_pnp == pnp
    )
    fake.target_entity_id = None
    fake.scan_candidate_seen_at = None
    fake.publish_status = lambda *_args, **_kwargs: None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_scan_sweep(fake)

    assert fake.state == "scan_approach"
    assert fake.target_entity_id == 4


def test_experimental_scan_sweeps_toward_left_endpoint(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "no_selected_candidate")
    fake.selected_candidate = lambda: (None, None)
    fake.odom_yaw = 0.0
    fake.scan_sweep_started_yaw = 0.0
    fake.scan_sweep_phase = 0
    fake.scan_candidate_seen_at = None
    fake.number = lambda key, default, *_args: default
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_scan_sweep(fake)

    assert commands == [(0.0, 0.0, pytest.approx(0.18))]


def test_scan_remembers_leftmost_tag_world_bearing(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "detections": [
            {"class": "heart", "box": [400, 100, 440, 140]},
            {"class": "spade", "box": [80, 100, 120, 140]},
            {"class": "pallet", "box": [20, 50, 200, 300]},
        ]
    }
    fake.latest_detection_at = 10.0
    fake.odom_yaw = 0.0
    fake.scan_sweep_started_yaw = 0.0
    fake.scan_leftmost_tag_yaw = None
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.remember_leftmost_scan_tag(fake)

    assert math.degrees(fake.scan_leftmost_tag_yaw) == pytest.approx(20.625)


def test_forward_search_moves_and_sweeps_about_tag_bearing():
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "no_target")
    fake.odom_yaw = 0.0
    fake.odom_position = (0.0, 0.0)
    fake.scan_leftmost_tag_yaw = math.radians(10.0)
    fake.scan_forward_started_position = (0.0, 0.0)
    fake.scan_forward_phase = 0
    fake.number = lambda key, default, *_args: default
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_scan_forward_search(fake)

    assert commands[0][0] == pytest.approx(0.08)
    assert commands[0][1] == 0.0
    assert commands[0][2] > 0.0


def test_experimental_scan_approach_drives_forward_and_steers():
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "stale")
    fake.target_in_body = lambda: (0.60, 0.12, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_scan_approach(fake)

    assert commands[0][0] == pytest.approx(0.08)
    assert commands[0][1] == 0.0
    assert 0.0 < commands[0][2] <= 0.16


def test_experimental_scan_approach_hands_off_near_target():
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "stale")
    fake.target_in_body = lambda: (0.21, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None

    AutoDockNode.tick_scan_approach(fake)

    assert fake.state == "docking"


def test_lateral_search_caps_compensation_to_keep_wheels_out_of_low_speed(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.config = {
        "search_lateral_direction": "right",
        "tape_guidance_enabled": False,
    }
    fake.number = lambda key, default, *_args: {
        "search_lateral_speed_m_s": 0.12,
        "search_forward_compensation_m_s": 0.08,
        "search_min_wheel_component_m_s": 0.10,
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_search(fake)

    assert commands[0][0] == pytest.approx(0.02)
    assert commands[0][1:] == (-0.12, 0.0)


def test_tag_guided_search_uses_image_centered_individual_tag_depth(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "detections": [
            {
                "class": "heart",
                "box": [350, 100, 500, 300],
                "depth": {"forward_distance_cm": 28.0, "bearing_deg": 10.0},
            },
            {
                "class": "spade",
                "box": [50, 100, 200, 300],
                "depth": {"forward_distance_cm": 22.0, "bearing_deg": -20.0},
            },
        ]
    }
    fake.latest_detection_at = 10.0
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    observation = AutoDockNode.front_search_entity_observation(fake)

    assert observation == ("heart", 28.0, 10.0)


def test_tag_guided_search_rejects_depth_over_thirty_cm_as_noise(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "detections": [{
            "class": "heart",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 30.1, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    assert AutoDockNode.front_search_entity_observation(fake) is None


def test_tag_guided_search_corrects_forward_when_front_tag_exceeds_twenty_cm(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.config = {
        "search_lateral_direction": "left",
        "tag_guided_lateral_search_enabled": True,
    }
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = 0.0
    fake.latest_detection = {
        "detections": [{
            "class": "spade",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 25.0, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    fake.number = lambda key, default, *_args: {
        "search_lateral_speed_m_s": 0.12,
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert commands == [(0.12, 0.0, 0.0)]


def test_tag_guided_search_corrects_yaw_before_distance_or_lateral(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.config = {
        "search_lateral_direction": "left",
        "tag_guided_lateral_search_enabled": True,
    }
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = math.radians(10.0)
    fake.latest_detection = {
        "detections": [{
            "class": "spade",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 18.0, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    fake.number = lambda key, default, *_args: {
        "search_lateral_speed_m_s": 0.12,
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert commands == [(0.0, 0.0, pytest.approx(-0.35))]


def rear_lidar_search_fake(rear_distance_m):
    fake = type("FakeDock", (), {})()
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.config = {
        "search_lateral_direction": "left",
        "search_rear_lidar_guidance_enabled": True,
    }
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = 0.0
    fake.scan_updated_at = 10.0
    fake.nearest_by_direction = {
        "rear": (rear_distance_m, math.pi, 0.20),
    }
    fake.number = lambda key, default, *_args: {
        "search_lateral_speed_m_s": 0.12,
        "search_rear_lidar_min_distance_cm": 30.0,
    }.get(key, default)
    fake.commands = []
    fake.statuses = []
    fake.publish_drive = lambda x, y, yaw: fake.commands.append((x, y, yaw))
    fake.stop_drive = lambda: fake.commands.append((0.0, 0.0, 0.0))
    fake.publish_status = lambda state, reason, **extra: fake.statuses.append(
        (state, reason, extra)
    )
    return fake


def dock_reverse_search_fake(rear_distance_m, travelled_m=0.0):
    fake = type("FakeDock", (), {})()
    fake.config = {"dock_reverse_target_search_enabled": True}
    fake.scan_updated_at = 10.0
    fake.nearest_by_direction = {
        "rear": (rear_distance_m, math.pi, 0.20),
    }
    fake.odom_position = (travelled_m, 0.0)
    fake.dock_reverse_search_start_position = (0.0, 0.0)
    fake.dock_reverse_search_clearance_recovery_active = False
    fake.number = lambda key, default, *_args: {
        "dock_reverse_target_search_min_rear_distance_cm": 25.0,
        "dock_reverse_target_search_speed_m_s": 0.10,
    }.get(key, default)
    fake.commands = []
    fake.statuses = []
    fake.publish_drive = lambda x, y, yaw: fake.commands.append((x, y, yaw))
    fake.stop_drive = lambda: fake.commands.append((0.0, 0.0, 0.0))
    fake.publish_status = lambda state, reason, **extra: fake.statuses.append(
        (state, reason, extra)
    )
    return fake


def test_dock_missing_target_reverses_at_dead_zone_speed_with_clear_rear():
    fake = dock_reverse_search_fake(0.45)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.commands == [(-0.10, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "dock_reverse_target_search"


def test_dock_nearest_search_uses_reverse_instead_of_warning_tape(monkeypatch):
    fake = dock_reverse_search_fake(0.45)
    fake.location = "DOCK_1"
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.config.update({
        "dock_warning_tape_search_enabled": False,
        "search_lateral_direction": "left",
    })
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(-0.10, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "dock_reverse_target_search"


def test_dock_missing_target_restores_twenty_five_centimetre_rear_clearance():
    fake = dock_reverse_search_fake(0.24)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.commands == [(0.10, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "dock_reverse_search_restoring_clearance"


def test_dock_clearance_recovery_resumes_lateral_without_reversing():
    fake = dock_reverse_search_fake(0.27)
    fake.dock_reverse_search_clearance_recovery_active = True
    fake.dock_lateral_search_standoff_reached = True
    fake.odom_position = (0.08, 0.0)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "dock_rear_clearance_held_lateral_search"
    assert fake.dock_reverse_search_clearance_recovery_active is False

    AutoDockNode.command_dock_reverse_target_search(fake, 10.2)

    assert fake.commands[-1] == (0.0, 0.12, 0.0)
    assert fake.statuses[-1][1] == "dock_rear_clearance_held_lateral_search"


def test_dock_missing_target_reverses_until_rear_standoff_not_odom_limit():
    fake = dock_reverse_search_fake(0.49, travelled_m=0.15)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.commands == [(-0.10, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "dock_reverse_target_search"


def test_dock_missing_target_skips_reverse_with_fifty_cm_rear_space():
    fake = dock_reverse_search_fake(0.50)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.dock_lateral_search_standoff_reached is True
    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "dock_rear_clearance_held_lateral_search"


def test_dock_standoff_transitions_to_left_lateral_search():
    fake = dock_reverse_search_fake(0.26)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.dock_lateral_search_standoff_reached is True
    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "dock_rear_clearance_held_lateral_search"


def test_dock_lateral_search_zeros_yaw_from_any_visible_entity(monkeypatch):
    fake = dock_reverse_search_fake(0.26)
    fake.dock_lateral_search_standoff_reached = True
    fake.latest_detection_at = 10.0
    fake.latest_detection = {"entities": [{
        "visibility_score": 5000.0,
        "depth_yaw": {"yaw_deg": 8.0},
    }]}
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.command_dock_reverse_target_search(fake, 10.1)

    assert fake.commands == [(0.0, 0.0, pytest.approx(0.35))]
    assert fake.statuses[-1][1] == "dock_lateral_search_yaw_correction"
    assert fake.statuses[-1][2]["yaw_source"] == "front_entity"


def test_dock_lateral_yaw_reference_does_not_switch_entities(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection_at = 10.0
    fake.dock_lateral_yaw_entity_id = None
    fake.latest_detection = {"entities": [
        {
            "entity_id": 10,
            "visibility_score": 5000.0,
            "depth_yaw": {"yaw_deg": 5.0},
        },
        {
            "entity_id": 20,
            "visibility_score": 4000.0,
            "depth_yaw": {"yaw_deg": -3.0},
        },
    ]}
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    first = AutoDockNode.any_front_entity_yaw_error(fake)
    assert math.degrees(first) == pytest.approx(5.0)
    assert fake.dock_lateral_yaw_entity_id == 10

    fake.latest_detection["entities"] = [
        {
            "entity_id": 10,
            "visibility_score": 1000.0,
            "depth_yaw": {"yaw_deg": 6.0},
        },
        {
            "entity_id": 20,
            "visibility_score": 9000.0,
            "depth_yaw": {"yaw_deg": -20.0},
        },
    ]

    second = AutoDockNode.any_front_entity_yaw_error(fake)
    assert math.degrees(second) == pytest.approx(6.0)
    assert fake.dock_lateral_yaw_entity_id == 10

    fake.latest_detection["entities"] = [fake.latest_detection["entities"][1]]

    assert AutoDockNode.any_front_entity_yaw_error(fake) is None
    assert fake.dock_lateral_yaw_entity_id == 10


def test_rear_lidar_search_corrects_forward_below_thirty_cm(monkeypatch):
    fake = rear_lidar_search_fake(0.29)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.12, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "rear_lidar_distance_correction"
    assert fake.statuses[-1][2]["rear_distance_cm"] == pytest.approx(29.0)


def test_rear_lidar_search_strafes_at_thirty_cm(monkeypatch):
    fake = rear_lidar_search_fake(0.30)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "rear_lidar_clearance_held_lateral_search"
    assert fake.statuses[-1][2]["rear_distance_cm"] == pytest.approx(30.0)


def test_combined_search_checks_rear_then_uses_front_yolo(monkeypatch):
    fake = rear_lidar_search_fake(0.30)
    fake.config["tag_guided_lateral_search_enabled"] = True
    fake.latest_detection = {
        "detections": [{
            "class": "spade",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 25.0, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.12, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "front_tag_distance_correction"


def test_combined_search_does_not_move_forward_when_front_is_already_close(
    monkeypatch,
):
    fake = rear_lidar_search_fake(0.29)
    fake.config["tag_guided_lateral_search_enabled"] = True
    fake.latest_detection = {
        "detections": [{
            "class": "spade",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 18.0, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "search_longitudinal_clearance_conflict"


def test_combined_search_does_not_strafe_when_front_tag_is_too_close(
    monkeypatch,
):
    fake = rear_lidar_search_fake(0.30)
    fake.config["tag_guided_lateral_search_enabled"] = True
    fake.latest_detection = {
        "detections": [{
            "class": "spade",
            "box": [250, 100, 390, 300],
            "depth": {"forward_distance_cm": 8.0, "bearing_deg": 0.0},
        }]
    }
    fake.latest_detection_at = 10.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "search_longitudinal_clearance_conflict"


def test_combined_search_falls_back_to_rear_when_front_depth_missing(monkeypatch):
    fake = rear_lidar_search_fake(0.30)
    fake.config["tag_guided_lateral_search_enabled"] = True
    fake.latest_detection = {"detections": []}
    fake.latest_detection_at = 10.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "rear_lidar_clearance_held_lateral_search"


def tape_search_fake(angle_deg):
    fake = type("FakeDock", (), {})()
    fake.candidate_stop_due_at = None
    fake.candidate_retry_not_before = 0.0
    fake.selected_candidate = lambda: (None, None)
    fake.latest_tape_guidance = {
        "center_y_ratio": 0.80,
        "angle_deg": angle_deg,
    }
    fake.latest_tape_guidance_at = 10.0
    fake.tape_reference = None
    fake.tape_recovery_start_position = None
    fake.tape_recovery_direction = None
    fake.tape_recovery_done = False
    fake.config = {
        "search_lateral_direction": "left",
        "tape_guidance_enabled": False,
    }
    fake.number = lambda key, default, *_args: {
        "search_linear_speed_m_s": 0.08,
        "search_lateral_speed_m_s": 0.12,
        "search_forward_compensation_m_s": 0.02,
        "search_min_wheel_component_m_s": 0.10,
    }.get(key, default)
    fake.commands = []
    fake.statuses = []
    fake.publish_drive = lambda x, y, yaw: fake.commands.append((x, y, yaw))
    fake.publish_status = lambda state, reason, **extra: fake.statuses.append(
        (state, reason, extra)
    )
    return fake


def test_tape_only_search_latches_initial_pose_and_strafes(monkeypatch):
    fake = tape_search_fake(angle_deg=0.0)
    fake.config["tape_guidance_only"] = True
    fake.latest_tape_guidance["center_y_ratio"] = 0.65
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.tape_reference == {
        "center_y_ratio": 0.65,
        "angle_deg": 0.0,
    }
    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "warning_tape_pose_held_lateral_search"


def test_tape_only_search_corrects_yaw_before_lateral(monkeypatch):
    fake = tape_search_fake(angle_deg=10.0)
    fake.config["tape_guidance_only"] = True
    fake.tape_reference = {"center_y_ratio": 0.80, "angle_deg": 0.0}
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands[0][:2] == (0.0, 0.0)
    assert fake.commands[0][2] < 0.0
    assert fake.statuses[-1][1] == "warning_tape_yaw_correction_before_lateral"


def test_tape_only_search_uses_configured_angle_not_initial_diagonal(monkeypatch):
    fake = tape_search_fake(angle_deg=10.0)
    fake.config["tape_guidance_only"] = True
    fake.config["tape_target_angle_deg"] = 0.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.tape_reference["angle_deg"] == 0.0
    assert fake.commands[0][:2] == (0.0, 0.0)
    assert fake.commands[0][2] < 0.0


def test_tape_only_search_does_not_reverse_when_tape_is_already_close(monkeypatch):
    fake = tape_search_fake(angle_deg=5.0)
    fake.config["tape_guidance_only"] = True
    fake.latest_tape_guidance["center_y_ratio"] = 0.84
    fake.tape_reference = {"center_y_ratio": 0.80, "angle_deg": 5.0}
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "warning_tape_pose_held_lateral_search"
    assert fake.statuses[-1][2]["close_tape_reverse_suppressed"] is True


def test_tape_only_search_never_falls_through_to_rear_lidar_forward(monkeypatch):
    fake = tape_search_fake(angle_deg=0.0)
    fake.config["tape_guidance_only"] = True
    fake.config["search_rear_lidar_guidance_enabled"] = True
    fake.latest_tape_guidance = None
    fake.tape_recovery_done = True
    fake.stop_drive = lambda *_args: None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands == [(0.0, 0.12, 0.0)]
    assert fake.statuses[-1][1] == "warning_tape_missing_lateral_search"


def test_guarded_tick_retains_state_when_one_control_tick_raises(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.state = "coarse_align"
    fake.last_tick_error_at = 0.0
    fake.last_tick_error_signature = None
    fake.tick = lambda: (_ for _ in ()).throw(KeyError("bad frame"))
    stops = []
    statuses = []
    errors = []
    fake.stop_drive = lambda *_args: stops.append(True)
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    fake.get_logger = lambda: type(
        "Logger", (), {"error": lambda _self, message: errors.append(message)}
    )()
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.guarded_tick(fake)

    assert fake.state == "coarse_align"
    assert stops == [True]
    assert statuses[-1][1] == "internal_tick_error"
    assert "KeyError" in errors[-1]


def missing_tape_recovery_fake(front_margin, rear_margin):
    fake = type("FakeDock", (), {})()
    fake.tape_recovery_done = False
    fake.tape_recovery_direction = None
    fake.tape_recovery_start_position = None
    fake.scan_updated_at = 10.0
    fake.odom_position = (1.0, 2.0)
    fake.nearest_by_direction = {
        "front": (front_margin + 0.35, 0.0, 0.35),
        "rear": (rear_margin + 0.05, math.pi, 0.05),
    }
    fake.config = {}
    fake.number = lambda key, default, *_args: default
    fake.stops = []
    fake.commands = []
    fake.statuses = []
    fake.stop_drive = lambda *_args: fake.stops.append(True)
    fake.publish_drive = lambda x, y, yaw: fake.commands.append((x, y, yaw))
    fake.publish_status = lambda state, reason, **extra: fake.statuses.append(
        (state, reason, extra)
    )
    return fake


def test_missing_tape_nudges_toward_larger_front_clearance():
    fake = missing_tape_recovery_fake(front_margin=0.80, rear_margin=0.30)

    AutoDockNode.tick_missing_tape_recovery(fake, 10.1)

    assert fake.tape_recovery_direction == "front"
    assert fake.commands == [(0.08, 0.0, 0.0)]
    assert fake.statuses[-1][1] == "warning_tape_clearance_nudge"


def test_missing_tape_nudges_toward_larger_rear_clearance():
    fake = missing_tape_recovery_fake(front_margin=0.25, rear_margin=0.90)

    AutoDockNode.tick_missing_tape_recovery(fake, 10.1)

    assert fake.tape_recovery_direction == "rear"
    assert fake.commands == [(-0.08, 0.0, 0.0)]


def test_tape_only_recovery_forces_short_reverse_when_front_is_clearer():
    fake = missing_tape_recovery_fake(front_margin=0.80, rear_margin=0.30)

    AutoDockNode.tick_missing_tape_recovery(
        fake, 10.1, preferred_direction="rear"
    )

    assert fake.tape_recovery_direction == "rear"
    assert fake.commands == [(-0.08, 0.0, 0.0)]


def test_missing_tape_nudge_stops_after_configured_distance():
    fake = missing_tape_recovery_fake(front_margin=0.80, rear_margin=0.30)
    AutoDockNode.tick_missing_tape_recovery(fake, 10.1)
    fake.odom_position = (1.08, 2.0)

    AutoDockNode.tick_missing_tape_recovery(fake, 10.2)

    assert fake.tape_recovery_done is True
    assert fake.stops == [True]
    assert fake.statuses[-1][1] == "warning_tape_not_detected_after_nudge"


def test_invalid_candidate_resumes_search_after_confirmation_timeout(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "invalid_pnp")
    fake.identity_measurement = lambda: (None, None, "invalid_pnp")
    fake.selected_candidate = lambda: ({"entity_id": 45}, {})
    fake.candidate_confirmation_started_at = 10.0
    fake.number = lambda key, default, *_args: {
        "candidate_confirmation_timeout_sec": 0.8,
        "candidate_retry_cooldown_sec": 1.0,
    }.get(key, default)
    fake.stop_drive = lambda *_args: pytest.fail("must resume before another stop")
    published = []
    fake.publish_status = lambda state, reason, **extra: published.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 11.0)

    AutoDockNode.tick_confirm(fake)

    assert fake.state == "search"
    assert fake.candidate_retry_not_before == pytest.approx(12.0)
    assert published[0][1] == "candidate_invalid_resume_search"


def test_depth_fallback_locks_provisional_world_target_before_coarse_alignment():
    fake = type("FakeDock", (), {})()
    entered = []
    locked = []
    fake.enter_coarse_alignment = lambda reason: entered.append(reason)
    fake.update_world_target = lambda *args, **kwargs: locked.append(
        (args, kwargs)
    ) or True

    result = AutoDockNode.enter_alignment(
        fake, {"center_error": 0.4}, {"depth_fallback": True}
    )

    assert result is True
    assert locked
    assert entered == ["pnp_quality_fallback"]


def test_nearest_initial_entity_saves_pose_before_visual_tracking():
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = None
    fake.target_world = None
    fake.nearest_alignment_distance_cm = None
    fake.send_yolo_target = lambda: None
    fake.reset_coarse_alignment = lambda: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    fake.update_world_target = lambda *_args, **_kwargs: setattr(
        fake, "target_world", {"x": 1.0, "y": 2.0, "yaw": 0.0}
    ) or True

    candidate = {
        "entity_id": 155,
        "matrix": ["diamond", "diamond", "diamond", "heart"],
        "center_error": -0.4,
        "pnp": {"forward_distance_cm": 18.0},
    }
    result = AutoDockNode.enter_alignment(fake, candidate, candidate["pnp"])

    assert result is True
    assert fake.target_entity_id == 155
    assert fake.target_world == {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert fake.target_last_center_error == pytest.approx(-0.4)
    assert fake.state == "coarse_align"
    assert fake.nearest_center_reconfirm_pending is True
    assert fake.nearest_center_reconfirm_due_at is None
    assert statuses[-1][1] == "nearest_target_locked_centering_first"


def test_visible_requested_top_pair_recovers_without_pallet_entity(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection_at = 10.0
    fake.target_left = "spade"
    fake.target_right = "heart"
    fake.target_entity_id = None
    fake.latest_detection = {
        "target_top": ["spade", "heart"],
        "detections": [
            {
                "class": "spade", "box": [170, 210, 314, 342],
                "depth": {
                    "camera_depth_m": 0.239,
                    "forward_distance_cm": 9.7,
                    "bearing_deg": -9.2,
                },
            },
            {"class": "heart", "box": [318, 230, 462, 346]},
            {"class": "spade", "box": [20, 20, 80, 80]},
        ],
    }
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    candidate, pnp, reason = AutoDockNode.visible_target_top_pair_measurement(fake)

    assert reason is None
    assert candidate["center_error"] == pytest.approx(-0.0125)
    assert candidate["depth_yaw"]["forward_distance_cm"] == pytest.approx(9.7)
    assert pnp["depth_fallback"] is True
    assert pnp["visual_only"] is False


def test_visible_locked_top_pair_without_depth_is_valid_for_centering(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection_at = 10.0
    fake.target_left = "diamond"
    fake.target_right = "diamond"
    fake.target_entity_id = 155
    fake.target_last_center_error = -0.5
    fake.latest_detection = {
        "target_top": ["diamond", "diamond"],
        "detections": [
            {"class": "diamond", "box": [176, 123, 323, 253]},
            {"class": "diamond", "box": [321, 90, 447, 226]},
            {"class": "diamond", "box": [454, 120, 503, 245]},
        ],
    }
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    candidate, pnp, reason = AutoDockNode.visible_target_top_pair_measurement(fake)

    assert reason is None
    assert candidate["entity_id"] == 155
    assert candidate["depth_yaw"] is None
    assert pnp["depth_fallback"] is True
    assert pnp["visual_only"] is True


def test_coarse_target_loss_continues_with_provisional_world_target():
    fake = type("FakeDock", (), {})()
    fake.identity_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (
        None, None, "partial_entity_unavailable"
    )
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    fake.target_entity_id = 22
    reset = []
    fake.reset_coarse_alignment = lambda: reset.append(True)
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state == "docking"
    assert reset == [True]
    assert statuses[-1][1] == "coarse_target_lost_using_virtual_target"


def test_nearest_coarse_visual_loss_uses_initial_locked_pose():
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.identity_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (
        None, None, "partial_entity_unavailable"
    )
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    fake.target_entity_id = 22
    fake.candidate_stop_due_at = 10.0
    fake.candidate_confirmation_started_at = 9.0
    fake.candidate_retry_not_before = 12.0
    fake.stop_drive = lambda *_args: None
    resets = []
    fake.reset_coarse_alignment = lambda: resets.append(True)
    headings = []
    fake.latch_search_heading = lambda: headings.append(True)
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state == "docking"
    assert fake.target_world == {"x": 1.0, "y": 0.0, "yaw": 0.0}
    assert fake.target_entity_id == 22
    assert resets == [True]
    assert headings == []
    assert statuses[-1][1] == "nearest_visual_lost_using_locked_pose"


def test_nearest_coarse_target_without_locked_pose_resumes_search():
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.identity_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (
        None, None, "partial_entity_unavailable"
    )
    fake.target_world = None
    fake.target_entity_id = 22
    fake.candidate_stop_due_at = 10.0
    fake.candidate_confirmation_started_at = 9.0
    fake.candidate_retry_not_before = 12.0
    fake.stop_drive = lambda *_args: None
    fake.reset_coarse_alignment = lambda: None
    fake.latch_search_heading = lambda: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state == "search"
    assert fake.target_entity_id is None
    assert statuses[-1][1] == "nearest_target_lost_resume_search"


def test_coarse_alignment_timeout_keeps_locked_target(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.state = "coarse_align"
    candidate = {"center_error": 0.5, "depth_yaw": {"yaw_deg": 0.0}}
    fake.identity_measurement = lambda: (candidate, {}, None)
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.valid_measurement = lambda: (None, None, "invalid_pnp")
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.target_entity_id = 22
    fake.number = lambda key, default, *_args: default
    fake.publish_drive = lambda *_args: None
    fake.stop_drive = lambda *_args: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 16.0)

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state != "search"
    assert fake.coarse_alignment_started_at == pytest.approx(16.0)
    assert any(
        reason == "coarse_alignment_continuing_locked_target"
        for _state, reason, _extra in statuses
    )


def test_coarse_alignment_moves_laterally_from_image_center_error(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {"center_error": 0.5, "depth_yaw": {"yaw_deg": 0.0}}
    fallback_pnp = {"depth_fallback": True}
    fake.identity_measurement = lambda: (candidate, fallback_pnp, None)
    fake.valid_measurement = lambda: (candidate, fallback_pnp, None)
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: default
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = 0.0
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 11.0)

    AutoDockNode.tick_coarse_align(fake)

    assert commands == [(0.0, -0.05, 0.0)]


def test_nearest_centers_laterally_without_rotation_before_recheck(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {
        "entity_id": 8,
        "center_error": 0.5,
        "depth_yaw": {"yaw_deg": 35.0},
    }
    pnp = {"depth_fallback": True}
    fake.target_type = "NEAREST"
    fake.nearest_center_reconfirm_pending = True
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: default
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = 0.0
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 11.0)

    AutoDockNode.tick_coarse_align(fake)

    assert commands == [(0.0, -0.10, 0.0)]


def test_nearest_centered_waits_before_optimal_target_recheck(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {"entity_id": 8, "center_error": 0.02}
    pnp = {"depth_fallback": False}
    fake.target_type = "NEAREST"
    fake.target_entity_id = 8
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    fake.nearest_center_reconfirm_pending = True
    fake.nearest_center_reconfirm_due_at = None
    fake.latest_detection = {"source_stamp_ns": 100}
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: (
        3.1 if key == "nearest_optimal_recheck_delay_sec" else default
    )
    fake.stop_drive = lambda *_args: None
    fake.publish_drive = lambda *_args: pytest.fail("must remain stopped")
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_coarse_align(fake)

    assert fake.nearest_center_reconfirm_due_at == pytest.approx(13.1)
    assert statuses[-1][1] == "nearest_centered_waiting_optimal_recheck"


def test_nearest_uses_fresh_optimal_target_after_center_recheck(monkeypatch):
    fake = type("FakeDock", (), {})()
    candidate = {
        "entity_id": 12,
        "matrix": ["heart", "clover", "diamond", "spade"],
        "center_error": 0.01,
        "pnp": {"forward_distance_cm": 30.0},
    }
    pnp = candidate["pnp"]
    fake.target_type = "NEAREST"
    fake.target_entity_id = 8
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    fake.nearest_center_reconfirm_pending = True
    fake.nearest_center_reconfirm_due_at = 10.0
    fake.nearest_center_reconfirm_source_stamp_ns = 100
    fake.latest_detection = {"source_stamp_ns": 101}
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.coarse_alignment_started_at = 9.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.send_yolo_target = lambda: None
    fake.update_world_target = lambda *_args, **_kwargs: True
    fake.reset_coarse_alignment = lambda: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.5)

    AutoDockNode.tick_coarse_align(fake)

    assert fake.target_entity_id == 12
    assert fake.nearest_center_reconfirm_pending is False
    assert fake.state == "docking"
    assert statuses[-1][1] == "nearest_optimal_target_reconfirmed"


def test_nearest_centered_visual_pair_uses_saved_pose(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 155
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    candidate = {"center_error": 0.02, "depth_yaw": None}
    visual_pnp = {"depth_fallback": True, "visual_only": True}
    fake.identity_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.visible_target_top_pair_measurement = lambda: (
        candidate, visual_pnp, None
    )
    fake.valid_measurement = lambda: (None, None, "distance_unavailable")
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.reset_coarse_alignment = lambda: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state == "docking"
    assert fake.target_world == {"x": 1.0, "y": 0.0, "yaw": 0.0}
    assert statuses[-1][1] == "nearest_centered_using_locked_pose"


def test_nearest_centered_full_entity_does_not_require_current_valid_pnp(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 155
    fake.target_world = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    candidate = {"center_error": 0.02, "pnp": {"reprojection_error_px": 20.0}}
    fake.identity_measurement = lambda: (candidate, candidate["pnp"], None)
    fake.tracked_partial_measurement = lambda: (None, None, None)
    fake.valid_measurement = lambda: (None, None, "invalid_pnp")
    fake.coarse_alignment_started_at = 10.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.reset_coarse_alignment = lambda: None
    statuses = []
    fake.publish_status = lambda state, reason, **extra: statuses.append(
        (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_coarse_align(fake)

    assert fake.state == "docking"
    assert statuses[-1][1] == "nearest_centered_using_locked_pose"


def test_locked_entity_upper_pair_keeps_coarse_alignment(monkeypatch):
    fake = type("FakeDock", (), {})()
    partial = {
        "entity_id": 42,
        "center_error": -0.4,
        "age_sec": 0.6,
        "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 0.0},
    }
    fake.latest_detection = {
        "target_top": ["heart", "clover"],
        "tracked_partial": partial,
    }
    fake.latest_detection_at = 9.9
    fake.target_left = "heart"
    fake.target_right = "clover"
    fake.target_entity_id = 42
    fake.identity_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (
        AutoDockNode.tracked_partial_measurement(fake)
    )
    fake.valid_measurement = lambda: (None, None, "no_selected_candidate")
    fake.coarse_alignment_started_at = 9.0
    fake.coarse_depth_fallback_frames = 0
    fake.coarse_last_counted_stamp = None
    fake.search_heading_yaw = 0.0
    fake.odom_yaw = 0.0
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_coarse_align(fake)

    assert commands[0][0] == 0.0
    assert commands[0][1] == pytest.approx(0.04)
    assert commands[0][2] == 0.0


def test_partial_pair_keeps_locked_identity_even_if_tracker_entity_id_changes(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "target_top": ["heart", "clover"],
        "tracked_partial": {
            "entity_id": 99,
            "center_error": 0.0,
            "age_sec": 0.2,
            "depth_yaw": {"forward_distance_cm": 18.0, "yaw_deg": 0.0},
        },
    }
    fake.latest_detection_at = 9.9
    fake.target_left = "heart"
    fake.target_right = "clover"
    fake.target_entity_id = 42
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    candidate, pnp, reason = AutoDockNode.tracked_partial_measurement(fake)

    assert reason is None
    assert candidate["entity_id"] == 42
    assert candidate["center_error"] == pytest.approx(0.0)
    assert pnp["depth_fallback"] is True


def test_candidate_is_rejected_when_best_same_pallet_entity_disagrees():
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "entities": [
            {
                "matrix": ["clover", "spade", "heart", "clover"],
                "visibility_score": 7600.0,
                "image_pallet_box": [100, 364, 336, 406],
            },
            {
                "matrix": ["clover", "clover", "heart", "diamond"],
                "visibility_score": 1100.0,
                "image_pallet_box": [100, 364, 336, 406],
            },
        ]
    }
    fake.target_left = "clover"
    fake.target_right = "clover"
    fake.number = lambda key, default, *_args: default
    candidate = {"pallet_box": [100, 364, 336, 406]}

    matches, reason = AutoDockNode.candidate_matches_best_entity(fake, candidate)

    assert matches is False
    assert reason == "conflicting_entity_matrix"


def test_candidate_is_accepted_when_best_same_pallet_entity_agrees():
    fake = type("FakeDock", (), {})()
    fake.latest_detection = {
        "entities": [{
            "matrix": ["clover", "clover", "heart", "diamond"],
            "visibility_score": 7600.0,
            "image_pallet_box": [100, 364, 336, 406],
        }]
    }
    fake.target_left = "clover"
    fake.target_right = "clover"
    fake.number = lambda key, default, *_args: default
    candidate = {"pallet_box": [100, 364, 336, 406]}

    matches, reason = AutoDockNode.candidate_matches_best_entity(fake, candidate)

    assert matches is True
    assert reason is None


def test_stable_target_uses_depth_when_pnp_quality_is_bad():
    fake = type("FakeDock", (), {})()
    candidate = {
        "streak": 10,
        "center_error": 0.24,
        "frontal_error": -0.43,
        "depth_yaw": {"forward_distance_cm": 15.9, "yaw_deg": 4.1},
    }
    pnp = {
        "reprojection_error_px": 35.8,
        "lateral_ratio": 0.10,
        "yaw_deg": -13.8,
    }
    fake.selected_candidate = lambda: (candidate, pnp)
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.number = lambda key, *_args: 2 if key == "stable_detection_frames" else 0

    got_candidate, got_pnp, reason = AutoDockNode.valid_measurement(fake)

    assert got_candidate == candidate
    assert reason is None
    assert got_pnp["depth_fallback"] is True
    assert got_pnp["lateral_ratio"] == pytest.approx(0.12)


def test_good_pnp_distance_is_used_when_depth_is_missing():
    candidate = {"frontal_error": 0.06, "depth_yaw": None}
    pnp = {
        "forward_distance_cm": 18.4,
        "reprojection_error_px": 3.04,
        "lateral_ratio": -0.06,
    }
    fake = type("FakeDock", (), {})()
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.number = lambda key, default, *_args: {
        "max_pnp_reprojection_error_px": 3.5,
        "max_frontal_error": 0.35,
    }.get(key, default)

    got_candidate, got_pnp, reason = AutoDockNode.valid_measurement(fake)

    assert got_candidate == candidate
    assert reason is None
    assert got_pnp["distance_source"] == "pnp"


def test_close_eight_cm_pnp_measurement_remains_valid_for_insertion():
    candidate = {"frontal_error": 0.06, "depth_yaw": None}
    pnp = {
        "forward_distance_cm": 8.26,
        "reprojection_error_px": 2.35,
        "lateral_ratio": -0.03,
        "yaw_deg": 3.1,
    }
    fake = type("FakeDock", (), {})()
    fake.identity_measurement = lambda: (candidate, pnp, None)
    fake.number = lambda key, default, *_args: default

    got_candidate, got_pnp, reason = AutoDockNode.valid_measurement(fake)

    assert got_candidate == candidate
    assert reason is None
    assert got_pnp["distance_source"] == "pnp"


def test_close_valid_target_pauses_before_insertion(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.config = {"tape_guidance_enabled": True}
    fake.latest_tape_guidance = None
    fake.latest_tape_guidance_at = 0.0
    fake.tape_initial_detection_complete = False
    candidate = {"depth_yaw": {"yaw_deg": 0.0}}
    pnp = {"yaw_deg": 0.0, "depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.update_world_target = lambda *_args, **_kwargs: True
    fake.target_in_body = lambda: (0.159, 0.0, 0.0)
    fake.number = lambda key, default, *_args: {
        "dock_standoff_m": 0.20,
        "motion_transition_pause_sec": 0.10,
        "alignment_yaw_confirmation_frames": 1,
    }.get(key, default)
    fake.stop_drive = lambda *_args: None
    fake.odom_position = (1.0, 2.0)
    fake.odom_yaw = 0.0
    fake.insertion_start_due_at = None
    fake.publish_status = lambda *_args, **_kwargs: None
    now = [10.0]
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: now[0])

    AutoDockNode.tick_docking(fake)
    assert fake.insertion_start_due_at == pytest.approx(10.1)
    now[0] = 10.1
    AutoDockNode.tick_docking(fake)

    assert fake.state == "inserting"
    assert fake.insert_start_position == (1.0, 2.0)
    assert fake.insert_start_yaw == 0.0
    assert fake.insertion_entry_gap_m == pytest.approx(0.159)


def test_nearest_reaches_odom_target_then_stops_for_fresh_frame(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 40
    fake.latest_detection = {"source_stamp_ns": 100}
    fake.target_in_body = lambda: (0.20, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda state, reason, **extra: setattr(
        fake, "last_status", (state, reason, extra)
    )
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_docking(fake)

    assert fake.nearest_step_settle_until == pytest.approx(10.5)
    assert fake.nearest_step_source_stamp_ns == 100
    assert fake.insertion_start_due_at is None
    assert fake.last_status[1] == "nearest_odom_step_complete_settling"


def test_nearest_recheck_requires_new_frame_before_relocking(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 40
    fake.latest_detection = {"source_stamp_ns": 100}
    fake.nearest_step_settle_until = 10.0
    fake.nearest_step_source_stamp_ns = 100
    fake.nearest_step_wait_started_at = None
    fake.nearest_step_count = 1
    candidate = {"entity_id": 40}
    pnp = {"depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.update_world_target = lambda *_args, **_kwargs: pytest.fail(
        "stale frame must not relock the target"
    )
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda state, reason, **extra: setattr(
        fake, "last_status", (state, reason, extra)
    )
    fake.insertion_start_due_at = None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.5)

    AutoDockNode.tick_docking(fake)

    assert fake.last_status[1] == "nearest_waiting_fresh_recheck"
    assert fake.nearest_step_settle_until == 10.0


def test_nearest_recheck_accepts_locked_upper_pair_on_new_frame(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 52
    fake.latest_detection = {"source_stamp_ns": 101}
    fake.nearest_step_settle_until = 10.0
    fake.nearest_step_source_stamp_ns = 100
    fake.nearest_step_wait_started_at = 9.0
    fake.nearest_step_count = 1
    fake.nearest_step_verified = False
    partial = {"entity_id": 52, "depth_yaw": {"forward_distance_cm": 12.7}}
    partial_pnp = {"depth_fallback": True, "lateral_ratio": -0.25}
    fake.valid_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (partial, partial_pnp, None)
    updates = []
    fake.update_world_target = lambda candidate, pnp, **kwargs: (
        updates.append((candidate, pnp, kwargs)) or True
    )
    fake.target_in_body = lambda: (0.20, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda state, reason, **extra: setattr(
        fake, "last_status", (state, reason, extra)
    )
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.5)

    AutoDockNode.tick_docking(fake)

    assert updates == [(partial, partial_pnp, {"blend_existing": False})]
    assert fake.nearest_step_settle_until is None
    assert fake.nearest_step_verified is True
    assert fake.insertion_start_due_at == pytest.approx(10.6)


def test_nearest_recheck_accepts_changed_entity_id_by_pose(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 52
    fake.latest_detection = {"source_stamp_ns": 101}
    fake.nearest_step_settle_until = 10.0
    fake.nearest_step_source_stamp_ns = 100
    fake.nearest_step_wait_started_at = 9.0
    fake.nearest_step_count = 1
    fake.nearest_step_verified = False
    wrong = {"entity_id": 70}
    wrong_pnp = {"depth_fallback": False}
    fake.valid_measurement = lambda: (wrong, wrong_pnp, None)
    fake.tracked_partial_measurement = lambda: (None, None, "unused")
    fake.measurement_matches_locked_target = lambda candidate, pnp: True
    updates = []
    fake.update_world_target = lambda candidate, pnp, **kwargs: (
        updates.append((candidate, pnp, kwargs)) or True
    )
    fake.target_in_body = lambda: (0.20, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda state, reason, **extra: setattr(
        fake, "last_status", (state, reason, extra)
    )
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.5)

    AutoDockNode.tick_docking(fake)

    assert updates == [(wrong, wrong_pnp, {"blend_existing": False})]
    assert fake.nearest_step_verified is True
    assert fake.insertion_start_due_at == pytest.approx(10.6)


def test_nearest_recheck_waits_for_visual_before_insertion(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = None
    fake.latest_detection = {"source_stamp_ns": 101}
    fake.nearest_step_settle_until = 10.0
    fake.nearest_step_source_stamp_ns = 100
    fake.nearest_step_wait_started_at = 10.0
    fake.nearest_step_count = 1
    fake.nearest_step_verified = False
    fake.valid_measurement = lambda: (None, None, "no_selected_candidate")
    fake.tracked_partial_measurement = lambda: (
        None, None, "partial_entity_unavailable"
    )
    fake.update_world_target = lambda *_args, **_kwargs: pytest.fail(
        "locked odom target must be kept"
    )
    fake.target_in_body = lambda: (0.20, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda state, reason, **extra: setattr(
        fake, "last_status", (state, reason, extra)
    )
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.5)

    AutoDockNode.tick_docking(fake)

    assert fake.nearest_step_verified is False
    assert fake.insertion_start_due_at is None
    assert fake.last_status[1] == "nearest_waiting_visual_reacquire"


def test_nearest_recheck_lost_target_resumes_lateral_search(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 21
    fake.target_world = {"x": 0.20, "y": 0.0, "yaw": 0.0}
    fake.target_last_center_error = 0.0
    fake.dock_lateral_yaw_entity_id = 21
    fake.nearest_step_settle_until = 10.0
    fake.nearest_step_wait_started_at = 9.0
    fake.nearest_step_source_stamp_ns = 100
    fake.nearest_step_verified = False
    fake.candidate_stop_due_at = None
    fake.candidate_confirmation_started_at = None
    fake.candidate_retry_not_before = 0.0
    fake.latest_detection = {"source_stamp_ns": 101}
    fake.valid_measurement = lambda: (None, None, "identity_missing")
    fake.tracked_partial_measurement = lambda: (
        None, None, "tracked_partial_missing"
    )
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *args, **kwargs: setattr(
        fake, "last_status", (args, kwargs)
    )
    monkeypatch.setattr(
        "auto_dock.auto_dock_node.time.monotonic", lambda: 10.0
    )

    AutoDockNode.tick_docking(fake)

    assert fake.state == "search"
    assert fake.target_entity_id is None
    assert fake.target_world is None
    assert fake.dock_lateral_yaw_entity_id is None
    assert fake.last_status[0][1] == "nearest_visual_lost_resume_lateral_search"


def test_aligned_top_pair_at_nine_cm_can_start_insertion(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "top_pair_only")
    fake.target_in_body = lambda: (0.097, 0.0, 0.0)
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.odom_position = (1.0, 2.0)
    fake.odom_yaw = 0.0
    fake.insertion_start_due_at = None
    fake.publish_status = lambda *_args, **_kwargs: None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_docking(fake)

    assert fake.insertion_start_due_at == pytest.approx(10.1)
    assert fake.insertion_entry_gap_m == pytest.approx(0.097)


def test_translation_first_alignment_limits_but_keeps_large_yaw_correction():
    fake = type("FakeDock", (), {})()
    candidate = {"depth_yaw": {"yaw_deg": 0.0}}
    pnp = {"yaw_deg": 0.0, "depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.update_world_target = lambda *_args, **_kwargs: True
    fake.target_in_body = lambda: (0.50, 0.08, math.radians(80.0))
    fake.config = {"translation_first_alignment_enabled": True}
    fake.number = lambda key, default, *_args: (
        1 if key == "alignment_yaw_confirmation_frames" else default
    )
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_docking(fake)

    assert commands == [(0.08, 0.064, 0.20)]


def test_translation_first_alignment_enforces_minimum_yaw_speed():
    fake = type("FakeDock", (), {})()
    candidate = {"depth_yaw": {"yaw_deg": 0.0}}
    pnp = {"yaw_deg": 0.0, "depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.update_world_target = lambda *_args, **_kwargs: True
    fake.target_in_body = lambda: (0.20, 0.0, math.radians(4.0))
    fake.config = {"translation_first_alignment_enabled": True}
    fake.number = lambda key, default, *_args: (
        1 if key == "alignment_yaw_confirmation_frames" else default
    )
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_docking(fake)

    assert commands == [(0.0, 0.0, 0.20)]


def test_nearest_odom_alignment_clamps_lateral_motion_above_dead_zone():
    fake = type("FakeDock", (), {})()
    fake.target_type = "NEAREST"
    fake.target_entity_id = 40
    fake.latest_detection = {"source_stamp_ns": 100}
    fake.target_in_body = lambda: (0.20, -0.036, 0.0)
    fake.config = {}
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_docking(fake)

    assert commands == [(0.0, -0.10, 0.0)]


def test_alignment_keeps_locked_target_when_measurements_disagree():
    fake = type("FakeDock", (), {})()
    candidate = {"depth_yaw": {"yaw_deg": 15.0}}
    pnp = {"yaw_deg": 20.0, "depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.config = {"translation_first_alignment_enabled": True}
    updated = []
    fake.update_world_target = lambda *args, **kwargs: updated.append(
        (args, kwargs)
    )
    fake.target_in_body = lambda: (0.30, 0.0, 0.0)
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_docking(fake)

    assert updated
    assert commands == [(pytest.approx(0.08), 0.0, 0.0)]


def test_lost_alignment_continues_toward_locked_target(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "not_visible")
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.config = {"translation_first_alignment_enabled": True}
    fake.target_in_body = lambda: (0.30, 0.0, 0.0)
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_docking(fake)

    assert commands == [(pytest.approx(0.08), 0.0, 0.0)]


def test_camera_target_is_converted_from_physical_to_calibrated_odom_units():
    fake = type("FakeDock", (), {})()
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.target_world = None
    fake.number = lambda key, default, *_args: {
        "distance_coefficient": 0.5,
        "lateral_coefficient": 0.25,
        "centerline_offset_cm": 0.0,
    }.get(key, default)
    candidate = {"depth_yaw": {}}
    pnp = {
        "distance_source": "pnp", "forward_distance_cm": 100.0,
        "lateral_ratio": -0.10, "yaw_deg": 0.0,
    }

    assert AutoDockNode.update_world_target(fake, candidate, pnp)
    assert fake.target_world["x"] == pytest.approx(2.0)
    assert fake.target_world["y"] == pytest.approx(0.4)


def test_world_target_uses_pnp_distance_when_depth_distance_is_missing():
    fake = type("FakeDock", (), {})()
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.target_world = None
    fake.number = lambda key, default, *_args: default
    candidate = {"depth_yaw": {"yaw_deg": 2.0}}
    pnp = {
        "distance_source": "depth",
        "forward_distance_cm": 30.0,
        "lateral_ratio": 0.0,
        "depth_fallback": True,
    }

    assert AutoDockNode.update_world_target(fake, candidate, pnp) is True
    assert fake.target_world["x"] == pytest.approx(0.30)


def test_world_target_rejects_measurement_when_all_distances_are_missing():
    fake = type("FakeDock", (), {})()
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.target_world = None
    fake.number = lambda key, default, *_args: default

    assert AutoDockNode.update_world_target(
        fake, {"depth_yaw": {}}, {"distance_source": "depth"}
    ) is False


def test_insertion_depth_is_measured_beyond_entry_gap_with_calibration():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.insert_start_yaw = 0.0
    fake.insertion_entry_gap_m = 0.20
    fake.odom_position = (0.50, 0.0)
    commands = []
    fake.number = lambda key, default, *_args: {
        "insertion_distance_cm": 25.0,
        "distance_coefficient": 0.8936,
        "insertion_speed_m_s": 0.08,
    }.get(key, default)
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.valid_measurement = lambda: (None, None, "not_visible")
    fake.target_in_body = lambda: (0.10, 0.0, 0.0)
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert commands == [(0.08, 0.0, 0.0)]


def test_insertion_uses_configured_speed_before_distance_is_reached():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    commands = []
    values = {"insertion_distance_cm": 15.0, "insertion_speed_m_s": 0.08}
    fake.number = lambda key, default, *_args: values.get(key, default)
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.valid_measurement = lambda: (None, None, "not_visible")
    fake.target_in_body = lambda: (0.10, 0.0, 0.0)
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert commands == [(0.08, 0.0, 0.0)]


def test_insertion_corrects_lateral_and_yaw_from_visible_target():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    candidate, pnp = {"entity_id": 7}, {"distance_source": "pnp"}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    updated = []
    fake.update_world_target = lambda got_candidate, got_pnp, blend_existing: (
        updated.append((got_candidate, got_pnp, blend_existing)) or True
    )
    fake.target_in_body = lambda: (0.10, 0.02, 0.10)
    fake.number = lambda key, default, *_args: {
        "insertion_distance_cm": 15.0,
        "insertion_speed_m_s": 0.08,
    }.get(key, default)
    fake.cancel = lambda reason: pytest.fail(reason)
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert updated == [(candidate, pnp, True)]
    assert commands == [(0.08, pytest.approx(0.01), pytest.approx(0.08))]


def test_insertion_keeps_correcting_from_visible_upper_tag_pair():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    candidate, pnp = {"tag_ids": [1, 2]}, {"distance_source": "pnp"}
    fake.valid_measurement = lambda: (None, None, "full_not_visible")
    fake.tracked_partial_measurement = lambda: (
        None, None, "tracked_partial_not_visible"
    )
    fake.visible_target_top_pair_measurement = lambda: (candidate, pnp, None)
    updated = []
    fake.update_world_target = lambda got_candidate, got_pnp, blend_existing: (
        updated.append((got_candidate, got_pnp, blend_existing)) or True
    )
    fake.target_in_body = lambda: (0.10, -0.04, -0.10)
    fake.number = lambda key, default, *_args: {
        "insertion_distance_cm": 15.0,
        "insertion_speed_m_s": 0.08,
    }.get(key, default)
    fake.cancel = lambda reason: pytest.fail(reason)
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert updated == [(candidate, pnp, True)]
    assert commands == [(0.08, pytest.approx(-0.02), pytest.approx(-0.08))]


def test_insertion_drives_straight_after_upper_tag_pair_disappears():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    fake.valid_measurement = lambda: (None, None, "full_not_visible")
    fake.tracked_partial_measurement = lambda: (
        None, None, "tracked_partial_not_visible"
    )
    fake.visible_target_top_pair_measurement = lambda: (
        None, None, "top_pair_not_visible"
    )
    fake.target_in_body = lambda: (0.10, 0.04, 0.10)
    fake.number = lambda key, default, *_args: {
        "insertion_distance_cm": 15.0,
        "insertion_speed_m_s": 0.08,
    }.get(key, default)
    fake.cancel = lambda reason: pytest.fail(reason)
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert commands == [(0.08, 0.0, 0.0)]


def test_nearest_centering_switches_to_stable_closer_visible_candidate(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.config = {}
    fake.number = lambda key, default, *_args: default
    fake.target_type = "NEAREST"
    fake.product_type = "NORMAL"
    fake.target_entity_id = 113
    fake.nearest_alignment_distance_cm = 28.0
    fake.target_world = {"x": 1.0}
    fake.latest_detection_at = 10.0
    fake.latest_detection = {"entities": [{
        "entity_id": 124,
        "seen_count": 2,
        "matrix": ["clover", "diamond", "clover", "diamond"],
        "image_pallet_box": [300, 200, 450, 280],
        "pnp": {
            "forward_distance_cm": 20.0,
            "lateral_ratio": 0.0,
            "reprojection_error_px": 1.0,
        },
        "visibility_score": 8000,
    }]}
    statuses = []
    fake.send_yolo_target = lambda: None
    fake.reset_coarse_alignment = lambda: None
    fake.update_world_target = lambda *_args, **_kwargs: setattr(
        fake, "target_world", {"x": 2.0}
    ) or True
    fake.publish_status = lambda *args, **kwargs: statuses.append((args, kwargs))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    assert AutoDockNode.maybe_switch_nearest_alignment_target(fake) is True
    assert fake.target_entity_id == 124
    assert fake.nearest_alignment_distance_cm == pytest.approx(20.0)
    assert fake.target_world == {"x": 2.0}
    assert statuses[-1][0] == ("running", "nearest_centering_target_switched")


def test_warning_tape_must_be_below_visible_pallet_box():
    fake = type("FakeDock", (), {})()
    fake.config = {"tape_require_below_pallet_enabled": True}
    fake.number = lambda key, default, *_args: fake.config.get(key, default)
    detection = {
        "entities": [{"image_pallet_box": [100, 120, 300, 300]}]
    }

    minimum = AutoDockNode.warning_tape_pallet_minimum_y_ratio(
        fake, detection, 480
    )

    assert minimum == pytest.approx(300 / 480 + 0.02)
    pallet_product_type,
