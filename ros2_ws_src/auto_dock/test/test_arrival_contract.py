import json
import math

import cv2
import numpy as np
import pytest

from auto_dock.auto_dock_node import (
    AutoDockNode,
    SlotSelector,
    SlotGridVision,
    ZoneOccupancy,
    detect_warning_tape,
    normalize_slot_id,
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

    tape = detect_warning_tape(frame)

    assert tape is not None
    assert 4.0 < tape["angle_deg"] < 8.0
    assert tape["component_count"] >= 5


def test_warning_tape_detector_rejects_blank_floor():
    frame = np.full((480, 640, 3), 130, dtype=np.uint8)

    assert detect_warning_tape(frame) is None


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
        fake, math.radians(-16.5), 0.20
    ) is True
    assert AutoDockNode.is_lidar_self_return(
        fake, math.radians(-16.5), 0.201
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
    assert fake.backoff_command == (0.0, -0.12)


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


def test_persistent_lidar_obstacle_stops_after_one_backoff(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.backoff_until = 10.0
    fake.backoff_direction = "left"
    fake.nearest_by_direction = {"left": (0.12, math.pi / 2.0, 0.20)}
    fake.stop_drive = lambda *_args: None
    cancelled = []
    fake.cancel = lambda reason: cancelled.append(reason)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

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


def test_clipped_slot_grid_reverses_slowly_for_camera_view(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.slot_grid_recovery_requested = True
    fake.slot_grid_recovery_start_position = (1.0, 2.0)
    fake.slot_grid_recovery_start_yaw = 0.0
    fake.odom_position = (1.0, 2.0)
    fake.odom_yaw = 0.0
    fake.scan_updated_at = 9.9
    fake.nearest_by_direction = {"rear": (math.inf, None, 0.20)}
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: pytest.fail("rear view recovery must move")
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_slot_grid_recovery(fake)

    assert commands == [(-0.04, 0.0, 0.0)]


@pytest.mark.parametrize("reason", [
    "slot_grid_not_found",
    "slot_grid_too_small",
    "slot_grid_corners",
    "slot_grid_clipped",
    "slot_grid_pose_reprojection",
])
def test_all_slot_view_failures_request_same_recovery(reason):
    assert AutoDockNode.slot_view_error_recoverable(reason) is True


def test_non_view_slot_error_does_not_request_motion():
    assert AutoDockNode.slot_view_error_recoverable("empty_image") is False


def test_clipped_slot_grid_does_not_reverse_into_rear_obstacle(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.slot_grid_recovery_requested = True
    fake.slot_grid_recovery_start_position = (1.0, 2.0)
    fake.slot_grid_recovery_start_yaw = 0.0
    fake.odom_position = (1.0, 2.0)
    fake.odom_yaw = 0.0
    fake.scan_updated_at = 9.9
    fake.nearest_by_direction = {"rear": (0.18, math.pi, 0.20)}
    fake.number = lambda key, default, *_args: default
    stopped = []
    fake.stop_drive = lambda *_args: stopped.append(True)
    fake.publish_drive = lambda *_args: pytest.fail("rear is blocked")
    fake.publish_status = lambda *_args, **_kwargs: None
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_slot_grid_recovery(fake)

    assert stopped == [True]


def test_slot_target_locks_pallet_front_face_in_world_coordinates():
    fake = type("FakeDock", (), {})()
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.number = lambda key, default, *_args: default
    fake.reset_alignment_recovery = lambda: None
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None

    locked = AutoDockNode.lock_slot_target(fake, {
        "forward_m": 1.0,
        "lateral_m": 0.20,
        "yaw_error_rad": 0.10,
    })

    assert locked is True
    assert fake.state == "docking"
    assert fake.slot_docking_active is True
    assert fake.target_world["x"] == pytest.approx(0.7925)
    assert fake.target_world["y"] == pytest.approx(0.20)
    assert fake.target_world["yaw"] == pytest.approx(0.10)


def test_slot_docking_uses_locked_pose_without_symbol_measurement():
    fake = type("FakeDock", (), {})()
    fake.slot_docking_active = True
    fake.valid_measurement = lambda: pytest.fail("slot docking must not use symbols")
    fake.target_in_body = lambda: (0.50, 0.10, 0.0)
    fake.config = {"translation_first_alignment_enabled": True}
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.odom_position = (0.0, 0.0)
    fake.odom_yaw = 0.0
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_docking(fake)

    assert commands == [(0.08, 0.08, 0.0)]


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
    fake.right_turn_clearance_available = lambda: (False, 0.30, 0.50)

    AutoDockNode.tick_reversing_after_lift(fake)

    assert fake.state == "ready"
    assert fake.post_lift_reverse_target_m is None
    assert len(fake.drive_ready_pub.messages) == 1
    assert isinstance(fake.drive_ready_pub.messages[0], Empty)


def test_reverse_starts_right_turn_when_swept_circle_is_clear():
    fake = type("FakeDock", (), {})()
    fake.post_lift_reverse_start = (0.0, 0.0)
    fake.post_lift_reverse_start_yaw = 0.0
    fake.post_lift_reverse_target_m = 0.27
    fake.odom_position = (-0.28, 0.0)
    fake.odom_yaw = 0.0
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.drive_ready_pub = CapturePublisher()
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.right_turn_clearance_available = lambda: (True, 0.70, 0.50)

    AutoDockNode.tick_reversing_after_lift(fake)

    assert fake.state == "turning_right_for_ready"
    assert fake.right_turn_target_yaw == pytest.approx(-math.pi / 2.0)
    assert fake.drive_ready_pub.messages == []


def test_reverse_waits_for_fresh_scan_before_skipping_turn(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.state = "reversing_after_lift"
    fake.post_lift_reverse_start = (0.0, 0.0)
    fake.post_lift_reverse_start_yaw = 0.0
    fake.post_lift_reverse_target_m = 0.27
    fake.odom_position = (-0.28, 0.0)
    fake.odom_yaw = 0.0
    fake.right_turn_clearance_wait_started_at = None
    fake.number = lambda key, default, *_args: default
    fake.stop_drive = lambda *_args: None
    fake.drive_ready_pub = CapturePublisher()
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.right_turn_clearance_available = lambda: (False, None, None)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_reversing_after_lift(fake)

    assert fake.state == "reversing_after_lift"
    assert fake.post_lift_reverse_target_m == pytest.approx(0.27)
    assert fake.right_turn_clearance_wait_started_at == 10.0
    assert fake.drive_ready_pub.messages == []


def test_right_turn_ignores_close_point_outside_actual_swept_footprint(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.scan_updated_at = 10.0
    fake.load_state = "LOADED"
    fake.scan_points = [(0.102, math.radians(-154.0))]
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    clear, blocking, _extent = AutoDockNode.right_turn_clearance_available(fake)

    assert clear is True
    assert math.isinf(blocking)


def test_right_turn_rejects_point_inside_actual_swept_footprint(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.scan_updated_at = 10.0
    fake.load_state = "LOADED"
    fake.scan_points = [(math.hypot(0.20, 0.20), math.radians(-45.0))]
    fake.number = lambda key, default, *_args: default
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    clear, blocking, _extent = AutoDockNode.right_turn_clearance_available(fake)

    assert clear is False
    assert blocking == pytest.approx(math.hypot(0.20, 0.20))


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
    }.get(key, default)
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.config = {
        "search_lateral_direction": "left",
        "tape_guidance_enabled": False,
    }
    fake.search_heading_yaw = None
    fake.odom_yaw = 0.0
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_search(fake)

    assert fake.candidate_stop_due_at == pytest.approx(10.2)
    assert commands == [(pytest.approx(0.02), 0.12, 0.0)]


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

    assert commands == [(0.0, 0.0, pytest.approx(-0.06))]


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
    fake.publish_status = lambda state, reason, **extra: fake.statuses.append(
        (state, reason, extra)
    )
    return fake


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
        "tape_guidance_enabled": True,
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


def test_removed_tape_guidance_cannot_rotate_search_motion(monkeypatch):
    fake = tape_search_fake(angle_deg=10.0)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands[0][0] == pytest.approx(0.02)
    assert fake.commands[0][1:] == (0.12, 0.0)
    assert fake.statuses[-1][1] == "lateral_target_search"


def test_removed_tape_guidance_cannot_change_lateral_search(monkeypatch):
    fake = tape_search_fake(angle_deg=1.0)
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.1)

    AutoDockNode.tick_search(fake)

    assert fake.commands[0][0] == pytest.approx(0.02)
    assert fake.commands[0][1:] == (0.12, 0.0)
    assert fake.statuses[-1][1] == "lateral_target_search"


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


def test_depth_fallback_enters_coarse_alignment_before_world_target_lock():
    fake = type("FakeDock", (), {})()
    entered = []
    fake.enter_coarse_alignment = lambda reason: entered.append(reason)
    fake.update_world_target = lambda *_args: pytest.fail("must not lock edge pose")

    result = AutoDockNode.enter_alignment(
        fake, {"center_error": 0.4}, {"depth_fallback": True}
    )

    assert result is True
    assert entered == ["pnp_quality_fallback"]


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


def test_partial_pair_from_different_entity_is_rejected(monkeypatch):
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

    assert candidate is None
    assert pnp is None
    assert reason == "partial_entity_mismatch"


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


def test_close_valid_target_pauses_before_insertion(monkeypatch):
    fake = type("FakeDock", (), {})()
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

    assert commands == [(0.08, 0.064, 0.12)]


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

    assert commands == [(0.0, 0.0, 0.10)]


def test_alignment_holds_when_yaw_sources_disagree():
    fake = type("FakeDock", (), {})()
    candidate = {"depth_yaw": {"yaw_deg": 15.0}}
    pnp = {"yaw_deg": 20.0, "depth_fallback": False}
    fake.valid_measurement = lambda: (candidate, pnp, None)
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.stop_drive = lambda *_args: stopped.append(True)
    fake.publish_status = lambda *args, **kwargs: statuses.append((args, kwargs))
    stopped = []
    statuses = []

    AutoDockNode.tick_docking(fake)

    assert stopped == [True]
    assert statuses[-1][0][1] == "alignment_yaw_disagreement_holding"


def test_lost_alignment_returns_toward_best_pose(monkeypatch):
    fake = type("FakeDock", (), {})()
    fake.valid_measurement = lambda: (None, None, "not_visible")
    fake.number = lambda key, default, *_args: default
    fake.insertion_start_due_at = None
    fake.alignment_lost_since = 9.0
    fake.alignment_best_pose = (0.0, 0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    fake.odom_yaw = 0.0
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None
    commands = []
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)

    AutoDockNode.tick_docking(fake)

    assert commands == [(-0.04, 0.0, 0.0)]


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
