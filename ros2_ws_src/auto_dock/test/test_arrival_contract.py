import json

import cv2
import numpy as np
import pytest

from auto_dock.auto_dock_node import (
    AutoDockNode,
    SlotSelector,
    SlotGridVision,
    ZoneOccupancy,
    normalize_slot_id,
    parse_arrival,
)
from std_msgs.msg import String


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


def test_legacy_arrival_remains_pick_compatible():
    arrival = parse_arrival("arrived heart heart")
    assert arrival["legacy"] is True
    assert arrival["operation"] == "PICK"


def test_fresh_auto_slot_priority_starts_at_r3c3():
    occupancy = ZoneOccupancy(confirmation_frames=1)
    occupancy.observe("FRESH", 3, 3, False)
    selected = SlotSelector().select("FRESH", "PLACE", occupancy.snapshot("FRESH"))
    assert selected == "FRESH_R3_C3"


def test_slot_id_uses_arrival_zone():
    assert normalize_slot_id("NORMAL", "r2c1") == "NORMAL_R2_C1"
    with pytest.raises(ValueError, match="arrival_slot_zone_mismatch"):
        normalize_slot_id("NORMAL", "FRESH_R2_C1")


def test_green_grid_marks_brown_cell_occupied():
    frame = np.full((600, 600, 3), 210, dtype=np.uint8)
    green = (0, 180, 0)
    cv2.rectangle(frame, (60, 60), (540, 540), green, 18)
    for coordinate in (220, 380):
        cv2.line(frame, (coordinate, 60), (coordinate, 540), green, 18)
        cv2.line(frame, (60, coordinate), (540, coordinate), green, 18)
    cv2.rectangle(frame, (90, 90), (190, 190), (35, 90, 145), -1)

    observations, error = SlotGridVision().analyze(frame, "NORMAL")

    assert error is None
    assert observations["NORMAL_R3_C1"]["occupied"] is True
    assert observations["NORMAL_R3_C2"]["occupied"] is False
    assert observations["NORMAL_R1_C3"]["occupied"] is False


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.mark.parametrize("operation,expected", [("PICK", "UP"), ("PLACE", "DOWN")])
def test_insertion_completion_sends_operation_command(operation, expected):
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (1.0, 0.0)
    fake.operation = operation
    fake.arrival_is_legacy = False
    fake.fork_pub = CapturePublisher()
    fake.entry_complete_pub = CapturePublisher()
    fake.stop_drive = lambda *_args: None
    fake.number = lambda *_args: 12.0
    fake.publish_status = lambda *_args, **_kwargs: None

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
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.publish_status = lambda *_args, **_kwargs: None

    AutoDockNode.finish_fork_operation(fake, fork_state)

    assert fake.load_state == expected_load
    assert fake.post_lift_reverse_start == (1.0, 2.0)
    assert fake.state == "reversing_after_lift"


def test_search_locks_virtual_target_before_stopping():
    fake = type("FakeDock", (), {})()
    candidate = {"tag_id": 7}
    pnp = {"forward": 0.6, "lateral": 0.1, "yaw": 0.0}
    fake.candidate_stop_due_at = 0.0
    fake.valid_measurement = lambda: (candidate, pnp, "ok")
    fake.update_world_target = lambda got_candidate, got_pnp: (
        got_candidate == candidate and got_pnp == pnp
    )
    fake.boolean = lambda key, default: False if key == "use_nav_approach" else default
    fake.stop_drive = lambda *_args: None
    fake.publish_status = lambda *_args, **_kwargs: None
    fake.nav_approach_completed = True

    AutoDockNode.tick_search(fake)

    assert fake.state == "docking"
    assert fake.nav_approach_completed is False


def test_insertion_uses_configured_speed_before_distance_is_reached():
    fake = type("FakeDock", (), {})()
    fake.insert_start_position = (0.0, 0.0)
    fake.odom_position = (0.05, 0.0)
    commands = []
    values = {"insertion_distance_cm": 15.0, "insertion_speed_m_s": 0.08}
    fake.number = lambda key, *_args: values[key]
    fake.cancel = lambda reason: pytest.fail(reason)
    fake.publish_drive = lambda x, y, yaw: commands.append((x, y, yaw))

    AutoDockNode.tick_inserting(fake)

    assert commands == [(0.08, 0.0, 0.0)]
