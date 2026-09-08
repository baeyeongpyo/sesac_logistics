"""Safety boundaries between DOCK PICK and the loaded Y PLACE controller."""
import json
from types import SimpleNamespace

from std_msgs.msg import String

from auto_dock.auto_dock_node import AutoDockNode


def arrival_fake(load_state):
    statuses = []
    fake = SimpleNamespace(
        state="idle",
        mission_kind="NONE",
        operation="PICK",
        location="DOCK_1",
        product_type="NORMAL",
        target_left="ANY",
        target_right="ANY",
        load_state=load_state,
        config={},
        config_overrides={},
        slot_pending_depth_frame=None,
        slot_depth_frames=[],
        slot_rgb_observations=[],
        statuses=statuses,
    )
    fake.start_y_place = lambda: setattr(fake, "state", "y_slot_centering")
    fake.load_config = lambda: None
    fake.reset_coarse_alignment = lambda: None
    fake.latch_search_heading = lambda: None
    fake.send_yolo_target = lambda: None
    fake.boolean = lambda _key, default: default
    fake.number = lambda _key, default, *_args: default
    fake.publish_status = lambda *args, **kwargs: statuses.append((args, kwargs))
    return fake


def arrival(location, operation, target="NONE"):
    return String(data=json.dumps({
        "status": "SUCCEEDED",
        "location": location,
        "operation": operation,
        "product_type": "NORMAL",
        "target": {"type": target},
    }))


def test_dock_pick_does_not_gate_on_load_state():
    fake = arrival_fake("LOADED")

    AutoDockNode.on_trigger(fake, arrival("DOCK_1", "PICK", "NEAREST"))

    assert fake.state == "search"
    assert fake.mission_kind == "DOCK_PICK"
    assert fake.load_state == "LOADED"


def test_y_place_does_not_gate_on_load_state():
    fake = arrival_fake("UNLOADED")

    AutoDockNode.on_trigger(fake, arrival("Y", "PLACE"))

    assert fake.state == "y_slot_centering"
    assert fake.mission_kind == "Y_PLACE"
    assert fake.load_state == "UNLOADED"


def test_y_guidance_cannot_overwrite_dock_tape_guidance():
    dock_tape = {"center_y_ratio": 0.65, "angle_deg": 0.0}
    y_square = {"center_y_ratio": 0.91, "angle_deg": 3.0}
    fake = SimpleNamespace(
        mission_kind="Y_PLACE",
        state="y_slot_centering",
        latest_tape_guidance=dock_tape,
        latest_tape_guidance_at=9.0,
        y_slot_guidance=None,
        y_slot_guidance_at=0.0,
    )

    assert AutoDockNode.update_warning_tape_guidance(fake, y_square, 10.0)
    assert fake.latest_tape_guidance is dock_tape
    assert fake.latest_tape_guidance_at == 9.0
    assert fake.y_slot_guidance is y_square
    assert fake.y_slot_guidance_at == 10.0


def test_y_pick_is_fail_closed_without_mutating_active_mission():
    fake = arrival_fake("UNLOADED")
    commands = []
    fake.stop_drive = lambda *_args: commands.append("stop")

    AutoDockNode.on_trigger(fake, arrival("Y", "PICK"))

    assert (fake.state, fake.mission_kind, fake.load_state) == (
        "idle", "NONE", "UNLOADED",
    )
    assert (fake.operation, fake.location) == ("PICK", "DOCK_1")
    assert commands == []
    assert fake.statuses[-1][0][1] == "y_zone_requires_place"


def test_dock_place_is_fail_closed_without_mutating_active_mission():
    fake = arrival_fake("LOADED")
    commands = []
    fake.stop_drive = lambda *_args: commands.append("stop")

    AutoDockNode.on_trigger(fake, arrival("DOCK_1", "PLACE"))

    assert (fake.state, fake.mission_kind, fake.load_state) == (
        "idle", "NONE", "LOADED",
    )
    assert (fake.operation, fake.location) == ("PICK", "DOCK_1")
    assert commands == []
    assert fake.statuses[-1][0][1] == "dock_zone_requires_pick"


def test_supported_storage_slot_replaces_old_y_mission_kind():
    fake = arrival_fake("LOADED")
    fake.state = "ready"
    fake.mission_kind = "Y_PLACE"
    message = String(data=json.dumps({
        "status": "SUCCEEDED",
        "location": "NORMAL_1",
        "operation": "PLACE",
        "product_type": "NORMAL",
        "target": {"type": "SLOT", "slot_id": "R2C1"},
    }))

    AutoDockNode.on_trigger(fake, message)

    assert fake.state == "slot_target_ready"
    assert fake.mission_kind == "OTHER"
    assert fake.load_state == "LOADED"


def test_accepted_arrival_latches_an_immutable_mission_kind():
    dock = arrival_fake("UNLOADED")
    y = arrival_fake("LOADED")

    AutoDockNode.on_trigger(dock, arrival("DOCK_1", "PICK", "NEAREST"))
    AutoDockNode.on_trigger(y, arrival("Y", "PLACE"))

    assert dock.mission_kind == "DOCK_PICK"
    assert dock.state == "search"
    assert y.mission_kind == "Y_PLACE"
    assert y.state == "y_slot_centering"


def test_y_config_and_source_callbacks_are_rejected_during_dock_pick():
    statuses = []
    persisted = []
    fake = SimpleNamespace(
        mission_kind="DOCK_PICK", state="docking", config={},
        statuses=statuses,
        publish_status=lambda *args, **kwargs: statuses.append((args, kwargs)),
        persist_config=lambda: persisted.append(True),
    )

    AutoDockNode.on_y_slot_response_config(fake, String(data='{"forward_scale":1.0}'))
    AutoDockNode.on_y_slot_pose_source(fake, String(data="depth"))

    assert fake.config == {}
    assert persisted == []
    assert [row[0][1] for row in statuses] == [
        "y_slot_config_requires_y_place",
        "y_slot_config_requires_y_place",
    ]


def test_y_depth_callback_does_not_buffer_during_dock_pick():
    fake = SimpleNamespace(
        mission_kind="DOCK_PICK",
        slot_pending_depth_frame=None,
        slot_depth_frames=[],
        slot_rgb_observations=[],
    )
    message = SimpleNamespace(encoding="16UC1")

    AutoDockNode.on_slot_depth(fake, message)

    assert fake.slot_pending_depth_frame is None
    assert fake.slot_depth_frames == []


def test_generic_odom_updates_even_when_y_watermark_is_newer(monkeypatch):
    monkeypatch.setattr("auto_dock.auto_dock_node.time.monotonic", lambda: 10.0)
    fake = SimpleNamespace(
        mission_kind="DOCK_PICK",
        y_slot_last_odom_source_ns=999_000_000_000,
        y_slot_response_history=[],
        odom_position=None,
        odom_yaw=None,
    )
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.25, y=-0.5),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )),
    )

    AutoDockNode.on_odom(fake, message)

    assert fake.odom_received_at == 10.0
    assert fake.odom_position == (1.25, -0.5)
    assert fake.odom_yaw == 0.0
    assert fake.y_slot_response_history == []


def test_response_command_history_is_y_place_only():
    history = []
    fake = SimpleNamespace(
        mission_kind="DOCK_PICK", y_slot_command_history=history,
    )
    message = SimpleNamespace(
        linear=SimpleNamespace(x=0.1, y=0.0),
        angular=SimpleNamespace(z=0.2),
    )

    AutoDockNode.on_response_command(fake, message)

    assert history == []
