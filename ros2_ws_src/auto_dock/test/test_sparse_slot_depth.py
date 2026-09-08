"""Integrated test_y ignores legacy depth guidance; retained helper stays bounded."""
from collections import deque
from types import SimpleNamespace

import numpy as np

from auto_dock.auto_dock_node import AutoDockNode


def depth_message(stamp_ns=1_000_000_000):
    width, height, step = 640, 480, 1288
    raw = bytearray(step*height)
    image = np.ndarray((height, width), dtype='<u2', buffer=raw,
                       strides=(step, 2))
    image[:] = 540
    return SimpleNamespace(
        encoding='16UC1', width=width, height=height, step=step,
        is_bigendian=False, data=raw,
        header=SimpleNamespace(
            frame_id='rgb', stamp=SimpleNamespace(
                sec=stamp_ns//1_000_000_000,
                nanosec=stamp_ns%1_000_000_000)))


def observation(stamp_ns=1_000_000_000):
    return dict(rgb_stamp_ns=stamp_ns, rgb_frame_id='rgb', rgb_source_at=100.,
                image_width_px=640, image_height_px=480,
                square_top_line_px=[220., 240., 420., 240.])


def fake_node(observations=()):
    fake = SimpleNamespace(
        mission_kind='Y_PLACE',
        slot_depth_frames=deque(maxlen=8),
        slot_pending_depth_frame=None,
        slot_rgb_observations=deque(observations, maxlen=8),
        slot_camera_frame='rgb', slot_camera_size=(640, 480),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
    fake.consume_pending_slot_depth = lambda item: (
        AutoDockNode.consume_pending_slot_depth(fake, item))
    return fake


def test_test_y_ignores_depth_even_when_legacy_rgb_is_present(monkeypatch):
    import auto_dock.auto_dock_node as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    fake = fake_node([observation()])
    AutoDockNode.on_slot_depth(fake, depth_message())
    assert fake.slot_pending_depth_frame is None
    assert len(fake.slot_depth_frames) == 0


def test_test_y_does_not_queue_a_legacy_depth_frame(monkeypatch):
    import auto_dock.auto_dock_node as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    fake = fake_node()
    AutoDockNode.on_slot_depth(fake, depth_message())
    assert fake.slot_pending_depth_frame is None
    assert len(fake.slot_depth_frames) == 0


def test_stale_pending_frame_is_cleared(monkeypatch):
    import auto_dock.auto_dock_node as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    fake = fake_node()
    fake.slot_pending_depth_frame = dict(
        stamp_ns=1_000_000_000, frame_id='rgb', source_at=99.,
        width=640, height=480, step=1280, encoding='16UC1',
        is_bigendian=False, data=b'')
    assert not fake.consume_pending_slot_depth(observation())
    assert fake.slot_pending_depth_frame is None
