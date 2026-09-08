"""Offline candidate ordering checks; no ROS initialization."""
import numpy as np
from auto_dock import y_place_square_geometry as geometry


def test_initial_left_slot_wins_over_camera_center(monkeypatch):
    left = dict(center_px=[100, 100], roi_px=[70, 70, 60, 60])
    right = dict(center_px=[320, 240], roi_px=[290, 210, 60, 60])
    monkeypatch.setattr(geometry, 'x_candidates', lambda _: [right, left])
    monkeypatch.setattr(geometry, 'warning_centerline', lambda _, d, **kw: d)
    monkeypatch.setattr(geometry, 'side_intersections', lambda _, d, **kw: d)
    frame = np.zeros((480, 640, 3), np.uint8)
    assert geometry.detect_nearest_topline(frame, prefer_left=True)['x_identity'] == left
    assert geometry.detect_nearest_topline(frame)['x_identity'] == right
    assert geometry.detect_nearest_topline(frame, anchor_px=[100, 100])['x_identity'] == left


def test_left_slot_missing_sides_does_not_switch_right(monkeypatch):
    left = dict(center_px=[100, 100], roi_px=[70, 70, 60, 60])
    right = dict(center_px=[320, 240], roi_px=[290, 210, 60, 60])
    monkeypatch.setattr(geometry, 'x_candidates', lambda _: [right, left])
    monkeypatch.setattr(geometry, 'warning_centerline', lambda _, d, **kw: d)
    monkeypatch.setattr(geometry, 'side_intersections', lambda _, d, **kw: None if d['x_identity']==left else d)
    result = geometry.inspect_nearest_topline(np.zeros((480,640,3),np.uint8),prefer_left=True)
    assert result['selected'] == left and result['detection'] is None
