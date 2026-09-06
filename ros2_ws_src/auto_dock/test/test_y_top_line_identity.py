"""Moving a selected slot must not make its detector select the old-pixel stripe."""
import cv2
import numpy as np

from auto_dock.auto_dock_node import YTopLineTracker


def stripe_frame(*rows):
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    for y in rows:
        cv2.rectangle(frame, (200, y), (400, y + 20), (5, 5, 5), -1)
        for x in range(200, 400, 40):
            cv2.rectangle(frame, (x, y), (x + 20, y + 20), (0, 220, 220), -1)
    return frame


def slot(sequence, dy=0):
    result = dict(x_min_px=200, x_max_px=400, center_x_px=300,
                  center_x_ratio=300/640, center_y_ratio=(300+dy)/480,
                  image_width_px=640, image_height_px=480,
                  x_lines=[[200, 230+dy, 400, 350+dy], [200, 350+dy, 400, 230+dy]],
                  border_lines=[[200, 200+dy, 400, 200+dy]], border_angle_deg=0.,
                  tracking_sequence=sequence)
    if sequence > 1:
        result['tracking_transform'] = [[1., 0., 0.], [0., 1., dy], [0., 0., 1.]]
    return result


def line_y(line):
    assert line is not None
    return sum(line['top_line_px'][1::2])/2


def test_upper_endpoints_are_exact_and_order_independent(monkeypatch):
    import auto_dock.auto_dock_node as module
    def forbidden(*args,**kwargs):
        raise AssertionError('must not search for a separate stripe')
    monkeypatch.setattr(module,'detect_y_top_line',forbidden)
    anchor=slot(1)
    anchor['x_lines']=[[400,350,200,220],[400,235,200,350]]
    found=YTopLineTracker().update(stripe_frame(190),anchor)
    assert found['top_line_px']==[200.,220.,400.,235.]


def test_new_x_positions_move_line_without_transform_double_application():
    tracker=YTopLineTracker()
    assert line_y(tracker.update(stripe_frame(200),slot(1)))==230.
    moved=slot(2,70)
    found=tracker.update(stripe_frame(200),moved)
    assert line_y(found)==300.
    assert tracker.update(stripe_frame(200),moved) is None
    assert tracker.update(stripe_frame(200),None) is None


def test_fresh_x_does_not_require_separate_visible_stripe():
    tracker=YTopLineTracker()
    assert line_y(tracker.update(stripe_frame(),slot(1)))==230.
    assert line_y(tracker.update(stripe_frame(),slot(2,50)))==280.


def test_invalid_or_offscreen_upper_endpoints_do_not_become_depth_pixels():
    anchor=slot(1);anchor['x_lines']=[[0,0,400,350],[200,350,400,230]]
    assert YTopLineTracker().update(stripe_frame(),anchor) is None
    anchor['x_lines']=[[float('nan'),230,400,350],[200,350,400,230]]
    assert YTopLineTracker().update(stripe_frame(),anchor) is None
