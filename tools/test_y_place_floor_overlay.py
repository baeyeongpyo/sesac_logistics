"""Pure rendering and extracted-method checks; never initialize the GUI/ROS."""
import ast
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
from y_place_floor_overlay import render_floor_overlay


class FloorOverlayTests(unittest.TestCase):
    def test_overlay_does_not_modify_input_or_unrelated_tag_pixels(self):
        frame=np.full((480,640,3),123,np.uint8)
        debug=dict(detection=None,selected=None,rejected=[],reason='no_x_candidate')
        result=render_floor_overlay(frame,debug,frame.shape)
        np.testing.assert_array_equal(frame,np.full_like(frame,123))
        np.testing.assert_array_equal(result[:450],frame[:450])

    def test_gui_uses_raw_frame_caches_and_clears_stale_result(self):
        p=Path(__file__).with_name('vehicle_camera_teleop_gui.py')
        tree=ast.parse(p.read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='render_warning_tape_overlay')
        raw=np.zeros((480,640,3),np.uint8)
        visible=np.full_like(raw,200)
        detect=Mock(return_value=dict(detection=None,selected=None,rejected=[],reason='no_x_candidate'))
        scope=dict(time=time,inspect_nearest_topline=detect,render_floor_overlay=render_floor_overlay)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(p),'exec'),scope)
        fake=SimpleNamespace(node=SimpleNamespace(auto_dock_status={},tape_frame=raw,tape_frame_monotonic=time.monotonic()),
                             args=SimpleNamespace(tape_image_topic='/raw'),y_slot_center_label=Mock(),
                             render_auto_dock_status_banner=Mock())
        for _ in range(2):scope['render_warning_tape_overlay'](fake,visible)
        self.assertEqual(detect.call_count,1)
        np.testing.assert_array_equal(detect.call_args.args[0],raw)
        fake.node.tape_frame_monotonic=time.monotonic()-2
        scope['render_warning_tape_overlay'](fake,visible)
        self.assertEqual(fake.warning_tape_debug['reason'],'raw_frame_stale')
        self.assertFalse(fake.warning_tape_debug['detected'])


if __name__=='__main__':unittest.main()
