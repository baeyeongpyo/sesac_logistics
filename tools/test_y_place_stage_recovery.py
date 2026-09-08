"""No ROS or vehicle execution; bounded recovery math and orchestration checks."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from y_place_stage_recovery import predicted_target,recovery_turn,recovery_advance


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.c=SimpleNamespace(left_total_gain=1.,right_total_gain=1.,total_gain=1.,
                               forward_scale=1.,fork_offset_cm=28.)

    def pose(self,right=0.,forward=50.,heading=0.):
        a=math.radians(heading)
        return dict(top_center_cm=[right,forward],heading_left_deg=heading,
                    inward_normal=[-math.sin(a),math.cos(a)],staging_cm=40.)

    def test_measured_yaw_and_fork_arc_preserve_original_target(self):
        result=predicted_target([0,60],[[0,0,0,.35],[1,0,0,0]],
                                [[0,0],[1,math.pi/2]],0,self.c)
        np.testing.assert_allclose(result,[88,-28],atol=1e-6)
        result=predicted_target([0,60],[[0,.1,0,0],[1,0,0,0]],[[0,0],[1,0]],0,self.c)
        np.testing.assert_allclose(result,[0,50],atol=1e-6)

    def test_recorded_twenty_degree_error_turns_right_only_with_cap(self):
        actions=recovery_turn(3.6102,23.9221,self.c)
        self.assertEqual(len(actions),1)
        self.assertEqual(actions[0]['drive'],[0,0,-.35])
        self.assertLessEqual(actions[0]['duration_sec'],1.)
        self.assertEqual(recovery_turn(0,1,self.c),[])
        with self.assertRaises(RuntimeError):recovery_turn(0,31,self.c)

    def test_advance_uses_fresh_distance_and_minimum_speed(self):
        actions=recovery_advance(self.pose(),[0,50],0,self.c)
        self.assertEqual(actions[0]['drive'],[.1,0,0])
        self.assertAlmostEqual(actions[0]['duration_sec'],1.)
        self.assertEqual(recovery_advance(self.pose(forward=40),[0,40],0,self.c),[])

    def test_neighbour_heading_and_distance_fail_closed(self):
        for pose,predicted,heading in [(self.pose(right=20),[0,50],0),
                (self.pose(),[0,50],6),(self.pose(heading=7),[0,50],0),
                (self.pose(forward=65),[0,65],0),(self.pose(forward=30),[0,30],0)]:
            with self.assertRaises(RuntimeError):recovery_advance(pose,predicted,heading,self.c)

    def test_visibility_recovery_no_longer_turns(self):
        tree=ast.parse(Path(__file__).with_name('y_place_square_topline_trial.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='recovery_turn']
        self.assertEqual(calls,[])


if __name__=='__main__':unittest.main()
