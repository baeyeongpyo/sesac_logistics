"""Stopped checkpoint tests using saved/synthetic values only."""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from y_place_midpoint import stopped_heading,heading_error,correction_once,split_insertion,remaining_insertion_cm


class MidpointTests(unittest.TestCase):
    def samples(self):
        return [[float(t),math.radians(10+.02*math.sin(t))] for t in np.arange(0,2.01,.02)]

    def test_requires_stop_time_fresh_dense_stable_samples(self):
        rows=self.samples()
        self.assertAlmostEqual(stopped_heading(rows,2.,0.),10.,delta=.03)
        self.assertIsNone(stopped_heading(rows,2.,1.5))
        self.assertIsNone(stopped_heading(rows,2.4,0.))
        self.assertIsNone(stopped_heading(rows[::10],2.,0.))
        moving=[[t,y+math.radians(5*t)] for t,y in rows]
        self.assertIsNone(stopped_heading(moving,2.,0.))

    def test_wraparound_does_not_reverse_correction(self):
        self.assertEqual(heading_error(-179,179),2.)
        rows=[[t,math.radians(179.99 if i%2 else -179.99)] for i,(t,_) in enumerate(self.samples())]
        self.assertIsNotNone(stopped_heading(rows,2.,0.))

    def test_deadband_single_pulse_cap_and_large_error(self):
        gains=SimpleNamespace(left_total_gain=1.1,right_total_gain=1.4,total_gain=1.)
        self.assertEqual(correction_once(1.,0.,gains)['actions'],[])
        for error in (-7.,7.):
            actions=correction_once(error,0.,gains)['actions']
            self.assertEqual(len(actions),1)
            self.assertLessEqual(actions[0]['duration_sec'],.15)
            self.assertEqual(actions[0]['drive'][:2],[0.,0.])
            self.assertEqual(math.copysign(1,actions[0]['drive'][2]),math.copysign(1,error))
        with self.assertRaises(RuntimeError):correction_once(9.,0.,gains)

    def test_split_preserves_lateral_trim_and_total_forward_time(self):
        actions=[dict(action='stage_lateral_center',drive=[0,.1,0],duration_sec=.2),
                 dict(action='measured_heading_trim',drive=[0,0,.35],duration_sec=.03),
                 dict(action='final_straight_insertion',drive=[.1,0,0],duration_sec=35/9.8)]
        first,last=split_insertion(actions,35.)
        self.assertEqual(first[:-1],actions[:-1])
        self.assertAlmostEqual(first[-1]['duration_sec'],15/9.8)
        self.assertAlmostEqual(last[0]['duration_sec'],20/9.8)
        self.assertEqual(actions[-1]['action'],'final_straight_insertion')
        with self.assertRaises(ValueError):split_insertion(actions,15.)

    def test_live_checkpoint_has_no_correction_retry_loop(self):
        tree=ast.parse(Path(__file__).with_name('y_place_square_topline_trial.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
               and n.func.id=='remaining_insertion_cm']
        self.assertEqual(len(calls),2)
        for loop in (n for n in ast.walk(tree) if isinstance(n,(ast.For,ast.While))):
            self.assertFalse(any(n is calls[0] for n in ast.walk(loop)))


    def test_initial_observations_reset_on_miss_and_accept_five(self):
        tree=ast.parse(Path(__file__).with_name('y_place_square_topline_trial.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='image')
        detection=dict(x_identity=dict(center_px=[320,240],roi_px=[200,150,200,200]))
        results=iter([detection,detection,None]+[detection]*5)
        scope=dict(np=np,time=time,detect_nearest_topline=lambda *args:next(results))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'image_callback','exec'),scope)
        node=SimpleNamespace(image_after_ns=-1,stale_images=0,get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=9_000_000_000)),frozen=False,last_stamp=None,reacquiring=False,anchor=None,lock_size=None,
                             observations=[],bridge=SimpleNamespace(imgmsg_to_cv2=lambda *args:np.zeros((2,2,3))))
        for i in range(8):
            msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=i,nanosec=0)))
            scope['image'](node,msg)
            if i==2:self.assertEqual(node.observations,[])
        self.assertEqual(len(node.observations),5)

    def test_delayed_pre_stop_frame_does_not_consume_reacquisition(self):
        tree=ast.parse(Path(__file__).with_name('y_place_square_topline_trial.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='image')
        seen=[]
        detection=dict(x_identity=dict(center_px=[1,1],roi_px=[0,0,2,2]))
        def detect(*args):seen.append(True);return detection
        scope=dict(np=np,time=time,detect_nearest_topline=detect)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'image_callback','exec'),scope)
        node=SimpleNamespace(image_after_ns=10_000_000_000,stale_images=0,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=12_000_000_000)),
            frozen=False,last_stamp=None,reacquiring=True,lock_size=None,observations=[],
            bridge=SimpleNamespace(imgmsg_to_cv2=lambda *args:np.zeros((2,2,3))))
        for second in (9,10):
            scope['image'](node,SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=second,nanosec=0))))
        self.assertFalse(node.frozen)
        self.assertEqual(seen,[])
        scope['image'](node,SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=11,nanosec=0))))
        self.assertTrue(node.frozen)
        self.assertEqual(len(node.observations),1)
        self.assertEqual(node.image_timing['discarded_stale_frames'],2)
        self.assertEqual(node.image_timing['source_age_sec'],1.)

    def test_remaining_uses_observed_distance_not_commanded_travel(self):
        for gap,expected in [(40.,35.),(25.,20.),(5.,0.),(3.,0.)]:
            pose=dict(top_center_cm=[2.,gap],inward_normal=[0.,1.])
            self.assertEqual(remaining_insertion_cm(pose,5.),expected)
        angle=math.radians(20.)
        n=np.array([-math.sin(angle),math.cos(angle)])
        self.assertAlmostEqual(remaining_insertion_cm(dict(top_center_cm=40*n,inward_normal=n),5.),35.)


if __name__=='__main__':unittest.main()
