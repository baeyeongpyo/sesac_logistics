"""Pure offline tests: no ROS initialization or vehicle execution."""
import math
import time
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import cv2

from y_place_square_geometry import detect_nearest_topline, topline_pose, warning_centerline, detect_locked_topline
from y_place_square_motion import ResponseCoefficients, ResponseState, stage_reacquired_insertion


class ReacquisitionTests(unittest.TestCase):
    def test_live_callback_attempts_reacquisition_only_once_even_when_missing(self):
        # Extract only the callback; never import or initialize a ROS runtime.
        source = Path(__file__).with_name('y_place_square_topline_trial.py')
        tree = ast.parse(source.read_text())
        callback = next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='image')
        with patch('y_place_square_geometry.detect_nearest_topline',return_value=None) as detect:
            scope = dict(np=np,time=time,detect_nearest_topline=detect)
            exec(compile(ast.Module(body=[callback],type_ignores=[]),str(source),'exec'),scope)
            fake = SimpleNamespace(image_after_ns=0,stale_images=0,
                                   get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=4_000_000_000)),frozen=False,reacquiring=True,last_stamp=None,
                                   bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a: np.zeros((480,640,3),np.uint8)))
            for stamp in (1,2,3):
                msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0)))
                scope['image'](fake,msg)
            self.assertEqual(detect.call_count,1)
            self.assertTrue(fake.frozen)

    def test_confirmed_slot_survives_cropped_x(self):
        import json
        local=Path(__file__).resolve().parents[1]/'analysis/review_20260907_045600/test_y_20260907_045604_8'
        fixture=Path(__file__).parent
        if not (fixture/'locked_reference.png').exists():
            ref=local/'stage_reacquisition_raw.png'; current=local/'midpoint_reacquisition_raw.png'; meta=local/'stage_reacquisition.json'
        else:
            ref=fixture/'locked_reference.png'; current=fixture/'locked_current.png'; meta=fixture/'locked_reference.json'
        if not ref.exists():self.skipTest('raw fixture unavailable')
        f=cv2.imread(str(current));r=cv2.imread(str(ref));d=json.loads(meta.read_text())['detection']
        result=detect_locked_topline(f,r,d)
        self.assertIsNotNone(result)
        self.assertEqual(result['identity_source'],'tracked_confirmed_floor')
        np.testing.assert_allclose(result['top_edge_px'],[[341,386],[601,416]],atol=10)
        self.assertIsNone(detect_locked_topline(np.full_like(f,180),r,d))
        self.assertIsNone(detect_locked_topline(f,None,None))

    def pose(self, right, angle):
        a = math.radians(angle)
        n = np.array([-math.sin(a), math.cos(a)])
        return dict(top_center_cm=[right,40.], inward_normal=n.tolist(),
                    heading_left_deg=angle, staging_cm=40.)

    def test_nearest_x_selected_before_border_validation(self):
        frame = np.zeros((480,640,3),np.uint8)
        identities=[dict(center_px=[50,150],roi_px=[0,100,101,100]),
                    dict(center_px=[320,240],roi_px=[270,200,101,100])]
        with patch('y_place_square_geometry.x_candidates',return_value=identities), \
             patch('y_place_square_geometry.warning_centerline',side_effect=lambda f,d:d) as band, \
             patch('y_place_square_geometry.side_intersections',side_effect=lambda f,d:d):
            self.assertEqual(detect_nearest_topline(frame)['x_identity'],identities[1])
            self.assertEqual(band.call_count,1)

    def test_missing_nearest_border_never_falls_back_to_neighbour(self):
        frame = np.zeros((480,640,3),np.uint8)
        identities=[dict(center_px=[520,160],roi_px=[440,140,190,60]),
                    dict(center_px=[350,185],roi_px=[270,150,180,70])]
        for missing_top in (False,True):
            with patch('y_place_square_geometry.x_candidates',return_value=identities), \
                 patch('y_place_square_geometry.warning_centerline',
                       side_effect=lambda f,d:None if missing_top else d) as band, \
                 patch('y_place_square_geometry.side_intersections',return_value=None) as sides:
                self.assertIsNone(detect_nearest_topline(frame))
                self.assertEqual(band.call_args_list[0].args[1]['x_identity'],identities[1])
                self.assertEqual(band.call_count,2 if missing_top else 1)
                self.assertEqual(sides.call_count,0 if missing_top else 1)

    def test_nonfloor_yellow_object_does_not_lock_out_valid_slot(self):
        frame=np.zeros((480,640,3),np.uint8)
        part=dict(center_px=[231,176],roi_px=[204,158,67,27])
        slot=dict(center_px=[417,343],roi_px=[275,272,265,152])
        with patch('y_place_square_geometry.x_candidates',return_value=[part,slot]), \
             patch('y_place_square_geometry.warning_centerline',
                   side_effect=lambda f,d:None if d['x_identity']==part else d), \
             patch('y_place_square_geometry.side_intersections',side_effect=lambda f,d:d):
            self.assertEqual(detect_nearest_topline(frame)['x_identity'],slot)

    def test_missing_side_does_not_invent_intersection(self):
        frame = np.full((480,640,3),180,np.uint8)
        cv2.line(frame,(390,315),(710,505),(0,220,220),18)
        cv2.line(frame,(710,315),(390,505),(0,220,220),18)
        for x in range(390,640,60):
            cv2.rectangle(frame,(x,285),(x+25,293),(0,0,220),-1)
        for x in range(390,640,40):
            cv2.rectangle(frame,(x,294),(x+19,306),(0,220,220),-1)
            cv2.rectangle(frame,(x+20,294),(x+39,306),(0,0,0),-1)
        with patch('y_place_square_geometry._detect_selected_square',
                   side_effect=AssertionError('stage must not require a closed quad')):
            d = detect_nearest_topline(frame)
        self.assertIsNone(d)

    def test_warning_centre_replaces_skewed_quad_edge(self):
        frame = np.full((480,640,3),180,np.uint8)
        for x in range(200,480,60):
            cv2.rectangle(frame,(x,270),(x+25,278),(0,0,220),-1)
        for x in range(200,480,40):
            cv2.rectangle(frame,(x,279),(x+19,290),(0,220,220),-1)
            cv2.rectangle(frame,(x+20,279),(x+39,290),(0,0,0),-1)
        d = warning_centerline(frame,dict(top_edge_px=[[200,290],[480,280]]))
        self.assertIsNotNone(d)
        line = np.asarray(d['top_edge_px'])
        self.assertLess(abs(line[1,1]-line[0,1]),1.)
        self.assertTrue(279<line[:,1].mean()<290)
        self.assertEqual(d['border_source'],'warning_tape_centerline')

    def test_no_detection(self):
        self.assertIsNone(detect_nearest_topline(np.full((480,640,3),180,np.uint8)))

    def test_topline_only_pose(self):
        profile = dict(image_size=[640,480],image_to_floor_h=[[.2,0,-64],[0,-.2,110],[0,0,1]])
        d = dict(image_size=[640,480],top_edge_px=[[280,350],[380,350]])
        p = topline_pose(d,profile)
        np.testing.assert_allclose(p['top_center_cm'],[2,40])
        self.assertAlmostEqual(p['heading_left_deg'],0)

    def test_left_right_sign_and_zero(self):
        for right in (-5.,0.,5.):
            p = stage_reacquired_insertion(self.pose(right,0),ResponseState(),ResponseCoefficients())
            self.assertAlmostEqual(p['lateral_right_cm'],right)
            lateral = [a for a in p['actions'] if a['action']=='stage_lateral_center']
            self.assertEqual(bool(lateral),right!=0)
            if lateral:
                self.assertAlmostEqual(lateral[0]['drive'][1],-math.copysign(.1,right))
                self.assertAlmostEqual(lateral[0]['duration_sec'],.5)
            self.assertEqual(p['distance_cm'],38)
            self.assertTrue(p['frozen'])

    def test_current_frame_rebase_and_fork_arc(self):
        for angle in (-8.,8.):
            pose = self.pose(3,angle)
            state = ResponseState(yaw_deg=25.,equilibrium_deg=27.,pending_deg=2.)
            plan = stage_reacquired_insertion(pose,state,ResponseCoefficients())
            n = np.asarray(pose['inward_normal']); t = np.array([n[1],-n[0]])
            final = np.asarray(plan['path'][-1][:2])+[plan['lateral_right_cm'],0]
            self.assertAlmostEqual(float((np.asarray(pose['top_center_cm'])-final)@t),0,places=7)
            self.assertAlmostEqual(plan['predicted_final_heading_left_deg'],angle,places=7)
            self.assertEqual(plan['response_state']['yaw_deg'],0)
            for action in plan['actions']:
                for v in action['drive'][:2]:
                    self.assertTrue(v==0 or abs(v)>=.1)

    def test_left_offset_changes_only_lateral_translation(self):
        for right in (-5.,0.,5.):
            for angle in (-8.,0.,8.):
                args=(self.pose(right,angle),ResponseState(),ResponseCoefficients())
                base=stage_reacquired_insertion(*args,distance_cm=35.)
                shifted=stage_reacquired_insertion(*args,distance_cm=35.,center_offset_left_cm=2.)
                self.assertAlmostEqual(shifted['lateral_right_cm'],base['lateral_right_cm']-2.)
                self.assertEqual(shifted['path'],base['path'])
                self.assertEqual(shifted['target_heading_left_deg'],base['target_heading_left_deg'])
                trim=lambda p:[a for a in p['actions'] if a['action'] in
                               ('measured_heading_trim','final_straight_insertion')]
                self.assertEqual(trim(shifted),trim(base))


if __name__ == '__main__':
    unittest.main()
