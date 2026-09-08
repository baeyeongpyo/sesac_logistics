"""Offline geometry, frozen-path and response regression tests; no ROS."""
import copy
import math
import unittest

import cv2
import numpy as np

from y_place_square_geometry import detect_square, square_pose
from y_place_square_motion import (
    ResponseCoefficients, ResponseState, frozen_approach, simulate_actions,
    measured_response, final_insertion, loaded_visual_plan, turn_target_reached,
)


class GeometryTests(unittest.TestCase):
    def observation(self, heading):
        a = math.radians(heading)
        tangent = np.array([math.cos(a),math.sin(a)])
        normal = np.array([-math.sin(a),math.cos(a)])
        top = np.array([8.,75.])
        floor = np.array([top-10*tangent,top+10*tangent,
                          top-20*normal+10*tangent,top-20*normal-10*tangent])
        h = np.array([[.2,0,-64],[0,-.2,110],[0,0,1.]])
        pixel = np.column_stack((floor,np.ones(4)))@np.linalg.inv(h).T
        return dict(image_size=[640,480],outer_corners_px=pixel[:,:2].tolist()),dict(image_size=[640,480],image_to_floor_h=h.tolist())

    def test_both_side_poses_and_exact_normal_offset(self):
        for heading in (-30.,0.,30.):
            observation,profile = self.observation(heading)
            pose = square_pose(observation,profile)
            self.assertAlmostEqual(pose['heading_left_deg'],heading,places=9)
            delta = np.array(pose['top_center_cm'])-pose['stage_right_forward_cm']
            self.assertAlmostEqual(np.linalg.norm(delta),40.,places=9)
            self.assertAlmostEqual(pose['axis_disagreement_deg'],0.,places=5)

    def test_bottom_edge_really_changes_heading(self):
        observation,profile = self.observation(0.)
        original = square_pose(observation,profile)
        observation['outer_corners_px'][2][1] -= 15
        changed = square_pose(observation,profile)
        self.assertGreater(abs(changed['heading_left_deg']-original['heading_left_deg']),1.)

    def test_frozen_approach_reaches_target_and_detaches_inputs(self):
        for heading in (-20.,0.,20.):
            observation,profile = self.observation(heading)
            pose = square_pose(observation,profile)
            plan = frozen_approach(pose,ResponseCoefficients())
            self.assertTrue(plan['accepted'],plan['reason'])
            np.testing.assert_allclose(plan['predicted_stage'][:2],plan['target_stage'][:2],atol=1.)
            self.assertLess(abs(plan['predicted_stage'][2]-heading),2.)
            frozen = copy.deepcopy(plan)
            pose['top_center_cm'][0] = 1000
            self.assertEqual(plan,frozen)
            for a in plan['actions']:
                self.assertTrue(a['drive'][0]==0 or abs(a['drive'][0])>=.10)

    def test_x_alone_is_not_a_physical_square(self):
        frame = np.full((480,640,3),180,np.uint8)
        cv2.line(frame,(220,210),(420,330),(0,220,230),18)
        cv2.line(frame,(220,330),(420,210),(0,220,230),18)
        self.assertIsNone(detect_square(frame))

    def test_calibration_resolution_mismatch(self):
        observation,profile = self.observation(0.)
        profile['image_size'] = [1280,720]
        with self.assertRaisesRegex(ValueError,'image_mismatch'):
            square_pose(observation,profile)

    def test_side_solver_handles_heading_outside_node_limit(self):
        observation,profile=self.observation(50.)
        pose=square_pose(observation,profile)
        plan=frozen_approach(pose,ResponseCoefficients())
        self.assertTrue(plan['accepted'],plan['reason'])
        np.testing.assert_allclose(plan['predicted_stage'],plan['target_stage'],atol=.25)


class ResponseTests(unittest.TestCase):
    def test_measured_directional_response_and_final_trim(self):
        coefficients = ResponseCoefficients()
        actions = [dict(drive=drive,duration_sec=duration) for drive,duration in
                   [((0,0,.35),.8),((.1,0,0),3.),((0,0,-.35),.7),((.1,0,0),3.),((0,0,0),.2)]]
        sim = simulate_actions(actions,ResponseState(history_known=True,pending_bound_deg=0.),coefficients,step_sec=.02)
        commands=[]
        t=0.
        for action in actions:
            commands.append((t,*action['drive']))
            t+=action['duration_sec']
        commands.append((t,0,0,0))
        imu = np.column_stack((sim['seconds'],np.radians(sim['path'][:,2])))
        fitted,state,evidence = measured_response(commands,imu,coefficients)
        for direction in ('left','right'):
            self.assertEqual(evidence[direction]['source'],'approach_imu')
            self.assertAlmostEqual(getattr(fitted,direction+'_total_gain'),getattr(coefficients,direction+'_total_gain'),delta=.02)
        target = state.yaw_deg+3.
        insertion = final_insertion(target,state,fitted)
        self.assertAlmostEqual(insertion['predicted_final_heading_left_deg'],target,delta=.02)

    def test_lateral_history_replays_yaw_without_forward_simulator_crash(self):
        commands=[(0,0,0,0),(.1,0,-.1,0),(.5,0,0,0),(1.,.1,0,0),(2.,0,0,0)]
        imu=[(i/10,math.radians(i/10)) for i in range(21)]
        _,state,evidence=measured_response(commands,imu,ResponseCoefficients())
        self.assertAlmostEqual(state.yaw_deg,2.)
        self.assertAlmostEqual(evidence['yaw_replay']['lateral_duration_sec'],.4)

    def test_loaded_visual_centering_never_commands_lateral(self):
        for x in (-5.,5.):
            pose=dict(top_center_cm=[x,40.],inward_normal=[0.,1.],heading_left_deg=0.)
            plan=loaded_visual_plan(pose,ResponseState(history_known=True,pending_bound_deg=0.),ResponseCoefficients(),15.)
            self.assertTrue(plan['accepted'])
            np.testing.assert_allclose(plan['predicted_stage'][:2],[x-2.,15.],atol=.25)
            for action in plan['actions']:
                self.assertEqual(action['drive'][1],0.)
                self.assertGreaterEqual(action['drive'][0],0.)

    def test_close_start_skips_40cm_approach_and_stationary_history_is_valid(self):
        pose=dict(top_center_cm=[-10.3735933887,28.3252979028],
                  inward_normal=[-.24596338247,.96927912104],heading_left_deg=14.23877,
                  staging_cm=40.,stage_right_forward_cm=[-.535,-10.446])
        plan=frozen_approach(pose,ResponseCoefficients())
        self.assertTrue(plan['accepted'])
        self.assertEqual(plan['actions'],[])
        self.assertEqual(plan['next_phase'],'stage_reacquisition')
        _,state,g=measured_response([(0,0,0,0),(1,0,0,0)],[(0,.2),(.5,.2),(1,.2)],ResponseCoefficients())
        self.assertAlmostEqual(state.yaw_deg,0.)
        self.assertEqual(g['left']['source'],'saved_model')

    def test_short_pulse_does_not_replace_sustained_immediate_gain(self):
        c=ResponseCoefficients()
        commands=[(0,0,0,0),(.1,0,0,.35),(.25,.1,0,0),(2.,0,0,0)]
        imu=[(0,0),(.1,0),(.25,math.radians(.07)),(1.,math.radians(2.)),(2.,math.radians(3.))]
        fitted,_,evidence=measured_response(commands,imu,c)
        self.assertEqual(fitted.left_immediate_gain,c.left_immediate_gain)
        self.assertEqual(evidence['left']['short_pulses_using_saved_immediate'],1)

    def test_planned_turn_ends_at_measured_yaw_in_both_directions(self):
        for sign in (-1.,1.):
            self.assertFalse(turn_target_reached(0.,math.radians(sign*4.),sign*.35,sign*5.))
            self.assertTrue(turn_target_reached(0.,math.radians(sign*5.1),sign*.35,sign*5.))
            self.assertFalse(turn_target_reached(0.,math.radians(-sign*6.),sign*.35,sign*5.))
        self.assertTrue(turn_target_reached(math.radians(179),math.radians(-175),.35,5.))

    def test_no_history_is_not_called_live_gain(self):
        with self.assertRaisesRegex(ValueError,'history_required'):
            measured_response([],[],ResponseCoefficients())

    def test_history_ending_in_reverse_does_not_invent_response(self):
        with self.assertRaisesRegex(ValueError,'after_reverse'):
            measured_response([(0,0,0,0),(1,-.1,0,0)],[(0,0),(1,0)],ResponseCoefficients())


if __name__ == '__main__':
    unittest.main()
