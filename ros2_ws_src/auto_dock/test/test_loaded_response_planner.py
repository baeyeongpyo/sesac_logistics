"""Mathematical/contract tests; these do not validate real vehicle predictions."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from auto_dock.loaded_response_planner import (
    ResponseCoefficients, ResponseState, advance_state, simulate_actions,
    plan_approach, insertion_check, estimate_history,
    StaticSettledCoefficients, plan_static_settled_approach,
    simulate_static_settled_actions,
)

# Explicit old candidate exercises delayed release; never a runtime default.
C = ResponseCoefficients(
    .26337, 1.18777, 5.55796, .11491, 1.14659, 28.7528,
    model_id='test-delayed-candidate', validated=False,
    left_immediate_gain=None, left_total_gain=None,
    right_immediate_gain=None, right_total_gain=None)


def known(**kw):
    return ResponseState(history_known=True, pending_bound_deg=0., **kw)


def test_production_delayed_response_defaults_match_settled_forward_release_fit():
    coefficients = ResponseCoefficients()
    assert coefficients.immediate_gain == .18565
    assert coefficients.total_gain == 1.50914
    assert coefficients.left_immediate_gain == .1587
    assert coefficients.left_total_gain == 1.401222
    assert coefficients.right_immediate_gain == .2126
    assert coefficients.right_total_gain == 1.617065
    assert coefficients.release_command_cm == 3.80
    assert coefficients.settling_sec == .001
    assert coefficients.forward_scale == .980
    assert coefficients.fork_offset_cm == 28.7528
    assert coefficients.model_id == \
        'vehicle1-loaded-w035-source-time-regression-20260906'


@pytest.mark.parametrize('wz,immediate_gain,release_gain', [
    (.35, .1587, .9092),
    (-.35, .2126, 1.0277),
])
def test_w035_regression_replays_turn_then_five_command_cm(
    wz, immediate_gain, release_gain,
):
    coefficients = ResponseCoefficients()
    turn = {'action': 'turn', 'drive': (0., 0., wz), 'duration_sec': .3}
    forward = {'action': 'forward', 'drive': (.10, 0., 0.),
               'duration_sec': .5}
    command_yaw = math.degrees(wz*.3)

    after_turn = simulate_actions(
        [turn], known(), coefficients, step_sec=.005)['state']
    after_forward = simulate_actions(
        [turn, forward], known(), coefficients, step_sec=.005)['state']

    assert after_turn.yaw_deg == pytest.approx(
        immediate_gain*command_yaw, abs=.02)
    assert after_forward.yaw_deg == pytest.approx(
        (immediate_gain+release_gain)*command_yaw, abs=.03)


def test_vehicle_config_selects_production_response_and_staging_only():
    path = Path(__file__).parents[3] / 'config' / 'vehicle_pose_config.json'
    config = json.loads(path.read_text())
    assert config['y_slot_response_model'] == {
        'immediate_gain': .18565,
        'total_gain': 1.50914,
        'left_immediate_gain': .1587,
        'left_total_gain': 1.401222,
        'right_immediate_gain': .2126,
        'right_total_gain': 1.617065,
        'release_command_cm': 3.80,
        'settling_sec': .001,
        'forward_scale': .980,
        'fork_offset_cm': 28.7528,
        'model_id': 'vehicle1-loaded-w035-source-time-regression-20260906',
        'validated': False,
    }
    assert config['y_slot_response_angular_speed_rad_s'] == .35
    assert config['y_slot_response_settle_sec'] == .50
    assert config['y_slot_execute_insertion'] is True
    assert config['y_slot_insertion_distance_cm'] == 32.


def test_stationary_never_erases_pending():
    state = known(pending_deg=12., equilibrium_deg=3.)
    stopped = advance_state(state, C, 0., 0., 5.)
    assert stopped.pending_deg == 12.
    assert stopped.yaw_deg == pytest.approx(3.)
    assert not advance_state(ResponseState(), C, 0., 0., 5.).history_known


def test_rotation_and_following_forward_match_recorded_candidate_law():
    turn = advance_state(known(), C, 0., math.radians(10), 1.)
    assert turn.equilibrium_deg == pytest.approx(C.immediate_gain*10)
    assert turn.pending_deg == pytest.approx((C.total_gain-C.immediate_gain)*10)
    forward = advance_state(turn, C, .1, 0., 1.)
    assert forward.pending_deg == pytest.approx(turn.pending_deg*math.exp(-10/C.release_command_cm))
    assert forward.yaw_deg > turn.yaw_deg
    assert forward.equilibrium_deg+forward.pending_deg == pytest.approx(C.total_gain*10)


def test_directional_yaw_gains_are_applied_by_command_sign():
    directional = ResponseCoefficients(
        left_immediate_gain=.5, left_total_gain=1.7,
        right_immediate_gain=.2, right_total_gain=1.0)
    left = advance_state(known(), directional, 0., math.radians(10), 1.)
    right = advance_state(known(), directional, 0., math.radians(-10), 1.)
    assert left.equilibrium_deg == pytest.approx(5.)
    assert left.pending_deg == pytest.approx(12.)
    assert right.equilibrium_deg == pytest.approx(-2.)
    assert right.pending_deg == pytest.approx(-8.)


def test_static_settled_plan_matches_all_three_stage_coordinates():
    coefficients = StaticSettledCoefficients()
    plan = plan_static_settled_approach(
        15., 80., 5., coefficients, final_forward_cm=5.)
    assert plan['accepted'], plan
    assert plan['predicted_stage'] == pytest.approx(plan['target_stage'], abs=1e-9)
    assert plan['stage_gap_cm'] == pytest.approx(40., abs=1e-9)
    assert plan['stage_lateral_cm'] == pytest.approx(0., abs=1e-9)
    assert plan['model']['right_yaw_gain'] == 1.14
    assert plan['model']['left_yaw_gain'] == 1.35
    assert all(action['drive'][0] >= 0 and action['drive'][1] == 0
               for action in plan['actions'])
    assert all('reverse' not in action['action']
               and 'insertion' not in action['action']
               and 'feedback' not in action['action']
               for action in plan['actions'])
    settles = [action for action in plan['actions']
               if action['action'].endswith('_settle')]
    assert len(settles) == 2
    assert all(action['duration_sec'] == 1.5 for action in settles)
    assert plan['actions'][-1]['action'] == 'short_forward'
    assert (plan['actions'][-1]['duration_sec']*coefficients.forward_scale*10
            == pytest.approx(5.))


def test_static_settled_forward_has_no_yaw_release():
    coefficients = StaticSettledCoefficients()
    actions = [
        dict(action='turn_right', drive=(0., 0., -.35), duration_sec=.5),
        dict(action='turn_right_settle', drive=(0., 0., 0.), duration_sec=1.5),
        dict(action='forward', drive=(.1, 0., 0.), duration_sec=2.),
    ]
    path = simulate_static_settled_actions(actions, coefficients)
    assert path[1, 2] == pytest.approx(path[2, 2])
    assert path[2, 2] == pytest.approx(path[3, 2])


def test_static_settled_planner_has_no_arbitrary_turn_or_distance_cap():
    plan = plan_static_settled_approach(
        150., 200., 80., final_forward_cm=7.)
    assert plan['accepted'], plan
    assert plan['predicted_stage'] == pytest.approx(plan['target_stage'], abs=1e-9)


def test_static_settled_planner_rejects_goal_behind_vehicle():
    plan = plan_static_settled_approach(
        0., 20., 0., staging_cm=40., final_forward_cm=5.)
    assert not plan['accepted']
    assert plan['reason'] == 'approach_needs_reverse_clearance'


def test_fork_moves_when_rotating_stationary():
    result = simulate_actions([dict(drive=(0.,0.,.35), duration_sec=1.)], known(), C)
    right, forward, yaw = result['path'][-1]
    assert right == pytest.approx(-C.fork_offset_cm*math.sin(math.radians(yaw)))
    assert forward == pytest.approx(C.fork_offset_cm*(math.cos(math.radians(yaw))-1))


def test_reverse_model_not_silently_forward_model():
    with pytest.raises(ValueError, match='reverse_response_unidentified'):
        advance_state(known(), C, -.1, 0., .1)


def test_aligned_image_rejected_when_tail_releases_pending():
    check = insertion_check(0., 40., 0., known(pending_deg=9.), C)
    assert not check['accepted']
    assert 'straight_tail_yaw_or_unreleased_response' in check['reasons']
    assert 'straight_tail_lateral_drift' in check['reasons']


@pytest.mark.parametrize('pose', [(0.,80.,0.), (14.8,81.1,1.3), (-10.,75.,-3.)])
def test_whole_approach_compensates_response_and_tail(pose):
    plan = plan_approach(*pose, known(), C)
    assert plan['accepted'], plan.get('insertion')
    assert plan['insertion']['max_tail_yaw_deg'] < 2.
    assert abs(plan['predicted_state']['pending_deg']) < .15
    assert plan['model']['validated'] is False
    assert plan['evaluations'] <= 3*24*7
    assert all(a['drive'][1] == 0 for a in plan['actions'])
    assert all(a['drive'][0] == 0 or a['drive'][0] >= .1 for a in plan['actions'])
    forward_indices = [index for index, action in enumerate(plan['actions'])
                       if action['action'] == 'forward']
    assert forward_indices
    assert all(plan['actions'][index+1] == {
        'action': 'settle_after_forward', 'drive': (0., 0., 0.),
        'duration_sec': .5} for index in forward_indices)
    assert sum(action['action'] == 'settle_after_forward'
               for action in plan['actions']) == len(forward_indices)
    replay = simulate_actions(plan['actions'], known(), C)
    assert np.allclose(replay['path'], plan['path'])
    assert replay['state'].yaw_deg == pytest.approx(
        plan['predicted_state']['yaw_deg'])


def test_large_initial_history_uncertainty_cannot_vanish_into_stage_pose():
    state = ResponseState(history_known=True, pending_bound_deg=10.)
    plan = plan_approach(0.,80.,0., state, C)
    assert not plan['accepted']
    assert any(not p['accepted'] for p in plan['history_scenarios'])


def test_unknown_history_cannot_be_certified_by_stationary_image():
    result = plan_approach(0.,80.,0., ResponseState(), C)
    assert not result['accepted']
    assert result['reason'] == 'history_unobservable'


def test_history_identifies_hidden_response_only_with_forward_excitation():
    times = np.arange(0., 4.01, .05)
    vx = np.where(times < 1., 0., .1)
    wz = np.where(times < .5, .35, 0.)
    state = known(equilibrium_deg=1.5,pending_deg=-4.)
    yaw = [0.]
    for i, dt in enumerate(np.diff(times)):
        state = advance_state(state,C,float(vx[i]),float(wz[i]),float(dt))
        yaw.append(state.yaw_deg)
    estimate = estimate_history(times,vx,wz,yaw,C)
    assert estimate['observable']
    assert estimate['initial_pending_deg'] == pytest.approx(-4.)
    assert estimate['initial_equilibrium_deg'] == pytest.approx(1.5)
    assert estimate['state'].pending_deg == pytest.approx(state.pending_deg)
    stopped = estimate_history(times,np.zeros(len(times)),wz,np.zeros(len(times)),C)
    assert not stopped['observable']


def test_history_unwraps_imu_crossing_180_degrees():
    times = np.arange(0.,4.01,.05)
    vx = np.full(len(times),.1)
    wz = np.where(times < 1.,.35,0.)
    state = known(pending_deg=3.)
    yaw = [179.]
    for i,dt in enumerate(np.diff(times)):
        state=advance_state(state,C,vx[i],wz[i],float(dt))
        yaw.append((179.+state.yaw_deg+180)%360-180)
    fit=estimate_history(times,vx,wz,yaw,C)
    assert fit['observable']
    assert fit['initial_pending_deg'] == pytest.approx(3., abs=1e-8)


def test_nonzero_inherited_response_changes_frozen_commands():
    plans = [plan_approach(10., 80., 0.,
             ResponseState(yaw_deg=130., equilibrium_deg=131., pending_deg=pending,
                           history_known=True, pending_bound_deg=.1), C)
             for pending in (-8., 8.)]
    assert all(p['accepted'] for p in plans)
    assert plans[0]['actions'] != plans[1]['actions']
    assert all(abs(p['predicted_stage'][2]) < .1 for p in plans)


def test_uncertainty_is_not_erased_by_forward_release():
    state = ResponseState(history_known=True, pending_bound_deg=3., pending_deg=5.)
    after = advance_state(state,C,.1,0.,10.)
    assert after.pending_deg < .001
    assert after.pending_bound_deg == 3.
