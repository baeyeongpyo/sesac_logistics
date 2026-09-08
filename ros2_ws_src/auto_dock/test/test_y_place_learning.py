"""Offline telemetry and persistence tests. Never initializes ROS or hardware."""
from dataclasses import replace
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from auto_dock.loaded_response_planner import ResponseCoefficients, ResponseState, advance_state
from auto_dock.y_place_learning import (LearningSession, atomic_json, design, metrics,
                                       validate_candidate, vector, KEYS)
from auto_dock.y_place_square_motion import measured_response


BASE = ResponseCoefficients()
TRUTH = replace(BASE, **{k: getattr(BASE, k)*1.15 for k in KEYS})
ARGS = SimpleNamespace(speed=.1, angular=.35, stage_cm=40., insert_cm=35., center_offset_left_cm=2.)
CONFIG = {'y_slot_response_model': {}}


def history(coefficients=TRUTH, variation=0.):
    state = ResponseState(history_known=True, pending_bound_deg=0.)
    t = 0.
    commands, imu = [], [[0., 0.]]
    for vx, wz, duration in [(0,0,.2), (0,.35,.8+variation), (.1,0,2.), (0,0,.5),
                             (0,-.35,1.1), (.1,0,1.5+variation), (0,0,.5)]:
        for _ in range(round(duration/.02)):
            commands.append([t, vx, 0., wz])
            state = advance_state(state, coefficients, vx, wz, .02)
            t += .02
            imu.append([t, math.radians(state.yaw_deg)])
    commands.append([t,0.,0.,0.])
    return dict(commands=commands, imu=imu)


def test_replay_matches_independent_model_and_wraps_imu():
    sample = history()
    for row in sample['imu']:
        row[1] = (row[1]+math.radians(179)+math.pi)%(2*math.pi)-math.pi
    matrix, yaw = design(sample, BASE)
    assert metrics([(matrix,yaw)],vector(TRUTH))['rmse_deg'] < .04


@pytest.mark.parametrize('mutation,reason', [
    ('reverse','reverse_or_lateral'), ('lateral','reverse_or_lateral'),
    ('nan','invalid_telemetry'), ('clock','nonmonotonic'),
    ('imu_gap','moving_imu_gap'), ('command_gap','moving_command_gap'),
])
def test_bad_telemetry_excluded(mutation, reason):
    sample=history()
    if mutation=='reverse': sample['commands'][20][1]=-.1
    if mutation=='lateral': sample['commands'][20][2]=.1
    if mutation=='nan': sample['imu'][20][1]=float('nan')
    if mutation=='clock': sample['imu'][20][0]=sample['imu'][19][0]
    if mutation=='imu_gap': del sample['imu'][20:60]
    if mutation=='command_gap': del sample['commands'][20:60]
    with pytest.raises(ValueError,match=reason): design(sample,BASE)


def test_holdout_improves_with_bounded_update_but_rejects_opposite_response():
    training=[design(dict(history(variation=i*.04), estimated_gains=vector(BASE)),BASE) for i in range(6)]
    validation=[design(dict(history(variation=v),estimated_gains=vector(BASE)),BASE) for v in (.3,.4)]
    candidate, report=validate_candidate(training,validation)
    assert report['accepted']
    assert np.all(candidate<=1.1+1e-8)
    opposite=replace(BASE,**{k:getattr(BASE,k)*.85 for k in KEYS})
    _, rejected=validate_candidate(training,[design(dict(history(opposite),estimated_gains=vector(BASE)),BASE)]*2)
    assert not rejected['accepted']


def session(root, number):
    output=root/'vehicle1'/f'auto_dock_test_y_{number:020d}'
    output.mkdir(parents=True)
    return LearningSession(output,CONFIG,ARGS)


def test_current_run_only_mode_does_not_read_saved_corrections(tmp_path, monkeypatch):
    s = session(tmp_path, 99)
    monkeypatch.setattr(s, '_state', lambda *_: pytest.fail('saved correction read'))
    plan = dict(actions=[dict(drive=[0.,0.,.35], duration_sec=.3),
                         dict(drive=[.1,0.,0.], duration_sec=2.)])
    result = s.prepare(SimpleNamespace(), BASE, ResponseState(), plan,
                       'final_insertion', apply_correction=False)
    assert result == BASE
    assert s.pending['applied_factors'] == [1.,1.,1.,1.]


def complete(s, number=0, error=None, estimate=BASE, truth=TRUTH):
    from unittest.mock import patch
    sample=history(truth, variation=number*.02)
    node=SimpleNamespace(**sample)
    plan=dict(actions=[dict(drive=[0,0,.35],duration_sec=.8),
                       dict(drive=[.1,0,0],duration_sec=2.),
                       dict(drive=[0,0,-.35],duration_sec=1.1),
                       dict(drive=[.1,0,0],duration_sec=1.5)],predicted_stage=[0,0,0])
    with patch('auto_dock.y_place_learning.time.monotonic',return_value=0.):
        corrected=s.prepare(node,estimate,ResponseState(),plan,'final_insertion')
    s.set_plan(plan,0.,-2.)
    s.capture(node,math.degrees(sample['imu'][-1][1]))
    s.finish(error)
    report=json.loads((s.output/'learning_report.json').read_text())
    return report['segments'][0],corrected


def test_persistent_promotion_next_run_and_no_validation_reuse(tmp_path):
    for i in range(8):
        s=session(tmp_path,i)
        report,_=complete(s,i)
        if i<7: assert report['status']=='collecting'
    assert report['status']=='promoted'
    assert set(report['training_runs']).isdisjoint(report['validation_runs'])
    next_run=session(tmp_path,8)
    report,corrected=complete(next_run,8)
    assert corrected != BASE
    assert report['status']=='waiting_for_two_new_validation_runs'


def test_cancelled_run_never_enters_training(tmp_path):
    s=session(tmp_path,0)
    report,_=complete(s,error='InterruptedError:stop')
    assert report['status']=='excluded'
    assert not list(s.root.glob('*/auto_dock_test_y_*.json'))


def test_profile_change_does_not_reuse_model_and_corrupt_model_falls_back(tmp_path):
    for i in range(8): complete(session(tmp_path,i),i)
    s=session(tmp_path,8)
    changed=LearningSession(s.output, {'y_slot_response_model':{},'new_calibration':True},ARGS)
    _,corrected=complete(changed)
    assert corrected==BASE
    model=next(s.root.glob('*/model.json'))
    model.write_text('{invalid')
    fallback=session(tmp_path,9)
    _,corrected=complete(fallback)
    assert corrected==BASE


def test_opposite_load_bias_never_reuses_saved_correction(tmp_path):
    for i in range(8): complete(session(tmp_path,i),i)
    biased=replace(BASE,left_immediate_gain=BASE.left_immediate_gain*1.6,
                   left_total_gain=BASE.left_total_gain*1.6,
                   right_immediate_gain=BASE.right_immediate_gain*.7,
                   right_total_gain=BASE.right_total_gain*.7)
    new_load=session(tmp_path,8)
    _,corrected=complete(new_load,estimate=biased,truth=biased)
    assert corrected==biased
    assert len(list(new_load.root.glob('*/auto_dock_test_y_*.json')))==9
    assert len(list(new_load.root.iterdir()))==2


def test_prediction_inputs_saved_before_actual_response(tmp_path):
    s=session(tmp_path,0)
    report,_=complete(s)
    predicted=json.loads((s.output/'learning_final_insertion_prediction.json').read_text())
    actual=json.loads((s.output/'learning_final_insertion_actual.json').read_text())
    assert 'imu' not in predicted
    assert predicted['estimated_gains']==vector(BASE).tolist()
    assert actual['estimated_gains']==predicted['estimated_gains']
    assert report['uncorrected_prediction']['rmse_deg']>1.


def test_learning_failure_does_not_publish_motion(tmp_path):
    # Only pure file/telemetry functions are involved; failed JSON writes leave
    # the previously committed model intact.
    path=tmp_path/'model.json'
    atomic_json(path,{'old':True})
    with pytest.raises(ValueError): atomic_json(path,{'bad':float('nan')})
    assert json.loads(path.read_text())=={'old':True}


def test_learns_distinct_left_and_right_errors_from_current_gain_inputs():
    actual=replace(BASE,left_immediate_gain=BASE.left_immediate_gain*1.15,
                   left_total_gain=BASE.left_total_gain*1.15,
                   right_immediate_gain=BASE.right_immediate_gain*.85,
                   right_total_gain=BASE.right_total_gain*.85)
    data=[design(dict(history(actual,variation=i*.04),estimated_gains=vector(BASE)),BASE)
          for i in range(8)]
    candidate,report=validate_candidate(data[:6],data[6:])
    assert report['accepted']
    assert candidate[0]>1 and candidate[1]>1
    assert candidate[2]<1 and candidate[3]<1


def test_incoming_pending_response_is_not_mistaken_for_new_turn_gain():
    sample=history(BASE)
    base_matrix,base_yaw=design(sample,BASE)
    sample['initial_state']={'yaw_deg':0.,'equilibrium_deg':2.,'pending_deg':5.}
    matrix,adjusted_yaw=design(sample,BASE)
    assert np.allclose(matrix,base_matrix)
    assert adjusted_yaw[-1]==pytest.approx(base_yaw[-1]-7.,abs=.01)
