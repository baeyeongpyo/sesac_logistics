"""Timing and invalidation contracts; synthetic data does not validate vehicles."""
import math
from unittest.mock import patch

import numpy as np
import pytest

from auto_dock.loaded_response_history import estimate_from_history
from auto_dock.loaded_response_planner import ResponseCoefficients, ResponseState

C = ResponseCoefficients()


def samples():
    t = np.arange(0., 1.01, .1)
    return [(float(s), .1, 0., 0.) for s in t], [(float(s), 0.) for s in t]


def test_stationary_history_cannot_invent_zero_pending():
    commands, yaw = samples()
    commands = [(t, 0., vy, wz) for t, vx, vy, wz in commands]
    result = estimate_from_history(commands, yaw, C, 1.)
    assert not result['observable']
    assert result['reason'] == 'forward_excitation_required'
    assert not result['state'].history_known


@pytest.mark.parametrize('kind,reason', [('command', 'command_receipt_gap'), ('yaw', 'odom_source_gap')])
def test_gap_not_interpolated(kind, reason):
    commands, yaw = samples()
    if kind == 'command':
        commands = commands[:2]+commands[7:]
    else:
        yaw = yaw[:2]+yaw[7:]
    assert estimate_from_history(commands, yaw, C, 1.)['reason'] == reason


def test_stale_future_and_out_of_order_fail():
    commands, yaw = samples()
    assert estimate_from_history(commands, yaw, C, 1.5)['reason'] == 'stale_history'
    assert estimate_from_history(commands, yaw, C, .9)['reason'] == 'future_history_timestamp'
    commands[2], commands[3] = commands[3], commands[2]
    assert estimate_from_history(commands, yaw, C, 1.)['reason'] == 'nonmonotonic_history_timestamps'


def test_union_keeps_command_edge_and_yaw_wrap_without_future_label():
    commands, yaw = samples()
    commands.insert(3, (.25, .1, 0., .2))
    yaw = [(t, math.radians(179.+4*t)) for t, value in yaw]
    captured = {}
    def estimator(t, vx, wz, angles, coefficients, **kwargs):
        captured.update(t=t, vx=vx, wz=wz, yaw=angles, **kwargs)
        return dict(state=ResponseState(history_known=True, pending_bound_deg=1.), observable=True, reason='test')
    with patch('auto_dock.loaded_response_history.estimate_history', estimator):
        result = estimate_from_history(commands, yaw, C, 1.)
    index = list(captured['t']).index(.25)
    assert captured['wz'][index] == .2
    assert captured['wz'][index-1] == 0.
    assert captured['yaw'][index] == pytest.approx(180.)
    assert result['interpolated_yaw_samples'] == 1
    assert captured['effective_sample_count'] == len(yaw)


@pytest.mark.parametrize('vx,vy', [(-.1, 0.), (0., .1)])
def test_trim_after_unsupported_motion(vx, vy):
    commands, yaw = samples()
    commands[2] = (.2, vx, vy, 0.)
    result = estimate_from_history(commands, yaw, C, 1.)
    assert result['trimmed_unsupported_motion']
    assert result['fitted_time_range_sec'][0] >= .3


def test_active_reverse_and_load_reset_have_no_false_known_state():
    commands, yaw = samples()
    commands[-1] = (1., -.1, 0., 0.)
    assert estimate_from_history(commands, yaw, C, 1.)['reason'] == 'reverse_or_lateral_response_unidentified'
    commands, yaw = samples()
    result = estimate_from_history(commands, yaw, C, 1., reset_time=.9)
    assert result['reason'] == 'insufficient_post_reset_history'


def test_recent_state_propagation_is_explicit():
    commands, yaw = samples()
    result = estimate_from_history(commands, yaw, C, 1.05)
    assert result['observable']
    assert result['state_timestamp'] == 1.05
    assert result['unobserved_propagation_sec'] == pytest.approx(.05)
    assert 'receipt' in result['timestamp_assumption']
