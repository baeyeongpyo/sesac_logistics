"""Offline regression tests; all motion/time/IMU providers are fake."""
from dataclasses import replace
import math
from types import SimpleNamespace as NS

import pytest

from auto_dock.loaded_response_planner import ResponseCoefficients
from auto_dock.y_place_heading_feedback import correct_heading
from auto_dock.y_place_square_motion import measured_response


@pytest.mark.parametrize('duration,yaw_deg,expected', [
    (.4448891, 3.794229388, .425286054),
    (.270631233, 2.495993082, .459912046),
])
def test_recorded_short_approach_uses_measured_immediate_gain(duration, yaw_deg, expected):
    end = .1 + duration
    commands = [(0,0,0,0), (.1,0,0,.35), (end,.1,0,0), (end+2,0,0,0)]
    imu = [(0,0), (.1,0), (end,math.radians(yaw_deg)),
           (end+2,math.radians(yaw_deg+10))]
    fitted, _, evidence = measured_response(commands, imu, ResponseCoefficients())
    assert fitted.left_immediate_gain == pytest.approx(expected, abs=1e-8)
    assert evidence['left']['short_pulses_measured'] == 1
    assert evidence['left']['short_pulses_using_saved_immediate'] == 0


@pytest.mark.parametrize('target,initial', [
    (-43.973870731264434, -50.173792306666144),
    (-90.11080078299396, -98.69026862800116),
    (-179., 170.), (179., -170.),
])
def test_recorded_residuals_are_corrected_despite_different_current_response(target, initial):
    """The initial estimate is wrong and motor response changes between pulses."""
    yaw = initial
    calls = []
    def execute(actions):
        nonlocal yaw
        action = actions[0]
        # Different response from fitted .1587; alternate to check remeasurement.
        gain = .31 if len(calls) % 2 else .46
        delta = math.degrees(action['drive'][2] * action['duration_sec']) * gain
        yaw += delta
        calls.append(action)
        assert len(calls) < 40, 'offline controller did not converge'
        return actions
    actual, result = correct_heading(target, initial, ResponseCoefficients(), .35,
                                    execute, lambda _: yaw, lambda _: None,
                                    NS(check=lambda: None))
    assert abs((target-actual+180)%360-180) <= 1.5
    assert abs(result['initial_error_deg']) > 6
    assert calls
    assert not result['final_visual_position_verified']


def test_in_tolerance_does_not_move():
    actual, result = correct_heading(0, .3, ResponseCoefficients(), .35,
        lambda _: pytest.fail('unneeded motion'), lambda _: 0, lambda _: None,
        NS(check=lambda: None))
    assert actual == .3 and result['corrections'] == []


def test_emergency_stop_interrupts_correction_without_more_motion():
    def cancelled(): raise InterruptedError('operator_stop')
    with pytest.raises(InterruptedError, match='operator_stop'):
        correct_heading(0, -8.6, ResponseCoefficients(), .35,
            lambda _: pytest.fail('motion after stop'), lambda _: 0,
            lambda _: None, NS(check=cancelled))


def test_planned_insertion_preserves_duration_even_if_immediate_target_is_reached():
    """Exercise the actual nested executor with fake time and command output."""
    import ast
    from pathlib import Path
    from auto_dock import y_place_sequence
    tree = ast.parse(Path(y_place_sequence.__file__).read_text())
    function = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == 'execute')
    clock = NS(value=0.)
    drives = []
    def sleep(seconds): clock.value += seconds
    scope = dict(node=NS(report=lambda _: None, imu=[(0., 0.)],
                        command_monitor=NS(check=lambda: None), publish=drives.append),
                 time=NS(monotonic=lambda: clock.value, sleep=sleep),
                 stop=NS(check=lambda: None), spin_once=lambda *a, **kw: None,
                 turn_target_reached=lambda *a: True)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<executor>', 'exec'), scope)
    action = dict(action='turn_left', drive=[0.,0.,.35],
                  duration_sec=.514, target_turn_deg=1.6355)
    actual = scope['execute']([action], stop_on_turn_target=False)
    assert actual[0]['duration_sec'] == pytest.approx(.514)
    assert any(d[2] == .35 for d in drives)
    drives.clear()
    scope['execute']([action])
    assert all(d == (0.,0.,0.) for d in drives)
