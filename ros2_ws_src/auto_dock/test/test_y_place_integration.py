"""Offline adapter tests; no Node construction, ROS init or vehicle commands."""
import ast
import json
import os
import threading
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from auto_dock import y_place
from auto_dock.auto_dock_node import AutoDockNode, public_fsm_state, resolve_vehicle_id
from auto_dock.y_place import YPlaceRunner, apply_event, frozen_profile


BASELINE = Path(__file__).parent / 'fixtures/test_y_vehicle'
PACKAGE = Path(y_place.__file__).parent


@pytest.mark.parametrize('requested,domain,expected', [(0,215,1),(0,216,2),(1,216,1),(0,0,0)])
def test_actual_constructor_resolves_default_vehicle_without_starting_a_node(monkeypatch, requested, domain, expected):
    # Execute only the real constructor's vehicle assignments with a parameter
    # stub: no ROS context, Node, subscriptions, timer, or runtime is created.
    tree = ast.parse((PACKAGE / 'auto_dock_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'AutoDockNode')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    assignments = [n for n in init.body if isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == 'requested_vehicle' or
        isinstance(t, ast.Attribute) and t.attr == 'vehicle' for t in n.targets)]
    fake = NS(get_parameter=lambda _: NS(value=requested))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    exec(compile(ast.Module(body=assignments, type_ignores=[]), '<vehicle init test>', 'exec'),
         {'self': fake, 'os': os, 'resolve_vehicle_id': resolve_vehicle_id})
    assert fake.vehicle == expected


def runner(tmp_path):
    drives, forks = [], []
    value = YPlaceRunner(
        drive=lambda *v: drives.append(v), fork=forks.append,
        clock=lambda: NS(now=lambda: NS(nanoseconds=100)), bridge=None,
        output=tmp_path / 'mission', stop_file=tmp_path / 'stop',
    )
    return value, drives, forks


def test_exact_vehicle_profile_ignores_legacy_config():
    args, config = frozen_profile()
    assert (args.stage_cm, args.insert_cm, args.speed, args.angular,
            args.center_offset_left_cm, args.start_frames) == (40., 38., .1, .35, 2., 1)
    assert 'loaded' in config['floor_calibration_presets']
    assert config['y_slot_response_model']['model_id'].startswith('vehicle1-loaded')


@pytest.mark.parametrize('acknowledge', [True, False])
def test_single_insertion_waits_for_down_before_retreat(tmp_path, acknowledge):
    """Run the real post-stage control flow with hardware and time replaced."""
    from auto_dock import y_place_sequence as sequence
    from auto_dock.y_place_finish import ForkDownGate
    tree = ast.parse((PACKAGE / 'y_place_sequence.py').read_text())
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run_sequence')
    body = run.body[0].body
    start = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'stage_distance' for t in n.targets))
    code = compile(ast.Module(body=body[start:], type_ignores=[]), '<post-stage sequence>', 'exec')
    events, plans = [], []
    gate = ForkDownGate()
    ticks = iter(range(1000))
    node = NS(fork_gate=gate, publish=lambda command: events.append(('drive', command)),
              imu=[(0.,0.)],
              report=lambda event: None, command_monitor=NS(check=lambda: None),
              fork_pub=NS(publish=lambda msg: events.append(('fork', msg.data))))

    def spin_once(_node, **_kwargs):
        if gate.active and acknowledge:
            events.append(('ack', 'DOWN_COMPLETE'))
            gate.receive('{"state":"DOWN_COMPLETE"}')

    def plan(*args):
        plans.append(args[4])
        return dict(accepted=True, actions=[dict(action='insert', distance_cm=args[4])])

    scope = dict(vars(sequence), node=node, args=frozen_profile()[0], output=tmp_path,
                 stage_pose={'topline_forward_cm': 40., 'heading_left_deg': 0.}, fitted={}, state={}, gains={},
                 remaining_insertion_cm=lambda pose, endpoint: 40. - endpoint,
                 learned_insertion_plan=plan, capture_learning=lambda *_: None,
                 execute=lambda actions, **kw: events.extend(('action', a) for a in actions),
                 correct_heading=lambda *args: (0., {'final_error_deg': 0.}),
                 wait_stopped=lambda _: 0., spin_once=spin_once,
                 stop=NS(check=lambda: None), time=NS(monotonic=lambda: next(ticks)))
    if acknowledge:
        exec(code, scope)
        reverse = next(i for i, e in enumerate(events)
                       if e[0] == 'action' and e[1]['action'] == 'reverse_after_fork_down')
        assert events.index(('fork', 'DOWN')) < events.index(('ack', 'DOWN_COMPLETE')) < reverse
        assert events[reverse][1]['duration_sec'] == pytest.approx(3.8)
    else:
        with pytest.raises(RuntimeError, match='fork_down_timeout'):
            exec(code, scope)
        assert not any(e[0] == 'action' and e[1]['action'] == 'reverse_after_fork_down' for e in events)
    assert plans == [38.]
    assert sum(e[0] == 'action' and e[1]['action'] == 'insert' for e in events) == 1


@pytest.mark.parametrize('name', [
    'y_place_square_geometry', 'y_place_square_motion', 'y_place_slot_sides',
    'y_place_warning_tape', 'y_place_midpoint', 'y_place_finish',
    'y_place_command_monitor', 'y_place_trial_stop',
])
def test_every_algorithm_function_matches_vehicle_baseline(name):
    def definitions(path):
        tree = ast.parse(path.read_text())
        return [ast.dump(n, include_attributes=False) for n in tree.body
                if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                and n.name not in {"detect_nearest_topline", "inspect_nearest_topline", "side_intersections", "interior_x_evidence", "interior_yellow_evidence", "detect_locked_topline", "solve_side_approach", "measured_response"}]
    assert definitions(PACKAGE / (name + '.py')) == definitions(BASELINE / (name + '.py'))


def test_observation_callbacks_are_unchanged():
    tree = ast.parse((BASELINE / 'y_place_square_topline_trial.py').read_text())
    live = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run_live')
    trial = next(n for n in live.body if isinstance(n, ast.ClassDef))
    original = {n.name: ast.dump(n) for n in trial.body if isinstance(n, ast.FunctionDef)}
    packaged = ast.parse((PACKAGE / 'y_place_observations.py').read_text())
    cls = next(n for n in packaged.body if isinstance(n, ast.ClassDef))
    for callback in cls.body:
        if callback.name == 'image':
            continue  # Covered by recovery and recorded-frame behavior tests.
        assert ast.dump(callback) == original[callback.name]


def test_learning_capture_precedes_unloaded_motion_and_preserves_online_estimation():
    source = (PACKAGE / 'y_place_sequence.py').read_text()
    assert source.index("wait_stopped('before_fork_down')") < source.index('capture_learning(node, final_heading)')
    assert source.index('capture_learning(node, final_heading)') < source.index('node.fork_gate.begin()')
    assert source.index('node.fork_gate.begin()') < source.index('execute([retreat])')
    assert source.count('= measured_response(node.commands, node.imu,') == 2


def test_slow_worker_cannot_restart_motion_after_cancel(tmp_path, monkeypatch):
    value, drives, forks = runner(tmp_path)
    calculating, release = threading.Event(), threading.Event()

    def fake_sequence(node, *_args):
        calculating.set()
        assert release.wait(2.)
        node.publish((.1, 0., 0.))

    monkeypatch.setattr(y_place, 'run_sequence', fake_sequence)
    value.start()
    assert calculating.wait(2.)
    value.cancel()
    release.set()
    value.thread.join(timeout=2.)
    assert not value.is_alive()
    assert drives and all(not any(command) for command in drives)
    assert all(command == 'STOP' for command in forks)
    assert value.events.get_nowait()['error'].startswith('InterruptedError:')


def test_cancel_blocks_late_drive_and_fork_after_solver_returns(tmp_path):
    value, drives, forks = runner(tmp_path)
    value.publish((.1, 0., 0.))
    value.cancel()
    with pytest.raises(InterruptedError):
        value.publish((.1, 0., 0.))
    with pytest.raises(InterruptedError):
        value.publish_fork('DOWN')
    assert drives == [(.1, 0., 0.), (0., 0., 0.)]
    assert forks == ['STOP']


def test_marker_blocks_late_nonzero_without_signal_handler(tmp_path):
    value, drives, _ = runner(tmp_path)
    (tmp_path / 'stop').touch()
    with pytest.raises(InterruptedError):
        value.publish((.1, 0., 0.))
    assert drives == []


def test_inbox_latest_image_and_command_history_serviced_without_ros(tmp_path):
    value, _, _ = runner(tmp_path)
    received = []
    value.image = lambda msg: received.append(('image', msg))
    value.command = lambda msg: received.append(('command', msg))
    value.feed('image', 1)
    value.feed('image', 2)
    value.feed('command', 3)
    value.spin_once(value)
    value.spin_once(value)
    assert received == [('command', 3), ('image', 2)]


def host():
    value = NS(state='y_slot_centering', operation='PLACE', load_state='LOADED',
               ready=[], statuses=[], cancelled=[])
    value.y_place_runner = NS(args=NS(insert_cm=25.0))
    value.drive_ready_pub = NS(publish=value.ready.append)
    value.y_place_empty_message = lambda: 'empty'
    value.publish_status = lambda *a, **kw: value.statuses.append((a, kw))
    value.stop_drive = lambda *_: None
    def cancel(reason):
        value.state = 'idle'
        value.cancelled.append(reason)
    value.cancel = cancel
    return value


def test_fork_reverse_ready_contract_and_failure():
    value = host()
    apply_event(value, {'phase': 'waiting_fork'})
    assert public_fsm_state(value.state, 'PLACE') == 'WAIT_DOWN_COMPLETE'
    apply_event(value, {'phase': 'action', 'action': 'reverse_after_fork_down'})
    assert public_fsm_state(value.state, 'PLACE') == 'REVERSING'
    assert value.load_state == 'UNLOADED'
    apply_event(value, {'phase': 'command_sequence_complete'})
    assert value.ready == []
    apply_event(value, {'phase': 'result', 'error': None})
    assert public_fsm_state(value.state, 'PLACE') == 'READY'
    assert value.ready == ['empty']
    apply_event(value, {'phase': 'result', 'error': None})
    assert value.ready == ['empty']
    failed = host()
    apply_event(failed, {'phase': 'result', 'error': 'fork_down_timeout'})
    assert failed.cancelled == ['fork_down_timeout'] and failed.ready == []


def test_cancelled_result_cannot_emit_ready():
    value = host()
    value.cancel('emergency_stop')
    apply_event(value, {'phase': 'result', 'error': None})
    assert value.ready == []


def test_y_arrival_calls_new_runner_without_loading_old_config():
    value = host()
    value.state = 'idle'
    value.mission_kind = None
    value.y_place_runner = None
    value.started = []
    value.start_y_place = lambda: value.started.append(True)
    value.load_config = lambda: pytest.fail('old config must not control Y PLACE')
    AutoDockNode.on_trigger(value, NS(data=json.dumps({
        'status': 'SUCCEEDED', 'operation': 'PLACE', 'location': 'Y1',
        'product_type': 'NORMAL', 'target': {'type': 'NEAREST'},
    })))
    assert value.started == [True]
    assert value.location == 'Y' and value.mission_kind == 'Y_PLACE'


def test_y_tick_never_invokes_old_motion_or_lidar():
    value = NS(mission_kind='Y_PLACE', calls=[])
    value.tick_y_place = lambda: value.calls.append('test_y')
    value.interrupt_for_lidar = lambda: pytest.fail('legacy motion must not run')
    AutoDockNode.tick(value)
    assert value.calls == ['test_y']


@pytest.mark.parametrize('method', [
    'on_y_slot_manual_insertion', 'on_y_slot_insertion_default',
    'on_y_slot_response_config', 'on_y_slot_pose_source',
])
def test_legacy_y_controls_cannot_replace_test_y(method):
    value = host()
    value.mission_kind = 'Y_PLACE'
    getattr(AutoDockNode, method)(value, NS(data='{}'))
    assert value.statuses[-1][0] == ('rejected', 'test_y_profile_is_fixed')


def test_motion_helpers_except_adaptive_gain_switch_preserve_baseline():
    def definitions(path):
        return {n.name: ast.dump(n, include_attributes=False)
                for n in ast.parse(path.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name not in {'measured_response', 'solve_side_approach'}}
    assert definitions(PACKAGE/'y_place_square_motion.py') == definitions(BASELINE/'y_place_square_motion.py')


def test_runner_learning_updates_after_final_zero_only(tmp_path, monkeypatch):
    value, drives, forks = runner(tmp_path)
    order = []
    class FakeLearning:
        def __init__(self, *args): pass
        def finish(self, error):
            assert value.finished and drives[-1] == (0., 0., 0.)
            order.append(error)
    monkeypatch.setattr(y_place, 'LearningSession', FakeLearning)
    monkeypatch.setattr(y_place, 'run_sequence', lambda *args: None)
    value._run()
    assert order == [None]
    assert value.events.get_nowait() == {'phase': 'result', 'error': None}


def test_runner_learning_failure_preserves_completed_result(tmp_path, monkeypatch):
    value, drives, forks = runner(tmp_path)
    class FakeLearning:
        def __init__(self, *args): pass
        def finish(self, error): raise OSError('disk full')
    monkeypatch.setattr(y_place, 'LearningSession', FakeLearning)
    monkeypatch.setattr(y_place, 'run_sequence', lambda *args: None)
    value._run()
    assert drives == [(0., 0., 0.)]
    assert value.events.get_nowait()['phase'] == 'learning_unavailable'
    assert value.events.get_nowait() == {'phase': 'result', 'error': None}


def test_public_distances_follow_runner_insertion_distance():
    value = host()
    for event, field in [
        ({'phase': 'insertion_started'}, 'insertion_distance_cm'),
        ({'phase': 'action', 'action': 'reverse_after_fork_down'}, 'reverse_target_cm'),
        ({'phase': 'result', 'error': None}, 'reversed_cm'),
    ]:
        apply_event(value, event)
        assert value.statuses[-1][1][field] == 25.0
