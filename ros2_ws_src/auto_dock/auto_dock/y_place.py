"""Run the frozen vehicle test_y sequence inside auto_dock, without another Node.

The worker retains test_y's spin/drain/sleep sequence. ROS callbacks only enqueue
sensor messages; cancellation and command publication share a lock so a stopped
worker cannot publish another nonzero command, even after a slow solver returns.
"""
from collections import deque
import json
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace

from .y_place_command_monitor import CommandConflictMonitor
from .y_place_finish import ForkDownGate
from .y_place_observations import TrialObservations
from .y_place_sequence import run_sequence
from .y_place_trial_stop import TrialStop
from .y_place_learning import LearningSession


def frozen_profile():
    directory = Path(__file__).parent
    baseline = json.loads((directory / 'y_place_baseline.json').read_text())
    return (SimpleNamespace(**baseline['effective_args']),
            json.loads((directory / 'y_place_config.json').read_text()))


class YPlaceRunner(TrialObservations):
    def __init__(self, *, drive, fork, clock, bridge, output, stop_file=None):
        self.args, self.config = frozen_profile()
        if stop_file is not None:
            self.args.stop_file = str(stop_file)
        self.stop = TrialStop(self.args.stop_file)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self._drive, self._fork, self._clock = drive, fork, clock
        self._gate = threading.RLock()
        self._condition = threading.Condition()
        self._pending = {name: deque(maxlen=depth) for name, depth in
                         [('command', 50), ('orientation', 5), ('fork_state', 10), ('image', 1)]}
        self._order = deque(self._pending)
        self.events = queue.SimpleQueue()
        self.thread = None
        self.finished = False
        self.bridge = bridge
        self.observations = []
        self.anchor = self.args.anchor
        self.lock_size = None
        self.frozen = False
        self.reacquiring = False
        self.last_frame = None
        self.stage_frame = None
        self.last_stamp = None
        self.image_after_ns = self.get_clock().now().nanoseconds
        self.image_timing = None
        self.stale_images = 0
        self.commands, self.imu = [], []
        self.command_monitor = CommandConflictMonitor()
        self.moved = False
        self.fork_gate = ForkDownGate()
        self.fork_pub = SimpleNamespace(publish=lambda msg: self.publish_fork(msg.data))

    def get_clock(self):
        return self._clock()

    def other_publishers(self):
        # Registration is informational in test_y. Actual received commands are
        # still checked by the unchanged conflict monitor.
        return []

    def feed(self, kind, message):
        with self._condition:
            if self.finished or self.stop.requested.is_set():
                return
            self._pending[kind].append(message)
            self._condition.notify()

    def spin_once(self, _node, timeout_sec=0.):
        with self._condition:
            if not any(self._pending.values()) and timeout_sec > 0:
                self._condition.wait(timeout_sec)
            callback = None
            for _ in range(len(self._order)):
                kind = self._order[0]
                self._order.rotate(-1)
                if self._pending[kind]:
                    callback = getattr(self, kind)
                    message = self._pending[kind].popleft()
                    break
        if callback is not None:
            callback(message)

    def publish(self, drive):
        with self._gate:
            if any(drive):
                self.stop.check()
                if self.finished:
                    raise InterruptedError('y_place_finished')
            self.command_monitor.sent(drive)
            self._drive(*map(float, drive))
            self.moved |= any(drive)

    def publish_fork(self, command):
        with self._gate:
            if command != 'STOP':
                self.stop.check()
                if self.finished:
                    raise InterruptedError('y_place_finished')
            self._fork(command)
            if command == 'DOWN':
                self.report({'phase': 'waiting_fork'})

    def report(self, event):
        self.events.put(dict(event))

    def cancel(self):
        with self._gate:
            self.stop.requested.set()
            self._drive(0., 0., 0.)
            self._fork('STOP')
        with self._condition:
            self._condition.notify_all()

    def start(self):
        if self.thread is not None:
            raise RuntimeError('y_place_already_started')
        self.thread = threading.Thread(target=self._run, name='auto_dock_y_place', daemon=True)
        self.thread.start()

    def _run(self):
        error = None
        self.learning = None
        try:
            try:
                self.learning = LearningSession(self.output, self.config, self.args)
            except (OSError, ValueError, TypeError) as exc:
                self.report({'phase': 'learning_unavailable', 'reason': str(exc)})
            run_sequence(self, self.args, self.config, self.output, self.stop, self.spin_once)
            self.stop.check()
        except Exception as exc:
            error = f'{type(exc).__name__}:{exc}'
        finally:
            with self._gate:
                self.finished = True
                self.stop.requested.set()
                # Also cover exceptions before test_y's first move/fork command.
                try:
                    self._drive(0., 0., 0.)
                    if error:
                        self._fork('STOP')
                except Exception as exc:
                    error = error or f'final_stop_failed:{exc}'
            # Learning is offline work, after drive/fork finalization. It may
            # never delay sending zero or change this completed drive's model.
            if self.learning is not None:
                try:
                    self.learning.finish(error)
                except Exception as exc:
                    self.report({'phase': 'learning_unavailable', 'reason': str(exc)})
            try:
                (self.output / 'result.json').write_text(json.dumps({'phase': 'result', 'error': error}))
            finally:
                self.report({'phase': 'result', 'error': error})

    def is_alive(self):
        return self.thread is not None and self.thread.is_alive()


def apply_event(host, event):
    """Keep the existing public FSM and emit READY only after the full sequence."""
    phase = event.get('phase')
    if phase == 'result':
        if event.get('error'):
            host.cancel(event['error'])
            host.y_place_failure = event['error']
            host.y_place_failure_sent_at = time.monotonic()
            host.publish_status('cancelled', event['error'], terminal=True, error=event['error'])
        elif host.state not in {'idle', 'ready'}:
            host.stop_drive(10)
            host.state = 'ready'
            host.drive_ready_pub.publish(host.y_place_empty_message())
            host.publish_status('completed', 'drive_ready_after_straight_reverse',
                                reversed_cm=host.y_place_runner.args.insert_cm, actual_position_verified=False,
                                reverse_distance_model='unloaded nominal 1:1; not measured')
        return
    if host.state == 'idle':
        return
    if phase == 'waiting_fork':
        host.state = 'waiting_fork'
        host.publish_status('waiting', 'fork_command_sent', fork_command='DOWN')
    elif phase == 'action':
        if event.get('action') == 'reverse_after_fork_down':
            host.load_state = 'UNLOADED'
            host.state = 'reversing_after_lift'
            host.publish_status('running', 'fork_complete_reversing',
                                fork_state='DOWN_COMPLETE', reverse_target_cm=host.y_place_runner.args.insert_cm)
        else:
            host.publish_status('running', 'test_y_action', action=event.get('action'))
    elif phase in {'midpoint_reacquisition', 'midpoint_reacquisition_after_reverse'}:
        host.state = 'y_slot_inserting'
        host.publish_status('running', 'test_y_midpoint_reacquisition')
    elif phase.startswith('stage_reacquisition'):
        host.publish_status('running', phase)
    elif phase == 'insertion_started':
        host.state = 'y_slot_inserting'
        host.publish_status('running', 'y_slot_inserting', insertion_distance_cm=host.y_place_runner.args.insert_cm)
