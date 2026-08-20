from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from foxglove_ros_worker.state import ReconnectBackoff, WorkerState


class ReconnectBackoffTest(unittest.TestCase):
    def test_backoff_caps_at_thirty_seconds_and_reset_starts_over(self):
        backoff = ReconnectBackoff(initial=1.0, maximum=30.0)

        self.assertEqual(
            [backoff.next_delay() for _ in range(7)],
            [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0],
        )
        backoff.reset()
        self.assertEqual(backoff.next_delay(), 1.0)


class WorkerStateTest(unittest.TestCase):
    def test_snapshot_transitions_from_waiting_to_online_to_stale(self):
        state = WorkerState('robot_1', freshness_timeout_sec=10.0)
        state.connected()

        waiting = state.snapshot(now=100.0)
        state.record_message(received_at=100.0)
        online = state.snapshot(now=109.9)
        stale = state.snapshot(now=110.1)

        self.assertEqual(waiting['state'], 'waiting')
        self.assertEqual(online['connection'], 'connected')
        self.assertEqual(online['state'], 'online')
        self.assertEqual(online['last_message_at'], 100.0)
        self.assertEqual(stale['state'], 'stale')

    def test_error_and_reconnect_count_are_reported_without_losing_last_message(self):
        state = WorkerState('robot_2', freshness_timeout_sec=5.0)
        state.connected()
        state.record_message(received_at=10.0)
        state.disconnected('connection reset', reconnect=True)

        snapshot = state.snapshot(now=11.0)

        self.assertEqual(snapshot, {
            'robot_id': 'robot_2',
            'connection': 'disconnected',
            'state': 'offline',
            'last_message_at': 10.0,
            'reconnect_count': 1,
            'error': 'connection reset',
        })


if __name__ == '__main__':
    unittest.main()
