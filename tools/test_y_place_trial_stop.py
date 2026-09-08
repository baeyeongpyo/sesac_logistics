"""Non-runtime stop-latch tests; no ROS, motor commands or process signalling."""
from pathlib import Path
import signal
import tempfile
import unittest

from y_place_trial_stop import TrialStop


class TrialStopTests(unittest.TestCase):
    def test_new_stopper_marker_latches_even_after_marker_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'stop'
            stop=TrialStop(path)
            stop.check()
            path.touch()
            with self.assertRaises(InterruptedError):
                stop.check()
            path.unlink()
            with self.assertRaises(InterruptedError):
                stop.check()

    def test_new_explicit_trial_can_start_after_previous_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'stop'
            path.touch()
            stop=TrialStop(path)
            stop.check()
            # Replacing a marker changes its identity even at equal timestamp.
            replacement=Path(directory)/'next'
            replacement.touch()
            replacement.replace(path)
            with self.assertRaises(InterruptedError):
                stop.check()

    def test_ctrl_c_and_termination_both_stop(self):
        for signum in (signal.SIGINT,signal.SIGTERM):
            with tempfile.TemporaryDirectory() as directory:
                stop=TrialStop(Path(directory)/'stop')
                stop.handle_signal(signum,None)
                with self.assertRaises(InterruptedError):
                    stop.check()


if __name__=='__main__':
    unittest.main()
