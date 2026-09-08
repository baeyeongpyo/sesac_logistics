import unittest
from y_place_command_monitor import CommandConflictMonitor


class CommandMonitorTests(unittest.TestCase):
    def test_idle_zero_does_not_block_start(self):
        monitor=CommandConflictMonitor()
        monitor.received((0,0,0))
        monitor.check()

    def test_own_echoes_during_turn_forward_stop(self):
        monitor=CommandConflictMonitor()
        monitor.active=True
        for drive in ((0,0,.35),(.1,0,0),(0,0,0)):
            monitor.sent(drive)
            monitor.received(drive)
            monitor.check()

    def test_external_nonzero_before_start_blocks_until_zero(self):
        monitor=CommandConflictMonitor()
        monitor.received((.1,0,0))
        with self.assertRaises(InterruptedError): monitor.check()
        monitor.received((0,0,0))
        monitor.check()

    def test_external_zero_during_drive_stops_and_latches(self):
        monitor=CommandConflictMonitor()
        monitor.active=True
        monitor.sent((.1,0,0))
        monitor.received((.1,0,0))
        monitor.received((0,0,0))
        with self.assertRaises(InterruptedError): monitor.check()
        monitor.sent((.1,0,0))
        monitor.received((.1,0,0))
        with self.assertRaises(InterruptedError): monitor.check()

    def test_different_nonzero_is_a_conflict(self):
        monitor=CommandConflictMonitor()
        monitor.active=True
        monitor.sent((.1,0,0))
        monitor.received((0,0,-.35))
        with self.assertRaises(InterruptedError): monitor.check()


if __name__=='__main__': unittest.main()
