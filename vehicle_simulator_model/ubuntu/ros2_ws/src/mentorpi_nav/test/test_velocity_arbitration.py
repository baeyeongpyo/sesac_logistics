from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))

from velocity_arbitration import VelocityArbitrator


class VelocityArbitrationTest(unittest.TestCase):
    def test_safety_stop_has_priority_over_fresh_manual_and_nav_commands(self):
        arbitrator = VelocityArbitrator(timeout_seconds=0.5)
        arbitrator.record('nav', 'nav-command', 10.0)
        arbitrator.record('manual', 'manual-command', 10.1)
        arbitrator.stop(10.2)

        self.assertEqual(arbitrator.select(10.3), None)

    def test_fresh_manual_command_has_priority_over_nav_command(self):
        arbitrator = VelocityArbitrator(timeout_seconds=0.5)
        arbitrator.record('nav', 'nav-command', 10.0)
        arbitrator.record('manual', 'manual-command', 10.1)

        self.assertEqual(arbitrator.select(10.2), 'manual-command')

    def test_stale_command_resolves_to_safe_stop(self):
        arbitrator = VelocityArbitrator(timeout_seconds=0.5)
        arbitrator.record('nav', 'nav-command', 10.0)

        self.assertIsNone(arbitrator.select(10.6))


if __name__ == '__main__':
    unittest.main()
