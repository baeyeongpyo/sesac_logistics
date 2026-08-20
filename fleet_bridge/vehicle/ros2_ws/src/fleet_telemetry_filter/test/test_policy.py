from dataclasses import dataclass
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common/fleet_bridge_config'
sys.path[:0] = [str(PACKAGE), str(COMMON)]

from fleet_bridge_config.models import CriticalConfig, FilterConfig
from fleet_telemetry_filter.policy import ForwardPolicy


@dataclass
class Battery:
    percentage: float
    voltage: float


class ForwardPolicyTest(unittest.TestCase):
    def test_rate_policy_drops_samples_inside_period(self):
        policy = ForwardPolicy(FilterConfig(mode='rate', max_rate_hz=2.0))

        self.assertTrue(policy.should_forward(object(), 0))
        self.assertFalse(policy.should_forward(object(), 100_000_000))
        self.assertTrue(policy.should_forward(object(), 500_000_000))

    def test_on_change_obeys_rate_and_thresholds(self):
        policy = ForwardPolicy(FilterConfig(
            mode='on_change',
            max_rate_hz=0.2,
            heartbeat_sec=30.0,
            thresholds={'percentage': 0.01, 'voltage': 0.1},
        ))

        self.assertTrue(policy.should_forward(Battery(0.50, 12.3), 0))
        self.assertFalse(policy.should_forward(Battery(0.55, 12.3), 1_000_000_000))
        self.assertFalse(policy.should_forward(Battery(0.505, 12.35), 5_000_000_000))
        self.assertTrue(policy.should_forward(Battery(0.52, 12.3), 5_000_000_001))

    def test_on_change_emits_heartbeat_without_value_change(self):
        policy = ForwardPolicy(FilterConfig(
            mode='on_change',
            max_rate_hz=1.0,
            heartbeat_sec=30.0,
            thresholds={'percentage': 0.01},
        ))

        self.assertTrue(policy.should_forward(Battery(0.50, 12.3), 0))
        self.assertFalse(policy.should_forward(Battery(0.50, 12.3), 29_999_999_999))
        self.assertTrue(policy.should_forward(Battery(0.50, 12.3), 30_000_000_000))

    def test_battery_critical_sample_bypasses_rate_limit(self):
        policy = ForwardPolicy(FilterConfig(
            mode='on_change',
            max_rate_hz=0.2,
            heartbeat_sec=30.0,
            thresholds={'percentage': 0.01, 'voltage': 0.1},
            critical=CriticalConfig(
                field='percentage',
                below=0.2,
                bypass_rate_limit=True,
            ),
        ))

        self.assertTrue(policy.should_forward(Battery(0.50, 12.3), 0))
        self.assertTrue(policy.should_forward(Battery(0.19, 12.2), 1_000_000))

    def test_passthrough_always_forwards(self):
        policy = ForwardPolicy(FilterConfig(mode='passthrough'))

        self.assertTrue(policy.should_forward(object(), 0))
        self.assertTrue(policy.should_forward(object(), 1))


if __name__ == '__main__':
    unittest.main()
