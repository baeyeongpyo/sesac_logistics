from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE.parents[3]
COMMON = BUNDLE / 'common/fleet_bridge_config'
sys.path[:0] = [str(PACKAGE), str(COMMON)]

from fleet_bridge_config.loader import load_telemetry
from fleet_telemetry_filter.launch_config import bridge_parameters, filtered_topics


class VehicleLaunchConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.topics = load_telemetry(BUNDLE / 'config/telemetry.yaml', 'robot_1')

    def test_fleet_bridge_exposes_only_enabled_uplink_topics(self):
        parameters = bridge_parameters(self.topics, mode='fleet', port=8766)

        self.assertEqual(parameters['port'], 8766)
        self.assertEqual(parameters['address'], '0.0.0.0')
        self.assertEqual(parameters['capabilities'], ['none'])
        self.assertEqual(parameters['service_whitelist'], ['(?!)'])
        self.assertEqual(parameters['param_whitelist'], ['(?!)'])
        self.assertEqual(parameters['client_topic_whitelist'], ['(?!)'])
        self.assertIn('^/robot_1/odom$', parameters['topic_whitelist'])
        self.assertIn(
            '^/robot_1/fleet_bridge/battery_state$',
            parameters['topic_whitelist'],
        )
        self.assertNotIn('^/robot_1/scan$', parameters['topic_whitelist'])
        self.assertNotIn(
            '^/robot_1/fleet_bridge/scan$',
            parameters['topic_whitelist'],
        )
        self.assertIn(
            '^/robot_1/odom$',
            parameters['best_effort_qos_topic_whitelist'],
        )

    def test_debug_bridge_exposes_debug_sources_without_fleet_filter_topics(self):
        parameters = bridge_parameters(self.topics, mode='debug', port=8765)

        self.assertIn('^/robot_1/scan$', parameters['topic_whitelist'])
        self.assertIn('^/robot_1/scan_raw$', parameters['topic_whitelist'])
        self.assertIn(
            '^/robot_1/camera/image_raw/compressed$',
            parameters['topic_whitelist'],
        )
        self.assertNotIn(
            '^/robot_1/fleet_bridge/battery_state$',
            parameters['topic_whitelist'],
        )

    def test_only_enabled_non_passthrough_topics_start_filter_subscriptions(self):
        selected = filtered_topics(self.topics)

        self.assertEqual([topic.id for topic in selected], ['battery'])

    def test_bridge_mode_is_strictly_validated(self):
        with self.assertRaisesRegex(ValueError, 'mode'):
            bridge_parameters(self.topics, mode='all', port=8765)

if __name__ == '__main__':
    unittest.main()
