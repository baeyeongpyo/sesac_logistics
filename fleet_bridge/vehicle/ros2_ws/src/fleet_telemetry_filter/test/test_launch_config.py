from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE.parents[3]
COMMON = BUNDLE / 'common/fleet_bridge_config'
sys.path[:0] = [str(PACKAGE), str(COMMON)]

from fleet_bridge_config.loader import load_telemetry
from fleet_bridge_config.models import CommandConfig
from fleet_telemetry_filter.launch_config import (
    bridge_parameters,
    filtered_topics,
    forwarded_topics,
)


class VehicleLaunchConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.topics = load_telemetry(BUNDLE / 'config/telemetry.yaml', 'robot_1')
        cls.command = CommandConfig(
            topic='/cmd_vel',
            message_type='geometry_msgs/msg/Twist',
            max_linear_x=0.3,
            max_angular_z=1.0,
            max_hold_ms=1000,
            publish_rate_hz=10.0,
        )

    def test_fleet_bridge_exposes_only_enabled_uplink_topics(self):
        parameters = bridge_parameters(
            self.topics,
            self.command,
            mode='fleet',
            port=8766,
        )

        self.assertEqual(parameters['port'], 8766)
        self.assertEqual(parameters['address'], '0.0.0.0')
        self.assertEqual(parameters['capabilities'], ['none'])
        self.assertEqual(parameters['service_whitelist'], ['(?!)'])
        self.assertEqual(parameters['param_whitelist'], ['(?!)'])
        self.assertEqual(parameters['client_topic_whitelist'], ['(?!)'])
        self.assertIn('^/robot_1/odom$', parameters['topic_whitelist'])
        self.assertNotIn(
            '^/robot_1/battery$',
            parameters['topic_whitelist'],
        )
        self.assertNotIn('^/robot_1/scan_filtered$', parameters['topic_whitelist'])
        self.assertIn(
            '^/robot_1/odom$',
            parameters['best_effort_qos_topic_whitelist'],
        )

    def test_debug_bridge_exposes_debug_sources_without_fleet_filter_topics(self):
        parameters = bridge_parameters(
            self.topics,
            self.command,
            mode='debug',
            port=8765,
        )

        self.assertIn('^/scan_filtered$', parameters['topic_whitelist'])
        self.assertIn('^/scan_raw$', parameters['topic_whitelist'])
        self.assertIn(
            '^/ascamera/camera_publisher/rgb0/image$',
            parameters['topic_whitelist'],
        )
        self.assertNotIn(
            '^/robot_1/battery$',
            parameters['topic_whitelist'],
        )

    def test_safe_default_topics_do_not_apply_rate_or_change_filters(self):
        selected = filtered_topics(self.topics)

        self.assertEqual([topic.id for topic in selected], [])

    def test_enabled_prefix_rewrites_start_filter_subscriptions(self):
        selected = forwarded_topics(self.topics)

        self.assertEqual(
            [topic.id for topic in selected],
            ['odom', 'tf', 'tf_static', 'amcl_pose', 'diagnostics'],
        )

    def test_raw_bridge_exposes_root_topics_and_only_cmd_vel_client_publish(self):
        parameters = bridge_parameters(
            self.topics,
            self.command,
            mode='raw',
            port=8766,
        )

        self.assertIn('^/odom$', parameters['topic_whitelist'])
        self.assertIn('^/tf$', parameters['topic_whitelist'])
        self.assertNotIn('^/robot_1/odom$', parameters['topic_whitelist'])
        self.assertEqual(parameters['capabilities'], ['clientPublish'])
        self.assertEqual(parameters['client_topic_whitelist'], ['^/cmd_vel$'])

    def test_bridge_mode_is_strictly_validated(self):
        with self.assertRaisesRegex(ValueError, 'mode'):
            bridge_parameters(self.topics, self.command, mode='all', port=8765)

if __name__ == '__main__':
    unittest.main()
