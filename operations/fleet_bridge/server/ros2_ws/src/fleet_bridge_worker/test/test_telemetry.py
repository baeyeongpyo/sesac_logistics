import math
from pathlib import Path
from types import SimpleNamespace
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_worker.telemetry import (
    FleetTelemetryRecorder,
    TelemetryStore,
    TfFreshness,
    _arguments,
    active_robot_ids,
    quaternion_to_yaw,
    transform_belongs_to_robot,
)
from fleet_bridge_config.models import (
    CommandApiConfig,
    FleetConfig,
    ServerConfig,
    VehicleConfig,
)


class TelemetryStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.database_path = Path(self.temporary_directory.name) / 'telemetry.db'
        self.store = TelemetryStore(self.database_path)

    def tearDown(self):
        self.store.close()
        self.temporary_directory.cleanup()

    def test_pose_upsert_replaces_only_the_same_robot_latest_state(self):
        self.store.upsert_pose(
            'robot_1',
            x_m=1.0,
            y_m=2.0,
            yaw_rad=0.1,
            observed_at='2026-08-31T00:00:00Z',
            received_at='2026-08-31T00:00:01Z',
        )
        self.store.upsert_pose(
            'robot_1',
            x_m=3.0,
            y_m=4.0,
            yaw_rad=0.2,
            observed_at='2026-08-31T00:00:02Z',
            received_at='2026-08-31T00:00:03Z',
        )
        self.store.upsert_pose(
            'robot_2',
            x_m=5.0,
            y_m=6.0,
            yaw_rad=0.3,
            observed_at='2026-08-31T00:00:04Z',
            received_at='2026-08-31T00:00:05Z',
        )

        self.assertEqual(
            self.store.fetch_pose('robot_1'),
            {
                'robot_id': 'robot_1',
                'x_m': 3.0,
                'y_m': 4.0,
                'yaw_rad': 0.2,
                'observed_at': '2026-08-31T00:00:02Z',
                'received_at': '2026-08-31T00:00:03Z',
            },
        )
        self.assertEqual(
            self.store.fetch_pose('robot_2')['received_at'],
            '2026-08-31T00:00:05Z',
        )

    def test_battery_raw_is_saved_without_voltage_or_percent_conversion(self):
        self.store.upsert_battery(
            'robot_1',
            battery_raw=8354,
            received_at='2026-08-31T00:00:00Z',
        )

        self.assertEqual(
            self.store.fetch_battery('robot_1'),
            {
                'robot_id': 'robot_1',
                'battery_raw': 8354,
                'received_at': '2026-08-31T00:00:00Z',
            },
        )

    def test_invalid_pose_and_battery_values_are_rejected_before_storage(self):
        with self.assertRaisesRegex(ValueError, 'x_m'):
            self.store.upsert_pose(
                'robot_1',
                x_m=math.nan,
                y_m=0.0,
                yaw_rad=0.0,
                observed_at=None,
                received_at='2026-08-31T00:00:00Z',
            )
        with self.assertRaisesRegex(ValueError, 'battery_raw'):
            self.store.upsert_battery(
                'robot_1',
                battery_raw=65536,
                received_at='2026-08-31T00:00:00Z',
            )


class QuaternionToYawTest(unittest.TestCase):
    def test_returns_z_axis_heading_in_radians(self):
        self.assertAlmostEqual(
            quaternion_to_yaw(
                x=0.0,
                y=0.0,
                z=math.sqrt(0.5),
                w=math.sqrt(0.5),
            ),
            math.pi / 2,
        )


class FleetTelemetryRecorderTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TemporaryDirectory()
        self.database_path = Path(self.temporary_directory.name) / 'telemetry.db'
        self.store = TelemetryStore(self.database_path)

    def tearDown(self):
        self.store.close()
        self.temporary_directory.cleanup()

    def test_map_to_robot_base_transform_is_saved_as_current_pose(self):
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.5, y=-2.0),
                rotation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sqrt(0.5),
                    w=math.sqrt(0.5),
                ),
            ),
        )
        requested_robot_ids = []
        recorder = FleetTelemetryRecorder(
            self.store,
            lookup_transform=lambda robot_id: (
                requested_robot_ids.append(robot_id) or transform
            ),
            received_at=lambda: '2026-08-31T00:00:00Z',
        )

        recorder.record_latest_pose('robot_1')

        self.assertEqual(requested_robot_ids, ['robot_1'])
        row = self.store.fetch_pose('robot_1')
        self.assertEqual(
            {
                key: value
                for key, value in row.items()
                if key != 'yaw_rad'
            },
            {
                'robot_id': 'robot_1',
                'x_m': 1.5,
                'y_m': -2.0,
                'observed_at': '1970-01-01T00:00:12.000000500Z',
                'received_at': '2026-08-31T00:00:00Z',
            },
        )
        self.assertAlmostEqual(row['yaw_rad'], math.pi / 2)

    def test_battery_message_data_is_saved_for_its_robot(self):
        recorder = FleetTelemetryRecorder(
            self.store,
            lookup_transform=lambda _robot_id: None,
            received_at=lambda: '2026-08-31T00:00:00Z',
        )

        recorder.record_battery('robot_2', SimpleNamespace(data=8354))

        self.assertEqual(
            self.store.fetch_battery('robot_2'),
            {
                'robot_id': 'robot_2',
                'battery_raw': 8354,
                'received_at': '2026-08-31T00:00:00Z',
            },
        )

    def test_pose_uses_the_dynamic_tf_receipt_time_when_provided(self):
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=0)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        recorder = FleetTelemetryRecorder(
            self.store,
            lookup_transform=lambda _robot_id: transform,
            received_at=lambda: 'timer-write-time',
        )

        recorder.record_latest_pose(
            'robot_1',
            received_at='2026-08-31T00:00:00Z',
        )

        self.assertEqual(
            self.store.fetch_pose('robot_1')['received_at'],
            '2026-08-31T00:00:00Z',
        )


class TelemetryWriterArgumentsTest(unittest.TestCase):
    def test_cli_overrides_writer_storage_and_rate_without_consuming_ros_args(self):
        arguments, ros_arguments = _arguments([
            '--fleet-config',
            '/config/fleet.yaml',
            '--database-path',
            '/data/telemetry.db',
            '--pose-write-rate-hz',
            '2.5',
            '--tf-freshness-timeout-sec',
            '12.5',
            '--ros-args',
            '-p',
            'use_sim_time:=false',
        ])

        self.assertEqual(arguments.fleet_config, '/config/fleet.yaml')
        self.assertEqual(arguments.database_path, '/data/telemetry.db')
        self.assertEqual(arguments.pose_write_rate_hz, 2.5)
        self.assertEqual(arguments.tf_freshness_timeout_sec, 12.5)
        self.assertEqual(ros_arguments, ['--ros-args', '-p', 'use_sim_time:=false'])


class ActiveRobotIdsTest(unittest.TestCase):
    def test_disabled_vehicle_is_not_subscribed_or_persisted(self):
        fleet = FleetConfig(
            server=ServerConfig(
                domain_id=225,
                foxglove_port=8765,
                command_api=CommandApiConfig(host='127.0.0.1', port=8080),
            ),
            vehicles=(
                VehicleConfig(
                    id='robot_1',
                    foxglove_uri='ws://10.0.0.1:8766',
                    command_api_url='http://10.0.0.1:8082',
                    enabled=True,
                ),
                VehicleConfig(
                    id='robot_2',
                    foxglove_uri='ws://10.0.0.2:8766',
                    command_api_url='http://10.0.0.2:8082',
                    enabled=False,
                ),
            ),
        )

        self.assertEqual(active_robot_ids(fleet), ('robot_1',))


class TfFreshnessTest(unittest.TestCase):
    def test_stale_tf_is_not_treated_as_a_current_pose(self):
        now = [100.0]
        freshness = TfFreshness(
            timeout_sec=10.0,
            monotonic_time=lambda: now[0],
        )

        self.assertFalse(freshness.is_fresh('robot_1'))
        freshness.record('robot_1')
        now[0] = 109.9
        self.assertTrue(freshness.is_fresh('robot_1'))
        now[0] = 110.1
        self.assertFalse(freshness.is_fresh('robot_1'))


class TransformNamespaceTest(unittest.TestCase):
    def transform(self, parent: str, child: str):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=parent),
            child_frame_id=child,
        )

    def test_only_map_or_matching_robot_frames_are_accepted(self):
        self.assertTrue(transform_belongs_to_robot(
            self.transform('map', 'robot_1/odom'),
            'robot_1',
        ))
        self.assertTrue(transform_belongs_to_robot(
            self.transform('robot_1/odom', 'robot_1/base_footprint'),
            'robot_1',
        ))
        self.assertFalse(transform_belongs_to_robot(
            self.transform('map', 'robot_2/odom'),
            'robot_1',
        ))
        self.assertFalse(transform_belongs_to_robot(
            self.transform('odom', 'base_footprint'),
            'robot_1',
        ))


if __name__ == '__main__':
    unittest.main()
