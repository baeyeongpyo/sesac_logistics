import json
from pathlib import Path
import struct
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from foxglove_ros_worker.protocol import (
    Advertise,
    AdvertiseServices,
    IgnoredMessage,
    ProtocolError,
    ServerInfo,
    ServiceCallFailure,
    Unadvertise,
    client_advertise_message,
    client_message_frame,
    client_service_call_frame,
    parse_message_frame,
    parse_service_call_response_frame,
    parse_server_message,
    subscribe_message,
)


class FoxgloveProtocolTest(unittest.TestCase):
    def test_builds_client_advertise_and_message_data_frames(self):
        advertise = json.loads(client_advertise_message(
            channel_id=1,
            topic='/cmd_vel',
            schema_name='geometry_msgs/msg/Twist',
        ))
        frame = client_message_frame(1, b'cdr-payload')

        self.assertEqual(advertise, {
            'op': 'advertise',
            'channels': [{
                'id': 1,
                'topic': '/cmd_vel',
                'encoding': 'cdr',
                'schemaName': 'geometry_msgs/msg/Twist',
            }],
        })
        self.assertEqual(frame, b'\x01' + struct.pack('<I', 1) + b'cdr-payload')

    def test_builds_and_parses_cdr_service_call_frames(self):
        request = client_service_call_frame(
            service_id=7,
            call_id=12,
            encoding='cdr',
            payload=b'cancel-request',
        )
        response = parse_service_call_response_frame(
            b'\x03'
            + struct.pack('<III', 7, 12, 3)
            + b'cdr'
            + b'cancel-response',
        )

        self.assertEqual(
            request,
            b'\x02'
            + struct.pack('<III', 7, 12, 3)
            + b'cdr'
            + b'cancel-request',
        )
        self.assertEqual(response.service_id, 7)
        self.assertEqual(response.call_id, 12)
        self.assertEqual(response.encoding, 'cdr')
        self.assertEqual(response.payload, b'cancel-response')

    def test_parses_server_info_and_advertised_cdr_channels(self):
        server_info = parse_server_message(json.dumps({
            'op': 'serverInfo',
            'name': 'foxglove_bridge',
            'capabilities': [],
            'supportedEncodings': ['cdr'],
            'metadata': {'ROS_DISTRO': 'humble'},
            'sessionId': 'session-1',
        }))
        advertise = parse_server_message(json.dumps({
            'op': 'advertise',
            'channels': [
                {
                    'id': 3,
                    'topic': '/robot_1/odom',
                    'encoding': 'cdr',
                    'schemaName': 'nav_msgs/msg/Odometry',
                    'schema': 'schema body',
                    'schemaEncoding': 'ros2msg',
                },
            ],
        }))

        self.assertIsInstance(server_info, ServerInfo)
        self.assertEqual(server_info.supported_encodings, ('cdr',))
        self.assertIsInstance(advertise, Advertise)
        self.assertEqual(advertise.channels[0].id, 3)
        self.assertEqual(advertise.channels[0].topic, '/robot_1/odom')
        self.assertEqual(advertise.channels[0].schema_name, 'nav_msgs/msg/Odometry')

    def test_parses_advertised_services_and_service_call_failure(self):
        advertised = parse_server_message(json.dumps({
            'op': 'advertiseServices',
            'services': [{
                'id': 7,
                'name': '/navigate_to_pose/_action/cancel_goal',
                'type': 'action_msgs/srv/CancelGoal',
                'request': {
                    'encoding': 'cdr',
                    'schemaName': 'action_msgs/srv/CancelGoal_Request',
                    'schemaEncoding': 'ros2msg',
                    'schema': 'action_msgs/GoalInfo goal_info',
                },
                'response': {
                    'encoding': 'cdr',
                    'schemaName': 'action_msgs/srv/CancelGoal_Response',
                    'schemaEncoding': 'ros2msg',
                    'schema': 'int8 return_code',
                },
            }],
        }))
        failure = parse_server_message(json.dumps({
            'op': 'serviceCallFailure',
            'serviceId': 7,
            'callId': 12,
            'message': 'service unavailable',
        }))

        self.assertIsInstance(advertised, AdvertiseServices)
        self.assertEqual(advertised.services[0].id, 7)
        self.assertEqual(
            advertised.services[0].name,
            '/navigate_to_pose/_action/cancel_goal',
        )
        self.assertEqual(advertised.services[0].request_encoding, 'cdr')
        self.assertEqual(
            failure,
            ServiceCallFailure(7, 12, 'service unavailable'),
        )

    def test_parses_unadvertise_and_ignores_known_unneeded_operations(self):
        message = parse_server_message(json.dumps({
            'op': 'unadvertise',
            'channelIds': [4, 9],
        }))
        status = parse_server_message(json.dumps({
            'op': 'status',
            'level': 0,
            'message': 'ready',
        }))

        self.assertEqual(message, Unadvertise((4, 9)))
        self.assertIsInstance(status, IgnoredMessage)
        self.assertEqual(status.operation, 'status')

    def test_subscribe_message_assigns_client_ids_to_server_channels(self):
        payload = json.loads(subscribe_message(((11, 3), (12, 8))))

        self.assertEqual(payload, {
            'op': 'subscribe',
            'subscriptions': [
                {'id': 11, 'channelId': 3},
                {'id': 12, 'channelId': 8},
            ],
        })

    def test_parse_message_frame_extracts_subscription_timestamp_and_cdr(self):
        frame = parse_message_frame(
            b'\x01' + struct.pack('<IQ', 7, 1234) + b'cdr-payload',
        )

        self.assertEqual(frame.subscription_id, 7)
        self.assertEqual(frame.timestamp_ns, 1234)
        self.assertEqual(frame.payload, b'cdr-payload')

    def test_rejects_malformed_json_channels_and_binary_frames(self):
        invalid_values = [
            '[]',
            '{"name":"missing-op"}',
            '{"op":"advertise","channels":[{"id":1}]}',
        ]
        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(ProtocolError):
                    parse_server_message(value)

        for value in (b'', b'\x01short', b'\x02' + bytes(12)):
            with self.subTest(value=value):
                with self.assertRaises(ProtocolError):
                    parse_message_frame(value)


if __name__ == '__main__':
    unittest.main()
    client_advertise_message,
    client_message_frame,
