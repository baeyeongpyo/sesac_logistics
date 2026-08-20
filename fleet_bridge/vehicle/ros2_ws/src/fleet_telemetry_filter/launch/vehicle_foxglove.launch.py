import os

from launch import LaunchDescription
from launch_ros.actions import Node

from fleet_bridge_config.loader import load_fleet, load_telemetry
from fleet_telemetry_filter.launch_config import bridge_parameters, forwarded_topics


def _required_environment(name):
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f'{name} environment variable is required')
    return value


def generate_launch_description():
    robot_id = _required_environment('ROBOT_ID')
    telemetry_config = os.environ.get('TELEMETRY_CONFIG', '/config/telemetry.yaml')
    fleet_config = os.environ.get('FLEET_CONFIG', '/config/fleet.yaml')
    mode = os.environ.get('FOXGLOVE_MODE', 'raw')
    default_port = '8766' if mode in {'raw', 'fleet'} else '8765'
    port = int(os.environ.get('FOXGLOVE_PORT', default_port))
    fleet = load_fleet(fleet_config, os.environ)
    vehicle = fleet.vehicle(robot_id)
    if not vehicle.enabled:
        raise RuntimeError(f'{robot_id} is disabled in {fleet_config}')
    topics = load_telemetry(telemetry_config, robot_id)
    parameters = bridge_parameters(topics, vehicle.command, mode=mode, port=port)

    actions = []
    if mode == 'fleet' and forwarded_topics(topics):
        actions.append(Node(
            package='fleet_telemetry_filter',
            executable='telemetry_filter',
            name='fleet_telemetry_filter',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'telemetry_config': telemetry_config,
            }],
        ))
    actions.append(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        parameters=[parameters],
    ))
    return LaunchDescription(actions)
