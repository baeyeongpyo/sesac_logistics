"""ROS 2 bridge for a Yahboom DOFBOT controller on I2C bus 1."""

from __future__ import annotations

from typing import Sequence
import math
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray

from .Arm_Lib import Arm_Device


HARD_MINIMUMS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
HARD_MAXIMUMS = (180.0, 180.0, 180.0, 180.0, 270.0, 180.0)


class DofbotDriver(Node):
    """Accept six target joint angles and send them to the DOFBOT board."""

    def __init__(self) -> None:
        super().__init__('dofbot')
        self.declare_parameter('motion_time_ms', 250)
        self.declare_parameter(
            'limits_file', '/home/intelions/ros2_ws/config/dofbot_limits.json'
        )
        self._arm = Arm_Device()
        self._last_angles: list[float | None] = [None] * 6
        self._limits_file = Path(self.get_parameter('limits_file').value)
        self._minimums, self._maximums = self._load_limits()
        self._command_subscription = self.create_subscription(
            Float64MultiArray,
            'dofbot/command_joint_angles',
            self._on_joint_command,
            10,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState, 'dofbot/joint_states', 10
        )
        # Compatibility interfaces used by both DOFBOT GUIs.
        self._arm_angles_publisher = self.create_publisher(
            Float32MultiArray, '/arm/joint_angles', 10
        )
        self._move_all_subscription = self.create_subscription(
            Float32MultiArray, '/arm/move_all', self._on_move_all, 10
        )
        self._torque_cmd_subscription = self.create_subscription(
            Bool, '/arm/torque_cmd', self._on_torque, 10
        )
        # Six SMBus reads are relatively expensive; 5 Hz is sufficient for teaching.
        self._joint_state_timer = self.create_timer(0.2, self._publish_joint_state)
        self._limits_subscription = self.create_subscription(
            Float64MultiArray,
            'dofbot/safety_limits',
            self._on_safety_limits,
            10,
        )
        self._torque_subscription = self.create_subscription(
            Bool,
            "dofbot/set_torque",
            self._on_torque,
            10,
        )
        self.get_logger().info(
            'DOFBOT ready on I2C bus 1 (controller version %s). '
            'Waiting for dofbot/command_joint_angles.'
            % self._arm.Arm_get_hardversion()
        )

    def _on_joint_command(self, message: Float64MultiArray) -> None:
        angles = list(message.data)
        if not self._valid_angles(angles):
            self.get_logger().error(
                f'Command is outside calibrated limits: '
                f'min={self._minimums}, max={self._maximums}.'
            )
            return

        duration_ms = int(self.get_parameter('motion_time_ms').value)
        duration_ms = max(1, min(duration_ms, 30_000))
        target = [int(round(angle)) for angle in angles]
        self._arm.Arm_serial_set_torque(1)
        self._arm.Arm_serial_servo_write6(*target, duration_ms)
        self.get_logger().info(
            f'Commanded joints {target} over {duration_ms} ms.'
        )

    def _on_move_all(self, message: Float32MultiArray) -> None:
        if len(message.data) < 6:
            self.get_logger().error('/arm/move_all needs six angles and optional duration_ms.')
            return
        angles = [float(value) for value in message.data[:6]]
        if not self._valid_angles(angles):
            self.get_logger().error('Rejected /arm/move_all outside calibrated limits.')
            return
        duration_ms = (int(round(message.data[6])) if len(message.data) >= 7
                       else int(self.get_parameter('motion_time_ms').value))
        duration_ms = max(100, min(duration_ms, 30_000))
        target = [int(round(angle)) for angle in angles]
        self._arm.Arm_serial_set_torque(1)
        self._arm.Arm_serial_servo_write6(*target, duration_ms)

    def _publish_joint_state(self) -> None:
        readings = [self._arm.Arm_serial_servo_read(index) for index in range(1, 7)]
        for index, value in enumerate(readings):
            if value is not None:
                self._last_angles[index] = float(value)
        angles = self._last_angles.copy()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'joint_{index}' for index in range(1, 7)]
        message.position = [math.radians(angle) if angle is not None else math.nan for angle in angles]
        self._joint_state_publisher.publish(message)
        self._arm_angles_publisher.publish(Float32MultiArray(
            data=[float(angle) if angle is not None else -1.0 for angle in angles]
        ))

    def _on_torque(self, message: Bool) -> None:
        self._arm.Arm_serial_set_torque(1 if message.data else 0)
        self.get_logger().info(f"Torque {'ON' if message.data else 'OFF'}.")

    def _on_safety_limits(self, message: Float64MultiArray) -> None:
        if len(message.data) != 12:
            self.get_logger().error('Safety limits must contain six minimums then six maximums.')
            return
        minimums = tuple(message.data[:6])
        maximums = tuple(message.data[6:])
        if not self._valid_limits(minimums, maximums):
            self.get_logger().error('Rejected invalid safety limits.')
            return
        self._minimums, self._maximums = minimums, maximums
        self.get_logger().info(f'Updated safety limits: min={minimums}, max={maximums}.')

    def _load_limits(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        try:
            data = json.loads(self._limits_file.read_text())
            minimums = tuple(float(value) for value in data['minimums'])
            maximums = tuple(float(value) for value in data['maximums'])
            if self._valid_limits(minimums, maximums):
                return minimums, maximums
        except (FileNotFoundError, ValueError, KeyError, json.JSONDecodeError):
            pass
        return HARD_MINIMUMS, HARD_MAXIMUMS

    @staticmethod
    def _valid_limits(minimums: Sequence[float], maximums: Sequence[float]) -> bool:
        return (
            len(minimums) == 6
            and len(maximums) == 6
            and all(
                hard_min <= minimum < maximum <= hard_max
                for minimum, maximum, hard_min, hard_max in zip(
                    minimums, maximums, HARD_MINIMUMS, HARD_MAXIMUMS
                )
            )
        )

    def _valid_angles(self, angles: Sequence[float]) -> bool:
        return (
            len(angles) == 6
            and all(isinstance(angle, (int, float)) for angle in angles)
            and all(
                minimum <= angle <= maximum
                for angle, minimum, maximum in zip(
                    angles, self._minimums, self._maximums
                )
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DofbotDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
