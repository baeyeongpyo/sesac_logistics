#!/usr/bin/env python3
"""Independent GPIO fork controller with ROS command/result topics."""

import json
import os
import time
from pathlib import Path

import rclpy
from gpiozero import Button, Motor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class ForkController(Node):
    LIMIT_RELEASE_SEC = 0.4

    def __init__(self):
        super().__init__("fork_controller")
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("command_topic", "/fork/command")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("pose_config", "/shared/vehicle_pose_config.json")

        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError("vehicle must be 1/2, or ROS_DOMAIN_ID must be 215/216")
        # robot = f"/robot_{self.vehicle}"
        # state_topic = str(self.get_parameter("state_topic").value).strip() or f"{robot}/fork/state"

        state_topic = str(self.get_parameter("state_topic").value).strip() or "/fork/state"

        self.motor = Motor(forward=17, backward=18)
        self.lower_limit_switch = Button(27, pull_up=False, bounce_time=0.05)
        self.upper_limit_switch = Button(22, pull_up=False, bounce_time=0.05)
        self.lower_limit_latched = self.lower_limit_switch.is_pressed
        self.upper_limit_latched = self.upper_limit_switch.is_pressed
        self.lower_release_started_at = None
        self.upper_release_started_at = None
        self.active_command = "STOP"
        self.up_started_at = None
        self.pose_config_path = Path(str(self.get_parameter("pose_config").value))
        self.runtime_config = {}
        self.runtime_config_mtime_ns = None
        self.refresh_runtime_config(force=True)

        self.lower_limit_switch.when_pressed = self.lower_limit_pressed
        self.upper_limit_switch.when_pressed = self.upper_limit_pressed
        self.create_timer(0.05, self.update_limit_latches)
        self.create_subscription(
            String, str(self.get_parameter("command_topic").value),
            self.command_callback, 10,
        )
        self.state_pub = self.create_publisher(String, state_topic, 10)

        if self.lower_limit_latched or self.upper_limit_latched:
            self.motor.stop()
        self.get_logger().info(
            f"Fork controller ready: command=/fork/command state={state_topic}"
        )

    def publish_state(self, state, error=""):
        payload = json.dumps({"state": state, "error": error}, separators=(",", ":"))
        self.state_pub.publish(String(data=payload))
        self.get_logger().info(f"Published fork state: {payload}")

    def complete(self, command):
        self.motor.stop()
        self.active_command = "STOP"
        self.up_started_at = None
        self.publish_state(f"{command}_COMPLETE")

    def refresh_runtime_config(self, force=False):
        try:
            mtime_ns = self.pose_config_path.stat().st_mtime_ns
            if not force and mtime_ns == self.runtime_config_mtime_ns:
                return
            payload = json.loads(self.pose_config_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("pose config must be a JSON object")
            self.runtime_config = payload
            self.runtime_config_mtime_ns = mtime_ns
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Fork runtime config read failed: {exc}")

    def runtime_boolean(self, key, default=False):
        value = self.runtime_config.get(key, default)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def runtime_number(self, key, default, minimum, maximum):
        try:
            value = float(self.runtime_config.get(key, default))
        except (TypeError, ValueError):
            value = float(default)
        return min(maximum, max(minimum, value))

    def lower_limit_pressed(self):
        was_moving_down = self.active_command == "DOWN"
        self.motor.stop()
        self.active_command = "STOP"
        self.lower_release_started_at = None
        if self.lower_limit_latched:
            return
        self.lower_limit_latched = True
        if was_moving_down:
            self.complete("DOWN")

    def upper_limit_pressed(self):
        was_moving_up = self.active_command == "UP"
        self.motor.stop()
        self.active_command = "STOP"
        self.upper_release_started_at = None
        if self.upper_limit_latched:
            return
        self.upper_limit_latched = True
        if was_moving_up:
            self.complete("UP")

    def start_command(self, command):
        # Record intent before energizing the motor.  A limit callback can run
        # immediately from the GPIO thread when motion begins; it must see the
        # command that caused the motion so it can publish *_COMPLETE.
        self.active_command = command
        if command == "UP":
            self.up_started_at = time.monotonic()
            if self.upper_limit_latched or self.upper_limit_switch.is_pressed:
                self.complete("UP")
                return
            self.motor.forward()
        elif command == "DOWN":
            self.up_started_at = None
            if self.lower_limit_latched or self.lower_limit_switch.is_pressed:
                self.complete("DOWN")
                return
            self.motor.backward()
        self.get_logger().info(f"Motor: {command}")

    def command_callback(self, message):
        command = message.data.strip().upper()
        if command in {"UP", "DOWN"}:
            self.start_command(command)
        elif command == "STOP":
            self.motor.stop()
            self.active_command = "STOP"
            self.up_started_at = None
        else:
            self.motor.stop()
            self.active_command = "STOP"
            self.up_started_at = None
            self.publish_state("FAILED", f"unknown command: {command}")

    def update_limit_latches(self):
        now = time.monotonic()
        self.refresh_runtime_config()
        # Polling backs up gpiozero's edge callback so a completion event is
        # not lost if the limit changes during motor startup or contact bounce.
        if self.active_command == "UP" and self.upper_limit_switch.is_pressed:
            self.upper_limit_latched = True
            self.complete("UP")
        elif self.active_command == "DOWN" and self.lower_limit_switch.is_pressed:
            self.lower_limit_latched = True
            self.complete("DOWN")
        elif (
            self.active_command == "UP"
            and self.up_started_at is not None
            and self.runtime_boolean("fork_timed_up_complete_enabled", False)
            and now - self.up_started_at >= self.runtime_number(
                "fork_timed_up_complete_sec", 3.0, 0.5, 10.0
            )
        ):
            self.get_logger().warning(
                "Using temporary timed UP completion fallback; upper limit was not seen"
            )
            self.complete("UP")
        self.lower_release_started_at = self.update_limit_latch(
            self.lower_limit_switch.is_pressed, self.lower_limit_latched,
            self.lower_release_started_at, now, "lower",
        )
        self.upper_release_started_at = self.update_limit_latch(
            self.upper_limit_switch.is_pressed, self.upper_limit_latched,
            self.upper_release_started_at, now, "upper",
        )

    def update_limit_latch(self, pressed, latched, release_started_at, now, which):
        if not latched or pressed:
            return None
        if release_started_at is None:
            return now
        if now - release_started_at < self.LIMIT_RELEASE_SEC:
            return release_started_at
        if which == "lower":
            self.lower_limit_latched = False
        else:
            self.upper_limit_latched = False
        return None

    def cleanup(self):
        self.motor.stop()
        self.lower_limit_switch.close()
        self.upper_limit_switch.close()
        self.motor.close()


def main(args=None):
    rclpy.init(args=args)
    node = ForkController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
