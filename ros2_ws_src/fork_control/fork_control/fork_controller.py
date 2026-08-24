#!/usr/bin/env python3
"""Independent GPIO fork controller with ROS command/result topics."""

import json
import os
import time

import rclpy
from gpiozero import Button, Motor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String


class ForkController(Node):
    LIMIT_RELEASE_SEC = 0.4

    def __init__(self):
        super().__init__("fork_controller")
        self.declare_parameter("vehicle", 0)
        self.declare_parameter("command_topic", "/fork/command")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("entry_complete_topic", "")
        self.declare_parameter("lift_up_complete_topic", "")

        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError("vehicle must be 1/2, or ROS_DOMAIN_ID must be 215/216")
        robot = f"/robot_{self.vehicle}"
        state_topic = str(self.get_parameter("state_topic").value).strip() or f"{robot}/fork/state"
        entry_topic = str(self.get_parameter("entry_complete_topic").value).strip() or f"{robot}/auto_dock/entry_complete"
        legacy_complete_topic = str(
            self.get_parameter("lift_up_complete_topic").value
        ).strip() or f"{robot}/lift/up_complete"

        self.motor = Motor(forward=17, backward=18)
        self.lower_limit_switch = Button(27, pull_up=False, bounce_time=0.05)
        self.upper_limit_switch = Button(22, pull_up=False, bounce_time=0.05)
        self.lower_limit_latched = self.lower_limit_switch.is_pressed
        self.upper_limit_latched = self.upper_limit_switch.is_pressed
        self.lower_release_started_at = None
        self.upper_release_started_at = None
        self.active_command = "STOP"
        self.legacy_up_pending = False

        self.lower_limit_switch.when_pressed = self.lower_limit_pressed
        self.upper_limit_switch.when_pressed = self.upper_limit_pressed
        self.create_timer(0.05, self.update_limit_latches)
        self.create_subscription(
            String, str(self.get_parameter("command_topic").value),
            self.command_callback, 10,
        )
        self.create_subscription(Empty, entry_topic, self.entry_complete_callback, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.legacy_up_pub = self.create_publisher(Empty, legacy_complete_topic, 10)

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
        self.publish_state(f"{command}_COMPLETE")
        if command == "UP" and self.legacy_up_pending:
            self.legacy_up_pending = False
            self.legacy_up_pub.publish(Empty())

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

    def start_command(self, command, legacy_up=False):
        self.legacy_up_pending = bool(legacy_up and command == "UP")
        if command == "UP":
            if self.upper_limit_latched or self.upper_limit_switch.is_pressed:
                self.complete("UP")
                return
            self.motor.forward()
        elif command == "DOWN":
            if self.lower_limit_latched or self.lower_limit_switch.is_pressed:
                self.complete("DOWN")
                return
            self.motor.backward()
        self.active_command = command
        self.get_logger().info(f"Motor: {command}")

    def entry_complete_callback(self, _message):
        self.start_command("UP", legacy_up=True)

    def command_callback(self, message):
        command = message.data.strip().upper()
        if command in {"UP", "DOWN"}:
            self.start_command(command)
        elif command == "STOP":
            self.legacy_up_pending = False
            self.motor.stop()
            self.active_command = "STOP"
        else:
            self.motor.stop()
            self.active_command = "STOP"
            self.publish_state("FAILED", f"unknown command: {command}")

    def update_limit_latches(self):
        now = time.monotonic()
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
