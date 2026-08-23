#!/usr/bin/env python3

import os
import time

import rclpy

from gpiozero import Button, Motor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String


class ForkController(Node):
    LIMIT_RELEASE_SEC = 0.4

    def __init__(self) -> None:
        super().__init__("fork_controller")

        self.declare_parameter("vehicle", 0)
        self.declare_parameter("entry_complete_topic", "")
        self.declare_parameter("lift_up_complete_topic", "")
        requested_vehicle = int(self.get_parameter("vehicle").value)
        domain_vehicle = {215: 1, 216: 2}.get(
            int(os.environ.get("ROS_DOMAIN_ID", "0") or 0)
        )
        self.vehicle = requested_vehicle or domain_vehicle
        if self.vehicle not in (1, 2):
            raise RuntimeError(
                "vehicle must be 1/2, or ROS_DOMAIN_ID must be 215/216"
            )
        robot = f"/robot_{self.vehicle}"
        entry_complete_topic = (
            str(self.get_parameter("entry_complete_topic").value).strip()
            or f"{robot}/auto_dock/entry_complete"
        )
        lift_up_complete_topic = (
            str(self.get_parameter("lift_up_complete_topic").value).strip()
            or f"{robot}/lift/up_complete"
        )

        self.motor = Motor(forward=17, backward=18)
        self.lower_limit_switch = Button(27, pull_up=False, bounce_time=0.05)
        self.upper_limit_switch = Button(22, pull_up=False, bounce_time=0.05)

        self.lower_limit_latched = self.lower_limit_switch.is_pressed
        self.upper_limit_latched = self.upper_limit_switch.is_pressed
        self.lower_release_started_at = None
        self.upper_release_started_at = None
        self.active_command = "STOP"
        self.auto_lift_pending = False

        self.lower_limit_switch.when_pressed = self.lower_limit_pressed
        self.upper_limit_switch.when_pressed = self.upper_limit_pressed
        self.limit_latch_timer = self.create_timer(0.05, self.update_limit_latches)

        self.subscription = self.create_subscription(
            String, "/fork/command", self.command_callback, 10
        )
        self.entry_complete_subscription = self.create_subscription(
            Empty, entry_complete_topic, self.entry_complete_callback, 10
        )
        self.lift_up_complete_pub = self.create_publisher(
            Empty, lift_up_complete_topic, 10
        )

        if self.lower_limit_latched or self.upper_limit_latched:
            self.motor.stop()
        if self.lower_limit_latched:
            self.get_logger().info("Lower limit active at startup. Motor stopped.")
        if self.upper_limit_latched:
            self.get_logger().info("Upper limit active at startup. Motor stopped.")
        self.get_logger().info("Fork motor controller started.")

    def lower_limit_pressed(self) -> None:
        was_moving_down = self.active_command == "DOWN"
        self.motor.stop()
        self.active_command = "STOP"
        self.lower_release_started_at = None
        if self.lower_limit_latched:
            return
        self.lower_limit_latched = True
        if was_moving_down:
            self.get_logger().warning("Lower limit reached. Motor stopped.")

    def upper_limit_pressed(self) -> None:
        was_moving_up = self.active_command == "UP"
        self.motor.stop()
        self.active_command = "STOP"
        self.upper_release_started_at = None
        if self.upper_limit_latched:
            return
        self.upper_limit_latched = True
        if was_moving_up:
            self.get_logger().warning("Upper limit reached. Motor stopped.")
        if self.auto_lift_pending:
            self.auto_lift_pending = False
            self.lift_up_complete_pub.publish(Empty())
            self.get_logger().info("Published lift-up completion.")

    def entry_complete_callback(self, _message: Empty) -> None:
        if self.upper_limit_latched or self.upper_limit_switch.is_pressed:
            self.motor.stop()
            self.active_command = "STOP"
            self.auto_lift_pending = False
            self.lift_up_complete_pub.publish(Empty())
            self.get_logger().info(
                "Entry complete received; lift already up, completion published."
            )
            return
        self.auto_lift_pending = True
        self.motor.forward()
        self.active_command = "UP"
        self.get_logger().info("Entry complete received; motor lifting UP.")

    def update_limit_latches(self) -> None:
        now = time.monotonic()
        self.lower_release_started_at = self.update_limit_latch(
            "Lower",
            self.lower_limit_switch.is_pressed,
            self.lower_limit_latched,
            self.lower_release_started_at,
            now,
        )
        if self.lower_release_started_at is False:
            self.lower_limit_latched = False
            self.lower_release_started_at = None

        self.upper_release_started_at = self.update_limit_latch(
            "Upper",
            self.upper_limit_switch.is_pressed,
            self.upper_limit_latched,
            self.upper_release_started_at,
            now,
        )
        if self.upper_release_started_at is False:
            self.upper_limit_latched = False
            self.upper_release_started_at = None

    def update_limit_latch(
        self, name, is_pressed, is_latched, release_started_at, now
    ):
        if not is_latched or is_pressed:
            return None
        if release_started_at is None:
            return now
        if now - release_started_at < self.LIMIT_RELEASE_SEC:
            return release_started_at
        if self.active_command != "STOP":
            self.get_logger().info(f"{name} limit released and re-armed.")
        return False

    def command_callback(self, message: String) -> None:
        command = message.data.strip().upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "UP":
            self.auto_lift_pending = False
            if self.upper_limit_latched or self.upper_limit_switch.is_pressed:
                self.motor.stop()
                self.active_command = "STOP"
                self.get_logger().warning("UP blocked: upper limit switch pressed.")
                return
            self.motor.forward()
            self.active_command = "UP"
            self.get_logger().info("Motor: UP")
        elif command == "DOWN":
            self.auto_lift_pending = False
            if self.lower_limit_latched or self.lower_limit_switch.is_pressed:
                self.motor.stop()
                self.active_command = "STOP"
                self.get_logger().warning("DOWN blocked: lower limit switch pressed.")
                return
            self.motor.backward()
            self.active_command = "DOWN"
            self.get_logger().info("Motor: DOWN")
        elif command == "STOP":
            self.auto_lift_pending = False
            self.motor.stop()
            self.active_command = "STOP"
            self.get_logger().info("Motor: STOP")
        else:
            self.auto_lift_pending = False
            self.motor.stop()
            self.active_command = "STOP"
            self.get_logger().warning(f"Unknown command: {command}")

    def cleanup(self) -> None:
        self.motor.stop()
        self.lower_limit_switch.close()
        self.upper_limit_switch.close()
        self.motor.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Ctrl+C received. Stopping motor.")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
