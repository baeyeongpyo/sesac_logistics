#!/usr/bin/env python3

import rclpy

from gpiozero import Button, Motor
from rclpy.node import Node
from std_msgs.msg import String


class ForkController(Node):
    def __init__(self) -> None:
        super().__init__("fork_controller")

        # 모터 드라이버 입력 핀
        self.motor = Motor(
            forward=17,
            backward=18,
        )

        # 리미트 스위치
        # pull_up=False
        # GPIO ---- 스위치 ---- 3.3V
        self.lower_limit_switch = Button(
            27,
            pull_up=False,
            bounce_time=0.05,
        )

        self.upper_limit_switch = Button(
            22,
            pull_up=False,
            bounce_time=0.05,
        )

        # 스위치가 눌리면 즉시 정지
        self.lower_limit_switch.when_pressed = (
            self.lower_limit_pressed
        )

        self.upper_limit_switch.when_pressed = (
            self.upper_limit_pressed
        )

        self.subscription = self.create_subscription(
            String,
            "/fork/command",
            self.command_callback,
            10,
        )

        self.get_logger().info(
            "Fork motor controller started."
        )

    def lower_limit_pressed(self) -> None:
        self.motor.stop()
        self.get_logger().warning(
            "Lower limit reached. Motor stopped."
        )

    def upper_limit_pressed(self) -> None:
        self.motor.stop()
        self.get_logger().warning(
            "Upper limit reached. Motor stopped."
        )

    def command_callback(self, message: String) -> None:
        command = message.data.strip().upper()

        self.get_logger().info(f"Received command: {command}")

        if command == "UP":
            # 이미 upper 스위치가 눌려 있는데 새로운 메시지가 온 경우
            if self.upper_limit_switch.is_pressed:
                self.motor.stop()
                self.get_logger().warning(
                    "UP blocked: upper limit switch pressed."
                )
                return

            self.motor.forward()
            self.get_logger().info("Motor: UP")

        elif command == "DOWN":
            # 이미 lower 스위치가 눌려 있는데 새로운 메시지가 온 경우
            if self.lower_limit_switch.is_pressed:
                self.motor.stop()
                self.get_logger().warning(
                    "DOWN blocked: lower limit switch pressed."
                )
                return

            self.motor.backward()
            self.get_logger().info("Motor: DOWN")

        elif command == "STOP":
            self.motor.stop()
            self.get_logger().info("Motor: STOP")

        else:
            self.motor.stop()
            self.get_logger().warning(
                f"Unknown command: {command}"
            )

    def cleanup(self) -> None:
        self.motor.stop()
        self.lower_limit_switch.close()
        self.upper_limit_switch.close()
        self.motor.close()


def main(args=None) -> None:
    rclpy.init()
    node = ForkController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Ctrl+C received. Stopping motor."
        )

    finally:
        node.cleanup()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

