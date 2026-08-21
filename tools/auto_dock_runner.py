#!/usr/bin/env python3
"""Headless ROS entrypoint for search, automatic docking, then fork lift."""

import argparse
import json
import os
import shlex
import sys
import time
from pathlib import Path
from types import SimpleNamespace


_bootstrap = argparse.ArgumentParser(add_help=False)
_bootstrap.add_argument("--ros-domain-id", type=int, default=215)
_bootstrap_args, _ = _bootstrap.parse_known_args()

if not os.environ.get("VEHICLE_GUI_ROS_READY"):
    ros_setup = next(
        (path for path in (Path("/opt/ros/humble/setup.bash"), Path("/opt/ros/jazzy/setup.bash")) if path.exists()),
        None,
    )
    if ros_setup is not None:
        workspace_setup = Path("/home/ubuntu/ros2_ws/install/setup.bash")
        commands = [f"source {shlex.quote(str(ros_setup))}"]
        if workspace_setup.exists():
            commands.append(f"source {shlex.quote(str(workspace_setup))}")
        environment = os.environ.copy()
        environment["VEHICLE_GUI_ROS_READY"] = "1"
        environment["QT_QPA_PLATFORM"] = "offscreen"
        environment["ROS_DOMAIN_ID"] = str(_bootstrap_args.ros_domain_id)
        os.execve(
            "/bin/bash",
            ["bash", "-lc", f'{" && ".join(commands)} && exec "$@"',
             "auto-dock", "/usr/bin/python3", str(Path(__file__).resolve()), *sys.argv[1:]],
            environment,
        )

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["ROS_DOMAIN_ID"] = str(_bootstrap_args.ros_domain_id)
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
os.environ.pop("ROS_DISCOVERY_SERVER", None)
os.environ.pop("CYCLONEDDS_URI", None)

import rclpy
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication
from std_msgs.msg import String

from vehicle_camera_teleop_gui import TeleopNode, TeleopWindow


SYMBOLS = ("star", "diamond", "spade", "clover", "heart")


def make_args(cli):
    robot_namespace = f"/robot_{cli.vehicle}"
    return SimpleNamespace(
        vehicle=cli.vehicle, ros_domain_id=cli.ros_domain_id,
        webcam_ip="", image_topic="", secondary_image_topic="", secondary_video_url="",
        primary_video_url="", primary_video_command="", control_host="127.0.0.1",
        control_port=8091, control_url="", control_command="", webcam_1_video_url="",
        webcam_2_video_url="", scan_topic=f"{robot_namespace}/scan_raw", odom_topic="/odom_raw",
        motor_command_topic="/ros_robot_controller/set_motor", battery_topic="/ros_robot_controller/battery",
        detection_topic=f"{robot_namespace}/symbol_seg/detections",
        cmd_vel_topic=f"{robot_namespace}/controller/cmd_vel", fork_command_topic="/fork/command",
        output_dir=f"/home/ubuntu/recordings/vehicle{cli.vehicle}", linear_speed=cli.speed,
        angular_speed=cli.angular_speed, camera_pitch_deg=0.0, friction_coefficient=1.0,
        pose_config=cli.pose_config, stop_distance=0.20, safety_min_valid_range=0.25,
        record_fps=15.0, viewer_only=False, disable_external_webcams=True,
        disable_primary_camera=True, http_viewer_only=False,
    )


class AutoDockRunner:
    def __init__(self, window, trigger_topic, trigger_value, status_topic, left, right):
        self.window = window
        self.default_target = (left, right)
        self.trigger_value = trigger_value.strip().lower()
        self.started_at = None
        self.last_status_at = 0.0
        self.status_pub = window.node.create_publisher(String, status_topic, 10)
        self.trigger_sub = window.node.create_subscription(
            String, trigger_topic, self.on_trigger, 10
        )
        self.publish_status("idle", "ready")

    def publish_status(self, state, reason, **extra):
        message = {"state": state, "reason": reason, "stamp_monotonic": time.monotonic(), **extra}
        self.status_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))

    def set_target(self, left, right):
        self.window.target_left.setCurrentIndex(self.window.target_left.findData(left))
        self.window.target_right.setCurrentIndex(self.window.target_right.findData(right))
        self.window.on_target_changed()

    def on_trigger(self, msg):
        """Temporary Nav2-arrival adapter; topic and value are CLI-configurable."""
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError("JSON object required")
        except (TypeError, ValueError, json.JSONDecodeError):
            command = {"command": str(msg.data).strip().lower()}
        action = str(command.get("command", "")).lower()
        if action == self.trigger_value:
            if self.started_at is not None:
                self.publish_status("rejected", "already_running")
                return
            left = command.get("left", self.default_target[0])
            right = command.get("right", self.default_target[1])
            if left not in SYMBOLS or right not in SYMBOLS:
                self.publish_status("rejected", "invalid_target_symbols")
                return
            self.set_target(left, right)
            self.started_at = time.monotonic()
            self.window.start_target_search(auto_lift_after_dock=True)
            self.publish_status("running", "search_started", left=left, right=right)
        elif action in ("cancel", "stop"):
            self.window.cancel_arc_approach("외부 cancel 명령")
            self.window.node.publish_fork("STOP")
            self.started_at = None
            self.publish_status("cancelled", "external_cancel")
        else:
            self.publish_status("ignored", "trigger_value_mismatch")

    def tick(self):
        self.window.tick()
        if self.started_at is None:
            return
        if self.window.last_auto_lift_monotonic is not None:
            elapsed = self.window.last_auto_lift_monotonic - self.started_at
            self.started_at = None
            self.publish_status("completed", "docked_and_lift_up", elapsed_sec=elapsed)
            return
        active = (
            self.window.target_search_active or self.window.arc_active
            or self.window.arc_cycle_replan_due_at is not None
            or self.window.arc_auto_replan_due_at is not None
        )
        if not active:
            self.started_at = None
            self.publish_status("failed", "controller_stopped_before_completion")
        elif time.monotonic() - self.last_status_at >= 1.0:
            self.last_status_at = time.monotonic()
            self.publish_status("running", self.window.arc_label.text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=int, choices=(1, 2), required=True)
    parser.add_argument("--ros-domain-id", type=int, default=_bootstrap_args.ros_domain_id)
    parser.add_argument("--left", choices=SYMBOLS, default="spade")
    parser.add_argument("--right", choices=SYMBOLS, default="spade")
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--pose-config", default="/shared/vehicle_pose_config.json")
    parser.add_argument(
        "--trigger-topic", default="",
        help="temporary Nav2-arrival String topic; replace later without code changes",
    )
    parser.add_argument(
        "--trigger-value", default="arrived",
        help="String payload that begins search/dock/lift (default: arrived)",
    )
    parser.add_argument("--status-topic", default="")
    cli = parser.parse_args()
    trigger_topic = cli.trigger_topic or f"/robot_{cli.vehicle}/nav2/arrival"
    status_topic = cli.status_topic or f"/robot_{cli.vehicle}/auto_dock/status"

    rclpy.init()
    app = QApplication([])
    node = TeleopNode(make_args(cli))
    window = TeleopWindow(node, make_args(cli))
    window.timer.stop()
    runner = AutoDockRunner(
        window, trigger_topic, cli.trigger_value, status_topic, cli.left, cli.right
    )
    timer = QTimer()
    timer.timeout.connect(runner.tick)
    timer.start(20)
    try:
        app.exec_()
    finally:
        window.cancel_arc_approach("auto_dock_runner 종료")
        node.stop(repeats=10)
        node.publish_fork("STOP")
        node.secondary_stream_stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
