#!/usr/bin/env python3
"""Headless ROS entrypoint for search, automatic docking, then fork lift."""

import argparse
import json
import math
import os
import shlex
import sys
import time
from pathlib import Path
from types import SimpleNamespace


_bootstrap = argparse.ArgumentParser(add_help=False)
_bootstrap.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
_bootstrap.add_argument("--ros-domain-id", type=int, default=None)
_bootstrap_args, _ = _bootstrap.parse_known_args()
_bootstrap_domain = _bootstrap_args.ros_domain_id or 214 + _bootstrap_args.vehicle

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
        environment["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
        os.execve(
            "/bin/bash",
            ["bash", "-lc", f'{" && ".join(commands)} && exec "$@"',
             "auto-dock", "/usr/bin/python3", str(Path(__file__).resolve()), *sys.argv[1:]],
            environment,
        )

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
os.environ.pop("ROS_DISCOVERY_SERVER", None)
os.environ.pop("CYCLONEDDS_URI", None)

import rclpy
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication
from std_msgs.msg import Empty, String

from vehicle_camera_teleop_gui import TeleopNode, TeleopWindow, VEHICLE_HOSTS


SYMBOLS = ("star", "diamond", "spade", "clover", "heart")


def make_args(cli):
    robot_namespace = f"/robot_{cli.vehicle}"
    return SimpleNamespace(
        vehicle=cli.vehicle, ros_domain_id=cli.ros_domain_id, node_name="auto_dock",
        webcam_ip="", image_topic="", secondary_image_topic="", secondary_video_url="",
        primary_video_url="", primary_video_command="", control_host=VEHICLE_HOSTS[cli.vehicle],
        control_port=8091, control_url="", control_command="", webcam_1_video_url="",
        webcam_2_video_url="", scan_topic="/scan_raw", odom_topic="/odom_raw",
        motor_command_topic="/ros_robot_controller/set_motor", battery_topic="/ros_robot_controller/battery",
        detection_topic=f"{robot_namespace}/symbol_seg/detections",
        cmd_vel_topic="/controller/cmd_vel", fork_command_topic="/fork/command",
        output_dir=f"/home/ubuntu/recordings/vehicle{cli.vehicle}", linear_speed=cli.speed,
        angular_speed=cli.angular_speed, camera_pitch_deg=0.0, friction_coefficient=1.0,
        pose_config=cli.pose_config, stop_distance=0.20, safety_min_valid_range=0.25,
        lidar_self_filter_distance_m=0.25,
        search_linear_speed_m_s=cli.search_linear_speed,
        config_overrides=cli.config_overrides,
        record_fps=15.0, viewer_only=False, disable_external_webcams=True,
        disable_primary_camera=True, http_viewer_only=False,
    )


class AutoDockRunner:
    def __init__(self, window, trigger_topic, stop_topic, trigger_value, status_topic, left, right):
        self.window = window
        self.default_target = (left, right)
        self.trigger_value = trigger_value.strip().lower()
        self.started_at = None
        self.recovery_until = None
        self.recovery_was_docking = False
        self.stop_latched = False
        self.last_status_signature = None
        self.status_pub = window.node.create_publisher(String, status_topic, 10)
        self.trigger_sub = window.node.create_subscription(
            String, trigger_topic, self.on_trigger, 10
        )
        self.stop_sub = window.node.create_subscription(
            Empty, stop_topic, self.on_emergency_stop, 10
        )
        self.publish_status("idle", "ready")

    def emergency_stop(self, reason):
        """Disarm autonomous output until a new arrival command is received."""
        self.stop_latched = True
        self.recovery_until = None
        self.window.cancel_arc_approach("외부 즉시 정지 명령")
        self.window.node.stop(repeats=10)
        self.window.node.publish_fork("STOP")
        self.started_at = None
        self.publish_status("cancelled", reason)

    def on_emergency_stop(self, _msg):
        self.emergency_stop("emergency_stop")

    def publish_status(self, state, reason, **extra):
        signature = (state, reason, tuple(sorted(extra.items())))
        if signature == self.last_status_signature:
            return
        self.last_status_signature = signature
        message = {"state": state, "reason": reason, "stamp_monotonic": time.monotonic(), **extra}
        self.status_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))

    def set_target(self, left, right):
        mode_index = self.window.approach_mode.findData("arc")
        if mode_index >= 0:
            self.window.approach_mode.setCurrentIndex(mode_index)
        self.window.target_left.setCurrentIndex(self.window.target_left.findData(left))
        self.window.target_right.setCurrentIndex(self.window.target_right.findData(right))
        self.window.on_target_changed()

    def on_trigger(self, msg):
        """Temporary Nav2-arrival adapter; topic and value are CLI-configurable."""
        raw = str(msg.data).strip()
        # Deliberately keep the temporary integration human-readable:
        # "arrived diamond spade" = trigger, left tag, right tag.
        fields = raw.split()
        if len(fields) in (1, 3) and fields and fields[0].lower() in (
            self.trigger_value, "cancel", "stop",
        ):
            command = {"command": fields[0].lower()}
            if len(fields) == 3:
                command.update({"left": fields[1].lower(), "right": fields[2].lower()})
        else:
            try:
                command = json.loads(raw)
                if not isinstance(command, dict):
                    raise ValueError("JSON object required")
            except (TypeError, ValueError, json.JSONDecodeError):
                command = {"command": raw.lower()}
        action = str(command.get("command", "")).lower()
        if action == self.trigger_value:
            if self.started_at is not None:
                self.publish_status("rejected", "already_running")
                return
            self.window.load_rotation_calibration()
            left = command.get("left", self.default_target[0])
            right = command.get("right", self.default_target[1])
            if left not in SYMBOLS or right not in SYMBOLS:
                self.publish_status("rejected", "invalid_target_symbols")
                return
            self.set_target(left, right)
            self.stop_latched = False
            self.started_at = time.monotonic()
            self.window.start_target_search(auto_lift_after_dock=True)
            self.publish_status("running", "search_started", left=left, right=right)
        elif action in ("cancel", "stop"):
            self.emergency_stop("external_cancel")
        else:
            self.publish_status("ignored", "trigger_value_mismatch")

    def tick(self):
        if self.stop_latched:
            return
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
            if (self.window.last_arc_stop_reason or "").startswith("LiDAR interrupt"):
                now = time.monotonic()
                if self.recovery_until is None:
                    angle = self.window.node.closest_obstacle_angle_rad
                    angle = 0.0 if angle is None else angle
                    # Move away from the closest filtered LiDAR return.
                    self.recovery_was_docking = self.window.last_arc_stop_was_docking
                    self.recovery_until = now + 0.7
                    self.recovery_command = (
                        -0.08 * math.cos(angle), -0.08 * math.sin(angle)
                    )
                    self.publish_status("recovering", "lidar_backoff")
                if now < self.recovery_until:
                    self.window.node.publish(*self.recovery_command, 0.0)
                    return
                self.window.node.stop(repeats=5)
                self.recovery_until = None
                self.window.last_arc_stop_reason = None
                if self.recovery_was_docking and self.window.replan_arc_from_virtual_target():
                    self.window.start_arc_approach()
                    self.publish_status("running", "lidar_replanned_virtual_dock")
                else:
                    self.window.start_target_search(auto_lift_after_dock=True)
                    self.publish_status("running", "lidar_recovery_search")
                return
            # A temporary camera/odom/docking failure must not consume the
            # Nav2-arrival request.  Keep searching until an explicit cancel
            # arrives, so the pallet can be reacquired after it re-enters view.
            self.window.start_target_search(auto_lift_after_dock=True)
            self.publish_status("running", "search_restarted_after_controller_stop")
        else:
            phase = (
                "searching"
                if self.window.target_search_active
                else f"docking_{self.window.arc_dock_phase or 'replanning'}"
            )
            self.publish_status("running", phase)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=int, choices=(1, 2), required=True)
    parser.add_argument("--ros-domain-id", type=int, default=_bootstrap_domain)
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
    parser.add_argument("--stop-topic", default="")
    parser.add_argument(
        "--search-linear-speed", type=float, default=0.0,
        help="temporary search speed override in m/s; 0 uses pose config",
    )
    parser.add_argument(
        "--config-overrides", default="{}",
        help="temporary JSON object merged over pose config for this run only",
    )
    cli = parser.parse_args()
    try:
        cli.config_overrides = json.loads(cli.config_overrides)
        if not isinstance(cli.config_overrides, dict):
            raise ValueError("JSON object required")
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        parser.error(f"--config-overrides must be a JSON object: {exc}")
    trigger_topic = cli.trigger_topic or f"/robot_{cli.vehicle}/nav2/arrival"
    status_topic = cli.status_topic or f"/robot_{cli.vehicle}/auto_dock/status"
    stop_topic = cli.stop_topic or f"/robot_{cli.vehicle}/auto_dock/stop"

    rclpy.init()
    app = QApplication([])
    node = TeleopNode(make_args(cli))
    window = TeleopWindow(node, make_args(cli))
    window.timer.stop()
    runner = AutoDockRunner(
        window, trigger_topic, stop_topic, cli.trigger_value, status_topic, cli.left, cli.right
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
