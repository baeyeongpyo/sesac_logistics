#!/usr/bin/env python3
"""Terminal-only vehicle teleop using the GUI's existing planner/controller."""

import argparse
import curses
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace


# rclpy reads ROS_DOMAIN_ID when it initializes, so handle this option before
# importing any ROS modules (and before the bootstrap re-exec below).
_bootstrap_parser = argparse.ArgumentParser(add_help=False)
_bootstrap_parser.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
_bootstrap_parser.add_argument("--ros-domain-id", type=int, default=None)
_bootstrap_args, _ = _bootstrap_parser.parse_known_args()
_bootstrap_domain = _bootstrap_args.ros_domain_id or (214 + _bootstrap_args.vehicle)


if not os.environ.get("VEHICLE_GUI_ROS_READY"):
    setup = Path("/opt/ros/humble/setup.bash")
    workspace = Path("/home/ubuntu/ros2_ws/install/setup.bash")
    environment = os.environ.copy()
    environment["VEHICLE_GUI_ROS_READY"] = "1"
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
    os.execve(
        "/bin/bash",
        [
            "bash", "-lc",
            f"source {shlex.quote(str(setup))} && "
            f"source {shlex.quote(str(workspace))} && exec \"$@\"",
            "control-ui", "/usr/bin/python3", str(Path(__file__).resolve()),
            *sys.argv[1:],
        ],
        environment,
    )

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["ROS_DOMAIN_ID"] = str(_bootstrap_domain)
os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
os.environ.pop("ROS_DISCOVERY_SERVER", None)
os.environ.pop("CYCLONEDDS_URI", None)

import rclpy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QApplication

from vehicle_camera_teleop_gui import TeleopNode, TeleopWindow


SYMBOLS = ("star", "diamond", "spade", "clover", "heart")


def runtime_args(cli):
    return SimpleNamespace(
        vehicle=cli.vehicle, ros_domain_id=cli.ros_domain_id, webcam_ip="", image_topic="",
        secondary_image_topic="", secondary_video_url="", primary_video_url="",
        primary_video_command="", control_host="127.0.0.1", control_port=8091,
        control_url="", control_command="", webcam_1_video_url="",
        webcam_2_video_url="", scan_topic="/scan_raw", odom_topic="/odom_raw",
        motor_command_topic="/ros_robot_controller/set_motor",
        battery_topic="/ros_robot_controller/battery",
        detection_topic=f"/robot_{cli.vehicle}/symbol_seg/detections",
        cmd_vel_topic="/controller/cmd_vel", fork_command_topic="/fork/command",
        auto_dock_trigger_topic=f"/robot_{cli.vehicle}/nav2/arrival",
        auto_dock_stop_topic=f"/robot_{cli.vehicle}/auto_dock/stop",
        auto_dock_status_topic=f"/robot_{cli.vehicle}/auto_dock/status",
        output_dir=f"/home/ubuntu/recordings/vehicle{cli.vehicle}", linear_speed=cli.speed,
        angular_speed=cli.angular_speed, camera_pitch_deg=0.0,
        friction_coefficient=1.0, pose_config=cli.pose_config,
        stop_distance=0.20, safety_min_valid_range=0.25, record_fps=15.0,
        viewer_only=False, disable_external_webcams=True,
        disable_primary_camera=True, http_viewer_only=False,
    )


def set_target(window, left, right):
    window.target_left.setCurrentIndex(window.target_left.findData(left))
    window.target_right.setCurrentIndex(window.target_right.findData(right))
    window.on_target_changed()


def draw(stdscr, window, movement_name, fork_name):
    detection = window.node.latest_detection or {}
    candidate = detection.get("candidate") or {}
    pnp = candidate.get("pnp") or {}
    mode_text = "Auto Dock 1.2 ROS 클라이언트"
    lines = [
        f"control_ui 차량 {window.args.vehicle}  WASD: 전후/횡이동  Q/E: 회전  ↑/↓: 리프트  SPACE: 정지",
        "ENTER: Auto Dock 1.2 시작  O: 공통 설정 JSON  Z/SPACE: Auto Dock 정지",
        f"목표: {window.target_left.currentData()} / {window.target_right.currentData()}",
        f"중심선 보정: {window.centerline_offset_cm:+.1f} cm (+왼쪽/-오른쪽)",
        f"추가 주행보정: 횡이동 {window.lateral_overrun_cm:.1f} cm / "
        f"회전 {window.rotation_overrun_deg:.1f}° (+추가/-감소)",
        f"추가 삽입거리: {window.arc_insertion_distance.value():.1f} cm",
        f"모드: {mode_text}",
        f"속도: {window.linear.value()/window.linear.speed_scale:.2f} m/s  "
        f"회전: {window.angular.value()/window.angular.speed_scale:.2f} rad/s",
        f"수동 명령: {movement_name or '정지'}",
        f"리프트: {fork_name}",
        f"검출: streak={candidate.get('streak', '-')}  "
        f"거리={pnp.get('forward_distance_cm', '-')} cm  "
        f"reproj={pnp.get('reprojection_error_px', '-')} px",
        f"상태: {window.arc_label.text()}",
        f"시스템: {window.status.text()}",
    ]
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    for row, line in enumerate(lines[:height]):
        try:
            stdscr.addnstr(row, 0, line, max(0, width - 1))
        except curses.error:
            pass
    stdscr.refresh()


def terminal_loop(stdscr, window, app, key_timeout):
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.timeout(50)
    movement_deadline = 0.0
    movement_name = ""
    fork_name = "정지"
    movement_keys = {
        ord("w"): (Qt.Key_W, "전진"),
        ord("W"): (Qt.Key_W, "전진"),
        ord("s"): (Qt.Key_S, "후진"),
        ord("S"): (Qt.Key_S, "후진"),
        ord("a"): (Qt.Key_A, "좌 횡이동"),
        ord("A"): (Qt.Key_A, "좌 횡이동"),
        ord("d"): (Qt.Key_D, "우 횡이동"),
        ord("D"): (Qt.Key_D, "우 횡이동"),
        ord("q"): (Qt.Key_Q, "좌 회전"),
        ord("Q"): (Qt.Key_Q, "좌 회전"),
        ord("e"): (Qt.Key_E, "우 회전"),
        ord("E"): (Qt.Key_E, "우 회전"),
    }
    last_draw = 0.0
    while True:
        now = time.monotonic()
        key = stdscr.getch()
        if key == 27:
            break
        if key in movement_keys:
            qt_key, movement_name = movement_keys[key]
            if not (window.pressed & window.MOVEMENT_KEYS):
                window.node.stop_auto_dock()
            window.pressed.difference_update(window.MOVEMENT_KEYS)
            window.pressed.add(qt_key)
            movement_deadline = now + key_timeout
        elif key == curses.KEY_UP:
            window.node.publish_fork("UP")
            fork_name = "상승 중 (상단 리미트까지)"
        elif key == curses.KEY_DOWN:
            window.node.publish_fork("DOWN")
            fork_name = "하강 중 (하단 리미트까지)"
        elif key == ord(" "):
            window.emergency_stop()
            movement_deadline = 0.0
            movement_name = ""
            fork_name = "정지"
        elif key in (10, 13):
            window.pressed.difference_update(window.MOVEMENT_KEYS)
            window.node.stop(repeats=3)
            movement_name = ""
            window.run_selected_mode()
        elif key in (ord("z"), ord("Z")):
            window.stop_auto_dock_client()
            movement_name = ""
        elif key in (ord("o"), ord("O")):
            if window.arc_active or window.target_search_active:
                window.arc_label.setText("주행 또는 탐색 중에는 설정 파일을 열 수 없음")
                continue
            editor = shlex.split(os.environ.get("EDITOR", "gedit"))
            try:
                curses.def_prog_mode()
                curses.endwin()
                subprocess.run([*editor, str(window.rotation_calibration_path())], check=False)
                curses.reset_prog_mode()
                stdscr.refresh()
                window.load_rotation_calibration()
                window.arc_label.setText("공통 설정 JSON을 다시 읽음")
            except OSError as exc:
                window.arc_label.setText(f"설정 편집기 실행 실패: {exc}")
        if movement_deadline and now >= movement_deadline:
            window.pressed.difference_update(window.MOVEMENT_KEYS)
            window.node.stop(repeats=3)
            movement_deadline = 0.0
            movement_name = ""
        window.tick()
        app.processEvents()
        if now - last_draw >= 0.10:
            draw(stdscr, window, movement_name, fork_name)
            last_draw = now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", type=int, choices=(1, 2), default=1)
    parser.add_argument("--ros-domain-id", type=int, default=_bootstrap_domain)
    parser.add_argument("--left", choices=SYMBOLS, default="spade")
    parser.add_argument("--right", choices=SYMBOLS, default="spade")
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--key-timeout", type=float, default=0.35)
    parser.add_argument("--pose-config", default="/shared/vehicle_pose_config.json")
    cli = parser.parse_args()
    args = runtime_args(cli)
    rclpy.init()
    node = TeleopNode(args)
    app = QApplication([])
    window = TeleopWindow(node, args)
    window.timer.stop()
    set_target(window, cli.left, cli.right)
    try:
        curses.wrapper(terminal_loop, window, app, cli.key_timeout)
    finally:
        window.pressed.clear()
        window.cancel_arc_approach("control_ui 종료")
        node.stop(repeats=10)
        node.publish_fork("STOP")
        node.secondary_stream_stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
