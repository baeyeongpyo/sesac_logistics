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

from vehicle_camera_teleop_gui import DevControlClientNode, TeleopWindow


SYMBOLS = ("star", "diamond", "spade", "clover", "heart")

AUTO_DOCK_STATE_TEXT = {
    "IDLE": "대기",
    "SEARCHING": "탐색",
    "ALIGNING": "정렬",
    "INSERTING": "삽입",
    "WAIT_UP_COMPLETE": "포크 상승 대기",
    "WAIT_DOWN_COMPLETE": "포크 하강 대기",
    "REVERSING": "후진",
    "TURNING": "회전",
    "READY": "완료/READY",
    "ERROR": "오류/정지",
}

AUTO_DOCK_REASON_TEXT = {
    "ready": "시작 대기",
    "emergency_stop": "긴급정지됨",
    "tag_guided_search_depth_missing": "전면 태그 depth 없음",
    "rear_lidar_search_scan_stale": "후방 LiDAR 수신 지연으로 정지",
    "rear_lidar_search_distance_missing": "후방 LiDAR 거리 없음",
    "rear_lidar_distance_correction": "후방이 가까워 전진 중",
    "rear_lidar_yaw_correction_before_lateral": "횡이동 전 시작 yaw 복구 중",
    "rear_lidar_clearance_held_lateral_search": "후방 30cm 유지하며 횡탐색 중",
    "front_tag_distance_correction": "전면 태그 거리 맞추며 전진 중",
    "front_tag_pose_held_lateral_search": "전면 태그 기준 횡탐색 중",
    "candidate_paused_for_confirmation": "후보 확인을 위해 정지",
    "candidate_confirmation": "후보 유효성 확인 중",
    "candidate_lost_resume_search": "후보가 사라져 다시 탐색",
    "candidate_invalid_resume_search": "후보가 무효라 다시 탐색",
    "nearest_target_lost_resume_search": "최근접 후보를 잃어 다시 탐색",
    "edge_target_coarse_alignment": "후보 잠금 후 1차 정렬 중",
    "coarse_centering": "화면 중심으로 횡·yaw 정렬 중",
    "coarse_locked_target_temporarily_lost": "잠근 후보 재검출 대기",
    "coarse_target_lost_using_virtual_target": "가상 목표로 정렬 계속",
    "coarse_alignment_continuing_locked_target": "잠근 목표 정렬 계속",
    "virtual_target_locked_docking": "가상 목표 잠금 완료",
    "translation_first_alignment": "전진·횡·yaw 동시 정렬 중",
    "lidar_backoff": "LiDAR 장애물 반대 방향 회피 중",
    "lidar_replanned_virtual_dock": "LiDAR 회피 후 정렬 재개",
    "aligned_pause_before_insertion": "정렬 완료, 삽입 전 정지",
    "aligned_inserting": "팔레트 삽입 시작",
}


def auto_dock_status_text(status):
    state = str(status.get("state", "UNKNOWN")).upper()
    reason = str(status.get("reason", "no_status"))
    state_text = AUTO_DOCK_STATE_TEXT.get(state, state)
    reason_text = AUTO_DOCK_REASON_TEXT.get(reason, reason)
    stamp = status.get("stamp_monotonic")
    try:
        age_text = f" | {max(0.0, time.monotonic() - float(stamp)):.1f}초 전"
    except (TypeError, ValueError):
        age_text = ""
    location = status.get("location")
    location_text = f" | {location}" if location else ""
    return f"현재 AUTO-DOCK: [{state_text}] {reason_text}{location_text}{age_text}"


def runtime_args(cli):
    return SimpleNamespace(
        vehicle=cli.vehicle, ros_domain_id=cli.ros_domain_id, webcam_ip="", image_topic="",
        tape_image_topic="/ascamera/camera_publisher/rgb0/image",
        secondary_image_topic="", secondary_video_url="", primary_video_url="",
        primary_video_command="", control_host="127.0.0.1", control_port=8091,
        control_url="", control_command="", webcam_1_video_url="",
        webcam_2_video_url="", scan_topic="/scan_raw", odom_topic="/odom_raw",
        motor_command_topic="/ros_robot_controller/set_motor",
        battery_topic="/ros_robot_controller/battery",
        detection_topic=f"/robot_{cli.vehicle}/symbol_seg/detections",
        cmd_vel_topic="/controller/cmd_vel", fork_command_topic="/fork/command",
        arrival_topic=f"/robot_{cli.vehicle}/nav2/arrival",
        auto_dock_stop_topic=f"/robot_{cli.vehicle}/auto_dock/stop",
        auto_dock_status_topic=f"/robot_{cli.vehicle}/auto_dock/status",
        dock_inventory_topic=f"/robot_{cli.vehicle}/dock/inventory",
        dock_inventory_reset_topic=f"/robot_{cli.vehicle}/dock/inventory/reset",
        map_topic="/map", entity_map_topic=f"/robot_{cli.vehicle}/tag_entity_map",
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
    if window.terminal_run_mode == "search":
        mode_text = "Auto Dock Arrival → 자동 탐색/정렬"
    elif window.terminal_run_mode == "auto":
        mode_text = "무제한 자동 사이클 → 자동 삽입"
    elif window.arc_cycle_limit == 3:
        mode_text = "사이클 (최대 3회, 삽입 전 정지)"
    else:
        mode_text = "단일 (계산·주행 1회 후 삽입 전 정지)"
    lines = [
        f"control_ui 차량 {window.args.vehicle}  WASD: 전후/횡이동  Q/E: 회전  ↑/↓: 리프트  SPACE: 정지",
        auto_dock_status_text(window.node.auto_dock_status),
        ".: 주행계산  ENTER/P: Auto Dock Arrival  K: Auto Dock Stop  M: 모드  O: 설정  Z: 취소",
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
        elif key == ord("."):
            window.pressed.difference_update(window.MOVEMENT_KEYS)
            movement_name = ""
            if window.terminal_run_mode != "search":
                window.cancel_arc_approach("새 주행 계산")
                window.plan_arc_approach()
        elif key in (10, 13):
            window.pressed.difference_update(window.MOVEMENT_KEYS)
            window.node.stop(repeats=3)
            movement_name = ""
            if window.terminal_run_mode == "search":
                window.publish_arrival_trigger()
            else:
                window.start_arc_approach()
        elif key in (ord("z"), ord("Z")):
            window.cancel_arc_approach("터미널 사용자 취소")
            movement_name = ""
        elif key in (ord("p"), ord("P")):
            window.publish_arrival_trigger()
        elif key in (ord("k"), ord("K")):
            window.publish_auto_dock_stop()
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
        elif key in (ord("m"), ord("M")):
            if (
                window.arc_active
                or window.target_search_active
                or window.arc_cycle_replan_due_at is not None
            ):
                window.arc_label.setText("주행 중에는 모드 변경 불가")
            else:
                current = window.terminal_run_mode
                if current == "auto":
                    window.terminal_run_mode = "single"
                    window.arc_cycle_limit = 1
                    window.arc_auto_insert_after_verify = False
                    mode = "단일"
                elif current == "single":
                    window.terminal_run_mode = "cycle3"
                    window.arc_cycle_limit = 3
                    window.arc_auto_insert_after_verify = False
                    mode = "사이클(최대 3회, 삽입 전 정지)"
                elif current == "cycle3":
                    window.terminal_run_mode = "search"
                    window.arc_cycle_limit = 0
                    window.arc_auto_insert_after_verify = True
                    mode = "원형 탐색 → 발견 시 무제한 자동"
                else:
                    window.terminal_run_mode = "auto"
                    window.arc_cycle_limit = 0
                    window.arc_auto_insert_after_verify = True
                    mode = "무제한 자동 사이클 → 자동 삽입"
                window.arc_label.setText(f"주행 모드: {mode}")
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
    node = DevControlClientNode(args)
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
