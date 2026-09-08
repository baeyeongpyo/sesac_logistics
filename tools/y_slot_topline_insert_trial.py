#!/usr/bin/env python3
"""Standalone Y-slot 30 cm insertion trial using approach-measured yaw gains.

Start this script before issuing the GUI's Y stage-only command.  It records
the actual turn/forward response on the way to the 40 cm staging pose, freezes
a new top-line observation there, fits left/right yaw gains, and plans the
remaining 30 cm approach.  No AutoDock source or state machine is modified.
Without ``--execute`` the resulting plan is printed but never published.
"""

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

from auto_dock.auto_dock_node import (
    YSlotTracker,
    YTopLineTracker,
    y_slot_floor_pose,
)
from auto_dock.loaded_response_planner import (
    ResponseCoefficients,
    ResponseState,
    advance_state,
    plan_approach,
)


STATUS_TOPIC = "/auto_dock/status"
COMMAND_TOPIC = "/controller/cmd_vel"
CAMERA_TOPIC = "/ascamera/camera_publisher/rgb0/image"
IMU_TOPIC = "/imu"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _yaw_interpolator(yaw_rows):
    values = np.asarray(yaw_rows, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 2:
        raise ValueError("insufficient_imu_history")
    return values[:, 0], np.unwrap(values[:, 1])


def measured_directional_gains(command_rows, yaw_rows, end_time, fallback):
    """Fit immediate/total yaw gain for each signed turn in one approach."""
    commands = sorted(command_rows)
    if not commands:
        return fallback, {
            "left_samples": 0,
            "right_samples": 0,
            "left_immediate_gain": fallback.yaw_gains(1.0)[0],
            "left_total_gain": fallback.yaw_gains(1.0)[1],
            "right_immediate_gain": fallback.yaw_gains(-1.0)[0],
            "right_total_gain": fallback.yaw_gains(-1.0)[1],
        }
    edges = []
    for row in commands:
        current = tuple(float(value) for value in row[1:])
        if not edges or current != edges[-1][1]:
            edges.append((float(row[0]), current))
    if edges[-1][0] < end_time:
        edges.append((float(end_time), edges[-1][1]))
    yaw_time, yaw_value = _yaw_interpolator(yaw_rows)

    samples = {"left": [], "right": []}
    for index, (started, drive) in enumerate(edges[:-1]):
        wz = drive[2]
        if abs(wz) < 1e-9:
            continue
        stopped = edges[index + 1][0]
        if stopped <= started:
            continue
        response_end = float(end_time)
        for next_started, next_drive in edges[index + 1:-1]:
            if abs(next_drive[2]) > 1e-9:
                response_end = next_started
                break
        command_deg = math.degrees(wz * (stopped - started))
        if abs(command_deg) < 1e-6:
            continue
        yaw_start, yaw_stop, yaw_total = np.interp(
            (started, stopped, response_end), yaw_time, yaw_value
        )
        immediate = math.degrees(yaw_stop - yaw_start) / command_deg
        total = math.degrees(yaw_total - yaw_start) / command_deg
        if immediate > 0.0 and total > 0.0:
            samples["left" if wz > 0.0 else "right"].append(
                (float(immediate), float(max(immediate, total)))
            )

    def fitted(direction, fallback_pair):
        rows = samples[direction]
        if not rows:
            return (*fallback_pair, 0)
        immediate = float(np.median([row[0] for row in rows]))
        total = float(np.median([row[1] for row in rows]))
        return immediate, max(immediate, total), len(rows)

    left = fitted("left", fallback.yaw_gains(1.0))
    right = fitted("right", fallback.yaw_gains(-1.0))
    coefficients = replace(
        fallback,
        immediate_gain=(left[0] + right[0]) / 2.0,
        total_gain=(left[1] + right[1]) / 2.0,
        left_immediate_gain=left[0],
        left_total_gain=left[1],
        right_immediate_gain=right[0],
        right_total_gain=right[1],
        model_id="vehicle1-live-y-stage-directional-gain",
        validated=False,
    )
    return coefficients, {
        "left_samples": left[2],
        "right_samples": right[2],
        "left_immediate_gain": left[0],
        "left_total_gain": left[1],
        "right_immediate_gain": right[0],
        "right_total_gain": right[1],
    }


def response_state_from_history(command_rows, yaw_rows, end_time, coefficients):
    """Replay fitted response and anchor its visible yaw to the measured IMU."""
    commands = sorted(command_rows)
    if not commands:
        return ResponseState(history_known=True, pending_bound_deg=0.0)
    yaw_time, yaw_value = _yaw_interpolator(yaw_rows)
    last_reverse_index = max(
        (index for index, row in enumerate(commands) if float(row[1]) < 0.0),
        default=-1,
    )
    if last_reverse_index >= 0:
        commands = commands[last_reverse_index + 1:]
    if not commands:
        return ResponseState(history_known=True, pending_bound_deg=0.0)
    started = max(commands[0][0], float(yaw_time[0]))
    edges = [row for row in commands if started <= row[0] <= end_time]
    if not edges or edges[0][0] > started:
        active = max((row for row in commands if row[0] <= started), default=commands[0])
        edges.insert(0, (started, *active[1:]))
    if edges[-1][0] < end_time:
        edges.append((end_time, *edges[-1][1:]))
    state = ResponseState(history_known=True, pending_bound_deg=0.0)
    for current, following in zip(edges[:-1], edges[1:]):
        duration = following[0] - current[0]
        state = advance_state(
            state, coefficients, float(current[1]), float(current[3]), duration
        )
    measured_yaw = math.degrees(
        float(np.interp(end_time, yaw_time, yaw_value))
        - float(np.interp(started, yaw_time, yaw_value))
    )
    correction = measured_yaw - state.yaw_deg
    return replace(
        state,
        yaw_deg=measured_yaw,
        equilibrium_deg=state.equilibrium_deg + correction,
    )


class TrialNode(Node):
    def __init__(self, config):
        super().__init__("y_slot_topline_insert_trial")
        self.config = config
        self.bridge = CvBridge()
        self.slot_tracker = YSlotTracker()
        self.top_tracker = YTopLineTracker()
        self.command_rows = []
        self.yaw_rows = []
        self.observations = []
        self.saw_y_alignment = False
        self.stage_ready_at = None
        self.stage_observations = None
        self.last_rgb_stamp_ns = 0
        self.publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
        self.create_subscription(Twist, COMMAND_TOPIC, self.on_command, 50)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, CAMERA_TOPIC, self.on_rgb, qos_profile_sensor_data)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, STATUS_TOPIC, self.on_status, status_qos)

    def on_command(self, message):
        self.command_rows.append((
            time.monotonic(), message.linear.x, message.linear.y, message.angular.z
        ))

    def on_imu(self, message):
        self.yaw_rows.append((time.monotonic(), yaw_from_quaternion(message.orientation)))

    def on_rgb(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= self.last_rgb_stamp_ns:
            return
        self.last_rgb_stamp_ns = stamp
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        slot = self.slot_tracker.update(frame, filter_config=self.config)
        line = self.top_tracker.update(frame, slot, filter_config=self.config)
        guidance = self.top_tracker.guidance(line)
        if guidance is None:
            return
        self.observations.append(dict(
            guidance,
            square_top_line_px=line["square_top_line_px"],
            rgb_stamp_ns=stamp,
            received_monotonic=time.monotonic(),
        ))
        self.observations = self.observations[-30:]

    def on_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        reason = str(status.get("reason", ""))
        if (status.get("location") == "Y"
                and reason.startswith("y_slot_")
                and reason != "y_slot_staging_insertion_disabled"):
            self.saw_y_alignment = True
        if reason == "y_slot_staging_insertion_disabled":
            self.stage_ready_at = time.monotonic()
            minimum = int(self.config.get("y_slot_start_consensus_frames", 5))
            self.stage_observations = list(self.observations[-minimum:])

    def publish(self, drive=(0.0, 0.0, 0.0)):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = map(float, drive)
        self.publisher.publish(message)

    def stop(self, repeats=20):
        for _ in range(repeats):
            self.publish()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def base_coefficients(config):
    values = dict(config.get("y_slot_response_model", {}))
    allowed = set(ResponseCoefficients.__dataclass_fields__)
    return ResponseCoefficients(**{key: value for key, value in values.items() if key in allowed})


def wait_for_stage_pose(node, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.stage_ready_at is None:
            continue
        recent = list(node.stage_observations or node.observations)
        if not recent:
            continue
        profile = node.config["floor_calibration_presets"]["loaded"]
        observation = recent[-1]
        square_line = observation.get("square_top_line_px")
        if square_line is None:
            continue
        try:
            pose = y_slot_floor_pose(
                dict(observation, top_line_px=square_line), profile
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        pose["top_center_cm"][0] += float(
            node.config.get("y_slot_centerline_offset_cm", 0.0)
        )
        pose["right_cm"] = float(pose["top_center_cm"][0])
        pose["forward_cm"] = float(pose["top_center_cm"][1])
        pose["bearing_left_deg"] = math.degrees(math.atan2(
            -pose["right_cm"], pose["forward_cm"]
        ))
        return pose, time.monotonic()
    raise RuntimeError("stage_or_topline_timeout")


def execute_plan(node, actions):
    for action in actions:
        print(json.dumps({"phase": "action", **action}, ensure_ascii=False), flush=True)
        deadline = time.monotonic() + float(action["duration_sec"])
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish(action["drive"])
            time.sleep(0.02)
        node.stop(10)


def default_config_path():
    choices = (
        Path("/home/ubuntu/ros2_ws/config/vehicle_pose_config.json"),
        Path("/shared/vehicle_pose_config.json"),
        Path(__file__).resolve().parent.parent / "config" / "vehicle_pose_config.json",
    )
    return str(next((path for path in choices if path.exists()), choices[-1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", default=default_config_path())
    parser.add_argument("--insertion-cm", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--angular", type=float, default=0.35)
    parser.add_argument("--wait-sec", type=float, default=90.0)
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text())
    rclpy.init()
    node = TrialNode(config)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        pose, stage_time = wait_for_stage_pose(node, args.wait_sec)
        fallback = base_coefficients(config)
        coefficients, gain_evidence = measured_directional_gains(
            node.command_rows, node.yaw_rows, stage_time, fallback
        )
        state = response_state_from_history(
            node.command_rows, node.yaw_rows, stage_time, coefficients
        )
        stage_gap_cm = float(config.get("y_slot_staging_distance_cm", 40.0))
        target_gap_cm = stage_gap_cm - args.insertion_cm
        plan = plan_approach(
            *pose["top_center_cm"],
            float(pose["heading_left_deg"]),
            state,
            coefficients,
            staging_cm=target_gap_cm,
            insertion_cm=1.0,
            speed_m_s=args.speed,
            angular_rad_s=args.angular,
            settle_sec=float(config.get("y_slot_response_settle_sec", 0.5)),
        )
        result = {
            "phase": "planned",
            "execute": args.execute,
            "insertion_cm": args.insertion_cm,
            "target_gap_cm": target_gap_cm,
            "topline_pose": pose,
            "gain_evidence": gain_evidence,
            "response_state": asdict(state),
            "coefficients": asdict(coefficients),
            "plan": plan,
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if args.execute:
            execute_plan(node, plan["actions"])
            print(json.dumps({"phase": "complete"}, ensure_ascii=False), flush=True)
    finally:
        node.stop(30)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
