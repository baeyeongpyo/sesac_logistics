"""Unchanged vehicle test_y observation callbacks, serviced by its worker."""
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
from .y_place_square_geometry import detect_square, square_pose, draw_evidence, detect_nearest_topline, topline_pose, detect_locked_topline, inspect_nearest_topline
from .y_place_square_motion import base_coefficients, frozen_approach, measured_response, loaded_visual_plan, turn_target_reached
from .y_place_midpoint import stopped_heading, remaining_insertion_cm
from .y_place_finish import ForkDownGate, unloaded_retreat, camera_reacquisition_retreat
from .y_place_trial_stop import TrialStop
from .y_place_command_monitor import CommandConflictMonitor

class TrialObservations:

    def command(self, msg):
        drive = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.command_monitor.received(drive)
        self.commands.append((time.monotonic(), *drive))

    def fork_state(self, msg):
        self.fork_gate.receive(msg.data)

    def orientation(self, msg):
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.imu.append((time.monotonic(), yaw))

    def image(self, msg):
        if self.frozen:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        source_ns = stamp[0] * 1000000000 + stamp[1]
        if source_ns <= self.image_after_ns:
            self.stale_images += 1
            return
        received_ns = self.get_clock().now().nanoseconds
        detection_started = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.reacquiring:
            self.frozen = True
            self.stage_frame = frame.copy()
        inspection = (inspect_nearest_topline(frame) if self.reacquiring else
                      inspect_nearest_topline(frame, self.anchor, prefer_left=self.anchor is None))
        self.detection_failure = {k: inspection.get(k) for k in ("reason", "missing_side")}
        d = inspection["detection"]
        if d is None and self.reacquiring and (getattr(self, 'reference_detection', None) is not None):
            d = detect_locked_topline(frame, self.reference_frame, self.reference_detection)
        self.image_timing = dict(source_stamp_ns=source_ns, received_stamp_ns=received_ns, after_stamp_ns=self.image_after_ns, source_age_sec=(received_ns - source_ns) / 1000000000.0, detection_sec=time.monotonic() - detection_started, discarded_stale_frames=self.stale_images)
        if not d:
            if not self.reacquiring:
                self.observations.clear()
                self.lock_size = None
            return
        centre = np.asarray(d['x_identity']['center_px'])
        if not self.reacquiring and self.lock_size is not None and (np.linalg.norm(centre - self.anchor) > self.lock_size * 0.3):
            self.observations.clear()
            self.lock_size = None
            return
        if self.lock_size is None:
            self.anchor = centre
            self.lock_size = d['x_identity']['roi_px'][2]
        self.observations.append(d)
        self.last_frame = frame
        self.reference_frame = frame.copy()
        self.reference_detection = d
