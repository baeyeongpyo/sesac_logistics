"""Vehicle test_y sequence; ROS lifecycle belongs to AutoDockNode."""
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
from .y_place_square_geometry import detect_square, square_pose, draw_evidence, detect_nearest_topline, topline_pose, detect_locked_topline
from .y_place_square_motion import base_coefficients, frozen_approach, measured_response, loaded_visual_plan, turn_target_reached
from .y_place_midpoint import stopped_heading, remaining_insertion_cm
from .y_place_heading_feedback import correct_heading
from .y_place_finish import ForkDownGate, unloaded_retreat, camera_reacquisition_retreat
from .y_place_trial_stop import TrialStop
from .y_place_command_monitor import CommandConflictMonitor
from types import SimpleNamespace as String

def consensus_pose(observations, profile, staging_cm):
    observation = dict(observations[-1])
    if 'outer_corners_px' not in observation:
        observation['top_edge_px'] = np.median([o['top_edge_px'] for o in observations], axis=0).tolist()
        pose = topline_pose(observation, profile, staging_cm)
        pose['observation_count'] = len(observations)
        return (observation, pose)
    corners = np.median([o['outer_corners_px'] for o in observations], axis=0)
    observation.update(outer_corners_px=corners.tolist(), top_edge_px=corners[:2].tolist(), bottom_edge_px=corners[[3, 2]].tolist())
    pose = square_pose(observation, profile, staging_cm)
    pose['observation_count'] = len(observations)
    return (observation, pose)

def write_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False))

def learned_insertion_plan(node, coefficients, state, pose, travel, args, phase):
    """Plan from this approach's gains; retain learning records for inspection."""
    def build(model):
        return loaded_visual_plan(pose, state, model, travel, args.speed, args.angular,
                                  center_offset_left_cm=args.center_offset_left_cm)
    reference_heading = math.degrees(node.imu[-1][1])
    original = build(coefficients)
    learning = getattr(node, 'learning', None)
    if learning is None or not original['accepted'] or travel <= 0:
        return original
    try:
        corrected = learning.prepare(node, coefficients, state, original, phase,
                                     apply_correction=False)
        result = build(corrected) if corrected != coefficients else original
        if not result['accepted']:
            result = original
            learning.pending.update(applied_factors=[1., 1., 1., 1.],
                                    model_id='uncorrected_infeasible_correction')
        learning.set_plan(result, reference_heading, pose['heading_left_deg'])
        return result
    except (OSError, ValueError, TypeError) as exc:
        learning.pending = None
        node.report({'phase': 'learning_unavailable', 'reason': str(exc)})
        return original


def capture_learning(node, heading):
    learning = getattr(node, 'learning', None)
    if learning is not None:
        try:
            learning.capture(node, heading)
        except (OSError, ValueError, TypeError) as exc:
            learning.pending = None
            node.report({'phase': 'learning_unavailable', 'reason': str(exc)})

def run_sequence(node, args, config, output, stop, spin_once):
    try:
        deadline = time.monotonic() + args.wait_sec
        node.report({'phase': 'initial_detection', 'consecutive_frames': args.start_frames})
        while len(node.observations) < args.start_frames and time.monotonic() < deadline:
            stop.check()
            spin_once(node, timeout_sec=0.05)
        if len(node.observations) < args.start_frames:
            raise RuntimeError('no_consecutive_topline_observations')
        node.frozen = True
        write_json(output / 'initial_image_timing.json', node.image_timing)
        detection, pose = consensus_pose(node.observations, config['floor_calibration_presets']['loaded'], args.stage_cm)
        coefficients = base_coefficients(config)
        plan = frozen_approach(pose, coefficients, args.speed, args.angular, args.insert_cm)
        stop.check()
        payload = dict(detection=detection, pose=pose, stage_plan=plan, requested_insertion_cm=args.insert_cm)
        write_json(output / 'frozen_plan.json', payload)
        cv2.imwrite(str(output / 'frozen_detection.png'), draw_evidence(node.last_frame, detection, pose))
        node.report(dict(phase='planned', target_stage=plan.get('target_stage'), accepted=plan['accepted'], reason=plan['reason'], actions=plan['actions'], evidence=str(output / 'frozen_plan.json')))
        if not plan['accepted']:
            raise RuntimeError(plan['reason'])
        others = node.other_publishers()
        if others:
            node.report(dict(phase='registered_publishers', nodes=others, blocking=False))
        for _ in range(64):
            spin_once(node, timeout_sec=0.0)
        node.command_monitor.check()
        node.command_monitor.active = True
        if not node.imu:
            raise RuntimeError('no_imu_for_approach_response_measurement')
        node.commands.clear()
        node.imu = node.imu[-1:]
        approach_reference_yaw = math.degrees(node.imu[-1][1])
        node.commands.append((time.monotonic(), 0.0, 0.0, 0.0))

        def execute(actions, *, stop_on_turn_target=True):
            executed = []
            for action in actions:
                node.report({'phase': 'action', **action})
                turn_start = node.imu[-1][1] if node.imu else None
                started = time.monotonic()
                until = started + action['duration_sec']
                while time.monotonic() < until:
                    stop.check()
                    for _ in range(32):
                        spin_once(node, timeout_sec=0.0)
                    node.command_monitor.check()
                    if stop_on_turn_target and 'target_turn_deg' in action and turn_start is not None and node.imu and turn_target_reached(turn_start, node.imu[-1][1], action['drive'][2], action['target_turn_deg']):
                        node.publish((0.0, 0.0, 0.0))
                        node.report(dict(phase='turn_target_reached', action=action['action'], target_turn_deg=action['target_turn_deg']))
                        break
                    node.publish(action['drive'])
                    time.sleep(min(0.02, max(0.0, until - time.monotonic())))
                executed.append(dict(action, duration_sec=time.monotonic()-started))
            node.publish((0.0, 0.0, 0.0))
            spin_once(node, timeout_sec=0.02)
            return executed

        def wait_stopped(label):
            stopped_at = time.monotonic()
            while time.monotonic() - stopped_at < 4.0:
                stop.check()
                node.publish((0.0, 0.0, 0.0))
                spin_once(node, timeout_sec=0.02)
                node.command_monitor.check()
                yaw = stopped_heading(node.imu, time.monotonic(), stopped_at)
                if yaw is not None:
                    return yaw
            raise RuntimeError(label + '_imu_not_settled')
        execute(plan['actions'])
        stop.check()
        node.commands.append((time.monotonic(), 0.0, 0.0, 0.0))
        stage_imu_heading = wait_stopped('stage')
        node.commands.append((time.monotonic(), 0.0, 0.0, 0.0))
        fitted, state, gains = measured_response(node.commands, node.imu, coefficients)

        def reacquire(label):
            node.observations.clear()
            node.stage_frame = None
            node.image_after_ns = node.get_clock().now().nanoseconds
            node.image_timing = None
            node.stale_images = 0
            node.reacquiring = True
            node.frozen = False
            deadline = time.monotonic() + min(args.wait_sec, 3.0)
            node.report({'phase': label})
            while not node.frozen and time.monotonic() < deadline:
                stop.check()
                spin_once(node, timeout_sec=0.02)
                node.command_monitor.check()
            node.frozen = True
            write_json(output / (label + '_timing.json'), dict(image=node.image_timing, discarded_stale_frames=node.stale_images, failure=getattr(node, 'detection_failure', {})))
            if node.stage_frame is not None:
                cv2.imwrite(str(output / (label + '_raw.png')), node.stage_frame)
            return bool(node.observations)

        def reacquire_with_backup(label):
            if reacquire(label):
                return True
            action = camera_reacquisition_retreat(args.speed)
            recovery = dict(status='reversing_for_visibility', commanded_reverse_cm=10.0, reverse_distance_model='nominal; not calibrated loaded reverse', action=action)
            write_json(output / (label + '_reverse.json'), recovery)
            execute([action])
            recovery['after_imu_heading_deg'] = wait_stopped(label + '_reverse')
            found = reacquire(label + '_after_reverse')
            recovery.update(status='reacquired' if found else 'not_detected')
            write_json(output / (label + '_reverse.json'), recovery)
            return found
        if not reacquire_with_backup('stage_reacquisition'):
            failure = getattr(node, 'detection_failure', {})
            side = failure.get('missing_side')
            recovered = False
            if side in ('left', 'right'):
                # One small pure turn toward the missing side, then one fresh frame.
                sign = 1.0 if side == 'left' else -1.0
                action = dict(action='peek_missing_' + side + '_side',
                              drive=[0.0, 0.0, sign * args.angular],
                              duration_sec=math.radians(2.0) / args.angular,
                              target_turn_deg=sign * 2.0)
                execute([action])
                wait_stopped('side_recovery')
                recovered = reacquire('stage_reacquisition_after_peek')
            if not recovered:
                failure = getattr(node, 'detection_failure', {})
                raise RuntimeError('y_place_topline_recovery_failed:' +
                                   str(failure.get('missing_side', 'unknown')) + ':' +
                                   str(failure.get('reason', 'no_fresh_frame')))

        stage_detection = node.observations[-1]
        stage_pose = topline_pose(stage_detection, config['floor_calibration_presets']['loaded'], args.stage_cm)
        write_json(output / 'stage_reacquisition.json', dict(detection=stage_detection, pose=stage_pose))
        cv2.imwrite(str(output / 'stage_reacquisition.png'), draw_evidence(node.last_frame, stage_detection, stage_pose))
        node.commands.append((time.monotonic(), 0.0, 0.0, 0.0))
        fitted, state, gains = measured_response(node.commands, node.imu, fitted)
        stage_distance = remaining_insertion_cm(stage_pose, args.stage_cm - args.insert_cm)
        target_heading = math.degrees(node.imu[-1][1]) + stage_pose['heading_left_deg']
        node.report({'phase': 'insertion_started'})
        insertion = learned_insertion_plan(node, fitted, state, stage_pose, stage_distance, args, 'final_insertion')
        write_json(output / 'insertion_plan.json', dict(plan=insertion, measured_gains=gains, frozen=True))
        if not insertion['accepted']:
            raise RuntimeError('loaded_visual_plan_not_solved')
        if stage_distance > 0.0:
            # The model includes yaw released during later forward travel.
            # Cutting a turn at its immediate-yaw estimate invalidates that
            # future trajectory. Execute its exposure, then correct real yaw.
            execute(insertion['actions'], stop_on_turn_target=False)
        final_heading = wait_stopped('before_fork_down')
        capture_learning(node, final_heading)
        final_heading, heading_result = correct_heading(
            target_heading, final_heading, fitted, args.angular,
            execute, wait_stopped, node.report, stop)
        write_json(output / 'final_heading_correction.json', heading_result)
        for _ in range(64):
            spin_once(node, timeout_sec=0.0)
        stop.check()
        node.fork_gate.begin()
        finish = dict(status='waiting_fork_down', command='DOWN', reverse_cm=args.insert_cm, reverse_distance_model='unloaded nominal 1:1; not measured')
        write_json(output / 'placement_finish.json', finish)
        node.report({'phase': 'fork_down'})
        node.fork_pub.publish(String(data='DOWN'))
        deadline = time.monotonic() + 20.0
        while not node.fork_gate.complete():
            stop.check()
            if time.monotonic() >= deadline:
                raise RuntimeError('fork_down_timeout')
            node.publish((0.0, 0.0, 0.0))
            spin_once(node, timeout_sec=0.02)
            node.command_monitor.check()
        node.fork_gate.active = False
        finish.update(status='fork_down_complete', fork_result=node.fork_gate.result)
        write_json(output / 'placement_finish.json', finish)
        retreat = unloaded_retreat(args.insert_cm, args.speed)
        finish.update(status='reversing', reverse_action=retreat)
        write_json(output / 'placement_finish.json', finish)
        execute([retreat])
        finish['status'] = 'command_sequence_complete'
        write_json(output / 'placement_finish.json', finish)
        node.report({'phase': 'command_sequence_complete', 'actual_position_verified': False})
    except InterruptedError as exc:
        node.report({'phase': 'stopped', 'reason': str(exc)})
        raise
    finally:
        try:
            if node.fork_gate.active and True:
                node.fork_pub.publish(String(data='STOP'))
                spin_once(node, timeout_sec=0.0)
                node.fork_gate.active = False
            if node.moved and True:
                for _ in range(10):
                    node.publish((0.0, 0.0, 0.0))
                    spin_once(node, timeout_sec=0.0)
                    time.sleep(0.02)
            write_json(output / 'motion_history.json', dict(commands=node.commands, imu=node.imu))
        finally:
            pass
