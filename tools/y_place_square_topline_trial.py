#!/usr/bin/env python3
"""Standalone Y PLACE: 40cm approach, fresh topline centering, then insertion.

--image is offline, without ROS imports. --execute is for the user only, after
stopping other cmd_vel producers. No AutoDock source/config is changed.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from y_place_square_geometry import detect_square, square_pose, draw_evidence, detect_nearest_topline, topline_pose, detect_locked_topline
from y_place_square_motion import base_coefficients, frozen_approach, measured_response, loaded_visual_plan, turn_target_reached
from y_place_midpoint import stopped_heading, remaining_insertion_cm
from y_place_finish import ForkDownGate, unloaded_retreat, camera_reacquisition_retreat
from y_place_trial_stop import TrialStop
from y_place_command_monitor import CommandConflictMonitor


def consensus_pose(observations, profile, staging_cm):
    observation = dict(observations[-1])
    if 'outer_corners_px' not in observation:
        observation['top_edge_px']=np.median([o['top_edge_px'] for o in observations],axis=0).tolist()
        pose=topline_pose(observation,profile,staging_cm)
        pose['observation_count']=len(observations)
        return observation,pose
    corners = np.median([o['outer_corners_px'] for o in observations],axis=0)
    observation.update(outer_corners_px=corners.tolist(),top_edge_px=corners[:2].tolist(),
                       bottom_edge_px=corners[[3,2]].tolist())
    pose = square_pose(observation,profile,staging_cm)
    pose['observation_count'] = len(observations)
    return observation,pose


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload,indent=2,ensure_ascii=False))


def run_live(args, config, output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image, Imu
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    stop = TrialStop(args.stop_file)

    class Trial(Node):
        def __init__(self):
            super().__init__('y_place_square_standalone')
            self.bridge = CvBridge()
            self.observations = []
            self.anchor = args.anchor
            self.lock_size = None
            self.frozen = False
            self.reacquiring = False
            self.last_frame = None
            self.stage_frame = None
            self.last_stamp = None
            self.image_after_ns=self.get_clock().now().nanoseconds
            self.image_timing=None
            self.stale_images=0
            self.commands, self.imu = [], []
            self.command_monitor = CommandConflictMonitor()
            self.moved = False
            self.fork_gate=ForkDownGate()
            self.fork_pub=self.create_publisher(String,'/fork/command',10)
            self.create_subscription(String,'/fork/state',self.fork_state,10)
            self.pub = self.create_publisher(Twist,'/controller/cmd_vel',10)
            self.create_subscription(Image,'/ascamera/camera_publisher/rgb0/image',self.image,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Imu,'/imu',self.orientation,qos_profile_sensor_data)
            self.create_subscription(Twist,'/controller/cmd_vel',self.command,50)

        def command(self,msg):
            drive=(msg.linear.x,msg.linear.y,msg.angular.z)
            self.command_monitor.received(drive)
            self.commands.append((time.monotonic(),*drive))

        def fork_state(self,msg):
            self.fork_gate.receive(msg.data)

        def orientation(self,msg):
            q = msg.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.imu.append((time.monotonic(),yaw))

        def image(self,msg):
            if self.frozen:
                return
            stamp = (msg.header.stamp.sec,msg.header.stamp.nanosec)
            if stamp == self.last_stamp:
                return
            self.last_stamp = stamp
            source_ns=stamp[0]*1_000_000_000+stamp[1]
            if source_ns<=self.image_after_ns:
                self.stale_images+=1
                return
            received_ns=self.get_clock().now().nanoseconds
            detection_started=time.monotonic()
            frame = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            if self.reacquiring:
                self.frozen = True  # Exactly one fresh-frame attempt at staging.
                self.stage_frame = frame.copy()
            # Use the same physical topline detector as staging and the GUI.
            # The legacy full-quad search can enumerate thousands of warps.
            d = detect_nearest_topline(frame) if self.reacquiring else detect_nearest_topline(frame,self.anchor)
            if d is None and self.reacquiring and getattr(self,'reference_detection',None) is not None:
                d=detect_locked_topline(frame,self.reference_frame,self.reference_detection)
            self.image_timing=dict(source_stamp_ns=source_ns,received_stamp_ns=received_ns,
                after_stamp_ns=self.image_after_ns,source_age_sec=(received_ns-source_ns)/1e9,
                detection_sec=time.monotonic()-detection_started,discarded_stale_frames=self.stale_images)
            if not d:
                if not self.reacquiring:
                    self.observations.clear()
                    self.lock_size=None
                return
            centre = np.asarray(d['x_identity']['center_px'])
            if not self.reacquiring and self.lock_size is not None and np.linalg.norm(centre-self.anchor)>self.lock_size*.3:
                self.observations.clear()
                self.lock_size=None
                return
            if self.lock_size is None:
                self.anchor = centre
                self.lock_size = d['x_identity']['roi_px'][2]
            self.observations.append(d)
            self.last_frame = frame
            self.reference_frame=frame.copy()
            self.reference_detection=d

        def publish(self,drive):
            if any(drive):
                stop.check()
            message = Twist()
            message.linear.x,message.linear.y,message.angular.z = map(float,drive)
            self.pub.publish(message)
            self.command_monitor.sent(drive)
            self.moved |= any(drive)

        def other_publishers(self):
            return [i.node_namespace.rstrip('/')+'/'+i.node_name
                    for i in self.get_publishers_info_by_topic('/controller/cmd_vel')
                    if (i.node_name,i.node_namespace)!=(self.get_name(),self.get_namespace())]

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop.install()
    node = Trial()
    try:
        deadline = time.monotonic()+args.wait_sec
        print(json.dumps({'phase':'initial_detection','consecutive_frames':args.start_frames}),flush=True)
        while len(node.observations)<args.start_frames and time.monotonic()<deadline:
            stop.check()
            rclpy.spin_once(node,timeout_sec=.05)
        if len(node.observations)<args.start_frames:
            raise RuntimeError('no_consecutive_topline_observations')
        node.frozen = True
        write_json(output/'initial_image_timing.json',node.image_timing)
        detection,pose = consensus_pose(node.observations,config['floor_calibration_presets']['loaded'],args.stage_cm)
        coefficients = base_coefficients(config)
        plan = frozen_approach(pose,coefficients,args.speed,args.angular,args.insert_cm)
        stop.check()
        payload = dict(detection=detection,pose=pose,stage_plan=plan,requested_insertion_cm=args.insert_cm)
        write_json(output/'frozen_plan.json',payload)
        cv2.imwrite(str(output/'frozen_detection.png'),draw_evidence(node.last_frame,detection,pose))
        print(json.dumps(dict(phase='planned',target_stage=plan.get('target_stage'),
                              accepted=plan['accepted'],reason=plan['reason'],
                              actions=plan['actions'],evidence=str(output/'frozen_plan.json')),
                         ensure_ascii=False),flush=True)
        if not plan['accepted']:
            raise RuntimeError(plan['reason'])
        others = node.other_publishers()
        if others:
            print(json.dumps(dict(phase='registered_publishers',nodes=others,
                                  blocking=False)),flush=True)
        # Drain callbacks queued during image processing/planning before taking
        # command control. Publisher registration alone is never a veto.
        for _ in range(64):
            rclpy.spin_once(node,timeout_sec=0.)
        node.command_monitor.check()
        node.command_monitor.active=True
        if not node.imu:
            raise RuntimeError('no_imu_for_approach_response_measurement')
        node.commands.clear()
        node.imu = node.imu[-1:]
        approach_reference_yaw=math.degrees(node.imu[-1][1])
        node.commands.append((time.monotonic(),0.,0.,0.))

        def execute(actions):
            for action in actions:
                print(json.dumps({'phase':'action',**action}),flush=True)
                turn_start=node.imu[-1][1] if node.imu else None
                until = time.monotonic()+action['duration_sec']
                while time.monotonic()<until:
                    stop.check()
                    for _ in range(32):
                        rclpy.spin_once(node,timeout_sec=0.)
                    node.command_monitor.check()
                    if ('target_turn_deg' in action and turn_start is not None and node.imu
                            and turn_target_reached(turn_start,node.imu[-1][1],
                                                    action['drive'][2],action['target_turn_deg'])):
                        node.publish((0.,0.,0.))
                        print(json.dumps(dict(phase='turn_target_reached',action=action['action'],
                                              target_turn_deg=action['target_turn_deg'])),flush=True)
                        break
                    node.publish(action['drive'])
                    time.sleep(min(.02,max(0.,until-time.monotonic())))
            node.publish((0.,0.,0.))
            rclpy.spin_once(node,timeout_sec=.02)

        def wait_stopped(label):
            stopped_at=time.monotonic()
            while time.monotonic()-stopped_at<4.:
                stop.check()
                node.publish((0.,0.,0.))
                rclpy.spin_once(node,timeout_sec=.02)
                node.command_monitor.check()
                yaw=stopped_heading(node.imu,time.monotonic(),stopped_at)
                if yaw is not None:
                    return yaw
            raise RuntimeError(label+'_imu_not_settled')

        execute(plan['actions'])
        stop.check()
        node.commands.append((time.monotonic(),0.,0.,0.))
        # Stop at staging, discard queued pre-stop images, then reacquire without
        # the initial X anchor or any new position/angle association restriction.
        stage_imu_heading = wait_stopped('stage')
        node.commands.append((time.monotonic(),0.,0.,0.))
        fitted,state,gains = measured_response(node.commands,node.imu,coefficients)
        def reacquire(label):
            node.observations.clear()
            node.stage_frame=None
            node.image_after_ns=node.get_clock().now().nanoseconds
            node.image_timing=None
            node.stale_images=0
            node.reacquiring=True
            node.frozen=False
            deadline=time.monotonic()+args.wait_sec
            print(json.dumps({'phase':label}),flush=True)
            while not node.frozen and time.monotonic()<deadline:
                stop.check()
                rclpy.spin_once(node,timeout_sec=.02)
                node.command_monitor.check()
            node.frozen=True
            write_json(output/(label+'_timing.json'),dict(image=node.image_timing,discarded_stale_frames=node.stale_images))
            if node.stage_frame is not None:
                cv2.imwrite(str(output/(label+'_raw.png')),node.stage_frame)
            return bool(node.observations)

        def reacquire_with_backup(label):
            if reacquire(label):
                return True
            action=camera_reacquisition_retreat(args.speed)
            recovery=dict(status='reversing_for_visibility',commanded_reverse_cm=10.,
                          reverse_distance_model='nominal; not calibrated loaded reverse',action=action)
            write_json(output/(label+'_reverse.json'),recovery)
            execute([action])
            recovery['after_imu_heading_deg']=wait_stopped(label+'_reverse')
            found=reacquire(label+'_after_reverse')
            recovery.update(status='reacquired' if found else 'not_detected')
            write_json(output/(label+'_reverse.json'),recovery)
            return found

        if not reacquire_with_backup('stage_reacquisition'):
            raise RuntimeError('no_stage_topline_after_reverse')
        stage_detection = node.observations[-1]
        stage_pose = topline_pose(stage_detection,config['floor_calibration_presets']['loaded'],args.stage_cm)
        write_json(output/'stage_reacquisition.json',dict(detection=stage_detection,pose=stage_pose))
        cv2.imwrite(str(output/'stage_reacquisition.png'),draw_evidence(node.last_frame,stage_detection,stage_pose))
        node.commands.append((time.monotonic(),0.,0.,0.))
        fitted,state,gains=measured_response(node.commands,node.imu,fitted)
        stage_distance=remaining_insertion_cm(stage_pose,args.stage_cm-args.insert_cm)
        first_cm=min(15.,stage_distance)
        insertion = loaded_visual_plan(stage_pose,state,fitted,first_cm,args.speed,args.angular,
                                        center_offset_left_cm=args.center_offset_left_cm)
        write_json(output/'insertion_plan.json',dict(plan=insertion,measured_gains=gains))
        if not insertion['accepted']:
            raise RuntimeError('loaded_visual_plan_not_solved')
        # One stopped visual checkpoint; no feedback corrections while moving.
        # Commanded travel never substitutes for the newly observed distance.
        checkpoint=dict(status='already_at_insertion_endpoint')
        if stage_distance>0.:
            first=insertion['actions']
            checkpoint=dict(commanded_first_cm=first_cm,first_actions=first,status='planned')
            write_json(output/'midpoint_check.json',checkpoint)
            execute(first)
            actual=wait_stopped('midpoint')
            checkpoint.update(actual_imu_heading_deg=actual,status='stopped')
            write_json(output/'midpoint_check.json',checkpoint)
            if not reacquire_with_backup('midpoint_reacquisition'):
                raise RuntimeError('no_midpoint_topline_observation')
            fresh_pose=topline_pose(node.observations[-1],config['floor_calibration_presets']['loaded'],args.stage_cm)
            remaining_cm=remaining_insertion_cm(fresh_pose,args.stage_cm-args.insert_cm)
            node.commands.append((time.monotonic(),0.,0.,0.))
            fitted,state,gains=measured_response(node.commands,node.imu,fitted)
            remaining=loaded_visual_plan(fresh_pose,state,fitted,remaining_cm,args.speed,args.angular,
                                                 center_offset_left_cm=args.center_offset_left_cm)
            checkpoint.update(observed_pose=fresh_pose,remaining_cm=remaining_cm,
                              remaining_plan=remaining,measured_gains=gains,status='visually_replanned')
            write_json(output/'midpoint_check.json',checkpoint)
            if not remaining['accepted']:
                raise RuntimeError('loaded_visual_plan_not_solved')
            execute(remaining['actions'])
        checkpoint['status']='command_sequence_complete'
        write_json(output/'midpoint_check.json',checkpoint)
        wait_stopped('before_fork_down')
        # Flush queued old completion messages before arming this DOWN request.
        for _ in range(64):rclpy.spin_once(node,timeout_sec=0.)
        stop.check()
        node.fork_gate.begin()
        finish=dict(status='waiting_fork_down',command='DOWN',reverse_cm=args.insert_cm,
                    reverse_distance_model='unloaded nominal 1:1; not measured')
        write_json(output/'placement_finish.json',finish)
        print(json.dumps({'phase':'fork_down'}),flush=True)
        node.fork_pub.publish(String(data='DOWN'))
        deadline=time.monotonic()+20.
        while not node.fork_gate.complete():
            stop.check()
            if time.monotonic()>=deadline:
                raise RuntimeError('fork_down_timeout')
            node.publish((0.,0.,0.))
            rclpy.spin_once(node,timeout_sec=.02)
            node.command_monitor.check()
        node.fork_gate.active=False
        finish.update(status='fork_down_complete',fork_result=node.fork_gate.result)
        write_json(output/'placement_finish.json',finish)
        retreat=unloaded_retreat(args.insert_cm,args.speed)
        finish.update(status='reversing',reverse_action=retreat)
        write_json(output/'placement_finish.json',finish)
        execute([retreat])
        finish['status']='command_sequence_complete'
        write_json(output/'placement_finish.json',finish)
        print(json.dumps({'phase':'command_sequence_complete','actual_position_verified':False}),flush=True)
    except InterruptedError as exc:
        print(json.dumps({'phase':'stopped','reason':str(exc)}),flush=True)
    finally:
        try:
            if node.fork_gate.active and rclpy.ok():
                node.fork_pub.publish(String(data='STOP'))
                rclpy.spin_once(node,timeout_sec=0.)
                node.fork_gate.active=False
            if node.moved and rclpy.ok():
                # Keep DDS alive while the final zeros leave the publisher.
                for _ in range(10):
                    node.publish((0.,0.,0.))
                    rclpy.spin_once(node,timeout_sec=0.)
                    time.sleep(.02)
            write_json(output/'motion_history.json',dict(commands=node.commands,imu=node.imu))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            stop.restore()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--image',help='saved RGB image; entirely offline')
    mode.add_argument('--execute',action='store_true',help='USER ONLY: publishes vehicle commands')
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--anchor',nargs=2,type=float,metavar=('X','Y'))
    parser.add_argument('--stage-cm',type=float,default=40.)
    parser.add_argument('--insert-cm',type=float,default=30.)
    parser.add_argument('--center-offset-left-cm',type=float,default=2.,
                        help='Additional lateral shift toward vehicle left after stage reacquisition (cm)')
    parser.add_argument('--speed',type=float,default=.10)
    parser.add_argument('--angular',type=float,default=.35)
    parser.add_argument('--wait-sec',type=float,default=30.)
    parser.add_argument('--observe-sec',type=float,default=.5)
    parser.add_argument('--start-frames',type=int,default=1)
    parser.add_argument('--stop-file',default='/home/ubuntu/.local/state/y_place_trial/stop')
    args = parser.parse_args()
    if args.start_frames<1:
        parser.error('start frames must be positive')
    if args.insert_cm<=15.:
        parser.error('insertion must exceed the 15cm midpoint')
    if not math.isfinite(args.center_offset_left_cm):
        parser.error('center offset must be finite')
    values = (args.speed,args.angular,args.stage_cm,args.insert_cm,args.wait_sec,args.observe_sec)
    if not all(math.isfinite(v) for v in values) or args.speed<.10 or min(values[1:5])<=0 or args.observe_sec<0:
        parser.error('finite positive distances/rates required; nonzero linear speed must be >=0.10 m/s')
    config = json.loads(Path(args.config).read_text())
    output = Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            parser.error('image cannot be read')
        detection = detect_square(frame,args.anchor)
        if detection is None:
            write_json(output/'frozen_plan.json',dict(detected=False,reason='no_physical_square'))
            cv2.imwrite(str(output/'frozen_detection.png'),draw_evidence(frame,None))
            return
        pose = square_pose(detection,config['floor_calibration_presets']['loaded'],args.stage_cm)
        plan = frozen_approach(pose,base_coefficients(config),args.speed,args.angular,args.insert_cm)
        write_json(output/'frozen_plan.json',dict(detection=detection,pose=pose,stage_plan=plan,requested_insertion_cm=args.insert_cm))
        cv2.imwrite(str(output/'frozen_detection.png'),draw_evidence(frame,detection,pose))
        print(json.dumps(dict(accepted=plan['accepted'],reason=plan['reason'],pose=pose),ensure_ascii=False))
    else:
        run_live(args,config,output)


if __name__ == '__main__':
    main()
