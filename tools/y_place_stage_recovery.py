"""Pure bounded recovery planning and command/IMU target association."""
import math
import numpy as np
from y_place_midpoint import heading_error


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])


def predicted_target(initial_target,commands,imu,reference_yaw_deg,coefficients):
    """Approximate fork-frame target using commands and measured yaw, not odom."""
    rows=np.asarray(imu,float)
    if len(rows)<2 or not commands:
        raise ValueError('recovery_missing_motion_history')
    angles=np.unwrap(rows[:,1])-math.radians(reference_yaw_deg)
    offset=coefficients.fork_offset_cm
    body=np.array([0.,-offset])
    for a,b in zip(commands,commands[1:]):
        dt=b[0]-a[0]
        if dt<0:raise ValueError('recovery_nonmonotonic_commands')
        count=max(1,int(math.ceil(dt/.02)))
        for t in np.linspace(a[0],b[0],count,endpoint=False)+dt/count/2:
            yaw=float(np.interp(t,rows[:,0],angles))
            body+=rotation(yaw)@np.array([-a[2],a[1]*coefficients.forward_scale])*100*dt/count
    r=rotation(float(angles[-1]))
    front=body+r@np.array([0.,offset])
    return (r.T@(np.asarray(initial_target)-front)).tolist()


def recovery_turn(target_deg,actual_deg,coefficients,angular=.35):
    error=heading_error(target_deg,actual_deg)
    if abs(error)>30.:
        raise RuntimeError('stage_recovery_heading_out_of_range')
    if abs(error)<=1.5:
        return []
    gain=coefficients.left_total_gain if error>0 else coefficients.right_total_gain
    if gain is None:gain=coefficients.total_gain
    if not math.isfinite(gain) or gain<=0 or not math.isfinite(angular) or angular<=0:
        raise ValueError('invalid_recovery_gain')
    return [dict(action='stage_recovery_turn_once',drive=[0.,0.,math.copysign(angular,error)],
                 duration_sec=min(1.,math.radians(abs(error))/(angular*gain)))]


def recovery_advance(pose,predicted,heading_residual,coefficients,speed=.1):
    """Only a freshly associated, heading-aligned target permits translation."""
    if not all(math.isfinite(v) for v in [heading_residual,pose['heading_left_deg'],speed,coefficients.forward_scale]) or speed<.1 or coefficients.forward_scale<=0:
        raise ValueError('invalid_recovery_motion')
    centre=np.asarray(pose['top_center_cm'],float)
    if not np.isfinite(centre).all() or not np.isfinite(predicted).all():
        raise ValueError('invalid_recovery_target')
    difference=centre-np.asarray(predicted)
    if abs(difference[0])>5. or abs(difference[1])>12.:
        raise RuntimeError('stage_recovery_target_mismatch')
    if abs(heading_residual)>5. or abs(pose['heading_left_deg'])>5.:
        raise RuntimeError('stage_recovery_heading_not_aligned')
    normal=np.asarray(pose['inward_normal'])
    distance=float((centre@normal-pose['staging_cm'])/normal[1])
    if distance < -3. or distance>20.:
        raise RuntimeError('stage_recovery_distance_out_of_range')
    if distance<=1.:
        return []
    return [dict(action='stage_recovery_forward',drive=[speed,0.,0.],
                 duration_sec=distance/(100*speed*coefficients.forward_scale))]
