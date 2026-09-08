"""Pure IMU checks for one stopped correction; no ROS/runtime imports."""
import math
import numpy as np


def stopped_heading(samples, now, stopped_at, minimum_sec=1., window_sec=.5):
    """Require fresh, dense, settled measurements after commanding zero."""
    if now-stopped_at < minimum_sec:
        return None
    rows=np.asarray([r for r in samples if now-window_sec <= r[0] <= now],float)
    if len(rows)<5 or now-rows[-1,0]>.15 or rows[-1,0]-rows[0,0]<window_sec*.8:
        return None
    if np.max(np.diff(rows[:,0]))>.15 or not np.isfinite(rows).all():
        return None
    angles=np.unwrap(rows[:,1])*180/math.pi
    if np.ptp(angles)>.3:
        return None
    return float(np.median(angles))


def heading_error(target_deg, actual_deg):
    if not all(math.isfinite(v) for v in [target_deg,actual_deg]):
        raise ValueError('invalid_checkpoint_heading')
    return (target_deg-actual_deg+180)%360-180


def correction_once(target_deg, actual_deg, coefficients, angular=.35):
    """At most one bounded pulse. Never reverse direction in a retry loop."""
    error=heading_error(target_deg,actual_deg)
    if abs(error)>8.:
        raise RuntimeError('midpoint_heading_error_too_large')
    if abs(error)<=1.5:
        return dict(error_deg=error,actions=[])
    gain=coefficients.left_total_gain if error>0 else coefficients.right_total_gain
    if gain is None:gain=coefficients.total_gain
    if not math.isfinite(gain) or gain<=0 or not math.isfinite(angular) or angular<=0:
        raise ValueError('invalid_checkpoint_response')
    duration=min(.15,math.radians(min(3.,abs(error)))/(angular*gain))
    return dict(error_deg=error,actions=[dict(action='midpoint_heading_once',
                drive=[0.,0.,math.copysign(angular,error)],duration_sec=duration)])


def split_insertion(actions, total_cm, first_cm=15.):
    if not math.isfinite(total_cm) or not 0<first_cm<total_cm:
        raise ValueError('invalid_midpoint_distance')
    if not actions or actions[-1]['action']!='final_straight_insertion':
        raise ValueError('missing_final_straight')
    straight=actions[-1]
    first=dict(straight,action='insertion_to_midpoint',
               duration_sec=straight['duration_sec']*first_cm/total_cm)
    last=dict(straight,duration_sec=straight['duration_sec']-first['duration_sec'])
    return [*actions[:-1],first],[last]


def remaining_insertion_cm(pose, final_gap_cm):
    """Distance to the fixed dock-relative endpoint, measured from fresh topline."""
    centre=np.asarray(pose['top_center_cm'],float)
    normal=np.asarray(pose['inward_normal'],float)
    gap=float(centre@normal)
    if not math.isfinite(gap) or not math.isfinite(final_gap_cm):
        raise ValueError('invalid_observed_gap')
    return max(0.,gap-final_gap_cm)
