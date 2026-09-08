"""Frozen square approach and measured-response insertion, without ROS."""
from dataclasses import asdict, replace
import math

import numpy as np
from .y_place_fast_prediction import endpoint

from .loaded_response_planner import (
    ResponseCoefficients, ResponseState, plan_approach, simulate_actions, advance_state,
)


def base_coefficients(config):
    allowed = ResponseCoefficients.__dataclass_fields__
    return ResponseCoefficients(**{k:v for k,v in config.get('y_slot_response_model',{}).items() if k in allowed})


def frozen_approach(pose, coefficients, speed=.10, angular=.35, insertion_cm=30., settle_sec=.5):
    """Compute exactly once; serialize to detach actions from mutable observations."""
    import json
    observed_gap=float(np.asarray(pose['top_center_cm'])@np.asarray(pose['inward_normal']))
    if observed_gap<=pose['staging_cm']:
        return dict(accepted=True,reason='already_inside_staging_distance',actions=[],
                    observed_gap_cm=observed_gap,target_stage=[0.,0.,0.],
                    next_phase='stage_reacquisition',frozen=True,
                    initial_state_assumption='no approach motion; reobserve before insertion')
    result = plan_approach(*pose['top_center_cm'], pose['heading_left_deg'],
        ResponseState(history_known=True,pending_bound_deg=0.), coefficients,
        staging_cm=pose['staging_cm'], insertion_cm=insertion_cm,
        speed_m_s=speed, angular_rad_s=angular, settle_sec=settle_sec)
    if not result['accepted'] and pose['stage_right_forward_cm'][1]>=0:
        expanded = solve_side_approach(pose,coefficients,speed,angular,settle_sec)
        expanded['original_solver_reason'] = result['reason']
        result = expanded
    result['target_stage'] = [*pose['stage_right_forward_cm'],pose['heading_left_deg']]
    result['initial_state_assumption'] = 'stationary start; latent response assumed zero, not measured'
    result['frozen'] = True
    add_turn_targets(result,ResponseState(history_known=True,pending_bound_deg=0.),coefficients)
    return json.loads(json.dumps(result))


def solve_side_approach(pose, coefficients, speed, angular, settle_sec, state=None):
    """Invert the same response law with side-start seeds, outside node bounds.

    Solve staging x/y/yaw. Preserve latent yaw for the measured insertion
    correction instead of forcing it to zero using a long detour at staging.
    A successful solution is a prediction, not a free-space assertion.
    """
    from scipy.optimize import least_squares
    goal=np.asarray(pose['stage_right_forward_cm'])
    heading=pose['heading_left_deg']
    normal=np.asarray(pose['inward_normal'])
    c=coefficients
    state=state or ResponseState(history_known=True,pending_bound_deg=0.)
    def actions_for(p):
        actions=[]
        for i,value in enumerate(p):
            if abs(value)<1e-7:
                continue
            if i%2==0:
                actions.append(dict(action='turn_left' if value>0 else 'turn_right',
                                    drive=[0.,0.,math.copysign(angular,value)],
                                    duration_sec=abs(math.radians(value)/angular)))
            else:
                actions.append(dict(action='forward',drive=[speed,0.,0.],duration_sec=value/(speed*100)))
                actions.append(dict(action='settle_after_forward',drive=[0.,0.,0.],duration_sec=settle_sec))
        return actions
    def evaluate(p,details=False):
        actions = actions_for(p)
        sim = simulate_actions(actions, state, c) if details else None
        end = sim['path'][-1] if details else endpoint(actions, state, c)
        # Pending yaw is intentionally not a staging pose constraint: the next
        # phase measures/replays it and inverts trim + insertion jointly.
        r=np.array([*(end[:2]-goal),end[2]-heading,
                    .001*np.linalg.norm(np.asarray(p)[::2]),.001*(p[1]+p[3])])
        return (r,sim) if details else r
    translation=goal-c.fork_offset_cm*(normal-np.array([0.,1.]))
    bearing=math.degrees(math.atan2(-translation[0],translation[1]))
    distance=float(np.linalg.norm(translation))/c.forward_scale
    candidates=[]
    immediate,total=c.yaw_gains(bearing)
    release=distance/c.release_command_cm
    mean_gain=immediate+(total-immediate)*(1-(-math.expm1(-release))/max(release,1e-9))
    first=bearing/max(mean_gain,1e-6)
    released_heading=first*(immediate+(total-immediate)*(-math.expm1(-release)))
    final_angle=heading-released_heading
    seeds=[np.array([first,distance,final_angle/c.yaw_gains(final_angle)[0],0.,0.])]
    for split in (.15,.5,.85):
        seeds.append(np.array([bearing/c.yaw_gains(bearing)[1],distance*split,
                                (heading-bearing)/c.yaw_gains(heading-bearing)[1],distance*(1-split),0.]))
    # Command exposure is not actual yaw: low immediate gain can require more
    # than 180 command-degrees for an ordinary final body rotation.
    max_exposure=min(900.,math.degrees(angular*120.))
    for initial in seeds:
        initial=np.clip(initial,[-max_exposure+1,1e-6,-max_exposure+1,1e-6,-max_exposure+1],
                                [max_exposure-1,199,max_exposure-1,199,max_exposure-1])
        fit=least_squares(evaluate,initial,bounds=([-max_exposure,0,-max_exposure,0,-max_exposure],[max_exposure,200,max_exposure,200,max_exposure]),
                          diff_step=.001,max_nfev=120,xtol=1e-7,ftol=1e-7,gtol=1e-7)
        residual,sim=evaluate(fit.x,True)
        candidates.append((float(residual@residual),fit.x,sim))
    feasible=[row for row in candidates if np.max(np.abs(evaluate(row[1])[:3]))<.25]
    def path_length(row):
        return float(np.linalg.norm(np.diff(row[2]['path'][:,:2],axis=0),axis=1).sum())
    score,p,sim=min(feasible,key=path_length) if feasible else min(candidates,key=lambda row:row[0])
    residual,_=evaluate(p,True)
    accepted=bool(np.max(np.abs(residual[:3]))<.25)
    end=sim['path'][-1]
    return dict(accepted=accepted,reason='ok' if accepted else 'no_feasible_side_response_plan',
                solver='standalone_side_pose_inverse',model=asdict(c),initial_state=asdict(state),
                actions=actions_for(p),predicted_stage=end.tolist(),predicted_state=asdict(sim['state']),
                path=sim['path'].tolist(),residual_cm_deg=residual[:3].tolist(),
                requires_measured_insertion_correction=True,
                stage_gap_cm=float((np.asarray(pose['top_center_cm'])-end[:2])@normal))


def measured_response(command_rows, imu_rows, coefficients):
    """Fit each observed direction and replay after the last reverse interval.

    Rows use the same monotonic clock. Unobserved directions remain explicitly
    labelled saved; they are never described as measured during this approach.
    """
    commands = np.asarray(command_rows, dtype=float)
    imu = np.asarray(imu_rows,dtype=float)
    if commands.ndim != 2 or len(commands)<2 or imu.ndim != 2 or len(imu)<2:
        raise ValueError('approach_command_and_imu_history_required')
    if np.any(np.diff(commands[:,0])<0) or np.any(np.diff(imu[:,0])<=0):
        raise ValueError('non_monotonic_history')
    imu_yaw = np.unwrap(imu[:,1])
    reverse = np.flatnonzero(commands[:,1]<0)
    if len(reverse):
        commands = commands[reverse[-1]+1:]
    if len(commands)<2:
        raise ValueError('no_overlapping_history_after_reverse')
    start, end = max(commands[0,0],imu[0,0]), min(commands[-1,0],imu[-1,0])
    if end <= start:
        raise ValueError('no_overlapping_history_after_reverse')
    edges = []
    for row in commands:
        if not edges or not np.array_equal(row[1:],edges[-1][1:]):
            edges.append(row)
    edges.append(np.array([end,*commands[-1,1:]]))
    samples = {'left':[], 'right':[]}
    short_samples={'left':0,'right':0}
    for i,(a,b) in enumerate(zip(edges[:-1],edges[1:])):
        if a[0]<start or b[0]>end or abs(a[3])<1e-9 or b[0]<=a[0]:
            continue
        response_end = next((r[0] for r in edges[i+1:] if abs(r[3])>1e-9),end)
        response_end = min(response_end,end)
        # Total gain needs subsequent forward travel to release pending yaw.
        forward_cm = sum(max(0.,min(v[0],response_end)-max(u[0],b[0]))*u[1]*100
                         for u,v in zip(edges[:-1],edges[1:]) if u[1]>0)
        if forward_cm<=0:
            continue
        yaw = np.interp([a[0],b[0],response_end],imu[:,0],imu_yaw)
        exposure = math.degrees(a[3]*(b[0]-a[0]))
        immediate = math.degrees(yaw[1]-yaw[0])/exposure
        direction='left' if a[3]>0 else 'right'
        # The next insertion also uses short pulses. Preserve this run's
        # measured response instead of substituting a saved sustained gain.
        if b[0]-a[0]<.5:
            short_samples[direction]+=1
        observed_total = math.degrees(yaw[2]-yaw[0])/exposure
        released = -math.expm1(-forward_cm/coefficients.release_command_cm)
        total = immediate+(observed_total-immediate)/released
        if 0<immediate<=total:
            samples['left' if a[3]>0 else 'right'].append((immediate,total))
    updates, evidence = {}, {}
    for direction in samples:
        values = samples[direction]
        evidence[direction] = dict(samples=len(values), source='approach_imu' if values else 'saved_model',
                                  short_pulses_measured=short_samples[direction],
                                  short_pulses_using_saved_immediate=0)
        if values:
            immediate,total = np.median(values,axis=0)
            updates[f'{direction}_immediate_gain'] = float(immediate)
            updates[f'{direction}_total_gain'] = float(total)
        evidence[direction]['gains'] = [updates.get(f'{direction}_immediate_gain',getattr(coefficients,f'{direction}_immediate_gain')),
                                        updates.get(f'{direction}_total_gain',getattr(coefficients,f'{direction}_total_gain'))]
    fitted = replace(coefficients,**updates)
    # Replay yaw state only: lateral translation is absent from this model.
    # Preserve elapsed time and measured final yaw; do not send lateral drive
    # through the forward-only trajectory simulator or invent a lateral gain.
    state=ResponseState(history_known=True,pending_bound_deg=0.)
    lateral_sec=0.
    for a,b in zip(commands[:-1],commands[1:]):
        duration = min(b[0],end)-max(a[0],start)
        if duration>0:
            if not np.isfinite(a).all():
                raise ValueError('invalid_command_history')
            if a[2]!=0:lateral_sec+=duration
            count=max(1,math.ceil(duration/.04))
            for _ in range(count):
                state=advance_state(state,fitted,float(a[1]),float(a[3]),duration/count)
    evidence['yaw_replay']=dict(lateral_duration_sec=lateral_sec,
        lateral_response_model='not calibrated; elapsed time retained, final yaw anchored to IMU')
    observed = math.degrees(float(np.interp(end,imu[:,0],imu_yaw)-np.interp(start,imu[:,0],imu_yaw)))
    shift = observed-state.yaw_deg
    state = replace(state,yaw_deg=observed,equilibrium_deg=state.equilibrium_deg+shift)
    return fitted,state,evidence


def stage_reacquired_insertion(pose, state, coefficients, distance_cm=38., speed=.10, angular=.35,
                              center_offset_left_cm=0.):
    """Translate to the fresh centreline, then apply the existing yaw inversion.

    Pose is in the CURRENT camera/fork frame, not the initial approach frame.
    Lateral speed uses a nominal 1:1 scale; it has not been measured like forward.
    """
    if distance_cm == 0.:
        return dict(actions=[],distance_cm=0.,reacquired_pose=pose,
                    center_offset_left_cm=center_offset_left_cm,frozen=True)
    if not math.isfinite(center_offset_left_cm):
        raise ValueError('invalid_center_offset')
    local = replace(state, yaw_deg=0., equilibrium_deg=state.equilibrium_deg-state.yaw_deg)
    plan = final_insertion(pose['heading_left_deg'], local, coefficients,
                           distance_cm, speed, angular)
    normal = np.asarray(pose['inward_normal'])
    tangent = np.array([normal[1], -normal[0]])
    end = np.asarray(plan['path'][-1][:2])
    # Account for both insertion drift and the fork's arc during heading trim.
    # Translation is along current vehicle right, so project that axis onto
    # the newly observed centreline's tangent (not a global X coordinate).
    lateral_right = float((np.asarray(pose['top_center_cm'])-end)@tangent/tangent[0])
    nominal_lateral_right = lateral_right
    lateral_right -= center_offset_left_cm
    actions = []
    if abs(lateral_right)>1e-8:
        actions.append(dict(action='stage_lateral_center',
                            drive=[0., -math.copysign(speed, lateral_right), 0.],
                            duration_sec=abs(lateral_right)/(speed*100)))
        actions.append(dict(action='settle_after_lateral', drive=[0.,0.,0.], duration_sec=.5))
    plan.update(actions=actions+plan['actions'], lateral_right_cm=lateral_right,
                nominal_lateral_right_cm=nominal_lateral_right,
                center_offset_left_cm=center_offset_left_cm,
                lateral_scale_assumption=1., reacquired_pose=pose,
                frozen=True,
                path_frame='before_lateral_correction; path excludes lateral translation')
    return plan


def final_insertion(target_heading, state, coefficients, distance_cm=30., speed=.10, angular=.35):
    """Invert predicted yaw after trim + forward release; retain the frozen slot."""
    if not all(math.isfinite(v) for v in (target_heading,distance_cm,speed,angular)) or distance_cm<=0 or speed<.10 or angular<=0:
        raise ValueError('invalid_insertion_inputs')
    straight = dict(action='final_straight_insertion',drive=[speed,0.,0.],
                    duration_sec=distance_cm/(100*speed*coefficients.forward_scale))
    def evaluate(exposure):
        actions = []
        if abs(exposure)>1e-8:
            actions.append(dict(action='measured_heading_trim',drive=[0.,0.,math.copysign(angular,exposure)],
                                duration_sec=abs(math.radians(exposure)/angular)))
        actions.append(straight)
        sim = simulate_actions(actions,state,coefficients)
        return sim['state'].yaw_deg,actions,sim
    base,_,_ = evaluate(0.)
    direction = 1. if target_heading>=base else -1.
    probe,_,_ = evaluate(direction)
    exposure = (target_heading-base)/(probe-base)*direction if abs(probe-base)>1e-9 else 0.
    predicted,actions,sim = evaluate(exposure)
    return dict(actions=actions,target_heading_left_deg=target_heading,
                predicted_final_heading_left_deg=predicted,
                distance_cm=distance_cm,response_state=asdict(state),
                coefficients=asdict(coefficients),path=sim['path'].tolist())


def loaded_visual_plan(pose,state,coefficients,travel_cm,speed=.1,angular=.35,center_offset_left_cm=2.):
    """Replan the observed loaded fork endpoint with turns and forward only."""
    if travel_cm<=0:
        return dict(actions=[],accepted=True,distance_cm=0.)
    normal=np.asarray(pose['inward_normal'],float)
    gap=float(np.asarray(pose['top_center_cm'])@normal)-travel_cm
    goal=np.asarray(pose['top_center_cm'])-gap*normal-np.array([center_offset_left_cm,0.])
    local=replace(state,yaw_deg=0.,equilibrium_deg=state.equilibrium_deg-state.yaw_deg)
    target=dict(pose,stage_right_forward_cm=goal.tolist(),staging_cm=gap)
    plan=solve_side_approach(target,coefficients,speed,angular,.5,state=local)
    add_turn_targets(plan,local,coefficients)
    plan.update(distance_cm=travel_cm,reacquired_pose=pose,center_offset_left_cm=center_offset_left_cm)
    return plan


def add_turn_targets(plan,state,coefficients):
    """Attach predicted relative yaw to each turn for measured early completion."""
    for action in plan['actions']:
        after=simulate_actions([action],state,coefficients)['state']
        if action['drive'][2]!=0:
            action['target_turn_deg']=after.yaw_deg-state.yaw_deg
        state=after


def turn_target_reached(start_yaw_rad,actual_yaw_rad,angular,target_deg):
    """Terminate the same-direction turn at its planned yaw; never reverse it."""
    sign=math.copysign(1.,angular)
    travelled=math.degrees(math.atan2(math.sin(actual_yaw_rad-start_yaw_rad),
                                    math.cos(actual_yaw_rad-start_yaw_rad)))
    return travelled*sign>=max(0.,target_deg*sign)
