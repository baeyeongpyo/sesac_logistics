"""Candidate loaded response model, in cm / seconds / left-positive degrees.

Fork coordinates are right/forward from the initial fork tip. Command vx uses
m/s and wz rad/s. Coefficients are recording-derived candidates, not certified
vehicle geometry. No ROS or scipy dependency; no commands are published here.
"""
from dataclasses import asdict, dataclass, replace
import math
import numpy as np


@dataclass(frozen=True)
class ResponseCoefficients:
    # Loaded vehicle-1 settled-forward-release calibration. Directional gains
    # are authoritative; common gains are their midpoint fallback for zero or
    # otherwise unsigned command yaw.
    immediate_gain: float = 0.18565
    total_gain: float = 1.50914
    release_command_cm: float = 3.80
    settling_sec: float = 0.001
    forward_scale: float = 0.980
    fork_offset_cm: float = 28.7528
    model_id: str = 'vehicle1-loaded-w035-source-time-regression-20260906'
    validated: bool = False
    left_immediate_gain: float | None = 0.1587
    left_total_gain: float | None = 1.401222
    right_immediate_gain: float | None = 0.2126
    right_total_gain: float | None = 1.617065

    def __post_init__(self):
        directional = [value for value in (
            self.left_immediate_gain, self.left_total_gain,
            self.right_immediate_gain, self.right_total_gain) if value is not None]
        values = [self.immediate_gain, self.total_gain, self.release_command_cm,
                  self.settling_sec, self.forward_scale, self.fork_offset_cm,
                  *directional]
        if not all(math.isfinite(v) for v in values) or min(values) < 0:
            raise ValueError('invalid_response_coefficients')
        if min(values[1:5]) <= 0 or self.total_gain < self.immediate_gain:
            raise ValueError('invalid_response_coefficients')
        for immediate, total in (
            (self.left_immediate_gain, self.left_total_gain),
            (self.right_immediate_gain, self.right_total_gain),
        ):
            if ((immediate is None) != (total is None)
                    or (immediate is not None and total < immediate)):
                raise ValueError('invalid_response_coefficients')

    def yaw_gains(self, command_yaw_deg):
        if command_yaw_deg > 0 and self.left_total_gain is not None:
            return self.left_immediate_gain, self.left_total_gain
        if command_yaw_deg < 0 and self.right_total_gain is not None:
            return self.right_immediate_gain, self.right_total_gain
        return self.immediate_gain, self.total_gain


@dataclass(frozen=True)
class ResponseState:
    yaw_deg: float = 0.
    equilibrium_deg: float = 0.
    pending_deg: float = 0.
    history_known: bool = False
    # Conservative latent yaw uncertainty (pending AND equilibrium), retained
    # after release because uncertainty transfers rather than disappears.
    pending_bound_deg: float = math.inf

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.yaw_deg, self.equilibrium_deg,
                                              self.pending_deg)):
            raise ValueError('invalid_response_state')
        if math.isnan(self.pending_bound_deg) or self.pending_bound_deg < 0:
            raise ValueError('invalid_history_bound')


@dataclass(frozen=True)
class StaticSettledCoefficients:
    """Settled, direction-specific yaw calibration for frozen open-loop plans.

    A turn is followed by ``settle_sec`` of zero command before translation,
    so this model has no latent yaw state and no forward-triggered yaw release.
    """
    left_yaw_gain: float = 1.35
    right_yaw_gain: float = 1.14
    forward_scale: float = 1.14659
    fork_offset_cm: float = 28.7528
    settle_sec: float = 1.5
    model_id: str = 'vehicle1-loaded-static-settled-directional-20260906'
    validated: bool = False

    def __post_init__(self):
        values = (self.left_yaw_gain, self.right_yaw_gain,
                  self.forward_scale, self.fork_offset_cm, self.settle_sec)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError('invalid_static_settled_coefficients')

    def yaw_gain(self, command_yaw_deg):
        return self.left_yaw_gain if command_yaw_deg >= 0 else self.right_yaw_gain


def simulate_static_settled_actions(actions, coefficients):
    """Simulate endpoint samples for settled turns and forward-only motion."""
    c = coefficients
    body_right = body_forward = yaw_deg = 0.
    rows = [[0., 0., 0.]]
    for action in actions:
        vx, vy, wz = action['drive']
        duration = float(action['duration_sec'])
        if (not all(math.isfinite(value) for value in (vx, vy, wz, duration))
                or duration < 0 or duration > 120 or vy != 0 or vx < 0
                or (vx != 0 and wz != 0)):
            raise ValueError('unsupported_static_settled_command')
        command_yaw_deg = math.degrees(wz*duration)
        if command_yaw_deg:
            yaw_deg += c.yaw_gain(command_yaw_deg)*command_yaw_deg
        if vx:
            distance_cm = c.forward_scale*vx*100.*duration
            angle = math.radians(yaw_deg)
            body_right -= distance_cm*math.sin(angle)
            body_forward += distance_cm*math.cos(angle)
        angle = math.radians(yaw_deg)
        rows.append([
            body_right-c.fork_offset_cm*math.sin(angle),
            body_forward+c.fork_offset_cm*(math.cos(angle)-1.),
            yaw_deg,
        ])
    return np.asarray(rows)


def plan_static_settled_approach(top_right_cm, top_forward_cm,
                                 heading_left_deg, coefficients=None, *,
                                 staging_cm=40., final_forward_cm,
                                 speed_m_s=.10, angular_rad_s=.35):
    """Create one frozen forward-only plan using fully settled turn gains.

    ``final_forward_cm`` is deliberately required: the caller passes its
    existing trim calculation rather than this planner inventing a distance.
    The preceding turn, straight, and final turn are solved analytically so the
    fork tip reaches the requested staging right/forward/yaw pose together. No
    runtime feedback or response state enters the returned command array.
    """
    c = coefficients or StaticSettledCoefficients()
    values = (top_right_cm, top_forward_cm, heading_left_deg, staging_cm,
              final_forward_cm, speed_m_s, angular_rad_s)
    if (not all(math.isfinite(value) for value in values)
            or min(staging_cm, final_forward_cm, speed_m_s,
                   angular_rad_s) <= 0):
        raise ValueError('invalid_static_settled_plan_inputs')
    if speed_m_s < .10:
        raise ValueError('invalid_static_settled_plan_inputs')

    heading = math.radians(heading_left_deg)
    normal = np.array([-math.sin(heading), math.cos(heading)])
    tangent = np.array([math.cos(heading), math.sin(heading)])
    centre = np.array([top_right_cm, top_forward_cm], dtype=float)
    goal = centre-staging_cm*normal
    final_offset = c.fork_offset_cm*np.array([
        -math.sin(heading), math.cos(heading)-1.])
    final_direction = np.array([-math.sin(heading), math.cos(heading)])
    first_translation = goal-final_offset-final_forward_cm*final_direction
    first_forward_cm = float(np.linalg.norm(first_translation))
    first_heading_deg = (math.degrees(math.atan2(-first_translation[0],
                                                  first_translation[1]))
                         if first_forward_cm > 1e-9 else heading_left_deg)
    second_turn_deg = heading_left_deg-first_heading_deg
    if goal[1] < 0:
        return dict(accepted=False, reason='approach_needs_reverse_clearance',
                    model=asdict(c), actions=[])

    actions = []

    def add_turn(actual_yaw_deg, label):
        if abs(actual_yaw_deg) < 1e-9:
            return
        gain = c.left_yaw_gain if actual_yaw_deg > 0 else c.right_yaw_gain
        command_yaw_deg = actual_yaw_deg/gain
        wz = math.copysign(angular_rad_s, command_yaw_deg)
        actions.append(dict(action=label, drive=(0., 0., wz),
                            duration_sec=abs(math.radians(command_yaw_deg)/wz)))
        actions.append(dict(action=f'{label}_settle', drive=(0., 0., 0.),
                            duration_sec=c.settle_sec))

    def add_forward(distance_cm, label):
        if distance_cm < 1e-9:
            return
        actions.append(dict(action=label, drive=(speed_m_s, 0., 0.),
                            duration_sec=distance_cm/(c.forward_scale*100.*speed_m_s)))

    add_turn(first_heading_deg, 'turn_left' if first_heading_deg > 0 else 'turn_right')
    add_forward(first_forward_cm, 'forward')
    add_turn(second_turn_deg, 'final_turn_left' if second_turn_deg > 0
             else 'final_turn_right')
    add_forward(final_forward_cm, 'short_forward')
    path = simulate_static_settled_actions(actions, c)
    end = path[-1]
    return dict(
        accepted=True, reason='ok', model=asdict(c), actions=actions,
        predicted_stage=end.tolist(), path=path.tolist(),
        stage_gap_cm=float((centre-end[:2])@normal),
        stage_lateral_cm=float((centre-end[:2])@tangent),
        target_stage=[float(goal[0]), float(goal[1]), heading_left_deg],
    )


def advance_state(state, coefficients, vx_m_s, wz_rad_s, dt):
    """Advance one recorded exposure. Stop preserves memory; reverse is unknown.

    Use short timestamped intervals (<= 40ms for runtime prediction). This is
    the discrete law used by the original recording fit, not a new ODE fit.
    """
    if not all(math.isfinite(x) for x in (vx_m_s, wz_rad_s, dt)) or dt < 0:
        raise ValueError('invalid_command_exposure')
    if vx_m_s < 0:
        raise ValueError('reverse_response_unidentified')
    c = coefficients
    du = math.degrees(wz_rad_s * dt)
    immediate_gain, total_gain = c.yaw_gains(du)
    pending = state.pending_deg + (total_gain-immediate_gain)*du
    decay = math.exp(-vx_m_s*100.*dt/c.release_command_cm)
    release = pending*(1.-decay)
    equilibrium = state.equilibrium_deg+immediate_gain*du+release
    yaw = state.yaw_deg+(equilibrium-state.yaw_deg)*(-math.expm1(-dt/c.settling_sec))
    # Release transfers uncertainty into equilibrium/actual yaw; forward
    # exposure alone cannot certify that the initial uncertainty disappeared.
    bound = state.pending_bound_deg
    return replace(state, yaw_deg=yaw, equilibrium_deg=equilibrium,
                   pending_deg=pending-release, pending_bound_deg=bound)


def simulate_actions(actions, state, coefficients, *, step_sec=.04):
    """Return fork trajectory and response state, with initial yaw rebased to 0.

    Integrates translation at midpoint actual yaw plus the effective fork
    rotation offset. A stationary turn can therefore move the fork tip.
    """
    if not math.isfinite(step_sec) or step_sec <= 0:
        raise ValueError('invalid_step_sec')
    initial_yaw = state.yaw_deg
    current = state
    body_x = body_y = elapsed = 0.
    rows = [[0., 0., 0.]]
    times = [0.]
    for action in actions:
        vx, vy, wz = action['drive']
        duration = float(action['duration_sec'])
        if (not all(math.isfinite(x) for x in (vx, vy, wz, duration))
                or duration < 0 or duration > 120 or vy != 0 or vx < 0):
            raise ValueError('unsupported_command')
        count = max(1, math.ceil(duration/step_sec))
        dt = duration/count
        for _ in range(count):
            nxt = advance_state(current, coefficients, vx, wz, dt)
            mid = math.radians((current.yaw_deg+nxt.yaw_deg)/2-initial_yaw)
            body_x -= coefficients.forward_scale*vx*100*dt*math.sin(mid)
            body_y += coefficients.forward_scale*vx*100*dt*math.cos(mid)
            theta = math.radians(nxt.yaw_deg-initial_yaw)
            rows.append([body_x-coefficients.fork_offset_cm*math.sin(theta),
                         body_y+coefficients.fork_offset_cm*(math.cos(theta)-1),
                         nxt.yaw_deg-initial_yaw])
            elapsed += dt
            times.append(elapsed)
            current = nxt
    return dict(path=np.asarray(rows), seconds=np.asarray(times), state=current)


def straight_tail_prediction(state, coefficients, *, speed_m_s=.1,
                             insertion_cm=20., step_sec=.04):
    if not math.isfinite(speed_m_s) or speed_m_s < .10 or insertion_cm <= 0:
        raise ValueError('invalid_insertion_command')
    action = dict(action='forward', drive=(speed_m_s, 0., 0.),
                  duration_sec=insertion_cm/(100*speed_m_s*coefficients.forward_scale))
    return simulate_actions([action], state, coefficients, step_sec=step_sec)


def insertion_check(top_right_cm, top_forward_cm, heading_left_deg, state,
                    coefficients, *, staging_cm=40., insertion_cm=20.,
                    speed_m_s=.1, lateral_tolerance_cm=1.5,
                    heading_tolerance_deg=2., gap_tolerance_cm=1.):
    """Verify fresh top-line pose AND predicted tail; include history uncertainty."""
    if not all(math.isfinite(x) for x in (top_right_cm, top_forward_cm,
                                          heading_left_deg)):
        raise ValueError('invalid_top_line_pose')
    tail = straight_tail_prediction(state, coefficients, speed_m_s=speed_m_s,
                                    insertion_cm=insertion_cm)['path']
    theta = math.radians(heading_left_deg)
    tangent = np.array([math.cos(theta), math.sin(theta)])
    normal = np.array([-math.sin(theta), math.cos(theta)])
    remaining = np.array([top_right_cm, top_forward_cm])-tail[:, :2]
    lateral, gaps = remaining@tangent, remaining@normal
    yaw = (heading_left_deg-tail[:, 2]+180)%360-180
    bound = state.pending_bound_deg
    reasons = []
    if not math.isfinite(bound) or not state.history_known:
        reasons.append('history_unobservable')
    # The fork offset also moves laterally under uncertain rotation.
    yaw_uncertainty = min(bound, 90.)
    lateral_bound = (insertion_cm+coefficients.fork_offset_cm)*math.sin(math.radians(yaw_uncertainty))
    if abs(gaps[0]-staging_cm) > gap_tolerance_cm:
        reasons.append('stage_distance')
    if abs(lateral[0]) > lateral_tolerance_cm:
        reasons.append('stage_center')
    if abs(yaw[0]) > heading_tolerance_deg:
        reasons.append('top_line_yaw')
    if np.max(np.abs(lateral))+lateral_bound > lateral_tolerance_cm:
        reasons.append('straight_tail_lateral_drift')
    if np.max(np.abs(yaw))+bound > heading_tolerance_deg:
        reasons.append('straight_tail_yaw_or_unreleased_response')
    if abs(gaps[-1]-(staging_cm-insertion_cm)) > gap_tolerance_cm:
        reasons.append('straight_tail_distance')
    if np.any(np.diff(gaps) > 1e-6):
        reasons.append('straight_tail_reverses')
    return dict(accepted=not reasons, reasons=reasons,
                max_tail_lateral_cm=float(np.max(np.abs(lateral))),
                max_tail_yaw_deg=float(np.max(np.abs(yaw))),
                gap_cm=float(gaps[0]), lateral_cm=float(lateral[0]),
                pending_bound_deg=bound, tail=tail.tolist())


def plan_approach(top_right_cm, top_forward_cm, heading_left_deg, state,
                  coefficients, *, staging_cm=40., insertion_cm=20.,
                  speed_m_s=.10, angular_rad_s=.35, settle_sec=.5,
                  lateral_tolerance_cm=1.5, heading_tolerance_deg=2.,
                  gap_tolerance_cm=1., max_iterations=24):
    """Bounded whole-trajectory inversion; execution must freeze returned actions.

    Five exposures (turn/forward/turn/forward/turn) control fork x/y, stage
    heading, equilibrium and pending response. Three deterministic initial
    seeds use damped finite-difference least squares; no online vision enters
    this optimization. No collision/free-space claim is made by this module.
    """
    c = coefficients
    values = (top_right_cm, top_forward_cm, heading_left_deg, staging_cm,
              insertion_cm, speed_m_s, angular_rad_s, settle_sec,
              lateral_tolerance_cm, heading_tolerance_deg, gap_tolerance_cm)
    if not all(math.isfinite(x) for x in values) or min(values[3:]) <= 0:
        raise ValueError('invalid_plan_inputs')
    if speed_m_s < .10 or max_iterations < 1 or max_iterations > 50:
        raise ValueError('invalid_plan_limits')
    meta = dict(model=asdict(c), initial_state=asdict(state), actions=[])
    if not state.history_known or not math.isfinite(state.pending_bound_deg):
        # Expose the conditional solution for diagnosis, never as executable
        # actions. Zero uncertainty here is explicitly a shadow assumption.
        assumed = replace(state, history_known=True, pending_bound_deg=0.)
        shadow = plan_approach(
            top_right_cm, top_forward_cm, heading_left_deg, assumed, c,
            staging_cm=staging_cm, insertion_cm=insertion_cm,
            speed_m_s=speed_m_s, angular_rad_s=angular_rad_s,
            settle_sec=settle_sec, lateral_tolerance_cm=lateral_tolerance_cm,
            heading_tolerance_deg=heading_tolerance_deg,
            gap_tolerance_cm=gap_tolerance_cm, max_iterations=max_iterations)
        return dict(meta, accepted=False, reason='history_unobservable',
                    shadow_actions=shadow['actions'],
                    shadow_assumption='supplied_latent_state_assumed_exact_for_diagnostics_only',
                    shadow_predicted_stage=shadow.get('predicted_stage'))
    theta = math.radians(heading_left_deg)
    normal = np.array([-math.sin(theta), math.cos(theta)])
    tangent = np.array([math.cos(theta), math.sin(theta)])
    centre = np.array([top_right_cm, top_forward_cm])
    goal = centre-staging_cm*normal
    if goal[1] < 0 or np.linalg.norm(goal) > 120 or abs(heading_left_deg) > 45:
        return dict(meta, accepted=False, reason='approach_needs_reverse_clearance')

    def actions_for(p):
        actions = []
        for i, amount in enumerate(p):
            if abs(amount) < 1e-8:
                continue
            if i % 2 == 0:
                wz = math.copysign(angular_rad_s, amount)
                actions.append(dict(action='turn_left' if amount > 0 else 'turn_right',
                                    drive=(0., 0., wz),
                                    duration_sec=abs(math.radians(amount)/angular_rad_s)))
            else:
                actions.append(dict(action='forward', drive=(speed_m_s, 0., 0.),
                                    duration_sec=amount/(100*speed_m_s)))
                actions.append(dict(action='settle_after_forward',
                                    drive=(0., 0., 0.),
                                    duration_sec=settle_sec))
        return actions

    def evaluate(p, details=False):
        sim = simulate_actions(actions_for(p), state, c)
        end = sim['path'][-1]
        final = sim['state']
        # Pending and unsettled yaw constrain the full 20cm straight tail,
        # not merely the orientation of the stationary image.
        residual = np.array([(end[0]-goal[0])/lateral_tolerance_cm,
                             (end[1]-goal[1])/gap_tolerance_cm,
                             (end[2]-heading_left_deg)/heading_tolerance_deg,
                             final.pending_deg/heading_tolerance_deg,
                             (final.equilibrium_deg-final.yaw_deg)/heading_tolerance_deg])
        return (residual, sim) if details else residual

    travel = goal-c.fork_offset_cm*(normal-np.array([0., 1.]))
    bearing = math.degrees(math.atan2(-travel[0], max(travel[1], 1e-6)))
    distance = float(np.linalg.norm(travel))/c.forward_scale
    lo = np.array([-65., 0., -65., 0., -65.])
    hi = np.array([65., 120., 65., 120., 65.])
    best = None
    evaluations = 0
    for split in (.3, .6, .85):
        first_gain = c.yaw_gains(bearing)[1]
        second_angle = heading_left_deg-bearing
        second_gain = c.yaw_gains(second_angle)[1]
        p = np.clip([bearing/first_gain, distance*split,
                     second_angle/second_gain,
                     distance*(1-split), 0.], lo, hi)
        damping = .01
        for _ in range(max_iterations):
            r = evaluate(p); evaluations += 1
            score = float(r@r)
            if best is None or score < best[0]:
                best = (score, p.copy())
            if score < .0025:
                break
            jac = np.empty((5, 5))
            for j in range(5):
                probe = p.copy()
                delta = .06 if p[j]+.06 <= hi[j] else -.06
                probe[j] += delta
                jac[:, j] = (evaluate(probe)-r)/delta
                evaluations += 1
            step = np.linalg.solve(jac.T@jac+damping*np.eye(5), -jac.T@r)
            candidate = np.clip(p+np.clip(step, -12., 12.), lo, hi)
            trial = evaluate(candidate); evaluations += 1
            if float(trial@trial) < score:
                p, damping = candidate, max(damping*.4, 1e-6)
            else:
                damping = min(damping*6, 1e6)
    _, p = best
    _, sim = evaluate(p, True)
    end = sim['path'][-1]
    # Transform fixed top line into the final fork frame for tail checking.
    a = math.radians(end[2])
    rotation = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
    observed = rotation@(centre-end[:2])
    check = insertion_check(*observed, heading_left_deg-end[2], sim['state'], c,
                            staging_cm=staging_cm, insertion_cm=insertion_cm,
                            speed_m_s=speed_m_s, lateral_tolerance_cm=lateral_tolerance_cm,
                            heading_tolerance_deg=heading_tolerance_deg,
                            gap_tolerance_cm=gap_tolerance_cm)
    robust_checks = []
    if state.pending_bound_deg > 0:
        # A hidden initial turn can affect the entire approach before its
        # pending component decays; checking only final pending loses that.
        for eq_sign, pending_sign in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            alternative = replace(state,
                equilibrium_deg=state.equilibrium_deg+eq_sign*state.pending_bound_deg,
                pending_deg=state.pending_deg+pending_sign*state.pending_bound_deg,
                pending_bound_deg=0.)
            variant = simulate_actions(actions_for(p), alternative, c)
            v_end = variant['path'][-1]
            angle = math.radians(v_end[2])
            frame = np.array([[math.cos(angle), math.sin(angle)],
                              [-math.sin(angle), math.cos(angle)]])
            top = frame@(centre-v_end[:2])
            robust_checks.append(insertion_check(
                *top, heading_left_deg-v_end[2], variant['state'], c,
                staging_cm=staging_cm, insertion_cm=insertion_cm, speed_m_s=speed_m_s,
                lateral_tolerance_cm=lateral_tolerance_cm,
                heading_tolerance_deg=heading_tolerance_deg,
                gap_tolerance_cm=gap_tolerance_cm))
    accepted = check['accepted'] and all(v['accepted'] for v in robust_checks)
    return dict(meta, accepted=accepted,
                reason='ok' if accepted else 'no_feasible_response_plan',
                history_scenarios=robust_checks,
                actions=actions_for(p), predicted_stage=end.tolist(),
                predicted_state=asdict(sim['state']), insertion=check,
                path=sim['path'].tolist(), evaluations=evaluations,
                stage_gap_cm=float((centre-end[:2])@normal),
                stage_lateral_cm=float((centre-end[:2])@tangent))


def estimate_history(seconds, vx_m_s, wz_rad_s, observed_yaw_deg, coefficients,
                     *, yaw_noise_deg=.25, condition_limit=1000., effective_sample_count=None):
    """Estimate latent initial equilibrium/pending using actual command history.

    Arrays have equal length; command i applies on [t_i,t_(i+1)]. Observed
    yaw is unwrapped internally. Reverse invalidates this forward-only law:
    callers must provide a window wholly after the last reverse. The reported
    bound is a regression uncertainty estimate conditional on this candidate
    model, NOT a validated worst-case vehicle error guarantee.
    """
    arrays = [np.asarray(x, dtype=float) for x in
              (seconds, vx_m_s, wz_rad_s, observed_yaw_deg)]
    t, vx, wz, yaw = arrays
    if (any(a.ndim != 1 for a in arrays) or len(t) < 4
            or any(len(a) != len(t) for a in arrays)
            or not all(np.isfinite(a).all() for a in arrays)
            or np.any(np.diff(t) <= 0) or yaw_noise_deg <= 0):
        raise ValueError('invalid_history_samples')
    if np.any(vx < 0):
        return dict(state=ResponseState(), reason='reverse_response_unidentified',
                    observable=False)
    yaw = np.degrees(np.unwrap(np.radians(yaw)))
    yaw -= yaw[0]

    def replay(eq, pending):
        state = ResponseState(equilibrium_deg=eq, pending_deg=pending,
                              history_known=True, pending_bound_deg=0.)
        out = [0.]
        for i, dt in enumerate(np.diff(t)):
            # Preserve measured recording intervals; fitting law was discrete.
            state = advance_state(state, coefficients, vx[i], wz[i], float(dt))
            out.append(state.yaw_deg)
        return np.asarray(out), state

    baseline, base_end = replay(0., 0.)
    e, e_end = replay(1., 0.)
    p, p_end = replay(0., 1.)
    design = np.column_stack([e-baseline, p-baseline])
    _, singular, vt = np.linalg.svd(design, full_matrices=False)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else math.inf
    # We need the CURRENT latent state, not an identifiable initial split.
    # A fully released initial pending impulse may be indistinguishable from
    # initial equilibrium yet have no remaining unobservable effect now.
    end_map = np.array([[e_end.equilibrium_deg-base_end.equilibrium_deg,
                         p_end.equilibrium_deg-base_end.equilibrium_deg],
                        [e_end.pending_deg-base_end.pending_deg,
                         p_end.pending_deg-base_end.pending_deg]])
    retained = singular > max(1e-12, singular[0]/condition_limit)
    null_effect = end_map@vt[~retained].T
    if not retained.any() or (null_effect.size and np.linalg.norm(null_effect)>1e-10):
        return dict(state=ResponseState(yaw_deg=float(yaw[-1]),
                                        equilibrium_deg=float(yaw[-1])),
                    observable=False, reason='history_unobservable', condition=condition)
    inverse = np.linalg.pinv(design, rcond=1./condition_limit)
    initial = inverse@(yaw-baseline)
    prediction, fitted = replay(*initial)
    residual = prediction-yaw
    if effective_sample_count is None:
        effective_sample_count = len(t)
    if (not math.isfinite(effective_sample_count) or effective_sample_count < 3
            or effective_sample_count > len(t)):
        raise ValueError('invalid_effective_sample_count')
    # Union command edges may add interpolated yaw samples. They are not
    # independent observations; conservatively inflate both floor and RMSE.
    interpolation_inflation = math.sqrt(len(t)/effective_sample_count)
    sigma = max(yaw_noise_deg, float(np.sqrt(np.mean(residual**2))))*interpolation_inflation
    covariance = sigma**2*(end_map@inverse)@(end_map@inverse).T
    current_bound = 3.*max(yaw_noise_deg, math.sqrt(max(0.,float(np.linalg.eigvalsh(covariance)[-1]))))
    result = replace(fitted, yaw_deg=float(yaw[-1]), pending_bound_deg=current_bound)
    return dict(state=result, observable=True, reason='estimated_candidate_history',
                condition=condition, rmse_deg=float(np.sqrt(np.mean(residual**2))),
                max_error_deg=float(np.max(np.abs(residual))),
                initial_equilibrium_deg=float(initial[0]), initial_pending_deg=float(initial[1]),
                initial_state_identifiable=bool(retained.all()),
                uncertainty_kind='three_sigma_conditional_regression_not_certified',
                samples=len(t), effective_sample_count=effective_sample_count,
                interpolation_uncertainty_inflation=interpolation_inflation)
