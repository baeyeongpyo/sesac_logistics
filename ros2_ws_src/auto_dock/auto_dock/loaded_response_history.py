"""Align receipt-time commands and source-time odometry for latent yaw fitting.

Command receipt time is only a proxy for hardware application time. No source
stamp exists in Twist; this adapter cannot establish undetectable transport lag.
The caller must clear both buffers or pass reset_time on a load-state change.
"""
import math

import numpy as np

from .loaded_response_planner import ResponseState, advance_state, estimate_history


TIME_ASSUMPTION = ('cmd_vel receipt monotonic approximates hardware application; '
                   'odom timestamps are source times mapped to the same monotonic clock; '
                   'unmeasured command transport delay is not identifiable')


def estimate_from_history(command_rows, yaw_rows, coefficients, now, max_age=.35,
                          *, reset_time=None, max_gap=.35):
    """Return state/observable/reason from (t,vx,vy,wz) and (t,yaw_rad).

    Reject incomplete/stale/discontinuous input. All command edges are retained;
    yaw is interpolated only between valid source samples. A prior reverse or
    lateral command discards history through the next supported command. This
    is candidate-model identification, not a certified bound on vehicle error.
    """
    def fail(reason, **extra):
        return dict(state=ResponseState(), observable=False, reason=reason,
                    timestamp_assumption=TIME_ASSUMPTION, **extra)

    try:
        commands = np.asarray(list(command_rows), dtype=float)
        observed = np.asarray(list(yaw_rows), dtype=float)
    except (ValueError, TypeError):
        return fail('invalid_history_rows')
    if (not math.isfinite(now) or not math.isfinite(max_age) or max_age <= 0
            or not math.isfinite(max_gap) or max_gap <= 0
            or (reset_time is not None and not math.isfinite(reset_time))):
        return fail('invalid_history_clock')
    if (commands.ndim != 2 or commands.shape[1] != 4
            or observed.ndim != 2 or observed.shape[1] != 2
            or len(commands) < 2 or len(observed) < 4
            or not np.isfinite(commands).all() or not np.isfinite(observed).all()):
        return fail('insufficient_or_invalid_history')
    if np.any(np.diff(commands[:, 0]) <= 0) or np.any(np.diff(observed[:, 0]) <= 0):
        return fail('nonmonotonic_history_timestamps')
    if commands[-1, 0] > now+1e-6 or observed[-1, 0] > now+1e-6:
        return fail('future_history_timestamp')
    if now-commands[-1, 0] > max_age or now-observed[-1, 0] > max_age:
        return fail('stale_history')

    lower = max(commands[0, 0], observed[0, 0], reset_time if reset_time is not None else -math.inf)
    unsupported = np.flatnonzero((commands[:, 1] < -1e-9) | (np.abs(commands[:, 2]) > 1e-9))
    trimmed = False
    if len(unsupported):
        index = int(unsupported[-1])+1
        if index == len(commands):
            return fail('reverse_or_lateral_response_unidentified')
        lower = max(lower, commands[index, 0])
        trimmed = True
    # Begin on an actually measured yaw sample after the state reset.
    accepted = observed[observed[:, 0] >= lower]
    if len(accepted) < 4:
        return fail('insufficient_post_reset_history', trimmed_unsupported_motion=trimmed)
    lower, upper = accepted[0, 0], accepted[-1, 0]
    if np.any(np.diff(accepted[:, 0]) > max_gap+1e-9):
        return fail('odom_source_gap')
    first_command = int(np.searchsorted(commands[:, 0], lower, side='right')-1)
    relevant = commands[first_command:]
    if lower-relevant[0, 0] > max_gap or np.any(np.diff(relevant[:, 0]) > max_gap+1e-9):
        return fail('command_receipt_gap')
    if now-relevant[-1, 0] > max_gap:
        return fail('command_receipt_gap')

    edges = relevant[(relevant[:, 0] > lower) & (relevant[:, 0] < upper), 0]
    times = np.unique(np.r_[accepted[:, 0], edges])
    indices = np.searchsorted(commands[:, 0], times, side='right')-1
    vx, wz = commands[indices, 1], commands[indices, 3]
    forward_cm = float(np.sum(vx[:-1]*np.diff(times))*100.)
    if forward_cm <= 1e-6:
        return fail('forward_excitation_required')
    yaw = np.degrees(np.interp(times, accepted[:, 0], np.unwrap(accepted[:, 1])))
    result = estimate_history(times, vx, wz, yaw, coefficients,
                              effective_sample_count=len(accepted))
    result.update(timestamp_assumption=TIME_ASSUMPTION,
                  fitted_time_range_sec=[float(lower), float(upper)],
                  measured_yaw_samples=len(accepted), interpolated_yaw_samples=len(times)-len(accepted),
                  forward_command_cm=forward_cm, trimmed_unsupported_motion=trimmed,
                  load_reset_time=reset_time, state_timestamp=float(upper),
                  uncertainty_caveat='interpolated yaw samples are correlated; reported regression bound is conditional, not certified')
    if not result['observable']:
        return result
    # The last observation can lag now by at most max_age. Preserve every known
    # command edge and propagate that short unobserved interval explicitly.
    tail = np.unique(np.r_[upper, commands[(commands[:, 0] > upper) & (commands[:, 0] < now), 0], now])
    state = result['state']
    for lo, hi in zip(tail[:-1], tail[1:]):
        command = commands[np.searchsorted(commands[:, 0], lo, side='right')-1]
        steps = max(1, int(math.ceil((hi-lo)/.04)))
        for _ in range(steps):
            state = advance_state(state, coefficients, float(command[1]), float(command[3]), float((hi-lo)/steps))
    result.update(state=state, state_timestamp=float(now), unobserved_propagation_sec=float(now-upper))
    return result
