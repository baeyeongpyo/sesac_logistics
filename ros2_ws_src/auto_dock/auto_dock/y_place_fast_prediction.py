"""Endpoint-only replay of the existing 40ms model for Y's optimizer."""
import math
import numpy as np


def endpoint(actions, state, c):
    initial = yaw = state.yaw_deg
    equilibrium, pending = state.equilibrium_deg, state.pending_deg
    x = y = 0.0
    for action in actions:
        vx, vy, wz = action['drive']
        duration = float(action['duration_sec'])
        if (not all(math.isfinite(v) for v in (vx, vy, wz, duration))
                or duration < 0 or duration > 120 or vy != 0 or vx < 0):
            raise ValueError('unsupported_command')
        count = max(1, math.ceil(duration / .04))
        dt = duration / count
        du = math.degrees(wz * dt)
        immediate, total = c.yaw_gains(du)
        decay = math.exp(-vx * 100. * dt / c.release_command_cm)
        settle = -math.expm1(-dt / c.settling_sec)
        for _ in range(count):
            charged = pending + (total - immediate) * du
            release = charged * (1. - decay)
            equilibrium = equilibrium + immediate * du + release
            nxt = yaw + (equilibrium - yaw) * settle
            pending = charged - release
            mid = math.radians((yaw + nxt) / 2 - initial)
            x -= c.forward_scale * vx * 100 * dt * math.sin(mid)
            y += c.forward_scale * vx * 100 * dt * math.cos(mid)
            yaw = nxt
    theta = math.radians(yaw - initial)
    return np.array([x - c.fork_offset_cm * math.sin(theta),
                     y + c.fork_offset_cm * (math.cos(theta) - 1), yaw - initial])
