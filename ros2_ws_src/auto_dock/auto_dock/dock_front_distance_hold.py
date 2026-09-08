"""Optional, removable front-tag distance hold for DOCK PICK lateral motion."""
import math
import statistics
import time


def adjust(host, x, y, yaw):
    if (getattr(host, 'mission_kind', None) != 'DOCK_PICK'
            or not getattr(host, 'config', {}).get('dock_lateral_front_hold_enabled', True)):
        host.dock_front_distance_hold = None
        return x, y, yaw
    # Stops always pass through. Deliberate forward/reverse/rotation starts a
    # new distance reference next time lateral motion begins.
    if abs(y) < 1e-6 or abs(yaw) > 1e-6:
        if abs(x) > 1e-6 or abs(yaw) > 1e-6:
            host.dock_front_distance_hold = None
        return x, y, yaw
    detection = getattr(host, 'latest_detection', None) or {}
    if time.monotonic() - getattr(host, 'latest_detection_at', -math.inf) > .8:
        return x, y, yaw
    distances = []
    for tag in detection.get('detections') or []:
        if tag.get('class') not in {'star', 'diamond', 'spade', 'clover', 'heart'}:
            continue
        depth = tag.get('depth') or {}
        try:
            distance = float(depth['forward_distance_cm'])
            bearing = float(depth['bearing_deg'])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(distance) and distance > 0 and math.isfinite(bearing) and abs(bearing) <= 20.:
            distances.append(distance)
    if not distances:
        return x, y, yaw
    distance = statistics.median(distances)
    state = getattr(host, 'dock_front_distance_hold', None)
    if state is None:
        state = dict(reference_cm=distance, correcting=False)
        host.dock_front_distance_hold = state
    error = distance - state['reference_cm']
    if abs(error) >= 2.:
        state['correcting'] = True
    elif abs(error) <= 1.:
        state['correcting'] = False
    state.update(observed_cm=distance, error_cm=error)
    # Separate axes: mixing x=.10 with y=.12 leaves wheel components at .02.
    # A brief pure longitudinal correction keeps every driven wheel >= .10.
    if state['correcting']:
        return math.copysign(.10, error), 0., 0.
    return 0., y, yaw
