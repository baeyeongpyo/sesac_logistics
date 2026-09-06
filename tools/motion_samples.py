"""Extract rotation/forward experiments from time-stamped recording sidecars."""
import json
import math
from pathlib import Path

SCHEMA = {
    "schema_version": 1,
    "unit": "one rotation, stop, forward, stop sequence",
    "states": ["before", "after_rotation", "after_forward"],
    "state_fields": {
        "center_x_px": "tracked slot X horizontal position, pixels",
        "center_y_ratio": "tracked slot X vertical position / image height",
        "border_angle_deg": "image-plane border angle; NOT metric vehicle yaw",
        "yaw_image_deg": "atan2(top_right.y-top_left.y, top_right.x-top_left.x), degrees",
        "depth_top_y_px": "top border midpoint y in pixels; not distance cm",
        "depth_top_y_ratio": "top border midpoint y / image height",
        "yaw_deg": "null until metric slot pose calibration is available",
        "depth_cm": "null until slot pose calibration is available",
    },
    "actions": "actual received cmd_vel segments, with timestamps and velocities",
    "valid_visual_sample": "same track, stationary fresh stable views at all three states",
    "metric_pose_available": False,
}


def top_border_measurement(observation, image_height):
    """Image-space yaw and depth proxy from the tracked far border endpoints."""
    center_y = observation["center_y_ratio"] * image_height
    candidates = []
    for line in observation.get("border_lines", []):
        x1, y1, x2, y2 = map(float, line)
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
            continue
        if x2 < x1:
            x1, y1, x2, y2 = x2, y2, x1, y1
        span = x2-x1
        if span < 60 or (y1+y2)/2 >= center_y:
            continue
        candidates.append((span, x1, y1, x2, y2))
    if not candidates:
        return {}
    # Prefer a long edge over a short clutter edge above the same X.
    _, x1, y1, x2, y2 = max(candidates)
    return {"top_left_px": [x1, y1], "top_right_px": [x2, y2],
            "yaw_image_deg": math.degrees(math.atan2(y2-y1, x2-x1)),
            "depth_top_y_px": (y1+y2)/2,
            "depth_top_y_ratio": (y1+y2)/(2*image_height)}


def read_rows(path):
    with Path(path).open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def command_segments(rows):
    commands = [r for r in rows if r["type"] == "cmd_vel"]
    segments = []
    end = rows[-1]["monotonic"] if rows else 0.0
    for index, row in enumerate(commands):
        values = tuple(row[k] for k in ("linear_x", "linear_y", "angular_z"))
        stop = commands[index+1]["monotonic"] if index+1 < len(commands) else end
        # A missing command stream must never imply sustained motion.
        stop = min(stop, row["monotonic"] + 0.35)
        if values == (0.0, 0.0, 0.0):
            kind = "stop"
        elif values[0] == values[1] == 0.0:
            kind = "rotation"
        elif values[0] > 0.0 and values[1] == values[2] == 0.0:
            kind = "forward"
        else:
            kind = "other"
        item = dict(start=row["monotonic"], end=stop, kind=kind,
                    linear_x=values[0], linear_y=values[1], angular_z=values[2])
        if (segments and segments[-1]["kind"] == kind
                and all(segments[-1][k] == item[k] for k in
                        ("linear_x", "linear_y", "angular_z"))
                and item["start"] - segments[-1]["end"] < 0.001):
            segments[-1]["end"] = stop
        else:
            segments.append(item)
    return segments


def stable_state(observations, lower, upper):
    candidates = [r for r in observations if lower <= r["monotonic"] <= upper
                  and r.get("valid") and r.get("source_age_sec", 99) <= 0.35]
    for end in range(len(candidates), 2, -1):
        group = candidates[end-3:end]
        if (group[-1]["monotonic"]-group[0]["monotonic"] < 0.3
                or group[-1]["monotonic"]-group[0]["monotonic"] > 1.0
                or len({r["track_id"] for r in group}) != 1):
            continue
        if any(max(r[k] for r in group)-min(r[k] for r in group) > tolerance
               for k, tolerance in (("center_x_px", 5.0),
                                    ("center_y_ratio", 0.015), ("border_angle_deg", 2.0))):
            continue
        return group[-1]
    return None


def extract_samples(controls, observations, session):
    segments = command_segments(controls)
    moving = [s for s in segments if s["kind"] != "stop" and s["end"] > s["start"]]
    # Merge short tap/repeat gaps of the same motion into one action group,
    # retaining every individual command interval for regression inputs.
    groups = []
    for segment in moving:
        if (groups and groups[-1]["kind"] == segment["kind"]
                and segment["start"]-groups[-1]["end"] < 0.5):
            groups[-1]["segments"].append(segment)
            groups[-1]["end"] = segment["end"]
        else:
            groups.append(dict(kind=segment["kind"], start=segment["start"],
                               end=segment["end"], segments=[segment]))
    result = []
    for index in range(len(groups)-1):
        rotation, forward = groups[index:index+2]
        if rotation["kind"] != "rotation" or forward["kind"] != "forward":
            continue
        previous_end = groups[index-1]["end"] if index else controls[0]["monotonic"]
        next_start = groups[index+2]["start"] if index+2 < len(groups) else controls[-1]["monotonic"]
        before = stable_state(observations, max(previous_end+0.3, rotation["start"]-2), rotation["start"])
        middle = stable_state(observations, rotation["end"]+0.3, forward["start"])
        after = stable_state(observations, forward["end"]+0.3, min(next_start, forward["end"]+3))
        states = dict(before=before, after_rotation=middle, after_forward=after)
        reasons = [f"{name}_not_stable_or_visible" for name, value in states.items() if value is None]
        if not reasons and len({v["track_id"] for v in states.values()}) != 1:
            reasons.append("target_changed")
        if any(r["type"] == "fork_command" and rotation["start"] <= r["monotonic"] <= (after["monotonic"] if after else next_start) for r in controls):
            reasons.append("fork_operation_during_sample")
        row = dict(schema_version=1, session=session, sample_id=len(result)+1,
                   states=states, rotation=rotation, forward=forward,
                   valid_visual_sample=not reasons, rejection_reasons=reasons,
                   metric_pose_available=False, delta_yaw_deg=None, delta_depth_cm=None)
        for action in (rotation, forward):
            action["commanded_duration_sec"] = sum(s["end"]-s["start"] for s in action["segments"])
        if not reasons:
            row["top_line_delta"] = {
                name: {key: states[name][key]-before[key] for key in
                       ("yaw_image_deg", "depth_top_y_px", "depth_top_y_ratio")
                       if key in before and key in states[name]}
                for name in ("after_rotation", "after_forward")
            }
            row["visual_delta"] = {
                name: {key: states[name][key]-before[key] for key in
                       ("center_x_px", "center_y_ratio", "border_angle_deg")}
                for name in ("after_rotation", "after_forward")
            }
        result.append(row)
    return result


def save_samples(record_path):
    path = Path(record_path)
    rows = extract_samples(list(read_rows(path.with_suffix(".controls.jsonl"))),
                           list(read_rows(path.with_suffix(".slot_observations.jsonl"))), path.stem)
    target = path.with_suffix(".motion_samples.jsonl")
    with target.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    path.with_suffix(".motion_schema.json").write_text(json.dumps(SCHEMA, ensure_ascii=False, indent=2)+"\n")
    return len(rows), sum(r["valid_visual_sample"] for r in rows)
