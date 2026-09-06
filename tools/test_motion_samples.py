from motion_samples import extract_samples


def fixture():
    controls = [{"type": "recording_start", "monotonic": 0.0}]
    for start, end, x, yaw in [(1, 2, 0, .35), (2, 4, 0, 0), (4, 5, .1, 0), (5, 7, 0, 0)]:
        for n in range(round((end-start)*10)):
            controls.append(dict(type="cmd_vel", monotonic=start+n*.1,
                                 linear_x=x, linear_y=0, angular_z=yaw))
    controls.append(dict(type="recording_stop", monotonic=7))
    observations = []
    for times, x, angle in [([.4, .6, .9], 320, 2), ([2.4, 2.6, 2.9], 330, 0),
                            ([5.4, 5.6, 5.9], 340, 1)]:
        for t in times:
            observations.append(dict(monotonic=t, source_age_sec=.05, track_id="one",
                                     valid=True, center_x_px=x, center_y_ratio=.6,
                                     border_angle_deg=angle, yaw_deg=None, depth_cm=None))
    return controls, observations


def test_rotation_forward_sample_retains_intermediate_pose_and_actual_commands():
    controls, observations = fixture()
    rows = extract_samples(controls, observations, "test")
    assert len(rows) == 1
    row = rows[0]
    assert row["valid_visual_sample"]
    assert abs(row["forward"]["commanded_duration_sec"]-1) < .001
    assert row["visual_delta"]["after_rotation"]["center_x_px"] == 10
    assert row["visual_delta"]["after_forward"]["center_x_px"] == 20
    assert row["delta_yaw_deg"] is None and row["delta_depth_cm"] is None


def test_missing_intermediate_view_is_rejected():
    controls, observations = fixture()
    rows = extract_samples(controls, [r for r in observations if not 2 < r["monotonic"] < 3], "test")
    assert not rows[0]["valid_visual_sample"]
    assert "after_rotation_not_stable_or_visible" in rows[0]["rejection_reasons"]


def test_target_switch_and_stale_frames_not_valid_samples():
    controls, observations = fixture()
    for row in observations[-3:]:
        row["track_id"] = "another"
    assert "target_changed" in extract_samples(controls, observations, "test")[0]["rejection_reasons"]
    for row in observations:
        row["source_age_sec"] = .8
    assert not extract_samples(controls, observations, "test")[0]["valid_visual_sample"]


def test_top_endpoints_give_image_angle_and_depth_proxy():
    from motion_samples import top_border_measurement
    result = top_border_measurement({
        'center_y_ratio': .8,
        'border_lines': [[300, 220, 100, 200], [100, 430, 300, 430]],
    }, 480)
    assert result['top_left_px'] == [100., 200.]
    assert result['top_right_px'] == [300., 220.]
    assert 5.7 < result['yaw_image_deg'] < 5.8
    assert result['depth_top_y_px'] == 210
    assert result['depth_top_y_ratio'] == 210/480
