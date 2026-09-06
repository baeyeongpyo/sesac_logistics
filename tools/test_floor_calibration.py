import json
from pathlib import Path

import numpy as np
import pytest
from floor_calibration import fit_floor_calibration, floor_point, ground_corners, project, save_floor_profile


def samples():
    # Synthetic pinhole ground projection: known perspective and fork-centre offset.
    floor_to_image = np.array([[5., 6.4, 320.], [0., -.8, 450.], [0., .02, 1.]])
    return [dict(far_line_cm=d, image_size=[640, 480],
                 top_line_px=project(floor_to_image, ground_corners(20, 15, d)[:2]).tolist()) for d in (30, 50)]


def fit(s=None):
    return fit_floor_calibration(samples() if s is None else s, 20., vehicle=1, source_topic='/camera/raw')


def test_recover_unseen_ground_point_and_bearing():
    profile = fit()
    matrix = np.array([[5., 6.4, 320.], [0., -.8, 450.], [0., .02, 1.]])
    pixel = project(matrix, [[4., 32.]])[0]
    result = floor_point(profile, pixel, [640, 480], vehicle=1, source_topic='/camera/raw')
    assert result['right_cm'] == pytest.approx(4., abs=.001)
    assert result['forward_cm'] == pytest.approx(32., abs=.001)
    assert result['bearing_left_deg'] == pytest.approx(-7.125, abs=.001)


def test_reject_repeated_image_at_different_distances():
    points = samples()
    points[1]['top_line_px'] = points[0]['top_line_px']
    with pytest.raises(ValueError, match='화면 위'):
        fit(points)


def test_reject_degenerate_and_wrong_resolution():
    points = samples()
    points[0]['top_line_px'] = [[100,100]]*2
    with pytest.raises(ValueError): fit(points)
    profile = fit()
    with pytest.raises(ValueError, match='해상도'):
        floor_point(profile, [320,200], [1280,960], vehicle=1, source_topic='/camera/raw')
    with pytest.raises(ValueError, match='영역 밖'):
        floor_point(profile, [1,1], [640,480], vehicle=1, source_topic='/camera/raw')


def test_preserve_existing_pallet_and_motion_calibration(tmp_path):
    path = tmp_path/'pose.json'
    original = {'camera_distance_scale_cm_per_pnp_unit':6.3,
                'camera_distance_offset_cm':-17.3,
                'calibration_presets':{'unloaded':{'distance_coefficient':.89}},
                'floor_calibration_presets':{'unloaded':{'preserved':True}},
                'active_calibration_preset':'unloaded', 'other':42}
    path.write_text(json.dumps(original))
    save_floor_profile(path, fit())
    after = json.loads(path.read_text())
    for key in original:
        if key != 'floor_calibration_presets': assert after[key] == original[key]
    assert after['floor_calibration_presets']['unloaded'] == {'preserved':True}
    assert after['floor_calibration_presets']['loaded']['load_state'] == 'loaded'
    assert len(list(tmp_path.glob('*.before_floor_calibration_*'))) == 1


def test_save_profile_atomically_updates_fitted_depth_extrinsics(tmp_path):
    path = tmp_path/'pose.json'
    path.write_text(json.dumps({
        'camera_pitch_deg': 1.,
        'depth_camera_to_fork_tip_offset_cm': 14.,
        'unrelated': 7,
    }))
    profile = fit()
    save_floor_profile(path, profile, config_updates={
        'y_slot_depth_camera_pitch_deg': -6.5,
        'y_slot_depth_camera_to_fork_tip_offset_cm': 29.2,
        'y_slot_depth_extrinsic_calibration': {'residual_rmse_cm': .2},
    })
    saved = json.loads(path.read_text())
    assert saved['camera_pitch_deg'] == 1.
    assert saved['depth_camera_to_fork_tip_offset_cm'] == 14.
    assert saved['y_slot_depth_camera_pitch_deg'] == -6.5
    assert saved['y_slot_depth_camera_to_fork_tip_offset_cm'] == 29.2
    assert saved['y_slot_depth_extrinsic_calibration']['residual_rmse_cm'] == .2
    assert saved['unrelated'] == 7
    assert saved['floor_calibration_presets']['loaded'] == profile


def test_depth_only_save_preserves_dock_and_homography(tmp_path):
    path = tmp_path/'pose.json'
    original = {
        'camera_pitch_deg': 19.,
        'depth_camera_to_fork_tip_offset_cm': 14.,
        'floor_calibration_presets': {'loaded': {'existing': 'homography'}},
    }
    path.write_text(json.dumps(original))
    save_floor_profile(path, None, config_updates={
        'y_slot_depth_camera_pitch_deg': -7.,
        'y_slot_depth_camera_to_fork_tip_offset_cm': 30.,
        'y_slot_depth_extrinsic_calibration': {'residual_rmse_cm': .2},
    })
    saved = json.loads(path.read_text())
    assert saved['camera_pitch_deg'] == 19.
    assert saved['depth_camera_to_fork_tip_offset_cm'] == 14.
    assert saved['floor_calibration_presets'] == original['floor_calibration_presets']
    assert saved['y_slot_depth_camera_pitch_deg'] == -7.
    assert saved['y_slot_depth_camera_to_fork_tip_offset_cm'] == 30.


@pytest.mark.parametrize('global_key', [
    'camera_pitch_deg',
    'depth_camera_to_fork_tip_offset_cm',
])
def test_floor_calibration_rejects_global_dock_extrinsic_updates(tmp_path, global_key):
    path = tmp_path/'pose.json'
    path.write_text(json.dumps({global_key: 14.0}))
    with pytest.raises(ValueError, match='unsupported floor calibration config key'):
        save_floor_profile(path, fit(), config_updates={global_key: 30.0})


def test_arbitrary_distances_and_reverse_capture_order():
    matrix = np.array([[5., 6.4, 320.], [0., -.8, 450.], [0., .02, 1.]])
    entries = [dict(far_line_cm=d, image_size=[640,480],
                    top_line_px=project(matrix, [[-10,d],[10,d]]).tolist()) for d in (83.2, 42.7)]
    profile = fit(entries)
    assert profile['sample_distances_cm'] == [42.7,83.2]
    result = floor_point(profile, project(matrix, [[2,60]])[0], [640,480], vehicle=1, source_topic='/camera/raw')
    assert result['forward_cm'] == pytest.approx(60, abs=.001)
    assert result['right_cm'] == pytest.approx(2, abs=.001)


def test_equal_distances_rejected():
    entries = samples()
    entries[1]['far_line_cm'] = entries[0]['far_line_cm']
    with pytest.raises(ValueError, match='서로 다른'):
        fit(entries)
