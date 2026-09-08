from types import SimpleNamespace as NS

import pytest

from auto_dock import dock_front_distance_hold as hold


@pytest.fixture
def host(monkeypatch):
    monkeypatch.setattr(hold.time, 'monotonic', lambda: 10.)
    return NS(mission_kind='DOCK_PICK', config={}, latest_detection_at=10.,
              latest_detection={'detections': []})


def observe(host, distances):
    host.latest_detection['detections'] = [dict(
        **{'class': 'clover'}, depth=dict(forward_distance_cm=d, bearing_deg=0.))
        for d in distances]


@pytest.mark.parametrize('direction', [-1., 1.])
def test_backward_drift_is_corrected_then_lateral_resumes(host, direction):
    observe(host, [29., 30., 31.])
    assert hold.adjust(host, .02, direction*.12, 0.) == (0., direction*.12, 0.)
    observe(host, [32., 33., 80.])
    assert hold.adjust(host, .02, direction*.12, 0.) == (.10, 0., 0.)
    observe(host, [31.5])
    assert hold.adjust(host, .02, direction*.12, 0.) == (.10, 0., 0.)
    observe(host, [30.5])
    assert hold.adjust(host, .02, direction*.12, 0.) == (0., direction*.12, 0.)


def test_too_close_reverses_without_mixing_wheel_components(host):
    observe(host, [30.])
    hold.adjust(host, 0., .1, 0.)
    observe(host, [27.])
    assert hold.adjust(host, 0., .1, 0.) == (-.1, 0., 0.)
    assert hold.adjust(host, 0., 0., 0.) == (0., 0., 0.)
    assert host.dock_front_distance_hold['reference_cm'] == 30.


@pytest.mark.parametrize('command', [(.1, 0., 0.), (-.1, 0., 0.), (0., 0., .35)])
def test_deliberate_longitudinal_motion_or_turn_resets_reference(host, command):
    observe(host, [30.])
    hold.adjust(host, 0., .12, 0.)
    assert hold.adjust(host, *command) == command
    assert host.dock_front_distance_hold is None
    observe(host, [20.])
    hold.adjust(host, 0., .12, 0.)
    assert host.dock_front_distance_hold['reference_cm'] == 20.


@pytest.mark.parametrize('mode', ['stale', 'missing', 'side_only', 'disabled', 'y_place'])
def test_unavailable_data_or_disabled_feature_preserves_original_command(host, mode):
    observe(host, [30.])
    if mode == 'stale': host.latest_detection_at = 8.
    if mode == 'missing': host.latest_detection = {}
    if mode == 'side_only': host.latest_detection['detections'][0]['depth']['bearing_deg'] = 60.
    if mode == 'disabled': host.config['dock_lateral_front_hold_enabled'] = False
    if mode == 'y_place': host.mission_kind = 'Y_PLACE'
    assert hold.adjust(host, .02, .12, 0.) == (.02, .12, 0.)
