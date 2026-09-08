"""Offline selection of oblique targets; no ROS initialization."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from auto_dock.auto_dock_node import AutoDockNode


@pytest.mark.parametrize('yaw,expected', [(30.7, True), (-74.3, True), (89.9, True), (-89.9, True), (90., False), (-90., False), (100., False)])
@pytest.mark.parametrize('source', ['pnp', 'depth_yaw'])
def test_acquisition_accepts_oblique_front_without_weakening_side_boundary(yaw, expected, source):
    host = SimpleNamespace(target_entity_id=None)
    candidate = dict(entity_id=28, center_error=-.7,
                     peer_pnp_yaw_median_deg=0.)
    candidate[source] = dict(forward_distance_cm=24., yaw_deg=yaw)
    assert AutoDockNode.nearest_candidate_allowed(host, candidate, True) is expected


def test_recorded_left_diamond_is_selected_before_search_passes_it():
    frames = json.loads((Path(__file__).parent/'fixtures/oblique_diamond_20260907.json').read_text())
    # Old config values must no longer act as acquisition/peer rejection gates.
    config = dict(nearest_candidate_max_abs_yaw_deg=12.,
                  nearest_candidate_max_abs_pnp_yaw_deg=45.,
                  nearest_candidate_max_peer_yaw_delta_deg=20.)
    host = SimpleNamespace(product_type='NORMAL', number=lambda key, default, *limits: config.get(key, default))
    for frame in frames:
        candidate, _ = AutoDockNode.nearest_product_candidate(host, frame)
        assert candidate['entity_id'] == 28
        assert candidate['matrix'] == ['diamond'] * 4


@pytest.mark.parametrize('yaw', [30.7, -74.3])
def test_oblique_acquisition_does_not_allow_unaligned_insertion(yaw):
    import math
    commands = []
    host = SimpleNamespace(
        target_type='NEAREST', target_entity_id=28,
        target_in_body=lambda: (.20, 0., math.radians(yaw)),
        config={}, number=lambda key, default, *limits: default,
        insertion_start_due_at=None,
        publish_status=lambda *args, **kwargs: None,
        publish_drive=lambda *command: commands.append(command),
    )
    AutoDockNode.tick_docking(host)
    assert host.insertion_start_due_at is None
    assert commands[-1][:2] == (0., 0.)
    assert commands[-1][2] * yaw > 0.
