"""No ROS initialization: exercise controller methods on an in-memory host."""
from types import SimpleNamespace

import pytest

from auto_dock.auto_dock_node import AutoDockNode


@pytest.fixture
def host(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('auto_dock.auto_dock_node.time.monotonic', lambda: clock[0])
    candidate = {}
    events, drives = [], []
    node = SimpleNamespace(
        state='coarse_align', target_type='NEAREST', target_entity_id=3,
        target_world={'x': 1}, nearest_lock_signature={'entity_id': 3},
        nearest_center_reconfirm_pending=True, nearest_center_reconfirm_due_at=None,
        coarse_alignment_started_at=10., search_heading_yaw=None, odom_yaw=None,
        identity_measurement=lambda: (candidate, {}, None),
        valid_measurement=lambda: (None, None, 'invalid_pnp'),
        number=lambda key, default, *limits: default,
        stop_drive=lambda *args: drives.append((0., 0., 0.)),
        publish_drive=lambda *args: drives.append(args),
        publish_status=lambda state, reason, **extra: events.append(reason),
        latch_search_heading=lambda: None,
    )
    node.reset_coarse_alignment = lambda: AutoDockNode.reset_coarse_alignment(node)
    return node, candidate, clock, events, drives


@pytest.mark.parametrize('value', [None, 'bad', float('nan'), float('inf'), 1.1, -1.1])
def test_invalid_center_releases_lock_after_confirmation_and_allows_search(host, value):
    node, candidate, clock, events, drives = host
    candidate['center_error'] = value
    AutoDockNode.tick_coarse_align(node)
    assert node.state == 'coarse_align'
    assert node.target_entity_id == 3
    clock[0] = 10.81
    AutoDockNode.tick_coarse_align(node)
    assert node.state == 'search'
    assert node.target_world is None and node.target_entity_id is None
    assert node.nearest_lock_signature is None
    assert not node.nearest_center_reconfirm_pending
    assert node.candidate_retry_not_before > clock[0]
    assert events[-1] == 'coarse_center_invalid_resume_search'
    assert all(command == (0., 0., 0.) for command in drives)
    # The same bad visible candidate cannot immediately stop the resumed search.
    node.selected_candidate = lambda: (candidate, {})
    node.location = 'OTHER'
    node.config = {}
    # Drive the existing search path with tape guidance disabled and no heading.
    node.latest_detection = None
    node.search_heading_source = None
    node.search_heading_imu_yaw = None
    node.search_heading_yaw = None
    node.candidate_stop_due_at = None
    AutoDockNode.tick_search(node)
    assert node.candidate_stop_due_at is None
    assert drives[-1][1] != 0.


def test_valid_center_resets_timeout_before_later_dropout(host):
    node, candidate, clock, events, drives = host
    AutoDockNode.tick_coarse_align(node)
    clock[0] = 10.5
    candidate['center_error'] = .2
    AutoDockNode.tick_coarse_align(node)
    assert node.coarse_invalid_center_started_at is None
    assert events[-1] == 'coarse_centering'
    candidate.clear()
    clock[0] = 10.9
    AutoDockNode.tick_coarse_align(node)
    assert node.state == 'coarse_align'
    assert node.target_entity_id == 3
    assert node.coarse_invalid_center_started_at == 10.9


@pytest.mark.parametrize('initial_invalid', [False, True])
def test_recorded_visual_loss_cannot_wait_forever(host, initial_invalid):
    node, candidate, clock, events, drives = host
    if initial_invalid:
        AutoDockNode.tick_coarse_align(node)
        clock[0] = 10.3
    node.identity_measurement = lambda: (None, None, 'target_unavailable')
    node.tracked_partial_measurement = lambda: (None, None, 'partial_unavailable')
    node.visible_target_top_pair_measurement = lambda: (
        None, None, 'visible_target_top_pair_unavailable'
    )
    AutoDockNode.tick_coarse_align(node)
    assert events[-1] == 'nearest_center_recheck_waiting_visual'
    assert node.target_entity_id == 3
    clock[0] = 10.81
    AutoDockNode.tick_coarse_align(node)
    assert node.state == 'search'
    assert node.target_entity_id is None and node.target_world is None
    assert events[-1] == 'coarse_center_invalid_resume_search'
    assert all(command == (0., 0., 0.) for command in drives)
