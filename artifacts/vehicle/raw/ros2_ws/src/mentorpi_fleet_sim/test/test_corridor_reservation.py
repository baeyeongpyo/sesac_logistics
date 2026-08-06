from mentorpi_fleet_sim.corridor_reservation import CorridorReservation


def test_second_robot_waits_until_first_releases():
    table = CorridorReservation(ttl_seconds=5.0)
    assert table.acquire('corridor_a', 'robot_1', now=10.0)
    assert not table.acquire('corridor_a', 'robot_2', now=11.0)
    assert table.release('corridor_a', 'robot_1')
    assert table.acquire('corridor_a', 'robot_2', now=12.0)


def test_expired_lease_can_be_reassigned():
    table = CorridorReservation(ttl_seconds=5.0)
    assert table.acquire('corridor_a', 'robot_1', now=10.0)
    assert table.acquire('corridor_a', 'robot_2', now=15.1)


def test_wrong_robot_cannot_release_lease():
    table = CorridorReservation(ttl_seconds=5.0)
    table.acquire('corridor_a', 'robot_1', now=10.0)
    assert not table.release('corridor_a', 'robot_2')


def test_holder_expires_lease():
    table = CorridorReservation(ttl_seconds=5.0)
    table.acquire('corridor_a', 'robot_1', now=10.0)
    assert table.holder('corridor_a', now=15.1) is None
