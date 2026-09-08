import math

from auto_dock.loaded_response_planner import ResponseCoefficients
from y_slot_topline_insert_trial import (
    measured_directional_gains,
    response_state_from_history,
)


def test_measured_directional_gains_separates_left_and_right():
    commands = [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.5),
        (2.0, 0.0, 0.0, 0.0),
        (3.0, 0.1, 0.0, 0.0),
        (4.0, 0.0, 0.0, -0.5),
        (5.0, 0.0, 0.0, 0.0),
        (6.0, 0.1, 0.0, 0.0),
        (7.0, 0.0, 0.0, 0.0),
    ]
    # Left total gain 1.2; right total gain 1.6.
    yaw_deg = [0.0, 0.0, 12.0, 24.0, 34.38, 18.38, -11.62, -11.62]
    yaws = [(float(index), math.radians(value)) for index, value in enumerate(yaw_deg)]

    coefficients, evidence = measured_directional_gains(
        commands, yaws, 7.0, ResponseCoefficients()
    )

    assert evidence["left_samples"] == 1
    assert evidence["right_samples"] == 1
    assert coefficients.left_total_gain > 0.0
    assert coefficients.right_total_gain > coefficients.left_total_gain


def test_response_state_starts_after_last_reverse():
    commands = [
        (0.0, -0.1, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (2.0, 0.1, 0.0, 0.0),
        (3.0, 0.0, 0.0, 0.0),
    ]
    yaws = [(float(index), 0.0) for index in range(4)]

    state = response_state_from_history(
        commands, yaws, 3.0, ResponseCoefficients()
    )

    assert state.history_known
