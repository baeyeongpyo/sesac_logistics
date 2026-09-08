"""Final IMU heading correction using current-run response; no ROS imports."""
import math

from .y_place_midpoint import heading_error


def correct_heading(target, actual, coefficients, angular, execute, wait_stopped,
                    report, stop, tolerance_deg=1.5):
    """Correct measured residuals, including large ones, before lowering.

    Reobserve after each pulse and update its directional response. There is
    no error-magnitude rejection or fixed retry count. Cancellation and fresh
    stopped IMU checks remain owned by the caller's existing mechanisms.
    The 0.5s pulse horizon is a feedback sampling interval, not a failure gate.
    """
    gains = {side: coefficients.yaw_gains(sign)[0]
             for side, sign in (("left", 1), ("right", -1))}
    initial_error = heading_error(target, actual)
    steps = []
    while abs(heading_error(target, actual)) > tolerance_deg:
        stop.check()
        error = heading_error(target, actual)
        side = "left" if error > 0 else "right"
        gain = gains[side]
        if not math.isfinite(gain) or gain <= 0 or angular <= 0:
            raise ValueError('invalid_measured_heading_response')
        duration = min(.5, abs(math.radians(error)) / (angular * gain))
        action = dict(action='final_imu_heading_correction',
                      drive=[0., 0., math.copysign(angular, error)],
                      duration_sec=duration, target_turn_deg=error)
        before = actual
        executed = execute([action])
        actual = wait_stopped('final_heading_correction')
        elapsed = executed[0]['duration_sec']
        response = heading_error(actual, before)
        exposure = math.degrees(action['drive'][2] * elapsed)
        if exposure and response / exposure > 0:
            gains[side] = response / exposure
        step = dict(target_imu_heading_deg=target,
                    before_imu_heading_deg=before, actual_imu_heading_deg=actual,
                    remaining_error_deg=heading_error(target, actual),
                    measured_gain=gains[side], action=action,
                    actual_command_duration_sec=elapsed)
        steps.append(step)
        report(dict(phase='final_imu_heading_correction', **step))
    return actual, dict(initial_error_deg=initial_error,
                        final_error_deg=heading_error(target, actual),
                        target_imu_heading_deg=target, actual_imu_heading_deg=actual,
                        tolerance_deg=tolerance_deg, corrections=steps,
                        final_visual_position_verified=False)
