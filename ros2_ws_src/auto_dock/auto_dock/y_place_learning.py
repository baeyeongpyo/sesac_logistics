"""Persistent, held-out yaw-response learning. No ROS, commands, or node creation.

Receipt-time IMU/cmd alignment is approximate; validation is replay evidence,
not a guarantee of physical alignment. Reverse/lateral response is not fitted.
"""
from dataclasses import asdict, replace
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np
from scipy.optimize import lsq_linear

from .y_place_square_motion import base_coefficients

KEYS = ('left_immediate_gain', 'left_total_gain',
        'right_immediate_gain', 'right_total_gain')
MIN_RUNS = 8
WINDOW = 40


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def signature(config, args, vehicle):
    payload = dict(schema=2, vehicle=vehicle, load='loaded', config=config,
                   speed=args.speed, angular=args.angular, stage=args.stage_cm,
                   insertion=args.insert_cm, offset=args.center_offset_left_cm)
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def design(sample, coefficients):
    """Linear yaw-response basis, replaying the same <=40ms discrete model.

    Reject unmodelled motion, missing moving telemetry, clock breaks, and weak
    excitation. Keep command edges; do not interpolate across moving IMU gaps.
    """
    commands = np.asarray(sample['commands'], float)
    imu = np.asarray(sample['imu'], float)
    if (commands.ndim != 2 or commands.shape[1] != 4 or len(commands) < 3
            or imu.ndim != 2 or imu.shape[1] != 2 or len(imu) < 4
            or not np.isfinite(commands).all() or not np.isfinite(imu).all()):
        raise ValueError('invalid_telemetry')
    if np.any(np.diff(commands[:, 0]) < 0) or np.any(np.diff(imu[:, 0]) <= 0):
        raise ValueError('nonmonotonic_telemetry')
    if np.any(commands[:, 1] < 0) or np.any(commands[:, 2] != 0):
        raise ValueError('reverse_or_lateral_unmodelled')
    # Current profile permits pure .10m/s forward and pure +/- .35rad/s turns.
    if np.any((commands[:, 1] != 0) & (commands[:, 3] != 0)):
        raise ValueError('mixed_motion_unmodelled')
    if np.any((commands[:, 1] != 0) & (np.abs(commands[:, 1] - .1) > 1e-6)) or np.any(
            (commands[:, 3] != 0) & (np.abs(np.abs(commands[:, 3]) - .35) > 1e-6)):
        raise ValueError('command_profile_mismatch')
    start, end = commands[0, 0], commands[-1, 0]
    if end - start > 180 or imu[0, 0] > start or imu[-1, 0] < end - .1:
        raise ValueError('incomplete_imu_coverage')
    end = min(end, imu[-1, 0])
    active = np.any(commands[:-1, 1:] != 0, axis=1)
    if np.any(np.diff(commands[:, 0])[active] > .35):
        raise ValueError('moving_command_gap')
    for lo, hi in zip(imu[:-1, 0], imu[1:, 0]):
        if hi-lo > .35 and np.any(active & (commands[:-1, 0] < hi) & (commands[1:, 0] > lo)):
            raise ValueError('moving_imu_gap')
    dt = np.diff(commands[:, 0])
    exposure = [float(np.sum(dt * np.maximum(sign*commands[:-1, 3], 0))) for sign in (1, -1)]
    if max(exposure) < .04 or np.sum(dt*commands[:-1, 1])*100 < 2:
        raise ValueError('insufficient_directional_excitation')
    edges = commands[np.r_[True, np.any(np.diff(commands[:, 1:], axis=0) != 0, axis=1)], 0]
    times = np.unique(np.r_[start, np.arange(start, end, .04), edges[(edges > start) & (edges < end)], end])
    yaw = np.degrees(np.interp(times, imu[:, 0], np.unwrap(imu[:, 1])))
    yaw -= yaw[0]
    state = np.zeros((3, 5))  # four gains plus fixed incoming latent state
    initial = sample.get('initial_state', {})
    state[1, 4] = initial.get('equilibrium_deg', 0.)
    state[2, 4] = initial.get('pending_deg', 0.)
    if not np.isfinite(state).all():
        raise ValueError('invalid_initial_state')
    rows = [state[0].copy()]
    for lo, hi in zip(times[:-1], times[1:]):
        row = commands[np.searchsorted(commands[:, 0], lo, side='right')-1]
        duration = hi-lo
        du = math.degrees(row[3]*duration)
        immediate, total = np.zeros(5), np.zeros(5)
        side = 0 if du >= 0 else 2
        immediate[side], total[side+1] = du, du
        state[2] += total-immediate
        release = state[2]*(-math.expm1(-row[1]*100*duration/coefficients.release_command_cm))
        state[1] += immediate+release
        state[0] += (state[1]-state[0])*(-math.expm1(-duration/coefficients.settling_sec))
        state[2] -= release
        rows.append(state[0].copy())
    matrix = np.asarray(rows)
    basis, fixed = matrix[:, :4], matrix[:, 4]
    if 'estimated_gains' in sample:
        basis = basis*np.asarray(sample['estimated_gains'], float)
    return basis, yaw-fixed


def vector(coefficients):
    return np.array([getattr(coefficients, key) for key in KEYS], float)


def metrics(data, gains):
    errors = [matrix @ gains-yaw for matrix, yaw in data]
    return dict(rmse_deg=float(np.sqrt(np.mean([np.mean(e*e) for e in errors]))),
                endpoint_mae_deg=float(np.mean([abs(e[-1]) for e in errors])),
                per_run_rmse_deg=[float(np.sqrt(np.mean(e*e))) for e in errors])


def validate_candidate(training, heldout, incumbent=None):
    """Split by complete runs BEFORE fitting; bound every candidate update."""
    b = np.ones(4)
    old = np.ones(4) if incumbent is None else np.asarray(incumbent, float)
    lo, hi = np.maximum(b*.6, old*.9), np.minimum(b*1.4, old*1.1)
    matrix = np.vstack([a/math.sqrt(len(y)) for a, y in training])
    target = np.concatenate([y/math.sqrt(len(y)) for a, y in training])
    active = np.linalg.norm(matrix, axis=0) > 1e-6
    if not np.any(active) or np.linalg.matrix_rank(matrix[:, active]) < sum(active) or np.linalg.cond(matrix[:, active]) > 1000:
        raise ValueError('unidentifiable_directional_correction')
    result = lsq_linear(matrix[:, active], target-matrix[:, ~active]@old[~active], bounds=(lo[active], hi[active]))
    candidate = old.copy()
    candidate[active] = result.x
    baseline, previous, proposed = [metrics(heldout, v) for v in (b, old, candidate)]
    accepted = bool(result.success and np.isfinite(candidate).all()
                    and proposed['rmse_deg'] <= .9*min(baseline['rmse_deg'], previous['rmse_deg'])
                    and min(baseline['rmse_deg'], previous['rmse_deg'])-proposed['rmse_deg'] >= .2
                    and proposed['endpoint_mae_deg'] <= min(baseline['endpoint_mae_deg'], previous['endpoint_mae_deg'])
                    and all(n <= min(b, o)+.1 for n, b, o in zip(proposed['per_run_rmse_deg'], baseline['per_run_rmse_deg'], previous['per_run_rmse_deg'])))
    return candidate, dict(accepted=accepted, baseline=baseline, incumbent=previous,
                          candidate=proposed, validation_kind='held_out_command_imu_replay')


def condition_key(observed, base, plan, state, phase):
    """Condition on this run's estimated gains and proposed command regime.

    These are empirical response classes, not inferred physical mass labels.
    Both side gains, turn order/short pulses, forward exposure and pending yaw
    distinguish data. Model inputs always include the current measured gains.
    """
    ratio = vector(observed)/vector(base)
    if not np.isfinite(ratio).all() or np.any(ratio < .2) or np.any(ratio > 5.):
        raise ValueError('gain_input_out_of_range')
    bins = np.rint(np.log(ratio)/math.log(1.25)).astype(int).tolist()
    turns = [(1 if a['drive'][2] > 0 else -1, a['duration_sec'] < .5)
             for a in plan['actions'] if a['drive'][2] != 0]
    travel = sum(a['drive'][0]*100*a['duration_sec'] for a in plan['actions'])
    payload = dict(phase=phase, gain_bins=bins, turns=turns,
                   forward_10cm_bin=int(round(travel/10)),
                   pending_5deg_bin=int(round(state.pending_deg/5)))
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:20], payload


class LearningSession:
    def __init__(self, output, config, args):
        self.output = Path(output)
        self.base = base_coefficients(config)
        self.key = signature(config, args, self.output.parent.name)
        self.root = self.output.parent / 'y_place_learning' / self.key
        self.pending = None
        self.samples = []
        atomic_json(self.output / 'learning_selection.json', dict(
            status='current_run_gain_estimation_unchanged', signature=self.key,
            correction_scope='estimated_gain_and_command_to_actual_loaded_yaw'))

    def _state(self, directory, condition):
        path = directory / 'model.json'
        if not path.exists():
            return np.ones(4), 'uncorrected_current_run_gains'
        saved = json.loads(path.read_text())
        values = np.asarray(saved['factors'], float)
        if (saved['signature'] != self.key or saved['condition'] != condition
                or not saved['validation']['accepted'] or values.shape != (4,)
                or not np.isfinite(values).all() or np.any(values < .6) or np.any(values > 1.4)):
            raise ValueError('invalid_saved_correction')
        return values, saved['model_id']

    def prepare(self, node, coefficients, state, plan, phase, *, apply_correction=True):
        """Keep the online estimate; apply only a validated conditional residual.

        Snapshot the estimate BEFORE seeing this segment's actual IMU response.
        Later fitting learns actual yaw from this estimate and actual commands.
        """
        self.pending = None
        condition, inputs = condition_key(coefficients, self.base, plan, state, phase)
        factors, model_id = np.ones(4), 'uncorrected_current_run_gains'
        selection_error = None
        try:
            if apply_correction:
                factors, model_id = self._state(self.root / condition, condition)
            corrected = replace(coefficients, **dict(zip(KEYS, vector(coefficients)*factors)))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            corrected = coefficients
            factors, model_id = np.ones(4), 'uncorrected_current_run_gains'
            selection_error = str(exc)
        now = time.monotonic()
        local_state = dict(yaw_deg=0., equilibrium_deg=state.equilibrium_deg-state.yaw_deg,
                           pending_deg=state.pending_deg)
        self.pending = dict(schema=2, signature=self.key, condition=condition,
                            condition_inputs=inputs, phase=phase, load='loaded',
                            run_id=self.output.name, start_monotonic=now,
                            initial_state=local_state, estimated_gains=vector(coefficients).tolist(),
                            applied_factors=factors.tolist(), model_id=model_id,
                            selection_error=selection_error)
        atomic_json(self.output / ('learning_'+phase+'_prediction.json'), self.pending)
        return corrected

    def set_plan(self, plan, reference_heading, target_relative_heading):
        if self.pending is not None:
            self.pending.update(reference_imu_heading_deg=reference_heading,
                                predicted_final_imu_heading_deg=reference_heading+plan['predicted_stage'][2],
                                target_imu_heading_deg=reference_heading+target_relative_heading,
                                planned_actions=plan['actions'])
            atomic_json(self.output / ('learning_'+self.pending['phase']+'_prediction.json'), self.pending)

    def capture(self, node, final_heading, target_heading=None, predicted_heading=None):
        if self.pending is None:
            return
        sample, self.pending = self.pending, None
        start = sample['start_monotonic']
        target_heading = sample.get('target_imu_heading_deg') if target_heading is None else target_heading
        predicted_heading = sample.get('predicted_final_imu_heading_deg') if predicted_heading is None else predicted_heading
        # At prepare(), the preceding segment has stopped. The interval also
        # includes any zero-command solver wait before actual motion starts.
        commands = [[start, 0., 0., 0.]] + [list(c) for c in node.commands if c[0] > start]
        imu = [list(row) for row in node.imu]
        before = [i for i,row in enumerate(imu) if row[0] <= start]
        imu = imu[before[-1]:] if before else []
        sample.update(commands=commands, imu=imu, final_imu_heading_deg=final_heading,
                      target_imu_heading_deg=target_heading,
                      predicted_final_imu_heading_deg=predicted_heading,
                      prediction_error_deg=None if predicted_heading is None else (final_heading-predicted_heading+180)%360-180,
                      target_error_deg=None if target_heading is None else (final_heading-target_heading+180)%360-180,
                      timestamp_assumption='IMU and command callback receipt monotonic; transport/queue delay unmeasured',
                      alignment_evidence='camera target plus stopped IMU; final visual alignment unverified')
        self.samples.append(sample)
        atomic_json(self.output / ('learning_'+sample['phase']+'_actual.json'), sample)

    def finish(self, error):
        """Run only after final stop handling, never update a moving drive."""
        reports = []
        for sample in self.samples:
            report = dict(phase=sample['phase'], status='excluded', reason=error)
            if not error:
                try:
                    report = self._accumulate(sample)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    report['reason'] = str(exc)
            reports.append(report)
        atomic_json(self.output / 'learning_report.json', dict(
            status='excluded' if error else 'recorded', error=error, segments=reports))

    def _accumulate(self, sample):
        data = design(sample, self.base)
        condition = sample['condition']
        directory = self.root / condition
        directory.mkdir(parents=True, exist_ok=True)
        with (directory / 'update.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            atomic_json(directory / (self.output.name + '.json'), sample)
            report = dict(phase=sample['phase'], status='collecting',
                          uncorrected_prediction=metrics([data], np.ones(4)),
                          applied_prediction=metrics([data], np.asarray(sample['applied_factors'])),
                          target_error_deg=sample['target_error_deg'])
            samples, ids, gains = [], [], []
            for path in sorted(directory.glob('auto_dock_test_y_*.json'))[-WINDOW:]:
                try:
                    row = json.loads(path.read_text())
                    if row['signature'] != self.key or row['condition'] != condition:
                        continue
                    samples.append(design(row, self.base))
                    ids.append(row['run_id'])
                    gains.append(row['estimated_gains'])
                except (OSError, ValueError, KeyError, TypeError):
                    continue
            report['eligible_runs'] = len(samples)
            if len(samples) >= MIN_RUNS:
                incumbent, _ = self._state(directory, condition)
                assessment_path = directory / 'assessment.json'
                assessment = json.loads(assessment_path.read_text()) if assessment_path.exists() else {}
                if set(ids[-2:]).isdisjoint(assessment.get('validation_runs', [])):
                    candidate, validation = validate_candidate(samples[:-2], samples[-2:], incumbent)
                    # Ratios need not be ordered, but the resulting physical
                    # coefficients must be valid for every recorded estimate.
                    for values in gains:
                        adjusted = np.asarray(values)*candidate
                        if adjusted[1] < adjusted[0] or adjusted[3] < adjusted[2]:
                            validation['accepted'] = False
                    report.update(status='promoted' if validation['accepted'] else 'rejected', validation=validation,
                                  training_runs=ids[:-2], validation_runs=ids[-2:])
                    atomic_json(assessment_path, report)
                    if validation['accepted']:
                        saved = dict(signature=self.key, condition=condition,
                                     model_id='gain-residual-' + self.output.name,
                                     factors=candidate.tolist(), validation=validation,
                                     training_runs=ids[:-2], validation_runs=ids[-2:])
                        atomic_json(directory / ('model_version_'+self.output.name+'.json'), saved)
                        atomic_json(directory / 'model.json', saved)
                else:
                    report['status'] = 'waiting_for_two_new_validation_runs'
            return report
