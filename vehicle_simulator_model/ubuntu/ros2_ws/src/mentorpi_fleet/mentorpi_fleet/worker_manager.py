"""Per-vehicle Domain Bridge process lifecycle management."""

from dataclasses import dataclass
import os
from pathlib import Path
import subprocess

from .bridge_config import write_bridge_config
from .registry import VehicleSpec


@dataclass
class BridgeWorker:
    spec: VehicleSpec
    config_path: Path
    process: subprocess.Popen


class BridgeWorkerManager:
    """Own one Domain Bridge process per vehicle without fleet-wide restarts."""

    def __init__(self, control_domain: int, runtime_dir: Path):
        self._control_domain = control_domain
        self._runtime_dir = runtime_dir
        self._workers: dict[str, BridgeWorker] = {}

    @property
    def worker_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._workers))

    def reconcile(self, vehicles: list[VehicleSpec]) -> None:
        desired = {vehicle.vehicle_id: vehicle for vehicle in vehicles if vehicle.enabled}
        for vehicle_id in tuple(self._workers):
            if vehicle_id not in desired:
                self._stop(vehicle_id)
        for vehicle_id, spec in desired.items():
            current = self._workers.get(vehicle_id)
            if current is not None and current.spec == spec and current.process.poll() is None:
                continue
            if current is not None:
                self._stop(vehicle_id)
            self._start(spec)

    def stop_all(self) -> None:
        for vehicle_id in tuple(self._workers):
            self._stop(vehicle_id)

    def _start(self, spec: VehicleSpec) -> None:
        config_path = self._runtime_dir / spec.vehicle_id / 'domain_bridge.yaml'
        write_bridge_config(spec, self._control_domain, config_path)
        environment = os.environ.copy()
        process = subprocess.Popen(
            ['ros2', 'run', 'domain_bridge', 'domain_bridge', str(config_path)],
            env=environment,
        )
        self._workers[spec.vehicle_id] = BridgeWorker(spec, config_path, process)

    def _stop(self, vehicle_id: str) -> None:
        worker = self._workers.pop(vehicle_id)
        if worker.process.poll() is not None:
            return
        worker.process.terminate()
        try:
            worker.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            worker.process.kill()
            worker.process.wait(timeout=5)
