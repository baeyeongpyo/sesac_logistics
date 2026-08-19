#!/usr/bin/env python3
"""Optional lifecycle manager for registry-backed Gazebo vehicles."""

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Callable

from mentorpi_fleet.registry import VehicleSpec, enabled_vehicles, load_registry
from mentorpi_fleet.worker_manager import BridgeWorkerManager


@dataclass
class SimulationRuntime:
    spec: VehicleSpec
    adapter: subprocess.Popen
    navigation: subprocess.Popen | None


class SimulationManager:
    """Create and remove only simulation vehicles while preserving shared Gazebo."""

    def __init__(
        self,
        control_domain: int,
        runtime_dir: Path,
        bridge_template: Path,
        worker_manager: BridgeWorkerManager | None = None,
        run_command: Callable[[list[str], dict[str, str]], object] | None = None,
        start_process: Callable[[list[str], dict[str, str], str], subprocess.Popen] | None = None,
        delete_scene: Callable[[str], None] | None = None,
    ):
        self._control_domain = control_domain
        self._runtime_dir = runtime_dir
        self._bridge_template = bridge_template
        self._workers = worker_manager or BridgeWorkerManager(control_domain, runtime_dir / 'workers')
        self._run_command = run_command or self._default_run_command
        self._start_process = start_process or self._default_start_process
        self._delete_scene = delete_scene or (lambda vehicle_id: None)
        self._runtimes: dict[str, SimulationRuntime] = {}

    @property
    def vehicle_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._runtimes))

    def reconcile(self, vehicles: list[VehicleSpec]) -> None:
        desired = {
            vehicle.vehicle_id: vehicle
            for vehicle in enabled_vehicles_like(vehicles, kind='simulation')
        }
        for vehicle_id in tuple(self._runtimes):
            if vehicle_id not in desired or self._runtimes[vehicle_id].spec != desired[vehicle_id]:
                # Remove the command bridge before adapter/Nav2 teardown so no new
                # central command can reach a vehicle being deleted.
                self._workers.reconcile(list(desired.values()))
                self._remove(vehicle_id)
        for vehicle_id, spec in desired.items():
            if vehicle_id not in self._runtimes:
                self._start(spec)
        self._workers.reconcile(list(desired.values()))

    def stop_all(self) -> None:
        self._workers.reconcile([])
        for vehicle_id in tuple(self._runtimes):
            self._remove(vehicle_id)
        self._workers.reconcile([])

    def _start(self, spec: VehicleSpec) -> None:
        assert spec.spawn is not None
        environment = self._vehicle_environment(spec)
        bridge_config = self._write_vehicle_bridge(spec)
        adapter = self._start_process([
            'ros2', 'launch', 'mentorpi_gz_sim', 'vehicle_adapter.launch.py',
            f'robot_id:={spec.vehicle_id}', f'x:={spec.spawn.x}', f'y:={spec.spawn.y}',
            f'z:={spec.spawn.z}', f'yaw:={spec.spawn.yaw}', f'bridge_config:={bridge_config}',
        ], environment, f'adapter:{spec.vehicle_id}')
        navigation = None
        if spec.nav_enabled:
            navigation = self._start_process([
                'ros2', 'launch', 'mentorpi_nav', 'vehicle_navigation.launch.py',
                f'robot_id:={spec.vehicle_id}',
            ], environment, f'nav:{spec.vehicle_id}')
        self._runtimes[spec.vehicle_id] = SimulationRuntime(spec, adapter, navigation)

    def _remove(self, vehicle_id: str) -> None:
        runtime = self._runtimes.pop(vehicle_id)
        # New central commands are no longer bridged after this reconcile cycle.
        self._stop_process(runtime.navigation)
        self._stop_process(runtime.adapter)
        self._run_command([
            'ros2', 'run', 'ros_gz_sim', 'delete', '-world', 'mentorpi_warehouse',
            '-name', vehicle_id,
        ], self._vehicle_environment(runtime.spec))
        self._delete_scene(vehicle_id)

    def _write_vehicle_bridge(self, spec: VehicleSpec) -> Path:
        target = self._runtime_dir / spec.vehicle_id / 'vehicle_bridge.yaml'
        target.parent.mkdir(parents=True, exist_ok=True)
        text = self._bridge_template.read_text().replace('__ROBOT_ID__', spec.vehicle_id)
        temporary = target.with_suffix('.tmp')
        temporary.write_text(text)
        temporary.replace(target)
        return target

    @staticmethod
    def _default_run_command(command: list[str], environment: dict[str, str]) -> None:
        subprocess.run(command, env=environment, check=True)

    @staticmethod
    def _default_start_process(command: list[str], environment: dict[str, str], label: str) -> subprocess.Popen:
        return subprocess.Popen(command, env=environment)

    @staticmethod
    def _stop_process(process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

    @staticmethod
    def _vehicle_environment(spec: VehicleSpec) -> dict[str, str]:
        environment = os.environ.copy()
        environment['ROS_DOMAIN_ID'] = str(spec.domain_id)
        return environment


def enabled_vehicles_like(vehicles: list[VehicleSpec], kind: str) -> list[VehicleSpec]:
    return [vehicle for vehicle in vehicles if vehicle.enabled and vehicle.kind == kind]


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--registry', required=True, type=Path)
    parser.add_argument('--runtime-dir', default=Path('/run/mentorpi-fleet/simulation'), type=Path)
    parser.add_argument(
        '--bridge-template',
        default=Path('/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/config/vehicle_bridge.yaml.in'),
        type=Path,
    )
    args = parser.parse_args(argv)
    manager = SimulationManager(225, args.runtime_dir, args.bridge_template)
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    last_mtime = None
    try:
        while not stopping:
            try:
                mtime = args.registry.stat().st_mtime_ns
                if mtime != last_mtime:
                    registry = load_registry(args.registry)
                    if registry.control_domain != 225:
                        raise ValueError('control_domain must be 225')
                    manager.reconcile(list(registry.vehicles))
                    last_mtime = mtime
            except Exception as error:
                print(f'simulation registry reload rejected; keeping current vehicles: {error}', flush=True)
            time.sleep(1.0)
    finally:
        manager.stop_all()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
