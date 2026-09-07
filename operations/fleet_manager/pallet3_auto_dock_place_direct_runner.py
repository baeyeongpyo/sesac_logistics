"""Direct Inventory operation creation for a Pallet 3 Auto Dock PLACE delivery."""

from __future__ import annotations

import signal
import sys
from collections.abc import Sequence

try:
    from .pallet3_direct_runner import (
        DirectPallet3Runner,
        FleetFailure,
        RunnerConfig,
        RunnerError,
        parse_args,
    )
except ImportError:
    from pallet3_direct_runner import (
        DirectPallet3Runner,
        FleetFailure,
        RunnerConfig,
        RunnerError,
        parse_args,
    )


class AutoDockPlacePallet3Runner(DirectPallet3Runner):
    """Run a Pallet 3 delivery that delegates unloading to Auto Dock PLACE."""

    def send_outbound_route(self, operation_id: str) -> None:
        self._post_fleet("/commands/navigation/waypoints", {
            "operation_id": operation_id,
            "purpose": "PLACE",
            "waypoints": [
                {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
                {"frame_id": "map", "x": -0.440, "y": -1.690, "yaw": -1.5707963267948966},
            ],
        })

    def send_auto_dock_place(self, operation_id: str) -> None:
        self._post_fleet("/commands/auto-dock", {
            "operation_id": operation_id,
            "operation": "PLACE",
            "product_type": "FRESH",
            "location": "Y",
            "target": {"type": "NEAREST"},
        })

    def wait_for_fleet_report(
        self,
        operation_id: str,
        expected_source: str,
        expected_detail: str,
        expected_inventory_status: str,
    ) -> None:
        while True:
            self._require_active_operation(operation_id, expected_inventory_status)
            vehicle = self._require_object(
                self._request(
                    "GET",
                    self._fleet(f"/api/v1/vehicles/{self.config.robot_id}"),
                    None,
                ).body,
                "Fleet vehicle snapshot",
            )
            if (
                vehicle.get("operation_id") == operation_id
                and vehicle.get("state") == "FAIL"
            ):
                raise FleetFailure(
                    f"vehicle {self.config.robot_id} reported FAIL for operation "
                    f"{operation_id}"
                )
            if (
                vehicle.get("operation_id") == operation_id
                and vehicle.get("source") == expected_source
                and vehicle.get("detail") == expected_detail
            ):
                return
            self._sleep(0.5)

    def run(self) -> None:
        operation_id = self.create_operation()
        self.send_auto_dock_pick(operation_id)
        self.wait_for_report(operation_id, "AUTO_DOCK_PICK_COMPLETED", "TO_PICK")
        self.complete_pick(operation_id)
        self.send_outbound_route(operation_id)
        self.wait_for_report(operation_id, "NAVIGATION_SUCCEEDED", "TO_PLACE")
        self.send_auto_dock_place(operation_id)
        self.wait_for_fleet_report(
            operation_id,
            "AUTO_DOCK",
            "AUTO_DOCK_PLACE_COMPLETED",
            "TO_PLACE",
        )
        self.complete_place(operation_id)
        self.send_return_route()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    runner = AutoDockPlacePallet3Runner(
        RunnerConfig(
            robot_id=args.robot_id,
            inventory_url=args.inventory_url,
            fleet_url=args.fleet_url,
            request_timeout_sec=args.request_timeout_sec,
        )
    )
    signal.signal(signal.SIGINT, runner.request_stop)
    signal.signal(signal.SIGTERM, runner.request_stop)
    try:
        runner.run()
    except RunnerError as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
