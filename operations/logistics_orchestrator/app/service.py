from __future__ import annotations

from typing import Any, Protocol

from .models import EventEnvelope
from .planner import InventorySnapshot, select_next_transfer, validate_required_zones
from .store import OrchestratorStore


class InventoryGateway(Protocol):
    def snapshot(self) -> dict[str, list[dict[str, Any]]]: ...

    def create_operation(self, body: dict[str, Any]) -> dict[str, Any]: ...

    def complete_pick(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None: ...

    def complete_place(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None: ...


class FleetGateway(Protocol):
    def list_vehicles(self) -> list[dict[str, Any]]: ...
    def navigate(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def auto_dock(self, robot_id: str, payload: dict[str, Any]) -> None: ...


class OrchestratorService:
    def __init__(
        self,
        store: OrchestratorStore,
        inventory: InventoryGateway,
        fleet: FleetGateway,
    ) -> None:
        self._store = store
        self._inventory = inventory
        self._fleet = fleet

    def handle_recorded_event(self, source: str, event: EventEnvelope) -> None:
        """A durable event is only a trigger; reconciliation uses current ledgers."""
        del source, event
        self.reconcile()

    def reconcile(self) -> None:
        try:
            inventory = self._inventory.snapshot()
            vehicles = self._fleet.list_vehicles()
        except Exception as error:
            self._store.set_error("snapshot", str(error))
            return

        zones = inventory["zones"]
        configuration_errors = validate_required_zones(zones)
        if configuration_errors:
            self._store.set_error(
                "inventory_configuration", "; ".join(configuration_errors)
            )
            return
        self._store.clear_error("inventory_configuration")
        self._store.clear_error("snapshot")

        snapshot = InventorySnapshot(
            zones=zones,
            stocks=inventory["stocks"],
            active_operations=inventory["active_operations"],
        )
        active_by_robot = {
            str(operation["robot_id"]): operation
            for operation in inventory["active_operations"]
            if operation.get("robot_id")
        }
        for vehicle in sorted(vehicles, key=lambda value: str(value["robot_id"])):
            robot_id = str(vehicle["robot_id"])
            operation = active_by_robot.get(robot_id)
            vehicle_state = str(vehicle.get("state", ""))
            if vehicle_state == "FAIL":
                if operation is not None:
                    self._store.mark_recovery_required(
                        str(operation["operation_id"]), robot_id
                    )
                continue
            if vehicle_state != "WAIT":
                continue
            if operation is None:
                self._assign_next_operation(robot_id, vehicle, snapshot)
            else:
                self._continue_operation(robot_id, vehicle, operation, zones)

    def _assign_next_operation(
        self,
        robot_id: str,
        vehicle: dict[str, Any],
        snapshot: InventorySnapshot,
    ) -> None:
        if str(vehicle.get("detail", "")) in {
            "AUTO_DOCK_PICK_COMPLETED",
            "AUTO_DOCK_PLACE_COMPLETED",
        }:
            return
        candidate = select_next_transfer(snapshot)
        if candidate is None:
            return
        try:
            operation = self._inventory.create_operation(
                {
                    "robot_id": robot_id,
                    "payload_type": candidate.payload_type,
                    "source_zone_id": candidate.source_zone_id,
                    "destination_zone_id": candidate.destination_zone_id,
                    "priority": 0,
                }
            )
        except Exception as error:
            self._store.set_error("operation_reservation", str(error))
            return
        self._store.clear_error("operation_reservation")
        self._navigate_for_operation(operation, snapshot.zones, purpose="PICK")

    def _continue_operation(
        self,
        robot_id: str,
        vehicle: dict[str, Any],
        operation: dict[str, Any],
        zones: list[dict[str, Any]],
    ) -> None:
        operation_id = str(operation["operation_id"])
        status = str(operation["status"])
        detail = str(vehicle.get("detail", ""))
        if detail == "NAVIGATION_SUCCEEDED":
            if status == "TO_PICK":
                self._auto_dock_for_operation(operation, purpose="PICK")
            elif status == "TO_PLACE":
                self._auto_dock_for_operation(operation, purpose="PLACE")
            return
        if detail == "AUTO_DOCK_PICK_COMPLETED" and status == "TO_PICK":
            try:
                self._inventory.complete_pick(
                    operation_id, robot_id, f"{operation_id}:pick"
                )
            except Exception as error:
                self._store.set_error("pick_completion", str(error))
                return
            self._store.clear_error("pick_completion")
            self._store.upsert_step(
                operation_id=operation_id,
                robot_id=robot_id,
                phase="PICK_COMMITTED",
                source_zone_id=str(operation["source_zone_id"]),
                destination_zone_id=str(operation["destination_zone_id"]),
                payload_type=str(operation["payload_type"]),
            )
            self._navigate_for_operation(
                {**operation, "status": "TO_PLACE"}, zones, purpose="PLACE"
            )
            return
        if detail == "AUTO_DOCK_PLACE_COMPLETED" and status == "TO_PLACE":
            try:
                self._inventory.complete_place(
                    operation_id, robot_id, f"{operation_id}:place"
                )
            except Exception as error:
                self._store.set_error("place_completion", str(error))
                return
            self._store.clear_error("place_completion")
            self._store.upsert_step(
                operation_id=operation_id,
                robot_id=robot_id,
                phase="PLACE_COMMITTED",
                source_zone_id=str(operation["source_zone_id"]),
                destination_zone_id=str(operation["destination_zone_id"]),
                payload_type=str(operation["payload_type"]),
            )
            return
        if detail == "OPERATOR_READY":
            purpose = "PICK" if status == "TO_PICK" else "PLACE"
            recovery = self._store.recovery_required(operation_id)
            self._navigate_for_operation(operation, zones, purpose=purpose, recovery=recovery)
            if recovery:
                self._store.clear_recovery_required(operation_id)

    def _navigate_for_operation(
        self,
        operation: dict[str, Any],
        zones: list[dict[str, Any]],
        *,
        purpose: str,
        recovery: bool = False,
    ) -> None:
        operation_id = str(operation["operation_id"])
        robot_id = str(operation["robot_id"])
        zone_id = (
            str(operation["source_zone_id"])
            if purpose == "PICK"
            else str(operation["destination_zone_id"])
        )
        zone = _zone_by_id(zones, zone_id)
        command_type = f"NAV_TO_{purpose}"
        if recovery and self._store.has_command(operation_id, command_type):
            command_type = f"{command_type}_RECOVERY"
        payload = {
            "operation_id": operation_id,
            "purpose": purpose,
            "frame_id": "map",
            "x": zone["nav_x"],
            "y": zone["nav_y"],
            "yaw": zone["nav_yaw"],
        }
        phase = "NAV_TO_PICK_SENT" if purpose == "PICK" else "NAV_TO_PLACE_SENT"
        self._store.upsert_step(
            operation_id=operation_id,
            robot_id=robot_id,
            phase=phase,
            source_zone_id=str(operation["source_zone_id"]),
            destination_zone_id=str(operation["destination_zone_id"]),
            payload_type=str(operation["payload_type"]),
        )
        self._deliver_command(
            operation_id, robot_id, command_type, payload, self._fleet.navigate
        )

    def _auto_dock_for_operation(self, operation: dict[str, Any], *, purpose: str) -> None:
        operation_id = str(operation["operation_id"])
        robot_id = str(operation["robot_id"])
        zone_id = (
            str(operation["source_zone_id"])
            if purpose == "PICK"
            else str(operation["destination_zone_id"])
        )
        payload = {
            "operation_id": operation_id,
            "operation": purpose,
            "product_type": str(operation["payload_type"]),
            "location": zone_id.upper(),
            "target": {"type": "NEAREST"},
        }
        phase = "PICK_SENT" if purpose == "PICK" else "PLACE_SENT"
        self._store.upsert_step(
            operation_id=operation_id,
            robot_id=robot_id,
            phase=phase,
            source_zone_id=str(operation["source_zone_id"]),
            destination_zone_id=str(operation["destination_zone_id"]),
            payload_type=str(operation["payload_type"]),
        )
        self._deliver_command(
            operation_id,
            robot_id,
            f"AUTO_DOCK_{purpose}",
            payload,
            self._fleet.auto_dock,
        )

    def _deliver_command(
        self,
        operation_id: str,
        robot_id: str,
        command_type: str,
        payload: dict[str, Any],
        send: Any,
    ) -> None:
        command = self._store.enqueue_command(
            operation_id=operation_id,
            robot_id=robot_id,
            command_type=command_type,
            payload=payload,
        )
        if command.status != "PENDING":
            return
        try:
            send(robot_id, payload)
        except TimeoutError as error:
            self._store.mark_command_delivery_unknown(command.command_id, str(error))
        except Exception as error:
            self._store.mark_command_failed(command.command_id, str(error))
        else:
            self._store.mark_command_sent(command.command_id)


def _zone_by_id(zones: list[dict[str, Any]], zone_id: str) -> dict[str, Any]:
    for zone in zones:
        if str(zone["zone_id"]).lower() == zone_id.lower():
            return zone
    raise ValueError(f"configured zone disappeared: {zone_id}")
