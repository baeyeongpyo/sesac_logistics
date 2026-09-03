from __future__ import annotations

from typing import Any, Protocol

from .models import EventEnvelope, Pallet3Mission, Pallet3OperationWorkflow
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

    def pallet_state(self, robot_id: str) -> dict[str, Any]: ...


class FleetGateway(Protocol):
    def list_vehicles(self) -> list[dict[str, Any]]: ...
    def navigate(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def auto_dock(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def navigate_waypoints(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def fork_down(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def command_velocity(self, robot_id: str, payload: dict[str, Any]) -> None: ...
    def stop(self, robot_id: str) -> None: ...


class Pallet3MissionConflictError(ValueError):
    pass


class Pallet3OperationConflictError(ValueError):
    pass


OUTBOUND_PALLET3_WAYPOINTS = [
    {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
    {"frame_id": "map", "x": -0.420, "y": -2.000, "yaw": -1.5707963267948966},
    {"frame_id": "map", "x": -0.420, "y": -2.400, "yaw": -1.5707963267948966},
]

RETURN_DOCK1_WAYPOINTS = [
    {"frame_id": "map", "x": -0.420, "y": -2.000, "yaw": -1.5707963267948966},
    {"frame_id": "map", "x": -0.440, "y": -0.900, "yaw": 0.0},
    {"frame_id": "map", "x": 0.085, "y": -0.905, "yaw": 0.0},
]

AUTO_DOCK_LOCATION_BY_ZONE_ID = {
    "docker": "DOCK_1",
}


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

    def confirm_pallet3_manual_pick(
        self, operation_id: str, operator_confirmed: bool
    ) -> Pallet3OperationWorkflow:
        if not operator_confirmed:
            raise Pallet3OperationConflictError("operator_confirmed must be true")
        inventory = self._inventory.snapshot()
        operation = next(
            (
                item
                for item in inventory["active_operations"]
                if str(item.get("operation_id")) == operation_id
            ),
            None,
        )
        if operation is None:
            raise KeyError(f"unknown active operation: {operation_id}")
        if not _is_pallet3_operation(operation):
            raise Pallet3OperationConflictError(
                "operation is not a docker to pallet 3 transfer"
            )
        robot_id = str(operation.get("robot_id") or "").strip()
        if not robot_id:
            raise Pallet3OperationConflictError("operation has no assigned robot")
        workflow, _created = self._store.create_or_get_pallet3_workflow(
            operation_id, robot_id
        )
        if workflow.manual_pick_confirmed_at is not None:
            if str(operation.get("status")) == "TO_PLACE":
                return workflow
            raise Pallet3OperationConflictError(
                "manual pick was recorded but operation is not ready to place"
            )
        if str(operation.get("status")) != "TO_PICK":
            raise Pallet3OperationConflictError(
                "operation is not ready for pallet 3 manual pick"
            )
        pallet_state = self._inventory.pallet_state(robot_id)
        if bool(pallet_state.get("has_pallet")):
            raise Pallet3OperationConflictError("robot already carries a pallet")
        if self._store.has_command(operation_id, "NAV_TO_PICK") or self._store.has_command(
            operation_id, "AUTO_DOCK_PICK"
        ):
            raise Pallet3OperationConflictError("automatic pick has already started")
        try:
            self._inventory.complete_pick(
                operation_id, robot_id, f"{operation_id}:manual-pick"
            )
        except Exception as error:
            self._store.set_error("pallet3_manual_pick", str(error))
            raise
        self._store.clear_error("pallet3_manual_pick")
        workflow, _confirmed = self._store.confirm_pallet3_manual_pick(operation_id)
        if workflow is None:
            raise RuntimeError(f"pallet 3 workflow disappeared: {operation_id}")
        return workflow

    def start_pallet3_mission(
        self,
        robot_id: str,
        *,
        bypass_pick: bool = False,
        new_mission: bool = False,
    ) -> Pallet3Mission:
        robot_id = robot_id.strip()
        if not robot_id:
            raise ValueError("robot_id must be a non-empty string")
        if new_mission and not bypass_pick:
            raise Pallet3MissionConflictError(
                "new_mission is allowed only with bypass_pick for the pallet 3 POC"
            )
        try:
            vehicles = self._fleet.list_vehicles()
        except Exception as error:
            self._store.set_error("pallet3_vehicle_snapshot", str(error))
            raise
        vehicle = next(
            (item for item in vehicles if str(item.get("robot_id")) == robot_id),
            None,
        )
        if vehicle is None:
            raise KeyError(f"unknown vehicle: {robot_id}")
        mission_id: str | None = None
        if bypass_pick:
            if str(vehicle.get("state")) != "WAIT":
                raise Pallet3MissionConflictError(
                    "vehicle must be WAIT before bypassing Auto Dock PICK for the pallet 3 POC"
                )
        else:
            if (
                str(vehicle.get("state")) != "WAIT"
                or str(vehicle.get("source")) != "AUTO_DOCK"
                or str(vehicle.get("detail")) != "AUTO_DOCK_PICK_COMPLETED"
            ):
                raise Pallet3MissionConflictError(
                    "vehicle must report AUTO_DOCK_PICK_COMPLETED before the pallet 3 POC"
                )
            mission_id = str(vehicle.get("operation_id") or "").strip()
            if not mission_id:
                raise Pallet3MissionConflictError(
                    "AUTO_DOCK_PICK_COMPLETED must include the pickup operation_id"
                )
        self._store.clear_error("pallet3_vehicle_snapshot")
        mission, created = self._store.create_or_get_pallet3_mission(
            robot_id,
            mission_id,
            replace_active=new_mission,
        )
        if created:
            self._deliver_pallet3_command(
                mission,
                "POC_OUTBOUND_WAYPOINTS",
                {
                    "operation_id": mission.mission_id,
                    "purpose": "PLACE",
                    "waypoints": OUTBOUND_PALLET3_WAYPOINTS,
                },
                self._fleet.navigate_waypoints,
            )
        return self._store.get_pallet3_mission(mission.mission_id) or mission

    def confirm_pallet3_unload(self, mission_id: str) -> Pallet3Mission:
        mission, transitioned = self._store.confirm_pallet3_unload(mission_id)
        if mission is None:
            raise KeyError(f"unknown pallet 3 mission: {mission_id}")
        if not transitioned:
            if mission.phase in {
                "FORK_DOWN_SENT",
                "REVERSE_SENT",
                "RETURN_SENT",
                "COMPLETED",
            }:
                return mission
            raise Pallet3MissionConflictError(
                "unload confirmation is allowed only after pallet 3 arrival"
            )
        self._deliver_pallet3_command(
            mission,
            "POC_FORK_DOWN",
            {"operation_id": mission.mission_id},
            self._fleet.fork_down,
        )
        return self._store.get_pallet3_mission(mission.mission_id) or mission

    def reconcile(self) -> None:
        try:
            vehicles = self._fleet.list_vehicles()
        except Exception as error:
            self._store.set_error("fleet_snapshot", str(error))
            return

        self._store.clear_error("fleet_snapshot")
        vehicles_by_robot = {
            str(vehicle["robot_id"]): vehicle
            for vehicle in vehicles
            if vehicle.get("robot_id")
        }
        active_poc_robot_ids: set[str] = set()
        for mission in self._store.list_active_pallet3_missions():
            active_poc_robot_ids.add(mission.robot_id)
            vehicle = vehicles_by_robot.get(mission.robot_id)
            if vehicle is not None:
                self._reconcile_pallet3_mission(mission, vehicle)

        try:
            inventory = self._inventory.snapshot()
        except Exception as error:
            self._store.set_error("snapshot", str(error))
            return

        zones = inventory["zones"]
        configuration_errors = validate_required_zones(zones)
        if configuration_errors:
            self._store.set_error(
                "inventory_configuration", "; ".join(configuration_errors)
            )
        else:
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
        active_operation_ids = {
            str(operation["operation_id"])
            for operation in inventory["active_operations"]
            if operation.get("operation_id")
        }
        for vehicle in sorted(vehicles, key=lambda value: str(value["robot_id"])):
            robot_id = str(vehicle["robot_id"])
            if robot_id in active_poc_robot_ids:
                continue
            operation = active_by_robot.get(robot_id)
            vehicle_state = str(vehicle.get("state", ""))
            if vehicle_state == "FAIL":
                if operation is not None:
                    operation_id = str(operation["operation_id"])
                    self._store.mark_recovery_required(operation_id, robot_id)
                    if _is_pallet3_operation(operation):
                        self._handle_failed_pallet3_operation(
                            operation_id,
                            str(vehicle.get("detail") or "VEHICLE_FAIL"),
                        )
                continue
            if operation is not None and _is_pallet3_operation(operation):
                pallet3_errors = _pallet3_configuration_errors(zones)
                if pallet3_errors:
                    self._store.set_error(
                        "pallet3_inventory_configuration", "; ".join(pallet3_errors)
                    )
                    continue
                self._store.clear_error("pallet3_inventory_configuration")
                if vehicle_state == "WAIT":
                    self._continue_pallet3_operation(robot_id, vehicle, operation, zones)
                continue
            if configuration_errors:
                continue
            if vehicle_state != "WAIT":
                continue
            if operation is None:
                self._assign_next_operation(robot_id, vehicle, snapshot)
            else:
                self._continue_operation(robot_id, vehicle, operation, zones)

        for workflow in self._store.list_active_pallet3_workflows():
            if workflow.operation_id in active_operation_ids:
                continue
            if workflow.robot_id in active_poc_robot_ids:
                continue
            vehicle = vehicles_by_robot.get(workflow.robot_id)
            if vehicle is None:
                continue
            if str(vehicle.get("state")) == "FAIL":
                self._fail_pallet3_workflow(
                    workflow, str(vehicle.get("detail") or "VEHICLE_FAIL")
                )
                continue
            if str(vehicle.get("state")) == "WAIT":
                self._continue_pallet3_return(workflow, vehicle)

    def _continue_pallet3_operation(
        self,
        robot_id: str,
        vehicle: dict[str, Any],
        operation: dict[str, Any],
        zones: list[dict[str, Any]],
    ) -> None:
        operation_id = str(operation["operation_id"])
        workflow, _created = self._store.create_or_get_pallet3_workflow(
            operation_id, robot_id
        )
        status = str(operation["status"])
        detail = str(vehicle.get("detail", ""))
        recovery = self._store.recovery_required(operation_id)

        if status == "TO_PICK":
            if (
                detail == "NAVIGATION_SUCCEEDED"
                and _vehicle_report_matches(vehicle, operation_id, source="NAV2")
            ):
                self._auto_dock_for_operation(operation, purpose="PICK")
                return
            if (
                detail == "AUTO_DOCK_PICK_COMPLETED"
                and _vehicle_report_matches(vehicle, operation_id, source="AUTO_DOCK")
            ):
                self._complete_pallet3_pick(operation, workflow)
                return
            if detail == "OPERATOR_READY":
                self._navigate_for_operation(
                    operation, zones, purpose="PICK", recovery=recovery
                )
                if recovery:
                    self._store.clear_recovery_required(operation_id)
            return

        if status != "TO_PLACE" or workflow.phase in {"COMPLETED", "FAILED"}:
            return
        if workflow.phase == "PICK_PENDING" and detail == "OPERATOR_READY":
            self._send_pallet3_outbound(workflow, recovery=recovery)
            if recovery:
                self._store.clear_recovery_required(operation_id)
            return
        if workflow.phase == "OUTBOUND_SENT":
            if (
                detail == "NAVIGATION_SUCCEEDED"
                and _vehicle_report_matches(vehicle, operation_id, source="NAV2")
            ):
                next_workflow, transitioned = self._store.transition_pallet3_workflow(
                    operation_id, "OUTBOUND_SENT", "FORK_DOWN_SENT"
                )
                if transitioned and next_workflow is not None:
                    self._deliver_pallet3_workflow_command(
                        next_workflow,
                        "P3_FORK_DOWN",
                        {"operation_id": operation_id},
                        self._fleet.fork_down,
                    )
                return
            if detail == "OPERATOR_READY" and recovery:
                self._send_pallet3_outbound(workflow, recovery=True)
                self._store.clear_recovery_required(operation_id)
            return
        if workflow.phase == "FORK_DOWN_SENT":
            if not (
                detail == "FORK_DOWN_COMPLETE"
                and _vehicle_report_matches(vehicle, operation_id, source="FORK")
            ):
                return
            try:
                self._inventory.complete_place(
                    operation_id, robot_id, f"{operation_id}:place"
                )
            except Exception as error:
                self._store.set_error("pallet3_place_completion", str(error))
                return
            self._store.clear_error("pallet3_place_completion")
            next_workflow, transitioned = self._store.transition_pallet3_workflow(
                operation_id, "FORK_DOWN_SENT", "REVERSE_SENT"
            )
            if transitioned and next_workflow is not None:
                self._deliver_pallet3_workflow_command(
                    next_workflow,
                    "P3_REVERSE",
                    {
                        "operation_id": operation_id,
                        "linear_x": -0.18,
                        "linear_y": 0.0,
                        "angular_z": 0.0,
                        "hold_ms": 1000,
                    },
                    self._fleet.command_velocity,
                )
            return
        if workflow.phase == "REVERSE_SENT":
            if not (
                detail == "MANUAL_COMMAND_EXPIRED"
                and _vehicle_report_matches(vehicle, operation_id, source="API")
            ):
                return
            next_workflow, transitioned = self._store.transition_pallet3_workflow(
                operation_id, "REVERSE_SENT", "RETURN_SENT"
            )
            if transitioned and next_workflow is not None:
                self._deliver_pallet3_workflow_command(
                    next_workflow,
                    "P3_RETURN_WAYPOINTS",
                    {
                        "operation_id": operation_id,
                        "purpose": "RETURN_DOCK_1",
                        "waypoints": RETURN_DOCK1_WAYPOINTS,
                    },
                    self._fleet.navigate_waypoints,
                )
            return
        if (
            workflow.phase == "RETURN_SENT"
            and detail == "NAVIGATION_SUCCEEDED"
            and _vehicle_report_matches(vehicle, operation_id, source="NAV2")
        ):
            self._store.transition_pallet3_workflow(
                operation_id, "RETURN_SENT", "COMPLETED"
            )

    def _complete_pallet3_pick(
        self, operation: dict[str, Any], workflow: Pallet3OperationWorkflow
    ) -> None:
        operation_id = str(operation["operation_id"])
        robot_id = str(operation["robot_id"])
        try:
            self._inventory.complete_pick(operation_id, robot_id, f"{operation_id}:pick")
        except Exception as error:
            self._store.set_error("pallet3_pick_completion", str(error))
            return
        self._store.clear_error("pallet3_pick_completion")
        self._send_pallet3_outbound(workflow)

    def _continue_pallet3_return(
        self, workflow: Pallet3OperationWorkflow, vehicle: dict[str, Any]
    ) -> None:
        operation_id = workflow.operation_id
        detail = str(vehicle.get("detail", ""))
        if workflow.phase == "REVERSE_SENT":
            if not (
                detail == "MANUAL_COMMAND_EXPIRED"
                and _vehicle_report_matches(vehicle, operation_id, source="API")
            ):
                return
            next_workflow, transitioned = self._store.transition_pallet3_workflow(
                operation_id, "REVERSE_SENT", "RETURN_SENT"
            )
            if transitioned and next_workflow is not None:
                self._deliver_pallet3_workflow_command(
                    next_workflow,
                    "P3_RETURN_WAYPOINTS",
                    {
                        "operation_id": operation_id,
                        "purpose": "RETURN_DOCK_1",
                        "waypoints": RETURN_DOCK1_WAYPOINTS,
                    },
                    self._fleet.navigate_waypoints,
                )
            return
        if (
            workflow.phase == "RETURN_SENT"
            and detail == "NAVIGATION_SUCCEEDED"
            and _vehicle_report_matches(vehicle, operation_id, source="NAV2")
        ):
            self._store.transition_pallet3_workflow(
                operation_id, "RETURN_SENT", "COMPLETED"
            )

    def _send_pallet3_outbound(
        self, workflow: Pallet3OperationWorkflow, *, recovery: bool = False
    ) -> None:
        operation_id = workflow.operation_id
        if workflow.phase == "PICK_PENDING":
            workflow, transitioned = self._store.transition_pallet3_workflow(
                operation_id, "PICK_PENDING", "OUTBOUND_SENT"
            )
            if not transitioned or workflow is None:
                return
        elif workflow.phase != "OUTBOUND_SENT" or not recovery:
            return
        command_type = "P3_OUTBOUND_WAYPOINTS"
        if recovery and self._store.has_command(operation_id, command_type):
            command_type = "P3_OUTBOUND_WAYPOINTS_RECOVERY"
        self._deliver_pallet3_workflow_command(
            workflow,
            command_type,
            {
                "operation_id": operation_id,
                "purpose": "PLACE",
                "waypoints": OUTBOUND_PALLET3_WAYPOINTS,
            },
            self._fleet.navigate_waypoints,
        )

    def _deliver_pallet3_workflow_command(
        self,
        workflow: Pallet3OperationWorkflow,
        command_type: str,
        payload: dict[str, Any],
        send: Any,
    ) -> None:
        command = self._deliver_command(
            workflow.operation_id,
            workflow.robot_id,
            command_type,
            payload,
            send,
        )
        delivered = self._store.get_command(command.command_id)
        if delivered is not None and delivered.status != "SENT":
            detail = delivered.last_error or delivered.status
            self._fail_pallet3_workflow(workflow, f"{command_type}:{detail}")

    def _handle_failed_pallet3_operation(self, operation_id: str, detail: str) -> None:
        workflow = self._store.get_pallet3_workflow(operation_id)
        if workflow is None or workflow.phase in {"PICK_PENDING", "OUTBOUND_SENT"}:
            return
        self._fail_pallet3_workflow(workflow, detail)

    def _fail_pallet3_workflow(
        self, workflow: Pallet3OperationWorkflow, detail: str
    ) -> None:
        failed = self._store.fail_pallet3_workflow(workflow.operation_id, detail)
        if failed is not None and failed.phase == "FAILED":
            self._deliver_command(
                workflow.operation_id,
                workflow.robot_id,
                "P3_SAFETY_STOP",
                {},
                lambda robot_id, _payload: self._fleet.stop(robot_id),
            )

    def _reconcile_pallet3_mission(
        self, mission: Pallet3Mission, vehicle: dict[str, Any]
    ) -> None:
        if str(vehicle.get("state")) == "FAIL":
            self._fail_pallet3_mission(
                mission, str(vehicle.get("detail") or "VEHICLE_FAIL")
            )
            return
        if mission.phase == "OUTBOUND_SENT":
            if _poc_vehicle_report_matches(
                vehicle, mission, source="NAV2", detail="NAVIGATION_SUCCEEDED"
            ):
                self._store.transition_pallet3_mission(
                    mission.mission_id,
                    "OUTBOUND_SENT",
                    "AWAIT_UNLOAD_CONFIRMATION",
                )
            return
        if mission.phase == "FORK_DOWN_SENT":
            if _poc_vehicle_report_matches(
                vehicle, mission, source="FORK", detail="FORK_DOWN_COMPLETE"
            ):
                next_mission, transitioned = self._store.transition_pallet3_mission(
                    mission.mission_id,
                    "FORK_DOWN_SENT",
                    "REVERSE_SENT",
                )
                if transitioned and next_mission is not None:
                    self._deliver_pallet3_command(
                        next_mission,
                        "POC_REVERSE",
                        {
                            "operation_id": next_mission.mission_id,
                            "linear_x": -0.18,
                            "linear_y": 0.0,
                            "angular_z": 0.0,
                            "hold_ms": 1000,
                        },
                        self._fleet.command_velocity,
                    )
            return
        if mission.phase == "REVERSE_SENT":
            if _poc_vehicle_report_matches(
                vehicle, mission, source="API", detail="MANUAL_COMMAND_EXPIRED"
            ):
                next_mission, transitioned = self._store.transition_pallet3_mission(
                    mission.mission_id,
                    "REVERSE_SENT",
                    "RETURN_SENT",
                )
                if transitioned and next_mission is not None:
                    self._deliver_pallet3_command(
                        next_mission,
                        "POC_RETURN_WAYPOINTS",
                        {
                            "operation_id": next_mission.mission_id,
                            "purpose": "PLACE",
                            "waypoints": RETURN_DOCK1_WAYPOINTS,
                        },
                        self._fleet.navigate_waypoints,
                    )
            return
        if mission.phase == "RETURN_SENT" and _poc_vehicle_report_matches(
            vehicle, mission, source="NAV2", detail="NAVIGATION_SUCCEEDED"
        ):
            self._store.transition_pallet3_mission(
                mission.mission_id,
                "RETURN_SENT",
                "COMPLETED",
            )

    def _deliver_pallet3_command(
        self,
        mission: Pallet3Mission,
        command_type: str,
        payload: dict[str, Any],
        send: Any,
    ) -> None:
        command = self._deliver_command(
            mission.mission_id,
            mission.robot_id,
            command_type,
            payload,
            send,
        )
        delivered = self._store.get_command(command.command_id)
        if delivered is not None and delivered.status != "SENT":
            detail = delivered.last_error or delivered.status
            self._fail_pallet3_mission(mission, f"{command_type}:{detail}")

    def _fail_pallet3_mission(self, mission: Pallet3Mission, detail: str) -> None:
        failed = self._store.fail_pallet3_mission(mission.mission_id, detail)
        if failed is not None and failed.phase == "FAILED":
            self._deliver_command(
                mission.mission_id,
                mission.robot_id,
                "POC_SAFETY_STOP",
                {},
                lambda robot_id, _payload: self._fleet.stop(robot_id),
            )

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
            "location": AUTO_DOCK_LOCATION_BY_ZONE_ID.get(
                zone_id.lower(), zone_id.upper()
            ),
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
    ) -> Any:
        command = self._store.enqueue_command(
            operation_id=operation_id,
            robot_id=robot_id,
            command_type=command_type,
            payload=payload,
        )
        if command.status != "PENDING":
            return command
        try:
            send(robot_id, payload)
        except TimeoutError as error:
            self._store.mark_command_delivery_unknown(command.command_id, str(error))
        except Exception as error:
            self._store.mark_command_failed(command.command_id, str(error))
        else:
            self._store.mark_command_sent(command.command_id)
        return command


def _zone_by_id(zones: list[dict[str, Any]], zone_id: str) -> dict[str, Any]:
    for zone in zones:
        if str(zone["zone_id"]).lower() == zone_id.lower():
            return zone
    raise ValueError(f"configured zone disappeared: {zone_id}")


def _is_pallet3_operation(operation: dict[str, Any]) -> bool:
    return (
        str(operation.get("source_zone_id", "")).lower() == "docker"
        and str(operation.get("destination_zone_id", "")).lower() == "p3"
    )


def _pallet3_configuration_errors(zones: list[dict[str, Any]]) -> list[str]:
    indexed = {str(zone.get("zone_id", "")).lower(): zone for zone in zones}
    errors: list[str] = []
    for zone_id in ("docker", "p3"):
        zone = indexed.get(zone_id)
        if zone is None:
            errors.append(f"{zone_id} missing")
        elif not bool(zone.get("enabled", False)):
            errors.append(f"{zone_id} disabled")
    return errors


def _vehicle_report_matches(
    vehicle: dict[str, Any], operation_id: str, *, source: str
) -> bool:
    return (
        str(vehicle.get("state")) == "WAIT"
        and str(vehicle.get("operation_id")) == operation_id
        and str(vehicle.get("source")) == source
    )


def _poc_vehicle_report_matches(
    vehicle: dict[str, Any],
    mission: Pallet3Mission,
    *,
    source: str,
    detail: str,
) -> bool:
    return (
        str(vehicle.get("state")) == "WAIT"
        and str(vehicle.get("operation_id")) == mission.mission_id
        and str(vehicle.get("source")) == source
        and str(vehicle.get("detail")) == detail
    )
