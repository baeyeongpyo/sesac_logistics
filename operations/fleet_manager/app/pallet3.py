from __future__ import annotations

import json
from dataclasses import asdict
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .commands import BridgeCommandGateway, RegisteredVehicle, VehicleRegistry
from .fleet import DirectPallet3Mission, VehicleState, VehicleStateStore


class InventoryRequestError(RuntimeError):
    def __init__(self, message: str, *, status_code: int | None = None) -> None:
        super().__init__(message)
        self.status_code = status_code


class InventoryGateway(Protocol):
    def create_operation(
        self,
        payload_type: str,
        source_zone_id: str,
        destination_zone_id: str,
        robot_id: str,
    ) -> str: ...

    def complete_pick(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None: ...

    def complete_place(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None: ...


class InventoryClient:
    def __init__(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")

    def create_operation(
        self,
        payload_type: str,
        source_zone_id: str,
        destination_zone_id: str,
        robot_id: str,
    ) -> str:
        body = self._post(
            "/api/v1/operations",
            {
                "robot_id": robot_id,
                "payload_type": payload_type,
                "source_zone_id": source_zone_id,
                "destination_zone_id": destination_zone_id,
                "priority": 0,
            },
        )
        operation_id = body.get("operation_id") if isinstance(body, dict) else None
        if not isinstance(operation_id, str) or not operation_id:
            raise InventoryRequestError("inventory create operation response has no operation_id")
        return operation_id

    def complete_pick(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None:
        self._post(
            f"/api/v1/operations/{operation_id}/pick-completions",
            {"robot_id": robot_id, "idempotency_key": idempotency_key},
        )

    def complete_place(
        self, operation_id: str, robot_id: str, idempotency_key: str
    ) -> None:
        self._post(
            f"/api/v1/operations/{operation_id}/place-completions",
            {"robot_id": robot_id, "idempotency_key": idempotency_key},
        )

    def _post(self, path: str, body: dict[str, Any]) -> Any:
        request = Request(
            f"{self._base_url}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=5) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            raise InventoryRequestError(
                f"inventory returned HTTP {error.code}", status_code=error.code
            ) from error
        except (URLError, OSError, json.JSONDecodeError) as error:
            raise InventoryRequestError(f"inventory request failed: {error}") from error


class Pallet3MissionService:
    _PICK_REPLY = {"auto_dock": "AUTO_DOCK_PICK_COMPLETED", "manual": "FORK_UP_COMPLETE"}

    def __init__(
        self,
        store: VehicleStateStore,
        inventory: InventoryGateway,
        registry: VehicleRegistry,
        bridge: BridgeCommandGateway,
        *,
        fleet_manager_url: str,
    ) -> None:
        self._store = store
        self._inventory = inventory
        self._registry = registry
        self._bridge = bridge
        self._fleet_manager_url = fleet_manager_url.rstrip("/")

    def start(self, robot_id: str, pick_mode: str) -> dict[str, Any]:
        if pick_mode not in self._PICK_REPLY:
            raise ValueError("pick_mode must be auto_dock or manual")
        snapshot = self._store.get_vehicle(robot_id)
        if snapshot is None or snapshot.state != VehicleState.WAIT:
            raise ValueError(f"vehicle {robot_id} must be in WAIT state")
        active_mission = self._store.get_active_direct_pallet3_mission(robot_id)
        if active_mission is not None:
            raise ValueError(
                f"pallet3 mission {active_mission.operation_id} is already active"
            )
        vehicle = self._registry.require(robot_id, "pallet3_mission")
        operation_id = self._inventory.create_operation(
            "NORMAL", "docker", "p3", robot_id
        )
        self._store.create_direct_pallet3_mission(operation_id, robot_id, pick_mode)
        relay = self._bridge.relay(
            vehicle,
            f"/api/v1/vehicle-command/{robot_id}/missions/pallet3",
            {
                "operation_id": operation_id,
                "robot_id": robot_id,
                "pick_mode": pick_mode,
                "fleet_manager_url": self._fleet_manager_url,
            },
        )
        if not 200 <= relay.status_code < 300:
            raise RuntimeError(f"vehicle mission start returned HTTP {relay.status_code}")
        mission = self._store.mark_direct_pallet3_started(operation_id)
        return _mission_response(mission)

    def record_vehicle_reply(
        self, robot_id: str, operation_id: str, detail: str
    ) -> None:
        self._store.record_direct_pallet3_reply(robot_id, operation_id, detail)

    def record_event(
        self,
        robot_id: str,
        operation_id: str,
        event_type: str,
        idempotency_key: str,
    ) -> dict[str, Any]:
        mission = self._require_mission(robot_id, operation_id)
        if event_type == "PICK_COMPLETED":
            return self._complete_pick(mission, idempotency_key)
        if event_type == "PLACE_READY":
            return self._complete_place(mission, idempotency_key)
        if event_type == "RETURN_COMPLETED":
            return self._complete_return(mission, idempotency_key)
        raise ValueError(f"unsupported pallet3 event_type: {event_type}")

    def _require_mission(self, robot_id: str, operation_id: str) -> DirectPallet3Mission:
        mission = self._store.get_direct_pallet3_mission(operation_id)
        if mission is None:
            raise KeyError(f"unknown pallet3 mission: {operation_id}")
        if mission.robot_id != robot_id:
            raise ValueError(f"operation {operation_id} does not belong to {robot_id}")
        return mission

    def _complete_pick(
        self, mission: DirectPallet3Mission, idempotency_key: str
    ) -> dict[str, Any]:
        if mission.phase == "PICKED" and mission.pick_idempotency_key == idempotency_key:
            return _mission_response(mission)
        required_reply = self._PICK_REPLY[mission.pick_mode]
        if not self._store.has_direct_pallet3_reply(mission.operation_id, required_reply):
            raise ValueError(f"PICK_COMPLETED requires {required_reply}")
        if mission.phase != "STARTED":
            raise ValueError(f"PICK_COMPLETED is invalid in {mission.phase}")
        self._inventory.complete_pick(
            mission.operation_id, mission.robot_id, idempotency_key
        )
        completed = self._store.complete_direct_pallet3_event(
            mission.operation_id,
            "STARTED",
            "PICKED",
            "pick_idempotency_key",
            idempotency_key,
        )
        return _mission_response(completed)

    def _complete_place(
        self, mission: DirectPallet3Mission, idempotency_key: str
    ) -> dict[str, Any]:
        if mission.phase == "PLACED" and mission.place_idempotency_key == idempotency_key:
            return _mission_response(mission)
        if not self._store.has_direct_pallet3_reply(
            mission.operation_id, "FORK_DOWN_COMPLETE"
        ):
            raise ValueError("PLACE_READY requires FORK_DOWN_COMPLETE")
        if mission.phase != "PICKED":
            raise ValueError(f"PLACE_READY is invalid in {mission.phase}")
        self._inventory.complete_place(
            mission.operation_id, mission.robot_id, idempotency_key
        )
        completed = self._store.complete_direct_pallet3_event(
            mission.operation_id,
            "PICKED",
            "PLACED",
            "place_idempotency_key",
            idempotency_key,
        )
        return _mission_response(completed)

    def _complete_return(
        self, mission: DirectPallet3Mission, idempotency_key: str
    ) -> dict[str, Any]:
        if mission.phase == "RETURNED" and mission.return_idempotency_key == idempotency_key:
            return _mission_response(mission)
        if mission.phase != "PLACED":
            raise ValueError(f"RETURN_COMPLETED is invalid in {mission.phase}")
        completed = self._store.complete_direct_pallet3_event(
            mission.operation_id,
            "PLACED",
            "RETURNED",
            "return_idempotency_key",
            idempotency_key,
        )
        return _mission_response(completed)


def _mission_response(mission: DirectPallet3Mission) -> dict[str, Any]:
    return asdict(mission)
