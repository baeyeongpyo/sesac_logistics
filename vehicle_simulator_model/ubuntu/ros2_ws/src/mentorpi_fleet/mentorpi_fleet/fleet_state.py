"""Pure online/offline state for fleet presence publication."""

from .registry import VehicleKind, VehicleSpec


class FleetPresence:
    def __init__(self, timeout_seconds: float):
        self._timeout_seconds = timeout_seconds
        self._vehicles: dict[str, VehicleSpec] = {}
        self._last_odom: dict[str, float] = {}

    def reconcile(self, vehicles: list[VehicleSpec], kind: VehicleKind | None = None) -> None:
        selected = {
            vehicle.vehicle_id: vehicle for vehicle in vehicles
            if vehicle.enabled and (kind is None or vehicle.kind == kind)
        }
        self._vehicles = selected
        self._last_odom = {
            vehicle_id: timestamp for vehicle_id, timestamp in self._last_odom.items()
            if vehicle_id in selected
        }

    def record_odom(self, vehicle_id: str, now: float) -> None:
        if vehicle_id in self._vehicles:
            self._last_odom[vehicle_id] = now

    def snapshot(self, now: float) -> list[dict]:
        return [
            {
                'id': vehicle.vehicle_id,
                'kind': vehicle.kind,
                'domain_id': vehicle.domain_id,
                'online': self._is_online(vehicle.vehicle_id, now),
                'state': 'online' if self._is_online(vehicle.vehicle_id, now) else 'offline',
            }
            for vehicle in sorted(self._vehicles.values(), key=lambda item: item.vehicle_id)
        ]

    def _is_online(self, vehicle_id: str, now: float) -> bool:
        last_odom = self._last_odom.get(vehicle_id)
        return last_odom is not None and now - last_odom <= self._timeout_seconds
