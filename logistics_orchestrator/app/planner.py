from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence


REQUIRED_ZONE_IDS = (
    "docker",
    "p1",
    "p2",
    "p3",
    "f1",
    "f2",
    "f3",
    "f4",
    "f5",
    "f6",
    "f7",
    "f8",
    "f9",
    "n1",
    "n2",
    "n3",
    "n4",
    "n5",
    "n6",
    "n7",
    "n8",
    "n9",
)


@dataclass(frozen=True)
class TransferCandidate:
    source_zone_id: str
    destination_zone_id: str
    payload_type: str


@dataclass(frozen=True)
class InventorySnapshot:
    zones: Sequence[Mapping[str, Any]]
    stocks: Sequence[Mapping[str, Any]]
    active_operations: Sequence[Mapping[str, Any]]


def validate_required_zones(zones: Sequence[Mapping[str, Any]]) -> list[str]:
    indexed = {str(zone["zone_id"]).lower(): zone for zone in zones}
    errors: list[str] = []
    for zone_id in REQUIRED_ZONE_IDS:
        zone = indexed.get(zone_id)
        if zone is None:
            errors.append(f"{zone_id} missing")
        elif not bool(zone.get("enabled", False)):
            errors.append(f"{zone_id} disabled")
    return errors


def select_next_transfer(snapshot: InventorySnapshot) -> TransferCandidate | None:
    """Return the next safe one-pallet transfer, or None when no slot is usable."""
    if validate_required_zones(snapshot.zones):
        return None

    docker_fresh = _physical_quantity(snapshot, "docker", "FRESH")
    docker_normal = _physical_quantity(snapshot, "docker", "NORMAL")
    if docker_fresh > 0:
        return _from_source(snapshot, "docker", "FRESH", ("p", "f"))
    if docker_normal > 0:
        return _from_source(snapshot, "docker", "NORMAL", ("p", "n"))

    fresh = _first_source(snapshot, "f", "FRESH")
    if fresh is not None:
        candidate = _from_source(snapshot, fresh, "FRESH", ("p",))
        if candidate is not None:
            return candidate

    normal = _first_source(snapshot, "n", "NORMAL")
    if normal is not None:
        return _from_source(snapshot, normal, "NORMAL", ("p",))
    return None


def _from_source(
    snapshot: InventorySnapshot,
    source_zone_id: str,
    payload_type: str,
    destination_prefixes: Sequence[str],
) -> TransferCandidate | None:
    if _available_quantity(snapshot, source_zone_id, payload_type) <= 0:
        return None
    destination = _first_destination(snapshot, destination_prefixes)
    if destination is None:
        return None
    return TransferCandidate(source_zone_id, destination, payload_type)


def _first_source(
    snapshot: InventorySnapshot, prefix: str, payload_type: str
) -> str | None:
    for zone_id in _zone_ids(snapshot, prefix):
        if _available_quantity(snapshot, zone_id, payload_type) > 0:
            return zone_id
    return None


def _first_destination(
    snapshot: InventorySnapshot, prefixes: Sequence[str]
) -> str | None:
    for prefix in prefixes:
        for zone_id in _zone_ids(snapshot, prefix):
            if _has_destination_capacity(snapshot, zone_id):
                return zone_id
    return None


def _zone_ids(snapshot: InventorySnapshot, prefix: str) -> list[str]:
    zone_ids = [
        str(zone["zone_id"]).lower()
        for zone in snapshot.zones
        if str(zone["zone_id"]).lower().startswith(prefix)
    ]
    return sorted(zone_ids, key=_zone_sort_key)


def _zone_sort_key(zone_id: str) -> tuple[str, int]:
    return (zone_id[:1], int(zone_id[1:]) if zone_id[1:].isdigit() else 0)


def _physical_quantity(
    snapshot: InventorySnapshot, zone_id: str, payload_type: str
) -> int:
    return sum(
        int(stock.get("quantity", 0))
        for stock in snapshot.stocks
        if _stock_matches(stock, zone_id, payload_type)
    )


def _available_quantity(
    snapshot: InventorySnapshot, zone_id: str, payload_type: str
) -> int:
    return sum(
        int(
            stock.get(
                "available_quantity",
                int(stock.get("quantity", 0)) - int(stock.get("reserved_quantity", 0)),
            )
        )
        for stock in snapshot.stocks
        if _stock_matches(stock, zone_id, payload_type)
    )


def _stock_matches(stock: Mapping[str, Any], zone_id: str, payload_type: str) -> bool:
    return (
        str(stock.get("zone_id", "")).lower() == zone_id
        and str(stock.get("payload_type", "")).upper() == payload_type
    )


def _has_destination_capacity(snapshot: InventorySnapshot, zone_id: str) -> bool:
    zone = next(
        (
            item
            for item in snapshot.zones
            if str(item.get("zone_id", "")).lower() == zone_id
        ),
        None,
    )
    if zone is None or not bool(zone.get("enabled", False)):
        return False
    capacity = zone.get("capacity")
    if capacity is None:
        return True
    occupied = sum(
        int(stock.get("quantity", 0))
        for stock in snapshot.stocks
        if str(stock.get("zone_id", "")).lower() == zone_id
    )
    inbound = sum(
        1
        for operation in snapshot.active_operations
        if str(operation.get("destination_zone_id", "")).lower() == zone_id
    )
    return occupied + inbound < int(capacity)
