from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class Lease:
    robot_id: str
    expires_at: float


class CorridorReservation:
    def __init__(self, ttl_seconds: float):
        if ttl_seconds <= 0:
            raise ValueError('ttl_seconds must be positive')
        self._ttl = ttl_seconds
        self._leases: Dict[str, Lease] = {}

    def acquire(self, resource_id: str, robot_id: str, now: float) -> bool:
        lease = self._leases.get(resource_id)
        if lease is not None and lease.expires_at > now and lease.robot_id != robot_id:
            return False
        self._leases[resource_id] = Lease(robot_id, now + self._ttl)
        return True

    def renew(self, resource_id: str, robot_id: str, now: float) -> bool:
        lease = self._leases.get(resource_id)
        if lease is None or lease.robot_id != robot_id or lease.expires_at <= now:
            return False
        self._leases[resource_id] = Lease(robot_id, now + self._ttl)
        return True

    def release(self, resource_id: str, robot_id: str) -> bool:
        lease = self._leases.get(resource_id)
        if lease is None or lease.robot_id != robot_id:
            return False
        del self._leases[resource_id]
        return True

    def holder(self, resource_id: str, now: float) -> Optional[str]:
        lease = self._leases.get(resource_id)
        if lease is None:
            return None
        if lease.expires_at <= now:
            del self._leases[resource_id]
            return None
        return lease.robot_id
