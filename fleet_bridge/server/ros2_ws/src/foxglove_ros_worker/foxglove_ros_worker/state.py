"""Thread-safe connection and telemetry freshness state."""

import math
from threading import Lock
from typing import Any


class ReconnectBackoff:
    def __init__(self, initial: float = 1.0, maximum: float = 30.0) -> None:
        if initial <= 0 or maximum < initial:
            raise ValueError('backoff requires 0 < initial <= maximum')
        self._initial = float(initial)
        self._maximum = float(maximum)
        self._next = self._initial

    def next_delay(self) -> float:
        delay = self._next
        self._next = min(self._maximum, delay * 2.0)
        return delay

    def reset(self) -> None:
        self._next = self._initial


class WorkerState:
    def __init__(self, robot_id: str, freshness_timeout_sec: float) -> None:
        if not robot_id:
            raise ValueError('robot_id must not be empty')
        if freshness_timeout_sec <= 0 or not math.isfinite(freshness_timeout_sec):
            raise ValueError('freshness_timeout_sec must be a finite positive number')
        self._robot_id = robot_id
        self._freshness_timeout_sec = float(freshness_timeout_sec)
        self._connection = 'disconnected'
        self._last_message_at: float | None = None
        self._reconnect_count = 0
        self._error: str | None = None
        self._lock = Lock()

    def connected(self) -> None:
        with self._lock:
            self._connection = 'connected'
            self._error = None

    def record_message(self, received_at: float) -> None:
        if received_at < 0 or not math.isfinite(received_at):
            raise ValueError('received_at must be a finite non-negative number')
        with self._lock:
            self._last_message_at = float(received_at)

    def disconnected(self, error: str, reconnect: bool = False) -> None:
        with self._lock:
            self._connection = 'disconnected'
            self._error = error
            if reconnect:
                self._reconnect_count += 1

    def snapshot(self, now: float) -> dict[str, Any]:
        if now < 0 or not math.isfinite(now):
            raise ValueError('now must be a finite non-negative number')
        with self._lock:
            if self._connection != 'connected':
                state = 'offline'
            elif self._last_message_at is None:
                state = 'waiting'
            elif now - self._last_message_at <= self._freshness_timeout_sec:
                state = 'online'
            else:
                state = 'stale'
            return {
                'robot_id': self._robot_id,
                'connection': self._connection,
                'state': state,
                'last_message_at': self._last_message_at,
                'reconnect_count': self._reconnect_count,
                'error': self._error,
            }
