import math

from fleet_bridge_config.models import FilterConfig


def _field_value(message: object, field_path: str) -> float | None:
    value = message
    for segment in field_path.split('.'):
        if not hasattr(value, segment):
            return None
        value = getattr(value, segment)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


class ForwardPolicy:
    """Stateful vehicle-side filter driven only by monotonic timestamps."""

    def __init__(self, config: FilterConfig):
        self._config = config
        self._last_forward_ns: int | None = None
        self._last_values: dict[str, float] = {}

    def _rate_ready(self, now_ns: int) -> bool:
        if self._last_forward_ns is None or now_ns < self._last_forward_ns:
            return True
        if self._config.max_rate_hz is None:
            return True
        minimum_period_ns = int(1_000_000_000 / self._config.max_rate_hz)
        return now_ns - self._last_forward_ns >= minimum_period_ns

    def _is_critical(self, message: object) -> bool:
        critical = self._config.critical
        if not critical.bypass_rate_limit or critical.field is None or critical.below is None:
            return False
        value = _field_value(message, critical.field)
        return value is not None and value <= critical.below

    def _capture_values(self, message: object) -> dict[str, float]:
        values = {}
        for name in self._config.thresholds:
            value = _field_value(message, name)
            if value is not None:
                values[name] = value
        return values

    def _record(self, message: object, now_ns: int) -> None:
        self._last_forward_ns = now_ns
        if self._config.mode == 'on_change':
            self._last_values = self._capture_values(message)

    def should_forward(self, message: object, now_ns: int) -> bool:
        if now_ns < 0:
            raise ValueError('now_ns must not be negative')
        if self._config.mode == 'passthrough':
            return True
        if self._is_critical(message):
            self._record(message, now_ns)
            return True
        if self._last_forward_ns is None or now_ns < self._last_forward_ns:
            self._record(message, now_ns)
            return True
        if not self._rate_ready(now_ns):
            return False
        if self._config.mode == 'rate':
            self._record(message, now_ns)
            return True

        if self._config.heartbeat_sec is not None:
            heartbeat_ns = int(self._config.heartbeat_sec * 1_000_000_000)
            if now_ns - self._last_forward_ns >= heartbeat_ns:
                self._record(message, now_ns)
                return True

        current_values = self._capture_values(message)
        changed = any(
            name not in self._last_values
            or abs(value - self._last_values[name]) >= self._config.thresholds[name]
            for name, value in current_values.items()
        )
        if changed:
            self._record(message, now_ns)
        return changed

