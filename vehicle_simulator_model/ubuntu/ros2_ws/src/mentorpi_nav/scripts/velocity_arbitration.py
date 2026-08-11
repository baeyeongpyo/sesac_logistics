"""Framework-independent priority and timeout rules for local velocity control."""


class VelocityArbitrator:
    def __init__(self, timeout_seconds: float):
        self._timeout_seconds = timeout_seconds
        self._commands = {'manual': (None, None), 'nav': (None, None)}
        self._stop_time = None

    def record(self, source: str, command, now: float) -> None:
        if source not in self._commands:
            raise ValueError(f'unknown velocity source: {source}')
        self._commands[source] = (command, now)
        if source == 'manual':
            self._stop_time = None

    def stop(self, now: float) -> None:
        self._stop_time = now

    def select(self, now: float):
        if self._stop_time is not None:
            return None
        for source in ('manual', 'nav'):
            command, timestamp = self._commands[source]
            if command is not None and timestamp is not None and now - timestamp <= self._timeout_seconds:
                return command
        return None
