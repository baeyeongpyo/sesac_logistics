"""Process/file stop latch, independent of ROS for unit testing."""
from pathlib import Path
import signal
import threading


class TrialStop:
    def __init__(self, path):
        self.path = Path(path)
        self.baseline = self.version()
        self.requested = threading.Event()
        self.previous_handlers = {}

    def version(self):
        try:
            stat = self.path.stat()
            return stat.st_mtime_ns, stat.st_ino
        except FileNotFoundError:
            return None

    def handle_signal(self, signum, frame):
        self.requested.set()

    def install(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            self.previous_handlers[signum] = signal.signal(signum, self.handle_signal)

    def check(self):
        if self.version() != self.baseline:
            self.requested.set()
        if self.requested.is_set():
            raise InterruptedError('test_y stopped by signal or ~/stopper')

    def restore(self):
        for signum, handler in self.previous_handlers.items():
            signal.signal(signum, handler)
