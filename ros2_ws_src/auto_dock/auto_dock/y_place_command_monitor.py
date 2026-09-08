"""Detect conflicting Twist values, not merely registered ROS publishers.

Humble's Python executor does not pass publisher identity to callbacks. Match
our transmitted values against their queued echoes; equal values are compatible
regardless of source. This is conflict observation, not DDS ownership arbitration.
"""
from collections import deque


class CommandConflictMonitor:
    def __init__(self):
        self.expected = deque(maxlen=256)
        self.active = False
        self.conflict = None

    @staticmethod
    def same(a,b):
        return all(abs(x-y)<1e-8 for x,y in zip(a,b))

    def sent(self,drive):
        self.expected.append(tuple(drive))

    def received(self,drive):
        drive=tuple(drive)
        for index,expected in enumerate(self.expected):
            if self.same(drive,expected):
                del self.expected[index]
                return
        if self.active or any(abs(v)>1e-8 for v in drive):
            self.conflict=drive
        elif not self.active:
            self.conflict=None

    def check(self):
        if self.conflict is not None:
            raise InterruptedError(f'conflicting_received_cmd_vel: {self.conflict}')
