"""Exercise production shutdown methods without ROS or motor connections."""
import ast
import signal
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("drive_fails", [False, True])
def test_stop_before_context_shutdown(sig, drive_fails):
    tree = ast.parse((Path(__file__).parents[1]/"auto_dock/auto_dock_node.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "AutoDockNode")
    destroy = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "destroy_node")
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    events, handlers = [], {}
    live = [False]

    def init(**kwargs):
        assert kwargs["signal_handler_options"] == "NO"
        live[0] = True

    def shutdown():
        events.append("context_shutdown")
        live[0] = False

    def install(signum, handler):
        previous = handlers.get(signum, "previous")
        handlers[signum] = handler
        return previous

    def drive(repeats):
        assert live[0] and repeats == 1
        events.append("zero")
        if drive_fails:
            raise RuntimeError("publisher failed")

    class Base:
        def __init__(self):
            self.stop_drive = drive
            self.fork_pub = SimpleNamespace(publish=lambda msg: events.append(msg.data))
            self.control_socket = SimpleNamespace(close=lambda: events.append("socket_close"))

        def get_logger(self):
            return SimpleNamespace(error=lambda message: events.append("error"))

        def destroy_node(self):
            events.append("node_destroy")

    def sleep(seconds):
        assert live[0]
        # A second termination signal must not interrupt the stop burst.
        handlers[sig](sig, None)

    ns = dict(Base=Base, String=SimpleNamespace, time=SimpleNamespace(sleep=sleep),
              signal=SimpleNamespace(SIGINT=signal.SIGINT, SIGTERM=signal.SIGTERM, signal=install),
              SignalHandlerOptions=SimpleNamespace(NO="NO"), ExternalShutdownException=RuntimeError,
              rclpy=SimpleNamespace(init=init, ok=lambda: live[0], shutdown=shutdown,
                  spin_once=lambda node, timeout_sec: handlers[sig](sig, None)))
    cls.bases = [ast.Name(id="Base", ctx=ast.Load())]
    cls.body = [destroy]
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls, main], type_ignores=[])), "shutdown", "exec"), ns)
    ns["main"]()
    assert events.count("zero") == 10
    assert events.count("STOP") == 10
    assert events[-3:] == ["socket_close", "node_destroy", "context_shutdown"]
    assert all(handler == "previous" for handler in handlers.values())
