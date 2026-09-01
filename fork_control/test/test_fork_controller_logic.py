from fork_control.fork_controller import ForkController


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeSwitch:
    def __init__(self, pressed=False):
        self.is_pressed = pressed


class FakeMotor:
    def __init__(self):
        self.on_forward = None
        self.stopped = False

    def forward(self):
        if self.on_forward is not None:
            self.on_forward()

    def backward(self):
        pass

    def stop(self):
        self.stopped = True


def fork_fake():
    fake = type("FakeFork", (), {})()
    fake.motor = FakeMotor()
    fake.lower_limit_switch = FakeSwitch()
    fake.upper_limit_switch = FakeSwitch()
    fake.lower_limit_latched = False
    fake.upper_limit_latched = False
    fake.lower_release_started_at = None
    fake.upper_release_started_at = None
    fake.active_command = "STOP"
    fake.up_started_at = None
    fake.runtime_config = {}
    fake.refresh_runtime_config = lambda: None
    fake.runtime_boolean = lambda key, default=False: ForkController.runtime_boolean(
        fake, key, default
    )
    fake.runtime_number = lambda key, default, minimum, maximum: (
        ForkController.runtime_number(fake, key, default, minimum, maximum)
    )
    fake.get_logger = lambda: FakeLogger()
    fake.states = []
    fake.publish_state = lambda state, error="": fake.states.append((state, error))
    fake.complete = lambda command: ForkController.complete(fake, command)
    fake.upper_limit_pressed = lambda: ForkController.upper_limit_pressed(fake)
    return fake


def test_immediate_upper_limit_still_publishes_up_complete():
    fake = fork_fake()
    fake.motor.on_forward = fake.upper_limit_pressed

    ForkController.start_command(fake, "UP")

    assert fake.motor.stopped is True
    assert fake.active_command == "STOP"
    assert fake.states == [("UP_COMPLETE", "")]


def test_limit_polling_backs_up_missed_gpio_edge(monkeypatch):
    fake = fork_fake()
    fake.active_command = "UP"
    fake.upper_limit_switch.is_pressed = True
    fake.update_limit_latch = lambda *args: None
    monkeypatch.setattr("fork_control.fork_controller.time.monotonic", lambda: 10.0)

    ForkController.update_limit_latches(fake)

    assert fake.motor.stopped is True
    assert fake.states == [("UP_COMPLETE", "")]


def test_timed_up_fallback_stops_and_publishes_complete_after_three_seconds(
    monkeypatch,
):
    fake = fork_fake()
    fake.active_command = "UP"
    fake.up_started_at = 10.0
    fake.runtime_config = {
        "fork_timed_up_complete_enabled": True,
        "fork_timed_up_complete_sec": 3.0,
    }
    fake.update_limit_latch = lambda *args: None
    monkeypatch.setattr("fork_control.fork_controller.time.monotonic", lambda: 13.0)

    ForkController.update_limit_latches(fake)

    assert fake.motor.stopped is True
    assert fake.active_command == "STOP"
    assert fake.states == [("UP_COMPLETE", "")]


def test_timed_up_fallback_can_be_disabled(monkeypatch):
    fake = fork_fake()
    fake.active_command = "UP"
    fake.up_started_at = 10.0
    fake.runtime_config = {
        "fork_timed_up_complete_enabled": False,
        "fork_timed_up_complete_sec": 3.0,
    }
    fake.update_limit_latch = lambda *args: None
    monkeypatch.setattr("fork_control.fork_controller.time.monotonic", lambda: 20.0)

    ForkController.update_limit_latches(fake)

    assert fake.active_command == "UP"
    assert fake.states == []
