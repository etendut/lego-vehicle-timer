import pytest
from unittest.mock import MagicMock

from pytest_check import check

import modules.vehicle_train as vt
from modules.vehicle_train import RunTrainMotor
from pybricks.parameters import Button, Direction

MIN_SPEED = 30
MAX_SPEED = 80
STEP = 10


def make_helper() -> tuple[RunTrainMotor, MagicMock]:
    helper = RunTrainMotor(MagicMock(), min_speed=MIN_SPEED, max_speed=MAX_SPEED,
                           speed_step=STEP, reverse_motor=False, reverse_motor_2=False)
    motor: MagicMock = MagicMock()
    helper.train_motor_port_a = motor  # type: ignore[assignment]
    helper.train_motor_port_b = None
    return helper, motor


def press(helper: RunTrainMotor, buttons: list, monkeypatch):
    mock_remote = MagicMock()
    mock_remote.buttons.pressed.return_value = buttons
    monkeypatch.setattr(vt, 'remote', mock_remote)
    helper.handle_remote_press()


# --- speed logic ---

def test_plus_from_stopped_starts_at_min_speed(monkeypatch):
    helper, motor = make_helper()
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    check.equal(helper.current_motor_speed, MIN_SPEED)
    motor.dc.assert_called_with(MIN_SPEED)


def test_minus_from_stopped_starts_at_min_speed_reverse(monkeypatch):
    helper, motor = make_helper()
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    check.equal(helper.current_motor_speed, -MIN_SPEED)
    motor.dc.assert_called_with(-MIN_SPEED)


def test_plus_increments_from_running(monkeypatch):
    helper, _ = make_helper()
    helper.current_motor_speed = MIN_SPEED
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    check.equal(helper.current_motor_speed, MIN_SPEED + STEP)


def test_minus_decrements_from_running(monkeypatch):
    helper, _ = make_helper()
    helper.current_motor_speed = -MIN_SPEED
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    check.equal(helper.current_motor_speed, -MIN_SPEED - STEP)


def test_plus_clamps_at_max_speed(monkeypatch):
    helper, _ = make_helper()
    helper.current_motor_speed = MAX_SPEED
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    check.equal(helper.current_motor_speed, MAX_SPEED)


def test_minus_clamps_at_max_reverse_speed(monkeypatch):
    helper, _ = make_helper()
    helper.current_motor_speed = -MAX_SPEED
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    check.equal(helper.current_motor_speed, -MAX_SPEED)


def test_minus_snaps_to_zero_when_between_zero_and_min(monkeypatch):
    # e.g. speed=25, step=10 → 15 → snaps to 0 (still above 0 but below min)
    helper, _ = make_helper()
    helper.current_motor_speed = MIN_SPEED - (STEP // 2)  # 25
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    check.equal(helper.current_motor_speed, 0)


def test_plus_snaps_to_zero_when_between_zero_and_min_reverse(monkeypatch):
    helper, _ = make_helper()
    helper.current_motor_speed = -(MIN_SPEED - (STEP // 2))  # -25
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    check.equal(helper.current_motor_speed, 0)


# --- stop conditions ---

def test_no_buttons_stops(monkeypatch):
    helper, motor = make_helper()
    helper.current_motor_speed = MIN_SPEED
    press(helper, [], monkeypatch)
    motor.dc.assert_called_with(0)


def test_left_center_button_stops(monkeypatch):
    helper, motor = make_helper()
    helper.current_motor_speed = MIN_SPEED
    press(helper, [Button.LEFT], monkeypatch)
    motor.dc.assert_called_with(0)


def test_right_center_button_stops(monkeypatch):
    helper, motor = make_helper()
    helper.current_motor_speed = MIN_SPEED
    press(helper, [Button.RIGHT], monkeypatch)
    motor.dc.assert_called_with(0)


# --- right remote mirrors left ---

def test_right_plus_increments_same_as_left_plus(monkeypatch):
    helper, _ = make_helper()
    press(helper, [Button.RIGHT_PLUS], monkeypatch)
    check.equal(helper.current_motor_speed, MIN_SPEED)


def test_right_minus_decrements_same_as_left_minus(monkeypatch):
    helper, _ = make_helper()
    press(helper, [Button.RIGHT_MINUS], monkeypatch)
    check.equal(helper.current_motor_speed, -MIN_SPEED)


# --- init paths ---

def test_init_reverse_motor_uses_counterclockwise(monkeypatch):
    """reverse_motor=True should initialise the motor COUNTERCLOCKWISE."""
    captured: list[Direction] = []

    def fake_dc_motor(port, direction):
        captured.append(direction)
        return MagicMock()

    monkeypatch.setattr(vt, 'DCMotor', fake_dc_motor)
    RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, reverse_motor=True, reverse_motor_2=False)
    check.equal(captured[0], Direction.COUNTERCLOCKWISE)


def test_init_port_a_fails_falls_back_to_port_b(monkeypatch):
    """When port A raises, the motor should be found on port B."""
    call_count = [0]
    mock_motor = MagicMock()

    def make_dc_motor(port, direction):
        call_count[0] += 1
        if call_count[0] == 1:
            raise Exception('no motor on A')
        return mock_motor

    monkeypatch.setattr(vt, 'DCMotor', make_dc_motor)
    helper = RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, False)
    check.is_none(helper.train_motor_port_a)
    check.equal(helper.train_motor_port_b, mock_motor)


def test_init_port_b_reverse_motor_2_uses_counterclockwise(monkeypatch):
    """When reverse_motor_2=True the port-B motor should be COUNTERCLOCKWISE."""
    captured: list[Direction] = []
    call_count = [0]

    def make_dc_motor(port, direction):
        call_count[0] += 1
        if call_count[0] == 1:
            raise Exception('no motor on A')
        captured.append(direction)
        return MagicMock()

    monkeypatch.setattr(vt, 'DCMotor', make_dc_motor)
    RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, reverse_motor_2=True)
    check.equal(captured[0], Direction.COUNTERCLOCKWISE)


def test_init_both_ports_fail_raises(monkeypatch):
    """When both ports raise the helper should set the error code and re-raise."""
    def raise_exc(port, direction):
        raise Exception('no motor')

    monkeypatch.setattr(vt, 'DCMotor', raise_exc)
    err = MagicMock()
    with pytest.raises(Exception, match='Train motor needs to be connected'):
        RunTrainMotor(err, MIN_SPEED, MAX_SPEED, STEP, False, False)
    err.set_error_no_motor_on_a.assert_called_once()


def test_init_light_found_on_port_a_when_motor_on_b(monkeypatch):
    """Motor on B → Light(Port.A) is attempted."""
    call_count = [0]

    def make_dc_motor(port, direction):
        call_count[0] += 1
        if call_count[0] == 1:
            raise Exception('no motor on A')
        return MagicMock()

    mock_light = MagicMock()
    monkeypatch.setattr(vt, 'DCMotor', make_dc_motor)
    monkeypatch.setattr(vt, 'Light', lambda port: mock_light)
    helper = RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, False)
    check.equal(helper.lights, mock_light)


def test_init_light_found_on_port_b_when_motor_on_a(monkeypatch):
    """Motor on A → Light(Port.B) is attempted."""
    monkeypatch.setattr(vt, 'DCMotor', lambda port, direction: MagicMock())
    mock_light = MagicMock()
    monkeypatch.setattr(vt, 'Light', lambda port: mock_light)
    helper = RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, False)
    check.equal(helper.lights, mock_light)


def test_init_light_failure_is_silently_ignored(monkeypatch):
    """If Light() raises, lights stays None — no exception propagates."""
    monkeypatch.setattr(vt, 'DCMotor', lambda port, direction: MagicMock())

    def raise_light(port):
        raise Exception('no light')

    monkeypatch.setattr(vt, 'Light', raise_light)
    helper = RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, False)
    check.is_none(helper.lights)


def test_init_light_failure_on_port_a_when_motor_on_b_is_silently_ignored(monkeypatch):
    """Motor on B → Light(Port.A) raises → lights stays None, no exception propagates."""
    call_count = [0]

    def make_dc_motor(port, direction):
        call_count[0] += 1
        if call_count[0] == 1:
            raise Exception('no motor on A')
        return MagicMock()

    monkeypatch.setattr(vt, 'DCMotor', make_dc_motor)
    monkeypatch.setattr(vt, 'Light', lambda port: (_ for _ in ()).throw(Exception('no light')))
    helper = RunTrainMotor(MagicMock(), MIN_SPEED, MAX_SPEED, STEP, False, False)
    check.is_none(helper.lights)


# --- remote disabled ---

def test_handle_remote_press_remote_disabled_returns_early(monkeypatch):
    """mh__remote_disabled=True must short-circuit without touching the remote."""
    helper, motor = make_helper()
    helper.mh__remote_disabled = True
    mock_remote = MagicMock()
    monkeypatch.setattr(vt, 'remote', mock_remote)
    helper.handle_remote_press()
    mock_remote.buttons.pressed.assert_not_called()
    motor.dc.assert_not_called()


# --- port B motor ---

def test_handle_remote_press_drives_port_b_motor(monkeypatch):
    """When port_b is set it should receive the same dc() call as port_a."""
    helper, motor_a = make_helper()
    motor_b = MagicMock()
    helper.train_motor_port_b = motor_b
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    motor_a.dc.assert_called_with(MIN_SPEED)
    motor_b.dc.assert_called_with(MIN_SPEED)


def test_stop_motors_stops_port_b(monkeypatch):
    """stop_motors() should call dc(0) on port_b when it exists."""
    helper, motor_a = make_helper()
    motor_b = MagicMock()
    helper.train_motor_port_b = motor_b
    helper.stop_motors()
    motor_a.dc.assert_called_with(0)
    motor_b.dc.assert_called_with(0)


# --- lights interaction ---

def test_stop_motors_turns_lights_off():
    helper, _ = make_helper()
    mock_lights = MagicMock()
    helper.lights = mock_lights
    helper.stop_motors()
    mock_lights.off.assert_called_once()


def test_handle_remote_press_turns_lights_on_when_moving(monkeypatch):
    helper, _ = make_helper()
    mock_lights = MagicMock()
    helper.lights = mock_lights
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    mock_lights.on.assert_called_with(100)


def test_handle_remote_press_turns_lights_off_when_stopped(monkeypatch):
    helper, _ = make_helper()
    mock_lights = MagicMock()
    helper.lights = mock_lights
    press(helper, [], monkeypatch)
    mock_lights.off.assert_called()
