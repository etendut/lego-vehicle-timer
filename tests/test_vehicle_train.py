from unittest.mock import MagicMock

from pytest_check import check

import modules.vehicle_train as vt
from modules.vehicle_train import RunTrainMotor
from pybricks.parameters import Button

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
