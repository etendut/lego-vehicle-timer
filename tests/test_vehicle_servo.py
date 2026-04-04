from unittest.mock import MagicMock, patch

from pytest_check import check

import modules.vehicle_servo as vs
from modules.vehicle_servo import RunServoSteerMotors
from pybricks.parameters import Button
from pybricks.pupdevices import Motor

SPEED = 80
ANGLE = 45


def make_helper() -> tuple[RunServoSteerMotors, MagicMock, MagicMock]:
    # run_until_stalled returns None without hardware, which breaks calibrate_steering math
    with patch.object(Motor, 'run_until_stalled', return_value=100):
        helper = RunServoSteerMotors(MagicMock(), drive_speed=SPEED, turn_angle=ANGLE,
                                     reverse_drive_motor=False, reverse_steering_motor=False)
    drive: MagicMock = MagicMock()
    steer: MagicMock = MagicMock()
    helper.drive_motor = drive  # type: ignore[assignment]
    helper.steering_motor = steer  # type: ignore[assignment]
    return helper, drive, steer


def press(helper: RunServoSteerMotors, buttons: list, monkeypatch):
    mock_remote = MagicMock()
    mock_remote.buttons.pressed.return_value = buttons
    monkeypatch.setattr(vs, 'remote', mock_remote)
    helper.handle_remote_press()


# --- drive motor ---

def test_left_plus_drives_forward(monkeypatch):
    helper, drive, _ = make_helper()
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    drive.dc.assert_called_with(SPEED)


def test_left_minus_drives_reverse(monkeypatch):
    helper, drive, _ = make_helper()
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    drive.dc.assert_called_with(-SPEED)


def test_no_drive_button_stops_drive_motor(monkeypatch):
    helper, drive, _ = make_helper()
    press(helper, [Button.RIGHT_PLUS], monkeypatch)  # only steer, no drive
    drive.dc.assert_called_with(0)


# --- steering motor ---

def test_right_plus_steers_right(monkeypatch):
    helper, _, steer = make_helper()
    press(helper, [Button.RIGHT_PLUS], monkeypatch)
    steer.run_target.assert_called_with(200, ANGLE, wait=False)


def test_right_minus_steers_left(monkeypatch):
    helper, _, steer = make_helper()
    press(helper, [Button.RIGHT_MINUS], monkeypatch)
    steer.run_target.assert_called_with(200, -ANGLE, wait=False)


def test_no_steer_button_centers_steering(monkeypatch):
    helper, _, steer = make_helper()
    press(helper, [Button.LEFT_PLUS], monkeypatch)  # only drive, no steer
    steer.run_target.assert_called_with(200, 0, wait=False)


# --- stop ---

def test_no_buttons_stops(monkeypatch):
    helper, drive, steer = make_helper()
    press(helper, [], monkeypatch)
    drive.dc.assert_called_with(0)
    steer.run_target.assert_called_with(200, 0)


def test_center_button_stops(monkeypatch):
    helper, drive, steer = make_helper()
    press(helper, [Button.LEFT], monkeypatch)
    drive.dc.assert_called_with(0)
    steer.run_target.assert_called_with(200, 0)


# --- combined drive and steer ---

def test_drive_and_steer_together(monkeypatch):
    helper, drive, steer = make_helper()
    press(helper, [Button.LEFT_PLUS, Button.RIGHT_PLUS], monkeypatch)
    drive.dc.assert_called_with(SPEED)
    steer.run_target.assert_called_with(200, ANGLE, wait=False)
