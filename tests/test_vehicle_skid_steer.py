from unittest.mock import MagicMock, patch

from pytest_check import check

import modules.vehicle_skid_steer as ss
from modules.vehicle_skid_steer import RunSkidSteerMotors
from pybricks.parameters import Button, Side

SPEED = 80


def make_helper(swap=False, reverse_left=False, reverse_right=False) -> tuple[RunSkidSteerMotors, MagicMock, MagicMock]:
    helper = RunSkidSteerMotors(MagicMock(), drive_speed=SPEED, swap_motor_sides=swap,
                                reverse_left_motor=reverse_left, reverse_right_motor=reverse_right)
    left: MagicMock = MagicMock()
    right: MagicMock = MagicMock()
    helper.left_motor = left  # type: ignore[assignment]
    helper.right_motor = right  # type: ignore[assignment]
    return helper, left, right


def press(helper: RunSkidSteerMotors, buttons: list, monkeypatch):
    mock_remote = MagicMock()
    mock_remote.buttons.pressed.return_value = buttons
    monkeypatch.setattr(ss, 'remote', mock_remote)
    helper.handle_remote_press()


# --- button to motor mapping ---

def test_left_plus_drives_left_motor_forward(monkeypatch):
    helper, left, _ = make_helper()
    press(helper, [Button.LEFT_PLUS], monkeypatch)
    left.dc.assert_called_with(SPEED)


def test_left_minus_drives_left_motor_reverse(monkeypatch):
    helper, left, _ = make_helper()
    press(helper, [Button.LEFT_MINUS], monkeypatch)
    left.dc.assert_called_with(-SPEED)


def test_right_plus_drives_right_motor_forward(monkeypatch):
    helper, _, right = make_helper()
    press(helper, [Button.RIGHT_PLUS], monkeypatch)
    right.dc.assert_called_with(SPEED)


def test_right_minus_drives_right_motor_reverse(monkeypatch):
    helper, _, right = make_helper()
    press(helper, [Button.RIGHT_MINUS], monkeypatch)
    right.dc.assert_called_with(-SPEED)


def test_no_buttons_stops_both_motors(monkeypatch):
    helper, left, right = make_helper()
    press(helper, [], monkeypatch)
    left.dc.assert_called_with(0)
    right.dc.assert_called_with(0)


def test_center_button_stops_both_motors(monkeypatch):
    helper, left, right = make_helper()
    press(helper, [Button.LEFT], monkeypatch)
    left.dc.assert_called_with(0)
    right.dc.assert_called_with(0)


# --- flip detection ---

def test_flip_top_up_assigns_normal_motor_ports(monkeypatch):
    helper, _, _ = make_helper()
    mock_hub = MagicMock()
    mock_hub.imu.up.return_value = Side.TOP
    monkeypatch.setattr(ss, 'hub', mock_hub)

    with patch('modules.vehicle_skid_steer.DCMotor') as mock_dcmotor:
        first_call: MagicMock = MagicMock()
        second_call: MagicMock = MagicMock()
        mock_dcmotor.side_effect = [first_call, second_call]
        helper.handle_flip()

    # TOP: right_motor created first, left_motor second (normal assignment)
    check.equal(helper.right_motor, first_call)
    check.equal(helper.left_motor, second_call)


def test_flip_bottom_up_swaps_motor_ports(monkeypatch):
    helper, _, _ = make_helper()
    mock_hub = MagicMock()
    mock_hub.imu.up.return_value = Side.BOTTOM
    monkeypatch.setattr(ss, 'hub', mock_hub)

    with patch('modules.vehicle_skid_steer.DCMotor') as mock_dcmotor:
        first_call: MagicMock = MagicMock()
        second_call: MagicMock = MagicMock()
        mock_dcmotor.side_effect = [first_call, second_call]
        helper.handle_flip()

    # BOTTOM: right_motor uses left_motor_port, left_motor uses right_motor_port
    check.equal(helper.right_motor, first_call)
    check.equal(helper.left_motor, second_call)


def test_flip_same_side_twice_does_nothing(monkeypatch):
    helper, left, right = make_helper()
    mock_hub = MagicMock()
    mock_hub.imu.up.return_value = Side.TOP
    monkeypatch.setattr(ss, 'hub', mock_hub)
    helper.last_side = Side.TOP  # already on TOP

    helper.handle_flip()

    check.equal(helper.left_motor, left)
    check.equal(helper.right_motor, right)
