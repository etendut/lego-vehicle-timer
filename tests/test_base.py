"""Tests for modules/lego_vehicle_timer_base.py"""
import pytest
from unittest.mock import MagicMock
from pybricks.parameters import Color, Button

import modules.lego_vehicle_timer_base as base
from modules.lego_vehicle_timer_base import (
    ErrorFlashCodes,
    MotorHelper,
    CountdownTimer,
    convert_millis_hours_minutes_seconds,
    code_to_button_press_hash,
    wait_for_no_pressed_buttons,
    hub_battery_ok,
    _READY, _ACTIVE, _FINAL_MINUTE, _FINAL_20_SECS, _ENDED, _UNKNOWN,
)

# ── shared fixtures ────────────────────────────────────────────────────────────


@pytest.fixture()
def mock_hub():
    return MagicMock()


@pytest.fixture()
def mock_remote():
    return MagicMock()


@pytest.fixture(autouse=True)
def patch_globals(monkeypatch, mock_hub, mock_remote):
    """Inject mock hub/remote and reset _REMOTE_DISABLED for every test."""
    monkeypatch.setattr(base, 'hub', mock_hub, raising=False)
    monkeypatch.setattr(base, 'remote', mock_remote)
    monkeypatch.setattr(base, '_REMOTE_DISABLED', False)


def make_countdown() -> tuple['CountdownTimer', 'MagicMock']:
    """Return a CountdownTimer paired with its mock stopwatch (time() → 0)."""
    ct = CountdownTimer()
    sw = MagicMock()
    sw.time.return_value = 0
    ct.stopwatch = sw  # type: ignore[assignment]
    return ct, sw


# ── ErrorFlashCodes ────────────────────────────────────────────────────────────


class TestErrorFlashCodes:
    def test_default_flash_count(self):
        assert ErrorFlashCodes().flash_count == 1

    def test_set_error_no_motor_on_a(self):
        e = ErrorFlashCodes()
        e.set_error_no_motor_on_a()
        assert e.flash_count == 2

    def test_set_error_no_motor_on_b(self):
        e = ErrorFlashCodes()
        e.set_error_no_motor_on_b()
        assert e.flash_count == 3

    def test_set_error_no_remote(self):
        e = ErrorFlashCodes()
        e.set_error_no_remote()
        assert e.flash_count == 4

    def test_set_error_low_battery(self):
        e = ErrorFlashCodes()
        e.set_error_low_battery()
        assert e.flash_count == 5

    def test_flash_error_code_flashes_hub_and_remote(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        e = ErrorFlashCodes()
        e.flash_count = 2
        e.flash_error_code()
        # 2 flashes × (on + off) = 4 hub calls, 4 remote calls
        assert mock_hub.light.on.call_count == 4
        assert mock_remote.light.on.call_count == 4

    def test_flash_error_code_skips_remote_when_disabled(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        e = ErrorFlashCodes()
        e.flash_count = 2
        e.flash_error_code()
        assert mock_hub.light.on.call_count == 4
        mock_remote.light.on.assert_not_called()

    def test_flash_error_code_count_1_no_exception(self, monkeypatch, mock_hub):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        e = ErrorFlashCodes()
        e.flash_count = 1
        e.flash_error_code()  # flash_count == 1, so no extra wait(2000)


# ── MotorHelper ────────────────────────────────────────────────────────────────


class TestMotorHelper:
    def _make(self) -> MotorHelper:
        return MotorHelper(False, False)

    def test_init_stores_flags(self):
        mh = MotorHelper(True, True)
        assert mh.mh_supports_flip is True
        assert mh.mh_supports_homing is True
        assert mh.mh__remote_disabled is False
        assert mh.mh_auto_drive is False
        assert mh.mh_is_homed is False

    def test_stub_methods_do_not_raise(self):
        mh = self._make()
        mh.handle_flip()
        mh.home_and_unload()
        mh.reset_homing()
        mh.auto_unload()
        mh.auto_load()
        mh.handle_remote_press()
        mh.stop_motors()
        mh.reset_idle_timeout()

    def test_idle_timed_out_returns_false(self):
        assert self._make().idle_timed_out() is False

    def test_enable_auto_drive_sets_flag(self):
        mh = self._make()
        mh.enable_auto_drive()
        assert mh.mh_auto_drive is True

    def test_enable_auto_drive_idempotent(self):
        mh = self._make()
        mh.mh_auto_drive = True
        mh.enable_auto_drive()  # hits the early-return guard
        assert mh.mh_auto_drive is True

    def test_disable_auto_drive_clears_flag(self):
        mh = self._make()
        mh.mh_auto_drive = True
        mh.disable_auto_drive()
        assert mh.mh_auto_drive is False

    def test_disable_auto_drive_idempotent(self):
        mh = self._make()
        mh.disable_auto_drive()  # already False → early-return guard
        assert mh.mh_auto_drive is False

    def test_set_is_homed_sets_flag(self):
        mh = self._make()
        mh.set_is_homed()
        assert mh.mh_is_homed is True

    def test_set_is_homed_idempotent(self):
        mh = self._make()
        mh.mh_is_homed = True
        mh.set_is_homed()  # early-return guard
        assert mh.mh_is_homed is True

    def test_reset_is_homed_clears_flag(self):
        mh = self._make()
        mh.mh_is_homed = True
        mh.reset_is_homed()
        assert mh.mh_is_homed is False

    def test_reset_is_homed_idempotent(self):
        mh = self._make()
        mh.reset_is_homed()  # already False → early-return guard
        assert mh.mh_is_homed is False


# ── utility functions ──────────────────────────────────────────────────────────


class TestConvertMillis:
    def test_zero(self):
        assert convert_millis_hours_minutes_seconds(0) == (0, 0, 0)

    def test_one_minute(self):
        assert convert_millis_hours_minutes_seconds(60_000) == (0, 1, 0)

    def test_one_hour(self):
        assert convert_millis_hours_minutes_seconds(3_600_000) == (1, 0, 0)

    def test_mixed(self):
        assert convert_millis_hours_minutes_seconds(3_661_000) == (1, 1, 1)


class TestCodeToButtonPressHash:
    def test_all_center(self):
        pressed, not_pressed = code_to_button_press_hash('c,c,c')
        assert Button.LEFT in pressed
        assert Button.CENTER in pressed
        assert Button.RIGHT in pressed
        assert Button.LEFT_PLUS in not_pressed
        assert Button.RIGHT_PLUS in not_pressed

    def test_plus_codes(self):
        pressed, _np = code_to_button_press_hash('+,c,+')
        assert Button.LEFT_PLUS in pressed
        assert Button.CENTER in pressed
        assert Button.RIGHT_PLUS in pressed
        assert Button.LEFT not in pressed

    def test_minus_codes(self):
        pressed, _np = code_to_button_press_hash('-,c,-')
        assert Button.LEFT_MINUS in pressed
        assert Button.CENTER in pressed
        assert Button.RIGHT_MINUS in pressed

    def test_mixed_codes(self):
        pressed, _np = code_to_button_press_hash('+,-,c')
        assert Button.LEFT_PLUS in pressed
        assert Button.RIGHT in pressed
        assert Button.LEFT_MINUS not in pressed


class TestWaitForNoPressedButtons:
    def test_remote_disabled_returns_immediately(self, monkeypatch, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        wait_for_no_pressed_buttons()
        mock_remote.buttons.pressed.assert_not_called()

    def test_no_buttons_returns_immediately(self, monkeypatch, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        mock_remote.buttons.pressed.return_value = []
        wait_for_no_pressed_buttons()
        mock_remote.buttons.pressed.assert_called_once()

    def test_loops_until_buttons_released(self, monkeypatch, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        mock_remote.buttons.pressed.side_effect = [
            [Button.LEFT], [Button.LEFT], []
        ]
        wait_for_no_pressed_buttons()
        assert mock_remote.buttons.pressed.call_count == 3


# ── CountdownTimer ─────────────────────────────────────────────────────────────


class TestCountdownTimer:
    def test_init_default_state(self):
        ct, _ = make_countdown()
        assert ct.countdown_status == _UNKNOWN
        assert ct.end_time == 0
        assert ct.last_hub_remote_color is None

    # has_time_remaining
    def test_has_time_remaining_ready_false(self):
        ct, _ = make_countdown()
        ct.countdown_status = _READY
        assert ct.has_time_remaining() is False

    def test_has_time_remaining_ended_false(self):
        ct, _ = make_countdown()
        ct.countdown_status = _ENDED
        assert ct.has_time_remaining() is False

    def test_has_time_remaining_active_plenty_of_time(self, mock_hub):
        ct, sw = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.end_time = 120_000  # 2 minutes
        sw.time.return_value = 0
        assert ct.has_time_remaining() is True
        assert ct.countdown_status == _ACTIVE

    def test_has_time_remaining_active_final_minute(self, mock_hub):
        ct, sw = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.end_time = 50_000   # 50 s → final minute
        sw.time.return_value = 0
        assert ct.has_time_remaining() is True
        assert ct.countdown_status == _FINAL_MINUTE

    def test_has_time_remaining_active_final_20_secs(self, mock_hub):
        ct, sw = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.end_time = 15_000   # 15 s → final 20 secs
        sw.time.return_value = 0
        assert ct.has_time_remaining() is True
        assert ct.countdown_status == _FINAL_20_SECS

    def test_has_time_remaining_time_expired(self, mock_hub):
        ct, sw = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.end_time = 0
        sw.time.return_value = 0  # remaining = 0 → expired
        assert ct.has_time_remaining() is False
        assert ct.countdown_status == _ENDED

    # __start_countdown__
    def test_start_countdown_sets_active_and_end_time(self, mock_hub):
        ct, sw = make_countdown()
        sw.time.return_value = 1_000
        ct.__start_countdown__()
        assert ct.countdown_status == _ACTIVE
        assert ct.end_time == 1_000 + (3 * 60 * 1_000)

    # reset
    def test_reset_sets_ready_with_remote(self, monkeypatch):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        ct, _ = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.reset()
        assert ct.countdown_status == _READY

    def test_reset_sets_ready_without_remote(self, monkeypatch):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        ct, _ = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.reset()
        assert ct.countdown_status == _READY

    # should_check_battery
    def test_should_check_battery_first_call_false(self):
        ct, sw = make_countdown()
        sw.time.return_value = 0
        assert ct.should_check_battery() is False

    def test_should_check_battery_after_5s_true(self):
        ct, sw = make_countdown()
        ct._battery_check_time = 0
        sw.time.return_value = 5_001
        assert ct.should_check_battery() is True

    def test_should_check_battery_throttles(self):
        ct, sw = make_countdown()
        ct._battery_check_time = 0
        sw.time.return_value = 5_001
        ct.should_check_battery()          # True, advances _battery_check_time
        sw.time.return_value = 6_000
        assert ct.should_check_battery() is False  # still within 5 s window

    # show_status — all five states
    def test_show_status_ready_no_exception(self, mock_hub):
        ct, _ = make_countdown()
        ct.countdown_status = _READY
        ct.show_status()  # calls flash_hub_and_remote_light (no-op at t=0)

    def test_show_status_active_lights_hub_green(self, mock_hub):
        ct, _ = make_countdown()
        ct.countdown_status = _ACTIVE
        ct.show_status()
        mock_hub.light.on.assert_called_with(Color.GREEN)

    def test_show_status_final_20_secs_no_exception(self, mock_hub):
        ct, sw = make_countdown()
        ct.countdown_status = _FINAL_20_SECS
        sw.time.return_value = 150  # in ON phase (past off_msec=100)
        ct.show_status()

    def test_show_status_final_minute_no_exception(self, mock_hub):
        ct, _ = make_countdown()
        ct.countdown_status = _FINAL_MINUTE
        ct.show_status()

    def test_show_status_ended_lights_hub_orange(self, mock_hub):
        ct, _ = make_countdown()
        ct.countdown_status = _ENDED
        ct.show_status()
        mock_hub.light.on.assert_called_with(Color.ORANGE)

    # set_hub_and_remote_light
    def test_set_light_same_color_noop(self, mock_hub):
        # last_hub_remote_color starts as None; passing None again triggers
        # the early-return guard (None == None is True in Python).
        # Color.__eq__ returns None in the pybricks stub, so we can't test
        # the guard with real Color values — use None as the sentinel.
        ct, _ = make_countdown()
        ct.set_hub_and_remote_light(None, False)  # type: ignore[arg-type]
        mock_hub.light.on.assert_not_called()

    def test_set_light_new_color_include_remote_false(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        ct, _ = make_countdown()
        ct.set_hub_and_remote_light(Color.GREEN, False)
        mock_hub.light.on.assert_called_with(Color.GREEN)
        mock_remote.light.on.assert_not_called()

    def test_set_light_include_remote_true(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        ct, _ = make_countdown()
        ct.set_hub_and_remote_light(Color.GREEN, True)
        mock_hub.light.on.assert_called_with(Color.GREEN)
        mock_remote.light.on.assert_called_with(Color.GREEN)

    def test_set_light_include_remote_but_remote_disabled(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        ct, _ = make_countdown()
        ct.set_hub_and_remote_light(Color.GREEN, True)
        mock_hub.light.on.assert_called_with(Color.GREEN)
        mock_remote.light.on.assert_not_called()

    # flash_hub_and_remote_light
    def test_flash_before_off_phase_noop(self, mock_hub):
        ct, sw = make_countdown()
        sw.time.return_value = 0
        ct.led_flash_sw_time = 0
        ct.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, False)
        mock_hub.light.on.assert_not_called()  # 0 > 500 is False

    def test_flash_during_on_phase(self, mock_hub):
        ct, sw = make_countdown()
        sw.time.return_value = 600  # past off_msec=500, before full cycle (1000)
        ct.led_flash_sw_time = 0
        ct.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, False)
        mock_hub.light.on.assert_called_with(Color.GREEN)

    def test_flash_cycle_reset_shows_off_colour(self, mock_hub):
        ct, sw = make_countdown()
        sw.time.return_value = 1_100  # past full cycle (1000) → resets timer
        ct.led_flash_sw_time = 0
        ct.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, False)
        mock_hub.light.on.assert_called_with(Color.NONE)

    # check_remote_buttons
    def test_check_remote_buttons_disabled_noop(self, monkeypatch, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        ct, _ = make_countdown()
        ct.check_remote_buttons()
        mock_remote.buttons.pressed.assert_not_called()

    def test_check_remote_buttons_no_buttons_noop(self, monkeypatch, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        mock_remote.buttons.pressed.return_value = []
        ct, _ = make_countdown()
        ct.check_remote_buttons()

    def test_check_remote_buttons_center_starts_countdown(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        # First call: CENTER → start countdown.
        # Second call (inside wait_for_no_pressed_buttons): nothing pressed.
        mock_remote.buttons.pressed.side_effect = [[Button.CENTER], []]
        ct, _ = make_countdown()
        ct.countdown_status = _READY
        ct.check_remote_buttons()
        assert ct.countdown_status == _ACTIVE

    def test_check_remote_buttons_reset_code_resets_timer(self, monkeypatch, mock_hub, mock_remote):
        monkeypatch.setattr(base, '_REMOTE_DISABLED', False)
        # PROGRAM_RESET_CODE_PRESSED for default 'c,c,c' is [LEFT, CENTER, RIGHT]
        reset_buttons = [Button.LEFT, Button.CENTER, Button.RIGHT]
        mock_remote.buttons.pressed.side_effect = [reset_buttons, []]
        ct, _ = make_countdown()
        ct.countdown_status = _ACTIVE  # not _READY → center check won't start a new countdown
        ct.check_remote_buttons()
        assert ct.countdown_status == _READY


# ── hub_battery_ok ─────────────────────────────────────────────────────────────


class TestHubBatteryOk:
    def test_above_critical_returns_true(self, mock_hub):
        mock_hub.battery.voltage.return_value = 9_000
        assert hub_battery_ok() is True

    def test_below_critical_returns_false(self, mock_hub):
        mock_hub.battery.voltage.return_value = 8_000
        assert hub_battery_ok() is False

    def test_exactly_at_critical_returns_false(self, mock_hub):
        mock_hub.battery.voltage.return_value = 8_400
        assert hub_battery_ok() is False


# ── setup_hub ──────────────────────────────────────────────────────────────────


class TestSetupHub:
    def test_setup_hub_uses_city_hub_stub(self):
        """CityHub is available in the stub environment — setup_hub() should succeed."""
        base.setup_hub()
        assert base.hub is not None

    def test_setup_hub_returns_false_for_city_hub(self):
        result = base.setup_hub()
        assert result is False


# ── setup_remote ───────────────────────────────────────────────────────────────


class TestSetupRemote:
    def test_connects_on_first_try(self, monkeypatch, mock_hub):
        mock_remote_instance = MagicMock()
        monkeypatch.setattr(base, 'Remote', lambda: mock_remote_instance)
        err = MagicMock()
        base.setup_remote(err)
        assert base.remote is mock_remote_instance
        err.set_error_no_remote.assert_not_called()

    def test_connects_on_second_try(self, monkeypatch, mock_hub):
        call_count = [0]
        mock_remote_instance = MagicMock()

        def sometimes_remote():
            call_count[0] += 1
            if call_count[0] < 2:
                raise Exception('retry')
            return mock_remote_instance

        monkeypatch.setattr(base, 'Remote', sometimes_remote)
        err = MagicMock()
        base.setup_remote(err, retry=5)
        assert base.remote is mock_remote_instance

    def test_exhausts_retries_and_raises(self, monkeypatch, mock_hub):
        call_count = [0]

        def failing_remote():
            call_count[0] += 1
            raise Exception('no remote')

        monkeypatch.setattr(base, 'Remote', failing_remote)
        err = MagicMock()
        with pytest.raises(Exception):
            base.setup_remote(err, retry=3)
        assert call_count[0] == 3
        err.set_error_no_remote.assert_called_once()


# ── main() ─────────────────────────────────────────────────────────────────────


class TestMain:
    def test_remote_disabled_no_homing_exits_after_one_loop(self, monkeypatch):
        """
        With _REMOTE_DISABLED=True and MotorHelper(supports_homing=False),
        main() should run exactly one loop iteration and then raise SystemExit.
        """
        from pybricks.tools import StopWatch

        mock_hub = MagicMock()
        mock_hub.battery.voltage.return_value = 9_000

        def fake_setup_hub():
            base.hub = mock_hub

        monkeypatch.setattr(base, 'setup_hub', fake_setup_hub)
        monkeypatch.setattr(base, '_REMOTE_DISABLED', True)
        # StopWatch.time() returns None in the pybricks stub; patch to 0
        # so CountdownTimer arithmetic doesn't fail.
        monkeypatch.setattr(StopWatch, 'time', lambda self: 0)

        with pytest.raises(SystemExit):
            base.main()
