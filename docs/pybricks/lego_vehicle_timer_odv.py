# Timed Train, Servo Steer, Skid Steer, and ODV vehicle program for interactive displays
# Copyright Etendut
# https://etendut.github.io/lego-vehicle-timer/
# licence MIT
from micropython import const, mem_info
from pybricks.parameters import Color, Button
from pybricks.pupdevices import Remote
from pybricks.tools import wait, StopWatch

try:
    from typing import TYPE_CHECKING
except ImportError:
    TYPE_CHECKING = False

if TYPE_CHECKING:
    # noinspection PyUnusedImports
    from pybricks.hubs import CityHub, TechnicHub
    from pybricks.pupdevices import Remote

from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
try:
    from pybricks.tools import StopWatch, wait  # type: ignore[assignment]
except ImportError:
    class StopWatch:  # host-side stub for testing
        def time(self) -> int: return 0
        def reset(self) -> None: pass
    def wait(time: int) -> None: pass
try:
    from uerrno import ENODEV
except ImportError:
    ENODEV = -99


__BUILD__ = 'e5714d0-dirty @ 2026-05-23 14:57'  # replaced at compile time with git hash + timestamp
print('Version 3.0.0 build', __BUILD__)
##################################################################################
#  Settings
##################################################################################

# ── user configuration ────────────────────────────────────────────────────────
COUNTDOWN_LIMIT_MINUTES: int = const(3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
COUNTDOWN_RESET_CODE         = 'c,c,c'  # c = center button, + = + button, - = - button

# ── battery / voltage ─────────────────────────────────────────────────────────
# fresh battery = 1.6V; hub programming often fails below 1.5V;
# hub fails when batteries reach ~1.36V so critical level = 1.4V * 6 cells = 8400mV
MILLIVOLT_CRITICAL_LEVEL = const(8400)

_REMOTE_DISABLED = False # ODV overrides this from DRIVE_MODE; all other vehicles (servo/train/skid_steer) leave it False

# ── drive mode enum (do not change) ──────────────────────────────────────────
MANUAL = const(0)
HYBRID = const(1)
AUTO   = const(2)

# ── grid patterns ─────────────────────────────────────────────────────────────
ODV_GRID_DEFAULT = ["L#<#U", "X#<#X", "X###X"]
ODV_GRID_EX1     = ["###X#XX", "LX###XU", "###X###"]
ODV_GRID_EX2     = ["X###X", "L###U", "X###X"]
ODV_GRID_EX3     = ["X#>#X", "L#X#U", "X#<#X"]

# ── user configuration ────────────────────────────────────────────────────────
DRIVE_MODE        = MANUAL
_REMOTE_DISABLED   = (DRIVE_MODE == AUTO)  # AUTO runs headless; MANUAL/HYBRID require the remote
IDLE_TIMEOUT_SECS = const(20)  # HYBRID only: seconds idle before auto-drive engages
ODV_SPEED         = const(65)  # max speed in MANUAL and HYBRID modes
ODV_GRID          = ODV_GRID_DEFAULT

# ── debug / calibration ───────────────────────────────────────────────────────
DEBUG               = const(True)
_CALIBRATE_X_OFFSET = False  # halt auto_load at tile (3, 0) center for X-offset measurement

# ── internal tuning (change with caution) ────────────────────────────────────
_DEG_PER_TILE     = const(800)
_CART_SIZE_DEG    = const(640)

_LOOKAHEAD_DEG      = const(40)
_STOP_RAMP_MS       = const(200)
_BOTH_AXES_DUTY_NUM = const(71)
_BOTH_AXES_DUTY_DEN = const(100)
_AIM_SWITCH_DEG     = const(160)

_HOMING_MOTOR_ROT_SPEED = const(200)
_HOMING_DUTY            = const(45)
_MAX_MOTOR_ROT_SPEED    = const(1400)
# Auto-drive uses full duty — the controller knows what it's doing, no human in the loop.
_AUTO_DRIVE_DUTY    = const(80)
# Shorter ramp for auto-drive (100ms vs manual 200ms) keeps coast <80° wall clearance.
_AUTO_STOP_RAMP_MS  = const(100)
# Extra slack on predicted coast distance — absorbs motor non-linearity / battery sag.
_DEADBAND_SAFETY_DEG = const(10)
# Rig-measured: at east stall, physical cart center is 80° west of east_wall_deg
# (mechanical slack in the X drive — Y stall is clean to the wall, X is not).
_X_EAST_STALL_OFFSET_DEG = const(80)



##################################################################################
# ---------Main program below, editing should not be needed -------------

class ErrorFlashCodes:
    def __init__(self):
        self.flash_count = 1  # Other errors

    def set_error_no_motor_on_a(self):
        print('ERROR: NO MOTOR ON A')
        self.flash_count = 2

    def set_error_no_motor_on_b(self):
        print('ERROR: NO MOTOR ON B')
        self.flash_count = 3

    def set_error_no_remote(self):
        print('ERROR: NO REMOTE')
        self.flash_count = 4

    def set_error_low_battery(self):
        print('ERROR: LOW BATTERY')
        self.flash_count = 5

    def flash_error_code(self):
        """
            this flashes the remote led
        """

        global hub
        global remote

        for f in range(self.flash_count):
            hub.light.on(Color.RED)
            if not _REMOTE_DISABLED:
                remote.light.on(Color.RED)
            wait(350)
            hub.light.on(Color.NONE)
            if not _REMOTE_DISABLED:
                remote.light.on(Color.NONE)
            wait(350)
        if self.flash_count > 1:
            wait(2000)


class MotorHelper:

    def __init__(self, supports_flip: bool, supports_homing: bool):
        self.mh_supports_flip = supports_flip
        self.mh_supports_homing = supports_homing
        self.mh__remote_disabled = False
        self.mh_auto_drive = False
        self.mh_is_homed = False

    def handle_flip(self):
        """Tracked racer only"""
        pass

    def home_and_unload(self):
        """ODV only"""
        pass

    def reset_homing(self):
        """ODV only"""
        pass

    def auto_unload(self):
        """ODV only"""
        pass

    def auto_load(self):
        """ODV only"""
        pass

    def enable_auto_drive(self):
        """enable mh_auto_drive, ODV only"""
        if self.mh_auto_drive:
            return
        print("Enable Auto-Drive")
        self.mh_auto_drive = True

    def disable_auto_drive(self):
        """Disables mh_auto_drive, ODV only"""
        if not self.mh_auto_drive:
            return
        print("Disable Auto-Drive")
        self.mh_auto_drive = False

    def set_is_homed(self):
        """Set mh_is_homed, ODV only"""
        if self.mh_is_homed:
            return
        print("Set IsHomed")
        self.mh_is_homed = True

    def reset_is_homed(self):
        """Reset mh_is_homed, ODV only"""
        if not self.mh_is_homed:
            return
        print("Reset IsHomed")
        self.mh_is_homed = False

    def handle_remote_press(self):
        """All vehicles"""
        pass

    def stop_motors(self):
        """All vehicles"""
        pass

    def idle_timed_out(self) -> bool:
        """ODV only — returns True when auto-drive should engage."""
        return False

    def reset_idle_timeout(self):
        """ODV only — call after any activity that should delay auto-drive."""
        pass


##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    if _REMOTE_DISABLED:
        return
    remote_buttons_pressed = remote.buttons.pressed()
    while remote_buttons_pressed:
        wait(100)
        remote_buttons_pressed = remote.buttons.pressed()


def convert_millis_hours_minutes_seconds(millis: int):
    """
        utility for making milliseconds hours/minutes/seconds
    :param millis:
    :return hours, minutes, seconds:
    """
    hours = int((millis / (1000 * 60 * 60)) % 24)
    minutes = int((millis / (1000 * 60)) % 60)
    seconds = int((millis / 1000) % 60)

    return hours, minutes, seconds


_READY: int = const(0)
_ACTIVE: int = const(10)
_FINAL_MINUTE: int = const(20)
_FINAL_20_SECS: int = const(30)
_ENDED: int = const(40)
_UNKNOWN: int = const(99)


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.last_hub_remote_color = None
        self.last_countdown_message: str = ''
        self.countdown_status: int = _UNKNOWN

        # Start a timer.
        self.stopwatch = StopWatch()
        self.led_flash_sw_time = 0
        self.last_flash_color = None
        self.end_time = 0
        # battery check throttle
        self._battery_check_time = 0

    def has_time_remaining(self):
        """
            Checks if countdown has time remaining
        :return:
        """
        if self.countdown_status == _ENDED or self.countdown_status == _READY:
            return False

        # calculate remaining_time time
        remaining_time = self.end_time - self.stopwatch.time()

        # print a friendly console message
        con_hour, con_min, con_sec = convert_millis_hours_minutes_seconds(int(remaining_time))

        if con_sec % 10 == 0 and con_min < 1:
            countdown_message = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
            if self.last_countdown_message != countdown_message:
                self.last_countdown_message = countdown_message
                print(self.last_countdown_message)  # when time has run out end countdown
        if remaining_time <= 0:
            self.countdown_status = _ENDED
            self.show_status()
            return False
        # in last 20s fast flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = _FINAL_20_SECS
            self.show_status()
        # in last minute slow flash a warning
        elif remaining_time < (1000 * 60):
            self.countdown_status = _FINAL_MINUTE
            self.show_status()

        return True

    def __start_countdown__(self):
        """
            start the countdown sequence by resetting timers and status
        """
        print('start countdown')
        self.countdown_status = _ACTIVE
        self.show_status()
        self.end_time = self.stopwatch.time() + (COUNTDOWN_LIMIT_MINUTES * 60 * 1000)

    def reset(self):
        if _REMOTE_DISABLED:
            print('countdown time reset')
        else:
            print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = _READY

    def check_remote_buttons(self) -> bool:
        """
        Check countdown time buttons.
        Returns True if the reset code was pressed (caller should reset motor state).
        """
        if _REMOTE_DISABLED:
            return False

        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return False

        if self.countdown_status == _READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()
            return True

        return False

    def should_check_battery(self) -> bool:
        if self.stopwatch.time() > self._battery_check_time:
            self._battery_check_time = self.stopwatch.time() + 5000
            return True
        return False

    def show_status(self):
        if self.countdown_status == _READY:
            self.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, False)
        elif self.countdown_status == _ACTIVE:
            self.set_hub_and_remote_light(Color.GREEN, True)
        elif self.countdown_status == _FINAL_20_SECS:
            self.flash_hub_and_remote_light(Color.ORANGE, 200, Color.NONE, 100, False)
        elif self.countdown_status == _FINAL_MINUTE:
            self.flash_hub_and_remote_light(Color.ORANGE, 500, Color.NONE, 250, False)
        elif self.countdown_status == _ENDED:
            self.set_hub_and_remote_light(Color.ORANGE, False)

    def set_hub_and_remote_light(self, on_color:Color, include_remote:bool):
        """
        Set remote and hub light color if changed
        :param include_remote:
        :param on_color:
        :return:
        """
        global hub
        global remote

        # only set color if it's changed
        if on_color == self.last_hub_remote_color:
            return
        self.last_hub_remote_color = on_color

        hub.light.on(on_color)

        if include_remote and not _REMOTE_DISABLED:
            remote.light.on(on_color)

    def flash_hub_and_remote_light(self, on_color:Color, on_msec: int, off_color, off_msec: int, include_remote:bool):
        """
            this flashes the hub (and optionally remote) led
        :param include_remote:
        :param on_color:
        :param on_msec:
        :param off_color:
        :param off_msec:
        """
        # we use a timer to make it a non-blocking call
        now = self.stopwatch.time()
        if now > (on_msec + off_msec + self.led_flash_sw_time):
            self.led_flash_sw_time = now
            self.set_hub_and_remote_light(off_color, include_remote)
        elif now > (off_msec + self.led_flash_sw_time):
            self.set_hub_and_remote_light(on_color, include_remote)


##################################################################################
# Code helper
##################################################################################

def code_to_button_press_hash(button_code):
    """
    Returns the button needed to match a given code
    :param button_code:
    :return buttons_that_are_pressed[], buttons_that_should_not_pressed[]:
    """
    code_items = button_code.split(',')
    buttons_that_should_not_pressed = [Button.LEFT_PLUS, Button.LEFT_MINUS, Button.LEFT, Button.CENTER,
                                       Button.RIGHT_PLUS, Button.RIGHT_MINUS, Button.RIGHT]
    buttons_that_are_pressed = []
    # left code
    if '+' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT_PLUS)
        buttons_that_should_not_pressed.remove(Button.LEFT_PLUS)
    if '-' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT_MINUS)
        buttons_that_should_not_pressed.remove(Button.LEFT_MINUS)
    if 'c' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT)
        buttons_that_should_not_pressed.remove(Button.LEFT)

    # middle code
    if 'c' in code_items[1]:
        buttons_that_are_pressed.append(Button.CENTER)
        buttons_that_should_not_pressed.remove(Button.CENTER)

    # right code
    if '+' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT_PLUS)
        buttons_that_should_not_pressed.remove(Button.RIGHT_PLUS)
    if '-' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT_MINUS)
        buttons_that_should_not_pressed.remove(Button.RIGHT_MINUS)
    if 'c' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT)
        buttons_that_should_not_pressed.remove(Button.RIGHT)

    return buttons_that_are_pressed, buttons_that_should_not_pressed


PROGRAM_RESET_CODE_PRESSED, PROGRAM_RESET_CODE_NOT_PRESSED = code_to_button_press_hash(COUNTDOWN_RESET_CODE)

##################################################################################
# Main program
##################################################################################


hub: "CityHub | TechnicHub"
remote: "Remote" = None  # type: ignore  # bound by setup_remote(); stays None in headless modes


def setup_hub():
    global hub

    try:
        # this import will fail if the city hub is not connected.
        from pybricks.hubs import CityHub
        hub = CityHub()
        print('Lego City Hub found')
        return False
    except ImportError as ex1:
        print(ex1)
        try:
            from pybricks.hubs import TechnicHub
            hub = TechnicHub()
            print('Lego Technic Hub found')
            return True

        except ImportError as ex2:
            print(ex2)
            raise Exception('This program only support Lego City hub and Lego Technic hub')

def hub_battery_ok()->bool:
    global hub
    mv_voltage = hub.battery.voltage()
    return mv_voltage > MILLIVOLT_CRITICAL_LEVEL

LED_FLASHING_SEQUENCE = [75] * 5 + [1000]


def setup_remote(error_flash_code_helper, retry=5):
    global hub
    global remote

    # Flashing led while waiting connection
    hub.light.blink(Color.WHITE, LED_FLASHING_SEQUENCE)

    # try to connect to remote multiple times
    remote_retry_count = 1

    while True:
        # noinspection PyBroadException
        try:
            print("--looking for remote try " + str(remote_retry_count))
            # Connect to the remote
            remote = Remote()
            print("--remote connected.")
            break
        except:
            if remote_retry_count >= retry:
                error_flash_code_helper.set_error_no_remote()
                raise  # ignore first 20 errors
        remote_retry_count += 1
        wait(50)


WALL = 'X'
TRACK = '#'
WEST_ONLY_TRACK = '<'
EAST_ONLY_TRACK = '>'
LOAD = 'L'
UNLOAD = 'U'

_HALF = _CART_SIZE_DEG // 2  # 320

_X = const(0)
_Y = const(1)


class Grid:
    def __init__(self, layout):
        self.layout = layout
        self.n_rows = len(layout)
        self.n_cols = max(len(row) for row in layout)

        self.load_tile = (0, 0)
        self.unload_tile = (0, 0)

        wall_rects = []
        west_barriers = []
        east_barriers = []

        for ty, row in enumerate(layout):
            for tx, ch in enumerate(row):
                if ch == LOAD:
                    self.load_tile = (tx, ty)
                elif ch == UNLOAD:
                    self.unload_tile = (tx, ty)
                elif ch == WALL:
                    wl = tx * _DEG_PER_TILE
                    wt = ty * _DEG_PER_TILE
                    wr = wl + _DEG_PER_TILE
                    wb = wt + _DEG_PER_TILE
                    wall_rects.append((wl, wt, wr, wb))
                elif ch == WEST_ONLY_TRACK:
                    # '<' west-edge barrier blocks eastbound crossing
                    bx = tx * _DEG_PER_TILE
                    y_top = ty * _DEG_PER_TILE
                    y_bottom = (ty + 1) * _DEG_PER_TILE
                    west_barriers.append((bx, y_top, y_bottom))
                elif ch == EAST_ONLY_TRACK:
                    # '>' east-edge barrier blocks westbound crossing
                    bx = (tx + 1) * _DEG_PER_TILE
                    y_top = ty * _DEG_PER_TILE
                    y_bottom = (ty + 1) * _DEG_PER_TILE
                    east_barriers.append((bx, y_top, y_bottom))

        self._wall_rects = tuple(wall_rects)
        self._west_barriers = tuple(west_barriers)
        self._east_barriers = tuple(east_barriers)

    def tile_type(self, tx, ty):
        if tx < 0 or ty < 0 or ty >= self.n_rows or tx >= len(self.layout[ty]):
            return WALL
        return self.layout[ty][tx]

    def tile_center_deg(self, tile):
        tx, ty = tile
        return (tx * _DEG_PER_TILE + _DEG_PER_TILE // 2,
                ty * _DEG_PER_TILE + _DEG_PER_TILE // 2)

    def deg_to_tile(self, deg_pos):
        tx = deg_pos[0] // _DEG_PER_TILE
        ty = deg_pos[1] // _DEG_PER_TILE
        tx = max(0, min(tx, self.n_cols - 1))
        ty = max(0, min(ty, self.n_rows - 1))
        return (tx, ty)

    def _aabb_hits_wall(self, cx, cy):
        half = _HALF
        L = cx - half
        R = cx + half
        T = cy - half
        B = cy + half
        if L < 0 or T < 0 or R > self.n_cols * _DEG_PER_TILE or B > self.n_rows * _DEG_PER_TILE:
            return True
        for wl, wt, wr, wb in self._wall_rects:
            if R > wl and L < wr and B > wt and T < wb:
                return True
        return False

    def _boundary_overlap(self, cx, cy):
        """Total penetration depth into all obstacles (grid boundaries + wall tiles).
        Used to permit escape moves when motor coast leaves the cart stuck."""
        half = _HALF
        viol = 0
        top = cy - half
        if top < 0:
            viol -= top
        bot = cy + half - self.n_rows * _DEG_PER_TILE
        if bot > 0:
            viol += bot
        lft = cx - half
        if lft < 0:
            viol -= lft
        rgt = cx + half - self.n_cols * _DEG_PER_TILE
        if rgt > 0:
            viol += rgt
        L = cx - half
        R = cx + half
        T = cy - half
        B = cy + half
        for wl, wt, wr, wb in self._wall_rects:
            if R > wl and L < wr and B > wt and T < wb:
                viol += min(R - wl, wr - L, B - wt, wb - T)
        return viol

    def _overlaps_unload(self, cx, cy):
        """True if the AABB centred at (cx, cy) overlaps the unload tile rectangle."""
        tx, ty = self.unload_tile
        half = _HALF
        tile_l = tx * _DEG_PER_TILE
        tile_r = tile_l + _DEG_PER_TILE
        tile_t = ty * _DEG_PER_TILE
        tile_b = tile_t + _DEG_PER_TILE
        return (cx + half > tile_l and cx - half < tile_r and
                cy + half > tile_t and cy - half < tile_b)

    def _axis_step_legal(self, cx, cy, d, axis, block_unload=False):
        if axis == _X:
            new_cx = cx + d
            new_cy = cy
        else:
            new_cx = cx
            new_cy = cy + d

        if self._aabb_hits_wall(new_cx, new_cy):
            # Already out-of-bounds? Allow any step that reduces the violation (escape move).
            if self._aabb_hits_wall(cx, cy):
                return self._boundary_overlap(new_cx, new_cy) < self._boundary_overlap(cx, cy)
            return False

        if block_unload and self._overlaps_unload(new_cx, new_cy):
            # Allow escape if the cart is already inside the unload tile (e.g. after auto-drive),
            # but block fresh entry from outside.
            if not self._overlaps_unload(cx, cy):
                return False

        if axis == _X:
            half = _HALF
            if d > 0:
                # Eastbound: check '<' west-edge barriers
                east_face_before = cx + half
                east_face_after = east_face_before + d
                for bx, y_top, y_bottom in self._west_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    crossing = east_face_before <= bx and east_face_after > bx
                    if y_overlap and crossing:
                        return False
            elif d < 0:
                # Westbound: check '>' east-edge barriers
                west_face_before = cx - half
                west_face_after = west_face_before + d
                for bx, y_top, y_bottom in self._east_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    crossing = west_face_before >= bx and west_face_after < bx
                    if y_overlap and crossing:
                        return False

        return True

    def propose_step(self, deg_pos, d_deg_x, d_deg_y, block_unload=False):
        """
        Return (valid_dx, valid_dy): the largest per-axis step no greater in
        magnitude than the requested one that keeps the cart AABB legal.
        Per-axis independent: X is tested alone, Y is tested alone. If an
        axis is blocked, that axis returns 0; the other axis is unaffected.

        block_unload: when True, also blocks movement into the unload tile
        (used for manual/hybrid drive; auto-drive passes False).

        v1 limitation: combined step is not checked. A diagonal move can
        produce a position where the combined AABB overlaps a wall even if
        each individual axis passes. This is rare in practice because the
        AutoDriver's waypoint shaping avoids single-cell corner cuts.
        """
        cx, cy = deg_pos
        valid_dx = 0
        valid_dy = 0

        if d_deg_x != 0:
            if self._axis_step_legal(cx, cy, d_deg_x, _X, block_unload):
                valid_dx = d_deg_x

        if d_deg_y != 0:
            if self._axis_step_legal(cx, cy, d_deg_y, _Y, block_unload):
                valid_dy = d_deg_y

        return valid_dx, valid_dy


class VirtualJoystick:
    __slots__ = ('ax', 'ay')

    def __init__(self, ax=0, ay=0):
        self.ax = ax  # -1, 0, or +1
        self.ay = ay  # -1, 0, or +1


class AxisController:
    def __init__(self, motor_x, motor_y, grid, base_duty, _clock=None):
        # _clock: injectable StopWatch-like; defaults to pybricks StopWatch.
        self.motor_x = motor_x
        self.motor_y = motor_y
        self.grid = grid
        self.base_duty = base_duty
        if _clock is None:
            _clock = StopWatch()
        self._clock = _clock
        self._prev_duty_x = 0
        self._prev_duty_y = 0
        self._ramp_start_x = None  # ms timestamp, or None
        self._ramp_start_y = None

    def deg_pos(self):
        return (self.motor_x.angle(), self.motor_y.angle())

    def _ramp_stop_axis(self, motor, prev_duty, ramp_start, ramp_ms):
        """Run one ramp-stop tick for one axis.
        Returns (new_prev_duty, new_ramp_start)."""
        if prev_duty == 0:
            return 0, None
        now = self._clock.time()
        if ramp_start is None:
            ramp_start = now
        elapsed = now - ramp_start
        if elapsed >= ramp_ms:
            motor.dc(0)
            return 0, None
        factor = (ramp_ms - elapsed) * 100 // ramp_ms
        motor.dc(prev_duty * factor // 100)
        return prev_duty, ramp_start  # prev_duty unchanged during ramp

    def tick(self, vj, duty=None, ramp_ms=_STOP_RAMP_MS, block_unload=False):
        """One control tick. Proposes a lookahead step, clips it via Grid,
        issues motor.dc per axis. Active→idle transition ramps to zero over
        `ramp_ms` (manual default 200ms; AutoDriver passes a shorter value so
        coast stays inside wall clearances). `duty` overrides base_duty.
        `block_unload` prevents manual movement into the unload tile."""
        cx, cy = self.deg_pos()
        both = vj.ax != 0 and vj.ay != 0
        base = duty if duty is not None else self.base_duty
        duty = base * _BOTH_AXES_DUTY_NUM // _BOTH_AXES_DUTY_DEN if both else base

        requested_dx = vj.ax * _LOOKAHEAD_DEG
        requested_dy = vj.ay * _LOOKAHEAD_DEG
        valid_dx, valid_dy = self.grid.propose_step((cx, cy), requested_dx, requested_dy, block_unload)

        if DEBUG and (valid_dx != requested_dx or valid_dy != requested_dy):
            print('clip pos=(', cx, cy, ') req=(', requested_dx, requested_dy, ') valid=(', valid_dx, valid_dy, ')')

        # X axis
        if valid_dx > 0:
            self.motor_x.dc(+duty)
            self._prev_duty_x = +duty
            self._ramp_start_x = None
        elif valid_dx < 0:
            self.motor_x.dc(-duty)
            self._prev_duty_x = -duty
            self._ramp_start_x = None
        else:
            self._prev_duty_x, self._ramp_start_x = self._ramp_stop_axis(
                self.motor_x, self._prev_duty_x, self._ramp_start_x, ramp_ms
            )

        # Y axis
        if valid_dy > 0:
            self.motor_y.dc(+duty)
            self._prev_duty_y = +duty
            self._ramp_start_y = None
        elif valid_dy < 0:
            self.motor_y.dc(-duty)
            self._prev_duty_y = -duty
            self._ramp_start_y = None
        else:
            self._prev_duty_y, self._ramp_start_y = self._ramp_stop_axis(
                self.motor_y, self._prev_duty_y, self._ramp_start_y, ramp_ms
            )


_DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def _aim(delta, deadband):
    """Like _sign but with a deadband — prevents the idle axis from pulsing on
    sub-step drift, and lets the caller size the deadband to match expected
    coast distance so the motor stops pushing exactly when remaining coast
    carries the cart onto the target (no overshoot)."""
    if delta > deadband:
        return 1
    if delta < -deadband:
        return -1
    return 0


def _coast_distance_deg(duty, ramp_ms):
    """Predicted coast distance for a linear ramp from `duty%` to 0 over
    `ramp_ms` — half the triangle: (duty/100) * max_speed_degps * (ramp_ms/1000) / 2."""
    return duty * _MAX_MOTOR_ROT_SPEED * ramp_ms // 200000


def _sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0


def _within(a, b, r):
    return abs(a - b) <= r


class Planner:
    def __init__(self, grid):
        self.grid = grid

    def plan(self, start, goal):
        """Return an immutable tuple of coarse-tile waypoints from start to
        goal. First element == start, last == goal. Intermediate elements
        are only turn-points (cells where the direction changes). Returns
        empty tuple if unreachable or if start/goal is impassable."""
        if not self._passable(start) or not self._passable(goal):
            return ()
        if start == goal:
            return (start,)

        parents = {start: None}
        queue = [start]
        qi = 0
        found = False
        while qi < len(queue):
            tile = queue[qi]
            qi += 1
            if tile == goal:
                found = True
                break
            for dx, dy in _DIRECTIONS:
                nxt = (tile[0] + dx, tile[1] + dy)
                if nxt in parents:
                    continue
                if not self._can_step(nxt, dx):
                    continue
                parents[nxt] = tile
                queue.append(nxt)

        if not found:
            return ()

        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parents[cur]
        path.reverse()

        if len(path) <= 2:
            return tuple(path)
        turn_points = [path[0]]
        prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        for i in range(1, len(path) - 1):
            next_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            if next_dir != prev_dir:
                turn_points.append(path[i])
            prev_dir = next_dir
        turn_points.append(path[-1])
        return self._cut_corners(turn_points)

    def _cut_corners(self, waypoints):
        """Replace L-turn corner tiles with diagonal shortcuts where safe.
        For each triple (a, b, c) with cardinal-perpendicular legs, try
        replacing b with (b minus one step along a->b); keep the swap only
        if the simulated diagonal from candidate to c stays unblocked."""
        if len(waypoints) < 3:
            return tuple(waypoints)
        result = [waypoints[0]]
        n = len(waypoints)
        i = 1
        while i < n - 1:
            a = waypoints[i - 1]
            b = waypoints[i]
            c = waypoints[i + 1]
            d1x, d1y = _sign(b[0] - a[0]), _sign(b[1] - a[1])
            d2x, d2y = _sign(c[0] - b[0]), _sign(c[1] - b[1])
            # Perpendicular cardinals: dot product zero AND neither is the zero vector.
            perpendicular = (d1x * d2x + d1y * d2y) == 0 and (d1x or d1y) and (d2x or d2y)
            if perpendicular:
                candidate = (b[0] - d1x, b[1] - d1y)
                if candidate != a and self._safe_diagonal(candidate, c):
                    result.append(candidate)
                    i += 1
                    continue
            result.append(b)
            i += 1
        result.append(waypoints[-1])
        return tuple(result)

    def _safe_diagonal(self, cand_tile, goal_tile):
        """Simulate AutoDriver-style motion from cand_tile centre to
        goal_tile centre. Safe iff every tick advances each requested axis
        fully (no partial block)."""
        g = self.grid
        cx, cy = g.tile_center_deg(cand_tile)
        gx, gy = g.tile_center_deg(goal_tile)
        # Bound: Manhattan distance / lookahead, with headroom.
        max_ticks = 4 * (abs(gx - cx) + abs(gy - cy)) // _LOOKAHEAD_DEG + 4
        for _ in range(max_ticks):
            dx = gx - cx
            dy = gy - cy
            if dx == 0 and dy == 0:
                return True
            step_x = _sign(dx) * _LOOKAHEAD_DEG
            step_y = _sign(dy) * _LOOKAHEAD_DEG
            if step_x and abs(step_x) > abs(dx):
                step_x = dx
            if step_y and abs(step_y) > abs(dy):
                step_y = dy
            valid_dx, valid_dy = g.propose_step((cx, cy), step_x, step_y)
            if (step_x != 0 and valid_dx != step_x) or (step_y != 0 and valid_dy != step_y):
                return False
            cx += valid_dx
            cy += valid_dy
        return False

    def _passable(self, tile):
        tx, ty = tile
        g = self.grid
        if tx < 0 or ty < 0 or ty >= g.n_rows or tx >= g.n_cols:
            return False
        return g.tile_type(tx, ty) != WALL

    def _can_step(self, nxt, dx):
        if not self._passable(nxt):
            return False
        # N1d edge barriers: arrow direction is the allowed one.
        # '<' west edge blocks eastbound entry into '<'.
        # '>' east edge blocks westbound entry into '>'.
        t = self.grid.tile_type(*nxt)
        if dx > 0 and t == WEST_ONLY_TRACK:
            return False
        if dx < 0 and t == EAST_ONLY_TRACK:
            return False
        return True


class AutoDriver:
    def __init__(self, grid, planner, axis_controller):
        self.grid = grid
        self.planner = planner
        self.axis_controller = axis_controller
        self.waypoints = ()
        self.i = 0

    def start_journey(self, from_tile, to_tile):
        self.waypoints = self.planner.plan(from_tile, to_tile)
        self.i = 0

    def tick(self, remote):
        """Run one tick. Returns:
            None while journey continues,
            'reached_load' / 'reached_unload' / 'reached_end' on arrival,
            'yielded' if any real remote button is pressed (joystick not emitted).
        """
        if remote is not None and any(remote.buttons.pressed()):
            return 'yielded'

        if self.i >= len(self.waypoints) - 1:
            return self._end_tag()

        cx, cy = self.axis_controller.deg_pos()
        target = self.grid.tile_center_deg(self.waypoints[self.i + 1])

        if _within(cx, target[0], _AIM_SWITCH_DEG) and _within(cy, target[1], _AIM_SWITCH_DEG):
            self.i += 1
            if DEBUG:
                print('advance i=', self.i, 'of', len(self.waypoints))
            if self.i >= len(self.waypoints) - 1:
                return self._end_tag()
            target = self.grid.tile_center_deg(self.waypoints[self.i + 1])

        deadband = _coast_distance_deg(_AUTO_DRIVE_DUTY, _AUTO_STOP_RAMP_MS) + _DEADBAND_SAFETY_DEG
        vj = VirtualJoystick(_aim(target[0] - cx, deadband), _aim(target[1] - cy, deadband))
        self.axis_controller.tick(vj, duty=_AUTO_DRIVE_DUTY, ramp_ms=_AUTO_STOP_RAMP_MS)
        return None

    def _end_tag(self):
        if not self.waypoints:
            return 'reached_end'
        last = self.waypoints[-1]
        if last == self.grid.load_tile:
            return 'reached_load'
        if last == self.grid.unload_tile:
            return 'reached_unload'
        return 'reached_end'


class IdleTimeout:
    def __init__(self, seconds, _clock=None):
        self._interval_ms = seconds * 1000
        if _clock is None:
            _clock = StopWatch()
        self._clock = _clock
        self._last_reset_ms = self._clock.time()

    def reset(self):
        self._last_reset_ms = self._clock.time()

    def fired(self):
        return (self._clock.time() - self._last_reset_ms) >= self._interval_ms


class HomingRoutine:
    def __init__(self, motor_x, motor_y, grid):
        self.motor_x = motor_x
        self.motor_y = motor_y
        self.grid = grid

    def run(self):
        """Stall Y north, stall X east, reset encoders so motor.angle() reads
        cart-center position in the grid frame, then run to U's tile center.
        After run(), motor.angle() == grid.tile_center_deg(grid.unload_tile).
        """
        target_x, target_y = self.grid.tile_center_deg(self.grid.unload_tile)
        east_wall_deg = self.grid.n_cols * _DEG_PER_TILE

        # Stall Y NORTH against top wall; cart center is _HALF south of the wall.
        self.motor_y.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        self.motor_y.reset_angle(_HALF)
        self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, target_y)
        wait(200)

        # Stall X EAST against right wall — also unloads the cart.
        self.motor_x.run_until_stalled(_HOMING_MOTOR_ROT_SPEED * 3, duty_limit=_HOMING_DUTY)
        wait(2000)
        self.motor_x.reset_angle(east_wall_deg - _X_EAST_STALL_OFFSET_DEG)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x)
        wait(200)


_LOAD_DIP_DEG = const(240)


class RunODVMotors(MotorHelper):
    """ODV drive coordinator built on the Grid/AxisController/AutoDriver stack."""

    def __init__(self, error_flash_code_helper, drive_speed, grid_layout,
                 _motors=None, _remote=None, _clock=None):
        super().__init__(False, True)
        self.error_flash_code = error_flash_code_helper
        self.drive_speed = drive_speed
        self.has_load = False
        self._remote = _remote

        if _motors is not None:
            self.motor_x, self.motor_y = _motors
        else:
            self.motor_x_port = Port.A
            self.motor_y_port = Port.C
            try:
                self.motor_x = Motor(self.motor_x_port, Direction.COUNTERCLOCKWISE)
            except OSError as ex:
                if ex.errno == ENODEV:
                    self.error_flash_code.set_error_no_motor_on_a()
                raise
            try:
                self.motor_y = Motor(self.motor_y_port, Direction.CLOCKWISE)
            except OSError as ex:
                if ex.errno == ENODEV:
                    self.error_flash_code.set_error_no_motor_on_b()
                raise

        self.grid = Grid(grid_layout)
        self.planner = Planner(self.grid)
        self.axis_controller = AxisController(self.motor_x, self.motor_y, self.grid,
                                              drive_speed, _clock=_clock)
        self.auto_driver = AutoDriver(self.grid, self.planner, self.axis_controller)
        self.homing_routine = HomingRoutine(self.motor_x, self.motor_y, self.grid)

        if DRIVE_MODE == HYBRID:
            self.idle_timeout = IdleTimeout(IDLE_TIMEOUT_SECS, _clock=_clock)
        else:
            self.idle_timeout = None

        self.stop_motors()

    def _current_tile(self):
        return self.grid.deg_to_tile(self.axis_controller.deg_pos())

    def _get_remote(self):
        if self._remote is not None:
            return self._remote
        return remote

    def home_and_unload(self):
        if DEBUG:
            print('home_and_unload start pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')
        self.homing_routine.run()
        self.has_load = False
        self.set_is_homed()
        if DEBUG:
            print('home_and_unload done pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')

    def reset_homing(self):
        self.reset_is_homed()
        self.disable_auto_drive()

    def stop_motors(self):
        self.motor_x.stop()
        self.motor_y.stop()

    def idle_timed_out(self):
        if self.idle_timeout is None:
            return False
        return self.idle_timeout.fired()

    def reset_idle_timeout(self):
        if self.idle_timeout is not None:
            self.idle_timeout.reset()

    def handle_remote_press(self):
        if self.mh__remote_disabled:
            return
        rem = self._get_remote()
        if rem is None:
            return
        pressed = rem.buttons.pressed()

        if len(pressed) == 0 or Button.LEFT in pressed or Button.RIGHT in pressed:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            if len(pressed) > 0 and self.idle_timeout is not None:
                self.idle_timeout.reset()
            return

        ax = 0
        ay = 0
        if Button.LEFT_PLUS in pressed:
            ay = -1
        elif Button.LEFT_MINUS in pressed:
            ay = +1
        if Button.RIGHT_PLUS in pressed:
            ax = +1
        elif Button.RIGHT_MINUS in pressed:
            ax = -1

        if self.idle_timeout is not None:
            self.idle_timeout.reset()

        if ax == 0 and ay == 0:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            return

        cur = self._current_tile()
        if (ax, ay) == (-1, 0) and cur == self.grid.load_tile:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            self._do_load_()
            return
        if (ax, ay) == (+1, 0) and cur == self.grid.unload_tile:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            self.home_and_unload()
            return

        self.axis_controller.tick(VirtualJoystick(ax, ay), block_unload=True)

    def _do_load_(self):
        if self.has_load:
            if DEBUG:
                print('_do_load_ skip (has_load)')
            return
        target_x = self.grid.tile_center_deg(self.grid.load_tile)[0]
        if DEBUG:
            print('_do_load_ start pos=(', self.motor_x.angle(), self.motor_y.angle(),
                  ') dip to', target_x - _LOAD_DIP_DEG)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x - _LOAD_DIP_DEG)
        wait(2000)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x)
        self.has_load = True
        if DEBUG:
            print('_do_load_ done pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')

    def _drive_auto_journey(self, goal_tile):
        start = self._current_tile()
        self.auto_driver.start_journey(start, goal_tile)
        rem = self._get_remote()
        if DEBUG:
            print('journey', start, '->', goal_tile, 'waypoints', self.auto_driver.waypoints)
        tick_count = 0
        while True:
            result = self.auto_driver.tick(rem)
            tick_count += 1
            if DEBUG and tick_count % 50 == 0:
                cx, cy = self.axis_controller.deg_pos()
                i = self.auto_driver.i
                wps = self.auto_driver.waypoints
                nxt = wps[i + 1] if i + 1 < len(wps) else None
                print('t=', tick_count, 'i=', i, 'pos=(', cx, cy, ') next=', nxt)
            if result == 'yielded':
                if DEBUG:
                    print('yielded')
                self.disable_auto_drive()
                self.stop_motors()
                return False
            if result is not None:
                if DEBUG:
                    print(result)
                ex, ey = self.grid.tile_center_deg(goal_tile)
                self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, ex)
                self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, ey)
                self.stop_motors()
                wait(500)
                return True
            wait(10)

    def auto_load(self):
        if not self.mh_is_homed:
            return
        if _CALIBRATE_X_OFFSET:
            # Halt at tile (3, 0) center so the X offset can be measured on rig.
            if self._drive_auto_journey((3, 0)):
                ex, ey = self.grid.tile_center_deg((3, 0))
                self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, ex)
                self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, ey)
                cx, cy = self.axis_controller.deg_pos()
                print('CALIBRATE pos=(', cx, cy, ') expected=(', ex, ey, ')')
                raise SystemExit('calibration stop at tile (3, 0)')
            return
        if self._drive_auto_journey(self.grid.load_tile):
            self._do_load_()

    def auto_unload(self):
        if not self.mh_is_homed:
            return
        if not self.has_load:
            return
        if self._drive_auto_journey(self.grid.unload_tile):
            self.home_and_unload()



def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    setup_hub()
    print('voltage',hub.battery.voltage())
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)

        drive_motors.mh__remote_disabled = _REMOTE_DISABLED

        if _REMOTE_DISABLED:
            print('--no remote')
        else:
            print('--setup remote')
            setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        mem_info()
        while True:
            if countdown_timer.should_check_battery() and not hub_battery_ok():
                error_flash_code.set_error_low_battery()
                raise Exception('low battery')

            if not _REMOTE_DISABLED:
                if countdown_timer.check_remote_buttons():
                    drive_motors.stop_motors()
                    if drive_motors.mh_supports_homing:
                        drive_motors.reset_homing()

            if drive_motors.mh_supports_homing:
                if not drive_motors.mh_auto_drive:
                    # Full auto: enable immediately once homed, no remote needed
                    if _REMOTE_DISABLED and drive_motors.mh_is_homed:
                        drive_motors.enable_auto_drive()
                    # Hybrid: enable after idle timeout
                    elif drive_motors.idle_timed_out():
                        drive_motors.enable_auto_drive()

                if drive_motors.mh_auto_drive and drive_motors.mh_is_homed:
                    drive_motors.auto_unload()
                    drive_motors.auto_load()
                    # Hybrid: if a button press interrupted auto, reset the idle timer so auto
                    # doesn't re-enable immediately on the next loop iteration
                    if not drive_motors.mh_auto_drive:
                        countdown_timer.__start_countdown__()
                        drive_motors.reset_idle_timeout()

            # if there is no remote, then there is no point in a countdown
            if countdown_timer.has_time_remaining() or _REMOTE_DISABLED or drive_motors.mh_auto_drive or drive_motors.mh_is_homed:
                if drive_motors.mh_supports_homing and not drive_motors.mh_is_homed:
                    drive_motors.home_and_unload()
                if drive_motors.mh_supports_flip:
                    drive_motors.handle_flip()
                if not _REMOTE_DISABLED:
                    drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()
                if drive_motors.mh_supports_homing:
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)

            if _REMOTE_DISABLED and not drive_motors.mh_supports_homing:
                print("No remote exiting")
                raise SystemExit

    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
