# Timed train and vehicle program for interactive displays
# Copyright Etendut
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
    from modules.mock_types import MockHub, MockRemote

from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from uerrno import ENODEV
from umath import floor


print('Version 2.1.0')
##################################################################################
#  Settings
##################################################################################

# countdown time settings
COUNTDOWN_LIMIT_MINUTES: int = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button

#How many seconds to wait before doing a load/unload automatically. 0 = disabled
ODV_AUTO_DRIVE_TIMEOUT_SECS: int = const(10)

# for debugging or ODV full auto
REMOTE_DISABLED=True


# odv settings
ODV_SPEED: int = const(45)  # set between 40 and 70
# X= obstacle, H= Home, L = Load, U = Unload, # = grid tile
# ODV_GRID = ["H######", "###X#XX", "LX###XU", "###X###"]
ODV_GRID = ["XL##XU", "H#X#X#", "XXX###"]


##################################################################################
# ---------Main program below, editing should not be needed -------------

# {Insert drive module here}

class ErrorFlashCodes:
    def __init__(self):
        self.flash_count = 1  # Other errors

    def set_error_no_motor_on_a(self):
        self.flash_count = 2

    def set_error_no_motor_on_b(self):
        self.flash_count = 3

    def set_error_no_remote(self):
        self.flash_count = 4

    def flash_error_code(self):
        """
            this flashes the remote led
        """

        global hub

        for f in range(self.flash_count):
            hub.light.on(Color.RED)
            wait(350)
            hub.light.on(Color.NONE)
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

    def do_homing(self):
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

    def auto_home(self):
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


##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    if REMOTE_DISABLED:
        return
    remote_buttons_pressed = remote.buttons.pressed()
    while remote_buttons_pressed:
        remote_buttons_pressed = remote.buttons.pressed()
        wait(100)


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
        self.last_countdown_message:str = ''
        self.countdown_status:int = _UNKNOWN

        # Start a timer.
        self.stopwatch = StopWatch()
        self.led_flash_sw_time = 0
        self.end_time = 0
        # remote timing
        self.remote_buttons_time_out_ms = 0
        self.reset_time_since_last_remote_press()



    def reset_time_since_last_remote_press(self):
        self.remote_buttons_time_out_ms = self.stopwatch.time() + (ODV_AUTO_DRIVE_TIMEOUT_SECS*1000)

    def remote_button_press_timed_out(self)->bool:
        return self.stopwatch.time() > self.remote_buttons_time_out_ms

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
        # in last 25s slow flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = _FINAL_20_SECS
            self.show_status()

            # in last minute slow flash a warning
        if remaining_time < (1000 * 60):
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
        if REMOTE_DISABLED:
            print('countdown time reset')
        else:
            print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = _READY
        self.reset_time_since_last_remote_press()

    def check_remote_buttons(self):
        """
            check countdown time buttons
        """
        if REMOTE_DISABLED:
            return

        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return

        self.reset_time_since_last_remote_press()

        if self.countdown_status == _READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def show_status(self):
        global hub
        global remote
        if self.countdown_status == _READY:
            self.__flash_remote_and_hub_light__(Color.GREEN, 500, Color.NONE, 500)
        elif self.countdown_status == _ACTIVE:
            hub.light.on(Color.GREEN)
            if not REMOTE_DISABLED:
                remote.light.on(Color.GREEN)
        elif self.countdown_status == _FINAL_20_SECS:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 200, Color.NONE, 100)
        elif self.countdown_status == _FINAL_MINUTE:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 500, Color.NONE, 250)
        elif self.countdown_status == _ENDED:
            hub.light.on(Color.ORANGE)
            if not REMOTE_DISABLED:
                remote.light.on(Color.ORANGE)


    def __flash_remote_and_hub_light__(self, on_color, on_msec: int, off_color, off_msec: int):
        """
            this flashes the remote led
        :param on_color:
        :param on_msec:
        :param off_color:
        :param off_msec:
        """
        global hub
        # we use a timer to make it a non-blocking call
        if self.stopwatch.time() > (on_msec + off_msec + self.led_flash_sw_time):
            self.led_flash_sw_time = self.stopwatch.time()
            if not REMOTE_DISABLED:
                remote.light.on(off_color)
            hub.light.on(off_color)
        elif self.stopwatch.time() > (off_msec + self.led_flash_sw_time):
            if not REMOTE_DISABLED:
                remote.light.on(on_color)
            hub.light.on(on_color)


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



hub: "MockHub"
remote: "MockRemote"

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


##################################################################################
# ODV helper
##################################################################################
_NORTH_WEST = const(0)
_NORTH = const(1)
_NORTH_EAST = const(2)
_EAST = const(3)
_SOUTH_EAST = const(4)
_SOUTH = const(5)
_SOUTH_WEST = const(6)
_WEST = const(7)

WALL = 'X'
TRACK = '#'
HOME = 'H'
LOAD = 'L'
UNLOAD = 'U'
OK_MOVES = [TRACK, LOAD, UNLOAD]

_FINE_GRID_SIZE = const(10)
_ODV_SIZE = const(8)

_GEAR_RATIO_TO_GRID: int = const(80)  # Motor rotation angle per grid pitch (deg/pitch)
_MAX_MOTOR_ROT_SPEED: int = const(1400)  # Max motor speed (deg/s) ~1500
_HOMING_MOTOR_ROT_SPEED: int = const(200)  # Homing speed (deg/s)
_HOMING_DUTY: int = const(45)  # Homing motor duty (%) (adjustment required)

def position_from_direction(position: tuple[int, int], direction: int)->tuple[int, int]:
    if direction == _NORTH:
        return position[0], position[1] - 1
    if direction == _NORTH_EAST:
        return position[0] + 1, position[1] - 1
    if direction == _EAST:
        return position[0] + 1, position[1]
    if direction == _SOUTH_EAST:
        return position[0] + 1, position[1] + 1
    if direction == _SOUTH:
        return position[0], position[1] + 1
    if direction == _SOUTH_WEST:
        return position[0] - 1, position[1] + 1
    if direction == _WEST:
        return position[0] - 1, position[1]
    if direction == _NORTH_WEST:
        return position[0] - 1, position[1] - 1

    return position[0], position[1]


class ODVBox:
    def __init__(self, top_left: tuple[int, int], width: int, height: int):
        self.width = 0
        self.height = 0
        self.top_left: tuple[int, int]
        self.top_right: tuple[int, int]
        self.bottom_right: tuple[int, int]
        self.bottom_left: tuple[int, int]
        self.upper_left: tuple[int, int]
        self._update_dimensions_(top_left, width, height)

    def _update_dimensions_(self, top_left: tuple[int, int], width: int, height: int):
        self.width = width
        self.height = height
        self.top_left = top_left
        self.top_right = (self.top_left[0] + self.width, self.top_left[1])
        self.bottom_right = (self.top_right[0], self.top_left[1] + self.height)
        self.bottom_left = (self.top_left[0], self.bottom_right[1])

    def buffer(self, buffer: int):
        new_tl = (self.top_left[0] - buffer, self.top_left[1] - buffer)
        self._update_dimensions_(new_tl, self.width + (buffer * 2), self.height + (buffer * 2))

    def __str__(self):
        return f"[{self.top_left}, {self.top_right}]\n[{self.bottom_left}, {self.bottom_right}]"


class Queue:
    """ No Queue in micropython :("""

    def __init__(self) -> None:
        self._queue: list[list[tuple[int, int]]] = []

    def put(self, item: list[tuple[int, int]]):
        self._queue.append(item)

    def empty(self):
        return len(self._queue) == 0

    def get(self) -> list[tuple[int, int]]:
        first = self._queue[0]
        del self._queue[0]
        return first


class RunODVMotors(MotorHelper):
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, grid_layout: list[str]):

        super().__init__(False, True)
        # grid setup
        self.motors_running = None
        self.home_tile: tuple[int, int] = (0, 0)
        self.unload_tile: tuple[int, int] = (0, 0)
        self.load_tile: tuple[int, int] = (0, 0)
        self.last_fine_grid_position: tuple[int, int] = (0, 0)
        """current position"""
        self.grid_tracks = []
        self.coarse_grid_width = 0
        self.coarse_grid_height = 0
        self._load_grid_(grid_layout)

        self.has_load = False
        # motor setup
        self.error_flash_code = error_flash_code_helper

        self.motor_x_port = Port.A
        self.motor_y_port = Port.C

        self.drive_speed = drive_speed

        try:
            self.motor_x = Motor(self.motor_x_port, Direction.COUNTERCLOCKWISE)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.motor_x_port))
                self.error_flash_code.set_error_no_motor_on_a()
            raise
        try:
            self.motor_y = Motor(self.motor_y_port, Direction.CLOCKWISE)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.motor_y_port))
                self.error_flash_code.set_error_no_motor_on_b()
            raise

        self.stop_motors()

    def _load_grid_(self, lines: list[str]):
        # loop through grid lines
        y = 0
        print('Loading grid')
        mem_info()
        for y, line in enumerate(lines):
            print(f"line {y + 1}/{len(lines)}")
            self.coarse_grid_width = const(len(line.rstrip()))
            line = line.rstrip()
            for x, character in enumerate(line):
                print(f"line {y + 1} |col {x + 1}/{len(line)}|{character}")

                if character in OK_MOVES:
                    self.grid_tracks.append((x, y))
                # set load/unload points
                if character == HOME:
                    print('----home_tile----')
                    mem_info()
                    self.home_tile = const((x, y))
                    mem_info()
                    print('----home_tile----')
                if character == LOAD:
                    print('----load_tile----')
                    mem_info()
                    self.load_tile = const((x, y))
                    mem_info()
                    print('----load_tile----')
                if character == UNLOAD:
                    print('----unload_tile----')
                    mem_info()
                    self.unload_tile = const((x, y))
                    mem_info()
                    print('----unload_tile----')

            y += 1
        self.coarse_grid_height = const(y)
        mem_info()
        print('Grid Loaded')
        print(f"--home tile is {self.home_tile}")
        print(f"--loads tile is {self.load_tile}")
        print(f"--unload tile is {self.unload_tile}")
        self._display_grid_()

    def _display_grid_(self, position_x_y: tuple = None):
        # Display the maze:
        for y in range(self.coarse_grid_height):
            for x in range(self.coarse_grid_width):
                if position_x_y is not None and (x, y) == position_x_y:
                    print("R", end='')
                elif (x, y) == self.home_tile:
                    print(HOME, end='')
                elif (x, y) == self.load_tile:
                    print(LOAD, end='')
                elif (x, y) == self.unload_tile:
                    print(UNLOAD, end='')
                elif (x, y) in self.grid_tracks:
                    print(TRACK, end='')
                else:
                    print(WALL, end='')
            print()  # Print a newline after printing the row.

    def reset_homing(self) -> None:
        self.reset_is_homed()

    def do_homing(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then move forward by an offset distance and set that as the zero origin.
        if self.mh_is_homed:
            return
        # Homing axis Y
        self.motor_y.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        home_tile_angle = self._tile_to_angle(self.home_tile)
        self.motor_y.reset_angle(home_tile_angle[1])
        self.motor_y.run_angle(_MAX_MOTOR_ROT_SPEED, _GEAR_RATIO_TO_GRID)
        wait(200)

        # Homing axis X
        self.motor_x.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        self.motor_x.reset_angle(home_tile_angle[0])
        self.motor_x.run_angle(_MAX_MOTOR_ROT_SPEED, _GEAR_RATIO_TO_GRID)
        wait(200)

        self.set_is_homed()
        self._display_grid_(self.home_tile)

    def _can_move_in_direction_(self, direction: int) -> tuple[bool, bool, bool]:
        if direction not in [_NORTH, _NORTH_EAST, _EAST, _SOUTH_EAST, _SOUTH, _SOUTH_WEST, _WEST, _NORTH_WEST]:
            return False, False, False

        # work out cart dimensions
        cart = ODVBox(self.last_fine_grid_position, _ODV_SIZE, _ODV_SIZE)
        # shrink the cart to make moving smoother
        cart.buffer(-1)

        # print("Cart", cart)

        tl = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.top_left, direction))
        tr = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.top_right, direction))
        br = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.bottom_right, direction))
        bl = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.bottom_left, direction))

        # most tiles support universal movement
        # TRACK - any direction
        # LOAD - only on left
        # UNLOAD - only on right
        # HOME - can only be moved into from bottom or right

        can_move = (tl in OK_MOVES and tr in OK_MOVES and br in OK_MOVES and bl in OK_MOVES)
        # handle home tile only supporting 2 directions
        if not can_move and HOME in [tl, tr, bl, br]:
            # cart in home tile
            if tl == tr == br == bl == HOME:
                can_move = True
            # moving NW into tile
            elif tl == HOME and tr == br == bl == TRACK:
                can_move = True
            # moving N or S
            elif tl == tr == HOME and br == bl == TRACK:
                can_move = True
            # moving E or W
            elif tl == bl == HOME and tr == br == TRACK:
                can_move = True

        can_load = tl == tr == br == bl == LOAD
        can_unload = tl == tr == br == bl == UNLOAD

        # print("Cart", direction, can_move)
        return can_move, can_load, can_unload

    def _get_fine_grid_position_(self) -> tuple:

        x_grid = int(self.motor_x.angle() / _GEAR_RATIO_TO_GRID)
        y_grid = int(self.motor_y.angle() / _GEAR_RATIO_TO_GRID)
        return x_grid, y_grid

    def _get_grid_tile_type_from_fine_xy_(self, fine_position: tuple) -> str:
        tile_position, tile_type = self._get_grid_tile_from_fine_xy_(fine_position)
        return tile_type

    def _get_grid_tile_position_from_fine_xy_(self, fine_position: tuple) -> tuple[int, int]:
        tile_position, tile_type = self._get_grid_tile_from_fine_xy_(fine_position)
        return tile_position

    def _get_grid_tile_from_fine_xy_(self, fine_position: tuple) -> tuple[tuple[int, int], str]:

        x_grid = floor(fine_position[0] / _FINE_GRID_SIZE)
        y_grid = floor(fine_position[1] / _FINE_GRID_SIZE)
        # print("Coarse", x_grid, y_grid)
        tile = (x_grid, y_grid)
        if fine_position[0] < 1 or fine_position[
            1] < 1 or x_grid < 0 or y_grid < 0 or x_grid > self.coarse_grid_width or y_grid > self.coarse_grid_height:
            return tile, WALL
        if (x_grid, y_grid) in self.grid_tracks:
            return tile, TRACK

        return tile, WALL

    def _move_in_direction_(self, direction: int) -> bool:

        if direction not in [_NORTH, _NORTH_EAST, _EAST, _SOUTH_EAST, _SOUTH, _SOUTH_WEST, _WEST, _NORTH_WEST]:
            print('Invalid direction')
            return False

        if direction in [_NORTH, _NORTH_EAST, _NORTH_WEST]:
            self.motor_y.dc(-self.drive_speed)
        if direction in [_SOUTH, _SOUTH_EAST, _SOUTH_WEST]:
            self.motor_y.dc(self.drive_speed)

        if direction in [_EAST, _NORTH_EAST, _SOUTH_EAST]:
            self.motor_x.dc(self.drive_speed)
        if direction in [_WEST, _NORTH_WEST, _SOUTH_WEST]:
            self.motor_x.dc(-self.drive_speed)

        self.motors_running = True
        return True

    def _do_load_(self):
        if self.has_load:
            print('Already loaded')
            return
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_())
        if tile != self.load_tile:
            print(f'Not on load_tile')
            return
        self._navigate_to_grid_tile(self.load_tile)
        wait(200)
        # do load
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, (_GEAR_RATIO_TO_GRID * -3))
        wait(2000)
        print("loading..")
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, 0)
        wait(200)
        self.has_load = True
        print("ready to go")

    def _do_unload_(self):

        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_())
        if tile != self.unload_tile:
            print('Not on unload_tile')
            return

        tile_angle = self._navigate_to_grid_tile(self.unload_tile)
        wait(200)
        print("unloading..")
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle[0] + (_GEAR_RATIO_TO_GRID * 4))
        wait(2000)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle[0])
        wait(200)
        self.has_load = False
        print("ready to go")

    @staticmethod
    def _tile_to_angle(tile: tuple[int, int]) -> tuple[int, int]:
        """
        Convert grid tile to angle
        :param tile: tuple[int,int]
        :return: ODVAnglePosition
        """
        tile_angle_x = tile[0] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID
        tile_angle_y = (tile[1] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID) + _GEAR_RATIO_TO_GRID
        return tile_angle_x, tile_angle_y

    def _navigate_to_grid_tile(self, tile: tuple[int, int], stop=Stop.HOLD) -> tuple[int, int]:
        print(f"navigating to tile {tile}")
        tile_angle_x = tile[0] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID
        tile_angle_y = (tile[1] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID) + _GEAR_RATIO_TO_GRID
        self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle_y, then=stop)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle_x, then=stop)
        return tile_angle_x, tile_angle_y

    def _navigate_grid_tile_path(self, grid_tile_path: list[tuple[int, int]]) -> bool:
        """
        Navigates robot through list of tuple[int,int]
        :param grid_tile_path:
        :return: succeeded
        """
        for i, path in enumerate(grid_tile_path):
            # if user takes over break
            if self.mh_auto_drive and not self.mh__remote_disabled and len(remote.buttons.pressed()) > 0:
                self.disable_auto_drive()
                self.stop_motors()
                return False
            # TODO logic for direction
            # if path[1] is not None and i < (len(grid_tile_path) - 1) and grid_tile_path[i + 1][1] is not None and \
            #         grid_tile_path[i + 1][1] == path[1]:
            #     self._navigate_to_grid_tile(path, Stop.NONE)
            # else:
            self._navigate_to_grid_tile(path)

        return True

    def auto_home(self):
        if not self.mh_is_homed:
            return
        print('getting path to home')
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.home_tile)
        self._navigate_grid_tile_path(path)
        print('homed')

    def auto_load(self):
        if not self.mh_is_homed:
            return
        print('getting path to load')
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.load_tile)
        self._navigate_grid_tile_path(path)
        self._do_load_()

    def auto_unload(self):
        if not self.mh_is_homed:
            return
        print('getting path to unload')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.unload_tile)
        self._navigate_grid_tile_path(path)
        self._do_unload_()

    def _bfs_path_to_grid_tile(self, start_tile: tuple[int, int], end_tile: tuple[int, int]) -> list[
        tuple[int, int]]:
        print("---bfs_path_to_grid_tile---")
        self._display_grid_()
        print("--start",start_tile)
        print("--end",end_tile)
        print("--grid_tracks",self.grid_tracks)
        print("--home_tile",self.home_tile)
        print("--load_tile",self.load_tile)
        print("--unload_tile",self.unload_tile)
        mem_info()
        queue: Queue = Queue()
        queue.put([start_tile])  # Enqueue the start position

        path = []
        visited = [start_tile]
        while not queue.empty():
            path = queue.get()  # Dequeue the path
            current_path = path[-1]  # Current position is the last element of the path
            print(path)
            if current_path[0] == end_tile:
                break

            for direction in [_EAST, _NORTH, _WEST, _SOUTH]:  # Possible movements
                new_pos = position_from_direction(current_path, direction)
                if new_pos in visited:
                    continue
                if  new_pos in self.grid_tracks or new_pos == self.home_tile:
                    visited.append(new_pos)
                    new_path = list(path)
                    new_path.append(new_pos)
                    queue.put(new_path)  # Enqueue the new path
        if len(path) == 0:
            print("no path found")
        mem_info()
        print("---bfs_path_to_grid_tile---")
        return path[1:]

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        if self.mh__remote_disabled:
            return
        # Check which remote_buttons are pressed.
        remote_buttons_pressed = remote.buttons.pressed()
        #  handle button press
        # left +      North
        # right - West    East  right +
        # left -      South
        if len(remote_buttons_pressed) == 0 or Button.RIGHT in remote_buttons_pressed or Button.LEFT in remote_buttons_pressed:
            self.stop_motors()
            return

        fine_grid_pos = self._get_fine_grid_position_()
        if fine_grid_pos != self.last_fine_grid_position:
            self.last_fine_grid_position = fine_grid_pos
        elif self.motors_running:
            return

        direction = None
        if Button.LEFT_PLUS in remote_buttons_pressed and Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = _NORTH_EAST
        elif Button.LEFT_PLUS in remote_buttons_pressed and Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = _NORTH_WEST
        elif Button.LEFT_MINUS in remote_buttons_pressed and Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = _SOUTH_EAST
        elif Button.LEFT_MINUS in remote_buttons_pressed and Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = _SOUTH_WEST
        elif Button.LEFT_PLUS in remote_buttons_pressed:
            direction = _NORTH
        elif Button.LEFT_MINUS in remote_buttons_pressed:
            direction = _SOUTH
        elif Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = _EAST
        elif Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = _WEST

        self.stop_motors()

        if direction not in [_NORTH, _NORTH_EAST, _EAST, _SOUTH_EAST, _SOUTH, _SOUTH_WEST, _WEST, _NORTH_WEST]:
            print('Invalid direction')
            return

        # print(direction)
        can_move, can_load, can_unload = self._can_move_in_direction_(direction)
        if can_load and direction == _WEST:
            self._do_load_()
            return
        if can_unload and direction == _EAST:
            self._do_unload_()
            return

        if not can_move:
            return

        self._move_in_direction_(direction)

    # stop all motors
    def stop_motors(self):
        self.motor_x.stop()
        self.motor_y.stop()
        self.motors_running = False




def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    setup_hub()
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END

        drive_motors.mh__remote_disabled = REMOTE_DISABLED


        if REMOTE_DISABLED:
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
            if not REMOTE_DISABLED:
                countdown_timer.check_remote_buttons()

            if ODV_AUTO_DRIVE_TIMEOUT_SECS > 0 and countdown_timer.remote_button_press_timed_out():
                    drive_motors.enable_auto_drive()

            if drive_motors.mh_auto_drive and drive_motors.mh_is_homed:
                drive_motors.auto_unload()
                drive_motors.auto_load()
            # if there is no remote, then there is no point in a countdown
            elif countdown_timer.has_time_remaining() or REMOTE_DISABLED:
                if drive_motors.mh_supports_homing:
                    drive_motors.do_homing()
                if drive_motors.mh_supports_flip:
                    drive_motors.handle_flip()
                if not REMOTE_DISABLED:
                    drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()
                if drive_motors.mh_supports_homing:
                    drive_motors.auto_unload()
                    drive_motors.auto_home()
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)

            if REMOTE_DISABLED and ODV_AUTO_DRIVE_TIMEOUT_SECS == 0:
                print("No remote or auto drive exiting")
                raise SystemExit

    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
