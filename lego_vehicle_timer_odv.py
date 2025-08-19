# Timed train and vehicle program for interactive displays
# Copyright Etendut
# licence MIT
import time

from micropython import const
from pybricks.parameters import Color, Side, Button
from pybricks.pupdevices import Remote
from pybricks.tools import wait, StopWatch

from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from uerrno import ENODEV
from umath import floor


print('Version 1.4.0')
##################################################################################
#  Settings
##################################################################################

# countdown time settings
COUNTDOWN_LIMIT_MINUTES: int = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button


# odv settings
ODV_SPEED: int = const(45)  # set between 40 and 70
# X= obstacle, L = Load, U = Unload, # = grid tile
# ODV_GRID = ["###X#XX",
#             "LX###XU",
#             "###X###"]
ODV_GRID = ["###X", "LX#U", "###X"]


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
        self.supports_flip = supports_flip
        self.supports_homing = supports_homing

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


READY: int = const(0)
ACTIVE: int = const(10)
FINAL_MINUTE: int = const(20)
FINAL_20_SECS: int = const(30)
ENDED: int = const(40)
UNKNOWN: int = const(99)


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.remote_buttons_last_pressed:int = 0
        self.last_countdown_message:str = ''
        self.countdown_status:int = UNKNOWN

        # Start a timer.
        self.countdown_stopwatch = StopWatch()
        self.led_flash_stopwatch = StopWatch()
        self.end_time = 0

    def has_time_remaining(self):
        """
            Checks if countdown has time remaining
        :return:
        """
        if self.countdown_status == ENDED or self.countdown_status == READY:
            return False

        # calculate remaining_time time
        remaining_time = self.end_time - self.countdown_stopwatch.time()

        # print a friendly console message
        con_hour, con_min, con_sec = convert_millis_hours_minutes_seconds(int(remaining_time))

        if con_sec % 10 == 0 and con_min < 1:
            countdown_message = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
            if self.last_countdown_message != countdown_message:
                self.last_countdown_message = countdown_message
                print(self.last_countdown_message)  # when time has run out end countdown
        if remaining_time <= 0:
            self.countdown_status = ENDED
            self.show_status()
            return False
        # in last 25s slow flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = FINAL_20_SECS
            self.show_status()

            # in last minute slow flash a warning
        if remaining_time < (1000 * 60):
            self.countdown_status = FINAL_MINUTE
            self.show_status()

        return True

    def __start_countdown__(self):
        """
            start the countdown sequence by resetting timers and status
        """
        print('start countdown')
        self.countdown_status = ACTIVE
        self.show_status()
        self.countdown_stopwatch.reset()
        self.end_time = self.countdown_stopwatch.time() + (COUNTDOWN_LIMIT_MINUTES * 60 * 1000)

    def reset(self):
        print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = READY

    def check_remote_buttons(self):
        """
            check countdown time buttons
        """
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return

        self.remote_buttons_last_pressed = int(time.time())

        if self.countdown_status == READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def check_reset_code_pressed(self):
        """
           Check if reset code was pressed
        """
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return
        # if reset sequence pressed at other times, end countdown
        if self.countdown_status != READY and all(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
            i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            print('reset code pressed')
            self.reset()
            wait_for_no_pressed_buttons()

    def show_status(self):
        global hub
        global remote
        if self.countdown_status == READY:
            self.__flash_remote_and_hub_light__(Color.GREEN, 500, Color.NONE, 500)
        elif self.countdown_status == ACTIVE:
            hub.light.on(Color.GREEN)
            remote.light.on(Color.GREEN)
        elif self.countdown_status == FINAL_20_SECS:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 200, Color.NONE, 100)
        elif self.countdown_status == FINAL_MINUTE:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 500, Color.NONE, 250)
        elif self.countdown_status == ENDED:
            hub.light.on(Color.ORANGE)
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
        if self.led_flash_stopwatch.time() > (on_msec + off_msec):
            self.led_flash_stopwatch.reset()
            remote.light.on(off_color)
            hub.light.on(off_color)
        elif self.led_flash_stopwatch.time() > off_msec:
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
class MockLight:
    """For typing only, this will be replaced by CityHub or TechnicHub"""

    def blink(self, col, seq):
        pass

    def on(self, color):
        pass


class MockIMU:
    """For typing only, this will be replaced by TechnicHub"""

    def up(self) -> Side:
        pass


class MockHub:
    """For typing only, this will be replaced by CityHub or TechnicHub"""

    def __init__(self):
        self.imu = MockIMU()
        self.light = MockLight()


hub: MockHub = MockHub()


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


remote: Remote | None = None
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
            # Connect to the remote.
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
NORTH_WEST = 'NW'
NORTH = 'N'
NORTH_EAST = 'NE'
EAST = 'E'
SOUTH_EAST = 'SE'
SOUTH = 'S'
SOUTH_WEST = 'SW'
WEST = 'W'

DIRECTIONS = [NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, NORTH_WEST]

ROBOT = "R"
WALL = 'X'
TRACK = '#'
HOME = 'H'
LOAD = 'L'
UNLOAD = 'U'
OK_MOVES = [TRACK, LOAD, UNLOAD]

DEFAULT_GRID = ["H######", "###X#XX", "LX###XU", "###X###"]
FINE_GRID_SIZE = const(10)
ODV_SIZE = const(8)

GEAR_RATIO_TO_GRID: int = const(80)  # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED: int = const(1400)  # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED: int = const(200)  # Homing speed (deg/s)
HOMING_DUTY: int = const(45)  # Homing motor duty (%) (adjustment required)


class ODVPosition:
    def __init__(self, x:int, y:int):
        self.x = x
        self.y = y

    def position_from_direction(self, direction:str):
        if direction == NORTH:
            return ODVPosition(self.x, self.y - 1)
        if direction == NORTH_EAST:
            return ODVPosition(self.x + 1, self.y - 1)
        if direction == EAST:
            return ODVPosition(self.x + 1, self.y)
        if direction == SOUTH_EAST:
            return ODVPosition(self.x + 1, self.y + 1)
        if direction == SOUTH:
            return ODVPosition(self.x, self.y + 1)
        if direction == SOUTH_WEST:
            return ODVPosition(self.x - 1, self.y + 1)
        if direction == WEST:
            return ODVPosition(self.x - 1, self.y)
        if direction == NORTH_WEST:
            return ODVPosition(self.x - 1, self.y - 1)

        return ODVPosition(self.x, self.y)

    def value(self):
        return self.x, self.y

    def __str__(self):
        return f"({self.x}, {self.y})"

    def copy(self):
        return ODVPosition(self.x, self.y)

    def __eq__(self, other):
        return self.value() == other.value()

class ODVTilePosition(ODVPosition):
    def __init__(self, x:int, y:int):
        super().__init__(x, y)
        self.type = 'tile'
        self.direction = None

    def tile_path_from_direction(self, direction:str):
        """
        gets position_from_direction and adds direction property
        :param direction:
        :return:
        """
        pos = super().position_from_direction(direction)
        tp = ODVTilePosition(pos.x, pos.y)
        tp.direction = direction
        return tp


class ODVGridPosition(ODVPosition):
    def __init__(self, x:int, y:int):
        super().__init__(x, y)
        self.type = 'grid'

class ODVAnglePosition(ODVPosition):
    def __init__(self, x:int, y:int):
        super().__init__(x, y)
        self.type = 'angle'

class ODVBox:
    def __init__(self, top_left: ODVGridPosition, width: int, height: int):
        self.width = 0
        self.height = 0
        self.top_left: ODVGridPosition
        self.top_right: ODVGridPosition
        self.bottom_right: ODVGridPosition
        self.bottom_left: ODVGridPosition
        self.upper_left: ODVGridPosition
        self._update_dimensions_(top_left, width, height)

    def _update_dimensions_(self, top_left: ODVGridPosition, width: int, height: int):
        self.width = width
        self.height = height
        self.top_left = top_left
        self.top_right = ODVGridPosition(self.top_left.x + self.width, self.top_left.y)
        self.bottom_right = ODVGridPosition(self.top_right.x, self.top_left.y + self.height)
        self.bottom_left = ODVGridPosition(self.top_left.x, self.bottom_right.y)

    def buffer(self, buffer: int):
        new_tl = ODVGridPosition(self.top_left.x - buffer, self.top_left.y - buffer)
        self._update_dimensions_(new_tl, self.width + (buffer * 2), self.height + (buffer * 2))

    def __str__(self):
        return f"[{self.top_left}, {self.top_right}]\n[{self.bottom_left}, {self.bottom_right}]"


class Queue:
    """ No Queue in micropython :("""

    def __init__(self) -> None:
        self._queue: list[list[ODVTilePosition]] = []

    def put(self, item: list[ODVTilePosition]):
        self._queue.append(item)

    def empty(self):
        return len(self._queue) == 0

    def get(self) -> list[ODVTilePosition]:
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
        self.auto_mode = False
        self.motors_running = None
        self.home_tile: ODVTilePosition
        self.unload_tile: ODVTilePosition
        self.load_tile: ODVTilePosition
        self.last_fine_grid_position = ODVGridPosition(0, 0)
        """current position"""
        self.coarse_grid = {}
        self.coarse_grid_width = 0
        self.coarse_grid_height = 0
        self._load_grid_(grid_layout)

        self.has_load = False
        # motor setup
        self.error_flash_code = error_flash_code_helper

        self.x_motor_port = Port.A
        self.y_motor_port = Port.C

        self.drive_speed = drive_speed
        self.is_homed = False

        try:
            self.x_motor = Motor(self.x_motor_port, Direction.COUNTERCLOCKWISE)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.x_motor_port))
                self.error_flash_code.set_error_no_motor_on_a()
            raise
        try:
            self.y_motor = Motor(self.y_motor_port, Direction.CLOCKWISE)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.y_motor_port))
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
            self.coarse_grid_width = len(line.rstrip())
            line = line.rstrip()
            for x, character in enumerate(line):
                print(f"line {y + 1} |col {x + 1}/{len(line)}")
                self.coarse_grid[x, y] = character
                # set load/unload points
                if character == HOME:
                    self.home_tile = ODVTilePosition(x, y)
                if character == LOAD:
                    self.load_tile = ODVTilePosition(x, y)
                if character == UNLOAD:
                    self.unload_tile = ODVTilePosition(x, y)

            y += 1
        self.coarse_grid_height = y
        mem_info()
        print('Grid Loaded')

    def _display_grid_(self, position: ODVTilePosition, robot_symbol: str):
        # Display the maze:
        for y in range(self.coarse_grid_height):
            for x in range(self.coarse_grid_width):
                if (x, y) == position.value():
                    print(robot_symbol, end='')
                elif (x, y) == self.home_tile.value():
                    print(HOME, end='')
                elif (x, y) == self.load_tile.value():
                    print(LOAD, end='')
                elif (x, y) == self.unload_tile.value():
                    print(UNLOAD, end='')
                elif self.coarse_grid[(x, y)] == WALL:
                    print(WALL, end='')
                else:
                    print(self.coarse_grid[(x, y)], end='')
            print()  # Print a newline after printing the row.

    def reset_homing(self) -> None:
        self.is_homed = False

    def do_homing(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then move forward by an offset distance and set that as the zero origin.
        if self.is_homed:
            return
        # Homing axis Y
        self.y_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        home_tile_angle = self._tile_to_angle(self.home_tile)
        self.y_motor.reset_angle(home_tile_angle.y)
        self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, GEAR_RATIO_TO_GRID)
        wait(200)

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.reset_angle(home_tile_angle.x)
        self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, GEAR_RATIO_TO_GRID)
        wait(200)

        self.is_homed = True
        self._display_grid_(self.home_tile, ROBOT)

    def _can_move_in_direction_(self, direction: str) -> tuple[bool, bool, bool]:
        if direction not in DIRECTIONS:
            return False, False, False

        # work out cart dimensions
        cart = ODVBox(self.last_fine_grid_position, ODV_SIZE, ODV_SIZE)
        # shrink the cart to make moving smoother
        cart.buffer(-1)

        # print("Cart", cart)

        _, tl = self._get_grid_tile_from_fine_xy_(cart.top_left.position_from_direction(direction))
        _, tr = self._get_grid_tile_from_fine_xy_(cart.top_right.position_from_direction(direction))
        _, br = self._get_grid_tile_from_fine_xy_(cart.bottom_right.position_from_direction(direction))
        _, bl = self._get_grid_tile_from_fine_xy_(cart.bottom_left.position_from_direction(direction))

        can_move = tl in OK_MOVES and tr in OK_MOVES and br in OK_MOVES and bl in OK_MOVES

        can_load = tl == tr == br == bl == LOAD
        can_unload = tl == tr == br == bl == UNLOAD

        # print("Cart", direction, can_move)
        return can_move, can_load, can_unload

    def _get_fine_grid_position_(self) -> ODVGridPosition:

        x_grid = int(self.x_motor.angle() / GEAR_RATIO_TO_GRID)
        y_grid = int(self.y_motor.angle() / GEAR_RATIO_TO_GRID)
        return ODVGridPosition(x_grid, y_grid)

    def _get_grid_tile_from_fine_xy_(self, fine_position: ODVGridPosition) -> tuple[ODVTilePosition, str]:

        x_grid = floor(fine_position.x / FINE_GRID_SIZE)
        y_grid = floor(fine_position.y / FINE_GRID_SIZE)
        # print("Coarse", x_grid, y_grid)
        tile = ODVTilePosition(x_grid, y_grid)
        if fine_position.x < 1 or fine_position.y < 1 or x_grid < 0 or y_grid < 0 or x_grid > self.coarse_grid_width or y_grid > self.coarse_grid_height:
            return tile, WALL
        if (x_grid, y_grid) in self.coarse_grid:
            return tile, self.coarse_grid[(x_grid, y_grid)]

        return tile, WALL

    def _move_in_direction_(self, direction: str) -> bool:

        if direction not in DIRECTIONS:
            print('Invalid direction')
            return False

        if direction in [NORTH, NORTH_EAST, NORTH_WEST]:
            self.y_motor.dc(-self.drive_speed)
        if direction in [SOUTH, SOUTH_EAST, SOUTH_WEST]:
            # self.y_motor.run_target(self.drive_speed, -90)
            self.y_motor.dc(self.drive_speed)

        if direction in [EAST, NORTH_EAST, SOUTH_EAST]:
            # self.x_motor.run_target(self.drive_speed, 90)
            self.x_motor.dc(self.drive_speed)
        if direction in [WEST, NORTH_WEST, SOUTH_WEST]:
            self.x_motor.dc(-self.drive_speed)

        self.motors_running = True
        return True

    def _do_load_(self):
        if self.has_load:
            print('Already loaded')
            return

        self._navigate_to_grid_tile(self.load_tile)
        wait(200)
        # do load
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, (GEAR_RATIO_TO_GRID * -3))
        wait(2000)
        print("loading..")
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, 0)
        wait(200)
        self.has_load = True
        print("ready to go")

    def _do_unload_(self):
        tile_angle = self._navigate_to_grid_tile(self.unload_tile)
        wait(200)
        print("unloading..")
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle.x + (GEAR_RATIO_TO_GRID * 4))
        wait(2000)
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle.x)
        wait(200)
        self.has_load = False
        print("ready to go")

    @staticmethod
    def _tile_to_angle(tile: ODVTilePosition) -> ODVAnglePosition:
        """
        Convert grid tile to angle
        :param tile: ODVTilePosition
        :return: ODVAnglePosition
        """
        tile_angle_x = tile.x * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID
        tile_angle_y = (tile.y * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID) + GEAR_RATIO_TO_GRID
        return ODVAnglePosition(tile_angle_x, tile_angle_y)

    def _navigate_to_grid_tile(self, tile: ODVTilePosition, stop=Stop.HOLD) -> ODVAnglePosition:
        print(f"navigating to tile {tile}")
        tile_angle_x = tile.x * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID
        tile_angle_y = (tile.y * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID) + GEAR_RATIO_TO_GRID
        self.y_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle_y, then=stop)
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle_x, then=stop)
        return ODVAnglePosition(tile_angle_x, tile_angle_y)

    def _navigate_grid_tile_path(self, grid_tile_path: list[ODVTilePosition]):
        """
        Navigates robot through list of ODVTilePosition
        :param grid_tile_path:
        """
        for i, p in enumerate(grid_tile_path):
            # if user takes over break
            if self.auto_mode and  len(remote.buttons.pressed()) > 0:
                self.auto_mode = False
                break                
            if p.direction is not None and i < (len(grid_tile_path) - 1) and grid_tile_path[
                i + 1].direction is not None and grid_tile_path[i + 1].direction == p.direction:
                self._navigate_to_grid_tile(p, Stop.NONE)
            else:
                self._navigate_to_grid_tile(p)

    def auto_home(self):
        if not self.is_homed:
            return
        print('getting path to home')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.home_tile)
        self._navigate_grid_tile_path(path)
        print('homed')

    def auto_load(self):
        if not self.is_homed:
            return
        print('getting path to load')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.load_tile)
        self._navigate_grid_tile_path(path)
        self._do_load_()

    def auto_unload(self):
        if not self.is_homed:
            return
        print('getting path to unload')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.unload_tile)
        self._navigate_grid_tile_path(path)
        self._do_unload_()

    def _bfs_path_to_grid_tile(self, start_tile: ODVTilePosition, end_tile: ODVTilePosition):
        queue: Queue = Queue()
        queue.put([start_tile])  # Enqueue the start position

        while not queue.empty():
            path = queue.get()  # Dequeue the path
            current_pos = path[-1]  # Current position is the last element of the path

            if current_pos == end_tile:
                return path  # Return the path if end is reached

            for direction in [EAST, NORTH, WEST, SOUTH]:  # Possible movements
                new_pos = current_pos.tile_path_from_direction(direction)
                if new_pos.value() in self.coarse_grid and self.coarse_grid[new_pos.value()] in OK_MOVES:
                    new_path = list(path)
                    new_path.append(new_pos)
                    queue.put(new_path)  # Enqueue the new path
        return []

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
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
            self.last_fine_grid_position = fine_grid_pos.copy()
        elif self.motors_running:
            return

        direction = None
        if Button.LEFT_PLUS in remote_buttons_pressed and Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = NORTH_EAST
        elif Button.LEFT_PLUS in remote_buttons_pressed and Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = NORTH_WEST
        elif Button.LEFT_MINUS in remote_buttons_pressed and Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = SOUTH_EAST
        elif Button.LEFT_MINUS in remote_buttons_pressed and Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = SOUTH_WEST
        elif Button.LEFT_PLUS in remote_buttons_pressed:
            direction = NORTH
        elif Button.LEFT_MINUS in remote_buttons_pressed:
            direction = SOUTH
        elif Button.RIGHT_PLUS in remote_buttons_pressed:
            direction = EAST
        elif Button.RIGHT_MINUS in remote_buttons_pressed:
            direction = WEST

        self.stop_motors()

        if direction not in DIRECTIONS:
            print('Invalid direction')
            return

        # print(direction)
        can_move, can_load, can_unload = self._can_move_in_direction_(direction)
        if can_load and direction == WEST:
            self._do_load_()
            return
        if can_unload and direction == EAST:
            self._do_unload_()
            return

        if not can_move:
            return

        self._move_in_direction_(direction)

    # stop all motors
    def stop_motors(self):
        self.x_motor.stop()
        self.y_motor.stop()
        self.motors_running = False




def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END


        print('--setup remote')
        setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        while True:
            countdown_timer.check_remote_buttons()
            if countdown_timer.has_time_remaining():
                if drive_motors.supports_homing:
                    drive_motors.do_homing()
                if drive_motors.supports_flip:
                    drive_motors.handle_flip()
                drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()
                if drive_motors.supports_homing:
                    drive_motors.auto_unload()
                    drive_motors.auto_home()
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)
    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
