# Timed train and vehicle program for interactive displays
# Copyright Etendut
# licence MIT

from micropython import const
from pybricks.parameters import Color, Side, Button
from pybricks.pupdevices import Remote
from pybricks.tools import wait, StopWatch
from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import  Port, Direction
from uerrno import ENODEV


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
ODV_SPEED: int = 60  # set between 50 and 80
# X= obstacle, L = Load, U = Unload, # = grid tile
ODV_GRID = ["###X#XX", "LX###XU", "###X###"]


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
        pass

    def handle_homing(self):
        pass

    def handle_remote_press(self):
        pass

    def stop_motors(self):
        pass




##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    buttons_pressed = remote.buttons.pressed()
    while buttons_pressed:
        buttons_pressed = remote.buttons.pressed()
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


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.countdown_status = None

        # Start a timer.
        self.countdown_stopwatch = StopWatch()
        self.led_flash_stopwatch = StopWatch()
        self.last_console_msg = None
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
        msg = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
        if self.last_console_msg != msg:
            self.last_console_msg = msg
            print(msg)  # when time has run out end countdown
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
        pressed = remote.buttons.pressed()

        if self.countdown_status == READY and Button.CENTER in pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def check_reset_code_pressed(self):
        """
           Check if reset code was pressed
        """
        pressed = remote.buttons.pressed()
        # if reset sequence pressed at other times, end countdown
        if self.countdown_status != READY and all(i in pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            print('reset code pressed')
            self.reset()
            wait_for_no_pressed_buttons()

    def show_status(self):
        global hub
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
LOAD = 'L'
UNLOAD = 'U'

DEFAULT_GRID = ["###X#XX", "LX###XU", "###X###"]
FINE_GRID_SIZE = 10

HOMING_MOTOR_ROT_SPEED: int = 200  # Homing speed (deg/s)
HOMING_DUTY: int = 35  # Homing motor duty (%) (adjustment required)
HOMING_OFFSET_ANGLE_X: int = 110  # X-axis offset distance (deg) (adjustment required)
HOMING_OFFSET_ANGLE_Y: int = 110  # Y-axis offset distance (deg) (adjustment required)

class ODVPosition:
    def __init__(self, x: int, y: int, direction: str = None):
        self.x = x
        self.y = y
        self.direction = direction

    def value(self) -> tuple[int, int]:
        return self.x, self.y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class ODVGrid:
    def __init__(self):
        self.coarse_grid = {}
        self.grid ={}
        self.home = ODVPosition(5, 5)
        self.load_xy = None
        self.unload_xy = None
        self.width = 0
        self.height = 0

    def load_grid(self, lines: list[str]):
        # loop through grid lines
        y = 0
        print('Loading grid')
        mem_info()
        for y, line in enumerate(lines):
            print(f"line {y + 1}/{len(lines)}")
            self.width = len(line.rstrip())
            line = line.rstrip()
            for x, character in enumerate(line):
                print(f"line {y + 1} |col {x + 1}/{len(line)}")
                self.grid[x, y] = character
                # set load/unload points
                if character == LOAD:
                    self.load_xy = x, y
                if character == UNLOAD:
                    self.unload_xy = x, y

            y += 1
        self.height = y

        assert self.width > 1
        assert self.height > 1
        assert self.load_xy is not None
        assert self.unload_xy is not None
        mem_info()
        print('Grid Loaded')

    def can_unload(self, position: ODVPosition):
        return False
        # return position.value() == self.unload_xy.value()

    def can_load(self, position: ODVPosition):
        return False
        # return position.value() == self.load_xy.value()

    def can_move(self, new_position: ODVPosition):
        """
        Can robot move to grid position
        :param new_position:
        :return:
        """
        return True
        # if self.width < new_position.x < -1 or self.height < new_position.y < -1:
        #     return False
        # grid_box = self.fine_grid[new_position.value()]
        # return grid_box.box_type in [LOAD, UNLOAD, TRACK]

    def display(self, position: ODVPosition, robot_symbol: str):
        # Display the maze:
        for y in range(self.height):
            for x in range(self.width):
                if (x, y) == position.value():
                    print(robot_symbol, end='')
                elif (x, y) == self.load_xy.value():
                    print(LOAD, end='')
                elif (x, y) == self.unload_xy.value():
                    print(UNLOAD, end='')
                elif self.coarse_grid[(x, y)] == WALL:
                    print(WALL, end='')
                else:
                    print(self.coarse_grid[(x, y)], end='')
            print()  # Print a newline after printing the row.


class RunODVMotors(MotorHelper):
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, grid_layout: list[str]):

        super().__init__(False, True)
        # grid setup
        self.position = ODVPosition(0, 0)
        self.grid = ODVGrid()
        self.grid.load_grid(grid_layout)

        self.has_load = False
        # motor setup
        self.error_flash_code = error_flash_code_helper

        self.x_motor_port = Port.C
        self.y_motor_port = Port.A

        self.drive_speed = drive_speed
        self.last_side = None
        try:
            self.x_motor = Motor(self.x_motor_port, Direction.CLOCKWISE)
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

    def handle_homing(self):
        """
            odv only
        :return:
        """
        global hub
        # Check which side of the hub is up.
        if hub.imu.up() != Side.BOTTOM:
            return

        wait(2000)
        self._homing_()

    def _homing_(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then move forward by an offset distance and set that as the zero origin.

        # Homing axis Y
        self.y_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.y_motor.run_angle(self.drive_speed, HOMING_OFFSET_ANGLE_Y)
        wait(200)
        self.y_motor.reset_angle(0)

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.run_angle(self.drive_speed, HOMING_OFFSET_ANGLE_X)
        wait(200)
        self.x_motor.reset_angle(0)

        # todo this is wrong
        self.position = ODVPosition(0, 0)

    @staticmethod
    def _get_new_position_(position: ODVPosition, direction: str) -> ODVPosition:
        if direction not in DIRECTIONS:
            return position
        if direction == NORTH_WEST:
            return ODVPosition(position.x-1, position.y - 1, direction)
        if direction == NORTH:
            return ODVPosition(position.x, position.y - 1, direction)
        if direction == NORTH_EAST:
            return ODVPosition(position.x+1, position.y - 1, direction)
        if direction == EAST:
            return ODVPosition(position.x + 1, position.y, direction)
        if direction == SOUTH_EAST:
            return ODVPosition(position.x+1, position.y + 1, direction)
        if direction == SOUTH:
            return ODVPosition(position.x, position.y + 1, direction)
        if direction == SOUTH_WEST:
            return ODVPosition(position.x-1, position.y + 1, direction)
        if direction == WEST:
            return ODVPosition(position.x - 1, position.y, direction)
        return position

    def _can_move_(self, new_pos: ODVPosition):
        # if self.has_load and new_pos == self.grid.load_xy:
        #     print('you need to unload first')
        #     return False
        # if self.grid.can_move(new_pos):
        return True
        # print('cannot move there')
        # return False

    def _move_(self, direction: str):
        if direction not in DIRECTIONS:
            print('Invalid direction')
            self.stop_motors()
        #     return
        # new_position = self._get_new_position_(self.position, direction)
        # if new_position == self.position:
        #     print('no position change')
        #     return
        # if not self._can_move_(new_position):
        #     print('cannot move ' + direction)
        #     return

        # TODO fix motor movement
        if direction in [NORTH, NORTH_EAST, NORTH_WEST]:
            self.y_motor.dc(self.drive_speed)
        if direction in [SOUTH, SOUTH_EAST, SOUTH_WEST]:
            # self.y_motor.run_target(self.drive_speed, -90)
            self.y_motor.dc(-self.drive_speed)
        if direction in [EAST,NORTH_EAST,SOUTH_EAST]:
            # self.x_motor.run_target(self.drive_speed, 90)
            self.x_motor.dc(self.drive_speed)
        if direction in [WEST, NORTH_WEST, SOUTH_WEST]:
            self.x_motor.dc(-self.drive_speed)

        # self.position = new_position
        #
        # # handle load/unload
        # if self.position == self.grid.unload_xy:
        #     wait(2000)
        #     self.has_load = False
        #
        # if self.position == self.grid.load_xy:
        #     wait(2000)  # async?
        #     self.has_load = True

    # def _bfs_path_to_position(self, end: ODVPosition):
    #     queue: Queue[list[ODVPosition]] = Queue()
    #     queue.put([self.position])  # Enqueue the start position
    #
    #     while not queue.empty():
    #         path = queue.get()  # Dequeue the path
    #         current_pos = path[-1]  # Current position is the last element of the path
    #
    #         if current_pos == end:
    #             return path  # Return the path if end is reached
    #
    #         for direction in [EAST, NORTH, WEST, SOUTH]:  # Possible movements
    #             new_pos = self._get_new_position_(current_pos, direction)
    #             if self._can_move_(new_pos) and new_pos not in path:
    #                 new_path = list(path)
    #                 new_path.append(new_pos)
    #                 queue.put(new_path)  # Enqueue the new path
    #     return []

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        # Check which remote_buttons are pressed.
        remote_buttons = remote.buttons.pressed()

        # stop motors as this is bang-bang mode where a button
        #  needs to be held down for racer to run
        self.stop_motors()

        #  handle button press
        # left +      North
        # right - West    East  right +
        # left -      South

        if Button.LEFT_PLUS in remote_buttons and Button.RIGHT_PLUS in remote_buttons:
            self._move_(NORTH_EAST)
        elif Button.LEFT_PLUS in remote_buttons and Button.RIGHT_MINUS in remote_buttons:
            self._move_(NORTH_WEST)
        elif Button.LEFT_MINUS in remote_buttons and Button.RIGHT_PLUS in remote_buttons:
            self._move_(SOUTH_EAST)
        elif Button.LEFT_MINUS in remote_buttons and Button.RIGHT_MINUS in remote_buttons:
            self._move_(SOUTH_WEST)
        elif Button.LEFT_PLUS in remote_buttons:
            self._move_(NORTH)
        elif Button.LEFT_MINUS in remote_buttons:
            self._move_(SOUTH)
        elif Button.RIGHT_PLUS in remote_buttons:
            self._move_(EAST)
        elif Button.RIGHT_MINUS in remote_buttons:
            self._move_(WEST)

    # stop all motors
    def stop_motors(self):
        # unload the module
        # if self.has_load:
        #     unload_moves = self._bfs_path_to_position(self.grid.unload_xy)
        #     for move in unload_moves:
        #         self._move_(move.direction)
        #
        # if self.position != self.grid.home:
        #     # return to home
        #     homing_moves = self._bfs_path_to_position(self.grid.home)
        #     for move in homing_moves:
        #         self._move_(move.direction)

        self.x_motor.stop()
        self.y_motor.stop()


def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)


        print('--setup remote')
        setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        while True:
            countdown_timer.check_remote_buttons()
            if countdown_timer.has_time_remaining():
                if drive_motors.supports_flip:
                    drive_motors.handle_flip()
            else:
                drive_motors.stop_motors()
                if drive_motors.supports_homing:
                    drive_motors.handle_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(100)
    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
