# IMPORTS_START
from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from uerrno import ENODEV
from umath import floor
# IMPORTS_END

# local var only
from pybricks.tools import wait

from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Remote

hub: TechnicHub | None = None
remote: Remote | None = None
from pybricks.parameters import Button

# VARS_START
# odv settings
ODV_SPEED: int = 50  # set between 50 and 80
# X= obstacle, L = Load, U = Unload, # = grid tile
ODV_GRID = ["###X#XX", "LX###XU", "###X###"]
# VARS_END
# MODULE_START
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
ODV_SIZE = 8

GEAR_RATIO: int = 80  # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED: int = 1400  # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED: int = 200  # Homing speed (deg/s)
HOMING_DUTY: int = 35  # Homing motor duty (%) (adjustment required)
HOMING_OFFSET_ANGLE_X: int = 110  # X-axis offset distance (deg) (adjustment required)
HOMING_OFFSET_ANGLE_Y: int = 110  # Y-axis offset distance (deg) (adjustment required)


class RunODVMotors(MotorHelper):
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, grid_layout: list[str]):

        super().__init__(False, True)
        # grid setup
        self.unload_xy = (0, 0)
        self.load_xy = (0, 0)
        self.grid_width = 0
        self.grid_height = 0
        self.angular_grid_position = (0, 0)
        self.grid = {}
        self.load_grid(grid_layout)

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

    def load_grid(self, lines: list[str]):
        # loop through grid lines
        y = 0
        print('Loading grid')
        mem_info()
        for y, line in enumerate(lines):
            print(f"line {y + 1}/{len(lines)}")
            self.grid_width = len(line.rstrip())
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
        self.grid_height = y
        mem_info()
        print('Grid Loaded')

    def display_grid(self, position: tuple[int, int], robot_symbol: str):
        # Display the maze:
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if (x, y) == position:
                    print(robot_symbol, end='')
                elif (x, y) == self.load_xy:
                    print(LOAD, end='')
                elif (x, y) == self.unload_xy:
                    print(UNLOAD, end='')
                elif self.grid[(x, y)] == WALL:
                    print(WALL, end='')
                else:
                    print(self.grid[(x, y)], end='')
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
        self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_Y)
        wait(200)
        self.y_motor.reset_angle(0)

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_X)
        wait(200)
        self.x_motor.reset_angle(0)

        self.angular_grid_position = (0, 0)
        self.is_homed = True
        self.display_grid((0, 0), ROBOT)

    def _can_move_(self, direction: str) -> bool:
        if direction not in DIRECTIONS:
            return False
        # get angular grid pos
        x_pos = self.angular_grid_position[0]
        y_pos = self.angular_grid_position[1]

        # work out cart dimensions
        top_left = (x_pos, y_pos)
        top_right = (x_pos + ODV_SIZE, y_pos)
        bottom_right = (x_pos + ODV_SIZE, y_pos + ODV_SIZE)
        bottom_left = (x_pos, y_pos + ODV_SIZE)
        print("Cart", top_left, top_right, bottom_right, bottom_left)

        can_move = False
        ok_moves = [TRACK, LOAD, UNLOAD]
        # if direction == NORTH_WEST:
        #     x_tile = self.get_grid_tile(x_pos - 1, y_pos)
        #     y_tile = self.get_grid_tile(x_pos - 1, y_pos)

        if direction == NORTH:
            tl = self.get_grid_tile(top_left[0], top_left[1] - 1)
            tr = self.get_grid_tile(top_right[0], top_right[1] - 1)
            can_move = tl in ok_moves and tr in ok_moves

        # if direction == NORTH_EAST:
        #     return ODVPosition(position[0] + 1, position[1] - 1, direction)
        if direction == EAST:
            tr = self.get_grid_tile(top_right[0] + 1, top_right[1])
            br = self.get_grid_tile(bottom_right[0] + 1, bottom_right[1])
            can_move = tr in ok_moves and br in ok_moves  # if direction == SOUTH_EAST:
        #     return ODVPosition(position[0] + 1, position[1] + 1, direction)
        if direction == SOUTH:
            br = self.get_grid_tile(bottom_right[0] + 1, bottom_right[1])
            bl = self.get_grid_tile(bottom_left[0] + 1, bottom_left[1])
            can_move = bl in ok_moves and br in ok_moves

        # if direction == SOUTH_WEST:
        #     return ODVPosition(position[0] - 1, position[1] + 1, direction)
        if direction == WEST:
            tl = self.get_grid_tile(top_left[0] - 1, top_left[1])
            bl = self.get_grid_tile(bottom_left[0] - 1, bottom_left[1])
            can_move = bl in ok_moves and tl in ok_moves

        print("Cart", direction, can_move)
        return can_move

    def get_angular_grid_position(self) -> tuple[int, int]:

        x_grid = int(self.x_motor.angle() / GEAR_RATIO)
        y_grid = int(self.y_motor.angle() / GEAR_RATIO)
        return x_grid, y_grid

    def get_grid_tile(self, angular_position_x: int, angular_position_y: int) -> str:

        x_grid = floor(angular_position_x / FINE_GRID_SIZE)
        if x_grid < 0:
            x_grid = 1
        y_grid = floor(angular_position_y / FINE_GRID_SIZE)
        if y_grid < 1:
            y_grid = 1
        try:
            return self.grid[x_grid, y_grid]
        except IndexError:
            return WALL

    def _move_(self, direction: str):
        if direction not in DIRECTIONS:
            print('Invalid direction')
            self.stop_motors()
        #     return
        angular_pos = self.get_angular_grid_position()
        if angular_pos != self.angular_grid_position:
            self.angular_grid_position = angular_pos

        if not self._can_move_(direction):
            return

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


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END
