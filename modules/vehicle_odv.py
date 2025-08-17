# IMPORTS_START
from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from uerrno import ENODEV
from umath import floor
# IMPORTS_END

# local var only
from pybricks.tools import wait
from micropython import const
from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Remote

hub: TechnicHub | None = None
remote: Remote | None = None
from pybricks.parameters import Button

# VARS_START
# odv settings
ODV_SPEED: int = const(45)  # set between 40 and 70
# X= obstacle, L = Load, U = Unload, # = grid tile
# ODV_GRID = ["###X#XX",
#             "LX###XU",
#             "###X###"]
ODV_GRID = ["###X", "LX#U", "###X"]
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
HOME = 'H'
LOAD = 'L'
UNLOAD = 'U'
OK_MOVES = [TRACK, LOAD, UNLOAD]

DEFAULT_GRID = ["H######","###X#XX", "LX###XU", "###X###"]
FINE_GRID_SIZE = const(10)
ODV_SIZE = const(8)

GEAR_RATIO_TO_GRID: int = const(80)  # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED: int = const(1400)  # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED: int = const(200)  # Homing speed (deg/s)
HOMING_DUTY: int = const(45)  # Homing motor duty (%) (adjustment required)


class ODVPosition:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.direction = None

    def position_from_direction(self, direction):
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


class ODVBox:
    def __init__(self, top_left: ODVPosition, width: int, height: int):
        self.width = 0
        self.height = 0
        self.top_left: ODVPosition
        self.top_right: ODVPosition
        self.bottom_right: ODVPosition
        self.bottom_left: ODVPosition
        self.upper_left: ODVPosition
        self._update_dimensions_(top_left, width, height)

    def _update_dimensions_(self, top_left: ODVPosition, width: int, height: int):
        self.width = width
        self.height = height
        self.top_left = top_left
        self.top_right = ODVPosition(self.top_left.x + self.width, self.top_left.y)
        self.bottom_right = ODVPosition(self.top_right.x, self.top_left.y + self.height)
        self.bottom_left = ODVPosition(self.top_left.x, self.bottom_right.y)

    def buffer(self, buffer: int):
        new_tl = ODVPosition(self.top_left.x - buffer, self.top_left.y - buffer)
        self._update_dimensions_(new_tl, self.width + (buffer * 2), self.height + (buffer * 2))

    def __str__(self):
        return f"[{self.top_left}, {self.top_right}]\n[{self.bottom_left}, {self.bottom_right}]"


class Queue:
    """ No Queue in micropython :("""

    def __init__(self) -> None:
        self._queue: list[list[ODVPosition]] = []

    def put(self, item: list[ODVPosition]):
        self._queue.append(item)

    def empty(self):
        return len(self._queue) == 0

    def get(self) -> list[ODVPosition]:
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
        self.home_tile: ODVPosition
        self.unload_tile: ODVPosition
        self.load_tile: ODVPosition
        self.last_fine_grid_position: ODVPosition
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
                    self.home_tile = ODVPosition(x, y)
                if character == LOAD:
                    self.load_tile = ODVPosition(x, y)
                if character == UNLOAD:
                    self.unload_tile = ODVPosition(x, y)

            y += 1
        self.coarse_grid_height = y
        mem_info()
        print('Grid Loaded')

    def _display_grid_(self, position: ODVPosition, robot_symbol: str):
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

    def _get_fine_grid_position_(self) -> ODVPosition:

        x_grid = int(self.x_motor.angle() / GEAR_RATIO_TO_GRID)
        y_grid = int(self.y_motor.angle() / GEAR_RATIO_TO_GRID)
        return ODVPosition(x_grid, y_grid)

    def _get_grid_tile_from_fine_xy_(self, fine_position: ODVPosition) -> tuple[ODVPosition, str]:

        x_grid = floor(fine_position.x / FINE_GRID_SIZE)
        y_grid = floor(fine_position.y / FINE_GRID_SIZE)
        # print("Coarse", x_grid, y_grid)
        tile = ODVPosition(x_grid, y_grid)
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
    def _tile_to_angle(tile: ODVPosition) -> ODVPosition:
        tile_angle_x = tile.x * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID
        tile_angle_y = (tile.y * FINE_GRID_SIZE * GEAR_RATIO_TO_GRID) + GEAR_RATIO_TO_GRID
        return ODVPosition(tile_angle_x, tile_angle_y)

    def _navigate_to_grid_tile(self, tile: ODVPosition)-> ODVPosition:
        print(f"navigating to tile {tile}")
        tile_angle = self._tile_to_angle(tile)
        self.y_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle.y)
        self.x_motor.run_target(MAX_MOTOR_ROT_SPEED, tile_angle.x)
        return tile_angle

    def _navigate_grid_tile_path(self, grid_tile_path: list[ODVPosition]):
        for i, p in enumerate(grid_tile_path):
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
        if self.has_load:
            return
        print('getting path to load')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.load_tile)
        self._navigate_grid_tile_path(path)
        self._do_load_()

    def auto_unload(self):
        if not self.is_homed:
            return
        if not self.has_load:
            return
        print('getting path to unload')
        tile, _ = self._get_grid_tile_from_fine_xy_(self._get_fine_grid_position_())
        path = self._bfs_path_to_grid_tile(tile, self.unload_tile)
        self._navigate_grid_tile_path(path)
        self._do_unload_()

    def _bfs_path_to_grid_tile(self, start_tile: ODVPosition, end_tile: ODVPosition):
        queue: Queue = Queue()
        queue.put([start_tile])  # Enqueue the start position

        while not queue.empty():
            path = queue.get()  # Dequeue the path
            current_pos = path[-1]  # Current position is the last element of the path

            if current_pos == end_tile:
                return path  # Return the path if end is reached

            for direction in [EAST, NORTH, WEST, SOUTH]:  # Possible movements
                new_pos = current_pos.position_from_direction(direction)
                if new_pos.value() in self.coarse_grid and self.coarse_grid[new_pos.value()] in OK_MOVES:
                    new_path = list(path)
                    new_pos.direction = direction
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


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END
