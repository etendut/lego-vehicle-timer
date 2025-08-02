# IMPORTS_START
from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
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
ODV_SPEED: int = const(50)  # set between 50 and 80
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
LOAD = 'L'
UNLOAD = 'U'

DEFAULT_GRID = ["###X#XX", "LX###XU", "###X###"]
FINE_GRID_SIZE = const(10)
ODV_SIZE = const(8)

GEAR_RATIO: int = const(80)  # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED: int = const(1400)  # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED: int = const(200)  # Homing speed (deg/s)
HOMING_DUTY: int = const(45)  # Homing motor duty (%) (adjustment required)
LOAD_UNLOAD_ANGLE: int = const(160)

class ODVPosition:
    def __init__(self, x, y):
        self.x = x
        self.y = y

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


class RunODVMotors(MotorHelper):
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, grid_layout: list[str]):

        super().__init__(False, True)
        # grid setup
        self.motors_running = None
        self.unload_xy = (0, 0)
        self.load_xy = (0, 0)
        self.fine_grid_max_x = 0
        """used for unloading"""
        self.fine_grid_mid_y = 0
        """used for load/unloading"""
        self.last_fine_grid_position = ODVPosition(0, 0)
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
            self.fine_grid_max_x = (self.coarse_grid_width-1 * FINE_GRID_SIZE)
            line = line.rstrip()
            for x, character in enumerate(line):
                print(f"line {y + 1} |col {x + 1}/{len(line)}")
                self.coarse_grid[x, y] = character
                # set load/unload points
                if character == LOAD:
                    self.load_xy = x, y
                if character == UNLOAD:
                    self.unload_xy = x, y

            y += 1
        self.coarse_grid_height = y
        self.fine_grid_mid_y = int(self.coarse_grid_height  * FINE_GRID_SIZE / 2)
        mem_info()
        print('Grid Loaded')

    def _display_grid_(self, position: tuple[int, int], robot_symbol: str):
        # Display the maze:
        for y in range(self.coarse_grid_height):
            for x in range(self.coarse_grid_width):
                if (x, y) == position:
                    print(robot_symbol, end='')
                elif (x, y) == self.load_xy:
                    print(LOAD, end='')
                elif (x, y) == self.unload_xy:
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
        self.y_motor.reset_angle(0)
        self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, GEAR_RATIO)
        wait(200)

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.reset_angle(0)
        self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, GEAR_RATIO)
        wait(200)

        self.is_homed = True
        self._display_grid_((0, 0), ROBOT)

    def _can_move_in_direction_(self, direction: str) -> tuple[bool,bool,bool]:
        if direction not in DIRECTIONS:
            return False,False,False

        # work out cart dimensions
        cart = ODVBox(self.last_fine_grid_position, ODV_SIZE, ODV_SIZE)
        # shrink the cart to make moving smoother
        cart.buffer(-1)

        # print("Cart", cart)

        ok_moves = [TRACK, LOAD, UNLOAD]
        tl = self._get_grid_tile_from_fine_xy_(cart.top_left.position_from_direction(direction))
        tr = self._get_grid_tile_from_fine_xy_(cart.top_right.position_from_direction(direction))
        br = self._get_grid_tile_from_fine_xy_(cart.bottom_right.position_from_direction(direction))
        bl = self._get_grid_tile_from_fine_xy_(cart.bottom_left.position_from_direction(direction))

        can_move = tl in ok_moves and tr in ok_moves and br in ok_moves and bl in ok_moves

        can_load = tl == tr == br == bl == LOAD
        can_unload = tl == tr == br == bl == UNLOAD

        # print("Cart", direction, can_move)
        return can_move, can_load, can_unload

    def _get_fine_grid_position_(self) -> ODVPosition:

        x_grid = int(self.x_motor.angle() / GEAR_RATIO)
        y_grid = int(self.y_motor.angle() / GEAR_RATIO)
        return ODVPosition(x_grid, y_grid)

    def _get_grid_tile_from_fine_xy_(self, fine_position: ODVPosition) -> str:

        x_grid = floor(fine_position.x / FINE_GRID_SIZE)
        y_grid = floor(fine_position.y / FINE_GRID_SIZE)
        # print("Coarse", x_grid, y_grid)

        if fine_position.x < 1 or fine_position.y < 1 or x_grid < 0 or y_grid < 0 or x_grid > self.coarse_grid_width or y_grid > self.coarse_grid_height:
            return WALL
        if (x_grid, y_grid) in self.coarse_grid:
            return self.coarse_grid[(x_grid, y_grid)]

        return WALL

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

        print(f'current cart position({self.x_motor.angle()},{self.y_motor.angle()})')
        mid_y_angle = int(self.fine_grid_mid_y * GEAR_RATIO)
        print(f"position cart for load (0,{mid_y_angle})")
        # position cart
        # self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, mid_y_angle, wait=False)
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, 0)
        wait(200)
        # do load
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, -LOAD_UNLOAD_ANGLE)
        wait(2000)
        print("loading..")
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, 0)
        wait(200)
        self.has_load = True
        print("ready to go")

    def _do_unload_(self):

        mid_y_angle = int(self.fine_grid_mid_y * GEAR_RATIO)
        max_x_angle = int(self.fine_grid_max_x * GEAR_RATIO)
        print(f'current cart position({self.x_motor.angle()},{self.y_motor.angle()})')
        print(f"position cart for unload ({max_x_angle},{mid_y_angle})")
        # self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, mid_y_angle, wait=False)
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, max_x_angle)
        wait(200)
        print("unloading..")
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, max_x_angle+LOAD_UNLOAD_ANGLE)
        wait(2000)
        # self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, max_x_angle)
        wait(200)
        self.has_load = False
        print("ready to go")

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
            # print("current_pos", fine_grid_pos)
            # print("last_fine_grid_position", self.last_fine_grid_position)
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

        print(direction)
        can_move, can_load, can_unload = self._can_move_in_direction_(direction)
        if can_load and not self.has_load and direction == WEST:
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
        self.motors_running = False


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END
