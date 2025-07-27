from pybricks.tools import wait

from .lego_vehicle_timer import MotorHelper, ErrorFlashCodes
error_flash_code = ErrorFlashCodes()
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Remote
hub: TechnicHub | None = None
remote: Remote | None = None
from pybricks.parameters import Side, Button

#IMPORTS_START
from micropython import mem_info
from pybricks.pupdevices import Motor
from pybricks.parameters import  Port, Direction
from uerrno import ENODEV
#IMPORTS_END

#INCLUDE_AFTER_HERE

#VARS_START
# odv settings
ODV_SPEED: int = 60  # set between 50 and 80
# X= obstacle, L = Load, U = Unload, # = grid tile
ODV_GRID = ["###X#XX", "LX###XU", "###X###"]
#VARS_END
#MODULE_START
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
#MODULE_END
#DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)
#DRIVE_SETUP_END
