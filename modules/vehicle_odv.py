# IMPORTS_START
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
try:
    from uerrno import ENODEV
except ImportError:
    ENODEV = -99
try:
    from umath import floor, sqrt
except ImportError:
    from math import floor, sqrt
# IMPORTS_END

# local var only
from micropython import mem_info, const
def mock_const(val):
    return val
# for testing
if const(12) is None:
    const = mock_const
from pybricks.tools import wait
from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Remote

hub: TechnicHub | None = None
remote: Remote | None = None
from pybricks.parameters import Button

# VARS_START
DEBUG = const(False)

# odv settings
ODV_SPEED: int = const(45)  # set between 40 and 70
# X= obstacle, L = Load, U = Unload/End (homing wall NORTH and EAST), # = grid tile, < left direction only, > right direction only
# ODV_GRID = ["H######", "###X#XX", "LX###XU", "###X###"]
# ODV_GRID = ["XL##XU", "H#X###"]

ODV_GRID_DEFAULT = ["L##<U", "X#X#X", "X###X"]
ODV_GRID_EX1 = ["###X#XX", "LX###XU", "###X###"]
ODV_GRID_EX2 = ["X###X", "L###U", "X###X"]

ODV_GRID = ODV_GRID_DEFAULT

# VARS_END
# MODULE_START
##################################################################################
# ODV helper
##################################################################################
NORTH_WEST = const(0)
NORTH = const(1)
NORTH_EAST = const(2)
EAST = const(3)
SOUTH_EAST = const(4)
SOUTH = const(5)
SOUTH_WEST = const(6)
WEST = const(7)

_ALL_DIRECTIONS = (NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, NORTH_WEST)

WALL = 'X'
TRACK = '#'
WEST_ONLY_TRACK = '<'
EAST_ONLY_TRACK = '>'
LOAD = 'L'
UNLOAD = 'U'
OK_MOVES = [TRACK, LOAD, UNLOAD]

_FINE_GRID_SIZE = const(10)
_ODV_SIZE = const(8)

_GEAR_RATIO_TO_GRID: int = const(80)  # Motor rotation angle per grid pitch (deg/pitch)
_MAX_MOTOR_ROT_SPEED: int = const(1400)  # Max motor speed (deg/s) ~1500
_HOMING_MOTOR_ROT_SPEED: int = const(200)  # Homing speed (deg/s)
_HOMING_DUTY: int = const(45)  # Homing motor duty (%) (adjustment required)

def dir_to_str(direction:int)->str:
    if direction == NORTH:
        return "NORTH"
    if direction == NORTH_EAST:
        return "NORTH_EAST"
    if direction == EAST:
        return "EAST"
    if direction == SOUTH_EAST:
        return "SOUTH_EAST"
    if direction == SOUTH:
        return "SOUTH"
    if direction == SOUTH_WEST:
        return "SOUTH_WEST"
    if direction == WEST:
        return "WEST"
    if direction == NORTH_WEST:
        return "NORTH_WEST"
    return "UNKNOWN"

def position_from_direction(position: tuple[int, int], direction: int) -> tuple[int, int]:
    if direction == NORTH:
        return position[0], position[1] - 1
    if direction == NORTH_EAST:
        return position[0] + 1, position[1] - 1
    if direction == EAST:
        return position[0] + 1, position[1]
    if direction == SOUTH_EAST:
        return position[0] + 1, position[1] + 1
    if direction == SOUTH:
        return position[0], position[1] + 1
    if direction == SOUTH_WEST:
        return position[0] - 1, position[1] + 1
    if direction == WEST:
        return position[0] - 1, position[1]
    if direction == NORTH_WEST:
        return position[0] - 1, position[1] - 1

    return position[0], position[1]


def can_move_in_direction_by_type(direction: int, tl_type: str, tr_type: str, br_type: str, bl_type: str) -> tuple[
    bool, bool, bool]:
    if DEBUG:
        print("--")
        print(dir_to_str(direction), tl_type, tr_type, br_type, bl_type)

    if tl_type == WALL or tr_type == WALL or br_type == WALL or bl_type == WALL:
        can_move = False
    elif (tl_type == WEST_ONLY_TRACK or tr_type == WEST_ONLY_TRACK or br_type == WEST_ONLY_TRACK or bl_type == WEST_ONLY_TRACK) and direction != WEST:
        can_move = False
    elif (tl_type == EAST_ONLY_TRACK or tr_type == EAST_ONLY_TRACK or br_type == EAST_ONLY_TRACK or bl_type == EAST_ONLY_TRACK) and direction != EAST:
        can_move = False
    elif (tl_type == UNLOAD or tr_type == UNLOAD or br_type == UNLOAD or bl_type == UNLOAD) and direction == NORTH:
        can_move = False
    else:
        can_move = True

    can_load = tl_type == LOAD and tr_type == LOAD and br_type == LOAD and bl_type == LOAD
    can_unload = tl_type == UNLOAD and tr_type == UNLOAD and br_type == UNLOAD and bl_type == UNLOAD

    if DEBUG:
        print("can_move, can_load, can_unload", can_move, can_load, can_unload)
    return can_move, can_load, can_unload


def _can_traverse_coarse(from_type: str, to_type: str, direction: int) -> bool:
    if to_type == WALL:
        return False
    if (from_type == WEST_ONLY_TRACK or to_type == WEST_ONLY_TRACK) and direction != WEST:
        return False
    if (from_type == EAST_ONLY_TRACK or to_type == EAST_ONLY_TRACK) and direction != EAST:
        return False
    if from_type == UNLOAD and direction == NORTH:
        return False
    return True


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
        self._update_dimensions_(new_tl, (self.width + buffer), (self.height + buffer))

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
        self.unload_tile: tuple[int, int] = (0, 0)
        self.load_tile: tuple[int, int] = (0, 0)
        self.last_fine_grid_position: tuple[int, int] = (0, 0)
        """current position"""
        self.grid_tracks = []
        self.gt_one_way_right = []
        self.gt_one_way_left = []
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
                if DEBUG:
                    print('Motor needs to be connected to ' + str(self.motor_x_port))
                self.error_flash_code.set_error_no_motor_on_a()
            raise
        try:
            self.motor_y = Motor(self.motor_y_port, Direction.CLOCKWISE)
        except OSError as ex:
            if ex.errno == ENODEV:
                if DEBUG:
                    print('Motor needs to be connected to ' + str(self.motor_y_port))
                self.error_flash_code.set_error_no_motor_on_b()
            raise

        self.stop_motors()

    def _load_grid_(self, lines: list[str]):
        # loop through grid lines
        y = 0
        if DEBUG:
            print('Loading grid')
            mem_info()
        for y, line in enumerate(lines):
            if DEBUG:
                print(f"line {y + 1}/{len(lines)}")
            self.coarse_grid_width = const(len(line.rstrip()))
            line = line.rstrip()
            for x, character in enumerate(line):
                if DEBUG:
                    print(f"line {y + 1} |col {x + 1}/{len(line)}|{character}")

                if character in OK_MOVES:
                    self.grid_tracks.append((x, y))
                if character == WEST_ONLY_TRACK:
                    self.gt_one_way_left.append((x, y))
                if character == EAST_ONLY_TRACK:
                    self.gt_one_way_right.append((x, y))
                # set load/unload points
                if character == LOAD:
                    # print('----load_tile----')
                    # mem_info()
                    self.load_tile = const((x, y))  # mem_info()  # print('----load_tile----')
                if character == UNLOAD:
                    # print('----unload_tile----')
                    # mem_info()
                    self.unload_tile = const((x, y))  # mem_info()  # print('----unload_tile----')

            y += 1
        self.coarse_grid_height = const(y)
        if DEBUG:
            mem_info()
            print('Grid Loaded')
            print(f"--loads tile is {self.load_tile}")
            print(f"--unload tile is {self.unload_tile}")
            self._display_grid_()

    def _display_grid_(self, position_x_y: tuple | None = None):
        if not DEBUG:
            return
        # Display the maze:
        for y in range(self.coarse_grid_height):
            for x in range(self.coarse_grid_width):
                if position_x_y is not None and (x, y) == position_x_y:
                    print("R", end='')
                elif (x, y) == self.load_tile:
                    print(LOAD, end='')
                elif (x, y) == self.unload_tile:
                    print(UNLOAD, end='')
                elif (x, y) in self.grid_tracks:
                    print(TRACK, end='')
                elif (x, y) in self.gt_one_way_right:
                    print(EAST_ONLY_TRACK, end='')
                elif (x, y) in self.gt_one_way_left:
                    print(WEST_ONLY_TRACK, end='')
                else:
                    print(WALL, end='')
            print()  # Print a newline after printing the row.

    def reset_homing(self) -> None:
        self.reset_is_homed()

    def do_homing(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then step back one pitch and set that as the zero origin.
        # Homing wall is NORTH and EAST of the unload tile.
        if self.mh_is_homed:
            return
        unload_tile_angle = self._tile_to_angle(self.unload_tile)

        # Homing axis Y — run NORTH until stalled against top wall
        self.motor_y.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        self.motor_y.reset_angle(unload_tile_angle[1])
        self.motor_y.run_angle(_MAX_MOTOR_ROT_SPEED, _GEAR_RATIO_TO_GRID)
        wait(200)

        # Homing axis X — run EAST until stalled against right wall
        self.motor_x.run_until_stalled(_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        self.motor_x.reset_angle(unload_tile_angle[0] + (_FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID))
        self.motor_x.run_angle(_MAX_MOTOR_ROT_SPEED, -_GEAR_RATIO_TO_GRID)
        wait(200)

        self.has_load = False
        self.set_is_homed()
        self._display_grid_(self.unload_tile)

    def _can_move_in_direction_(self, direction: int) -> tuple[bool, bool, bool]:
        if direction != NORTH and direction != EAST and direction != SOUTH and direction != WEST:
        # if direction not in [_NORTH, _NORTH_EAST, _EAST, _SOUTH_EAST, _SOUTH, _SOUTH_WEST, _WEST, _NORTH_WEST]:
            return False, False, False

        # work out cart dimensions
        cart = ODVBox(self.last_fine_grid_position, _ODV_SIZE, _ODV_SIZE)
        # shrink the cart to make moving smoother
        cart.buffer(-1)

        # print("Cart", cart)

        tl_type = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.top_left, direction), False)
        tr_type = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.top_right, direction), False)
        br_type = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.bottom_right, direction), False)
        bl_type = self._get_grid_tile_type_from_fine_xy_(position_from_direction(cart.bottom_left, direction), False)

        return can_move_in_direction_by_type(direction, tl_type, tr_type, br_type, bl_type)

    def _can_move_in_direction_from_tile_(self, coarse_position: tuple[int, int], direction: int) -> tuple[
        bool, bool, bool]:
        ex_type = self._get_grid_tile_type_from_coarse_xy_(coarse_position)
        new_type = self._get_grid_tile_type_from_coarse_xy_(position_from_direction(coarse_position, direction))
        can_move, can_load, can_unload =False, False, False
        if direction == NORTH:
            can_move, can_load, can_unload= can_move_in_direction_by_type(direction, new_type, new_type, ex_type, ex_type)
        if direction == EAST:
            can_move, can_load, can_unload= can_move_in_direction_by_type(direction, ex_type, new_type, new_type, ex_type)
        if direction == SOUTH:
            can_move, can_load, can_unload= can_move_in_direction_by_type(direction, ex_type, ex_type, new_type, new_type)
        if direction == WEST:
            can_move, can_load, can_unload= can_move_in_direction_by_type(direction, new_type, ex_type, ex_type, new_type)
        return can_move, can_load, can_unload

    def _get_fine_grid_position_(self) -> tuple[int, int]:

        x_grid = int(self.motor_x.angle() / _GEAR_RATIO_TO_GRID)
        y_grid = int(self.motor_y.angle() / _GEAR_RATIO_TO_GRID)
        fine_grid_position = (x_grid, y_grid)
        if DEBUG:
            print("fine_grid_position", fine_grid_position)
        return fine_grid_position

    def _get_grid_tile_type_from_fine_xy_(self, fine_position: tuple[int, int], use_fuzzy: bool) -> str:
        tile_position, tile_type = self._get_grid_tile_from_fine_xy_(fine_position, use_fuzzy)
        return tile_type

    def _get_grid_tile_position_from_fine_xy_(self, fine_position: tuple[int, int], use_fuzzy: bool) -> tuple[int, int]:
        tile_position, tile_type = self._get_grid_tile_from_fine_xy_(fine_position, use_fuzzy)
        return tile_position

    def _get_grid_tile_from_fine_xy_(self, fine_position: tuple[int, int], use_fuzzy: bool) -> tuple[
        tuple[int, int], str]:

        # move to center of cart
        fuzzy = floor(_ODV_SIZE / 2) if use_fuzzy else 0
        x_grid = floor((fine_position[0] + fuzzy) / _FINE_GRID_SIZE)
        y_grid = floor((fine_position[1] + fuzzy) / _FINE_GRID_SIZE)
        if DEBUG:
            print("Fine", fine_position)
        tile = (x_grid, y_grid)
        if fine_position[0] < 1 or fine_position[1] < 1:
            return tile, WALL
        return self._get_grid_tile_from_coarse_xy_(tile)

    def _get_grid_tile_type_from_coarse_xy_(self, coarse_position: tuple[int, int]) -> str:
        tile_position, tile_type = self._get_grid_tile_from_coarse_xy_(coarse_position)
        return tile_type

    def _get_grid_tile_position_from_coarse_xy_(self, coarse_position: tuple[int, int]) -> tuple[int, int]:
        tile_position, tile_type = self._get_grid_tile_from_coarse_xy_(coarse_position)
        return tile_position

    def _get_grid_tile_from_coarse_xy_(self, coarse_position: tuple[int, int]) -> tuple[tuple[int, int], str]:

        if DEBUG:
            print("-Coarse", coarse_position)
        if coarse_position == self.load_tile:
            return coarse_position, LOAD
        if coarse_position == self.unload_tile:
            return coarse_position, UNLOAD
        if coarse_position in self.gt_one_way_right:
            return coarse_position, EAST_ONLY_TRACK
        if coarse_position in self.gt_one_way_left:
            return coarse_position, WEST_ONLY_TRACK
        if coarse_position in self.grid_tracks:
            return coarse_position, TRACK
        return coarse_position, WALL

    def _move_in_direction_(self, direction: int) -> bool:

        if direction not in _ALL_DIRECTIONS:
            if DEBUG:
                print('Invalid direction')
            return False

        if direction == NORTH or direction == NORTH_EAST or direction == NORTH_WEST:
            self.motor_y.dc(-self.drive_speed)
        if direction == SOUTH or direction == SOUTH_EAST or direction == SOUTH_WEST:
            self.motor_y.dc(self.drive_speed)

        if direction == EAST or direction == NORTH_EAST or direction == SOUTH_EAST:
            self.motor_x.dc(self.drive_speed)
        if direction == WEST or direction == NORTH_WEST or direction == SOUTH_WEST:
            self.motor_x.dc(-self.drive_speed)

        self.motors_running = True
        return True

    def _do_load_(self):
        if self.has_load:
            if DEBUG:
                print('Already loaded')
            return
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_(), True)
        if tile != self.load_tile and self._distance(tile, self.load_tile) > 1:
            if DEBUG:
                print(f'{tile} is too far away from load_tile {self.load_tile}')
            return
        tile_angle = self._navigate_to_grid_tile(self.load_tile)
        wait(200)
        # do load
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle[0] - (_GEAR_RATIO_TO_GRID * 3))
        wait(2000)
        if DEBUG:
            print("loading..")
        self._navigate_to_grid_tile(self.load_tile)
        wait(200)
        self.has_load = True
        if DEBUG:
            print("ready to go")

    def _do_unload_(self):

        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_(), True)
        if tile != self.unload_tile and self._distance(tile, self.unload_tile) > 1:
            if DEBUG:
                print(f'{tile} is too far away from unload_tile {self.load_tile}')
            return

        tile_angle = self._navigate_to_grid_tile(self.unload_tile)
        wait(200)
        if DEBUG:
            print("unloading..")
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle[0] + (_GEAR_RATIO_TO_GRID * 5))
        wait(2000)
        self._navigate_to_grid_tile(self.unload_tile)
        wait(200)
        self.has_load = False
        if DEBUG:
            print("ready to go")

    @staticmethod
    def _tile_to_angle(tile: tuple[int, int]) -> tuple[int, int]:
        """
        Convert grid tile to angle
        :param tile: tuple[int,int]
        :return: ODVAnglePosition
        """
        tile_angle_x = tile[0] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID
        tile_angle_y = (tile[1] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID)
        return tile_angle_x, tile_angle_y

    def _navigate_to_grid_tile(self, tile: tuple[int, int], stop=Stop.HOLD) -> tuple[int, int]:
        if DEBUG:
            print(f"navigating to tile {tile}")
        tile_angle_x = tile[0] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID
        tile_angle_y = (tile[1] * _FINE_GRID_SIZE * _GEAR_RATIO_TO_GRID) + _GEAR_RATIO_TO_GRID
        self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle_y, then=stop)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, tile_angle_x, then=stop)
        return tile_angle_x, tile_angle_y

    def _navigate_grid_tile_path(self, grid_tile_path: list[tuple[tuple[int, int], int]]) -> bool:
        """
        Navigates robot through list of tuple[int,int]
        :param grid_tile_path:
        :return: succeeded
        """
        for i, path in enumerate(grid_tile_path):
            # if user takes over break
            if self.mh_auto_drive and not self.mh__remote_disabled and remote is not None and len(remote.buttons.pressed()) > 0:
                self.disable_auto_drive()
                self.stop_motors()
                return False

            if path[1] is not None and i < (len(grid_tile_path) - 1) and grid_tile_path[i + 1][1] is not None and \
                    grid_tile_path[i + 1][1] == path[1]:
                self._navigate_to_grid_tile(path[0], Stop.NONE)
            else:
                self._navigate_to_grid_tile(path[0])

        return True

    def auto_home(self):
        if not self.mh_is_homed:
            return
        if DEBUG:
            print('getting path to home')
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_(), True)
        path = self._bfs_path_to_grid_tile(tile, self.unload_tile)
        self._navigate_grid_tile_path(path)
        if DEBUG:
            print('homed')

    def auto_load(self):
        if not self.mh_is_homed:
            return
        if DEBUG:
            print('getting path to load')
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_(), True)
        path = self._bfs_path_to_grid_tile(tile, self.load_tile)
        self._navigate_grid_tile_path(path)
        self._do_load_()

    def auto_unload(self):
        if not self.mh_is_homed:
            return
        if DEBUG:
            print('getting path to unload')
        tile = self._get_grid_tile_position_from_fine_xy_(self._get_fine_grid_position_(), True)
        path = self._bfs_path_to_grid_tile(tile, self.unload_tile)
        self._navigate_grid_tile_path(path)
        self._do_unload_()

    @staticmethod
    def _distance(start_tile: tuple[int, int], end_tile: tuple[int, int]) -> int:
        return floor(sqrt(pow(start_tile[0] - end_tile[0], 2) + pow(start_tile[1] - end_tile[1], 2)))

    def print_tile_pos(self, tile_name: str, tile: tuple[int, int]):
        if DEBUG:
            print(f"tile {tile_name} at {tile}, {self._tile_to_angle(tile)}")

    def _bfs_path_to_grid_tile(self, start_tile: tuple[int, int], end_tile: tuple[int, int]) -> list[
        tuple[tuple[int, int], int]]:
        if DEBUG:
            print("---bfs_path_to_grid_tile---")
            self._display_grid_()
            self.print_tile_pos("--start", start_tile)
            self.print_tile_pos("--end", end_tile)
            print("--grid_tracks", self.grid_tracks)
            print("--gt_one_way_left", self.gt_one_way_left)
            print("--gt_one_way_right", self.gt_one_way_right)
            self.print_tile_pos("--load_tile", self.load_tile)
            self.print_tile_pos("--unload_tile", self.unload_tile)
        # parent[tile] = (parent_tile, direction_taken_to_reach_tile)
        parent: dict[tuple[int, int], tuple[tuple[int, int] | None, int]] = {start_tile: (None, -1)}
        queue: list[tuple[int, int]] = [start_tile]
        head = 0
        found = False

        while head < len(queue):
            current = queue[head]
            head += 1

            if current == end_tile:
                found = True
                break

            from_type = self._get_grid_tile_type_from_coarse_xy_(current)
            for direction in _ALL_DIRECTIONS:
                new_pos = position_from_direction(current, direction)
                if new_pos in parent:
                    continue
                to_type = self._get_grid_tile_type_from_coarse_xy_(new_pos)
                if _can_traverse_coarse(from_type, to_type, direction):
                    parent[new_pos] = (current, direction)
                    queue.append(new_pos)

        if not found:
            if DEBUG:
                print("no path found")
            return []

        # Reconstruct path by walking back through parent map
        path: list[tuple[tuple[int, int], int]] = []
        tile: tuple[int, int] | None = end_tile
        while tile is not None:
            parent_tile, direction = parent[tile]
            path.append((tile, direction))
            tile = parent_tile
        path.reverse()

        if DEBUG:
            print(path)
            print("---bfs_path_to_grid_tile---")
        return path

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        if self.mh__remote_disabled:
            return
        # Check which remote_buttons are pressed.
        assert remote is not None
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



        if direction is None:
            self.stop_motors()
            if DEBUG:
                print('Invalid direction')
            return

        # print(direction)
        can_move, can_load, can_unload = self._can_move_in_direction_(direction)
        if can_load and direction == WEST:
            self.stop_motors()
            self._do_load_()
            return
        if can_unload and direction == EAST:
            self.stop_motors()
            self._do_unload_()
            return

        if not can_move:
            self.stop_motors()
            return

        self._move_in_direction_(direction)

    # stop all motors
    def stop_motors(self):
        self.motor_x.stop()
        self.motor_y.stop()
        self.motors_running = False



# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)  # DRIVE_SETUP_END
