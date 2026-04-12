import pytest
from pytest_check import check

from unittest.mock import MagicMock

from modules.vehicle_odv import (
    can_move_in_direction_by_type, _can_traverse_coarse, RunODVMotors,
    NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, NORTH_WEST,
)

# Shorthand tile types
T = '#'   # TRACK
W = 'X'   # WALL
L = 'L'   # LOAD
U = 'U'   # UNLOAD (end tile — homing wall NORTH and EAST)
LT = '<'  # WEST_ONLY
RT = '>'  # EAST_ONLY

can_move_tests = [
    # --- Rule 1: any corner on WALL → block ---
    pytest.param(NORTH, W, T, T, T, False, id="wall-tl"),
    pytest.param(NORTH, T, W, T, T, False, id="wall-tr"),
    pytest.param(NORTH, T, T, W, T, False, id="wall-br"),
    pytest.param(NORTH, T, T, T, W, False, id="wall-bl"),

    # --- Rule 2: any corner on < and direction != WEST → block ---
    pytest.param(EAST,  LT, T,  T,  T,  False, id="lt-tl-east"),
    pytest.param(NORTH, T,  LT, T,  T,  False, id="lt-tr-north"),
    pytest.param(SOUTH, T,  T,  LT, T,  False, id="lt-br-south"),
    pytest.param(EAST,  T,  T,  T,  LT, False, id="lt-bl-east"),
    # < allowed when moving WEST
    pytest.param(WEST, LT, T,  T,  LT, True,  id="lt-west-allowed"),
    pytest.param(WEST, T,  LT, LT, T,  True,  id="lt-west-allowed-2"),

    # --- Rule 3: any corner on > and direction != EAST → block ---
    pytest.param(WEST,  RT, T,  T,  T,  False, id="rt-tl-west"),
    pytest.param(NORTH, T,  RT, T,  T,  False, id="rt-tr-north"),
    pytest.param(SOUTH, T,  T,  RT, T,  False, id="rt-br-south"),
    pytest.param(WEST,  T,  T,  T,  RT, False, id="rt-bl-west"),
    # > allowed when moving EAST
    pytest.param(EAST, RT, T,  T,  RT, True,  id="rt-east-allowed"),
    pytest.param(EAST, T,  RT, RT, T,  True,  id="rt-east-allowed-2"),

    # --- Rule 4: any corner on UNLOAD and direction == NORTH → block ---
    pytest.param(NORTH, U, T, T, T, False, id="unload-north-tl"),
    pytest.param(NORTH, T, U, T, T, False, id="unload-north-tr"),
    pytest.param(NORTH, T, T, U, T, False, id="unload-north-br"),
    pytest.param(NORTH, T, T, T, U, False, id="unload-north-bl"),
    # UNLOAD allowed in other directions
    pytest.param(EAST,  U, T, T, T, True,  id="unload-east-allowed"),
    pytest.param(SOUTH, U, T, T, T, True,  id="unload-south-allowed"),
    pytest.param(WEST,  U, T, T, T, True,  id="unload-west-allowed"),

    # --- Rule 5: all other combinations → allow ---
    pytest.param(NORTH, T, T, T, T, True, id="all-track-north"),
    pytest.param(EAST,  T, T, T, T, True, id="all-track-east"),
    pytest.param(SOUTH, T, T, T, T, True, id="all-track-south"),
    pytest.param(WEST,  T, T, T, T, True, id="all-track-west"),
    # mixed track/load or track/unload → can move
    pytest.param(EAST, T, L, T, T, True, id="mixed-load-track"),
    pytest.param(EAST, T, U, T, T, True, id="mixed-unload-track"),
]


@pytest.mark.parametrize("direction,tl,tr,br,bl,expected_move", can_move_tests)
def test_can_move(direction, tl, tr, br, bl, expected_move):
    can_move, _, _ = can_move_in_direction_by_type(direction, tl, tr, br, bl)
    check.equal(can_move, expected_move)


can_load_unload_tests = [
    pytest.param(EAST, L, L, L, L, True,  False, id="all-load"),
    pytest.param(EAST, U, U, U, U, False, True,  id="all-unload"),
    pytest.param(EAST, L, T, T, T, False, False, id="partial-load"),
    pytest.param(EAST, T, T, T, T, False, False, id="all-track"),
]


@pytest.mark.parametrize("direction,tl,tr,br,bl,expected_load,expected_unload", can_load_unload_tests)
def test_can_load_unload(direction, tl, tr, br, bl, expected_load, expected_unload):
    _, can_load, can_unload = can_move_in_direction_by_type(direction, tl, tr, br, bl)
    check.equal(can_load, expected_load)
    check.equal(can_unload, expected_unload)


# ---------------------------------------------------------------------------
# _can_traverse_coarse tests
# ---------------------------------------------------------------------------

coarse_tests = [
    # WALL destination always blocks
    pytest.param(T,  W,  NORTH, False, id="to-wall"),
    pytest.param(T,  W,  EAST,  False, id="to-wall-from-track"),

    # < (WEST_ONLY): either tile blocks if direction != WEST
    pytest.param(LT, T,  EAST,  False, id="from-lt-east"),
    pytest.param(T,  LT, NORTH, False, id="to-lt-north"),
    pytest.param(LT, LT, SOUTH, False, id="both-lt-south"),
    pytest.param(LT, T,  WEST,  True,  id="from-lt-west-allowed"),
    pytest.param(T,  LT, WEST,  True,  id="to-lt-west-allowed"),

    # > (EAST_ONLY): either tile blocks if direction != EAST
    pytest.param(RT, T,  WEST,  False, id="from-rt-west"),
    pytest.param(T,  RT, NORTH, False, id="to-rt-north"),
    pytest.param(RT, T,  EAST,  True,  id="from-rt-east-allowed"),
    pytest.param(T,  RT, EAST,  True,  id="to-rt-east-allowed"),

    # UNLOAD source: NORTH is blocked (homing wall above)
    pytest.param(U,  T,  NORTH, False, id="from-unload-north"),
    pytest.param(U,  T,  EAST,  True,  id="from-unload-east-allowed"),
    pytest.param(U,  T,  SOUTH, True,  id="from-unload-south-allowed"),
    pytest.param(U,  T,  WEST,  True,  id="from-unload-west-allowed"),
    # UNLOAD as destination: allowed from any direction
    pytest.param(T,  U,  SOUTH, True,  id="to-unload-south"),
    pytest.param(T,  U,  EAST,  True,  id="to-unload-east"),

    # free traversal
    pytest.param(T, T, NORTH, True, id="track-to-track"),
    pytest.param(T, L, EAST,  True, id="track-to-load"),
    pytest.param(T, U, WEST,  True, id="track-to-unload"),
]


@pytest.mark.parametrize("from_type,to_type,direction,expected", coarse_tests)
def test_can_traverse_coarse(from_type, to_type, direction, expected):
    check.equal(_can_traverse_coarse(from_type, to_type, direction), expected)


# ---------------------------------------------------------------------------
# BFS tests
# Grid: ["X#<U", "X#X#", "L#>#"]
#   (0,0)=X  (1,0)=#  (2,0)=<  (3,0)=U
#   (0,1)=X  (1,1)=#  (2,1)=X  (3,1)=#
#   (0,2)=L  (1,2)=#  (2,2)=>  (3,2)=#
# ---------------------------------------------------------------------------

TEST_GRID = ["X#<U", "X#X#", "L#>#"]

bfs_tests = [
    pytest.param(
        (0, 2), (3, 0),
        [((0,2),-1), ((1,2),EAST), ((2,2),EAST), ((3,2),EAST), ((3,1),NORTH), ((3,0),NORTH)],
        id="load-to-unload",
    ),
    pytest.param(
        (3, 0), (0, 2),
        # diagonal SW from (1,1) reaches (0,2) directly
        [((3,0),-1), ((2,0),WEST), ((1,0),WEST), ((1,1),SOUTH), ((0,2),SOUTH_WEST)],
        id="unload-to-load",
    ),
    pytest.param(
        (1, 0), (3, 0),
        [((1,0),-1), ((1,1),SOUTH), ((1,2),SOUTH), ((2,2),EAST), ((3,2),EAST), ((3,1),NORTH), ((3,0),NORTH)],
        id="track-to-unload",
    ),
    pytest.param(
        (3, 0), (1, 0),
        [((3,0),-1), ((2,0),WEST), ((1,0),WEST)],
        id="unload-to-track",
    ),
]


@pytest.mark.parametrize("start_tile,end_tile,expected_path", bfs_tests)
def test_bfs(start_tile, end_tile, expected_path):
    helper = RunODVMotors(MagicMock(), 80, TEST_GRID)
    result = helper._bfs_path_to_grid_tile(start_tile, end_tile)
    check.equal(result, expected_path)


# ---------------------------------------------------------------------------
# All-directions BFS tests
# Grid: ["X###X", "L###U", "X###X"]
#   (0,0)=X  (1,0)=#  (2,0)=#  (3,0)=#  (4,0)=X
#   (0,1)=L  (1,1)=#  (2,1)=#  (3,1)=#  (4,1)=U
#   (0,2)=X  (1,2)=#  (2,2)=#  (3,2)=#  (4,2)=X
#
# Each diagonal test uses a corner-to-opposite-corner path through centre (2,1),
# exercising all 8 direction constants (cardinal directions covered in bfs_tests).
# ---------------------------------------------------------------------------

ALL_DIRS_GRID = ["X###X", "L###U", "X###X"]

all_dirs_bfs_tests = [
    pytest.param(
        (1, 0), (3, 2),
        [((1,0),-1), ((2,1),SOUTH_EAST), ((3,2),SOUTH_EAST)],
        id="diagonal-south-east",
    ),
    pytest.param(
        (3, 0), (1, 2),
        [((3,0),-1), ((2,1),SOUTH_WEST), ((1,2),SOUTH_WEST)],
        id="diagonal-south-west",
    ),
    pytest.param(
        (1, 2), (3, 0),
        [((1,2),-1), ((2,1),NORTH_EAST), ((3,0),NORTH_EAST)],
        id="diagonal-north-east",
    ),
    pytest.param(
        (3, 2), (1, 0),
        [((3,2),-1), ((2,1),NORTH_WEST), ((1,0),NORTH_WEST)],
        id="diagonal-north-west",
    ),
]


@pytest.mark.parametrize("start_tile,end_tile,expected_path", all_dirs_bfs_tests)
def test_bfs_all_directions(start_tile, end_tile, expected_path):
    helper = RunODVMotors(MagicMock(), 80, ALL_DIRS_GRID)
    result = helper._bfs_path_to_grid_tile(start_tile, end_tile)
    check.equal(result, expected_path)
