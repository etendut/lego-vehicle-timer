import pytest
from pytest_check import check

from modules.vehicle_odv import (
    can_move_in_direction_by_type,
    NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, NORTH_WEST,
)

# Shorthand tile types
T = '#'   # TRACK
W = 'X'   # WALL
H = 'H'   # HOME
L = 'L'   # LOAD
U = 'U'   # UNLOAD
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

    # --- Rule 4: any corner on HOME and direction not in {N, W, NW} → block ---
    pytest.param(EAST,       H, T, T, T, False, id="home-east"),
    pytest.param(SOUTH,      T, H, T, T, False, id="home-south"),
    pytest.param(SOUTH_EAST, T, T, H, T, False, id="home-south-east"),
    pytest.param(SOUTH_WEST, T, T, T, H, False, id="home-south-west"),
    pytest.param(NORTH_EAST, H, T, T, T, False, id="home-north-east"),
    # HOME allowed when entering from NORTH, WEST, or NORTH_WEST
    pytest.param(NORTH,      H, T, T, T, True,  id="home-north-allowed"),
    pytest.param(WEST,       T, T, T, H, True,  id="home-west-allowed"),
    pytest.param(NORTH_WEST, H, T, T, T, True,  id="home-north-west-allowed"),

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
