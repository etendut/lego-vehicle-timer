import pytest
from pytest_check import check


from modules.vehicle_odv import position_from_direction, NORTH,NORTH_EAST, EAST,SOUTH_EAST, SOUTH,SOUTH_WEST, WEST, NORTH_WEST

position_from_direction_tests = [
    pytest.param((0,0), NORTH, (0, -1), id="NORTH"),
    pytest.param((0,0), NORTH_EAST, (1, -1), id="NORTH_EAST"),
    pytest.param((0,0), EAST, (1, 0), id="EAST"),
    pytest.param((0,0), SOUTH_EAST, (1, 1), id="SOUTH_EAST"),
    pytest.param((0,0), SOUTH, (0, 1), id="SOUTH"),
    pytest.param((0,0), SOUTH_WEST, (-1, 1), id="SOUTH_WEST"),
    pytest.param((0,0), WEST, (-1, 0), id="WEST"),
    pytest.param((0,0), NORTH_WEST, (-1, -1), id="NORTH_WEST"),

]

@pytest.mark.parametrize("current_pos, direction,new_pos",position_from_direction_tests)
def test_position_from_direction(current_pos, direction,new_pos):
    # setup
    # test
    result = position_from_direction(current_pos, direction)
    # assert
    check.equal(result, new_pos)