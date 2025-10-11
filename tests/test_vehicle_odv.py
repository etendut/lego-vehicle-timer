import pytest
# noinspection PyProtectedMember
from pytest_check import check

from modules.vehicle_odv import position_from_direction, NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, \
    NORTH_WEST, can_move_in_direction_by_type

position_from_direction_tests = [
    pytest.param((0, 0), NORTH, (0, -1), id="NORTH"),
    pytest.param((0, 0), NORTH_EAST, (1, -1), id="NORTH_EAST"),
    pytest.param((0, 0), EAST, (1, 0), id="EAST"),
    pytest.param((0, 0), SOUTH_EAST, (1, 1), id="SOUTH_EAST"),
    pytest.param((0, 0), SOUTH, (0, 1), id="SOUTH"),
    pytest.param((0, 0), SOUTH_WEST, (-1, 1), id="SOUTH_WEST"),
    pytest.param((0, 0), WEST, (-1, 0), id="WEST"),
    pytest.param((0, 0), NORTH_WEST, (-1, -1), id="NORTH_WEST"),

]


@pytest.mark.parametrize("current_pos, direction,new_pos", position_from_direction_tests)
def test_position_from_direction(current_pos, direction, new_pos):
    # setup
    # test
    result = position_from_direction(current_pos, direction)
    # assert
    check.equal(result, new_pos)


can_move_in_direction_by_type_tests = [
    pytest.param(NORTH, "#","#","#","#",True,False,False, id="NORTH-#,#,#,#"),
    pytest.param( NORTH_EAST,"#","#","#","#",True,False,False, id="NORTH_EAST-#,#,#,#"),
    pytest.param( EAST, "#","#","#","#",True,False,False, id="EAST-#,#,#,#"),
    pytest.param( SOUTH_EAST,  "#","#","#","#",True,False,False, id="SOUTH_EAST-#,#,#,#"),
    pytest.param( SOUTH, "#","#","#","#",True,False,False, id="SOUTH-#,#,#,#"),
    pytest.param(SOUTH_WEST, "#","#","#","#",True,False,False, id="SOUTH_WEST-#,#,#,#"),
    pytest.param( WEST, "#","#","#","#",True,False,False, id="WEST-#,#,#,#"),
    pytest.param( NORTH_WEST, "#","#","#","#",True,False,False, id="NORTH_WEST-#,#,#,#"),
    pytest.param( EAST, "<","#","#","<",True,False,False, id="EAST-<,#,#,<"),
    pytest.param( EAST, "#","<","<","#",False,False,False, id="EAST-<,#,#,<"),
    pytest.param( WEST, ">","#","#",">",False,False,False, id="EAST-<,#,#,<"),
    pytest.param( WEST, "#",">",">","#",True,False,False, id="EAST-<,#,#,<"),
    ]
@pytest.mark.parametrize("direction,tl_type, tr_type,br_type,bl_type,can_move,can_load,can_unload", can_move_in_direction_by_type_tests)
def test_can_move_in_direction_by_type(direction: int, tl_type: str, tr_type: str, br_type: str, bl_type: str, can_move: bool, can_load: bool, can_unload: bool):
    # setup
    # test
    r_can_move, r_can_load, r_can_unload = can_move_in_direction_by_type(direction,tl_type, tr_type,br_type,bl_type)
    # assert
    check.equal(r_can_move, can_move)
    check.equal(r_can_load, can_load)
    check.equal(r_can_unload, can_unload)
