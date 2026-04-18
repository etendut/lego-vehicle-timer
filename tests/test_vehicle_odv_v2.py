import pytest
from pytest_check import check

from modules.vehicle_odv_v2 import Grid

DEFAULT = ["L#<#U", "X#<#X", "X###X"]
EX3 = ["X#>#X", "L#X#U", "X#<#X"]


def cen(tx, ty):
    return (tx * 800 + 400, ty * 800 + 400)


# --- Task 1: Grid.propose_step ---

propose_step_tests = [
    pytest.param(
        ["L#U"], cen(0, 0), 100, 0, (100, 0),
        id="case01-clear-east-full-step",
    ),
    pytest.param(
        ["L#U"], (2080, 400), 10, 0, (0, 0),
        id="case02-exit-east-grid-edge",
    ),
    pytest.param(
        ["LXU"], cen(0, 0), 100, 0, (0, 0),
        id="case03-wall-blocks-east",
    ),
    pytest.param(
        ["LXU"], cen(0, 0), 0, 100, (0, 0),
        id="case04-exit-south-grid-edge",
    ),
    pytest.param(
        ["L#<#U"], cen(1, 0), 500, 0, (0, 0),
        id="case05-west-barrier-blocks-east",
    ),
    pytest.param(
        ["L#<#U"], cen(2, 0), -500, 0, (-500, 0),
        id="case06-inside-west-only-westbound-allowed",
    ),
    pytest.param(
        ["L#<#U"], cen(3, 0), -500, 0, (-500, 0),
        id="case07-west-of-barrier-westbound-allowed",
    ),
    pytest.param(
        ["L#>#U"], cen(3, 0), -500, 0, (0, 0),
        id="case08-east-barrier-blocks-west",
    ),
    pytest.param(
        ["L#>#U"], cen(2, 0), 500, 0, (500, 0),
        id="case09-inside-east-only-eastbound-allowed",
    ),
    pytest.param(
        DEFAULT, cen(1, 2), 800, -800, (800, -800),
        id="case10-diagonal-corner-cut-both-axes-legal",
    ),
    pytest.param(
        EX3, cen(1, 0), 800, 800, (800, 800),
        id="case11-diagonal-into-wall-per-axis-both-pass",
    ),
    pytest.param(
        ["L#U"], cen(0, 0), 0, 0, (0, 0),
        id="case12-no-op",
    ),
    pytest.param(
        ["L#U"], (400, 400), -400, 0, (0, 0),
        id="case13-exit-west-grid-edge",
    ),
]


@pytest.mark.parametrize(
    "layout,deg_pos,d_deg_x,d_deg_y,expected",
    propose_step_tests,
)
def test_propose_step(layout, deg_pos, d_deg_x, d_deg_y, expected):
    g = Grid(layout)
    result = g.propose_step(deg_pos, d_deg_x, d_deg_y)
    check.equal(result, expected)


def test_case14_load_unload_parse():
    g = Grid(DEFAULT)
    check.equal(g.load_tile, (0, 0))
    check.equal(g.unload_tile, (4, 0))
