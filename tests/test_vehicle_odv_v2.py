import pytest
from pytest_check import check
from unittest.mock import MagicMock

from modules.vehicle_odv_v2 import Grid, VirtualJoystick, AxisController, HomingRoutine

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


# --- Task 2: AxisController ---

def _make_clock(ms=0):
    """Return a mock StopWatch-like whose .time() returns the given value."""
    clock = MagicMock()
    clock.time.return_value = ms
    return clock


def _make_ac(layout, base_duty=45, motor_x_angle=400, motor_y_angle=400, clock_ms=0):
    """Build an AxisController with mock motors over the given layout."""
    grid = Grid(layout)
    motor_x = MagicMock()
    motor_x.angle.return_value = motor_x_angle
    motor_y = MagicMock()
    motor_y.angle.return_value = motor_y_angle
    clock = _make_clock(clock_ms)
    ac = AxisController(motor_x, motor_y, grid, base_duty, _clock=clock)
    return ac, motor_x, motor_y, clock


# --- 2a: zero joystick, both axes already stopped → dc(0) called via ramp ---

def test_zero_joystick_first_tick_no_active_ramp():
    # prev_duty starts at 0; _ramp_stop_axis returns early without calling dc.
    ac, mx, my, _ = _make_ac(["L###U"])
    ac.tick(VirtualJoystick(0, 0))
    mx.dc.assert_not_called()
    my.dc.assert_not_called()


def test_zero_joystick_after_ramp_duration_calls_dc_zero():
    # First drive X active, then go idle; advance clock past _STOP_RAMP_MS.
    ac, mx, my, clock = _make_ac(["L###U"], motor_x_angle=400, motor_y_angle=400)
    # Drive east to record prev_duty_x = 45
    ac.tick(VirtualJoystick(+1, 0))
    mx.dc.assert_called_with(+45)
    # Now zero joystick; clock still at 0 — ramp starts
    ac.tick(VirtualJoystick(0, 0))
    # Clock past ramp duration
    clock.time.return_value = 200
    ac.tick(VirtualJoystick(0, 0))
    # Last call on motor_x must be dc(0)
    last_x = mx.dc.call_args_list[-1]
    check.equal(last_x.args[0], 0)


# --- 2b: joystick (+1, 0) in clear space ---

def test_joystick_x_only_clear_space():
    # motor_x should get dc(+45); motor_y should not be driven (prev_duty 0 → no dc)
    ac, mx, my, _ = _make_ac(["L###U"])
    ac.tick(VirtualJoystick(+1, 0))
    mx.dc.assert_called_with(+45)
    my.dc.assert_not_called()


# --- 2c: joystick (+1, +1) in clear space → 45 * 71 // 100 = 31 ---

def test_diagonal_joystick_speed_compensation():
    ac, mx, my, _ = _make_ac(["L###U"])
    ac.tick(VirtualJoystick(+1, +1))
    check.equal(45 * 71 // 100, 31)  # sanity: formula gives 31
    mx.dc.assert_called_with(+31)
    my.dc.assert_called_with(+31)


# --- 2d: joystick (+1, 0) blocked by east grid edge → ramp path on X ---

def test_joystick_x_blocked_first_tick_ramp_decay():
    # Cart at tile (0,0) centre in a 1-tile grid ["LXU"]; east is blocked.
    # But ["LXU"] is 1-row x 3-cols; cart at (400, 400), step east → X wall.
    # Use a tighter approach: cart near east edge of a 1-col grid.
    # ["L#U"] has cols 0,1,2. Cart at (1680, 400): east face = 2000, grid east = 2400.
    # Step east 40 → east face 2040, still inside. Use ["LU"] instead (1 open col each).
    # Simpler: put cart very close to east grid edge so the lookahead exits bounds.
    # Grid ["L#U"]: n_cols=3, east_bound=2400. Cart at (2080, 400): east face=2400,
    # flush — dc(0) on first tick because _aabb_hits_wall sees R > bound on step.
    # Actually use cart position where east face + 40 > 2400: cx=2080, R=2400, step=40 → R=2440 > 2400.
    ac, mx, my, clock = _make_ac(["L#U"], motor_x_angle=2080, motor_y_angle=400)
    # Drive one tick first so prev_duty is set
    ac.tick(VirtualJoystick(+1, 0))
    # propose_step returns (0,0) because east face 2400+40 > 2400 → blocked
    # So first tick already goes to ramp path; prev_duty=0 initially → no dc call
    # Let's instead start with an active state by driving in clear space first
    # then move to a blocked position.
    ac2, mx2, my2, clock2 = _make_ac(["L###U"], motor_x_angle=400, motor_y_angle=400)
    # Tick east while clear
    ac2.tick(VirtualJoystick(+1, 0))
    mx2.dc.assert_called_with(+45)
    # Now simulate motor moved to blocked position (near east edge of 5-col grid, east=4000)
    # Cart at 3680: east face = 4000, step 40 → 4040 > 4000 → blocked
    mx2.motor_x = MagicMock()
    mx2.motor_x.angle.return_value = 3680
    ac2.motor_x = mx2.motor_x
    clock2.time.return_value = 0
    ac2.tick(VirtualJoystick(+1, 0))
    # Ramp started; elapsed=0, factor=100, duty applied = 45 * 100 // 100 = 45 — same as before.
    # Advance to mid-ramp: elapsed=100ms, factor=(200-100)*100//200=50
    clock2.time.return_value = 100
    ac2.tick(VirtualJoystick(+1, 0))
    mid_call = mx2.motor_x.dc.call_args_list[-1]
    mid_value = mid_call.args[0]
    check.is_true(0 < mid_value < 45, f"mid-ramp value {mid_value} not in (0, 45)")


# --- 2e: ramp completion → dc(0) after _STOP_RAMP_MS elapsed ---

def test_ramp_completion_calls_dc_zero():
    ac, mx, my, clock = _make_ac(["L###U"], motor_x_angle=400, motor_y_angle=400)
    # Drive east to set prev_duty_x
    ac.tick(VirtualJoystick(+1, 0))
    # Move to blocked position (near east edge)
    mx.angle.return_value = 3680
    # First stop tick: start ramp
    clock.time.return_value = 0
    ac.tick(VirtualJoystick(+1, 0))
    # Advance past ramp duration
    clock.time.return_value = 200
    ac.tick(VirtualJoystick(+1, 0))
    last_call = mx.dc.call_args_list[-1]
    check.equal(last_call.args[0], 0)


# --- 2f: deg_pos() returns (motor_x.angle(), motor_y.angle()) ---

def test_deg_pos():
    ac, mx, my, _ = _make_ac(["L###U"], motor_x_angle=1234, motor_y_angle=5678)
    check.equal(ac.deg_pos(), (1234, 5678))


# --- Task 3: HomingRoutine ---

def test_homing_call_sequence_default_grid():
    """DEFAULT grid: unload_tile = (4, 0). Legacy parking:
       motor_y final = 0 + 80; motor_x final = 4*800 + 400 = 3600.
    """
    grid = Grid(DEFAULT)
    motor_x = MagicMock()
    motor_y = MagicMock()
    h = HomingRoutine(motor_x, motor_y, grid)
    h.run()

    # Y stalled north first
    motor_y.run_until_stalled.assert_called_once_with(-200, duty_limit=45)
    motor_y.reset_angle.assert_called_once_with(0)  # uy * 800 = 0
    motor_y.run_angle.assert_called_once_with(1400, 80)

    # Then X stalled east
    motor_x.run_until_stalled.assert_called_once_with(600, duty_limit=45)  # 200*3
    motor_x.reset_angle.assert_called_once_with(4 * 800 + 720)  # ux*800 + 720
    motor_x.run_target.assert_called_once_with(1400, 4 * 800 + 400)  # centre on U


def test_homing_uses_unload_tile_from_grid():
    """EX3 grid has unload at (4, 1)."""
    grid = Grid(EX3)
    check.equal(grid.unload_tile, (4, 1))
    motor_x = MagicMock()
    motor_y = MagicMock()
    HomingRoutine(motor_x, motor_y, grid).run()

    motor_y.reset_angle.assert_called_once_with(1 * 800)  # uy=1
    motor_x.reset_angle.assert_called_once_with(4 * 800 + 720)
    motor_x.run_target.assert_called_once_with(1400, 4 * 800 + 400)


def test_homing_order_y_then_x():
    """Y stall+reset+back-off must happen before X stall+reset+centre."""
    grid = Grid(DEFAULT)
    motor_x = MagicMock()
    motor_y = MagicMock()
    parent = MagicMock()
    parent.attach_mock(motor_x, 'x')
    parent.attach_mock(motor_y, 'y')

    HomingRoutine(motor_x, motor_y, grid).run()

    names = [c[0] for c in parent.mock_calls]
    # Only care about the high-level ordering of Y vs X actions
    y_idx = names.index('y.run_until_stalled')
    x_idx = names.index('x.run_until_stalled')
    check.less(y_idx, x_idx)
    y_reset_idx = names.index('y.reset_angle')
    x_reset_idx = names.index('x.reset_angle')
    check.less(y_reset_idx, x_reset_idx)
