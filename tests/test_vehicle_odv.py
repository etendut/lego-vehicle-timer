import pytest
from pytest_check import check
from unittest.mock import MagicMock

from pybricks.parameters import Button

from modules.vehicle_odv import (
    Grid, VirtualJoystick, AxisController, HomingRoutine, Planner, AutoDriver,
    IdleTimeout, RunODVMotors, MANUAL, HYBRID, AUTO,
)

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


# --- Boundary overrun / escape (motor coast past edge) ---
# _HALF = 320, so legal north edge is cy >= 320 for any grid.
# Cart at cy=255 is 65° past the north boundary — both ±40 steps land
# in an illegal position, but the southward step reduces the overlap so
# the escape logic must allow it while blocking the northward step.

_OVERRUN_GRID = ["L#U"]   # 1 row, 3 cols — simple boundary test surface


def test_boundary_overrun_escape_south():
    """Escape toward centre is allowed when cart is past north boundary."""
    g = Grid(_OVERRUN_GRID)
    pos = (400, 255)        # 65° past north edge (legal min = 320)
    assert g.propose_step(pos, 0, 40) == (0, 40)   # southward escape permitted


def test_boundary_overrun_blocked_north():
    """Moving further into the north boundary is blocked."""
    g = Grid(_OVERRUN_GRID)
    pos = (400, 255)
    assert g.propose_step(pos, 0, -40) == (0, 0)   # deeper into wall → blocked


def test_boundary_overrun_escape_east():
    """Escape toward centre is allowed when cart is past east boundary."""
    # Legal east edge: cx <= n_cols*800 - 320 = 2080; cart at 2100 is 40° past.
    g = Grid(_OVERRUN_GRID)
    pos = (2100, 400)
    assert g.propose_step(pos, -40, 0) == (-40, 0)  # westward escape permitted


def test_boundary_overrun_blocked_east():
    """Moving further into the east boundary is blocked."""
    g = Grid(_OVERRUN_GRID)
    pos = (2100, 400)
    assert g.propose_step(pos, 40, 0) == (0, 0)     # deeper into wall → blocked


def test_boundary_overlap_zero_when_legal():
    """_boundary_overlap returns 0 for a position safely inside the grid."""
    g = Grid(_OVERRUN_GRID)
    assert g._boundary_overlap(400, 400) == 0


def test_boundary_overlap_nonzero_past_north():
    """_boundary_overlap returns the overlap amount when past north edge."""
    g = Grid(_OVERRUN_GRID)
    # cy=255: top face = 255-320 = -65, overlap = 65
    assert g._boundary_overlap(400, 255) == 65


def test_boundary_overlap_decreases_on_escape():
    """Overlap is strictly smaller after an escape step than before."""
    g = Grid(_OVERRUN_GRID)
    before = g._boundary_overlap(400, 255)
    after  = g._boundary_overlap(400, 295)
    assert after < before


# Wall-tile escape: cart coasted into a wall tile rect (not a grid boundary).
# Grid: row 1 col 2 is a wall — cart at ~(2*800+400, 400) = (2000, 400) is fine,
# but if it drifts east past cx=2080 (HALF=320 away from wall left edge at 2400)
# the right face clips the wall. Must be able to drive west to escape.
_WALL_ESCAPE_GRID = ["L###U", "##X##"]  # wall tile at (2,1)


def test_wall_tile_overrun_escape_west():
    """Escape westward is allowed when cart is clipped into a wall tile on the east."""
    g = Grid(_WALL_ESCAPE_GRID)
    # Wall tile (2,1): wl=1600, wt=800. Cart at row 1 center y=1200.
    # Right face clips wall when cx + 320 > 1600 → cx > 1280.
    # Place cart at 1300 (right face = 1620, 20° into wall).
    pos = (1300, 1200)
    assert g._aabb_hits_wall(*pos), "setup: position must be in wall"
    assert g.propose_step(pos, -40, 0) == (-40, 0)   # westward escape permitted


def test_wall_tile_overrun_blocked_east():
    """Driving further into the wall tile is blocked."""
    g = Grid(_WALL_ESCAPE_GRID)
    pos = (1300, 1200)
    assert g.propose_step(pos, 40, 0) == (0, 0)       # deeper into wall → blocked


def test_boundary_overlap_includes_wall_tile_penetration():
    """_boundary_overlap counts wall tile penetration, not just grid edges."""
    g = Grid(_WALL_ESCAPE_GRID)
    # At (1300, 1200): right face = 1620, wall left edge = 1600 → 20° penetration
    assert g._boundary_overlap(1300, 1200) == 20
    assert g._boundary_overlap(1260, 1200) == 0   # right face = 1580 < 1600, clear


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


def test_tick_ramp_ms_override_shortens_decay():
    """AutoDriver passes ramp_ms=100 (half of manual 200ms) so motor coast past
    the deadband fits inside the 80° wall-clearance corridor. Rig-measured: at
    full 200ms ramp + 80% duty the cart bounces off walls; 100ms keeps coast
    under 80° and preserves smooth deceleration (no hard-stop jerk)."""
    ac, mx, _my, clock = _make_ac(["L###U"], motor_x_angle=400, motor_y_angle=400)
    ac.tick(VirtualJoystick(+1, 0))  # prime prev_duty_x = 45
    mx.dc.assert_called_with(+45)
    # Idle tick with ramp_ms=100; clock at 0 → ramp starts.
    ac.tick(VirtualJoystick(0, 0), ramp_ms=100)
    # Advance past 100ms; next idle tick must finalize to dc(0).
    clock.time.return_value = 100
    ac.tick(VirtualJoystick(0, 0), ramp_ms=100)
    check.equal(mx.dc.call_args_list[-1].args[0], 0)


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
    """DEFAULT grid: unload_tile = (4, 0). Cart-center frame parking:
       at N-stall motor_y = _HALF (320); at E-stall motor_x = n_cols*800 - _HALF (3680);
       run_target lands cart center on U's tile center (3600, 400).
    """
    grid = Grid(DEFAULT)
    motor_x = MagicMock()
    motor_y = MagicMock()
    h = HomingRoutine(motor_x, motor_y, grid)
    h.run()

    # Y stalled north first
    motor_y.run_until_stalled.assert_called_once_with(-200, duty_limit=45)
    motor_y.reset_angle.assert_called_once_with(320)  # _HALF = cart center south of N wall
    motor_y.run_target.assert_called_once_with(1400, 0 * 800 + 400)  # U tile center Y

    # Then X stalled east — reset to east_wall_deg - 80 (rig-measured stall offset)
    motor_x.run_until_stalled.assert_called_once_with(600, duty_limit=45)  # 200*3
    motor_x.reset_angle.assert_called_once_with(5 * 800 - 80)
    motor_x.run_target.assert_called_once_with(1400, 4 * 800 + 400)  # U tile center X


def test_homing_uses_unload_tile_from_grid():
    """EX3 grid has unload at (4, 1)."""
    grid = Grid(EX3)
    check.equal(grid.unload_tile, (4, 1))
    motor_x = MagicMock()
    motor_y = MagicMock()
    HomingRoutine(motor_x, motor_y, grid).run()

    motor_y.reset_angle.assert_called_once_with(320)  # always _HALF (cart at N wall)
    motor_y.run_target.assert_called_once_with(1400, 1 * 800 + 400)  # uy=1 tile center
    motor_x.reset_angle.assert_called_once_with(5 * 800 - 80)
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


# --- Task 4: Planner ---

EX2 = ["X###X", "L###U", "X###X"]


def test_planner_same_tile_returns_single_waypoint():
    g = Grid(EX2)
    check.equal(Planner(g).plan((0, 1), (0, 1)), ((0, 1),))


def test_planner_ex2_straight_east():
    g = Grid(EX2)
    check.equal(Planner(g).plan((0, 1), (4, 1)), ((0, 1), (4, 1)))


def test_planner_default_l_to_u():
    g = Grid(DEFAULT)
    # L at (0,0), U at (4,0). Corner-cut pass replaces the (3,2) E->N corner
    # with (2,2): from (2,2) the cart can safely cut NE toward (3,0). The
    # (1,2) S->E corner is kept because cutting via (1,1) would try to enter
    # '<' at (2,1) eastbound. The (3,0) N->E corner is kept because cutting
    # via (3,1) would drive the cart into 'X' at (4,1).
    result = Planner(g).plan((0, 0), (4, 0))
    check.equal(result, ((0, 0), (1, 0), (1, 2), (2, 2), (3, 0), (4, 0)))


def test_planner_corner_cut_skipped_when_barrier_intervenes():
    # Corner-cut from (1,1) into (3,2) SE would cross '<' at (2,1) eastbound
    # -> the cut must be rejected and (1,2) retained as the S->E turn point.
    g = Grid(DEFAULT)
    p = Planner(g)
    check.is_false(p._safe_diagonal((1, 1), (3, 2)),
                   "SE cut through '<' at (2,1) should be rejected")


def test_planner_corner_cut_taken_when_safe():
    g = Grid(DEFAULT)
    p = Planner(g)
    check.is_true(p._safe_diagonal((2, 2), (3, 0)),
                  "NE cut (2,2)->(3,0) should be safe")


def test_planner_corner_cut_skipped_when_wall_blocks():
    # Cutting (3,0) N->E corner via (3,1) would route cart through 'X' at
    # (4,1). AABB sweep rejects it.
    g = Grid(DEFAULT)
    p = Planner(g)
    check.is_false(p._safe_diagonal((3, 1), (4, 0)),
                   "NE cut (3,1)->(4,0) should fail due to 'X' at (4,1)")


def test_planner_default_u_to_l():
    g = Grid(DEFAULT)
    # Westbound through '<' is the arrow direction — allowed.
    # Direct west all the way, compressed to endpoints only.
    result = Planner(g).plan((4, 0), (0, 0))
    check.equal(result, ((4, 0), (0, 0)))


def test_planner_ex3_l_to_u():
    g = Grid(EX3)
    # (2,1)=X in the middle blocks the straight shot. Path goes north
    # through '>' (eastbound allowed), east, then south to U.
    result = Planner(g).plan((0, 1), (4, 1))
    check.equal(result, ((0, 1), (1, 1), (1, 0), (3, 0), (3, 1), (4, 1)))


def test_planner_unreachable_returns_empty():
    # A grid fully walled off between start and goal.
    walled = ["LX#U"]
    g = Grid(walled)
    check.equal(Planner(g).plan((0, 0), (3, 0)), ())


def test_planner_eastbound_into_west_only_blocked():
    # '<' blocks eastbound entry; must detour.
    # Simple 2-row grid: eastbound direct is blocked by '<' at (2,0),
    # detour south available.
    layout = ["L#<#U", "#####"]
    g = Grid(layout)
    result = Planner(g).plan((0, 0), (4, 0))
    # Must go south to escape '<'; can't enter (2,0) eastbound.
    check.is_true((1, 1) in result or (2, 1) in result,
                  f"expected detour through row 1, got {result}")


def test_planner_westbound_into_east_only_blocked():
    # '>' blocks westbound entry from east; must detour.
    layout = ["L#>#U", "#####"]
    g = Grid(layout)
    result = Planner(g).plan((4, 0), (0, 0))
    # Westbound from (3,0) to (2,0)='>' is blocked.
    check.is_true(len(result) > 2, f"expected detour, got {result}")


# --- Task 5: AutoDriver ---

def _mock_remote(pressed=()):
    remote = MagicMock()
    remote.buttons.pressed.return_value = pressed
    return remote


def _mock_ac(cx=0, cy=0):
    ac = MagicMock()
    ac.deg_pos.return_value = (cx, cy)
    return ac


def test_autodriver_start_journey_plans_and_resets_index():
    g = Grid(["L#U"])
    planner = Planner(g)
    ac = _mock_ac()
    ad = AutoDriver(g, planner, ac)
    ad.i = 99  # pretend we had a prior journey
    ad.start_journey(g.load_tile, g.unload_tile)
    check.equal(ad.i, 0)
    check.is_true(len(ad.waypoints) >= 2)
    check.equal(ad.waypoints[0], g.load_tile)
    check.equal(ad.waypoints[-1], g.unload_tile)


def test_autodriver_yields_on_remote_press():
    g = Grid(["L#U"])
    ac = _mock_ac(400, 400)
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)

    remote = _mock_remote(pressed=["Button.CENTER"])
    check.equal(ad.tick(remote), 'yielded')
    ac.tick.assert_not_called()


def test_autodriver_emits_joystick_toward_next_waypoint():
    # L at (0,0), U at (2,0). Cart at (0,0) centre = (400, 400).
    # Expected next waypoint is (2,0); joystick = (+1, 0).
    g = Grid(["L#U"])
    ac = _mock_ac(400, 400)
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)

    result = ad.tick(_mock_remote())
    check.is_none(result)
    ac.tick.assert_called_once()
    vj = ac.tick.call_args.args[0]
    check.equal(vj.ax, +1)
    check.equal(vj.ay, 0)


def test_autodriver_drives_axis_controller_with_auto_duty_and_ramp():
    """AutoDriver runs at _AUTO_DRIVE_DUTY with _AUTO_STOP_RAMP_MS for the idle-axis
    ramp (shorter than manual 200ms so coast fits wall clearance)."""
    import modules.vehicle_odv as odv
    g = Grid(["L#U"])
    ac = _mock_ac(400, 400)
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)

    ad.tick(_mock_remote())
    check.equal(ac.tick.call_args.kwargs.get('duty'), odv._AUTO_DRIVE_DUTY)
    check.equal(ac.tick.call_args.kwargs.get('ramp_ms'), odv._AUTO_STOP_RAMP_MS)


def test_autodriver_deadband_scales_with_duty_and_ramp():
    """AutoDriver's _aim deadband is scaled to predicted coast distance at current
    auto duty + ramp so the motor stops pushing exactly when the remaining coast
    carries the cart onto the target — no overshoot, no wall bounce.
    With _AUTO_DRIVE_DUTY=80, _AUTO_STOP_RAMP_MS=100, _MAX_MOTOR_ROT_SPEED=1400:
    coast = 80 * 1400 * 100 // 200000 = 56°; + _DEADBAND_SAFETY_DEG=10 → 66°."""
    g = Grid(["L#U#U"])  # L at (0,0), U at (4,0) — clear corridor
    # Target for the next waypoint is tile center of U = (4*800+400, 400) = (3600, 400).
    # Y drift 200° is outside both the 66° deadband AND the 160° aim-switch window
    # so the tick emits a VJ instead of advancing i to the end.
    ac = _mock_ac(3600 - 50, 400 + 200)
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)

    ad.tick(_mock_remote())
    vj = ac.tick.call_args.args[0]
    check.equal(vj.ax, 0)   # 50° X drift is inside 66° dynamic deadband
    check.equal(vj.ay, -1)  # 200° Y drift is outside → push north


def test_autodriver_advances_index_when_near_waypoint():
    # Two-waypoint path L(0,0) -> U(2,0). Cart placed within aim-switch
    # tolerance of (2,0) centre = (2000, 400). Within 160° on both axes.
    g = Grid(["L#U"])
    ac = _mock_ac(1900, 450)  # |Δ|=(100, 50), within 160
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)
    # Waypoints: ((0,0), (2,0)). Index 0 targets (2,0). When we arrive,
    # i goes to 1 which == len-1 -> end.
    check.equal(ad.tick(_mock_remote()), 'reached_unload')


def test_autodriver_reports_reached_load():
    g = Grid(["L#U"])
    ac = _mock_ac(400, 400)  # at load-tile centre
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.unload_tile, g.load_tile)
    check.equal(ad.tick(_mock_remote()), 'reached_load')


def test_autodriver_unreachable_returns_reached_end():
    # Goal surrounded by walls -> empty plan.
    g = Grid(["LX#U"])
    ac = _mock_ac(400, 400)
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey((0, 0), (3, 0))
    check.equal(ad.waypoints, ())
    check.equal(ad.tick(_mock_remote()), 'reached_end')


def test_autodriver_diagonal_aim():
    # Multi-waypoint journey where the next waypoint requires a diagonal
    # joystick. DEFAULT plan from L has waypoints including (2,2); place
    # cart at (1,2) centre so the next aim is SE toward (2,2).
    g = Grid(DEFAULT)
    ac = _mock_ac(400, 400)  # at L
    ad = AutoDriver(g, Planner(g), ac)
    ad.start_journey(g.load_tile, g.unload_tile)
    # First real tick: cart at L centre, target waypoint (1,0) center=(1200,400)
    # -> joystick (+1, 0).
    ad.tick(_mock_remote())
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (+1, 0))

    # Now move cart to (1,0) centre to cross aim-switch; next tick should
    # advance and emit joystick toward (1,2) -> pure south (0, +1).
    ac.deg_pos.return_value = (1200, 400)
    ad.tick(_mock_remote())
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (0, +1))


# --- Task 6: drive-mode enum + IdleTimeout ---

def test_drive_mode_enum_distinct_values():
    check.equal(MANUAL, 0)
    check.equal(HYBRID, 1)
    check.equal(AUTO, 2)


def test_idle_timeout_not_fired_after_reset():
    clock = MagicMock()
    clock.time.return_value = 0
    t = IdleTimeout(30, _clock=clock)
    check.is_false(t.fired())
    # advance below threshold
    clock.time.return_value = 29_000
    check.is_false(t.fired())


def test_idle_timeout_fires_after_interval():
    clock = MagicMock()
    clock.time.return_value = 0
    t = IdleTimeout(30, _clock=clock)
    clock.time.return_value = 30_000
    check.is_true(t.fired())


def test_idle_timeout_reset_extends_deadline():
    clock = MagicMock()
    clock.time.return_value = 0
    t = IdleTimeout(30, _clock=clock)
    clock.time.return_value = 25_000
    t.reset()
    # 20s after reset -> 45_000 total, but reset anchors to 25_000
    clock.time.return_value = 45_000
    check.is_false(t.fired())
    clock.time.return_value = 55_000  # 30s after reset
    check.is_true(t.fired())


# --- Task 7: RunODVMotors ---


def _make_rom(layout=None, pressed=(), motor_x_angle=400, motor_y_angle=400):
    if layout is None:
        layout = DEFAULT
    efc = MagicMock()
    mx = MagicMock()
    my = MagicMock()
    mx.angle.return_value = motor_x_angle
    my.angle.return_value = motor_y_angle
    rem = MagicMock()
    rem.buttons.pressed.return_value = pressed
    clock = MagicMock()
    clock.time.return_value = 0
    rom = RunODVMotors(efc, 45, layout,
                       _motors=(mx, my), _remote=rem, _clock=clock)
    return rom, mx, my, rem


def test_rom_init_builds_stack():
    rom, mx, my, _ = _make_rom()
    check.is_instance(rom.grid, Grid)
    check.is_instance(rom.planner, Planner)
    check.is_instance(rom.axis_controller, AxisController)
    check.is_instance(rom.auto_driver, AutoDriver)
    check.is_instance(rom.homing_routine, HomingRoutine)
    # idle_timeout is only constructed in HYBRID mode
    from modules.vehicle_odv import DRIVE_MODE, HYBRID
    if DRIVE_MODE == HYBRID:
        check.is_instance(rom.idle_timeout, IdleTimeout)
    else:
        check.is_none(rom.idle_timeout)
    check.is_false(rom.has_load)
    check.is_false(rom.mh_is_homed)
    # supports_homing=True, supports_flip=False
    check.is_true(rom.mh_supports_homing)
    check.is_false(rom.mh_supports_flip)
    # Constructor calls stop on both motors
    mx.stop.assert_called_once()
    my.stop.assert_called_once()


def test_rom_stop_motors_stops_both():
    rom, mx, my, _ = _make_rom()
    mx.stop.reset_mock()
    my.stop.reset_mock()
    rom.stop_motors()
    mx.stop.assert_called_once()
    my.stop.assert_called_once()


def test_rom_home_and_unload_delegates_sets_state():
    rom, mx, my, _ = _make_rom()
    rom.has_load = True
    rom.home_and_unload()
    # Delegated to HomingRoutine.run()
    my.run_until_stalled.assert_called_once()
    mx.run_until_stalled.assert_called_once()
    check.is_false(rom.has_load)
    check.is_true(rom.mh_is_homed)


def test_rom_reset_homing_clears_is_homed():
    rom, _, _, _ = _make_rom()
    rom.set_is_homed()
    rom.reset_homing()
    check.is_false(rom.mh_is_homed)


def _stub_axis_controller(rom, cx=1200, cy=1200):
    """Replace axis_controller with a MagicMock whose deg_pos() returns (cx, cy)."""
    ac = MagicMock()
    ac.deg_pos.return_value = (cx, cy)
    rom.axis_controller = ac
    return ac


def test_rom_handle_remote_press_empty_stops():
    rom, _, _, _ = _make_rom(pressed=())
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (0, 0))


def test_rom_handle_remote_press_left_plus_north():
    rom, _, _, _ = _make_rom(pressed=[Button.LEFT_PLUS])
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (0, -1))


def test_rom_handle_remote_press_right_plus_east():
    rom, _, _, _ = _make_rom(pressed=[Button.RIGHT_PLUS])
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (+1, 0))


def test_rom_handle_remote_press_diagonal_ne():
    rom, _, _, _ = _make_rom(pressed=[Button.LEFT_PLUS, Button.RIGHT_PLUS])
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (+1, -1))


def test_rom_handle_remote_press_center_button_stops():
    rom, _, _, _ = _make_rom(pressed=[Button.LEFT])
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    vj = ac.tick.call_args.args[0]
    check.equal((vj.ax, vj.ay), (0, 0))


def test_rom_handle_remote_press_resets_idle_timeout():
    rom, _, _, _ = _make_rom(pressed=[Button.LEFT_PLUS])
    _stub_axis_controller(rom)
    rom.idle_timeout = MagicMock()
    rom.handle_remote_press()
    rom.idle_timeout.reset.assert_called_once()


def test_rom_handle_remote_press_remote_disabled_returns():
    rom, _, _, _ = _make_rom(pressed=[Button.LEFT_PLUS])
    rom.mh__remote_disabled = True
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    ac.tick.assert_not_called()


def test_rom_handle_remote_press_at_unload_east_triggers_home():
    # Place cart at U tile centre; pressing east triggers home_and_unload.
    rom, mx, my, _ = _make_rom(pressed=[Button.RIGHT_PLUS])
    _stub_axis_controller(rom, *cen(4, 0))
    rom.has_load = True
    rom.handle_remote_press()
    # home_and_unload ran -> motors stalled and state updated
    my.run_until_stalled.assert_called_once()
    mx.run_until_stalled.assert_called_once()
    check.is_false(rom.has_load)
    check.is_true(rom.mh_is_homed)


def test_rom_handle_remote_press_at_load_west_triggers_load():
    # Place cart at L tile centre; pressing west triggers _do_load_.
    rom, mx, _, _ = _make_rom(pressed=[Button.RIGHT_MINUS])
    _stub_axis_controller(rom, *cen(0, 0))
    rom.handle_remote_press()
    check.is_true(rom.has_load)
    # Two run_target calls: dip and return
    check.equal(mx.run_target.call_count, 2)


def test_rom_auto_load_requires_homed():
    rom, _, _, _ = _make_rom()
    rom.auto_driver = MagicMock()
    rom.auto_load()
    rom.auto_driver.start_journey.assert_not_called()


def test_rom_auto_unload_requires_has_load():
    rom, _, _, _ = _make_rom()
    rom.set_is_homed()
    rom.auto_driver = MagicMock()
    rom.auto_unload()
    rom.auto_driver.start_journey.assert_not_called()


def test_rom_auto_load_journey_loop_completes():
    rom, mx, _, _ = _make_rom(motor_x_angle=cen(4, 0)[0], motor_y_angle=cen(4, 0)[1])
    rom.set_is_homed()
    ad = MagicMock()
    ad.tick.side_effect = [None, None, 'reached_load']
    rom.auto_driver = ad
    rom.auto_load()
    ad.start_journey.assert_called_once_with((4, 0), rom.grid.load_tile)
    check.equal(ad.tick.call_count, 3)
    check.is_true(rom.has_load)
    # 1 run_target from _drive_auto_journey force-park + 2 from _do_load_
    check.equal(mx.run_target.call_count, 3)


def test_rom_auto_load_journey_yields_on_interrupt():
    rom, mx, my, _ = _make_rom(motor_x_angle=cen(4, 0)[0], motor_y_angle=cen(4, 0)[1])
    rom.set_is_homed()
    rom.enable_auto_drive()
    ad = MagicMock()
    ad.tick.side_effect = [None, 'yielded']
    rom.auto_driver = ad
    rom.auto_load()
    check.is_false(rom.mh_auto_drive)  # disable_auto_drive called
    check.is_false(rom.has_load)       # no load on yield


def test_rom_idle_timed_out_reflects_idle_timeout():
    rom, _, _, _ = _make_rom()
    rom.idle_timeout = MagicMock()
    rom.idle_timeout.fired.return_value = False
    check.is_false(rom.idle_timed_out())
    rom.idle_timeout.fired.return_value = True
    check.is_true(rom.idle_timed_out())


def test_rom_idle_timed_out_false_when_timer_absent():
    rom, _, _, _ = _make_rom()
    rom.idle_timeout = None
    check.is_false(rom.idle_timed_out())


def test_rom_reset_idle_timeout_delegates():
    rom, _, _, _ = _make_rom()
    rom.idle_timeout = MagicMock()
    rom.reset_idle_timeout()
    rom.idle_timeout.reset.assert_called_once()


def test_rom_reset_idle_timeout_noop_when_timer_absent():
    rom, _, _, _ = _make_rom()
    rom.idle_timeout = None
    rom.reset_idle_timeout()  # must not raise


def test_rom_auto_unload_journey_completes_then_homes():
    rom, mx, my, _ = _make_rom(motor_x_angle=cen(0, 0)[0], motor_y_angle=cen(0, 0)[1])
    rom.set_is_homed()
    rom.has_load = True
    ad = MagicMock()
    ad.tick.side_effect = ['reached_unload']
    rom.auto_driver = ad
    rom.auto_unload()
    ad.start_journey.assert_called_once_with((0, 0), rom.grid.unload_tile)
    # home_and_unload ran
    my.run_until_stalled.assert_called_once()
    check.is_false(rom.has_load)
    check.is_true(rom.mh_is_homed)


# --- Block manual movement into the load / unload tiles ---

def test_overlaps_special_false_when_outside():
    # Cart at tile-1 centre (a regular corridor tile) overlaps neither special tile.
    g = Grid(["L#<#U"])
    assert g._overlaps_special(*cen(2, 0)) is False


def test_overlaps_special_true_when_at_unload_centre():
    g = Grid(["L#<#U"])
    assert g._overlaps_special(*cen(4, 0)) is True


def test_overlaps_special_true_when_at_load_centre():
    g = Grid(["L#<#U"])
    assert g._overlaps_special(*cen(0, 0)) is True


def test_propose_step_block_special_blocks_eastward_entry_into_unload():
    # From tile 3, moving east would overlap the unload tile → blocked.
    g = Grid(["L#<#U"])
    result = g.propose_step(cen(3, 0), 500, 0, block_special=True)
    check.equal(result, (0, 0))


def test_propose_step_default_allows_entry_into_unload():
    # Without the flag the step is valid (existing behaviour preserved).
    g = Grid(["L#<#U"])
    result = g.propose_step(cen(3, 0), 500, 0)
    check.equal(result, (500, 0))


def test_propose_step_block_special_allows_escape_from_unload():
    # Cart already at unload centre (placed there by auto-drive) — escape west must succeed.
    g = Grid(["L#<#U"])
    result = g.propose_step(cen(4, 0), -500, 0, block_special=True)
    check.equal(result, (-500, 0))


def test_propose_step_block_special_blocks_westward_entry_into_load():
    # From tile 1, moving west would overlap the load tile → blocked.
    g = Grid(["L#<#U"])
    result = g.propose_step(cen(1, 0), -500, 0, block_special=True)
    check.equal(result, (0, 0))


def test_propose_step_block_special_allows_escape_from_load():
    # Cart already at load centre (placed there by auto-drive) — escape east must succeed.
    g = Grid(["L#<#U"])
    result = g.propose_step(cen(0, 0), 500, 0, block_special=True)
    check.equal(result, (500, 0))


def test_handle_remote_press_passes_block_special_to_tick():
    # Normal movement tick must be called with block_special=True.
    rom, _, _, _ = _make_rom(pressed=[Button.RIGHT_PLUS])
    ac = _stub_axis_controller(rom)
    rom.handle_remote_press()
    call_kwargs = ac.tick.call_args.kwargs
    check.equal(call_kwargs.get('block_special'), True)
