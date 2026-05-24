"""Headless end-to-end simulation of ODV auto-drive journeys.

Runs against the compiled `docs/pybricks/lego_vehicle_timer_odv.py` so splice
bugs and compiled-only state (e.g. the `remote = None` initialiser) are
exercised alongside the journey logic.

Physical model: each `motor.dc(duty%)` sets a constant duty. Each `wait(ms)`
advances both motor angles by `duty% * 1400 deg/s * ms`. `run_target` /
`reset_angle` / `run_until_stalled` are modelled as instant.
"""
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import pytest

PROJECT_ROOT = Path(__file__).parent.parent.resolve()
COMPILED_PATH = PROJECT_ROOT / 'docs' / 'pybricks' / 'lego_vehicle_timer_odv.py'

MAX_TICKS_PER_JOURNEY = 5000  # 50s of wait(10) — way beyond a real journey


class FakeMotor:
    """Stateful motor stub: `dc(duty)` sets duty, `advance(dt_ms)` integrates angle."""

    def __init__(self, angle: int = 0) -> None:
        self._angle: int = angle
        self._duty: int = 0

    def angle(self) -> int:
        return self._angle

    def dc(self, duty: int) -> None:
        self._duty = duty

    def stop(self) -> None:
        self._duty = 0

    def brake(self) -> None:
        self._duty = 0

    def reset_angle(self, a: int) -> None:
        self._angle = a
        self._duty = 0

    def run_target(self, speed: int, target: int, then=None, wait: bool = True) -> None:
        del speed, then, wait
        self._angle = target
        self._duty = 0

    def run_angle(self, speed: int, angle: int, then=None, wait: bool = True) -> None:
        del speed, then, wait
        self._angle += angle
        self._duty = 0

    def run_until_stalled(self, speed: int, duty_limit: int | None = None) -> int:
        del speed, duty_limit
        self._duty = 0
        return 0

    def advance(self, dt_ms: int) -> None:
        # 1400 deg/s at 100% duty → 1.4 deg/ms per % → integer math to match hub.
        self._angle += self._duty * 14 * dt_ms // 1000


class FakeClock:
    """Monotonic clock stub advanced manually by the sim loop."""

    def __init__(self) -> None:
        self._t: int = 0

    def time(self) -> int:
        return self._t

    def advance(self, dt_ms: int) -> None:
        self._t += dt_ms


@pytest.fixture
def compiled():
    spec = importlib.util.spec_from_file_location('_compiled_odv_sim', COMPILED_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_rom(m, layout):
    motor_x = FakeMotor()
    motor_y = FakeMotor()
    clock = FakeClock()
    remote = MagicMock()
    remote.buttons.pressed.return_value = ()
    rom = m.RunODVMotors(
        m.ErrorFlashCodes(), m.ODV_SPEED, layout,
        _motors=(motor_x, motor_y), _remote=remote, _clock=clock,
    )
    return rom, motor_x, motor_y, clock


def _install_sim_wait(m, motor_x, motor_y, clock, max_ticks=MAX_TICKS_PER_JOURNEY):
    state = {'ticks': 0}

    def fake_wait(ms):
        motor_x.advance(ms)
        motor_y.advance(ms)
        clock.advance(ms)
        state['ticks'] += 1
        if state['ticks'] > max_ticks:
            raise RuntimeError(
                'journey watchdog exceeded {} ticks at pos=({}, {})'.format(
                    max_ticks, motor_x.angle(), motor_y.angle()
                )
            )

    m.wait = fake_wait
    return state


def _park_on_tile(m, motor_x, motor_y, tile):
    cx, cy = m.Grid(m.ODV_GRID_DEFAULT).tile_center_deg(tile)
    motor_x.reset_angle(cx)
    motor_y.reset_angle(cy)


def test_journey_unload_to_load_on_default_grid(compiled):
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)

    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True

    reached = rom._drive_auto_journey(rom.grid.load_tile)

    assert reached is True
    final_tile = rom.grid.deg_to_tile((mx.angle(), my.angle()))
    assert final_tile == rom.grid.load_tile


def test_journey_parks_exactly_at_goal_tile_center(compiled):
    """AutoDriver emits 'reached' before hitting tile center — short by ~160° on rig.
    _drive_auto_journey must force-park at the exact goal tile center on arrival so
    load/unload sequences (which assume cart-at-tile-center) start from the right spot."""
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)
    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True

    reached = rom._drive_auto_journey(rom.grid.load_tile)

    assert reached is True
    expected = rom.grid.tile_center_deg(rom.grid.load_tile)
    assert (mx.angle(), my.angle()) == expected


def test_journey_load_to_unload_on_default_grid(compiled):
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)

    _park_on_tile(m, mx, my, rom.grid.load_tile)
    rom.mh_is_homed = True

    reached = rom._drive_auto_journey(rom.grid.unload_tile)

    assert reached is True
    final_tile = rom.grid.deg_to_tile((mx.angle(), my.angle()))
    assert final_tile == rom.grid.unload_tile


def test_homing_parks_on_unload_tile_center(compiled):
    """HomingRoutine must land the cart at U's tile center in the grid frame,
    so the AABB collision model doesn't see the cart wedged in the N/E walls."""
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)

    rom.homing_routine.run()

    expected = rom.grid.tile_center_deg(rom.grid.unload_tile)
    assert (mx.angle(), my.angle()) == expected


def test_journey_from_homed_position_reaches_load(compiled):
    """After HomingRoutine.run(), the auto-drive journey to L must succeed
    (regression: Y was parking at wall-offset 80 instead of tile center 400,
    causing the AABB to clip every tick to (0, 0))."""
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)

    rom.homing_routine.run()
    rom.mh_is_homed = True

    reached = rom._drive_auto_journey(rom.grid.load_tile)

    assert reached is True
    assert rom.grid.deg_to_tile((mx.angle(), my.angle())) == rom.grid.load_tile


def test_journey_stops_motors_on_arrival(compiled):
    """On arrival, _drive_auto_journey must stop motors so subsequent actions
    (home_and_unload, _do_load_) start from rest — not mid-stride.
    Regression: cart slid diagonally into Y wall because X duty carried over
    into home_and_unload's Y stall."""
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    _install_sim_wait(m, mx, my, clock)
    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True

    reached = rom._drive_auto_journey(rom.grid.load_tile)

    assert reached is True
    assert mx._duty == 0  # motors stopped before return
    assert my._duty == 0


def test_autodriver_zero_dy_inside_deadband(compiled):
    """AutoDriver must emit ay=0 when cart Y is within deadband of the target
    Y line — small drift shouldn't pulse the idle axis.
    Regression: _sign(dy) was ±1 for any nonzero dy, causing ±10° Y wobble
    during straight-X transits on the rig."""
    m = compiled
    grid = m.Grid(m.ODV_GRID_DEFAULT)
    planner = MagicMock()
    planner.plan.return_value = ((4, 0), (0, 0))
    ac = MagicMock()
    ac.deg_pos.return_value = (3459, 396)  # rig-observed mid-transit drift

    driver = m.AutoDriver(grid, planner, ac)
    driver.start_journey((4, 0), (0, 0))
    driver.tick(remote=None)

    vj = ac.tick.call_args[0][0]
    assert vj.ax == -1  # still pushing west toward (400, 400)
    assert vj.ay == 0   # 4° Y drift is inside deadband — no pulse


def test_auto_load_calibration_stops_at_tile_3_0(compiled, monkeypatch):
    """CALIBRATION HACK: with _CALIBRATE_X_OFFSET flag set, auto_load must drive
    to tile (3, 0), park at center, and raise SystemExit so the X offset can be
    measured on rig. Flag defaults off — normal auto_load is unaffected."""
    m = compiled
    monkeypatch.setattr(m, '_CALIBRATE_X_OFFSET', True)
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    _install_sim_wait(m, mx, my, clock)
    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True

    with pytest.raises(SystemExit):
        rom.auto_load()

    expected = rom.grid.tile_center_deg((3, 0))
    assert (mx.angle(), my.angle()) == expected
    assert mx._duty == 0
    assert my._duty == 0


def test_auto_load_calibration_off_by_default(compiled):
    """With flag off (default), auto_load runs the normal journey to L and loads."""
    m = compiled
    assert m._CALIBRATE_X_OFFSET is False
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    _install_sim_wait(m, mx, my, clock)
    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True

    rom.auto_load()  # must not raise

    assert rom.has_load is True


def test_journey_yields_on_remote_press(compiled):
    m = compiled
    layout = m.ODV_GRID_DEFAULT
    rom, mx, my, clock = _build_rom(m, layout)
    _install_sim_wait(m, mx, my, clock)

    _park_on_tile(m, mx, my, rom.grid.unload_tile)
    rom.mh_is_homed = True
    # Simulate a stuck button — journey should yield on the first tick.
    rom._remote.buttons.pressed.return_value = (1,)

    reached = rom._drive_auto_journey(rom.grid.load_tile)

    assert reached is False
    assert rom.mh_auto_drive is False


def _install_recording_wait(m, motor_x, motor_y, clock):
    """Variant of _install_sim_wait that returns the list of wait amounts."""
    waits: list[int] = []

    def recording_wait(ms):
        waits.append(ms)
        motor_x.advance(ms)
        motor_y.advance(ms)
        clock.advance(ms)

    m.wait = recording_wait
    return waits


def test_manual_load_centers_y_before_loading(compiled):
    """Manual load (RIGHT_MINUS at load tile) must center cart on tile before _do_load_
    runs, mirroring auto-mode arrival. Without centering, Y stays at whatever the cart
    was at when the button was pressed and the load chute alignment is off."""
    from pybricks.parameters import Button
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    _install_sim_wait(m, mx, my, clock)

    cx, cy = rom.grid.tile_center_deg(rom.grid.load_tile)
    mx.reset_angle(cx)
    my.reset_angle(cy + 100)  # 100° off tile Y-center, still inside tile

    rom._remote.buttons.pressed.return_value = (Button.RIGHT_MINUS,)
    rom.handle_remote_press()

    assert my.angle() == cy  # centering happened before _do_load_
    assert rom.has_load is True


def test_manual_load_pauses_500ms_before_loading(compiled):
    """Manual load arrival must wait(500) between centering and _do_load_,
    same as auto. _do_load_ itself only uses wait(2000)."""
    from pybricks.parameters import Button
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    waits = _install_recording_wait(m, mx, my, clock)

    cx, cy = rom.grid.tile_center_deg(rom.grid.load_tile)
    mx.reset_angle(cx)
    my.reset_angle(cy)
    rom._remote.buttons.pressed.return_value = (Button.RIGHT_MINUS,)

    rom.handle_remote_press()

    assert 500 in waits  # the arrival pause


def test_manual_unload_pauses_500ms_before_home_and_unload(compiled):
    """Manual unload (RIGHT_PLUS at unload tile) must wait(500) between centering
    and home_and_unload, same as auto. home_and_unload uses wait(200) and wait(2000)
    but never wait(500), so 500 is a unique marker for the new arrival pause."""
    from pybricks.parameters import Button
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    waits = _install_recording_wait(m, mx, my, clock)

    cx, cy = rom.grid.tile_center_deg(rom.grid.unload_tile)
    mx.reset_angle(cx)
    my.reset_angle(cy)
    rom.has_load = True
    rom._remote.buttons.pressed.return_value = (Button.RIGHT_PLUS,)

    rom.handle_remote_press()

    assert 500 in waits
    assert rom.mh_is_homed is True
    assert rom.has_load is False


def test_manual_west_drive_from_u_reaches_l(compiled):
    """Starting parked at U and holding west on the remote, the cart must be
    able to drive across the grid all the way into L. Regression: an over-
    aggressive block_special check on the manual tick prevented the cart's
    AABB from ever entering the L/U tile from outside, so the load trigger
    never fired."""
    from pybricks.parameters import Button
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_DEFAULT)
    _install_sim_wait(m, mx, my, clock)

    cx, cy = rom.grid.tile_center_deg(rom.grid.unload_tile)
    mx.reset_angle(cx)
    my.reset_angle(cy)
    rom._remote.buttons.pressed.return_value = (Button.RIGHT_MINUS,)

    for _ in range(2000):  # 20s of 10ms ticks — way more than the journey needs
        rom.handle_remote_press()
        if rom.grid.deg_to_tile((mx.angle(), my.angle())) == rom.grid.load_tile:
            return
        m.wait(10)
    raise AssertionError(
        'cart stuck at ({}, {}) — never reached L'.format(mx.angle(), my.angle()))


def test_manual_east_drive_from_l_reaches_u(compiled):
    """Mirror: on a clear-path grid (EX2 has no one-way tiles between L and U),
    holding east at L with a load must let the cart drive all the way into U.
    EX2 = ['X###X', 'L###U', 'X###X']."""
    from pybricks.parameters import Button
    m = compiled
    rom, mx, my, clock = _build_rom(m, m.ODV_GRID_EX2)
    _install_sim_wait(m, mx, my, clock)

    cx, cy = rom.grid.tile_center_deg(rom.grid.load_tile)
    mx.reset_angle(cx)
    my.reset_angle(cy)
    rom.has_load = True
    rom._remote.buttons.pressed.return_value = (Button.RIGHT_PLUS,)

    for _ in range(2000):
        rom.handle_remote_press()
        if rom.grid.deg_to_tile((mx.angle(), my.angle())) == rom.grid.unload_tile:
            return
        m.wait(10)
    raise AssertionError(
        'cart stuck at ({}, {}) — never reached U'.format(mx.angle(), my.angle()))
