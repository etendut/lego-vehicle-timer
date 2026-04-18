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
