# ODV Movement Redesign — Implementation Plan

## Context

The ODV movement system is being rewritten around the architecture in
`.claude/analysis/odv_movement_redesign.md` Part B. Core shift:

- Movement model becomes **AABB-in-tile-grid** (video-game polygon
  style), validated by a single `Grid.propose_step` per tick.
- Runtime coordinate is **motor degrees**; fine-grid intermediate
  dropped.
- Control feel is **arcade hold-to-glide** with a `VirtualJoystick`
  shared by manual (remote) and auto (`AutoDriver`) inputs.
- Drive modes become an explicit enum (`MANUAL`, `HYBRID`, `AUTO`).
- Idle-timeout logic moves out of `CountdownTimer` into an ODV-local
  helper.

**Geometric invariants** (used by every task; wrong once, wrong
everywhere):

- `_DEG_PER_TILE = 800` (tile size in motor degrees;
  `_FINE_GRID_SIZE × _GEAR_RATIO_TO_GRID = 10 × 80`).
- `_CART_SIZE_DEG = 640` starting value (8 fine units × 80°/unit).
- **Axis convention (preserved from existing code):** motor `angle()`
  X grows east, Y grows south. Grid row 0 is north. A tile `(tx, ty)`
  occupies motor-degree rectangle `[tx·800, (tx+1)·800] × [ty·800,
  (ty+1)·800]`. Its **centre** is `(tx·800 + 400, ty·800 + 400)`.
- Cart centred at `(cx, cy)` has AABB
  `[cx − 320, cx + 320] × [cy − 320, cy + 320]` with the default
  `_CART_SIZE_DEG`.
- AABB overlap with a wall rect `[wl, wr] × [wt, wb]` is strict:
  `cart_right > wl AND cart_left < wr AND cart_bottom > wt AND
  cart_top < wb`. A cart **flush** with a wall edge (e.g.
  `cart_right == wl`) does **not** overlap — it's touching, allowed.

## Execution model

Each numbered task below is one commit per `CLAUDE.md` rule. Stop at
each task boundary and wait for the user to commit. After editing any
`modules/vehicle_*.py`, run:

```
python tools/compile_pybricks_files.py
python -m pytest tests/
```

The user will run on Opus for Task 1 (geometry-heavy) and on Sonnet
for the rest. Opus will review each Sonnet diff before commit.

Work happens in `modules/vehicle_odv.py` unless stated — the compile
tool expects all ODV code between `# MODULE_START` / `# MODULE_END`
markers. New support modules are possible but add splice-tool work;
avoid unless a task explicitly calls for one.

---

## Task 1 — `Grid` class with `propose_step` (geometry-heavy)

**Goal:** Introduce a `Grid` class that owns the `list[str]` layout,
resolves LOAD/UNLOAD tiles, materialises wall rectangles and
edge-barrier segments in degree-space, and exposes the single
validation primitive `propose_step`. Nothing else in the codebase
changes yet — existing runtime still uses the old validation paths.

### Files

- **Modify:** `modules/vehicle_odv.py` — add `Grid` class inside the
  `# MODULE_START` / `# MODULE_END` block, above `class RunODVMotors`.
  Add constants `_DEG_PER_TILE = const(800)` and
  `_CART_SIZE_DEG = const(640)` in the `# VARS_START` block.
- **Modify:** `tests/test_vehicle_odv.py` — append new parametrised
  tests for `Grid` (section "Task 1 tests" below). Do **not** remove
  existing tests in this task.

### `Grid` API

```python
class Grid:
    def __init__(self, layout: list[str]):
        # Parse layout. Store:
        #   self.layout          : list[str]
        #   self.n_cols, self.n_rows
        #   self.load_tile       : tuple[int,int]
        #   self.unload_tile     : tuple[int,int]
        #   self._wall_rects     : tuple[tuple[int,int,int,int], ...]   # (l, t, r, b) per X tile
        #   self._west_barriers  : tuple[tuple[int,int,int], ...]       # (x, y_top, y_bottom) per '<' tile
        #   self._east_barriers  : tuple[tuple[int,int,int], ...]       # (x, y_top, y_bottom) per '>' tile
        # Use tuples (not lists) for the immutable collections.

    def tile_type(self, tx: int, ty: int) -> str: ...

    def tile_center_deg(self, tile: tuple[int, int]) -> tuple[int, int]:
        # (tx*_DEG_PER_TILE + _DEG_PER_TILE//2, ty*_DEG_PER_TILE + _DEG_PER_TILE//2)

    def deg_to_tile(self, deg_pos: tuple[int, int]) -> tuple[int, int]:
        # (deg_x // _DEG_PER_TILE, deg_y // _DEG_PER_TILE). Clamped to grid bounds.

    def propose_step(
        self,
        deg_pos: tuple[int, int],
        d_deg_x: int,
        d_deg_y: int,
    ) -> tuple[int, int]:
        """
        Return (valid_dx, valid_dy): the largest per-axis step no greater in
        magnitude than the requested one that keeps the cart AABB legal.
        Per-axis independent: X is tested alone, Y is tested alone. If an
        axis is blocked, that axis returns 0; the other axis is unaffected.
        """
```

### `propose_step` algorithm (pseudocode)

```
def propose_step(self, deg_pos, d_deg_x, d_deg_y):
    cx, cy = deg_pos
    valid_dx = 0
    valid_dy = 0

    if d_deg_x != 0:
        if self._axis_step_legal(cx, cy, d_deg_x, axis=_X):
            valid_dx = d_deg_x

    if d_deg_y != 0:
        if self._axis_step_legal(cx, cy, d_deg_y, axis=_Y):
            valid_dy = d_deg_y

    return valid_dx, valid_dy
```

Where `_axis_step_legal(cx, cy, d, axis)` returns `True` iff, after
moving the cart by `d` on the given axis (other axis unchanged):
  1. The cart AABB at the new position does not overlap any wall rect
     and does not exit the grid bounds.
  2. The step does not cross a forbidden edge barrier.

### AABB legality (shared helper)

```
def _aabb_hits_wall(self, cx, cy):
    half = _CART_SIZE_DEG // 2
    L, R, T, B = cx - half, cx + half, cy - half, cy + half
    # Grid bounds
    if L < 0 or T < 0 or R > self.n_cols * _DEG_PER_TILE or B > self.n_rows * _DEG_PER_TILE:
        return True
    # X-tile walls
    for wl, wt, wr, wb in self._wall_rects:
        if R > wl and L < wr and B > wt and T < wb:
            return True
    return False
```

### Edge-barrier crossing logic

Barriers are vertical line segments (X-axis boundaries only — `<`
and `>` never create horizontal barriers, so Y-axis motion is never
blocked by a barrier).

Given `deg_pos = (cx, cy)` and a step `d_deg_x != 0`:

- **East-bound (`d_deg_x > 0`):** the cart's east face moves from
  `cx + half` to `cx + half + d_deg_x`. For every `<` barrier
  `(bx, y_top, y_bottom)`:
  - Y overlap: `cy - half < y_bottom AND cy + half > y_top`.
  - Crossing: `(cx + half) <= bx AND (cx + half + d_deg_x) > bx`.
  - Both true → blocked.
- **West-bound (`d_deg_x < 0`):** cart's west face moves from
  `cx - half` to `cx - half + d_deg_x`. For every `>` barrier
  `(bx, y_top, y_bottom)`:
  - Y overlap: `cy - half < y_bottom AND cy + half > y_top`.
  - Crossing: `(cx - half) >= bx AND (cx - half + d_deg_x) < bx`.
  - Both true → blocked.

Barriers in the arrow's own direction are never tested — a `<` tile
allows westbound crossing, a `>` tile allows eastbound crossing.

Y-axis motion never checks barriers.

### Barrier segment extents

For a `<` tile at `(tx, ty)`:
- `bx = tx * _DEG_PER_TILE`
- `y_top = ty * _DEG_PER_TILE`
- `y_bottom = (ty + 1) * _DEG_PER_TILE`

For a `>` tile at `(tx, ty)`:
- `bx = (tx + 1) * _DEG_PER_TILE`
- `y_top = ty * _DEG_PER_TILE`
- `y_bottom = (ty + 1) * _DEG_PER_TILE`

### Task 1 tests

All tests use `_DEG_PER_TILE = 800` and `_CART_SIZE_DEG = 640`.
Tile centre shorthand: `cen(tx, ty) = (tx*800 + 400, ty*800 + 400)`.

Place each test case in `tests/test_vehicle_odv.py` as a parametrised
test function. Import `Grid` from `modules.vehicle_odv`.

| # | Grid | Start `deg_pos` | `(d_deg_x, d_deg_y)` | Expect `propose_step` | Reason |
|---|------|-----------------|----------------------|------------------------|--------|
| 1 | `["L#U"]` | `cen(0,0) = (400, 400)` | `(+100, 0)` | `(100, 0)` | Clear east, full step |
| 2 | `["L#U"]` | `(880, 400)` (cart east face = 1200, flush w/ boundary) | `(+10, 0)` | `(0, 0)` | Would exit east grid edge |
| 3 | `["LXU"]` | `cen(0,0)` | `(+100, 0)` | `(0, 0)` | X tile blocks east step |
| 4 | `["LXU"]` | `cen(0,0)` | `(0, +100)` | `(0, 0)` | Would exit south grid edge |
| 5 | `["L#<#U"]` | `cen(1,0) = (1200, 400)` | `(+500, 0)` | `(0, 0)` | `<` at (2,0): cart east face 1520 → 2020 crosses bx=1600 |
| 6 | `["L#<#U"]` | `cen(2,0) = (2000, 400)` | `(-500, 0)` | `(-500, 0)` | From inside `<`, westbound is allowed (arrow direction) |
| 7 | `["L#<#U"]` | `cen(3,0) = (2800, 400)` | `(-500, 0)` | `(-500, 0)` | West of the barrier exit — no barrier crossed |
| 8 | `["L#>#U"]` | `cen(3,0) = (2800, 400)` | `(-500, 0)` | `(0, 0)` | `>` at (2,0): cart west face 2480 → 1980 crosses bx=2400 |
| 9 | `["L#>#U"]` | `cen(2,0) = (2000, 400)` | `(+500, 0)` | `(+500, 0)` | From inside `>`, eastbound is allowed |
| 10 | DEFAULT `["L#<#U","X#<#X","X###X"]` | `cen(1,2) = (1200, 2000)` | `(+800, -800)` | `(800, -800)` | Corner-cut (1,2)→(2,1) via diagonal step; no walls or barriers cross (tile (2,1) is `<` but we aren't crossing bx=1600 at east face — check carefully) |
| 11 | EX3 `["X#>#X","L#X#U","X#<#X"]` | `cen(1,0) = (1200, 400)` | `(+800, +800)` | `(800, 0)` or `(0, 800)` | Diagonal into (2,1)=X blocked per axis; only one axis can advance. (Sub-case: check actual expected output from per-axis independent testing.) |
| 12 | `["L#U"]` | `(cen(0,0))` | `(0, 0)` | `(0, 0)` | No-op |
| 13 | `["L#U"]` | `(400, 400)` | `(-400, 0)` | `(0, 0)` | Cart west face 80 → −320; exits grid |
| 14 | `["L#<#U","X#<#X","X###X"]` — load/unload detection | n/a | n/a | `Grid(DEFAULT).load_tile == (0, 0)` and `.unload_tile == (4, 0)` | Parse correctness |

**Case 10 verification** (corner cut on DEFAULT going NE from (1,2) to (2,1)):

- Start: `(1200, 2000)`. AABB: `[880, 1520] × [1680, 2320]`.
- Target: `(2000, 1200)`. Step `(+800, −800)`.
- X-only test: new center `(2000, 2000)`. AABB: `[1680, 2320] × [1680, 2320]`.
  - Wall rects: (0,1)=X → `[0,800]×[800,1600]` — 1680<800? no, overlap X: 2320>0✓, 1680<800? no. No overlap.
  - (4,1)=X → `[3200,4000]×[800,1600]` — AABB X: 2320>3200? no. No overlap.
  - (0,2)=X → `[0,800]×[1600,2400]` — AABB X: 2320>0✓, 1680<800? no. No overlap.
  - (4,2)=X → `[3200,4000]×[1600,2400]` — AABB X: 2320>3200? no. No overlap.
  - Barrier `<` at (2,0) → bx=1600, y∈[0,800]. East face 1520→2320 crosses bx=1600, but Y overlap: `2320−half=1680 < 800`? `1680 < 800` is false. No Y overlap → barrier not relevant. ✓
  - Barrier `<` at (2,1) → bx=1600, y∈[800,1600]. East face crosses bx=1600, Y overlap: `1680 < 1600`? false. No Y overlap. ✓
  - X-only legal → `valid_dx = 800`.
- Y-only test: new center `(1200, 1200)`. AABB: `[880, 1520] × [880, 1520]`.
  - (0,1)=X: AABB X: 1520>0✓, 880<800? no. No overlap.
  - (4,1)=X: 1520>3200? no. No overlap.
  - Barriers irrelevant for Y motion.
  - Y-only legal → `valid_dy = −800`.
- Expected: `(800, −800)`.

**Case 11 verification** (EX3, cart at (1,0), diagonal NE into (2,1)=X):

- Start: `(1200, 400)`. AABB: `[880, 1520] × [80, 720]`.
- X-only `dx=+800` → `(2000, 400)`. AABB: `[1680, 2320] × [80, 720]`.
  - (2,1)=X → `[1600,2400]×[800,1600]` — AABB X: 2320>1600✓, 1680<2400✓. AABB Y: 720>800? no. No Y overlap. Safe.
  - (2,0)=#, (0,1)=L, (4,0)=X → (4,0) is X? check `["X#>#X", ...]` row 0: X#>#X → (0,0)=X, (1,0)=#, (2,0)=>, (3,0)=#, (4,0)=X. So (4,0)=X, rect `[3200,4000]×[0,800]`. AABB X: 2320>3200? no. Safe.
  - `>` at (2,0): bx=2400 (east edge). Eastbound cart checks `<`, not `>`. No block.
  - `<` at (2,2): bx=1600, y∈[1600,2400]. East face 1520→2320 crosses bx=1600 going east. Y overlap: cart Y [80,720] vs barrier y [1600,2400]. 720>1600? no. No Y overlap. Safe.
  - X-only legal → `valid_dx = 800`.
- Y-only `dy=+800` → `(1200, 1200)`. AABB: `[880, 1520] × [880, 1520]`.
  - (2,1)=X → `[1600,2400]×[800,1600]` — AABB X: 1520>1600? no. Safe.
  - (0,1)=L → not a wall.
  - Y-only legal → `valid_dy = 800`.
- Expected: `(800, 800)`. **Update the table entry 11 accordingly** — per-axis independent testing passes both, even though the combined move would put the cart AABB `[1680, 2320] × [880, 1520]` overlapping (2,1)=X (`1680<2400, 2320>1600, 1520>800, 880<1600` → overlap).

This is the gap I flagged in my analysis: per-axis test passes but combined doesn't. For v1, **accept this**. Corner-cuts on single-cell X tiles are rare in practice and the AutoDriver's waypoint shaping avoids them. The cart body will briefly be inside the X tile during the cut; physically this means a nudge past an obstacle, which the existing hub runs can tolerate since the X tile is invisible structure.

If this becomes a problem on the rig, revisit by adding a **combined** AABB check in `propose_step`:

```python
if valid_dx != 0 and valid_dy != 0:
    if self._aabb_hits_wall(cx + valid_dx, cy + valid_dy):
        # prefer the axis with larger magnitude; zero the other
        if abs(valid_dx) >= abs(valid_dy):
            valid_dy = 0
        else:
            valid_dx = 0
```

**For v1: do not add the combined check.** Document the limitation in a
comment on `propose_step`.

### Verification for Task 1

- `python tools/compile_pybricks_files.py` succeeds and produces a
  compiled output with the new class + consts.
- `python -m pytest tests/test_vehicle_odv.py -k "grid or propose_step"` passes.
- Old tests still pass (we haven't removed old paths yet).

### Commit message

```
Add Grid class with AABB propose_step

Introduces _DEG_PER_TILE and _CART_SIZE_DEG constants and a Grid
class that owns layout parsing, wall rectangles, edge-barrier
segments, and a single propose_step primitive for future movement
validation. Old validation paths remain in place.
```

---

## Task 2 — `VirtualJoystick` + `AxisController`

**Goal:** Add the runtime motion layer that converts a joystick
struct into motor commands, gated by `Grid.propose_step`, with
speed compensation and a fixed-ms ramped stop.

### Files

- **Modify:** `modules/vehicle_odv.py` — add `VirtualJoystick` and
  `AxisController` classes in the same block as `Grid`. Do not wire
  them into `RunODVMotors` yet (Task 8 does the wiring).
- **Modify:** `tests/test_vehicle_odv.py` — add tests for
  `AxisController` using `MagicMock` motors and a real `Grid`.

### `VirtualJoystick`

Tiny value type. Either a class with two int fields or a plain
tuple; pick the class form for readability:

```python
class VirtualJoystick:
    __slots__ = ('ax', 'ay')
    def __init__(self, ax: int = 0, ay: int = 0):
        self.ax = ax   # -1, 0, or +1
        self.ay = ay
```

### `AxisController`

```python
class AxisController:
    def __init__(self, motor_x, motor_y, grid: Grid, base_duty: int):
        # base_duty: integer duty % for single-axis motion (e.g. 45)
        # keep: self._prev_duty_x, self._prev_duty_y (for ramp)
        # keep: self._stop_ramp_start_x, self._stop_ramp_start_y (timestamps)
        ...

    def deg_pos(self) -> tuple[int, int]:
        return self.motor_x.angle(), self.motor_y.angle()

    def tick(self, vj: VirtualJoystick) -> None:
        """One control tick. Proposes a small step, asks the Grid to
        clip it, issues motor.dc commands per axis. Axis going from
        active→idle enters a fixed-ms duty ramp to zero."""
```

**Speed compensation:**

```python
_BOTH_AXES_DUTY_NUM = const(71)   # 0.71 as integer math
_BOTH_AXES_DUTY_DEN = const(100)
```

When `vj.ax != 0 AND vj.ay != 0`, each axis uses
`duty = base_duty * 71 // 100`. Otherwise `duty = base_duty`.

**Proposed step magnitude per tick:**

The controller doesn't step the cart directly — `motor.dc(duty)` runs
the motor freely; the motor moves "some amount" per tick. For the
`propose_step` call, we need a forward-looking step size in degrees
representing "how far will the cart likely move before the next tick
can correct it." Starting value:

```python
_LOOKAHEAD_DEG = const(40)   # half a fine unit; one tick's worth at ~45% duty and 20ms tick
```

So each tick:

```python
def tick(self, vj):
    cx, cy = self.deg_pos()
    both = vj.ax != 0 and vj.ay != 0
    duty = base_duty * 71 // 100 if both else base_duty

    requested_dx = vj.ax * _LOOKAHEAD_DEG
    requested_dy = vj.ay * _LOOKAHEAD_DEG
    valid_dx, valid_dy = self.grid.propose_step(
        (cx, cy), requested_dx, requested_dy
    )

    # X axis
    if valid_dx > 0:
        self.motor_x.dc(+duty)
        self._prev_duty_x = +duty
        self._stop_ramp_start_x = None
    elif valid_dx < 0:
        self.motor_x.dc(-duty)
        self._prev_duty_x = -duty
        self._stop_ramp_start_x = None
    else:
        self._ramp_stop(self.motor_x, axis='x')

    # Y axis — symmetric
    ...
```

**Ramped stop** (fixed-ms, starting value 200ms):

```python
_STOP_RAMP_MS = const(200)

def _ramp_stop(self, motor, axis):
    if self._prev_duty[axis] == 0:
        return  # already stopped
    if self._stop_ramp_start[axis] is None:
        self._stop_ramp_start[axis] = ticks_ms()
    elapsed = ticks_ms() - self._stop_ramp_start[axis]
    if elapsed >= _STOP_RAMP_MS:
        motor.dc(0)
        self._prev_duty[axis] = 0
        self._stop_ramp_start[axis] = None
    else:
        factor = (_STOP_RAMP_MS - elapsed) * 100 // _STOP_RAMP_MS
        motor.dc(self._prev_duty[axis] * factor // 100)
```

Use `pybricks.tools.StopWatch` or `ticks_ms` — match whatever the
existing code uses for timing (grep for `StopWatch` in the module).

### Task 2 tests

With mocked motors and a real `Grid`:

1. Joystick all-zero → both motors get `dc(0)` (or enter stop ramp).
2. Joystick `(+1, 0)` in clear space → `motor_x.dc(+45)`, `motor_y.dc(0)`.
3. Joystick `(+1, +1)` in clear space → both motors get `dc(45 * 71 / 100) = dc(31)`.
4. Joystick `(+1, 0)` against east grid boundary →
   `motor_x.dc(...)` enters ramp; within 200ms+ subsequent tick,
   `motor_x.dc(0)`.
5. After active→idle transition, `motor_x.dc` call values across
   successive ticks decrease monotonically toward 0 over 200ms.

### Verification

- Compile + tests green.
- No regression in old tests.

### Commit message

```
Add VirtualJoystick and AxisController classes

AxisController consumes a VirtualJoystick, clips against Grid
.propose_step, applies 71% speed compensation on diagonals, and
ramps to stop over 200ms when an axis goes active→idle. Not yet
wired into RunODVMotors.
```

---

## Task 3 — `HomingRoutine` class

**Goal:** Extract the existing `home_and_unload` logic into a
standalone `HomingRoutine` class. Keep `run_until_stalled` for the
blocking homing sequence. Park on `U` tile centre in degree-space.

### Files

- **Modify:** `modules/vehicle_odv.py` — add `HomingRoutine` alongside
  the other new classes. Do not delete `home_and_unload` yet (Task 8
  does the cutover).

### API

```python
class HomingRoutine:
    def __init__(self, motor_x, motor_y, grid: Grid,
                 homing_speed: int, homing_duty: int):
        ...

    def run(self) -> None:
        """Stall Y north, stall X east, reset both encoders so that
        after completion motor.angle() is at grid.tile_center_deg
        (grid.unload_tile)."""
```

### Algorithm

Preserve the existing sequence (see current `home_and_unload`):

1. `motor_y.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)`.
2. After stall, `motor_y.angle()` is at the north wall. Reset:
   `motor_y.reset_angle(grid.unload_tile[1] * _DEG_PER_TILE + _DEG_PER_TILE // 2)`
   — so that after one half-tile of southward motion we'd be at the
   unload tile centre. Actually: the north wall is immediately north
   of U, so the cart's north face is flush with Y=`ty*_DEG_PER_TILE`
   where ty = unload row. With cart centred on U, cart_north =
   ty*_DEG_PER_TILE + (_DEG_PER_TILE − _CART_SIZE_DEG)/2 = ty*800 + 80.
   At stall, cart_north ≈ ty*800 (cart pushed flush against wall), so
   cart_center_y = ty*800 + _CART_SIZE_DEG/2 = ty*800 + 320.
   **Verify on-rig** — the existing `reset_angle(unload_tile_angle[1])`
   in `home_and_unload` resets to `unload_tile_y * 800` which is the
   tile origin, not the centre. The new version must preserve whatever
   the old one did so the cart sits in the same physical spot after
   homing. Read the existing homing carefully and match it exactly; do
   not rederive the magic numbers.
3. Stall east: `motor_x.run_until_stalled(+_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)`.
4. `motor_x.reset_angle(<match existing>)`.
5. `motor_x.run_target(speed, grid.tile_center_deg(grid.unload_tile)[0])` to
   centre on the unload tile.

**The critical rule:** the `reset_angle` targets must match the
existing code's behaviour one-for-one. Do not change the physical
parking position in this task — only lift the logic out of
`home_and_unload`.

### Task 3 tests

Only `MagicMock`-based: verify call sequence (stall Y, reset, stall X,
reset, run_target). Physical correctness is verified on the rig by
the dev.

### Commit message

```
Extract HomingRoutine from RunODVMotors.home_and_unload

Pure refactor — no change to physical parking position. Old
home_and_unload still intact; the new class will take over in Task 8.
```

---

## Task 4 — `Planner` with 4-direction BFS

**Goal:** Replace the 8-direction `_bfs_path_to_grid_tile` with a
4-direction BFS that returns a waypoint list (turn-points only).
Keep the old BFS functions in place until Task 8.

### Files

- **Modify:** `modules/vehicle_odv.py` — add `Planner` class.
- **Modify:** `tests/test_vehicle_odv.py` — add tests that plan L→U
  and U→L on DEFAULT, EX1, EX2, EX3.

### API

```python
class Planner:
    def __init__(self, grid: Grid):
        self.grid = grid

    def plan(self, start: tuple[int, int], goal: tuple[int, int]) -> tuple[tuple[int, int], ...]:
        """Return an immutable tuple of coarse-tile waypoints from start to goal.
        First element == start, last == goal. Intermediate elements are
        only turn-points (cells where the direction changes). Returns
        empty tuple if unreachable."""
```

### Algorithm

1. Standard BFS with parent-pointer dict `{tile: (parent_tile, step_dir)}`
   where `step_dir` is one of the four `(dx, dy)` tuples
   `(1,0), (-1,0), (0,1), (0,-1)` — no enum.
2. Neighbour expansion: for each of the four cardinal directions,
   compute `next_tile = (tile[0]+dx, tile[1]+dy)`. Reject if:
   - Out of bounds.
   - `tile_type(next_tile) == 'X'`.
   - Direction violates a one-way rule:
     - `dx > 0` (east) and `next_tile` is `<`.
     - `dx < 0` (west) and `tile` is `>` (cart leaving a `>` westbound).
     - Mirror for the other direction.
     - Match the semantics from N1d precisely: a `<` tile's west edge
       blocks eastbound entry into it; a `>` tile's east edge blocks
       westbound entry into it. So:
       - Eastbound blocked if `next_tile` is `<`.
       - Westbound blocked if `tile` is `>` OR `next_tile` is `<`? No —
         westbound through `<` is allowed (arrow direction). Westbound
         blocked if `next_tile` is `>`… wait, `>` east-edge blocks
         westbound entry _into `>`_ from the east. So eastbound cart
         leaving `>` eastward is fine; westbound cart entering `>` from
         the east is blocked.
     - Consolidated rule (tile-level for BFS; `propose_step` owns the
       runtime version): reject the move from `tile` to `next_tile`
       iff crossing the shared edge is forbidden. The shared edge is
       walled if `tile == '>'` (eastbound wall between `tile` and
       east neighbour) or `next_tile == '<'` (westbound wall between
       tile and its east neighbour). A walled edge admits crossing
       only in the arrow direction.
     - Translating to code:
       - Eastbound (`dx > 0`): blocked if `next_tile == '<'`.
       - Westbound (`dx < 0`): blocked if `tile == '>'`.
       - N/S (`dy != 0`): barriers only live on east-west edges, so
         no one-way rule applies to N/S motion.
3. After BFS finds goal, reconstruct path tile-by-tile via the
   parent-pointer map. Reverse to get start→goal order.
4. Compress to waypoints: keep only tiles where the incoming
   direction differs from the outgoing direction. Always keep first
   and last tile.

### Task 4 tests

| Grid | Start | Goal | Expected waypoints |
|------|-------|------|--------------------|
| DEFAULT `["L#<#U","X#<#X","X###X"]` | (0,0) | (4,0) | `((0,0),(1,0),(1,2),(3,2),(3,0),(4,0))` — go south to exit the `<` barrier region, east along row 2, then north to U. **Verify by running the BFS mentally; if the actual shortest path differs, update expected.** |
| DEFAULT | (4,0) | (0,0) | U→L path — expect `((4,0),(3,0),(3,2),(1,2),(1,0),(0,0))`, mirror of above, noting that westbound through `<` at (2,0)/(2,1) is allowed by the arrow. |
| EX2 `["X###X","L###U","X###X"]` | (0,1) | (4,1) | `((0,1),(4,1))` — straight shot |
| EX3 `["X#>#X","L#X#U","X#<#X"]` | (0,1) | (4,1) | Clockwise loop through row 0 due to one-ways + (2,1)=X. Expect something like `((0,1),(0,0),(3,0)? or (4,0)?),...` — compute by hand and hard-code. |

For each test, run BFS, print the result during development, and
bake the known-good output into the expected constants. Do not
guess.

### Commit message

```
Add Planner with 4-dir BFS producing waypoint paths

Turn-point-only waypoint list; single-edge barrier rules match N1d.
Old _bfs_path_to_grid_tile still present; cutover in Task 8.
```

---

## Task 5 — `AutoDriver`

**Goal:** Walk a waypoint list by emitting `VirtualJoystick` values
aimed at each waypoint's tile-centre, with a small neighbourhood for
the aim-switch. Trigger endpoint actions (load, unload, home) when
the terminal waypoint is reached. Yield control when a real remote
button is pressed.

### Files

- **Modify:** `modules/vehicle_odv.py` — add `AutoDriver` class.

### API

```python
class AutoDriver:
    def __init__(self, grid: Grid, planner: Planner, axis_controller: AxisController):
        ...

    def start_journey(self, from_tile, to_tile) -> None:
        """Plan a path and reset waypoint index to 0."""

    def tick(self, remote) -> str | None:
        """Run one tick. Returns None while journey continues, or a
        string tag on journey end ('reached_load' | 'reached_unload').
        If any real remote button is pressed, returns 'yielded' and
        does not emit a joystick value (AxisController receives a
        zero joystick via the caller)."""
```

### Algorithm

```
_AIM_SWITCH_DEG = const(160)   # ~2 fine units of slack

def tick(self, remote):
    if any(remote.buttons.pressed()):
        return 'yielded'

    cx, cy = self.axis_controller.deg_pos()
    target = self.grid.tile_center_deg(self.waypoints[self.i + 1])

    if _within(cx, target[0], _AIM_SWITCH_DEG) and _within(cy, target[1], _AIM_SWITCH_DEG):
        self.i += 1
        if self.i == len(self.waypoints) - 1:
            if self.waypoints[-1] == self.grid.load_tile:
                return 'reached_load'
            if self.waypoints[-1] == self.grid.unload_tile:
                return 'reached_unload'
            return 'reached_end'

    vj = VirtualJoystick(
        sign(target[0] - cx),
        sign(target[1] - cy),
    )
    self.axis_controller.tick(vj)
    return None
```

`_within(a, b, r)` = `abs(a - b) <= r`. `sign(n)` returns −1 / 0 / +1.

### Task 5 tests

Mocked motors + real Grid + real Planner. Step the AutoDriver with
simulated encoder advances, assert it returns `reached_unload` after
the cart "arrives" at U's tile centre. Separate test: simulate a
remote press mid-journey and assert return value is `'yielded'`.

### Commit message

```
Add AutoDriver walking Planner waypoints via AxisController

Aim-switch neighbourhood 160°. Real remote presses yield control.
Not yet wired into RunODVMotors.
```

---

## Task 6 — Drive-mode enum + `IdleTimeout`

**Goal:** Introduce explicit drive modes and a small ODV-local
idle-timeout helper. Do not yet replace the existing two-boolean
branch logic — Task 8 does that.

### Files

- **Modify:** `modules/vehicle_odv.py` — in the `# VARS_START` block,
  add:
  ```python
  MANUAL = const(0)
  HYBRID = const(1)
  AUTO   = const(2)

  DRIVE_MODE = HYBRID   # default — mirrors today's hybrid config
  IDLE_TIMEOUT_SECS = const(30)  # used only when DRIVE_MODE == HYBRID
  ```
- **Modify:** `modules/vehicle_odv.py` — add `IdleTimeout` class:
  ```python
  class IdleTimeout:
      def __init__(self, seconds: int):
          ...
      def reset(self) -> None: ...
      def fired(self) -> bool: ...
  ```
  Implement with a `StopWatch` or `ticks_ms` delta.

Task 6 does **not** yet remove `ODV_AUTO_DRIVE_TIMEOUT_SECS` or
`REMOTE_DISABLED` — those are lifted in Task 7/8 during the cutover.

### Verification

- New consts compile.
- `IdleTimeout` unit-tested with a patched clock: `fired() == False`
  immediately after `reset()`, `True` after the configured interval
  elapses.

### Commit message

```
Add DRIVE_MODE enum (MANUAL/HYBRID/AUTO) and IdleTimeout helper

No behaviour change yet — the cutover from the old two-boolean
encoding lands in a later task.
```

---

## Task 7 — Strip `CountdownTimer`; fix `FINAL_20_SECS` bug

**Goal:** Remove the ODV-specific methods from `CountdownTimer` in
`lego_vehicle_timer_base.py` and fix the `has_time_remaining` bug
where the `< 20s` branch is overwritten by the `< 60s` branch.

### Files

- **Modify:** `modules/lego_vehicle_timer_base.py`:
  - Remove `reset_time_since_last_remote_press` and
    `remote_button_press_timed_out` methods.
  - Remove `ODV_AUTO_DRIVE_TIMEOUT_SECS` const.
  - In `has_time_remaining`, change the two sequential `if`
    guarded blocks to `if / elif` so `< 20s` wins when true.
- **Modify:** the base `main()` loop to stop calling the removed
  methods — the old calls have to go somewhere. In this task, fall
  back to just not calling them (ODV-side timeout handling is
  absent until Task 8). If the tests need a shim, leave a one-line
  method that's a no-op. Document any temporary shim with a TODO
  that references Task 8.
- **Modify:** `tests/` — remove or rewrite any test that exercised
  the removed methods.

### Verification

- Base tests green.
- ODV tests that don't depend on the removed methods still green.
  (The hybrid timeout flow is broken between Task 7 and Task 8; the
  dev should expect this gap.)

### Commit message

```
Strip CountdownTimer of ODV-specific timeout logic; fix FINAL_20_SECS

reset_time_since_last_remote_press and remote_button_press_timed_out
are ODV concerns — they move to IdleTimeout in Task 8. Also fixes a
bug where has_time_remaining's < 20s branch was overwritten by the
< 60s branch, making FINAL_20_SECS unreachable.
```

---

## Task 8 — Wire `RunODVMotors` to the new stack

**Goal:** The big cutover. `RunODVMotors.__init__` constructs `Grid`,
`Planner`, `AxisController`, `AutoDriver`, `HomingRoutine`, and (if
`DRIVE_MODE == HYBRID`) `IdleTimeout`. The main per-tick method
routes input from the remote or the AutoDriver into `AxisController`.
Old methods (`_can_move_in_direction_`, `_move_in_direction_`,
`_navigate_to_grid_tile`, `_navigate_grid_tile_path`, `auto_load`,
`auto_unload`, `_bfs_path_to_grid_tile`, etc.) are **not yet
deleted** — they stay in place but are no longer called. Task 9
deletes them.

### Files

- **Modify:** `modules/vehicle_odv.py`:
  - `__init__`: instantiate the new stack.
  - `home_and_unload`: delegate to `HomingRoutine.run()`.
  - `handle_remote_press`: map remote buttons to `VirtualJoystick`,
    hand to `axis_controller.tick(vj)`. Reset `IdleTimeout` on any
    button press.
  - Add a new `tick_auto()` method or equivalent: when in `AUTO` or
    after `IdleTimeout.fired()` in `HYBRID`, call
    `AutoDriver.tick(remote)`. Handle `reached_load` /
    `reached_unload` return codes.
- **Modify:** `modules/lego_vehicle_timer_base.py`:
  - Main loop calls whichever tick method ODV exposes. The base
    class's `handle_remote_press` hook stays, but ODV's
    implementation now does the routing.

### Verification

- Full test suite green (unit tests with mocks).
- Compile succeeds.
- On the rig: manual drive feels fluid; hybrid mode switches to
  auto after `IDLE_TIMEOUT_SECS` of no button presses; real button
  press during auto yields back to manual.

### Commit message

```
Wire RunODVMotors to Grid/AxisController/AutoDriver stack

Manual and auto input both feed a VirtualJoystick into a single
AxisController; DRIVE_MODE chooses the auto-engage policy. Old
methods remain in place but are no longer called; Task 9 removes
them.
```

---

## Task 9 — Remove old code

**Goal:** Delete everything the new stack replaced. This is a pure
deletion commit.

### Files

- **Modify:** `modules/vehicle_odv.py`:
  - Delete: `dir_to_str`, `position_from_direction`,
    `can_move_in_direction_by_type`, `_can_traverse_coarse`.
  - Delete: `ODVBox` class.
  - Delete: constants `NORTH`, `EAST`, etc., `_ALL_DIRECTIONS`.
  - Delete: `_FINE_GRID_SIZE`, `_GEAR_RATIO_TO_GRID` (only if nothing
    else still imports them — double-check compile tool output and
    the base class).
  - Delete old methods: `_can_move_in_direction_`,
    `_move_in_direction_`, `_navigate_to_grid_tile`,
    `_navigate_grid_tile_path`, `_bfs_path_to_grid_tile`, the old
    `home_and_unload` body (replaced by delegation in Task 8),
    `auto_load`, `auto_unload`, `_get_fine_grid_position_`,
    `_get_grid_tile_position_from_fine_xy_`,
    `_get_grid_tile_type_from_fine_xy_`, `_tile_to_angle`.
  - Delete: `last_fine_grid_position` attribute and its freshness
    check in `handle_remote_press`.
- **Modify:** `tests/test_vehicle_odv.py`:
  - Delete tests that exercised the removed symbols.
  - Any BFS-result test that referred to 8-direction output should
    be deleted (Planner has its own tests from Task 4).

### Verification

- Full test suite green.
- Compile succeeds.
- Grep for each deleted symbol across the repo to confirm no stale
  references remain.

### Commit message

```
Remove legacy ODV movement code

Deletes ODVBox, direction enum and helpers, both legacy validation
functions, old BFS, tile-hopping run_target navigation, and the
fine-grid conversion helpers. All functionality is covered by the
Grid/AxisController/Planner/AutoDriver stack.
```

---

## Task 10 — Compile-tool guards + compiled-file test/lint

**Goal:** Teach `tools/compile_pybricks_files.py` to validate the
splice markers and make the compiled output pass tests and lint.

### Files

- **Modify:** `tools/compile_pybricks_files.py`:
  - Before splicing, scan each source file and assert that every
    expected marker is present exactly once and that pairs are
    properly nested in order (`START` precedes matching `END`; no
    overlap with other markers).
  - On any violation, raise with a clear message naming the file
    and marker.
- **Add:** a pytest file `tests/test_compiled_files.py` that imports
  each `docs/pybricks/lego_vehicle_timer_*.py` and runs a light
  smoke test (e.g. confirms `RunODVMotors` exists and has expected
  public methods).
- **Add or modify:** a lint step in the compile script that runs
  `ruff check docs/pybricks/` (or whichever linter the project
  uses — check `pyproject.toml` / `.pre-commit-config.yaml`) and
  fails the compile if lint fails.

### Verification

- Artificially corrupt a marker in a source file → compile fails
  loudly.
- Compiled files import cleanly and pass the smoke tests.
- Lint is green on all four compiled files.

### Commit message

```
Validate splice markers; run tests and lint against compiled output

Compile tool now fails on missing, duplicated, or out-of-order
markers. Compiled lego_vehicle_timer_*.py files must pass import
smoke tests and ruff lint, catching splice bugs that only manifest
post-concatenation.
```

---

## Rollback plan

Each task commits independently. If a task fails on the rig and
can't be quickly fixed, `git revert` that commit only. Earlier
tasks remain landed. Task 8 is the riskiest — the cutover changes
runtime behaviour end to end — so we can revert 8 without losing
the work from 1–7.

## End-to-end verification (after Task 10)

1. Fresh checkout, `python tools/compile_pybricks_files.py`,
   `python -m pytest tests/` — all green.
2. Deploy compiled `docs/pybricks/lego_vehicle_timer_odv.py` to the
   hub.
3. Power cycle; confirm homing completes.
4. Manual mode: each cardinal button holds the cart at constant
   speed; diagonals are two-button combos at ~71% per-axis speed;
   release ramps to stop over ~200ms.
5. One-way tiles: confirm cart refuses to cross `<`/`>` edges
   against arrow direction.
6. Hybrid mode: leave the remote idle for 30s; auto-drive engages
   and completes a full L→U→L cycle; a button press mid-journey
   yields immediately (no stale auto re-engage).
7. Full auto: set `DRIVE_MODE = AUTO`, confirm continuous cycling.
8. Calibration: run a quick "stall E, stall W, subtract" remote
   script; if the measured cart footprint differs from 640°,
   update `_CART_SIZE_DEG`.

## Critical files

- `modules/vehicle_odv.py` — all tasks touch this.
- `modules/lego_vehicle_timer_base.py` — Tasks 7, 8, 10.
- `tools/compile_pybricks_files.py` — Task 10.
- `tests/test_vehicle_odv.py` — Tasks 1, 2, 4, 5, 9.
- `tests/test_compiled_files.py` — Task 10 (new file).
- `docs/pybricks/lego_vehicle_timer_*.py` — regenerated by compile
  tool at every checkpoint.
