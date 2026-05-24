# IMPORTS_START
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
try:
    from pybricks.tools import StopWatch, wait  # type: ignore[assignment]
except ImportError:
    class StopWatch:  # host-side stub for testing
        def time(self) -> int: return 0
        def reset(self) -> None: pass
    def wait(time: int) -> None: pass
try:
    from uerrno import ENODEV
except ImportError:
    ENODEV = -99
# IMPORTS_END

from micropython import const
if const(0) is None:  # desktop micropython package returns None; swap in a passthrough
    def const(x):  # type: ignore[assignment]
        return x
from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes
from pybricks.parameters import Button

error_flash_code = ErrorFlashCodes()
remote = None

# VARS_START

# ── drive mode enum (do not change) ──────────────────────────────────────────
MANUAL = const(0)
HYBRID = const(1)
AUTO   = const(2)

# ── grid patterns ─────────────────────────────────────────────────────────────
ODV_GRID_DEFAULT = ["L#<#U", "X#<#X", "X###X"]
ODV_GRID_EX1     = ["###X#XX", "LX###XU", "###X###"]
ODV_GRID_EX2     = ["X###X", "L###U", "X###X"]
ODV_GRID_EX3     = ["X#>#X", "L#X#U", "X#<#X"]

# ── user configuration ────────────────────────────────────────────────────────
DRIVE_MODE        = const(MANUAL)
IDLE_TIMEOUT_SECS = const(20)  # HYBRID only: seconds idle before auto-drive engages
ODV_SPEED         = const(60)  # note above 60 gets a bit chaotic on full battery. max speed in MANUAL and HYBRID modes
ODV_GRID          = ODV_GRID_DEFAULT
AUTO_UNLOAD_ON_TIMER_END = const(True)  # when True, the cart parks at U on natural timer expiry (not on reset-code press)

# ── debug / calibration ───────────────────────────────────────────────────────
DEBUG               = const(False)
CALIBRATE_X_OFFSET = const(False)  # halt auto_load at tile (3, 0) center for X-offset measurement

# ── internal tuning (change with caution) ────────────────────────────────────
_DEG_PER_TILE     = const(800)
_CART_SIZE_DEG    = const(640)

_LOOKAHEAD_DEG      = const(40)
_STOP_RAMP_MS       = const(200)
_BOTH_AXES_DUTY_NUM = const(71)
_BOTH_AXES_DUTY_DEN = const(100)
_AIM_SWITCH_DEG     = const(160)

_HOMING_MOTOR_ROT_SPEED = const(200)
_HOMING_DUTY            = const(45)
_MAX_MOTOR_ROT_SPEED    = const(1400)
# Auto-drive uses full duty — the controller knows what it's doing, no human in the loop.
_AUTO_DRIVE_DUTY    = const(80)
# Shorter ramp for auto-drive (100ms vs manual 200ms) keeps coast <80° wall clearance.
_AUTO_STOP_RAMP_MS  = const(100)
# Extra slack on predicted coast distance — absorbs motor non-linearity / battery sag.
_DEADBAND_SAFETY_DEG = const(10)
# Rig-measured: at east stall, physical cart center is 80° west of east_wall_deg
# (mechanical slack in the X drive — Y stall is clean to the wall, X is not).
_X_EAST_STALL_OFFSET_DEG = const(80)

# ── derived from user configuration (do not edit) ────────────────────────────
_REMOTE_DISABLED          = (DRIVE_MODE == AUTO)         # AUTO runs headless; MANUAL/HYBRID require the remote
_AUTO_UNLOAD_ON_TIMER_END = AUTO_UNLOAD_ON_TIMER_END     # base default is False; ODV opts in here
_CALIBRATE_X_OFFSET = CALIBRATE_X_OFFSET
# VARS_END

# MODULE_START
WALL = 'X'
TRACK = '#'
WEST_ONLY_TRACK = '<'
EAST_ONLY_TRACK = '>'
LOAD = 'L'
UNLOAD = 'U'

_HALF = _CART_SIZE_DEG // 2  # 320

_X = const(0)
_Y = const(1)


class Grid:
    def __init__(self, layout):
        self.layout = layout
        self.n_rows = len(layout)
        self.n_cols = max(len(row) for row in layout)

        self.load_tile = (0, 0)
        self.unload_tile = (0, 0)

        wall_rects = []
        west_barriers = []
        east_barriers = []

        for ty, row in enumerate(layout):
            for tx, ch in enumerate(row):
                if ch == LOAD:
                    self.load_tile = (tx, ty)
                elif ch == UNLOAD:
                    self.unload_tile = (tx, ty)
                elif ch == WALL:
                    wl = tx * _DEG_PER_TILE
                    wt = ty * _DEG_PER_TILE
                    wr = wl + _DEG_PER_TILE
                    wb = wt + _DEG_PER_TILE
                    wall_rects.append((wl, wt, wr, wb))
                elif ch == WEST_ONLY_TRACK:
                    # '<' west-edge barrier blocks eastbound crossing
                    bx = tx * _DEG_PER_TILE
                    y_top = ty * _DEG_PER_TILE
                    y_bottom = (ty + 1) * _DEG_PER_TILE
                    west_barriers.append((bx, y_top, y_bottom))
                elif ch == EAST_ONLY_TRACK:
                    # '>' east-edge barrier blocks westbound crossing
                    bx = (tx + 1) * _DEG_PER_TILE
                    y_top = ty * _DEG_PER_TILE
                    y_bottom = (ty + 1) * _DEG_PER_TILE
                    east_barriers.append((bx, y_top, y_bottom))

        self._wall_rects = tuple(wall_rects)
        self._west_barriers = tuple(west_barriers)
        self._east_barriers = tuple(east_barriers)

    def tile_type(self, tx, ty):
        if tx < 0 or ty < 0 or ty >= self.n_rows or tx >= len(self.layout[ty]):
            return WALL
        return self.layout[ty][tx]

    def tile_center_deg(self, tile):
        tx, ty = tile
        return (tx * _DEG_PER_TILE + _DEG_PER_TILE // 2,
                ty * _DEG_PER_TILE + _DEG_PER_TILE // 2)

    def deg_to_tile(self, deg_pos):
        tx = deg_pos[0] // _DEG_PER_TILE
        ty = deg_pos[1] // _DEG_PER_TILE
        tx = max(0, min(tx, self.n_cols - 1))
        ty = max(0, min(ty, self.n_rows - 1))
        return (tx, ty)

    def _aabb_hits_wall(self, cx, cy):
        half = _HALF
        L = cx - half
        R = cx + half
        T = cy - half
        B = cy + half
        if L < 0 or T < 0 or R > self.n_cols * _DEG_PER_TILE or B > self.n_rows * _DEG_PER_TILE:
            return True
        for wl, wt, wr, wb in self._wall_rects:
            if R > wl and L < wr and B > wt and T < wb:
                return True
        return False

    def _boundary_overlap(self, cx, cy):
        """Total penetration depth into all obstacles (grid boundaries + wall tiles).
        Used to permit escape moves when motor coast leaves the cart stuck."""
        half = _HALF
        viol = 0
        top = cy - half
        if top < 0:
            viol -= top
        bot = cy + half - self.n_rows * _DEG_PER_TILE
        if bot > 0:
            viol += bot
        lft = cx - half
        if lft < 0:
            viol -= lft
        rgt = cx + half - self.n_cols * _DEG_PER_TILE
        if rgt > 0:
            viol += rgt
        L = cx - half
        R = cx + half
        T = cy - half
        B = cy + half
        for wl, wt, wr, wb in self._wall_rects:
            if R > wl and L < wr and B > wt and T < wb:
                viol += min(R - wl, wr - L, B - wt, wb - T)
        return viol

    def _axis_step_legal(self, cx, cy, d, axis):
        if axis == _X:
            new_cx = cx + d
            new_cy = cy
        else:
            new_cx = cx
            new_cy = cy + d

        if self._aabb_hits_wall(new_cx, new_cy):
            # Already out-of-bounds? Allow any step that doesn't worsen the
            # violation. Strict `<` blocks axes that are independent of the
            # overrun (e.g. cart past east edge wants to slide north along the
            # wall — Y step doesn't change east overrun, so use `<=`). Strict
            # would falsely freeze the cart on the wall corner.
            if self._aabb_hits_wall(cx, cy):
                return self._boundary_overlap(new_cx, new_cy) <= self._boundary_overlap(cx, cy)
            return False

        if axis == _X:
            half = _HALF
            if d > 0:
                # Eastbound: '<' is one-way west. Block while the cart's AABB
                # still straddles the barrier line after the step (west face
                # west of bx AND east face east of bx). Straddle semantics
                # keep the block live across motor-coast ticks — the cart's
                # east face may already be past bx by the time the next tick
                # fires, but as long as the west face hasn't cleared bx we
                # must still refuse further eastbound motion.
                new_x_west = (cx - half) + d
                new_x_east = (cx + half) + d
                for bx, y_top, y_bottom in self._west_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    if y_overlap and new_x_west < bx and new_x_east > bx:
                        return False
            elif d < 0:
                # Westbound: '>' is one-way east. Mirror — block while AABB
                # straddles bx after the step.
                new_x_west = (cx - half) + d
                new_x_east = (cx + half) + d
                for bx, y_top, y_bottom in self._east_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    if y_overlap and new_x_east > bx and new_x_west < bx:
                        return False

        return True

    def propose_step(self, deg_pos, d_deg_x, d_deg_y):
        """
        Return (valid_dx, valid_dy): the largest per-axis step no greater in
        magnitude than the requested one that keeps the cart AABB legal.
        Per-axis independent: X is tested alone, Y is tested alone. If an
        axis is blocked, that axis returns 0; the other axis is unaffected.

        v1 limitation: combined step is not checked. A diagonal move can
        produce a position where the combined AABB overlaps a wall even if
        each individual axis passes. This is rare in practice because the
        AutoDriver's waypoint shaping avoids single-cell corner cuts.
        """
        cx, cy = deg_pos
        valid_dx = 0
        valid_dy = 0

        if d_deg_x != 0:
            if self._axis_step_legal(cx, cy, d_deg_x, _X):
                valid_dx = d_deg_x

        if d_deg_y != 0:
            if self._axis_step_legal(cx, cy, d_deg_y, _Y):
                valid_dy = d_deg_y

        return valid_dx, valid_dy


class VirtualJoystick:
    __slots__ = ('ax', 'ay')

    def __init__(self, ax=0, ay=0):
        self.ax = ax  # -1, 0, or +1
        self.ay = ay  # -1, 0, or +1


class AxisController:
    def __init__(self, motor_x, motor_y, grid, base_duty, _clock=None):
        # _clock: injectable StopWatch-like; defaults to pybricks StopWatch.
        self.motor_x = motor_x
        self.motor_y = motor_y
        self.grid = grid
        self.base_duty = base_duty
        if _clock is None:
            _clock = StopWatch()
        self._clock = _clock
        self._prev_duty_x = 0
        self._prev_duty_y = 0
        self._ramp_start_x = None  # ms timestamp, or None
        self._ramp_start_y = None

    def deg_pos(self):
        return (self.motor_x.angle(), self.motor_y.angle())

    def _ramp_stop_axis(self, motor, prev_duty, ramp_start, ramp_ms):
        """Run one ramp-stop tick for one axis.
        Returns (new_prev_duty, new_ramp_start)."""
        if prev_duty == 0:
            return 0, None
        now = self._clock.time()
        if ramp_start is None:
            ramp_start = now
        elapsed = now - ramp_start
        if elapsed >= ramp_ms:
            motor.dc(0)
            return 0, None
        factor = (ramp_ms - elapsed) * 100 // ramp_ms
        motor.dc(prev_duty * factor // 100)
        return prev_duty, ramp_start  # prev_duty unchanged during ramp

    def tick(self, vj, duty=None, ramp_ms=_STOP_RAMP_MS):
        """One control tick. Proposes a lookahead step, clips it via Grid,
        issues motor.dc per axis. Active→idle transition ramps to zero over
        `ramp_ms` (manual default 200ms; AutoDriver passes a shorter value so
        coast stays inside wall clearances). `duty` overrides base_duty."""
        cx, cy = self.deg_pos()
        base = duty if duty is not None else self.base_duty

        requested_dx = vj.ax * _LOOKAHEAD_DEG
        requested_dy = vj.ay * _LOOKAHEAD_DEG
        valid_dx, valid_dy = self.grid.propose_step((cx, cy), requested_dx, requested_dy)

        # Diagonal duty reduction only applies when BOTH axes actually move.
        # If propose_step blocked one axis (slide-along-wall), the surviving
        # axis runs at full base_duty — otherwise sliding feels sluggish.
        both_moving = valid_dx != 0 and valid_dy != 0
        duty = base * _BOTH_AXES_DUTY_NUM // _BOTH_AXES_DUTY_DEN if both_moving else base

        if DEBUG and (valid_dx != requested_dx or valid_dy != requested_dy):
            print('clip pos=(', cx, cy, ') req=(', requested_dx, requested_dy, ') valid=(', valid_dx, valid_dy, ')')

        # X axis
        if valid_dx > 0:
            self.motor_x.dc(+duty)
            self._prev_duty_x = +duty
            self._ramp_start_x = None
        elif valid_dx < 0:
            self.motor_x.dc(-duty)
            self._prev_duty_x = -duty
            self._ramp_start_x = None
        elif requested_dx != 0:
            # User pressing into an obstacle — brake hard, don't ramp.
            # The ramp keeps the motor driving at decreasing duty for ramp_ms,
            # which on fresh batteries lets the cart coast past walls/edges.
            self.motor_x.brake()
            self._prev_duty_x = 0
            self._ramp_start_x = None
        else:
            self._prev_duty_x, self._ramp_start_x = self._ramp_stop_axis(
                self.motor_x, self._prev_duty_x, self._ramp_start_x, ramp_ms
            )

        # Y axis
        if valid_dy > 0:
            self.motor_y.dc(+duty)
            self._prev_duty_y = +duty
            self._ramp_start_y = None
        elif valid_dy < 0:
            self.motor_y.dc(-duty)
            self._prev_duty_y = -duty
            self._ramp_start_y = None
        elif requested_dy != 0:
            self.motor_y.brake()
            self._prev_duty_y = 0
            self._ramp_start_y = None
        else:
            self._prev_duty_y, self._ramp_start_y = self._ramp_stop_axis(
                self.motor_y, self._prev_duty_y, self._ramp_start_y, ramp_ms
            )


_DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def _aim(delta, deadband):
    """Like _sign but with a deadband — prevents the idle axis from pulsing on
    sub-step drift, and lets the caller size the deadband to match expected
    coast distance so the motor stops pushing exactly when remaining coast
    carries the cart onto the target (no overshoot)."""
    if delta > deadband:
        return 1
    if delta < -deadband:
        return -1
    return 0


def _coast_distance_deg(duty, ramp_ms):
    """Predicted coast distance for a linear ramp from `duty%` to 0 over
    `ramp_ms` — half the triangle: (duty/100) * max_speed_degps * (ramp_ms/1000) / 2."""
    return duty * _MAX_MOTOR_ROT_SPEED * ramp_ms // 200000


def _sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0


def _within(a, b, r):
    return abs(a - b) <= r


class Planner:
    def __init__(self, grid):
        self.grid = grid

    def plan(self, start, goal):
        """Return an immutable tuple of coarse-tile waypoints from start to
        goal. First element == start, last == goal. Intermediate elements
        are only turn-points (cells where the direction changes). Returns
        empty tuple if unreachable or if start/goal is impassable."""
        if not self._passable(start) or not self._passable(goal):
            return ()
        if start == goal:
            return (start,)

        parents = {start: None}
        queue = [start]
        qi = 0
        found = False
        while qi < len(queue):
            tile = queue[qi]
            qi += 1
            if tile == goal:
                found = True
                break
            for dx, dy in _DIRECTIONS:
                nxt = (tile[0] + dx, tile[1] + dy)
                if nxt in parents:
                    continue
                if not self._can_step(nxt, dx):
                    continue
                parents[nxt] = tile
                queue.append(nxt)

        if not found:
            return ()

        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parents[cur]
        path.reverse()

        if len(path) <= 2:
            return tuple(path)
        turn_points = [path[0]]
        prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        for i in range(1, len(path) - 1):
            next_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            if next_dir != prev_dir:
                turn_points.append(path[i])
            prev_dir = next_dir
        turn_points.append(path[-1])
        return self._cut_corners(turn_points)

    def _cut_corners(self, waypoints):
        """Replace L-turn corner tiles with diagonal shortcuts where safe.
        For each triple (a, b, c) with cardinal-perpendicular legs, try
        replacing b with (b minus one step along a->b); keep the swap only
        if the simulated diagonal from candidate to c stays unblocked."""
        if len(waypoints) < 3:
            return tuple(waypoints)
        result = [waypoints[0]]
        n = len(waypoints)
        i = 1
        while i < n - 1:
            a = waypoints[i - 1]
            b = waypoints[i]
            c = waypoints[i + 1]
            d1x, d1y = _sign(b[0] - a[0]), _sign(b[1] - a[1])
            d2x, d2y = _sign(c[0] - b[0]), _sign(c[1] - b[1])
            # Perpendicular cardinals: dot product zero AND neither is the zero vector.
            perpendicular = (d1x * d2x + d1y * d2y) == 0 and (d1x or d1y) and (d2x or d2y)
            if perpendicular:
                candidate = (b[0] - d1x, b[1] - d1y)
                if candidate != a and self._safe_diagonal(candidate, c):
                    result.append(candidate)
                    i += 1
                    continue
            result.append(b)
            i += 1
        result.append(waypoints[-1])
        return tuple(result)

    def _safe_diagonal(self, cand_tile, goal_tile):
        """Simulate AutoDriver-style motion from cand_tile centre to
        goal_tile centre. Safe iff every tick advances each requested axis
        fully (no partial block)."""
        g = self.grid
        cx, cy = g.tile_center_deg(cand_tile)
        gx, gy = g.tile_center_deg(goal_tile)
        # Bound: Manhattan distance / lookahead, with headroom.
        max_ticks = 4 * (abs(gx - cx) + abs(gy - cy)) // _LOOKAHEAD_DEG + 4
        for _ in range(max_ticks):
            dx = gx - cx
            dy = gy - cy
            if dx == 0 and dy == 0:
                return True
            step_x = _sign(dx) * _LOOKAHEAD_DEG
            step_y = _sign(dy) * _LOOKAHEAD_DEG
            if step_x and abs(step_x) > abs(dx):
                step_x = dx
            if step_y and abs(step_y) > abs(dy):
                step_y = dy
            valid_dx, valid_dy = g.propose_step((cx, cy), step_x, step_y)
            if (step_x != 0 and valid_dx != step_x) or (step_y != 0 and valid_dy != step_y):
                return False
            cx += valid_dx
            cy += valid_dy
        return False

    def _passable(self, tile):
        tx, ty = tile
        g = self.grid
        if tx < 0 or ty < 0 or ty >= g.n_rows or tx >= g.n_cols:
            return False
        return g.tile_type(tx, ty) != WALL

    def _can_step(self, nxt, dx):
        if not self._passable(nxt):
            return False
        # N1d edge barriers: arrow direction is the allowed one.
        # '<' west edge blocks eastbound entry into '<'.
        # '>' east edge blocks westbound entry into '>'.
        t = self.grid.tile_type(*nxt)
        if dx > 0 and t == WEST_ONLY_TRACK:
            return False
        if dx < 0 and t == EAST_ONLY_TRACK:
            return False
        return True


class AutoDriver:
    def __init__(self, grid, planner, axis_controller):
        self.grid = grid
        self.planner = planner
        self.axis_controller = axis_controller
        self.waypoints = ()
        self.i = 0

    def start_journey(self, from_tile, to_tile):
        self.waypoints = self.planner.plan(from_tile, to_tile)
        self.i = 0

    def tick(self, remote):
        """Run one tick. Returns:
            None while journey continues,
            'reached_load' / 'reached_unload' / 'reached_end' on arrival,
            'yielded' if any real remote button is pressed (joystick not emitted).
        """
        if remote is not None and any(remote.buttons.pressed()):
            return 'yielded'

        if self.i >= len(self.waypoints) - 1:
            return self._end_tag()

        cx, cy = self.axis_controller.deg_pos()
        target = self.grid.tile_center_deg(self.waypoints[self.i + 1])

        if _within(cx, target[0], _AIM_SWITCH_DEG) and _within(cy, target[1], _AIM_SWITCH_DEG):
            self.i += 1
            if DEBUG:
                print('advance i=', self.i, 'of', len(self.waypoints))
            if self.i >= len(self.waypoints) - 1:
                return self._end_tag()
            target = self.grid.tile_center_deg(self.waypoints[self.i + 1])

        deadband = _coast_distance_deg(_AUTO_DRIVE_DUTY, _AUTO_STOP_RAMP_MS) + _DEADBAND_SAFETY_DEG
        vj = VirtualJoystick(_aim(target[0] - cx, deadband), _aim(target[1] - cy, deadband))
        self.axis_controller.tick(vj, duty=_AUTO_DRIVE_DUTY, ramp_ms=_AUTO_STOP_RAMP_MS)
        return None

    def _end_tag(self):
        if not self.waypoints:
            return 'reached_end'
        last = self.waypoints[-1]
        if last == self.grid.load_tile:
            return 'reached_load'
        if last == self.grid.unload_tile:
            return 'reached_unload'
        return 'reached_end'


class IdleTimeout:
    def __init__(self, seconds, _clock=None):
        self._interval_ms = seconds * 1000
        if _clock is None:
            _clock = StopWatch()
        self._clock = _clock
        self._last_reset_ms = self._clock.time()

    def reset(self):
        self._last_reset_ms = self._clock.time()

    def fired(self):
        return (self._clock.time() - self._last_reset_ms) >= self._interval_ms


class HomingRoutine:
    def __init__(self, motor_x, motor_y, grid):
        self.motor_x = motor_x
        self.motor_y = motor_y
        self.grid = grid

    def run(self):
        """Stall Y north, stall X east, reset encoders so motor.angle() reads
        cart-center position in the grid frame, then run to U's tile center.
        After run(), motor.angle() == grid.tile_center_deg(grid.unload_tile).
        """
        target_x, target_y = self.grid.tile_center_deg(self.grid.unload_tile)
        east_wall_deg = self.grid.n_cols * _DEG_PER_TILE

        # Stall Y NORTH against top wall; cart center is _HALF south of the wall.
        self.motor_y.run_until_stalled(-_HOMING_MOTOR_ROT_SPEED, duty_limit=_HOMING_DUTY)
        wait(200)
        self.motor_y.reset_angle(_HALF)
        self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, target_y)
        wait(200)

        # Stall X EAST against right wall — also unloads the cart.
        self.motor_x.run_until_stalled(_HOMING_MOTOR_ROT_SPEED * 3, duty_limit=_HOMING_DUTY)
        wait(2000)
        self.motor_x.reset_angle(east_wall_deg - _X_EAST_STALL_OFFSET_DEG)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x)
        wait(200)


_LOAD_DIP_DEG = const(240)


class RunODVMotors(MotorHelper):
    """ODV drive coordinator built on the Grid/AxisController/AutoDriver stack."""

    def __init__(self, error_flash_code_helper, drive_speed, grid_layout,
                 _motors=None, _remote=None, _clock=None):
        super().__init__(False, True)
        self.error_flash_code = error_flash_code_helper
        self.drive_speed = drive_speed
        self.has_load = False
        self._remote = _remote

        if _motors is not None:
            self.motor_x, self.motor_y = _motors
        else:
            self.motor_x_port = Port.A
            self.motor_y_port = Port.C
            try:
                self.motor_x = Motor(self.motor_x_port, Direction.COUNTERCLOCKWISE)
            except OSError as ex:
                if ex.errno == ENODEV:
                    self.error_flash_code.set_error_no_motor_on_a()
                raise
            try:
                self.motor_y = Motor(self.motor_y_port, Direction.CLOCKWISE)
            except OSError as ex:
                if ex.errno == ENODEV:
                    self.error_flash_code.set_error_no_motor_on_b()
                raise

        self.grid = Grid(grid_layout)
        self.planner = Planner(self.grid)
        self.axis_controller = AxisController(self.motor_x, self.motor_y, self.grid,
                                              drive_speed, _clock=_clock)
        self.auto_driver = AutoDriver(self.grid, self.planner, self.axis_controller)
        self.homing_routine = HomingRoutine(self.motor_x, self.motor_y, self.grid)

        if DRIVE_MODE == HYBRID:
            self.idle_timeout = IdleTimeout(IDLE_TIMEOUT_SECS, _clock=_clock)
        else:
            self.idle_timeout = None

        self.stop_motors()

    def _current_tile(self):
        return self.grid.deg_to_tile(self.axis_controller.deg_pos())

    def _get_remote(self):
        if self._remote is not None:
            return self._remote
        return remote

    def home_and_unload(self):
        if DEBUG:
            print('home_and_unload start pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')
        self.homing_routine.run()
        self.has_load = False
        self.set_is_homed()
        if DEBUG:
            print('home_and_unload done pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')

    def reset_homing(self):
        self.reset_is_homed()
        self.disable_auto_drive()

    def stop_motors(self):
        self.motor_x.stop()
        self.motor_y.stop()

    def idle_timed_out(self):
        if self.idle_timeout is None:
            return False
        return self.idle_timeout.fired()

    def reset_idle_timeout(self):
        if self.idle_timeout is not None:
            self.idle_timeout.reset()

    def handle_remote_press(self):
        if self.mh__remote_disabled:
            return
        rem = self._get_remote()
        if rem is None:
            return
        pressed = rem.buttons.pressed()

        if len(pressed) == 0 or Button.LEFT in pressed or Button.RIGHT in pressed:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            if len(pressed) > 0 and self.idle_timeout is not None:
                self.idle_timeout.reset()
            return

        ax = 0
        ay = 0
        if Button.LEFT_PLUS in pressed:
            ay = -1
        elif Button.LEFT_MINUS in pressed:
            ay = +1
        if Button.RIGHT_PLUS in pressed:
            ax = +1
        elif Button.RIGHT_MINUS in pressed:
            ax = -1

        if self.idle_timeout is not None:
            self.idle_timeout.reset()

        if ax == 0 and ay == 0:
            self.axis_controller.tick(VirtualJoystick(0, 0))
            return

        cur = self._current_tile()
        # Only trigger load when there's nothing to load and the cart is at L
        # pressing west. Otherwise (already loaded, or just driving through),
        # fall through so _arrive_at_tile doesn't snap the cart back to centre
        # — looks like "drifting toward the ramp" to the operator.
        if (ax, ay) == (-1, 0) and cur == self.grid.load_tile and not self.has_load:
            self._arrive_at_tile(self.grid.load_tile)
            self._do_load_()
            return
        # Same idea for unload: only home_and_unload when there's load to dump.
        if (ax, ay) == (+1, 0) and cur == self.grid.unload_tile and self.has_load:
            self._arrive_at_tile(self.grid.unload_tile)
            self.home_and_unload()
            return

        self.axis_controller.tick(VirtualJoystick(ax, ay))

    def _do_load_(self):
        if self.has_load:
            if DEBUG:
                print('_do_load_ skip (has_load)')
            return
        target_x = self.grid.tile_center_deg(self.grid.load_tile)[0]
        if DEBUG:
            print('_do_load_ start pos=(', self.motor_x.angle(), self.motor_y.angle(),
                  ') dip to', target_x - _LOAD_DIP_DEG)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x - _LOAD_DIP_DEG)
        wait(2000)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, target_x)
        self.has_load = True
        if DEBUG:
            print('_do_load_ done pos=(', self.motor_x.angle(), self.motor_y.angle(), ')')

    def _drive_auto_journey(self, goal_tile):
        start = self._current_tile()
        self.auto_driver.start_journey(start, goal_tile)
        rem = self._get_remote()
        if DEBUG:
            print('journey', start, '->', goal_tile, 'waypoints', self.auto_driver.waypoints)
        tick_count = 0
        while True:
            result = self.auto_driver.tick(rem)
            tick_count += 1
            if DEBUG and tick_count % 50 == 0:
                cx, cy = self.axis_controller.deg_pos()
                i = self.auto_driver.i
                wps = self.auto_driver.waypoints
                nxt = wps[i + 1] if i + 1 < len(wps) else None
                print('t=', tick_count, 'i=', i, 'pos=(', cx, cy, ') next=', nxt)
            if result == 'yielded':
                if DEBUG:
                    print('yielded')
                self.disable_auto_drive()
                self.stop_motors()
                return False
            if result is not None:
                if DEBUG:
                    print(result)
                self._arrive_at_tile(goal_tile)
                return True
            wait(10)

    def _arrive_at_tile(self, tile):
        """Center cart on tile, stop, then pause 500ms before the caller's
        load/unload action. Shared by auto-drive arrival and manual load/unload."""
        ex, ey = self.grid.tile_center_deg(tile)
        self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, ex)
        self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, ey)
        self.stop_motors()
        wait(500)

    def auto_load(self):
        if not self.mh_is_homed:
            return
        if _CALIBRATE_X_OFFSET:
            # Halt at tile (3, 0) center so the X offset can be measured on rig.
            if self._drive_auto_journey((3, 0)):
                ex, ey = self.grid.tile_center_deg((3, 0))
                self.motor_x.run_target(_MAX_MOTOR_ROT_SPEED, ex)
                self.motor_y.run_target(_MAX_MOTOR_ROT_SPEED, ey)
                cx, cy = self.axis_controller.deg_pos()
                print('CALIBRATE pos=(', cx, cy, ') expected=(', ex, ey, ')')
                raise SystemExit('calibration stop at tile (3, 0)')
            return
        if self._drive_auto_journey(self.grid.load_tile):
            self._do_load_()

    def auto_unload(self):
        if not self.mh_is_homed:
            return
        if not self.has_load:
            return
        if self._drive_auto_journey(self.grid.unload_tile):
            self.home_and_unload()

    def park_at_unload(self):
        """Timer-end auto-park at U. If the encoder is calibrated, navigate via
        the planner so we respect '<' / '>' barriers. Otherwise skip the
        journey — home_and_unload's physical stall works from any position
        regardless of encoder state."""
        if self.mh_is_homed:
            self._drive_auto_journey(self.grid.unload_tile)
        self.home_and_unload()

# MODULE_END

# DRIVE_SETUP_START
drive_motors = RunODVMotors(error_flash_code, ODV_SPEED, ODV_GRID)
# DRIVE_SETUP_END
