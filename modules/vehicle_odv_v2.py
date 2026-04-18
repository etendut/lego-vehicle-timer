# IMPORTS_START
# IMPORTS_END

# local var only
from micropython import const
def mock_const(val):
    return val
# for testing
if const(12) is None:
    const = mock_const

# VARS_START
DEBUG = const(False)

_DEG_PER_TILE = const(800)
_CART_SIZE_DEG = const(640)
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

    def _axis_step_legal(self, cx, cy, d, axis):
        if axis == _X:
            new_cx = cx + d
            new_cy = cy
        else:
            new_cx = cx
            new_cy = cy + d

        if self._aabb_hits_wall(new_cx, new_cy):
            return False

        if axis == _X:
            half = _HALF
            if d > 0:
                # Eastbound: check '<' west-edge barriers
                east_face_before = cx + half
                east_face_after = east_face_before + d
                for bx, y_top, y_bottom in self._west_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    crossing = east_face_before <= bx and east_face_after > bx
                    if y_overlap and crossing:
                        return False
            elif d < 0:
                # Westbound: check '>' east-edge barriers
                west_face_before = cx - half
                west_face_after = west_face_before + d
                for bx, y_top, y_bottom in self._east_barriers:
                    y_overlap = (cy - half) < y_bottom and (cy + half) > y_top
                    crossing = west_face_before >= bx and west_face_after < bx
                    if y_overlap and crossing:
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

# MODULE_END

# DRIVE_SETUP_START
# placeholder — populated in Task 7 cutover
# DRIVE_SETUP_END
