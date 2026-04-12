# Work Item: Combine HOME and END tiles

## Summary
Remove the HOME (`H`) tile type entirely. Homing is now performed on the END (UNLOAD `U`) tile —
first up (NORTH, Y-axis stall), then right (EAST, X-axis stall). This also empties the cart.
The END tile blocks NORTH movement during normal driving (the homing wall lives there).

## Tasks

### 1 — Remove HOME tile from constants, grid parsing, and tile lookup
**Status**: pending  
**Files**: `modules/vehicle_odv.py`  
- Delete `HOME = 'H'` constant and update the grid-legend comment
- Remove `self.home_tile` from `__init__`
- Remove `HOME` character handling from `_load_grid_`
- Remove `home_tile` branch from `_get_grid_tile_from_coarse_xy_`
- Remove `home_tile` display from `_display_grid_`
- Remove `self.print_tile_pos("--home_tile", ...)` from `_bfs_path_to_grid_tile`

---

### 2 — Update movement rules: drop HOME rule, add UNLOAD-blocks-NORTH rule
**Status**: pending  
**Files**: `modules/vehicle_odv.py`  
- `can_move_in_direction_by_type`: remove Rule 4 (HOME entry restriction); add new rule:
  any corner on UNLOAD **and** direction == NORTH → block
- `_can_traverse_coarse`: remove HOME entry restriction; add:
  `from_type == UNLOAD` **and** direction == NORTH → block

---

### 3 — Rewrite `do_homing` to home on UNLOAD tile (first up, then right)
**Status**: pending  
**Files**: `modules/vehicle_odv.py`  
- Y axis: `run_until_stalled(-speed)` (NORTH), reset angle to `unload_tile_angle[1]`, move SOUTH one pitch
- X axis: `run_until_stalled(+speed)` (EAST), reset angle to `unload_tile_angle[0] + FINE_GRID_SIZE * GEAR_RATIO_TO_GRID`, move WEST one pitch
- Set `self.has_load = False` (homing empties the cart)
- `auto_home` navigates to `unload_tile` instead of `home_tile`

---

### 4 — Update grid definitions (replace H → X)
**Status**: pending  
**Files**: `modules/vehicle_odv.py`  
- `ODV_GRID_YE2`: `"XH#U"` → `"XX#U"`
- `ODV_GRID_GR3`: `"H#X#X"` → `"X#X#X"`
- `ODV_GRID_BL4`: no H, unchanged

---

### 5 — Update tests
**Status**: pending  
**Files**: `tests/test_vehicle_odv.py`  
- Remove `H = 'H'` shorthand
- `can_move_tests`: remove 8 HOME Rule 4 entries; add UNLOAD+NORTH block cases
- `coarse_tests`: remove HOME entry tests; add `from_type=UNLOAD, dir=NORTH → False` cases
- `TEST_GRID`: `"XH<U"` → `"X#<U"` (H→# since (1,0) is reachable track in the test grid)
- Rename `id="home-to-unload"` → `id="track-to-unload"`, `id="unload-to-home"` → `id="unload-to-track"`

---

### 6 — Compile and run tests
**Status**: pending  
- `python -m modules.compile_pybricks_files`
- `python -m pytest tests/`
