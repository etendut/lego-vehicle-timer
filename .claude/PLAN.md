# ODV Pathfinding & Memory Improvement Plan

## Background
The ODV vehicle module (`modules/vehicle_odv.py`) was hitting out-of-memory crashes on the Technic Hub.
Two root causes were identified: fragile/broken movement validation logic and an O(N²) BFS memory pattern.
This plan addresses both in discrete, independently committable steps.

## After each task
Run `python -m modules.compile_pybricks_files` to regenerate the compiled `lego_vehicle_timer_*.py` files, then run `python -m pytest tests/`.

## Agreed Constraints (from planning session)
- One-way tiles (`<`, `>`) are one-way *streets*: enter from one side, exit the other (e.g. `<` = enter from east, exit to west)
- HOME tile has physical barriers: can only be entered via direction NORTH, WEST, or NORTH_WEST
- Pathfinding (BFS) and real-time driving can use separate movement validation logic
- BFS should explore all 8 directions including diagonals
- Debug prints kept behind `DEBUG = const(False)` flag (not deleted)
- Target hardware: LEGO Technic Hub (MicroPython)

## Tasks

### Task 1 — Add DEBUG flag and wrap all print statements
**Status**: pending  
**File**: `modules/vehicle_odv.py`  
**Details**: Add `DEBUG = const(False)` near the top of the module section. Wrap every `print()` call in `if DEBUG:`. MicroPython's const() dead-branch elimination means zero runtime cost when DEBUG is False — no string allocation on hot paths.  
**Commit message**: `perf: add DEBUG flag to eliminate print overhead in vehicle_odv`

---

### Task 2 — Rewrite `can_move_in_direction_by_type` with clean rule-based logic
**Status**: done  
**Files**: `modules/vehicle_odv.py`, `tests/test_vehicle_odv.py`  
**Details**: Replace the 60-line fragile special-case function with ~12 lines of sequential rules applied to the 4 corner tile types:
1. Any corner on WALL → block
2. Any corner on `<` (WEST_ONLY) and direction != WEST → block
3. Any corner on `>` (EAST_ONLY) and direction != EAST → block
4. Any corner on HOME and direction not in [NORTH, WEST, NORTH_WEST] → block
5. Otherwise → allow

Update `tests/test_vehicle_odv.py` `can_move_in_direction_by_type_tests` to cover the new logic.  
**Commit message**: `fix: rewrite can_move_in_direction_by_type with clean rule-based logic`

---

### Task 3 — Add coarse-tile movement check for BFS pathfinding
**Status**: done  
**File**: `modules/vehicle_odv.py`  
**Details**: Add a new module-level function `_can_traverse_coarse(from_type, to_type, direction) -> bool` with simple tile-pair rules (no bounding box needed):
- `to_type == WALL` → False
- `to_type` or `from_type` is WEST_ONLY and direction != WEST → False
- `to_type` or `from_type` is EAST_ONLY and direction != EAST → False
- `to_type == HOME` and direction not in [NORTH, WEST, NORTH_WEST] → False
- Otherwise → True

Replace `_can_move_in_direction_from_tile_` usage in BFS with this function.  
**Commit message**: `refactor: add coarse-tile movement check to decouple BFS from fine-grid logic`

---

### Task 4 — Rewrite BFS with parent-pointer map and 8-directional search
**Status**: pending (blocked by Task 3)  
**Files**: `modules/vehicle_odv.py`, `tests/test_vehicle_odv.py`  
**Details**:
- Replace `new_path = list(path)` pattern (O(N²) memory) with a parent dict `{tile: (parent_tile, direction)}`
- Reconstruct path by walking back through the parent map after goal is found
- Expand search from 4 cardinal directions to all 8 (N, NE, E, SE, S, SW, W, NW)
- Use `_can_traverse_coarse` for movement validation
- Remove the `Queue` class — replace with a simple list used as a queue with an index pointer
- Update `bfs_test` fixtures in `tests/test_vehicle_odv.py` — expected paths will change as diagonal routes become available

**Commit message**: `perf: rewrite BFS with parent-pointer map and 8-directional diagonal search`

## Order of implementation
```
Task 1 (DEBUG flag)         — no dependencies, do first or anytime
Task 2 (movement rewrite)   — no dependencies, do alongside Task 1
Task 3 (coarse check)       — requires Task 2
Task 4 (BFS rewrite)        — requires Task 3
```
