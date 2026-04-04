# CLAUDE.md — lego-vehicle-timer

## Project context
PyBricks LEGO robot project targeting a **LEGO Technic Hub** (MicroPython/PyBricks). Memory is tight — avoid patterns that allocate on hot paths.

## Working style
- Ask clarifying questions **one at a time**, not as a list.
- Implement and commit **each task separately** — stop after each task and wait for the user to commit before moving to the next.

## Project files
All Claude-related files live under `.claude/`:
- `.claude/PLAN.md` — active implementation plan; read this at the start of every session and update task statuses as work completes.

## Active plan summary
See `.claude/PLAN.md` for full details. Current tasks (all pending):

1. **Task 1** — Add `DEBUG = const(False)` flag and wrap all `print()` calls in `vehicle_odv.py`
2. **Task 2** — Rewrite `can_move_in_direction_by_type` with clean rule-based logic (+ tests)
3. **Task 3** — Add `_can_traverse_coarse()` for BFS movement validation (blocked by Task 2)
4. **Task 4** — Rewrite BFS with parent-pointer map and 8-directional search (blocked by Task 3)
