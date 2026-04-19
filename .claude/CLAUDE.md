# CLAUDE.md — lego-vehicle-timer

## Project context
PyBricks LEGO robot project targeting a **LEGO Technic Hub** (MicroPython/PyBricks). Memory is tight — avoid patterns that allocate on hot paths.

## Working style
- Ask clarifying questions **one at a time**, not as a list.
- Implement and commit **each task separately** — stop after each task and wait for the user to commit before moving to the next.

## After modifying any `modules/vehicle_*.py`
Run `python tools/compile_pybricks_files.py` to regenerate the compiled `lego_vehicle_timer_*.py` files, then `python -m pytest tests/`.

## Project files
All Claude-related files live under `.claude/`:
- `.claude/analysis/` — design docs (current-state analyses, architecture write-ups).
- `.claude/work/` — implementation plans. Active plans live here during work; completed plans stay for historical reference, prefixed `YYYY-MM_`.

## Active work
No active redesign. Branch `v3` is the as-shipped ODV rewrite
(arcade-style virtual-joystick, AABB-in-tile-grid validation,
4-direction BFS, explicit `DRIVE_MODE` enum) pending on-rig
verification.

Completed work (for historical context):
- `.claude/work/2026-04_movement_redesign.md` — full ODV rewrite.
  All 10 tasks landed 2026-04-18. The as-shipped architecture is
  captured in `.claude/analysis/odv_movement_redesign.md` Part B;
  read that at the start of any session touching ODV movement.
- `.claude/work/2026-04_bfs_rewrite.md` — earlier ODV improvement
  plan (DEBUG flag, rule-based validation, coarse check, BFS with
  parent-pointer map). Superseded by the movement redesign.
- `.claude/work/2026-04_home_unload_merge.md` — collapsed the earlier
  HOME + END tiles into a single `U` (unload) tile.
