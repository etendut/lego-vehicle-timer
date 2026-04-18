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
The ODV movement system is mid-redesign. The current-state analysis
and target architecture are in
`.claude/analysis/odv_movement_redesign.md` — read it at the start of
every session. Part A describes the system as it stands, Part B
captures the target architecture (arcade-style virtual-joystick with
AABB-in-tile-grid validation). All seven open questions in §8 are
closed.

Completed work (for historical context):
- `.claude/work/2026-04_bfs_rewrite.md` — prior ODV improvement plan
  (DEBUG flag, rule-based validation, coarse check, BFS with
  parent-pointer map). All four tasks done, plus remote-control
  follow-ups.
- `.claude/work/2026-04_home_unload_merge.md` — collapsed the earlier
  HOME + END tiles into a single `U` (unload) tile.
