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
- `.claude/PLAN.md` — active implementation plan; read this at the start of every session and update task statuses as work completes.

## Active plan summary
See `.claude/PLAN.md` for full details. All 4 ODV improvement tasks are complete, plus subsequent remote control fixes (box centering, one-way tile wall-on-edge semantics, diagonal support).
