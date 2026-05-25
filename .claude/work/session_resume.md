# Session resume — battery swap

Last commit: `4cb257c` (pre-commit hook)

## What landed this session
- `df7e638` removed `block_special` (manual drive can now freely enter L/U)
- `561a8ea` skip load/unload re-trigger when `has_load` already matches
- `c7931be` full duty when sliding along a wall
- `1d4eca0` opt-in `AUTO_UNLOAD_ON_TIMER_END` (confirmed working on rig)
- `c570b95` `park_at_unload` — planner-navigate when homed, stall otherwise
- `9c5c76e` derived vars moved out of "user configuration" section
- `1bd6a38` wrap remaining user-config booleans in `const()`
- `f3e1c9f` / `65512fd` doc icon fixes (orange tile, triangle-head arrow)
- `bcd4097` use single-tile PNGs in grid legend (rendered, not emoji)
- `44d8a25` 2x grid image resolution
- `3a55f51` park short of tile-centre after load / unload (introduced
  `_RETURN_OFFSET_DEG = 80`)
- `f8046b6` homing: full Y return, only short the X stall return
  (regression — Y stall is too shallow to short)
- `9896f16` `then=Stop.COAST` on final return so motor doesn't snap back on
  overshoot
- `4cb257c` pre-commit hook auto-recompiles `docs/pybricks/*.py`

## To verify after batteries swap
1. **COAST fix on final return (`9896f16`)** — after a manual UNLOAD the
   cart should move west (return from east stall) and then SIT. No
   opposite-direction jerk back east. Same shape for LOAD.
2. **Accelerate-into-L/U regression (`df7e638`)** — manual drive can
   now reach L/U at full speed. Confirm whether the cart actually
   slams into the chute or whether the boundary check + brake-on-block
   handles it cleanly. If it's a problem, options are in the chat
   history (soft cap on approach, hard cap above a speed threshold, or
   one-time brake on tile entry).

## Notes
- Pre-commit hook is enabled on this clone (`core.hooksPath .githooks`).
  Any commit touching `modules/vehicle_*.py`,
  `modules/lego_vehicle_timer_base.py`, or the compile tool will
  auto-recompile and stage `docs/pybricks/*.py`.
- `__BUILD__` tag still lags by one commit — chicken-and-egg with the
  commit SHA. Pre-commit only removes the manual recompile step.
- `AUTO_UNLOAD_ON_TIMER_END` is currently `const(True)` in your config.
- `DEBUG` is currently `const(False)`.
