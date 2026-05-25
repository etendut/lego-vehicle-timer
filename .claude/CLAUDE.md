# CLAUDE.md — lego-vehicle-timer

## Visibility
**This repo is public.** Everything committed here — code, `.claude/analysis/`, `.claude/work/`, `.claude/CLAUDE.md`, `.claude/settings*.json`, commit messages — is world-readable on GitHub. Be deliberate about what lands in a commit:

- **Never commit** user context, session UUIDs, verbatim user quotes that weren't meant for publication, credentials, or anything in `~/.claude/` that the harness would normally keep private.
- **Memory stays at the harness default path** (`~/.claude/projects/<slug>/memory/`), never under `.claude/memory/` — enforced via `.gitignore`.
- Project artifacts like design docs and implementation plans under `.claude/analysis/` and `.claude/work/` are fine to commit, but write them as if an outside reader will see them (no session-internal UUIDs, no quotes attributing off-hand remarks, no personal aside).

## Project context
PyBricks LEGO robot project targeting a **LEGO Technic Hub** (MicroPython/PyBricks). Memory is tight — avoid patterns that allocate on hot paths.

The carts live inside a **GBC (Great Ball Contraption)** — a stopped cart stalls the ball flow, so "keep moving" is a product constraint, not a preference. The `IDLE_TIMEOUT_SECS = 20` auto-drive takeover exists for this reason. For any multi-cart work, "both carts frozen" is a hard failure — design for asymmetry. Racing between carts is **human-adjudicated**; do not add software race / winner / scoring logic unless asked.

## Working style
- Ask clarifying questions **one at a time**, not as a list.
- Implement and commit **each task separately** — stop after each task and wait for the user to commit before moving to the next.

## After modifying any `modules/vehicle_*.py`
Run `python tools/compile_pybricks_files.py` to regenerate the compiled `lego_vehicle_timer_*.py` files, then `python -m pytest tests/`. The pre-commit hook in `.githooks/pre-commit` will recompile automatically when committing changes to `modules/vehicle_*.py`, `modules/lego_vehicle_timer_base.py`, or the compile tool — enable once per clone with `git config core.hooksPath .githooks`.

## Project files
All Claude-related files live under `.claude/`:
- `.claude/analysis/` — design docs (current-state analyses, architecture write-ups).
- `.claude/work/` — implementation plans. Active plans live here during work; completed plans stay for historical reference, prefixed `YYYY-MM_`.

## Active work
No active redesign. Branch `v3` is the as-shipped ODV rewrite
(arcade-style virtual-joystick, AABB-in-tile-grid validation,
4-direction BFS, explicit `DRIVE_MODE` enum).

Rig-verification state:
- **Auto mode**: verified (2026-04-19).
- **Manual mode**: verified (2026-05-25) — full refinement pass
  landed (one-way barriers across coast, brake-on-block, slide-along-
  wall full duty, has_load guards on load/unload triggers,
  AUTO_UNLOAD_ON_TIMER_END, COAST + reset_ramp_state on routine exit).
- **Hybrid mode**: NOT rig-verified — deferred to a future version.

Future directions (not started; 2026-04-19 exploration):
- `.claude/analysis/multi_cart_exploration.md` — multi-cart on one
  grid. Decisions locked: Topology A (one hub / two carts), own
  L/U per cart, A+B+C crossing resolution.
- **Partial tile** — a tile where only part is passable (e.g. SE
  corner, triangle walls). Deferred; not yet explored. Grid-encoding
  question is the key open design call.
- **Dynamic maze** — grid that changes during a run. Deferred; not
  yet explored. Composes naturally with multi-cart (peer cart = moving
  obstacle reuses the same replan machinery).

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
