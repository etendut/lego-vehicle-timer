# Future exploration — multi-cart on one grid (GBC)

## Context
Manual/hybrid mode is not yet rig-verified on branch `v3`, but the
user wants to scope multi-cart support ahead of that work. This
file is an **exploration**, not an implementation plan — no code
changes until manual is verified and a concrete scenario is
chosen.

**Product framing:**
The carts live inside a GBC (Great Ball Contraption). A cart must
keep moving to keep the ball flow healthy — that is what the 20s
idle timeout in `IDLE_TIMEOUT_SECS` is for. Multi-cart adds a
second cart on the **same grid** with **its own L and U**, running
its own L↔U cycle on its own schedule. Three operational modes:

1. **Human + human** — two players load/unload simultaneously;
   they may informally race, but racing is not a software concern.
   Light system (nice-to-have) is the only visible "winner" cue.
2. **Human + robot** — one human plays against the autopilot.
3. **Robot + robot** — both auto; no race, just "don't crash".

The interesting play factor is **paths can cross in the middle of
the grid**. That is the feature, not a bug — the carts have to
negotiate a shared crossing. For GBC throughput, the worst outcome
is both carts frozen: the balls stop flowing. So the software
requirement collapses to: **two independent L↔U cycles that keep
moving and do not collide.**

## Current stack, in one paragraph
`Grid` derives wall rects + vertical edge barriers from a
`list[str]` layout in its `__init__` (deg-space, axis-aligned).
`Planner` runs a once-per-journey 4-dir BFS and caches nothing.
`AxisController` owns motor handles, reads `motor.angle()` each
tick, calls `Grid.propose_step(deg_pos, dx, dy) → (valid_dx,
valid_dy)`. `AutoDriver` walks a waypoint list and emits a
`VirtualJoystick` feeding the same controller as manual.
`RunODVMotors` (ln ~580) instantiates the whole stack from a grid
layout and motor ports, with a test-time `_motors` / `_remote`
injection path already in place.

Key files: `modules/vehicle_odv.py`
(Grid ~ln 90, Planner ~ln 346, AxisController ~ln 232,
AutoDriver ~ln 480, RunODVMotors ~ln 580).

## What's already multi-cart-friendly
- `Grid`, `Planner`, `AxisController`, `AutoDriver` each own no
  singletons — one instance per cart is a drop-in.
- `RunODVMotors.__init__` already accepts `_motors=(mx, my)` and
  `_remote=…` as injected deps for tests; same seam works for a
  "second cart" construction.
- `propose_step` is pure over `(deg_pos, dx, dy)` — extending it
  with a peer-cart AABB check is a natural fit.

## What needs to change
1. **`Cart` class extracted from `RunODVMotors`** — today's
   `RunODVMotors` bundles orchestration + per-cart hardware. For
   multi-cart we need a `Cart` (motors + grid + planner + axis
   controller + auto-driver + homing + per-cart idle timeout +
   per-cart DRIVE_MODE) and a thin orchestrator that ticks a list
   of carts.
2. **Per-cart config**: `(port_x, port_y, dir_x, dir_y, drive_mode,
   L_tile, U_tile, remote_or_None)`. Endpoints move out of the
   grid string (see §Grid vocabulary below).
3. **Grid vocabulary for multi-endpoint**: the grid `list[str]`
   currently encodes `L` and `U` as tile characters. With per-cart
   L/U, the cleanest path is to **keep the grid as pure topology**
   (`#`, `X`, `<`, `>`) and move endpoints to per-cart config.
4. **Collision avoidance** — the only genuinely new design
   problem. See §Crossing resolution.
5. **Remote arbitration** — one remote per human cart. With one
   hub, that is two BLE remotes paired to the hub. Per-cart
   `_remote` injection already exists; the pairing code and
   remote-index assignment are new.
6. **Homing** — two carts cannot both stall against the east wall
   at once. Simplest: serialise (cart 1 homes, then cart 2) at
   startup. Mid-run re-homing is rare and can also serialise.
7. **Light system (optional)** — hub LED per cart state
   (moving / idle-timed-out / homed / has-load). No winner logic
   in software; if you want a winner light, a start-stop-reset
   button on each remote could toggle a per-cart "finished" flag.

## Grid vocabulary — multi-endpoint (decided)
Grid string stays `#` / `X` / `<` / `>` only. Load and unload
tiles are per-cart config:
```
carts = [
    {ports: (A, C), mode: AUTO, L: (0, 0), U: (4, 0)},
    {ports: (B, D), mode: AUTO, L: (0, 2), U: (4, 2)},
]
```
Keeps the existing single-char grid encoding (and all of
`Grid.__init__`) intact. Deletes the `L`/`U` lookup branches in
`Grid.__init__` — those tiles become plain `#`, with their
"endpoint" role carried by per-cart config.

Two alternatives were considered and rejected:
- **Digit-suffixed chars** (`L1`, `U2`, `##`) — turns the grid
  into `list[list[str]]`, touches every tile lookup.
- **Case/number chars** (`L`/`l`, `U`/`u`) — keeps single-char but
  caps at two carts and reads poorly in source.

## Crossing resolution — decided: A + B + C
GBC constraint: **never both frozen**. Four strategies, not
mutually exclusive:

A. **Grid-authored one-way crossing** (safest for auto+auto).
   Use `<` / `>` barriers to force crossings to be one-way at the
   tile level so two auto-planned routes cannot both pass through
   the same tile in opposing directions. BFS naturally respects
   barriers today; no runtime conflict logic needed. Cost: grid
   authors have to think about it. Does not handle human drivers
   ignoring the rule.

B. **AABB-vs-peer in `propose_step`** (runtime safety net).
   Cart B's AABB (plus a coast margin) is passed into cart A's
   `propose_step` as a "temporary wall rect". Same mechanism as
   static wall-vs-cart. Both carts see each other, both yield.
   Risk: symmetric yielding → both stop → GBC stalls. Needs an
   asymmetry rule.

C. **Priority / tiebreak rule** (asymmetry for B).
   When AABB-vs-peer would freeze both, the lower-indexed cart
   keeps moving and the higher-indexed cart waits one tick. Cheap,
   guarantees one cart always progresses. Doesn't handle both-
   human case (humans don't consult tiebreak rules).

D. **Replan around peer** (auto only) — deferred.
   When cart A's next waypoint is blocked by cart B, re-run BFS
   with cart B's tile pre-marked as wall. Expensive per tick,
   cheap per event. Revisit only if one-way authoring gets
   painful.

**Decided: A + B + C.** Author crossings with one-way barriers so
auto + auto never actually conflict; keep the peer-AABB check as a
safety net for human drivers; priority tiebreak (lower-indexed
cart wins) ensures GBC never fully stalls.

## Topology — one hub vs two hubs
**One hub (Topology A)** — `RunODVMotors` becomes a thin
orchestrator over a list of `Cart` objects. `Grid` and `Planner`
shared (read-only from carts). Caps at 2 carts (4 motor ports).
No comms layer. Strongly preferred for GBC because state is
centralised and there is no wire-latency penalty on the crossing
AABB check.

**Two hubs (Topology B)** — each cart has its own hub + remote.
Coordinate over BLE hub-to-hub. Much bigger scope: comms protocol,
position reporting, latency margins. Only needed if duty budget
for 4 motors on one hub is too tight (measurable later) or if a
third cart is ever in scope.

Recommended direction: **Topology A first.** The `Cart` extraction
is work you'd pay for under B anyway. Two carts on one hub is a
shippable product; three carts becomes a later conversation.

## Drive-mode mix — all three in scope
`DRIVE_MODE` becomes per-cart (moves off the module level into the
per-cart config). Implications:

- **AUTO + AUTO**: easiest to build first — no remote plumbing at
  all. Natural A/B harness for tuning planner / duty / ramp.
- **HUMAN + AUTO**: near-free once AUTO+AUTO works + remote
  arbitration is solved for a single remote.
- **HUMAN + HUMAN**: needs two BLE remotes paired to the hub, one
  per cart. PyBricks supports multiple `Remote` handles; the
  pairing code is incremental.

## Rough sequencing (no commitment)
1. Extract `Cart` from `RunODVMotors`, behaviour-neutral with one
   cart. Compile + pytest + rig-verify.
2. Add per-cart config + endpoint-out-of-grid. Still one cart.
3. Orchestrator ticks a list-of-carts. Add second cart in AUTO +
   AUTO on a grid authored with one-way crossing barriers. No
   collision logic yet — prove the barriers are enough.
4. Add peer-AABB check in `propose_step` with priority tiebreak.
   Test by authoring a crossing where both carts can technically
   enter.
5. Add second remote for HUMAN + HUMAN. Test all three drive-mode
   mixes.
6. Optional: hub LED state per cart; "finished" flag on a remote
   button if a winner light is wanted.

## Composability with the other two future ideas
- **Dynamic maze** becomes nearly free once peer-AABB is in:
  a peer cart is effectively a moving obstacle already.
- **Partial tile** is still orthogonal, but "one-way crossing
  tile" is basically a partial-tile concept in disguise. If
  Strategy A feels too restrictive (authoring-wise), partial
  tiles become the natural upgrade.

## Open questions
1. **Physical parts** — do you actually have a second cart's
   worth of motors / track / remote? Decides whether this is a
   sim-only exploration or a rig-testable plan.
2. **Hub count** — Topology A preferred; confirm two carts on one
   hub is acceptable before committing to the port-pair config.

## What this plan does NOT do
- Commit to a `Cart` class API.
- Specify file-level changes or line numbers.
- Add a software race / winner / timer mechanism — racing is
  human-adjudicated; software just keeps carts moving and safe.

The next step is to confirm physical scope and hub count, then
produce a one-task-at-a-time plan — after manual mode is
rig-verified on v3.
