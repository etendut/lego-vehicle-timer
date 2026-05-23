# ODV Rig Sign-off — DRIVE_MODE

On-rig verification checklist for all three DRIVE_MODE values.
Run through each mode in order before publishing a release.

To switch modes, edit `modules/vehicle_odv.py` line 62, then:

```
python tools/compile_pybricks_files.py
# flash docs/pybricks/lego_vehicle_timer_odv.py to the hub
```

---

## Mode 1 — AUTO

`DRIVE_MODE = AUTO` (default)

Remote: **not required**

- [ ] Hub starts and homes successfully; parks at unload tile
- [ ] Auto-cycles run continuously: unload → load → unload → …
- [ ] No remote connected — hub does not raise an error
- [ ] GBC ball flow uninterrupted; cart never freezes

**Result:** ☐ Pass &nbsp; ☐ Fail &nbsp; — verified: __________ by: __________

---

## Mode 2 — MANUAL

`DRIVE_MODE = MANUAL`

Remote: **required**

- [ ] Hub starts and homes successfully (remote must be connected)
- [ ] All 4 remote directions drive the cart correctly
- [ ] Cart stops cleanly when buttons are released (no runaway)
- [ ] Auto-drive never engages, regardless of how long the cart sits idle
- [ ] Load and unload tiles can both be reached manually

**Result:** ☐ Pass &nbsp; ☐ Fail &nbsp; — verified: __________ by: __________

---

## Mode 3 — HYBRID

`DRIVE_MODE = HYBRID`

Remote: **required**

- [ ] Hub starts and homes successfully (remote must be connected)
- [ ] Manual control works immediately after homing
- [ ] After ~20 s idle → auto-cycles engage automatically
- [ ] During auto: any remote button press stops auto and restores manual
- [ ] Idle timer resets after button press; another ~20 s elapses before auto re-engages
- [ ] The manual → auto → manual handoff cycle repeats correctly

**Result:** ☐ Pass &nbsp; ☐ Fail &nbsp; — verified: __________ by: __________
