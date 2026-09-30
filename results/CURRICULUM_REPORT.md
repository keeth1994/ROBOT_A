# Straight-line and gentle-ramp training results

The original Parallax motor torque/speed limits, current foot geometry and robot mass were retained. Actions are learned joint targets; there is no programmed gait, forced path, body propulsion or terrain-specific motion script.

## Final fresh-seed checks

Each selected checkpoint was checked on twenty seeds **8000-8019**, separate from training, checkpoint-selection seeds (3000-3002), and earlier evaluations (4000-4019 / 5000-5019). Seeds perturb the initial joints slightly. Friction, mass, battery and sensor errors were not randomized.

| Lesson | Physical completion | Strict clean passes | Mean maximum lateral drift |
|---|---:|---:|---:|
| Flat | 20/20 | 20/20 | 1.54 cm |
| 3 degrees | 17/20 | 6/20 | 3.50 cm |
| 5 degrees | 3/20 | 1/20 | 3.75 cm |

Flat completion means at least 0.5 m forward over the 10 s episode. Ramp completion means contacting ascent, top and descent and clearing the far end without an edge violation or fall. A clean pass additionally requires maximum lateral drift <=3 cm, heading error <=10 degrees, zero non-tread ground contact and no physics warning. Ramp episodes last at most 25 s and stop when the robot steps off the side, before a later fall could occur. Therefore low fall counts alone would not demonstrate safety.

**Stage 1 meets the 90% gate on this test set. Stage 2 does not.** The 3-degree checkpoint often traverses the ramp but does not reliably meet alignment/contact limits. The 5-degree result is weak and varied significantly between evaluation sets: the earlier set had 9/20 physical completions; the fresh set had 3/20. This is not evidence of readiness for the full obstacle course.

These are **three separately selected checkpoints**, not one policy proven across all three lessons. A unified, robust terrain policy remains unfinished. Selection and metric files are preserved; the fresh checks were not followed by additional tuning in this round.

## What changed

- Flat reward saturates speed credit, emphasizes line alignment, and preserves conservative contact checks.
- Small pose/velocity signals are scaled for the network, with an equivalent first-layer conversion when resuming an older policy.
- Fine-tuning uses lower exploration noise and a 0.0001 learning rate.
- Gentle ramps have a 30 cm approach, 40 cm ascent, 20 cm top, 40 cm descent, and 30 cm evaluation width. Heights are about 21 mm (3 degrees) and 35 mm (5 degrees).
- Later slope training mixed flat episodes, wider 50 cm practice ramps and 30 cm ramps. All reported ramp checks use 30 cm width.
- The final trainer selects checkpoints by clean success, then physical completion, then reward. Earlier trial histories retain their original results.

A contact diagnostic on the earlier flat checkpoint found a brief foot-crossbar scrape (~0.03 mm simulated penetration). The geometry and contact criteria were not weakened to make that test pass.

## Watch the results

Default `watch_b3_rl.py` now opens the selected flat policy. VS Code has dedicated tasks for refined straight-line motion and 3/5-degree ramps.

```powershell
.\.venv\Scripts\python.exe robot.py watch
.\.venv\Scripts\python.exe robot.py watch --model results/curriculum_selected/3deg/policy.zip --seed 8000
.\.venv\Scripts\python.exe robot.py watch --model results/curriculum_selected/5deg/policy.zip --seed 8000
```

Replays (seed 8000, one trial each): [flat](curriculum_final_validation/flat/replay.mp4), [3 degrees](curriculum_final_validation/3deg/replay.mp4), [5 degrees](curriculum_final_validation/5deg/replay.mp4). A replay is not a success-rate estimate; use the twenty-trial metrics above.

The policy still uses ideal simulator joint/body feedback. Hardware sensing/estimation, calibrated friction and servo dynamics, and broader robustness checks are required before physical deployment.

## Trial history

Per-command times include setup and validation. Some commands overlapped, so these durations must not be summed as elapsed session time. Steps shown are the checkpoint's cumulative counter along its own branch.

| Run | Final step counter | Command time | Selected checkpoint |
|---|---:|---:|---:|
| curriculum_flat | 593,920 | 252.9 s | 331,776 |
| curriculum_flat_refined | 724,992 | 281.8 s | 724,992 |
| curriculum_flat_refined2 | 856,064 | 113.6 s | 856,064 |
| curriculum_3deg | 1,118,208 | 206.8 s | 1,118,208 |
| curriculum_3deg_refined | 1,249,280 | 131.2 s | 1,249,280 |
| curriculum_5deg | 1,380,352 | 211.9 s | 1,314,816 |
| curriculum_5deg_transfer | 1,380,352 | 142.3 s | 1,380,352 |

Checks passed: Gym API, reproducible resets, motor limits/no external forces, full-CAD versus headless physics parity, physical slope heights, 30 cm width, edge/contact accounting and rejection of a stationary policy. See `RL_CURRICULUM.md` for reproducible training commands.
