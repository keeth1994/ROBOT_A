# Group meeting demos

Double-click MEETING_DEMOS.cmd in the repository and select 1, 2, or 3.
Close the MuJoCo window to exit. Live episodes repeat automatically.
Alternatively, VS Code: Terminal > Run Task > Meeting demo - normal / rl-flat / rl-ramp.
Option 4 opens the labelled video backups in results/meeting. Videos play at real time (1x).

## Suggested presentation order

1. Normal walk: a manually programmed joint gait using simulated motor forces.
2. RL flat: PPO chooses eight joint targets from observations; no programmed gait or forced body trajectory.
3. RL ramp: your successful my_3deg_run_02/best.zip climbs, crosses the top, and descends a 3 degree lesson ramp.

All demos retain the original servo torque/speed limits and current geometry.
The flat and ramp RL demos use separate checkpoints. They are not one policy validated on every course.
The camera follows on flat ground; checkerboard motion and the distance counter show progress.

## Recorded examples (seed 4000)

- Normal: 0.194 m in 12 seconds, including its initial 2-second standing period.
- RL flat: 1.356 m in 10 seconds, clean success.
- RL ramp: 1.450 m in 9.76 seconds, clean success across all three ramp surfaces.

These are individual demonstrations, not a controlled speed benchmark: the normal gait has a startup delay and the RL environments settle before timing.
Your saved ramp evaluation reports 19/20 successful trials (95%). This does not establish performance on the much steeper photographed obstacle course.
Ideal simulation state feedback is used. Real sensor estimation, friction, printed stiffness and hardware behavior still need validation.

## Terminal commands (from the repository)

```powershell
.\.venv\Scripts\python.exe meeting_demo.py normal
.\.venv\Scripts\python.exe meeting_demo.py rl-flat
.\.venv\Scripts\python.exe meeting_demo.py rl-ramp
```

Add --record to regenerate the corresponding labelled MP4 and JSON report in results/meeting.
Add --seed 4010 to inspect another RL initial condition. No training is launched by these demos.
