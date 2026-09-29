# B3 walking and ramp test

The simulation uses the exported B3 CAD bodies, including the pitch servo moving with the rocker. Only B3 is retained in this repository. Earlier working files were archived outside it.

## Run

In Visual Studio Code, **Ctrl+Shift+B** starts **Start B3 - flat uphill downhill - original servos**, now the default build task. The simulation stops advancing after 100 simulated seconds but leaves the viewer open for inspection. Close and relaunch to restart. Another task, **Start B3 - flat walking**, runs the 25-second flat test.

From the project terminal:

```powershell
.\.venv\Scripts\python.exe run_robot_b3.py --ramp --seconds 100
.\.venv\Scripts\python.exe test_b3_course.py --seconds 100
.\.venv\Scripts\python.exe test_b3_course.py --flat --seconds 25
```

Recorded replay: `results/b3_flat_up_down.mp4`, 2x playback of 100 seconds of physics.

## Course

Flat approach to x=0.40 m; 0.60 m horizontal uphill run at 10 degrees; a 0.30 m crest; 0.60 m horizontal downhill run at 10 degrees; flat exit beginning at x=1.90 m. Crest height is 105.8 mm. All raised sections are 300 mm wide. Transitions are continuous in height. The ramp is rigid; the real flexible panel is not simulated.

## Results

| Test | Result |
|---|---|
| Flat, 25 s including 2 s settling | 0.782 m forward; 25 mm lateral drift; maximum tilt 0.39 degrees |
| Course, 100 s including settling | 2.315 m forward; uphill, crest and downhill all contacted; reaches flat exit |
| Stability | No falls, no non-foot terrain contact, no MuJoCo warnings |
| Corridor | FAIL: tread vertices overhang a side by up to 78.3 mm, sampled every 20 ms |
| Off-ramp floor support | No detected foot contact with the floor beside the raised sections |

This is a traversal demonstration, not a clean 300 mm corridor pass. Width, joint interference and hardware feasibility still need work. The initial gait without lateral correction drifted away from the course; the selected controller adds ideal lateral position feedback. This is a hand-coded gait with feedback, not reinforcement learning.

## Model assumptions

- Eight original Parallax 900-00005 motors. Maximum pitch torque 0.26834 Nm. Yaw output maximum 0.15206 Nm after the 1.5 speed-increasing gears and assumed 85% efficiency. No 1.26 Nm motor upgrade.
- Torque falls linearly with motoring speed, using the inherited 315.79 degrees/second no-load servo speed. Peak values are not continuous thermal ratings; supply sag and overheating are not modelled.
- Estimated mass 1.0757 kg: solid PLA 277.8 g, servos 352 g, battery reservation 180 g, electronics 90 g, IMU 3 g, bearings 40 g, modelled hardware 35.2 g, rubber 17.7 g, miscellaneous allowance 80 g. Battery and electronics are not finalized. This is not measured or slicer-verified mass.
- Pitch +/-45 degrees; yaw restricted to +/-45 degrees. Current target yaw amplitude is 25.8 degrees and pitch lift amplitude 37.2 degrees. Combined CAD poses are not certified collision-free.
- Visual geometry comes from B3. Collision geometry uses simplified open rocker segments, rods, motor boxes and platform proxies. Small mounting features are omitted; adjacent link contacts are filtered by MuJoCo. Dynamics cannot certify exact CAD clearances.
- Friction 0.8 assumed, approximate box inertias, ideal heading/lateral pose feedback, no injected base forces or teleporting.

Machine-readable metrics: `results/b3_flat_test.json`, `results/b3_course_test.json`. Model and mass manifest: `reference/robot_b3_manifest.json`. Controller parameters: `reference/b3_gait.json`.
