# Robot B9-8 CAD and simulation

The current Fusion assembly is B9-8, with a bridge-supported equipment deck, centered matrix-board IMU, and accessible equipment mounts. Yaw turrets and pitch rockers retain the previous user-edited geometry.

The B9-8 MuJoCo model uses four Parallax pitch servos, two HS-475HB front yaw servos and two MG995 rear yaw servos, modeled at 5 V. Visuals, collisions and inertias are generated from the B9-8 STEP assembly.

## Run

From this folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe robot.py walk
.\.venv\Scripts\python.exe robot.py walk --gait-profile slope
.\.venv\Scripts\python.exe robot.py walk --gait-profile transition
.\.venv\Scripts\python.exe robot.py walk --ramp --seconds 100
.\.venv\Scripts\python.exe robot.py walk --ramp --test
.\.venv\Scripts\python.exe robot.py walk --ramp --video results/b98_course.mp4
.\.venv\Scripts\python.exe robot.py walk --test --seconds 15
.\.venv\Scripts\python.exe robot.py check all
```

`start.ps1` starts the course. `--ramp` automatically selects the `course` gait and pauses the simulation after a verified completion. Explicit `--gait-profile` selections still override this. Run `robot.py --help` for available commands. No RL training pipeline or trained policy is included.

A small gait control window opens, allowing you to switch between gait profiles.

## Current files

- [Fusion assembly](cad/B9_8/Robot_B9_8.f3d)
- [STEP assembly](cad/B9_8/Robot_B9_8.step)
- [CAD workflow](cad/README.md)
- [Simulation assumptions and results](docs/DESIGN_B91_SIM.md)
- [Ramp controller, validation and physical transfer requirements](docs/RAMP_CONTROLLER.md)
- [ESP32 firmware prototype and commissioning](firmware/README.md)
- [Historical B9-1 course traversal (2x playback)](results/b91_course.mp4)
- [Machine-readable current design](cad/current_design.json)

| Folder | Contents |
|---|---|
| `robot_b91/` | B9-8 model builder and walking controller (legacy package name) |
| `cad/` | Current assembly, source yaw and Fusion exporter |
| `reference/` | Current CAD export, mass, kinematics, gait and terrain settings |
| `models/` | MuJoCo models and visual meshes |
| `tests/` | Model and entry-point checks |
| `results/` | B9-8 reports (`b98_*`) and historical B9-1 reports/video (`b91_*`) |

See [B9-8 changes and clearance checks](cad/B9_8/README.md) for the current mechanical revision.

## Setup and rebuild

Use Python 3.12 and install `requirements.txt` into `.venv`. The lock file records pinned dependencies.

Run `python robot.py build` to regenerate B9-8 models from the saved CAD mesh export.
After editing the CAD assembly, install the optional `requirements-cad.txt` dependencies,
run `python cad/export_b98_sim.py`, then rebuild. Fusion is not needed for this STEP export.
Edit `reference/b98_mass.json` and `reference/b98_kinematics.json` for B9-8 mass/servo
assumptions. Terrain, gait and sensor-axis settings retain their `b91_*` filenames.

B9-8 provisional simulation mass is **1.448 kg**. Model, structure and nominal
course checks pass; the ramp is **29 cm wide**. Camera/IMU traversal completes
in 82.330 simulated seconds,
with non-foot terrain contact during 2.1% of evaluated timesteps.
See [simulation notes](docs/DESIGN_B91_SIM.md) for validation details. Existing B9-1
course records and video are historical and do not validate the revised model.
An ESP32 firmware prototype reuses the gait with IMU heading and operator lateral
steering; physical calibration, robot testing and autonomous localization remain unfinished.
