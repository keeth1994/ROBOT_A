# Robot B9-1

Current Fusion assembly and CAD-derived MuJoCo simulation, using eight servos: four Parallax on pitch, two HS-475HB on front yaw and two MG995 on rear yaw, modeled at 5 V.

## Run

From this folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe robot.py walk
.\.venv\Scripts\python.exe robot.py walk --gait-profile slope
.\.venv\Scripts\python.exe robot.py walk --gait-profile transition
.\.venv\Scripts\python.exe robot.py walk --ramp --seconds 100
.\.venv\Scripts\python.exe robot.py walk --ramp --test
.\.venv\Scripts\python.exe robot.py walk --ramp --video results/b91_course.mp4
.\.venv\Scripts\python.exe robot.py walk --test --seconds 15
.\.venv\Scripts\python.exe robot.py check all
```

`start.ps1` starts the course. `--ramp` automatically selects the `course` gait and pauses the simulation after a verified completion. Explicit `--gait-profile` selections still override this. Run `robot.py --help` for available commands. No RL training pipeline or trained policy is included.

A small gait control window opens, allowing you to switch between gait profiles.

## Current files

- [Fusion assembly](cad/B9_1/Robot_B9_1.f3d)
- [STEP assembly](cad/B9_1/Robot_B9_1.step)
- [CAD workflow](cad/README.md)
- [Simulation assumptions and results](docs/DESIGN_B91_SIM.md)
- [Ramp controller, validation and physical transfer requirements](docs/RAMP_CONTROLLER.md)
- [ESP32 firmware prototype and commissioning](firmware/README.md)
- [Recorded course traversal (2x playback)](results/b91_course.mp4)
- [Machine-readable current design](cad/current_design.json)

| Folder | Contents |
|---|---|
| `robot_b91/` | Model builder, walking controller, geometry and terrain helpers |
| `cad/` | Current assembly, source yaw and Fusion exporter |
| `reference/` | Current CAD export, mass, kinematics, gait and terrain settings |
| `models/` | MuJoCo models and visual meshes |
| `tests/` | Model and entry-point checks |
| `results/` | Current standing, walking and course reports plus preview |

## Setup and rebuild

Use Python 3.12 and install `requirements.txt` into `.venv`. The lock file records pinned dependencies.

After CAD edits, export the saved assembly using `cad/ExportB91` inside Fusion, then run `robot.py build`. This regenerates the simulation from the CAD export. Edit `reference/b91_course_settings.json` to change terrain, then rebuild.

Provisional mass: 1.163 kg. Flat test: 0.502 m in 15 seconds without falling.
The course gait completes all five ramp sections in 63.406 simulated seconds,
with every foot visiting every section, no side-floor bypass and no fall.
CAD, terrain, mass, friction and servo ratings are unchanged. This is a simulation
result: steering uses ideal heading and lateral position. An ESP32 firmware
prototype reuses the gait with IMU heading and operator lateral steering, but
physical calibration, robot testing and autonomous localization remain unfinished.
