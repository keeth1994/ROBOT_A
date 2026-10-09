# Robot B9-1

Current Fusion assembly and CAD-derived MuJoCo simulation, using eight Parallax servos on pitch, HS-475HB on front yaw and MG995 on rear yaw, modeled at 5 V

## Run

From this folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe robot.py walk
.\.venv\Scripts\python.exe robot.py walk --gait-profile slope
.\.venv\Scripts\python.exe robot.py walk --gait-profile transition
.\.venv\Scripts\python.exe robot.py walk --ramp --seconds 100
.\.venv\Scripts\python.exe robot.py walk --test --seconds 15
.\.venv\Scripts\python.exe robot.py check all
```

`start.ps1` and Ctrl+Shift+B in VS Code start the course. Run `robot.py --help` for available commands. No RL training pipeline or trained policy is included.

## Current files

- [Fusion assembly](cad/B9_1/Robot_B9_1.f3d)
- [STEP assembly](cad/B9_1/Robot_B9_1.step)
- [CAD workflow](cad/README.md)
- [Simulation assumptions and results](docs/DESIGN_B91_SIM.md)
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
