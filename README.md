# Robot B9-1

Current Fusion assembly and CAD-derived MuJoCo simulation, using eight original Parallax servos and a physics-driven walking controller.

## Run

From this folder in PowerShell:

```powershell
.\.venv\Scripts\python.exe robot.py walk
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

The provisional mass is 1.21 kg. The initial flat test moved 0.505 m in 15 seconds without falling. The 100-second course test drifted and fell; successful course traversal is not established. Physical mass, friction, tolerances and strength still need verification.
