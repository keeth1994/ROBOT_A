# Robot B3 - Y-fork rocker quadruped

Original Parallax servos, top-facing yaw outputs, Y-forks, and pitch servos moving with the rocker legs. Only B3 is retained in this repository.

## Start

In VS Code, press **Ctrl+Shift+B** for the flat / 10-degree uphill / crest / 10-degree downhill test, or run `./start.ps1` in PowerShell. The viewer stays open after 100 simulated seconds.

## Setup and reproduce

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe build_robot_b3.py
.\.venv\Scripts\python.exe test_b3_course.py --flat --seconds 25
.\.venv\Scripts\python.exe test_b3_course.py --seconds 100
.\.venv\Scripts\python.exe record_b3_course.py --seconds 100
```

The builder uses only B3 reference files and `mesh_utils.py`. No previous version is required. `requirements-lock.txt` captures the existing environment.

## Contents

- `cad/`: Fusion and STEP snapshots, previews and CAD clearance audit.
- `models/`: B3 flat/course models and visual meshes.
- `reference/`: B3 geometry, kinematics, gait, course and mass assumptions.
- `results/`: current flat/course measurements and replay.
- `DESIGN_B3_SIM.md`: test setup, results and limitations.

## Current result

Flat walking: 0.782 m in 25 s. Course traversal: 2.315 m in 100 s, no falls. **Not a clean 300 mm corridor pass:** tread overhang reaches 78 mm. Steering uses ideal simulated pose; collision geometry and mass are approximate. Full 270-degree CAD yaw is not achieved. These results do not certify hardware performance.
