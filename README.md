# Robot B3

Compact four-legged rocker robot with original Parallax servos, MuJoCo simulation, a programmed walking controller and PPO reinforcement learning.

## Run

From this folder in PowerShell:

```powershell
# Normal walking
.\.venv\Scripts\python.exe robot.py walk

# Full obstacle course
.\.venv\Scripts\python.exe robot.py walk --ramp --seconds 100

# Meeting demos: normal, rl-flat, or rl-ramp
.\.venv\Scripts\python.exe robot.py demo rl-ramp

# All available commands
.\.venv\Scripts\python.exe robot.py --help
```

`MEETING_DEMOS.cmd`, `start.ps1`, and the VS Code tasks also launch these workflows. Ctrl+Shift+B starts the full course. Relative checkpoint and output paths resolve against this repository, even when the launcher is called from another folder.

## Continue training

```powershell
.\.venv\Scripts\python.exe robot.py train --slope 5 --resume results/my_3deg_run_02/best.zip --out results/my_5deg_run_01 --steps 1000000 --minutes 15
```

Always choose a new output folder. Continue a lesson from `latest.zip`; advance to a harder lesson from a validated `best.zip`. See the [curriculum guide](docs/RL_CURRICULUM.md) for evaluation and stage gates. The 3-degree lesson and the full photographed course are different terrains.

## Repository map

| Location | Purpose |
|---|---|
| `robot.py` | One command entry point |
| `robot_b3/` | Simulation, controllers, sensors, model builder and RL tools |
| `tests/` | Physics, terrain, sensor and RL checks |
| `docs/` | Design notes, meeting instructions and training guides |
| `cad/` | Current Fusion/STEP model and Fusion scripts |
| `reference/` | Geometry, sensor, course and selected-policy configuration |
| `models/` | MuJoCo XML and meshes |
| `results/` | Experiment history, checkpoints, reports and videos |
| `logs/` | Ignored local diagnostics |

Model and result paths are stable. Historical experiments remain available because their reports and checkpoints reference them. Source modules moved from the root to `robot_b3/`; use `robot.py` instead of the previous standalone script commands. Existing source-file hashes in old reports describe the source at the time of that experiment.

## Setup and checks

Use Python 3.12. Install `requirements.txt` into `.venv`, plus `requirements-rl.txt` for learning. The corresponding lock files record the dependency versions.

```powershell
.\.venv\Scripts\python.exe robot.py check all
.\.venv\Scripts\python.exe robot.py build
```

`check all` runs regression checks; it does not certify successful course traversal. `build` regenerates simulation geometry from the current CAD export. Run it only after changing the source geometry/configuration.

## Guides

- [Meeting demos](docs/MEETING_GUIDE.md)
- [Curriculum training](docs/RL_CURRICULUM.md)
- [Original flat-ground RL workflow](docs/RL_B3.md)
- [Project details](docs/PROJECT.md)
- [Simulation assumptions](docs/DESIGN_B3_SIM.md)
- [Sensors](docs/SENSORS_B3.md)
- [Experiment results](results/README.md)

Locomotion comes from simulated motor forces. Feedback remains idealized; simulation success alone does not demonstrate hardware performance or corridor navigation.
