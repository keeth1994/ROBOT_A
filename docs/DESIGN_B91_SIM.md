# B9-1 MuJoCo transfer

`robot.py walk` and `robot.py build` now use B9-1. The default VS Code course task and `start.ps1` also use B9-1. Use `walk --test` for a saved report. No trained policy or RL training pipeline is included.

```powershell
.\.venv\Scripts\python.exe robot.py walk
.\.venv\Scripts\python.exe robot.py walk --ramp --seconds 100
.\.venv\Scripts\python.exe robot.py walk --test --seconds 15
.\.venv\Scripts\python.exe robot.py check b91
```

## Source and rebuild

Saved source: `cad/B9_1/Robot_B9_1.f3d`. The Fusion exporter `cad/export_b91_sim.py` loads this archive and exports 97 bodies in assembly coordinates to `reference/b91_cad_meshes.json.gz`, including source SHA-256. To run the exporter, register `cad/ExportB91` in Fusion Scripts and Add-Ins. It does not edit the CAD source. Save any new design edits into the source archive before exporting.

`robot.py build` verifies the archive hash, regenerates `models/robot_b91.xml`, `models/robot_b91_course.xml`, their meshes and the mass manifest. Course settings are in `reference/b91_course_settings.json`. B9-1 course metadata is saved separately.

## Mechanical model

- Neutral size: approximately 272 x 272 x 132 mm.
- Yaw axes at (+/-52, +/-52) mm; pitch axes at local (55, 0, 35) mm.
- Pitch motors and rockers move together; the original pitch crosses remain on the yaw forks.
- Eight torque-controlled actuators. Original servo stall limit 0.26834 Nm; yaw output 0.15206 Nm with assumed 1.5 speed ratio and 85% gear efficiency. Speed reduces available motoring torque using the inherited servo model.
- Pitch and yaw are limited to +/-45 degrees pending broader collision validation. Driver gears follow a joint equality constraint.
- CAD visuals, plus 639 segmented convex collision hulls. Collision surfaces use 0.5 mm vertex clustering; small holes, fasteners and bearing internals are omitted. Concave openings are approximated, not exact contact geometry.
- Provisional total mass 1.20999 kg. `reference/b91_mass.json` defines component allowances and material densities. PLA uses full solid CAD volume; actual manufactured mass can differ. Inertias use per-part bounding boxes and CAD centres of mass.
- Three ideal IR rays, a camera and acceleration sensor at CAD-derived package positions. These are not calibrated electrical sensor models; exact IMU capabilities remain to be confirmed.
- Rigid terrain, assumed friction 0.8, no backlash, thermal limits or flexible parts. No imposed base movement or forced course path.

## Initial checks and results

`tests/test_b91.py` verifies source provenance, body count, mass, pitch-axis positions, moving-link assignments, stable standing, foot-only floor contacts, joint motion and torque limits. Both flat and course XML load successfully.

The programmed gait moved 0.505 m forward with 0.0155 m lateral drift in a 15 s flat test (including 2 s settling), without falling or physics warnings. See `results/b91_walk.json`.

In a 100 s course test it moved 0.873 m forward, drifted 0.209 m sideways and fell (maximum tilt 54.9 degrees). There were no physics warnings. See `results/b91_ramp.json`. This is a failed traversal, not a successful demonstration of the obstacle course.

No learned policy is included or validated for this model.
