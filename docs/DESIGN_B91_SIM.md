# B9-8 simulation

Run `python robot.py walk` for flat walking; add `--ramp` for the course.
Use `--test --seconds 15` to save a report, or `python robot.py check all` for checks.
On headless Linux set `MUJOCO_GL=egl` for rendering and sensor feedback.

## Model

The default flat and course models are `models/robot_b98.xml` and
`models/robot_b98_course.xml`. The ramp is 0.29 m (29 cm) wide;
`reference/b91_course_settings.json` and the camera navigator use that width.
They use all 225 solids from
`cad/B9_8/Robot_B9_8.step`: revised chassis, yaw turrets, braced rocker feet,
two equipment levels, battery/controller mounts, matrix board and camera.
`reference/robot_b98_manifest.json` records source SHA-256, component masses,
link assignments and collision count. Legacy B9-1 assets remain available as history.

- Provisional mass: 1.448 kg. Printed parts use solid PLA density, not slicer infill.
  Equipment masses are allowances; see `reference/b98_mass.json`. The 40 g matrix
  board allowance includes ESP32, and the IMU chip is 2 g. Camera ears share the
  camera's 14 g allowance. Explicit steel fasteners are counted separately from
  the extra 50 g wiring/straps/unmodeled hardware allowance.
- OpenCascade solid inertia tensors, scaled to each component's assigned mass,
  are combined with the parallel-axis theorem. Equipment internals are approximated
  by their CAD envelopes; the wiring allowance uses a box approximation.
- 829 segmented convex CAD collision hulls use 0.5 mm vertex clustering. Small
  hardware remains in visuals and mass but is omitted from collision.
- Pitch pivots are 52 mm from the yaw centers at Z=35 mm, matching B9-8 CAD,
  versus 55 mm in B9-1. Driver axes follow the circular horn centers.
- IMU site: (0, 0, 84.1) mm in chassis coordinates. IR sites follow the updated
  sensor positions; retained mounting orientations come from `b91_sensor_axes.json`.
  The camera is placed at the forward face of its upright B9-8 envelope.
  The navigator takes the camera offset from the loaded model; its 15-degree
  downward optical pitch and field of view remain simulation assumptions.
- Eight servos at 5 V: Parallax pitch, HS-475HB front yaw, MG995 rear yaw.
  Output torque estimates: pitch 0.224 Nm, front yaw 0.255 Nm, rear yaw 0.486 Nm.
  Yaw uses a 1.5 speed ratio and 85% assumed gear efficiency. Torque falls with
  speed, with commands at 50 Hz. See `reference/b98_kinematics.json`.
- Standard-model joint limits in the CAD frame: yaw -45 to +45 degrees,
  pitch -135 to 0 degrees (foot-down maximum). Pitch calibration uses a raised zero 45 degrees
  above CAD neutral: calibration -90/-45/0/+45 = CAD -135/-90/-45/0.
  The ramp model uses pitch -45 to 0 degrees and yaw +/-45 degrees. The former
  calibration +90 degree point was removed because it collides with the body.
  Existing gait profiles retain their previous pitch commands; the reduced
  physical upper stop may still affect dynamics. An explicit
  `pitch_min = -2.356194490192345` in a non-ramp gait profile permits higher
  lifting if `pitch_lift`/`pitch_stance` request it. The B9-8 fixed-part CAD motion
  report samples pitch only through +25 degrees; it does not certify the expanded
  simulated range or moving-part clearance.
- Rigid bodies and nominal friction 0.8; no supply limits, voltage sag, heat,
  backlash or calibrated sensor noise. No trained policy is included.

## Rebuilding

`python robot.py build` uses the saved `reference/b98_cad_meshes.json.gz` export;
it checks the STEP hash to reject stale geometry. After CAD changes, install
`requirements-cad.txt` and run `python cad/export_b98_sim.py`, then rebuild.
The exporter preserves named assembly occurrences and converts STEP millimetres
to metres; Fusion is not needed. Optional OpenCascade dependencies are only
required for export, not for running or rebuilding the saved simulation.

The Python package remains named `robot_b91` for compatibility. Shared terrain,
gait, navigation and sensor-axis configuration files retain their `b91_*` names.
The existing flat, slope, transition and course gait parameters are retained.
`--ramp` selects `course` by default; `--gait-profile` overrides it. Interactive
runs provide a gait-control window with a 0.5-second transition between profiles.
New evaluation reports use `results/b98_*`; the B9-1 video/reports are historical.

## B9-8 validation

- Source hash, 225 solids, mass accounting, CAD pitch axes, driver axes and sensor
  positions pass the model checks. Eight motor limits and standing tread contacts pass.
- Standing, 15 s with ideal feedback: stable, no physics warnings.
- Flat, 15 s with ideal feedback: 0.462 m forward, 0.0248 m sideways, no fall or
  physics warnings. See `results/b98_walk.json`.
- Historical 30 cm-wide ideal-feedback course, 82.354 s: complete, all feet on all five sections in
  order, no fall, bypass or warnings. Maximum tilt 21.72 degrees. See
  `results/b98_ramp.json`.
- Current 29 cm-wide camera/IMU course feedback, 82.330 s: complete, no fall, bypass or physics
  warnings. Maximum tilt 21.52 degrees. Non-foot terrain contact occurs in
  2.1% of evaluated timesteps, so this is not a scrape-free traversal. See
  `results/b98_course_validation.json`.
- `MUJOCO_GL=egl .venv/bin/python robot.py check all` passes model, structure and
  nominal full-course checks. Perturbed-condition robustness was not rerun.
- Preview: `results/b98_standing.png`.

## Historical B9-1 results

- Standing, 15 s: stable.
- Flat, 15 s: 0.502 m forward, 0.0226 m sideways; no fall.
- Original flat gait on course, 100 s: 0.495 m forward, 0.227 m sideways; course incomplete.
- New course gait: completed in 63.406 s, 3.554 m forward, 0.0115 m sideways at
  exit, maximum tilt 21.60 degrees. All four feet visit all five panels in order;
  no fall, side-floor bypass or physics warnings. Nonfoot terrain contact occurs
  in 0.335% of evaluated timesteps, so this is not a scrape-free traversal.

No physics warnings in these tests. Reports are in `results/`.
No trained policy is included.

See [Ramp controller](RAMP_CONTROLLER.md) for the gait changes, robustness tests,
recording command and the distinction between simulated success and physical deployment.
