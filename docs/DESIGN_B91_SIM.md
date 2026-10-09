# B9-1 simulation

Run `python robot.py walk` for flat walking, add `--ramp` for the course.
Use `--test --seconds 15` to save a report, or `python robot.py check all` for checks.

## Model

- 97 CAD parts and 639 simplified convex collision hulls; CAD geometry is unchanged.
- Eight servos at 5 V: Parallax on pitch, HS-475HB front yaw, MG995 rear yaw.
  Front/rear yaw placement remains provisional.
- Estimated output torque: pitch 0.224 Nm, front yaw 0.255 Nm, rear yaw 0.486 Nm.
  Yaw uses a 1.5 speed ratio and 85% gear efficiency. Torque falls with speed;
  commands update at 50 Hz. PD gains are experimental.
- Joint limits: +/-45 degrees. Provisional mass: 1.163 kg; inertia uses CAD bounding boxes.
- BMI270 acceleration/gyro, IR distance validity 4-30 cm at nominal 16.5 ms,
  TimerCamera-X projection at 4:3. Sensors are ideal; the gait uses simulator state.
- Rigid bodies, assumed friction 0.8; no supply limits, voltage sag, heat or backlash.

Servo settings: `reference/b91_kinematics.json`. Masses: `reference/b91_mass.json`.
Commented flat, slope and transition gait settings: `reference/b91_gait.toml`.
Select one with `--gait-profile`; selection is manual and does not change with terrain.
Run `python robot.py build` after changes. CAD edits additionally require export
through `cad/ExportB91` in Fusion. No CAD edits are needed for servo settings.

## Results

- Standing, 15 s: stable.
- Flat, 15 s: 0.502 m forward, 0.0226 m sideways; no fall.
- Course, 100 s: 0.495 m forward, 0.227 m sideways; no fall, course incomplete.

No physics warnings in these tests. Reports are in `results/`.
No trained policy is included.
