# B3 simulation: narrow feet and supplied sensors

## Model

Native Fusion and STEP exports, CAD-derived visual meshes, approximate inertias and contact shapes are rebuilt from the same B3 geometry. Runner length is 42 mm, flat sole 8 mm, inward tread radius 12 mm, outward tread radius 22 mm, PLA thickness 3 mm and radial section 5 mm. Pitch axle is 60 mm above the neutral sole. Inward reach is deliberately shorter (18 mm versus 24 mm outward).

Estimated total mass: 1.1094 kg. Solid PLA: 284.2 g. Sensor mass additions and the retained electronics/fastener allowances are itemized in `reference/robot_b3_manifest.json`. These are not measured or sliced masses.

Original Parallax 900-00005 servos remain: pitch stall limit 0.26834 Nm, yaw output 0.15206 Nm after 1.5 speed-increasing gears and assumed 85% efficiency. Torque decreases with motoring speed; no thermal, battery-sag or backlash model. Pitch and yaw are limited to +/-45 degrees. Full requested 270-degree yaw is not clearance-certified.

## Course and controller

The photo-inspired ramp has a 300 mm width and five 600 mm horizontal sections. Its provisional height profile is 0 -> 160 -> 0 -> 200 -> 200 -> 0 mm: first climb, first descent into a valley, second climb, raised flat platform, final descent. Thin 6 mm rigid panels have black surfaces, exposed wood-coloured edges and simplified supports. The first slopes are about +/-14.9 degrees and the second climb/descent +/-18.4 degrees. The checkerboard floor remains for visual tracking. Dimensions are assumptions pending measurements; flex and grip are not calibrated.

An independently positioned open zigzag corridor has a provisional 400 mm clear width, 180 mm wall height and 12 mm wall thickness. The settings file exposes both obstacles' positions, rotations and dimensions. There is no ceiling because the photograph shows open walls. The existing controller is unchanged and cannot be assumed to navigate from the ramp into this corridor.

The programmed gait acts through torque/speed-limited motors, without teleportation, base forces or terrain-specific choreography. Ideal simulated heading and lateral feedback remain. After reducing the feet, yaw sweep direction was changed to +0.45 rad; no other gait gains were changed. An earlier unchanged-gait comparison is retained separately and is not the current run.

## Previous-geometry tests (not rerun)

The programmed-gait tests below have not been rerun for the short-sole/rounded-toe revision; new RL experiments on that geometry are documented in `RL_B3.md` and `RL_CURRICULUM.md`. On the preceding foot geometry, the 100 s run advanced 0.499 m, contacted first_climb, floor, and did not complete the ramp. Detected fall: False; warnings: 0; maximum tilt 16.06 degrees. Those previous-geometry results remain in `results/b3_course_test.json` and the replay. The pass criterion requires contact with all five ramp surfaces, clearance of the ramp exit, no fall, no non-foot contact and no ramp-width violation. It assesses the ramp only, not successful corridor navigation. Geometry and sensor validation do not prove traversal capability.

## Sensors

Three Sharp GP2Y0A41SK0F rays provide left, right and downward-forward range observations. Outside 4-30 cm is unknown. MC6470 accelerometer and magnetometer channels are present, with no invented gyro. The TimerCamera-X is represented by a forward RGB camera. `robot.py sensors` logs observations and captures a camera view. `robot.py check sensors` checks wall distances, ground-ray orientation, invalid ranges and channel presence. These tests passed on the previous foot geometry; analogue noise, surface reflectivity, magnetic interference and perception software remain unmodelled. The gait does not yet use these readings. See `SENSORS_B3.md`.

## CAD checks

The audit has 59 recorded overlap cases; inspect `cad/B3_clearance_audit.json` for scope and locations. Own pitch is sampled at 5-degree intervals, FL neutral-pitch yaw at 15 degrees, sensor-vs-leg poses at yaw/pitch -45/0/+45 for all four legs, and stationary sensor packaging is checked. Combined continuous poses, neighbouring moving legs, wiring and strength are not certified. Sensor brackets and other small mounting details contribute mass but are not fully represented by MuJoCo contact geometry.

## Files

`cad/` holds native exports and previews. `models/` holds flat and course XML plus visual meshes. `reference/` holds geometry, sensor and gait configuration. `results/` holds previous-geometry metrics, replay, sensor log and camera view.
