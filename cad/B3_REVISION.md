# B3 - narrow asymmetric feet and project sensors

Each runner is now 42 mm long instead of 64 mm. The pitch axis is at local x=38 mm: the platform-facing end stops at x=20 (18 mm reach) while the outward end reaches x=62 (24 mm reach). The flat sole is shortened from 18 to 8 mm. The inward tread keeps its 12 mm radius; the outward tread uses a gentler 22 mm radius, centred 10 mm higher so the sole remains level. PLA rail thickness is reduced from 4 to 3 mm, with a 5 mm radial section; tread width is 6 mm instead of 8 mm. Rail spacing is retained to accommodate the servo and opposite support. Ground distance from the pitch axis is 60 mm.

Mass is regenerated from CAD in `B3_size_mass.json` and `reference/robot_b3_manifest.json`; simulation results have not been rerun for this revision.

Three supplied Sharp GP2Y0A41SK0F sensors face left, right and forward/downward. The supplied TimerCamera-X faces forward. The generic IMU placeholder is replaced by a MIKROE-4228 envelope: this is accelerometer plus magnetometer, with no gyro. Open cradles provide adhesive/cable-tie mounting. Sensor envelopes, connectors and installed board height require physical fit checks; these are not detailed vendor CAD models.

Own pitch is sampled every 5 degrees, FL neutral-pitch yaw every 15 degrees, and all four legs versus sensor packages/cradles at yaw/pitch -45, 0, +45 degrees. Static sensor-package checks are also included. These checks do not certify continuous combined-pose clearance, neighbouring legs, wiring or printed strength.

The canonical F3D, STEP, previews and source exports are rebuilt together. The prior revision is archived outside this repository at `C:/PROJECTS/ROBOT_ARCHIVE/B3_before_round_outer_20260929`. See `SENSORS_B3.md` and `DESIGN_B3_SIM.md` for sensor assumptions and previous-geometry tests.
