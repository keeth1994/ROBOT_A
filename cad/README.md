# B9-8 CAD

Latest: [complete assembly](B9_8/Robot_B9_8.f3d), [STEP assembly](B9_8/Robot_B9_8.step), [revised components](B9_8/parts), and [changes and checks](B9_8/README.md).

B9-8 adds a bridge-mounted centre support, slightly stronger chassis posts, two removable equipment levels, a centered matrix-board IMU, accessible sensor/camera fasteners, and battery/controller retention. The user-cleaned B9-7-1 chassis is the base; yaw and pitch parts are retained. Static and sampled clearance checks pass within the documented scope. Physical fit and wiring clearance remain to be checked. Simulation now uses the B9-8 STEP assembly.

The source, measured interfaces and validation scope are documented in [B9-8](B9_8/README.md). Export with `python cad/export_b98_sim.py` (requires `requirements-cad.txt`), then run `python robot.py build`. The preserved B9-1 Fusion exporter and assets are historical.
