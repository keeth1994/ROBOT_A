# B9-8 serviceable equipment assembly

The current mechanical assembly is [Robot_B9_8.f3d](Robot_B9_8.f3d), with a [STEP copy](Robot_B9_8.step) and standalone revised components in [parts](parts). It starts from the user-cleaned B9-7-1 chassis and retains its yaw and pitch geometry.

## Changes

- Separate bridge-mounted centre support: 4.2 mm shaft wall, 2 mm thicker than the reinforced chassis post wall. Existing post stems gain 0.4 mm radially; their holes and mounting positions remain.
- Two removable equipment levels. The lower deck carries the battery, SERVO2, regulator and sensor/camera mounts. Four flared pillars support the matrix platform. All new structural fasteners are M3.
- Matrix board: 76.5 x 100.1 x 2 mm, with 2.5 mm clearance below for solder and wires. Its four M3 holes are 5 mm from each corner. The IMU chip is at chassis X=Y=0 using the measured 51 mm top and 23.5 mm right offsets. Board TOP points toward robot FRONT (+X), and the ESP32 occupies the +Y side.
- Upper platform screw heads are fully recessed; install the empty platform before installing the board. Side-entry nut pockets allow assembly without reaching underneath the board.
- Camera: 32 mm ear spacing and 8 mm ear depth, retained by two M3x12 screws in separate rear nut-capture mounts. Conservative housing allowance is 48.3 x 24 x 15 mm. Lens faces +X with bottom connector access.
- Sensor-to-bracket interfaces remain unchanged. New exposed tabs allow M3 attachment to the lower deck. Side sensors move 4 mm forward; the ground sensor moves 16 mm forward with its original angle.
- Battery cradle accepts the measured 111.2 x 35 x 15 mm pack, with both leads leaving the same open end. Separate SERVO2 and regulator supports retain connector space and strap routes.

## Checks and limits

[Static checks](check.json) report one solid per revised part and no tested overlaps above 0.1 mm3. [Tool-access checks](access_report.json) cover 16 exposed mount screws with a 5.7 mm driver envelope, new fasteners versus equipment, and an approximate 60-degree forward camera cone.

[Sampled motion checks](motion_report.json) contain 63,196 comparisons for all four legs: yaw -45 to +45 degrees in 15-degree increments, combined with pitch -45, -30, -15, 0, +15 and +25 degrees. No overlaps with the revised fixed structure/equipment were found in that grid. This is not continuous motion, leg-to-leg, cable, strength, or physical fit validation. Existing movement limits still apply.

The motion report records both evaluated-candidate and final STEP hashes. Their geometry is byte-identical: only the STEP filename and timestamp fields differ. Equipment models are clearance envelopes. Actual connector plugs, solder tails, straps and wire bends require a physical assembly check. New plain M3 nut pockets allow 5.8 mm across flats and 2.6 mm thickness; assumed nuts are 5.5 x 2.4 mm.

## CAD maintenance

The Fusion source is [B98Builder](../B98Builder). Run build_b98.py within Fusion to create a candidate from [the preserved user assembly base](source/User_assembly_base.step). Then run validate_b98.py and access_b98.py on that active candidate. These scripts require Fusion's adsk API and are not ordinary terminal programs. Inspect the reports before replacing the final assembly.

B9-8 is the default MuJoCo model. `python cad/export_b98_sim.py` exports this STEP assembly, and `python robot.py build` regenerates the simulation. See [simulation notes](../../docs/DESIGN_B91_SIM.md) for provisional masses and validation. Historical B9-1 results do not describe this assembly.
