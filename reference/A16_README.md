# A16 MuJoCo transfer

Run `.venv\Scripts\python.exe run_robot_a16.py --physics` or VS Code task **Start robot A16**. The default build task now targets A16. A15 is preserved.

This model uses the final visible A16 CAD assembly. The four corner interfaces moved from (+/-115,+/-100) mm to (+/-42.5,+/-42.5) mm. Existing leg joint axes receive the same translations. Sensor packages on fixed hip housings belong to the platform link. Battery and electronics centres of mass use their new CAD positions.

Printed-part masses use P2S 0.4 mm, 0.20 mm layers, 4 walls, 25% gyroid, assumed PLA density 1.24 g/cm3. Original leg parts reuse their verified A15 slices; new centre and sensor saddles use A16 slices. 24 servo envelopes remain 44 g each; wheels remain 40 g per rotating assembly. Combined electronics/battery allowance remains 200 g including the second PWM reservation and is not measured. Steel axle classification is corrected.

The dual hip servo model is ideal synchronized torque summation. Maximum instantaneous torque remains 0.53668 Nm per inner hip and 0.26834 Nm per other actuated axis. Torque decreases with speed in the existing controller; free speed remains 315.789 deg/s. No thermal, supply sag or gearbox backlash calibration is provided.

Collision shapes are simplified envelopes, including compact centre walls, underslung battery and fixed sensor packages. They do not validate detailed clearance or structural strength. Inertias approximate each part by its bounding box, positioned at CAD centre of mass. This is a mechanics test environment; no trained walking/skating controller is supplied.
