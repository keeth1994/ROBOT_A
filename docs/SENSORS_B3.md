# Supplied project sensors on B3

Selection is based on page 9 (Project components) of `Final project introduction 2026.pdf`. Sensor quantities are a proposed layout; the document does not state available quantities.

| Part | Quantity | Location / purpose |
|---|---:|---|
| Sharp GP2Y0A41SK0F, RS 666-6568 | 3 | One left, one right, one front looking down 30 degrees. Wall clearance and ground profile. |
| MIKROE-4228 / MC6470, RS 249-4068 | 1 | Rigid rear deck cradle. Acceleration and magnetic field. **No gyroscope.** |
| M5Stack TimerCamera-X U082-X, RS 230-5140 | 1 | Forward-facing camera in open deck cradle. RGB observation; no depth output. |

The suggested VL53L5CX, VL53L1X and BMI270 are not in the project list and are not fitted. The listed Ohmite FSR02CE is a 622.3 mm strip, so it is excluded instead of being misrepresented as a small foot pad. No servo position encoders are added.

## Geometry and mass

Sharp packages use the manufacturer's 29.5 x 13 x 13.5 mm envelope and 3.6 g mass. The camera uses 48 x 24 x 15 mm and 14 g. The MC6470 board footprint is 28.6 x 25.4 mm; a 10 mm installed-height envelope and the manufacturer's listed 17 g weight are conservative provisional allowances, to be checked physically. The old 3 g generic IMU reservation is replaced. These are package envelopes, not exact connector/optical-window models.

Open PLA cradles use adhesive pads/cable-tie slots. Body clearance and connector access need physical checks before printing. Sensor cases have dedicated simulation mass and collision boxes. Small cradle collision details remain omitted from MuJoCo, but are included in CAD checks and mass. The 90 g controller/electronics and 80 g miscellaneous allowances remain separate; neither is a finalized bill of materials.

## Simulation interface

`robot.py sensors` exposes three range readings, 3-axis specific acceleration and magnetic field, plus a `project_camera` RGB view. Run:

```powershell
.\.venv\Scripts\python.exe robot.py check sensors
.\.venv\Scripts\python.exe robot.py sensors --ramp --seconds 3
```

Outputs are `results/b3_sensor_test.json`, `results/b3_sensor_samples.json` and `results/b3_camera_view.png`. Range observations outside 0.04-0.30 m are marked unknown. A missing return is not a proven drop-off. The rays are ideal point samples, not models of Sharp analogue response, beam footprint, dark-rubber reflectance or electrical noise. Magnetometer motor interference is not simulated. Camera FOV uses the published 66.5-degree diagonal angle with an assumed 4:3 image.

The locomotion controller still uses ideal simulated heading/lateral position. Adding sensors does not implement autonomous perception or navigation, and the new sensor data do not secretly replace that feedback. Ground readings change with robot tilt; the listed accelerometer/magnetometer cannot provide gyro rates.

## Wiring notes

Sharp sensors need a regulated 4.5-5.5 V supply, local bypass capacitors and calibrated analogue inputs. Verify ADC voltage protection/dividers for the selected ESP32. The Click board uses 3.3 V I2C. Camera power/communications follow the TimerCamera-X documentation. Measure the installed modules, cable bend space and printed fit before manufacturing.

## Sources

- Project component links: https://no.rs-online.com/web/p/reflective-optical-sensors/6666568 ; https://no.rs-online.com/web/p/sensor-development-tools/2494068 ; https://no.rs-online.com/web/p/development-tool-accessories/2305140
- Sharp datasheet: https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf
- MC6470 board: https://www.mikroe.com/6dof-imu-13-click
- Camera: https://docs.m5stack.com/en/unit/timercam_x
- Ohmite force sensor: https://docs.rs-online.com/8bfb/A700000007054155.pdf
