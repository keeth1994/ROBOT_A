# Ramp traversal

The results below describe the historical B9-1 model. The default simulation now
uses B9-8; see [current simulation notes](DESIGN_B91_SIM.md) for its validation.

The `course` profile traverses the existing up/down/up/platform/down course with
the original CAD geometry, 1.163 kg estimated mass, 0.8 nominal friction, 5 V
servo torque/speed limits, +/-45 degree joints and 50 Hz servo commands.
It applies only joint motor torques. There are no imposed body forces, terrain
changes, extra joints or resets during the traversal.

## What changed

The original flat gait slid and drifted off the first incline. Merely slowing it
to the existing crawl profiles did not fix the approach. The new gait has:

- A -0.3 radian (17.2 degree) resting pitch on each rocker foot. This changes the
  contact geometry; it does not increase the assumed coefficient of friction.
- A swing that reaches, and briefly stays at, the existing -45 degree limit.
  All eight joint commands are explicitly bounded before being sent to motors.
- Separate heading and sideways corrections. Left/right stride differences
  correct heading; front/rear stride differences correct lateral drift. The old
  steering tried to correct position only by turning, which could put feet off
  the narrow ramp during transitions.

The one-second diagonal gait continues across each transition; it does not need
a scripted rotation or advance knowledge of the positions of crests and valleys.
The controller uses the course heading and centerline, not surface heights or
contact information. The scorer observes contacts but does not control motion.

The pure calculation is in `robot_b91/gait.py`. Inputs are elapsed time, gait
phase time, heading error and lateral error; outputs are eight joint angles in
the order FL yaw/pitch, RL yaw/pitch, FR yaw/pitch, RR yaw/pitch. The simulation
adapter in `robot_b91/run.py` supplies ideal pose feedback and models the servos.

## Run and reproduce

```powershell
.\.venv\Scripts\python.exe robot.py walk --ramp
.\.venv\Scripts\python.exe robot.py walk --ramp --test
.\.venv\Scripts\python.exe robot.py walk --ramp --video results/b91_course.mp4
.\.venv\Scripts\python.exe robot.py check all
.\.venv\Scripts\python.exe -m tests.test_course --robust
```

`--ramp` selects `course` unless another profile or parameter file is explicitly
requested. Flat walking retains its existing default. Tests and interactive runs
stop/pause after a verified completion; the default course timeout is 120 seconds.
Recording runs the same evaluation headlessly, saves its JSON report, and writes
an MP4 at 2x playback speed. Yellow lines in the video are simulated IR rays.

Completion requires each foot to contact all five panels in order, the body to
stay within the course width, no supporting floor contacts beside the ramp,
no fall or physics warnings, and all rubber feet to clear the exit. It cannot be
earned by walking beside the ramp or touching only its first panel. The checker
allows nonfoot contacts, consistent with the traversal-only requirement, and
reports their frequency separately.

Original failure report: `results/b91_ramp_baseline.json` (0.495 m in 100 seconds).
Nominal result in `results/b91_ramp.json`: **63.406 s**, **3.554 m** forward,
**21.60 degrees** maximum tilt, no warnings or bypass. Nonfoot terrain contacts
occurred during 0.335% of evaluated timesteps. Flat walking still advances
0.502 m in 15 seconds without a fall.

`tests.test_course --robust` saves `results/b91_course_validation.json`. It runs
the nominal case plus initial lateral offsets of +/-10 mm, initial headings of
+/-0.05 radian (2.9 degrees), and friction values of 0.7 and 0.9. These perturbations
are separate trials; the nominal model remains at friction 0.8. All seven runs
complete in 63.4-78.3 seconds. Tests assert completion and no bypass/fall/warnings.
The +0.05 radian start-heading trial spends 18.1% of evaluated timesteps in nonfoot
terrain contact while getting over the first crest; successful traversal does not
mean scrape-free motion.
This is a small sensitivity test, not a guarantee over all disturbances or surfaces.

## What remains for the physical robot

This result establishes traversal in the supplied simulation, not physical
autonomy. An [ESP32 firmware prototype](../firmware/README.md) now implements
the gait, IMU attitude estimation and USB operator steering. Servo channel
mapping and measured PWM zeroes/mounting signs remain unconfigured. Autonomous
localization is not implemented. Nothing has been flashed or sent to hardware.

The wiring diagram and the [linked IMU module](https://no.rs-online.com/web/p/sensor-development-tools/2494029)
identify a BMI270 accelerometer/gyroscope. An attitude estimate can supply
heading relative to the start, with drift; the IMU alone does not supply the
accurate lateral position used here. The simulated IR readings and camera are
not consumed by this gait. An onboard solution still needs tested heading and
cross-track estimates, potentially using the existing camera, or operator
steering. The firmware provides the operator-steering path; autonomous lateral
localization is not implemented, and the attitude estimate is not hardware-validated.

For hardware integration, call the gait at 50 Hz with measured/commanded path
errors, calibrate each output joint angle to its existing servo channel and PWM
range, and validate stance and direction before a course attempt. The internal
PD torque model belongs to the simulator; actual hobby servos receive calibrated
position commands. The available torque, mass distribution, tread friction,
power supply and actual ramp dimensions must also match the modeled assumptions
closely enough. The saved course dimensions and component masses remain provisional.
