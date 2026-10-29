# ESP32 firmware prototype

This implements the simulated course gait on the existing ESP32, Module13.2
Servo2 and BMI270. It is **compiled and host-tested, not tested on a physical
robot**. The operator supplies lateral steering over USB. Autonomous camera/IR
navigation and wireless control are not implemented.

The default configuration cannot arm: measured PWM values are deliberately
unset. Servo channels follow the user-supplied wiring below. No firmware has
been uploaded by this change.

## Build and check

From the repository root in PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -r firmware/requirements.txt
.\.venv\Scripts\python.exe -m platformio run --project-dir firmware
.\.venv\Scripts\python.exe firmware/check.py
```

PlatformIO pins the ESP32 platform and SparkFun BMI270 library. Its downloads
stay under `.tools/platformio`; build files stay in `firmware/.pio`. The native
check uses an available g++/clang++ compiler (or Zig as a fallback) and compares 1,003 firmware trajectories against
`robot_b91/gait.py`, as well as testing output calibration and the watchdog.
The existing simulation environment supplies NumPy for that comparison.

The build target is `esp32dev`, matching the classic ESP32 development-board
family in the [linked ESP32-DEV-38P listing](https://no.rs-online.com/web/p/microcontroller-development-tools/2863991).
Confirm the marking on the actual board before uploading. The upload command,
after commissioning configuration, is:

```powershell
.\.venv\Scripts\python.exe -m platformio run --project-dir firmware --target upload --upload-port COM3
```

Replace `COM3` with the actual port. This is an instruction, not an upload that
has already been performed.

## Wiring and calibration configuration

Edit `include/hardware_config.h`. GPIO21/SDA and GPIO22/SCL are proposed assignments;
the supplied diagram does not specify ESP32 GPIO numbers. The default I2C
addresses are 0x40 for Servo2 and 0x68 for BMI270; match the board switches/jumpers.
Keep the existing common ground and 3.3 V logic connections from the diagram.

The [Servo2 documentation](https://docs.m5stack.com/en/module/Module13.2_Servo2)
identifies a PCA9685 and two separately powered groups of eight outputs. A
user-supplied channel arrangement distributes this robot's eight servos across
both groups:

| Joint | Servo2 channel | Servo |
|---|---:|---|
| FL yaw | 8 | HS-475HB |
| FL pitch | 3 | Parallax |
| RL yaw | 1 | MG995 |
| RL pitch | 2 | Parallax |
| FR yaw | 0 | HS-475HB |
| FR pitch | 11 | Parallax |
| RR yaw | 9 | MG995 |
| RR pitch | 10 | Parallax |

Yaw calibration uses **output-joint angles** -45, 0 and +45 degrees relative
to CAD neutral: `{channel, minus45us, zeroUs, plus45us}`.

Pitch uses the operator's raised zero, 45 degrees above CAD neutral. Measure
five points and enter `{channel, minus45us, zeroUs, plus45us, plus90us, minus90us}`.
Use the exporter to preserve this C++ field order (the last two are +90, -90):

| Pitch calibration angle | CAD/simulation angle | Physical reference |
|---:|---:|---|
| -90° | -135° | Foot 45° above horizontal outward |
| -45° | -90° | Foot horizontal outward |
| 0° | -45° | Halfway between CAD pose and horizontal |
| +45° | 0° | Original CAD pose, foot down |
| +90° | +45° | Original lower travel endpoint |

`servo_calibration/gui.py` records these points and exports the array. Existing
three-point measurements remain usable; add the -90° and +90° points for each
pitch joint (eight extra measurements in total). The previous `servo_table.h`
must be exported again after all extra points are measured. Firmware validation
rejects incomplete pitch calibration.
Never extrapolate endpoint pulses or force a joint against a mechanical stop.
The enlarged pitch envelope is -135° to +45° in the CAD frame, equivalent to
-90° to +90° around raised zero (180° total). The walking
waveform keeps its previous -45° to +45° limits; extending the envelope does
not automatically change the gait or add a raise-legs operator command.

Values can increase or decrease with joint angle; each measured interval has
its own linear scale. Calibration incorporates mounting direction and yaw
gearing, so do not multiply by 1.5 again. Never substitute unmeasured pulse
values just to pass validation. New pitch travel requires physical clearance
checks; it is not established by changing software limits.

For an unloaded bench setup, `benchCalibration=true` permits `PULSE channel us`.
It operates one channel at a time within `benchMinUs`/`benchMaxUs` and expires
after two seconds. Start with the horn disconnected to identify neutral and
direction, then check assembled clearance while the body is supported. Expand
the configured bench pulse range only as the physical mechanism permits.
`OFF` cancels it immediately. Disable bench mode after recording calibration.

Set `imuRotation` to the measured sensor-to-body rotation: body X forward,
Y left, Z up. Check printed roll/pitch/yaw against real movements. The matrix
must be a right-handed rotation. When these assignments and the measured PWM
table are correct, set `calibrated=true` and rebuild. `ARM` then commands the
course resting stance; hobby servos provide no initial-position measurement to
this firmware, so the first arming must be done with the robot supported.

## Operator control

```powershell
.\.venv\Scripts\python.exe firmware/teleop.py COM3
```

The Windows console never arms or starts walking automatically:

1. Press **z** with the robot stationary to collect a two-second gyro bias.
2. Press **r** to arm in the resting stance.
3. Press **w** to walk. Use **a/d** to turn left/right and **j/l** to steer
   sideways left/right. Repeating/holding a key keeps that steering request active.
4. **Space** or **s** pauses and holds the current joint commands. **x** disables
   PWM. **q** exits with a hold command; it does not remove servo power.

The console sends a heartbeat every 100 ms. No valid `DRIVE` for 500 ms freezes
the gait at its current commands. A new heartbeat does not resume it; **w** is
required. Holding a mid-step pose is not a guarantee of static balance on a ramp.
This USB console is intended for initial tests; cable routing must not supply
support or pulling force when evaluating physical traversal.

Serial protocol at 115200 baud, one newline-terminated command per line:

| Command | Meaning |
|---|---|
| `STATUS` | Read readiness and attitude in radians |
| `ZERO` | Stationary gyro calibration, only while disarmed |
| `ARM` | Validate configuration and command resting stance |
| `DRIVE turn side` | Refresh heartbeat; both values in [-1,1], positive is left |
| `WALK` | Start/resume after ARM and a fresh DRIVE |
| `STOP` | Hold current targets |
| `OFF` | Disable PWM, retaining configuration |
| `PULSE channel us` | Timed, single-channel bench calibration |

The turn command changes the desired heading at up to 0.5 rad/s. The side command
directly requests front/rear stride correction; it is **not an estimate of the
robot's lateral position**. Roll/pitch come from a complementary accelerometer/
gyro estimate; yaw integrates tilt-compensated gyro rates with a stationary
bias correction. Yaw still drifts, so the operator must correct the route.
IMU sampling uses the [SparkFun BMI270 library](https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library).

Servo updates are 50 Hz; walking commands retain the original +/-45 degree
limits in the CAD frame. Pitch PWM mapping supports the extended -135 to +45
degree envelope with five measured calibration points. The
[PCA9685 register driver](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
checks I2C acknowledgements, configures the PWM prescaler, and disables its
ALLCALL response. Gyro/accelerometer data staleness, excessive tilt and failed
servo writes latch a fault and attempt to disable PWM. A broken I2C connection
can prevent that write: the existing PCA9685 can continue its previous pulses.
The physical main switch remains the way to cut power; this code adds no
hardware emergency-stop circuit.

The IR sensors and TimerCamera-X are not used by this prototype. Real autonomous
traversal still needs calibrated localization/vision and physical validation;
the simulated 63.4-second result does not establish performance of this manually
steered firmware, its attitude estimate, or the calibrated real servos.
