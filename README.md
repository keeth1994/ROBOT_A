# Robot B10_1

- MuJoCo simulation
- PC dashboard
- Servo calibration
- ESP32 firmware

## Setup

- Python 3.12+ with Tkinter
- Run commands from the repository root

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt -r servo_calibration/requirements.txt
```

## Run

| Task | Command |
|---|---|
| Dashboard | `python dashboard/run.py` |
| Servo calibration | `python servo_calibration/gui.py` |
| Course simulation | `python robot.py walk --ramp` or `.\start.ps1` |
| Simulation options | `python robot.py walk --help` |
| Checks | `python robot.py check all` |

- Dashboard: http://127.0.0.1:8765
- Dashboard gait: `dashboard/walking.toml` (no camera/IMU feedback)
- Simulation gait: `reference/b91_gait.toml`
- Generated reports and recordings: `results/` (ignored by Git)

## Calibration and firmware

1. Upload `servo_calibration/esp32_calibration/esp32_calibration.ino`.
2. Calibrate using the GUI; measurements save to `servo_calibration/measurements.json`.
3. Export the controller header using the GUI or `python servo_calibration/export_controller.py`.
4. Upload `firmware/servo_camera/controller_esp32/controller_esp32.ino` with its accompanying headers.

- Arduino libraries: Adafruit PWM Servo Driver and SparkFun BMI270
- Camera firmware: `firmware/servo_camera/timer_camera/`
- Wi-Fi: sketch `wifi_config.h` files configure connection to the PC's 2.4 GHz hotspot
- Yaw angles: -45 to +45 degrees
- Pitch angles: -90 to +45 degrees; +45 means foot down
- Stop: disables PWM; servo power remains on

### Experimental first-crest gait

`python dashboard/run.py --ramp-only --crest-wave --adaptive-ramp-speed`

The opt-in crest wave lifts, returns and plants one foot at a time, with intervals
where all four feet are commanded down. It uses smaller yaw strokes and less lift
than the diagonal crest gait: duty 0.8, yaw amplitude 0.55, lift 0.25, stance 0,
inward yaw bias -0.25 radians. Onboard pitch triggers the first-crest transition;
established camera/IMU guidance is retained. The crest uses the requested speed
(65% planned for the next trial); adaptive speed is automatically enabled for the
wave and reduces speed to 50% when roll, heading error or lateral drift increases.
It restores requested speed after one second of stable alignment. The original
gait resumes on descent. `--crest-assist` retains the
previous diagonal crest gait for comparison. Neither option starts motion by itself.

Upload the servo-controller sketch and accompanying headers before using
`--crest-wave`; the dashboard requires the new `crestWaveGait` status capability
before starting the climb. Joint limits, stale-data pauses, heartbeat stop,
12-degree ramp roll limit and 40-second crest timeout remain in effect.

Ramp profiles now use gentler steering: heading gain 1.2 instead of 3.0 and
maximum turn correction 0.2 instead of 0.5 (a 60% reduction). Sideways offset
uses the strafe term without adding a turn request. Heading error over 12 degrees
stops the run for recentering. The controller must report `boundedSteering` before
the dashboard starts a ramp run, so older firmware cannot silently ignore the cap.
This revision follows a reported hard-turn fall; the exact run is unconfirmed,
and the change has not been physically tested.

Offline comparison: `python -m robot_b91.crest_experiment --build --compare --seconds 40`.
This creates separate results under `results/ramp_trials/crest_wave_sim/`, with
bare plastic feet, restored camera mass and a sharp crest. The selected wave
crossed 10/18 cases versus 6/18 for the previous crest gait at 50% speed. Two cases
per gait became unstable during neutral settling before walking; excluding those,
the counts are 10/16 and 6/16. These are deterministic local crest tests, not full
ramp runs or measured success probabilities. Starts 4 cm before the peak still
failed with the wave; it is not a general replacement for the climbing gait.
Those comparisons predate the gentler steering revision above.
Slope, dimensions, friction and contact softness are assumptions. Softness uses
[MuJoCo contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters),
not a calibrated deformation model of the foam. The new gait has not yet been
uploaded or physically tested.

### Experimental sideways steps

The dashboard's **Side step left/right** controls move sideways while the IMU
holds the starting heading. They require the updated servo-controller firmware
(`sidestepGait` capability), a fresh IMU and a level stance. Each command performs
one cycle, limited by the ESP32 itself, at up to 65% speed. The feet finish down;
outputs then turn off. Stale observations, excessive tilt/heading error, lost
heartbeat and a 12-second dashboard timeout stop the trial. These controls are
for an initial floor test; automatic ramp recentering is not enabled.

Front and rear legs sweep in opposing directions, with no forward stride and
no lateral-position-to-turn correction. The single-foot lift/return/plant
sequence is retained. `python -m robot_b91.sidestep_experiment` checks both
directions with three assumed friction levels using bare-foot geometry and
restored camera mass. In six simulated one-cycle trials at 65%, the selected
gait moved 11–24 mm sideways with up to 6.9 mm fore/aft drift and 2.5 degrees
maximum heading deviation. These are flat-floor simulation results, not a
guarantee of perfectly straight sideways motion or physical ramp performance.
Neither this gait nor the crest wave has been uploaded yet.

## CAD and rebuild

- Current assembly and parts: `cad/B10_1/`
- Rebuild simulation: `python robot.py build`
- After STEP changes:
  1. `python -m pip install -r requirements-cad.txt`
  2. `python cad/export_b10_sim.py`
  3. `python robot.py build`
