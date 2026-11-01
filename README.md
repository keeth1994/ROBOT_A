# Robot B9-8

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

## CAD and rebuild

- Current assembly and parts: `cad/B9_8/`
- Rebuild simulation: `python robot.py build`
- After STEP changes:
  1. `python -m pip install -r requirements-cad.txt`
  2. `python cad/export_b98_sim.py`
  3. `python robot.py build`
- Fusion CAD tools: `cad/B98Builder/` (build, validate and access scripts)
