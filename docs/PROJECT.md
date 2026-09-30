# B3 project details

## Current design

- 42 mm runners, reduced from 64 mm, with 8 mm flat soles; inward tread radius 12 mm and outward radius 22 mm.
- Platform-facing reach 18 mm; outward reach 24 mm. PLA rails 3 mm thick; rubber strips 6 mm wide. Servo support spacing retained.
- Neutral bounds 241.2 x 241.2 x 106.0 mm. Estimated mass 1.1094 kg (not sliced or measured).
- Three supplied Sharp IR sensors, one supplied accelerometer/magnetometer board, one supplied ESP32 camera, with open cradles. See [sensor details](SENSORS_B3.md).

The MC6470 board has **no gyroscope**. Sensor shapes are packaging envelopes and need physical fit verification.

## Start and rebuild

In VS Code, **Ctrl+Shift+B** runs the photo-inspired double ramp, valley, raised platform and open zigzag corridor. The viewer remains open after 100 simulated seconds. `start.ps1` does the same. The task **B3 - inspect project sensors** writes a sensor log and camera image.

```powershell
.\.venv\Scripts\python.exe robot.py build
.\.venv\Scripts\python.exe robot.py check sensors
.\.venv\Scripts\python.exe robot.py check-course --flat --seconds 25
.\.venv\Scripts\python.exe robot.py check-course --seconds 100
.\.venv\Scripts\python.exe robot.py record-course --seconds 100
```

For a fresh environment install `requirements.txt` in `.venv`; `requirements-lock.txt` records the pinned environment. To regenerate native CAD, register `cad/B3Builder` in Fusion Scripts and Add-Ins, run B3Builder, then rebuild MuJoCo. `reference/b3_foot_geometry.json`, `b3_sensors.json` and `b3_course_settings.json` hold the geometry settings.

## Validation

The course follows the supplied photo. The top-level programmed-gait results predate the short-sole/rounded-toe update. New reinforcement-learning runs use the current geometry; see `RL_CURRICULUM.md` and the experiment subdirectories. Ramp completion and corridor navigation are separate: the existing controller does not navigate the corridor.

The selected yaw sweep is +0.45 rad; the previous -0.45 setting initially moved the smaller feet backward. Motor torque limits are unchanged. This is a physics-driven programmed gait with ideal pose feedback, not learned navigation. No base motion is imposed and no terrain-specific motion sequence is used. Sensor readings are available but do not yet control locomotion.

See [test details](DESIGN_B3_SIM.md), `results/b3_course_test.json` and `results/b3_flat_up_down.mp4`. CAD clearance checks are sampled and do not establish continuous clearance or printed strength. Previous versions are backed up outside this repo.

## Photo-inspired obstacle layout

Edit `reference/b3_course_settings.json` and run `robot.py build`. The 300 mm-wide ramp is five joined rigid planks: rise to 160 mm, descend to floor level, rise to 200 mm, cross a raised flat section, then descend to the floor. Each section has a provisional 600 mm horizontal length. These are layout assumptions, not measurements from the photo. The resulting slopes are about 14.9 and 18.4 degrees.

The separate zigzag corridor is open at the top, matching the photo. Its provisional clear width is 400 mm and wall height 180 mm. Change its `clear_width_m`, `centerline_xy_m`, `origin_xy_m` and `yaw_deg` independently of the ramp. The ramp has its own `origin_xy_m`, `yaw_deg` and `profile_xz_m`. Setting `corridor.enabled` false hides it. No scripted turn, waypoint path or automatic course traversal was added.

Course reference: `reference/obstacle_course.jpg`. Run `robot.py check terrain` to check panel heights, the valley, corridor width and independently moved/rotated configurations.

## Reinforcement learning

Flat-ground PPO training and a learned-policy viewer are now available. See [RL setup and results](RL_B3.md). Run `robot.py watch` to watch the learned policy; `robot.py walk` remains the programmed gait. New RL results live in `results/rl_flat/`, separately from the historical gait tests.

## Straight-line and gentle-ramp lessons

See [curriculum setup](RL_CURRICULUM.md) for the refined straight-line policy and separate 3/5-degree training ramps. These are separate from the full photographed obstacle course. VS Code has dedicated viewing tasks for each lesson.

## Group meeting demos

Double-click MEETING_DEMOS.cmd for normal walking, learned flat walking, and your successful 3-degree RL ramp. See [meeting guide](MEETING_GUIDE.md). Labelled video backups are in results/meeting.

