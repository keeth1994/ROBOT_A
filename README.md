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

Current physical candidate:
`python dashboard/run.py --ramp-only --crest-assist --adaptive-ramp-speed`.
Trial 45 completed the full ramp automatically in 226.46 seconds on 2026-11-19.
It started at the entrance, crossed both crests, recovered camera guidance,
completed the exit-clearance step and stopped without repositioning or a
software restart. The user confirmed all four feet were on the floor beyond
the ramp. Requested speed was 65%, with the approach capped at 50% and adaptive
slowing: recorded time was approximately 206 seconds at 50% and 20 seconds at
65%, including settling and holds. The first-crest phase was mostly at 50%.
Peak recorded control roll was 8.68 degrees. This is one verified full automatic
run; repeatability and different exam-course geometry remain unverified.
Results and source hashes are in `results/ramp_trials/attempt_45_full_approach_heading/`.

This uses the earlier diagonal crest gait with gentle steering. Assisted trial
40 cleared the first ridge, confirmed by the user, and changed to the descent
gait after 7.7 seconds. It stopped at the 35-second trial limit after losing
camera guidance on the descent; this was not a full entrance-to-exit run.
Full entrance trial 41 stopped on the first incline at the 12-degree heading
guard after 34.03 seconds, before either crest gait or new rear search engaged.
It spent about 29 seconds at50% and five at65%, including startup. The current
candidate now caps the first approach at50% to avoid brief accelerations during
that climb; the pitch-triggered crest and subsequent descent may use the
requested65%, with the existing automatic slowing. This speed change still
needs further validation: trial42 held50% while moving but again drifted left
and stopped at the heading guard after36.96 seconds. It reached neither the
crest nor the new rear search view. The user returned it to the entrance.

The next first-climb candidate restores heading gain 3.0 while retaining the
0.2 turn limit and the 50% approach cap. It reaches that same turn limit at
3.8 degrees of heading error instead of 9.5 degrees. Crest and descent retain
gain 1.2, and lateral correction is unchanged. No firmware upload is needed.
Trial 44 completed its 50-second first-climb check without a heading stop or
fall. Estimated heading stayed between -9.54 and +5.50 degrees; peak recorded
control roll was 9.43 degrees. It stopped at the time limit before reaching
the crest. Lateral estimates still varied from -49 to +40 mm, so this is an
encouraging partial test, not evidence that drift is resolved or the full
ramp is reliable.

Floor trial 43 completed one right sidestep at 50%. The user could not identify
a clear direction. Recorded webcam frames show an initial leftward settling
movement followed by a rightward step, with a small net rightward movement;
stationary onboard estimates changed by about 12 mm to the right. This does
not establish repeatable sidestep distance or justify reversing steering.
The MCU confirmed completion and outputs were off. A dashboard race reported
a session error when completion occurred between status and adjustment; the
dashboard now checks fresh completion status without retrying motion.

`python dashboard/run.py --ramp-only --crest-wave --adaptive-ramp-speed`

The opt-in crest wave lifts, returns and plants one foot at a time, with intervals
where all four feet are commanded down. It uses smaller yaw strokes and less lift
than the diagonal crest gait: duty 0.8, front yaw amplitude 0.55, rear stroke at
30% of the front stroke (`rear_stride_scale=0.30`), lift 0.25, stance 0,
inward yaw bias -0.25 radians. Rear support height and stance bias remain unchanged.
Onboard pitch triggers the first-crest transition;
established camera/IMU guidance is retained. The crest uses the requested speed
(65% planned for the next trial); adaptive speed is automatically enabled for the
wave and reduces speed to 50% when roll, heading error or lateral drift increases.
It restores requested speed after one second of stable alignment. The original
gait resumes on descent. `--crest-assist` retains the
previous diagonal crest gait for comparison. Neither option starts motion by itself.

Upload the servo-controller sketch and accompanying headers before using
`--crest-wave`; the dashboard requires `crestWaveGait` and `rearStrideScale`
capabilities before starting the climb. Older firmware cannot silently ignore
the reduced rear stroke. Joint limits, stale-data pauses, heartbeat stop,
12-degree ramp roll limit and 40-second crest timeout remain in effect.

Ramp profiles use gentler steering: heading gain 1.2 except for the experimental
first-climb candidate above, and
maximum turn correction 0.2 instead of 0.5 (a 60% reduction). Sideways offset
uses the strafe term without adding a turn request. Heading error over 12 degrees
stops the run for recentering. The controller must report `boundedSteering` before
the dashboard starts a ramp run, so older firmware cannot silently ignore the cap.
This revision follows a reported hard-turn fall; the exact run is unconfirmed.
Physical trial 30 stayed on the ramp and stopped at the first crest when the
heading estimate exceeded 12 degrees; the IMU also recorded a yaw excursion.

Offline comparison: `python -m robot_b91.crest_experiment --build --compare --seconds 40`.
This creates separate results under `results/ramp_trials/crest_wave_sim/`, with
bare plastic feet, restored camera mass and a sharp crest. The selected wave
with yaw amplitude 0.55 crossed 10/18 cases versus 6/18 for the previous crest gait at 50% speed. Two cases
per gait became unstable during neutral settling before walking; excluding those,
the counts are 10/16 and 6/16. These are deterministic local crest tests, not full
ramp runs or measured success probabilities. Starts 4 cm before the peak still
failed with the wave; it is not a general replacement for the climbing gait.
Those comparisons predate the gentler steering revision above.
Slope, dimensions, friction and contact softness are assumptions. Softness uses
[MuJoCo contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters),
not a calibrated deformation model of the foam. The combined firmware was
uploaded and its capabilities verified on 2026-11-19. In physical trial 30,
the wave engaged at the crest and body pitch reached about 13 degrees downhill
before the heading guard stopped motion. Underside clearance was not confirmed.
Although 65% was requested, adaptive slowing kept most of this run at 50%.
A subsequent candidate shortened equal yaw amplitudes from 0.55 to 0.35 radians. In a separate
12-case comparison per setting using the gentler steering, two speeds and three
assumed friction levels, it crossed six cases without exceeding 12 degrees of
heading, versus two for 0.55. Some shorter-stride cases still drifted or stalled;
this is not a measured success rate. The heading guard remains unchanged.
In assisted physical trial 33, the shorter setting ran for 35 seconds without
reaching a guard: peak roll 2.16 degrees and heading within 10 degrees. Body
pitch reached about 9 degrees downhill; the user confirmed the underside was
still resting on the ridge. About
28 seconds were at 50% and 6.5 seconds at 65%, including settling; progress was
slow. Trials 31 and 32 stopped before reaching this crest gait.

The reduced rear stroke follows that observation. Reproduce its comparison with
`python -m robot_b91.rear_stride_experiment`. It samples 20-, 25- and 30-degree
sharp crests, crest positions 0.45/0.60/0.75m, two initial offsets, two speeds and
three assumed friction levels. Of 18 cases per gait, the selected 0.45 front/
35% rear setting crossed 10 within the 12-degree heading limit, versus eight
for equal 0.35 strokes. All six 30-degree cases per gait stayed grounded. These
are sampled mechanics scenarios with ideal feedback and uncalibrated contact;
The reduced-rear-stroke firmware was uploaded and its capability verified on
2026-11-19. Assisted trial 34 with front yaw 0.45/rear 35% ran 35 seconds, with
peak roll 2.75 degrees and pitch 12.56 degrees, then stopped; the user confirmed
it remained caught with little progress. The next candidate widens front yaw
to 0.55 and uses rear 30%, keeping rear excursion about 0.16 radians.
Six of six 25-degree model cases crossed, five within the heading limit.
In physical trial 35, that wider setting stopped after 23.89 seconds at the
12-degree heading guard; peak roll was 3.63 degrees and pitch 11.94 degrees.
The robot stayed on the ramp, but no crest crossing was observed. These physical
results do not establish an improvement from the wider stroke.

Rear flare and extra lift were then compared offline with
`python -m robot_b91.rear_flare_experiment` and `--refine`. Larger rear lifts
did not clear either modeled crest; flaring alone occasionally cleared the
30-degree crest but frequently caused lateral drift. No candidate was selected
for the live walking gait, and no new firmware was uploaded.

`python dashboard/rear_support_probe.py --dry-run` shows a separate 15-second
support plan: enable each channel at its calibrated neutral angle, gradually
restore the crest stance, then hold the front feet while each rear foot lifts,
flares from 14 to 23 physical servo degrees (mirrored left/right), and replants;
then each rear foot lifts and replants once more. It uses the installed manual
API with a fresh stationary IMU calibration, onboard camera alignment checks,
the existing heading/tilt guards, a 22-second wall limit, and stop on exit.
Initial camera alignment is now checked before any leg outputs are enabled.
The first pose must also match the controller's required neutral angles.
It requires an explicitly prepared crest placement and operator readiness;
it is not integrated into automatic navigation. The laptop camera only records
operator feedback. Offline checks (`python -m robot_b91.rear_support_experiment`)
completed 10/12 sampled cases with neutral initialization; two stopped at an
attitude or lateral bound. The model is uncalibrated and includes the initial
stance transition. Physical trial 39 completed the full sequence in 18.40s
(including command overhead), peak roll 0.76 degrees, corrected heading change
2.71 degrees. The user confirmed the underside still rested on the ridge with
no noticeable lift. This probe has not established a useful recovery motion.

Course transitions use observed pitch and camera guidance, not a fixed crest
coordinate or travel duration. Tests cover slope sequences at different times,
but changed exam layouts still need validation. The fixed 35-degree pitch stop
remains in force. The current sequence assists the first crest only.

The camera search also tries a more downward rear view (`backward_search_servo_deg`
= -20) after the original rear view fails. On the stopped trial-40 descent,
all five frames at this angle gave stable edge estimates while the standard
views failed. Offline replay reacquired guidance after 11.2 seconds with the
added view, including preservation of an existing heading estimate; the old
sequence did not reacquire in 50 simulated seconds. Calibration anchors,
steering limits and the requirement for sustained detection remain unchanged.
Physical automatic use of the new view still needs validation.

For a deliberate assisted placement at the first crest, add `--start-at-crest`
to the dashboard command and `--assisted-crest-start --seconds 35` to
`dashboard/ramp_trial.py`. Both flags must agree before the recorder starts
motion. Select `--crest-wave` for the wave or `--crest-assist` for the earlier
diagonal crest gait; the latter can be combined with `--adaptive-ramp-speed`.
This begins directly in the selected crest gait, retains the heading/tilt guards,
and restores the normal gait on descent. These tests are recorded as assisted
and do not establish a full ramp traversal.

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
Sidestepping, the crest wave and gentler steering are installed in the verified
2026-11-19 controller firmware. One physical left-step floor trial at 65%
completed its cycle and stopped with outputs off: peak roll 0.21 degrees,
pitch 0.62 degrees and heading change below 2.5 degrees. Webcam feedback showed
a small leftward shift during the cycle, largely offset by the initial stance
adjustment. Displacement is not calibrated; right steps and ramp use remain
unvalidated.

## CAD and rebuild

- Current assembly and parts: `cad/B10_1/`
- Rebuild simulation: `python robot.py build`
- After STEP changes:
  1. `python -m pip install -r requirements-cad.txt`
  2. `python cad/export_b10_sim.py`
  3. `python robot.py build`
