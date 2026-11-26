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
| Foam crest simulation | `python robot.py crest --help` |
| Simulation options | `python robot.py walk --help` |
| Checks | `python robot.py check all` |

- Dashboard: http://127.0.0.1:8765
- Dashboard gait: `dashboard/walking.toml` (no camera/IMU feedback)
- Simulation gait: `reference/b91_gait.toml`
- Generated reports and recordings: `results/` (ignored by Git)

### Foam crest trials

Compare the dashboard's current crest wave and the simulation candidate on identical starts:

```powershell
python robot.py crest --suite all --gait both --seed 20261127 --workers 4 --output results/ramp_trials/foam_crest_no_cutoffs_20261126.json
```

The default suite uses 10, 14, 18 and 22.5 cm crests. Supply `--heights-cm 12 16 20`
to test other finished heights. The full suite adds alternate slopes, approach
distances and offsets, then 24 seeded randomized starts. Each run starts from rest
near one sharp crest on a 29 cm wide deck. Printed feet contact a compliant 10 mm layer
over rigid backing. Sliding friction (0.9 to 1.3) and compliance are assumptions,
not measurements. The candidate blends the climbing gait into the crest wave from
body pitch, using ideal attitude with a pitch error in the perturbed cases.

A pass requires the body to stay 22 cm past the crest for 0.2 s, all four feet
to have loaded the downhill panel, at least two to be supporting the robot at
finish, and no chassis support at finish. Roll or heading beyond 12 degrees,
drift beyond 4 cm, floor contact and excess foam penetration are diagnostics.
They do not end a run. The report records the first time of each excursion in
`first_excursion_s` (negative during settling), and whether the robot crossed
without any excursion in `clean_crossing`. Each case ends at a crossing, the
requested simulation duration, or invalid physics. The command exits nonzero
if the selected gait fails to cross. It never connects to the physical robot.

On 2026-11-26 with MuJoCo 3.13.0 and seed `20261127`:

| Crossings (within the old limits) | Dashboard wave | Foam candidate |
|---|---:|---:|
| Standard heights and starts | 7/12 (3) | 11/12 (11) |
| Alternative slopes and starts | 7/15 (4) | 15/15 (15) |
| Randomized starts | 15/24 (8) | 24/24 (22) |

The foam candidate crossed 50/51; 48/51 stayed within the former limits. The
22.5 cm left start still timed out. Random cases 8 and 15 crossed after an
excursion. Above the course design height,
`python robot.py crest --heights-cm 25 30` crossed 1/3 starts at 25 cm and 0/3
at 30 cm. These are isolated crest trials, not a full-course or hardware qualification.

One-off recovery probes and simulation experiment scripts are local, Git-ignored
tools. Historical experiment commands below require those local files and are
not available in a fresh checkout. The dashboard regression tests, trial recorder
(`dashboard/ramp_trial.py`) and stationary camera checker
(`dashboard/capture_ramp_views.py`) remain tracked.

### Running the ramp

Start `python dashboard/run.py --ramp-only --ramp-drift-control` and select
**Follow ramp (camera + IMU)**. The approach starts directly, with no entrance
alignment or stationary pose verification. Normal sensor settling and camera
guidance still apply. Large roll, pitch, heading and offset can affect steering
or speed but do not shut down a run. Drift recovery has no deadline. Without
`--ramp-only`, the course continues to corridor search after the ramp exit.

**Stop all servos**, the operator heartbeat, the controller/link watchdog and
hardware fault handling remain active. Missing guidance can pause steps and
resume on fresh data. `dashboard/ramp_trial.py` records until course completion
or operator Stop (including the `STOP` file), unless an explicit `--seconds`
duration is supplied. Peak roll and pitch are recorded. No firmware update is needed.

### Historical handoff (2026-11-20)

The observations below describe older revisions. Entrance alignment and the
course cutoffs described in these trial notes are no longer available.

The dashboard was left running at http://127.0.0.1:8765 with outputs off and
the operator's requested speed retained. Place the robot on flat floor at the ramp entrance, clear
your hands, and press **Follow ramp (camera + IMU)**. Keep the dashboard tab
visible so its operator heartbeat stays active. **Stop all servos** stops it.

Automatic entrance alignment, drift correction and crest assistance are enabled.
The first climb is capped at 50%; subsequent phases allow up to the requested
speed, with automatic slowing. After a disturbance or gait change, speed now stays
at 50% until a complete stable gait cycle is observed. This latest speed-recovery
change passes the tests but still needs physical validation. GUI runs stop at the
ramp exit or a safety stop; the recorder's duration limits do not apply to the GUI.

Navigation follows the paired ramp edges using the onboard camera and IMU.
Edge confirmation now tolerates control updates up to 0.9 seconds apart while
still requiring individually fresh images (under 0.35 seconds old, or 0.25 for
entrance alignment). This fixes a recovery deadlock: recorded updates averaged
about 0.6 seconds apart, but the former confirmation limit was 0.35 seconds.
Alignment allows 15 seconds to reacquire edges or entrance floor visibility,
holding steps during that interval; it allows 45 seconds without improvement
and 120 seconds overall. On the ramp, brief gaps retain the existing one-second
IMU guidance allowance, then hold steps and search for edges. Tilt, heading,
controller connection and operator Stop guards remain active. These changes
pass offline tests and a recorded-decision replay; physical reliability still
needs validation by the operator. No firmware upload is needed.

Entrance alignment now corrects toward 8 mm from the center and 2 degrees of
the ramp direction, using the latest four independent edge measurements instead
of a long median of already-filtered offsets. After correction, it switches to
the installed firmware's zero-lift, zero-yaw stance, settles for two seconds,
and verifies fresh, stable measurements with all feet down. If that check finds
remaining error, it restores the correction gait and repeats. This fixes the
observed loop where Wi-Fi delay let a foot lift between the feet-down status
check and the pause command, gradually changing the heading during verification.
The stance changes use confirmed stop/start requests; a failed change ends the
run. No USB upload is required. This behavior still needs a physical test.

Ramp centering now computes body offset from the paired-edge fit's own heading,
when sufficient edge depth is visible. Previously it substituted the slowly
corrected gyro heading into that geometry, which could reverse the reported
side error. Gyro fusion still supplies steering and narrow-edge fallback.
Logs now include the independent edge heading/offset and offset source. With
drift control enabled, strafe gain is 12 (the sideways command remains capped
at 0.3), and forward stride reduces smoothly from full at 1 cm offset to half
at 3 cm. The 0.2 turn limit is unchanged; no fixed rightward bias is added.
Crest, speed recovery and descent confirmation tolerate fresh control samples
up to 0.9 seconds apart, fixing another 0.35-second cadence restriction that
prevented crest assistance from arming in the latest runs. These changes are
tested offline; physical drift and crossing performance remain unverified.

With ramp drift control enabled, crossing 12 degrees of heading error now starts
a bounded recovery instead of immediately ending the run. It caps speed at 50%
and forward stride at 35%, retaining the existing turn and sideways limits.
Correction requires fresh paired edges and roll below 10 degrees; otherwise
steps hold while guidance settles. A single raw edge-offset excursion beyond
5 cm also holds correction for rechecking. Recovery ends after heading stays
within 7 degrees and offset within 2.5 cm for 0.8 seconds. It stops after 15
seconds without recovery, at 20 degrees heading, 5 cm filtered offset, or the
existing 12-degree roll/35-degree pitch envelope. Operator Stop and connection
watchdogs remain immediate. Crest transitions wait during recovery. This is
tested offline but not yet validated as a successful physical recovery.

That session additionally recorded controls and onboard images under
`results/ramp_trials/manual_heading_recovery/`. Repeated GUI runs can be
distinguished by their motion-session IDs and timestamps. For a fresh recording
after restarting, append `--control-log results/ramp_trials/NEW/control.jsonl`,
using a new folder name. The requested speed resets to 100% on a dashboard
restart, so set its slider back to 65% before starting.

Trial 54 data and analysis are in `results/ramp_trials/attempt_54_drift_control/`:
`analysis.json`, `control_metrics.csv`, `control.jsonl`, `control_frames/`,
`telemetry.jsonl` and webcam snapshots. That full attempt stopped at the first
crest after 100.3 seconds; it did not complete the ramp. All 106 dashboard tests
pass. The robot's full-ramp repeatability is still unresolved.

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

### Historical first-crest experiments

Physical candidate at the time of trial 45:
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

Repeat trial 46 on 2026-11-20 stopped on the first descent after 107.32 seconds
at the 12-degree heading guard. It used the same control source and settings
as trial 45 and remained on the ramp. Recorded navigation heading reached
-12.16 degrees and peak control roll was 8.07 degrees. The cause of the drift
is not established; reliable full traversal remains an open task.

Trial 49 added automatic entrance alignment and handed over to climbing after
3.65 seconds. It reached the descent phase at 92.29 seconds, then stopped at
106.42 seconds after 12 seconds without camera reacquisition. It remained on
the ramp with outputs off. This validates the alignment-to-climb transition,
but is not another completed ramp run.

Trial 50 corrected about 5 cm of entrance offset and 7 degrees of heading error,
then entered the climb after 21.61 seconds. It switched to the crest gait at
95.46 seconds and descent at 105.48 seconds. The user confirmed that the chassis
was completely past the first ridge. After ordinary camera reacquisition,
the IMU recorded a rapid turn during a diagonal foot lift; the 12-degree heading
guard stopped the run at 124.24 seconds. The new two-view heading recovery did
not trigger on this run. Outputs were confirmed off. Full-ramp repeatability
is still unresolved, particularly during the first descent.

Preliminary descent simulations compared the baseline with raised support,
single-foot lifting, lower lifts, longer support time and earlier heading
correction. None improved the sampled baseline result. These local experiments
use assumed 25-degree geometry, unmeasured friction and ideal feedback; some
cases slide substantially during initial settling. They do not justify replacing
the physical gait. Results are under `results/ramp_trials/descent_support_sim/`,
`descent_support_refined/` and `descent_steering_sim/`.

Assisted descent trial 51 stopped after 2.63 seconds, before any walking steps.
All four feet stayed in the commanded starting stance. The heading estimate
changed from about 1.3 degrees in the stationary precheck to 12.5 degrees during
settling; independent checks afterward gave 13.5 and 12.0 degrees at the two
forward camera angles. This suggests turning or slipping during initial stance
placement. The MCU deliberately holds yaw at zero during its first two seconds,
so that reading cannot exclude physical turning in this interval. Outputs were
confirmed off. This trial did not measure downhill walking performance; the next
diagnostic starts with all four feet beyond the crest on the descending surface.
Assisted section tests do not establish a full automatic traversal.

Trial 52, placed with all four feet beyond the crest, exposed a separate startup
error: a -19.3-degree heading fit made during IMU settling remained latched even
though a stationary check afterward measured about zero. Unseeded camera/IMU
ramp starts now wait until the MCU's two-second settling period has passed,
then require at least four independent visual fits spanning 0.6 seconds at a
forward camera angle. Heading spread must stay within 2 degrees, offset spread
within 15 mm, and IMU attitude spread within 0.75 degrees. The median pose seeds
normal tracking; an alignment handover keeps its already verified pose. The
existing heading stop and camera search remain active.

All 72 dashboard tests pass. Recorded-image replay confirmed startup after
5.6 seconds, and a live check with legs disabled confirmed it after 2.93 seconds.
Assisted trial 53 then walked down the first descent, through the valley and
onto the next climb at 50%, stopping at its 35-second limit with outputs off.
Peak recorded roll was 7.02 degrees. Camera search reacquired the downward rear
view during the descent. The user still observed excessive left drift; estimated
sideways error peaked near 50 mm. This validates startup and forward progress,
but does not establish satisfactory lateral control or a full ramp traversal.

The next correction candidate adds `--ramp-drift-control` to the dashboard
command. Sideways gain rises from 6 to 8, retaining its existing 0.3 saturation
and the 0.2 turn limit. Forward stride smoothly reduces from 100% at 15 mm offset
to 65% at 40 mm, giving sideways correction more time without changing cadence.
Both directions use the same response. This uses the installed controller's
variable-forward protocol; a capability check rejects unsupported firmware
before motion. Camera-only, manual, corridor and alignment profiles keep their
existing settings.

In an uncalibrated simulation screen, the selected response reduced lateral
error in the three cases that completed, while the same three of six cases
completed overall. Its first full physical test is described below. The change
passed 78 dashboard tests, including 200 additional comparisons with MCU gait math and
the required forward command during gait restarts and exit clearance.
Monitored comparisons use a five-minute overall ceiling, allowing completion
instead of the earlier 35-second cutoff.
At that point, a 25-second guidance timeout and heading and tilt cutoffs were
still used. The current recorder has no such course cutoffs.

The user requested a full entrance start for trial 54. Alignment completed in
about 12 seconds, then the robot climbed at 50% and entered the crest gait at
91.7 seconds. It stopped at 100.3 seconds on the 12-degree heading guard and
remained on the ramp. Fresh stationary fits from both forward views confirmed
about -12.2 degrees heading, with only 2.4 mm lateral offset. Outputs were off.
This was a real turn rather than a completed ramp run or a false heading stop.

Detailed recording captured 521 acknowledged control updates and 519 onboard
frames. The approach's median absolute estimated offset was 11.6 mm, with a
45.3 mm peak; this single run does not establish improved repeatability.
At the crest, diagonal front-right/rear-left lifting coincided with sharp yaw
motion. The governor restored 65% after one quiet second between lifts; the
next lift preceded the final heading stop. It also slipped at 50%, so speed
alone is not a proven explanation.

The governor now requires a whole stable gait cycle before restoring speed
(four seconds at 50% for this gait). Gait changes restart that stability check,
and gaps in observations cannot count toward it. Replay of trial 54 removes
all four recorded 65% crest commands, but does not predict the resulting motion.
All 80 dashboard tests pass; a physical repeat is pending.

For detailed recording, add `--control-log results/ramp_trials/NEW/control.jsonl`
to the dashboard command. Each acknowledged command records the fresh MCU
status, navigation decision, timing and corresponding camera frame. A bounded
background queue keeps image encoding and disk writes outside the control loop.
Final samples now flush even after control stops; trial 54's last buffered
records were lost on shutdown, so its stop event comes from `telemetry.jsonl`.

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
It restores requested speed after a full stable gait cycle. The original
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
Trial 45 subsequently reacquired this view automatically on both descents.

On the trial-49 descent, the retained heading of about -6 degrees rejected all
ten stationary forward-view samples. Fresh fits at both forward camera angles
agreed near +10 to +11 degrees with little lateral offset. The source of that
heading disagreement is not established. A paused recovery now requires at
least four consistent fresh observations spanning 0.6 seconds at each forward
angle. Both views must agree within 2.5 degrees and 15 mm, with no more than
2 degrees of body attitude change. Candidate heading must remain within the
12-degree stop limit, offset within 50 mm, and heading correction within
20 degrees. Rear views cannot trigger this recovery. The controller keeps steps
held until ordinary sustained camera reacquisition succeeds after the update.

Recorded-image replay reacquired in five seconds with recovery enabled; the
previous search did not reacquire within 16 seconds. A live camera-only check
reacquired in 7.1 seconds with leg outputs disabled throughout. Recovery during
a moving full ramp run remains unvalidated. The recorder's 12-second search
timeout used in those tests was 12 seconds; it is now 25 seconds following the
request to allow more recovery time. Existing heading/tilt stops are unchanged.

For a deliberate assisted placement at the first crest, add `--start-at-crest`
to the dashboard command and `--assisted-crest-start --seconds 35` to
`dashboard/ramp_trial.py`. Both flags must agree before the recorder starts
motion. Select `--crest-wave` for the wave or `--crest-assist` for the earlier
diagonal crest gait; the latter can be combined with `--adaptive-ramp-speed`.
This begins directly in the selected crest gait, retains the heading/tilt guards,
and restores the normal gait on descent. These tests are recorded as assisted
and do not establish a full ramp traversal.

### Retired ramp alignment trials

Physical trial 47 declared success during initial settling, but an independent
stationary check disagreed; it is not counted as successful alignment. After
the visual-heading correction, trial 48 made corrective steps and completed in
22.78 seconds at 50%, with peak roll 0.95 degrees. A separate five-frame check
gave median offset 10 mm right and heading -0.7 degrees, within the target;
one image fit was an outlier. Outputs were confirmed off. This is one successful
standalone test; larger offsets/headings and the opposite correction direction
still need physical validation. Trial 49 subsequently validated the automatic
transition into climbing, but stopped on the first descent after camera loss.
The climb inherits the verified median heading and offset, rather than the
latest individual frame. All 80 dashboard tests pass, including compiled
controller gait comparisons, rejection of a final-frame outlier at handover,
and the paused recovery's consistency and movement guards.

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
