# B3 straight-line and gentle-ramp curriculum

These lessons retain the current robot, original motor torque/speed limits, 20 ms policy interval and learned joint-target action space. They use ideal simulator feedback, with no programmed gait, imposed body motion or physical guide rails. No new terrain sensors are invented: the policy reacts using the same body/joint observations as the flat-ground experiment.

Stage 1 is a 10 s flat-ground trial: at least 0.5 m forward, maximum lateral displacement 3 cm, maximum heading error 10 degrees, no fall, non-foot ground contact or warnings. Speed reward saturates at 0.08 m/s, and alignment reduces the reward for faster diagonal motion. Drifting beyond 10 cm or turning beyond 40 degrees ends training episodes.

Stage 2 uses separate 3-degree and 5-degree lessons: 30 cm flat approach, 40 cm ascent, 20 cm level top, 40 cm descent. Ramp width is 30 cm. Heights are approximately 21 mm and 35 mm. The panels are rigid, with the same assumed friction as the existing simulation. Completion requires contact with ascent, top and descent and the body clearing x=1.45 m within 25 s. The same 3 cm/10 degree limits apply. Stepping off the side is a failure. A standing policy cannot pass.

Checkpoint selection prioritizes the success rate on three development seeds, then physical completion, then reward. Twenty separate seeds (4000-4019) assess each selected checkpoint. A stage gate requires 90% success. These seeds vary initial joint angles slightly; they are not a broad hardware/domain-randomization study. Results on gently sloping ramps do not establish capability on the full course.

## Commands

```powershell
.\.venv\Scripts\python.exe robot.py train --resume results/rl_flat_straight/best.zip --out results/curriculum_flat --steps 393216
.\.venv\Scripts\python.exe robot.py train --slope 3 --resume results/curriculum_flat/best.zip --out results/curriculum_3deg --steps 262144
.\.venv\Scripts\python.exe robot.py train --slope 5 --resume results/curriculum_3deg/best.zip --out results/curriculum_5deg --steps 262144
.\.venv\Scripts\python.exe robot.py watch --curriculum --slope 3 --model results/curriculum_3deg/best.zip
.\.venv\Scripts\python.exe robot.py watch --curriculum --slope 5 --model results/curriculum_5deg/best.zip
```

Each run saves its initial evaluation, learning curve, best and latest checkpoints, and a summary with validation metrics and model/environment hashes. Continuing from a checkpoint adds `--steps` transitions; it does not reset the learned policy. `--minutes` caps the training portion, with evaluation time additional. Keep each experiment in a distinct output folder.

Checks: `robot.py check curriculum` verifies physical ramp heights/width, headless/full-CAD parity, contact accounting and rejection of a stationary policy. `robot.py check rl` checks original environment compatibility and actuator limits.

## Observation scaling and continued practice

The refined runs scale small velocity, tilt and lateral-position inputs before the network. The first layer is adjusted when converting an old policy, preserving its initial action mapping. Checkpoint metadata records this convention, and the viewer detects it automatically. `--normalized` enables conversion; later resumes inherit it. `--exploration-std 0.2` or `0.15` reduces exploration during fine-tuning.

Slope runs use two flat and six sloped training environments. After training, the selected ramp policy is also checked on twenty flat-ground seeds (5000-5019) for regression. Existing experiment directories cannot be overwritten by the curriculum trainer.

VS Code offers separate viewing tasks for the refined flat policy and each gentle ramp. These do not replace the original full obstacle-course task.

## Earlier selected-checkpoint results

See [final fresh-seed report](../results/CURRICULUM_REPORT.md). Flat: 20/20 clean passes. 3 degrees: 17/20 physical completions, 6/20 clean passes. 5 degrees: 3/20 physical completions, 1/20 clean pass. These are separate selected checkpoints, not one policy verified for every terrain. Selected files are listed in `reference/b3_curriculum_policies.json`; the default viewer opens the selected flat policy.

The commands above reproduce the original experiment sequence; choose unused output directories when rerunning them. To continue the 5-degree lesson:

```powershell
.\.venv\Scripts\python.exe robot.py train --slope 5 --resume results/curriculum_selected/5deg/policy.zip --out results/curriculum_5deg_next --steps 1000000 --minutes 15
```

For a fresh evaluation, use `--evaluate-only --evaluation-seed 9000` with a new output directory. Viewer `--seed` selects the reset seed.

## Latest successful 3-degree run

`results/my_3deg_run_02/best.zip` is the later checkpoint used by the meeting ramp demo. Its saved summary reports `gate_passed: true`. The earlier selected-checkpoint statistics above describe different policies and must not be presented as the result of this newer run.

To advance from this successful checkpoint, choose an unused output folder:

```powershell
.\.venv\Scripts\python.exe robot.py train --slope 5 --resume results/my_3deg_run_02/best.zip --out results/my_5deg_run_01 --steps 1000000 --minutes 15
```
