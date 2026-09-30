# B3 reinforcement learning on flat ground

This is a separate PPO controller that learns eight absolute yaw/pitch targets. It does not call the programmed gait, prescribe a foot trajectory, teleport the body, or apply external forces. MuJoCo contacts and the original Parallax torque/speed limits produce movement. The existing course controller remains available separately.

## Run

From the repository terminal:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-rl.txt
.\.venv\Scripts\python.exe train_b3_rl.py --strict --steps 262144 --envs 8 --minutes 10
.\.venv\Scripts\python.exe watch_b3_rl.py
```

VS Code: **Terminal > Run Task > B3 RL - train straight line** or **B3 RL - watch learned policy**. The watch command repeats a deterministic 10-second evaluation. The default build task still opens the programmed obstacle-course gait, not RL.

Continue from a saved policy into a new output directory:

```powershell
.\.venv\Scripts\python.exe train_b3_rl.py --resume results/rl_flat/latest.zip --out results/rl_flat_continued --steps 1000000 --minutes 20
.\.venv\Scripts\python.exe watch_b3_rl.py --model results/rl_flat_continued/best.zip
```

`--steps` is the number of additional policy transitions across all environments; each transition represents 20 ms, using ten 2 ms physics steps. The time cap applies during learning; startup and evaluation add time. Checkpoints are local. A policy must be retrained/revalidated when the robot geometry, control mapping or observation definition changes.

## Task and measurement

Each episode starts on the checkerboard floor, settles for 0.3 s, then lasts up to 10 s. Initial joint angles have small seeded perturbations. PPO receives ideal simulator joint angles/velocities, base velocity, angular velocity, orientation, lateral position, height, and its previous action. These are privileged simulator observations; the supplied servo/sensor package does not directly provide all of them. This experiment is not yet a deployable hardware controller.

Reward favours forward velocity and penalizes lateral motion/displacement, heading error, tilt, power, command changes and non-foot floor contact. Episodes terminate for falls, excessive drift/heading, or numerical warnings. Success is at least 0.30 m forward in 10 s, maximum lateral error 0.04 m, maximum heading error 15 degrees, and no fall, non-foot contact or warnings. Distance alone is not success.

The policy outputs joint positions within +/-45 degrees; a PD servo controller uses the same torque-speed envelope as the existing gait. Pitch stall limit is approximately 0.2683 Nm and geared yaw approximately 0.1521 Nm. No motor upgrade. Mass, friction, actuator and inertia approximations are inherited from the current model. No curriculum, gait imitation, phase clock, obstacle navigation or domain randomization is included in this first experiment.

PPO implementation: [Stable Baselines3](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html), with a [custom Gymnasium environment](https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html).

## Outputs and checks

`results/rl_flat/` contains standing and untrained baselines, `progress.csv`, periodic `learning_curve.json`, `best.zip`, `latest.zip`, and final `summary.json` with model hash and revision. Best checkpoint means highest evaluation reward, not necessarily successful walking. Check actual success and distance in the summary. Training environments use different seeds; checkpoint evaluation uses seeds 1000-1001 and final reporting 1000-1004. One training seed is a preliminary experiment, not proof of robust learning.

```powershell
.\.venv\Scripts\python.exe test_b3_rl.py
.\.venv\Scripts\python.exe watch_b3_rl.py --record results/rl_flat/replay.mp4
```

The tests check the Gym API, reproducible resets, full-CAD/headless trajectory parity, motor torque caps and absence of external forces. Training removes only non-colliding CAD visual meshes; contact geometry, body masses and inertia remain intact.

## Initial experiment results

See [measured training and evaluation results](results/RL_REPORT.md). The default viewer uses the selected policy in `reference/b3_rl_policy.json`. The first experiment learned forward movement quickly, but passes strict straight-line criteria on only 2/10 new seeds.

`--strict` increases the lateral-displacement penalty from 4 to 15 and the heading penalty from 0.3 to 1.5. Reward weights are recorded in the run summary. Use `evaluate_b3_rl.py --model PATH --episodes 10 --out REPORT.json` for new-seed evaluation; add `--fine` for 1 ms physics.
