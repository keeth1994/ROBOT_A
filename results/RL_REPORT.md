# First B3 reinforcement-learning experiment

Current CAD revision: short_flat_round_outer_feet. Original Parallax motor limits, flat floor. No programmed gait or imposed base motion.

Forward motion appeared after about 100 seconds of learning (131,072 collected transitions). Across all three trials, setup/training/evaluation inside the training commands took 6.91 minutes; package installation and extra checks are excluded. Eight CPU environments were used. More training did not improve performance monotonically.

| Trial | Collected steps at end | Command wall time | Selected-policy distance / up to 10 s | Strict success, five evaluation seeds |
|---|---:|---:|---:|---:|
| rl_flat | 131,072 | 119.9 s | 1.119 m | 20% |
| rl_flat_continued | 393,216 | 187.3 s | 1.119 m | 20% |
| rl_flat_straight | 262,144 | 107.2 s | 1.322 m | 20% |

The continuation resumes the initial trial; the stronger-penalty trial branches from the initial best policy, so the step counts above must not be added as independent cumulative checkpoints.

## New-seed evaluation of the selected policy

- Mean forward distance: **1.318 m per episode of up to 10 s** (one episode stopped at 9.46 s for excessive drift).
- Strict straight-line success: **2/10**.
- Falls: **0/10**. One episode ended early for sideways drift.
- Mean maximum lateral deviation: **10.3 cm**.
- Worst lateral deviation: **25.2 cm**.
- Finer 1 ms physics check, three seeds: **1.113 m average**, 0% strict success.

Success requires at least 0.30 m forward, maximum sideways deviation 4 cm, maximum heading error 15 degrees, and no fall, non-foot ground contact or simulation warning. Forward locomotion is learned; reliable straight-line control is **not yet achieved**. This is one training seed, not a convergence or hardware demonstration.

Selected policy: `rl_flat_straight/best.zip`. Full new-seed metrics: `rl_flat_straight/heldout.json`. Video: `rl_flat_straight/replay.mp4`. The default viewer follows `reference/b3_rl_policy.json` and checks the robot XML hash.

The policy uses ideal simulator feedback including joint states, base velocity and orientation. A hardware estimator and realistic sensor observations remain necessary. Further work should focus on reducing drift and validating friction, timing and actuator assumptions before obstacle training.
