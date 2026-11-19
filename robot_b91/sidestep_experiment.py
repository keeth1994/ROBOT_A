"""Offline sidestep mechanics on a flat floor; no network or hardware commands.

Uses the crest experiment's bare-foot CAD and restored camera mass. Friction and
contact dynamics are assumptions. Ideal yaw feedback isolates gait mechanics;
these results are not physical validation or onboard-sensor validation.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math

import mujoco
import numpy as np

from robot_b91.crest_experiment import OUT as CREST_OUT, build
from robot_b91.gait import rocker_targets
from robot_b91.paths import ROOT
from robot_b91.run import Robot, load_gait_parameters

OUT = ROOT / 'results/ramp_trials/sidestep_sim'


def simulate(case):
    pattern, direction, friction, amplitude, bias, cycles = case
    params = load_gait_parameters() | dict(pattern=pattern, period=2., duty=.8,
        pitch_lift=.35, pitch_stance=0., yaw_amplitude=amplitude, yaw_bias=bias,
        sideways=True, friction=friction)
    robot = Robot(params, feedback='ideal', model_path=CREST_OUT/'model.xml')
    model, data = robot.m, robot.d
    data.qpos[:3] = [-.6, 0, .06]  # Away from all ramp geometry.
    mujoco.mj_forward(model, data)
    origin = None
    max_roll = max_heading = 0.
    speed = .65
    duration = cycles * params['period'] / speed
    reason = 'cycles completed'
    final_target = None
    while data.time < 2 + duration + .3:
        t = data.time - 2
        rotation = data.xmat[model.body('platform').id].reshape(3, 3)
        heading = math.atan2(rotation[1, 0], rotation[0, 0])
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        if t <= 0:
            target = np.zeros(8)
        else:
            if origin is None:
                origin = data.qpos[:3].copy()
            max_roll = max(max_roll, abs(math.degrees(roll)))
            max_heading = max(max_heading, abs(math.degrees(heading)))
            if abs(roll) > math.radians(12) or abs(heading) > math.radians(12):
                reason = 'attitude limit'
                break
            if final_target is None:
                target = rocker_targets(params, t, min(t*speed, cycles*params['period']),
                                        heading, 0, robot.joint_ranges, sideways_scale=direction)
                if t >= duration:
                    final_target = target.copy()
            else:
                target = final_target
        robot.step_targets(target)
    displacement = data.qpos[:3] - origin
    result = dict(pattern=pattern, direction=direction, friction=friction,
        amplitude=amplitude, bias=bias, cycles=cycles, speed=speed, reason=reason, seconds=float(data.time),
        dx_mm=float(displacement[0]*1000), dy_mm=float(displacement[1]*1000),
        max_roll_deg=max_roll, max_heading_deg=max_heading,
        warnings=int(sum(w.number for w in data.warning)))
    robot.close()
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--search', action='store_true')
    parser.add_argument('--tune', action='store_true')
    parser.add_argument('--cycles', type=int, choices=(1,2), default=1)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if not (CREST_OUT/'model.xml').exists():
        build()
    cases = [(pattern, direction, friction, amplitude, bias, args.cycles)
             for pattern in (('crest', 'crawl', 'trot') if args.search else ('crest',))
             for direction in (-1, 1) for friction in (.5, .8, 1.1)
             for amplitude, bias in ([(a,b) for a in (.25,.35,.45) for b in (-.25,0,.25)]
                                     if args.tune else [(.45, 0)])]
    with ProcessPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(simulate, cases))
    (OUT/'latest_results.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
