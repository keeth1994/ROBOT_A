"""Offline check of the bounded manual rear support probe, with no robot access."""
from concurrent.futures import ProcessPoolExecutor
import json
import math
import sys

import mujoco
import numpy as np

from robot_b91.crest_experiment import ROOT, build
from robot_b91.run import Robot

sys.path.insert(0, str(ROOT / 'dashboard'))
from rear_support_probe import plan


def simulate(case):
    slope, crest, offset, friction = case
    folder = ROOT / f'results/ramp_trials/rear_stride_sim/slope{slope}_crest{crest}'
    metadata = json.loads((folder / 'model.json').read_text())
    robot = Robot({'friction': friction}, feedback='ideal', model_path=folder/'model.xml')
    m, d = robot.m, robot.d
    stages = plan()
    def model_angles(degrees):
        angles = np.radians(degrees)
        angles[1::2] -= math.pi/4  # Physical +45 means model foot-down zero.
        angles[[2, 6]] *= -1     # Rear yaw mounting.
        return angles
    initial = model_angles(stages[0][2])
    d.qpos[robot.qa] = initial
    d.qpos[:3] = [crest+offset, 0, metadata['crest_z_m']+.055]
    pitch = math.radians(5)  # Approximate current stopped crest attitude.
    d.qpos[3:7] = [math.cos(pitch/2), 0, math.sin(pitch/2), 0]
    mujoco.mj_forward(m, d)
    origin = None
    peak_roll = peak_heading = peak_lateral = peak_rise = 0.
    contacts = count = 0
    reason = 'completed'
    duration = sum(s[1] for s in stages)
    history = []
    last_log = -1
    while d.time < 2+duration:
        t = d.time-2
        target = initial.copy()
        label = 'settle'
        if t >= 0:
            r = d.xmat[m.body('platform').id].reshape(3, 3)
            roll = math.atan2(r[2, 1], r[2, 2])
            pitch = math.asin(-np.clip(r[2, 0], -1, 1))
            heading = math.atan2(r[1, 0], r[0, 0])
            if origin is None:
                origin = d.qpos[:3].copy()
                initial_heading = heading
            peak_roll = max(peak_roll, abs(math.degrees(roll)))
            peak_heading = max(peak_heading, abs(math.degrees(heading-initial_heading)))
            peak_lateral = max(peak_lateral, abs(float(d.qpos[1]-origin[1])))
            peak_rise = max(peak_rise, float(d.qpos[2]-origin[2]))
            if (abs(roll) > math.radians(12) or abs(pitch) > math.radians(30)
                    or peak_heading > 10 or peak_lateral > .015):
                reason = 'attitude or lateral limit'
                break
            count += 1
            contacts += int(any(m.geom_bodyid[c.geom1] == m.body('platform').id or
                                m.geom_bodyid[c.geom2] == m.body('platform').id for c in d.contact))
            elapsed = t
            previous = stages[0][2]
            for label, seconds, pose in stages:
                if elapsed <= seconds:
                    u = np.clip(elapsed/seconds, 0, 1)
                    u = u*u*(3-2*u)
                    target = model_angles(np.round(np.array(previous)+(np.array(pose)-previous)*u))
                    break
                elapsed -= seconds
                previous = pose
        robot.step_targets(target)
        if d.time-last_log >= .2:
            history.append(dict(t=float(d.time), stage=label, xyz=d.qpos[:3].tolist()))
            last_log = d.time
    result = dict(slope=slope, offset=offset, friction=friction, reason=reason,
                  seconds=float(d.time), max_roll_deg=peak_roll, max_heading_deg=peak_heading,
                  max_lateral_mm=peak_lateral*1000, peak_body_rise_mm=peak_rise*1000,
                  platform_contact_fraction=contacts/max(count, 1),
                  warnings=int(sum(w.number for w in d.warning)))
    robot.close()
    print(json.dumps(result), flush=True)
    return dict(result=result, history=history)


def main():
    out = ROOT/'results/ramp_trials/rear_support_sim'
    out.mkdir(parents=True, exist_ok=True)
    cases = []
    for slope, crest in ((25, .6), (30, .75)):
        folder = ROOT / f'results/ramp_trials/rear_stride_sim/slope{slope}_crest{crest}'
        if not (folder/'model.xml').exists():
            build(slope, crest, folder)
        cases.extend((slope, crest, offset, friction) for offset in (-.02, 0)
                     for friction in (.5, .8, 1.1))
    with ProcessPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(simulate, cases))
    (out/'comparison.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
