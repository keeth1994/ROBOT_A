"""Validate corridor acquisition with rendered RGB, IMU and side IR in full physics."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image

from robot_b91.paths import ROOT
from robot_b91.progress import corridor_entry_score
from robot_b91.run import Robot


CASES = {
    'side': (3.9, -.15, 0),
    'front': (4.15, -1.1, math.pi),
    'angled': (3.95, -.45, -1.2),
    'far_wall_first': (3.75, -.15, -2.2),
    'ramp_exit': (3.55, 0, 0),
}


def evaluate(case='side', seconds=300, output=None, observer=None):
    robot = Robot(ramp=True, feedback='sensors')
    samples, transitions = [], []
    collision = False
    max_tilt = 0.0
    try:
        x, y, yaw = CASES[case]
        robot.d.qpos[:2] = [x, y]
        robot.d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(robot.m, robot.d)
        robot.navigation.begin_corridor_search(0)
        search = robot.navigation.search
        wall_ids = {i for i in range(robot.m.ngeom)
                    if 'corridor_wall' in (robot.m.geom(i).name or '')}
        previous_stage = None
        next_sample = 0
        while robot.d.time < seconds:
            robot.step()
            if search.stage != previous_stage:
                print(f'{case}: {robot.d.time:.1f}s {search.stage} '
                      f'xy={robot.d.qpos[:2].round(3).tolist()} scan={search.turn_degrees:.0f}', flush=True)
                transitions.append(dict(time_s=robot.d.time, stage=search.stage,
                                        xy=robot.d.qpos[:2].tolist()))
                previous_stage = search.stage
                if search.stage == 'approach':
                    print('  goal', search.approach_goal.round(3).tolist(), 'wall heading', search.wall_heading,
                          'candidates', [(c['point'].round(3).tolist(), c['samples']) for c in search.candidates], flush=True)
                if output:
                    Image.fromarray(search.debug_image()).save(Path(output) / f'{case}_{search.stage}.png')
            if robot.d.time >= next_sample:
                samples.append(dict(time_s=robot.d.time, xy=robot.d.qpos[:2].tolist(),
                                    estimated_xy=search.position.tolist(), stage=search.stage,
                                    heading=search.imu_heading, steering=search.estimate.heading,
                                    forward=search.forward,
                                    speed=search.speed,
                                    approach_side=search.approach_side, wall_side=search.wall_side,
                                    ir_m=robot.ir_distances_m.tolist()))
                next_sample += .5
            collision |= any(int(c.geom1) in wall_ids or int(c.geom2) in wall_ids for c in robot.d.contact)
            max_tilt = max(max_tilt, robot.tilt())
            if observer is not None and not observer(robot):
                break
            if search.finished or collision or max_tilt > 40:
                break
            if not search.moving and robot.d.time - search.stage_started > 20:
                break
        score = corridor_entry_score(robot.m, robot.d, robot.course)
        result = dict(case=case, elapsed_s=robot.d.time, collision=collision,
                      max_tilt_deg=max_tilt, warnings=sum(w.number for w in robot.d.warning),
                      **search.report(), **score, transitions=transitions,
                      candidates=[dict(point=c['point'].tolist(), tangent=c['tangent'].tolist(),
                                       samples=c['samples']) for c in search.candidates],
                      samples=samples)
        if output:
            Path(output, case + '.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items()
                          if k not in ('candidates', 'samples', 'transitions')}), flush=True)
        return result
    finally:
        robot.close()


def run_interactive(case, seconds, output, sensors=False):
    from mujoco import viewer

    with ExitStack() as stack:
        window = overlay = None
        started = next_frame = 0.0

        def update(robot):
            nonlocal window, overlay, started, next_frame
            if window is None:
                window = stack.enter_context(viewer.launch_passive(robot.m, robot.d))
                window.cam.lookat[:] = [3.55, -.65, .1]
                window.cam.distance = 2.2
                window.cam.azimuth = 100
                window.cam.elevation = -55
                window.opt.geomgroup[3] = 0
                window.opt.sitegroup[:] = 0
                if sensors:
                    from robot_b91.sensors import SensorOverlay
                    overlay = SensorOverlay(robot, window)
                    stack.callback(overlay.close)
                started = time.perf_counter()
            if not window.is_running():
                return False
            if robot.d.time >= next_frame:
                if overlay:
                    overlay.update(time.perf_counter())
                window.sync()
                time.sleep(max(0, robot.d.time - (time.perf_counter() - started)))
                next_frame = robot.d.time + 1 / 30
            return True

        result = evaluate(case, seconds, output, observer=update)
        if window is not None and window.is_running():
            print('Search paused. Close the simulation window to exit.', flush=True)
            while window.is_running():
                if overlay:
                    overlay.update(time.perf_counter())
                window.sync()
                time.sleep(1 / 30)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, default='side')
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--output', type=Path, default=ROOT / 'results/corridor_search')
    parser.add_argument('--viewer', action='store_true', help='Watch the search in the interactive simulation')
    parser.add_argument('--sensors', action='store_true', help='Show the onboard camera, IMU and IR readings')
    args = parser.parse_args()
    if args.sensors and not args.viewer:
        parser.error('--sensors requires --viewer')
    args.output.mkdir(parents=True, exist_ok=True)
    result = (run_interactive(args.case, args.seconds, args.output, args.sensors) if args.viewer else
              evaluate(args.case, args.seconds, args.output))
    if not args.viewer and (not result['succeeded'] or not result['inside_and_centered'] or result['collision']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
