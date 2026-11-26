"""Headless crossing trials on printed feet and 10 mm of foam over a rigid deck.

Heights are finished surface elevations. Friction and contact compliance are
sensitivity assumptions. Terrain position is used for placement and scoring.
The controller receives body attitude at the existing servo command rate.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from dashboard.robot_dashboard.crest import CREST_WAVE_GAIT
from robot_b91.course import build_course
from robot_b91.gait import (FOAM_CREST_GAIT, LEGS, RAMP_SPEED_SCALE, RAMP_STEERING,
                            FoamCrestController, rocker_targets)
from robot_b91.paths import ROOT
from robot_b91.run import Robot, load_gait_parameters


RUN_M = .45
FOAM_M = .01
BASE_M = .01 + FOAM_M
SETTLE_S = 2.
CLEARANCE_M = .22
FINISH_HOLD_S = .2
LOAD_N = .05
MAX_ANGLE_DEG = 12.
MAX_LATERAL_M = .04
MAX_SINK_M = FOAM_M + .0015
STEER_TRIM = .01
GAITS = {'baseline': CREST_WAVE_GAIT, 'foam': FOAM_CREST_GAIT}


@dataclass(frozen=True)
class Placement:
    name: str
    before_peak_m: float
    offset_m: float
    yaw_deg: float
    friction: float
    speed: float
    softness_s: float
    run_m: float = RUN_M
    pitch_error_deg: float = 0.


PLACEMENTS = (
    Placement('center', .02, 0., 0., 1.1, .5, .025),
    Placement('left', .05, .008, 2., .9, .65, .035),
    Placement('right', .05, -.008, -2., 1.3, .5, .018),
)
VALIDATION_PLACEMENTS = (
    Placement('approach', .095, -.004, 1.5, 1.2, .55, .022, .50, 1.),
    Placement('ridge', 0., .010, 3., 1., .60, .032, .40, -1.),
    Placement('offset', .035, -.012, -3., 1.05, .58, .027, .47, .6),
)
SUITES = {'standard': ([10., 14., 18., 22.5], PLACEMENTS),
          'validation': ([12., 16., 20., 21.5, 22.5], VALIDATION_PLACEMENTS)}


def make_model(path, height_m, run_m=RUN_M):
    """Use a sharp symmetric crest with a fixed run, so height changes steepness."""
    tree = ET.parse(ROOT / 'models/robot_b10.xml')
    xml = tree.getroot()
    xml.find('compiler').set('meshdir', str(ROOT / 'models/robot_meshes_b10'))
    world = xml.find('worldbody')
    # Use the printed legs' existing collision meshes and the measured robot inertia.
    for body in world.iter('body'):
        for geom in body.findall('geom'):
            if '_tread_' in geom.get('name', ''):
                body.remove(geom)
    build_course(xml.find('asset'), world, dict(
        description='Foam-covered crest', width_m=.29, panel_thickness_m=FOAM_M,
        corner_rounding_m=0., origin_xy_m=[0., 0.], yaw_deg=0.,
        profile_xz_m=[[0., BASE_M], [run_m, height_m], [2*run_m, BASE_M]],
        segment_names=['uphill', 'downhill'], corridor={'enabled': False}))
    # A normal separation of 10 mm is a larger vertical separation on a slope.
    backing_depth = FOAM_M * math.hypot(1., (height_m - BASE_M) / run_m)
    for name in ('uphill', 'downhill'):
        skin = world.find(f"geom[@name='{name}']")
        skin.set('priority', '1')
        skin.set('solimp', '.75 .97 .01 .5 2')
        board = deepcopy(skin)
        board.set('name', name + '_board')
        board.set('pos', f'0 0 {-backing_depth}')
        board.set('solref', '.004 1')
        board.set('solimp', '.98 .999 .001 .5 2')
        world.append(board)
    tree.write(path, encoding='utf-8')


def run_trial(path, height_m, start, seconds, gait):
    climb = load_gait_parameters() | RAMP_STEERING
    params = climb | GAITS[gait]
    params['period'] /= RAMP_SPEED_SCALE
    params['friction'] = start.friction
    robot = Robot(params, feedback='ideal', model_path=path)
    m, d = robot.m, robot.d
    run_m = start.run_m
    foam_ids = [m.geom(name).id for name in ('uphill', 'downhill')]
    m.geom_solref[foam_ids, 0] = start.softness_s
    slope = (height_m - BASE_M) / run_m
    x = run_m - start.before_peak_m
    # Start aligned with the span between the front and rear supports. Give
    # neutral feet two seconds to settle before enabling the crest gait.
    def surface(x_pos):
        return height_m - abs(x_pos - run_m) * slope
    pitch = -math.atan2(surface(x + .10) - surface(x - .10), .20)
    yaw = math.radians(start.yaw_deg)
    base_clearance = d.qpos[2]
    d.qpos[:3] = [x, start.offset_m, surface(x) + base_clearance / math.cos(pitch) + .003]
    mujoco.mju_euler2Quat(d.qpos[3:7], np.array([yaw, pitch, 0.]), 'zyx')
    mujoco.mj_forward(m, d)

    platform = m.body('platform').id
    foot_bodies = {m.body(f'{leg}_pitch').id for leg in LEGS}
    downhill, floor = m.geom('downhill').id, m.geom('floor').id
    planted_downhill = set()
    past_since = None
    max_roll = max_heading = max_lateral = max_sink = 0.
    reason = 'timeout'
    target = np.tile([0., params['pitch_stance']], 4)
    controller = FoamCrestController(params, climb) if gait == 'foam' else None
    max_x = x
    # Inspect the state at physics rate and update the command at the servo rate.
    for _ in range(math.ceil((SETTLE_S + seconds) / m.opt.timestep)):
        rotation = d.xmat[platform].reshape(3, 3)
        heading = math.atan2(rotation[1, 0], rotation[0, 0])
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        max_roll = max(max_roll, abs(math.degrees(roll)))
        max_heading = max(max_heading, abs(math.degrees(heading)))
        max_lateral = max(max_lateral, abs(d.qpos[1]))
        max_x = max(max_x, d.qpos[0])
        if any(w.number for w in d.warning) or not np.isfinite(d.qpos).all():
            reason = 'solver failure'
            break
        if max_roll > MAX_ANGLE_DEG or max_heading > MAX_ANGLE_DEG or max_lateral > MAX_LATERAL_M:
            reason = ('roll' if max_roll > MAX_ANGLE_DEG else
                      'heading' if max_heading > MAX_ANGLE_DEG else 'lateral')
            break
        pairs = d.contact.geom
        if len(pairs):
            foam_contact = ((pairs == foam_ids[0]) | (pairs == foam_ids[1])).any(axis=1)
            if foam_contact.any():
                max_sink = max(max_sink, -min(d.contact.dist[foam_contact]))
        if max_sink > MAX_SINK_M:
            reason = 'backing penetration'
            break
        active = d.contact.efc_address >= 0
        # With this model's elliptic friction cone the first row is normal force.
        loaded = pairs[active][d.efc_force[d.contact.efc_address[active]] > LOAD_N]
        if np.any(loaded == floor):
            reason = 'floor contact'
            break
        elapsed = d.time - SETTLE_S
        if elapsed >= 0.:
            supports = set()
            for terrain, other in ((0, 1), (1, 0)):
                touching = loaded[loaded[:, terrain] == downhill, other]
                supports.update(m.geom_bodyid[touching])
            planted_downhill.update(supports & foot_bodies)
            if d.qpos[0] >= run_m + CLEARANCE_M:
                if past_since is None:
                    past_since = d.time
            else:
                past_since = None
            if (past_since is not None and d.time - past_since >= FINISH_HOLD_S
                    and planted_downhill == foot_bodies and len(supports & foot_bodies) >= 2
                    and not np.any(m.geom_bodyid[loaded] == platform)):
                reason = 'crossed'
                break
            if d.time + 1e-10 >= robot.next_command_time:
                # The installed controller applies +0.01 steering trim after its gain.
                if controller is not None:
                    pitch = math.asin(max(-1., min(1., -rotation[2, 0]))) + math.radians(start.pitch_error_deg)
                    target = controller.targets(elapsed, elapsed * start.speed, pitch, heading,
                                                robot.joint_ranges, trim=STEER_TRIM)
                else:
                    target = rocker_targets(params, elapsed, elapsed*start.speed,
                                            heading + STEER_TRIM / params['heading_gain'], 0.,
                                            robot.joint_ranges)
        robot.step_targets(target)
    motion_s = max(0., d.time - SETTLE_S)
    robot.close()
    return dict(height_m=height_m, slope_deg=math.degrees(math.atan(slope)),
                **asdict(start), outcome=reason, motion_s=round(motion_s, 3),
                max_x_past_crest_m=round(max_x - run_m, 4), max_roll_deg=round(max_roll, 2),
                max_heading_deg=round(max_heading, 2), max_lateral_m=round(max_lateral, 4),
                max_sink_mm=round(1000 * max_sink, 2))


def scenarios(suites, heights_cm, samples, seed):
    rng = random.Random(seed)
    for suite in suites:
        if suite == 'random':
            for i in range(samples):
                cm = rng.choice(heights_cm) if heights_cm else rng.uniform(10., 22.5)
                start = Placement(f'random-{i+1}', rng.uniform(0., .095),
                                  rng.uniform(-.012, .012), rng.uniform(-3., 3.),
                                  rng.uniform(.9, 1.3), rng.uniform(.5, .65),
                                  rng.uniform(.018, .035), rng.uniform(.40, .55), rng.uniform(-1., 1.))
                yield suite, cm / 100., start
        else:
            default_heights, placements = SUITES[suite]
            for cm in heights_cm or default_heights:
                for start in placements:
                    yield suite, cm / 100., start


def sweep(cases, seconds, workers, gaits):
    results = []
    with TemporaryDirectory() as directory, ProcessPoolExecutor(max_workers=workers) as pool:
        jobs = []
        models = {}
        for suite, height, start in cases:
            geometry = (height, start.run_m)
            if geometry not in models:
                path = Path(directory) / f'crest-{len(models)}.xml'
                make_model(path, *geometry)
                models[geometry] = path
            for label in gaits:
                job = pool.submit(run_trial, models[geometry], height, start, seconds, label)
                jobs.append((suite, label, job))
        for suite, label, job in jobs:
            row = dict(suite=suite, gait=label, **job.result())
            results.append(row)
            print(f"{label:8}  {100*row['height_m']:5.1f} cm  {row['name']:8}  {row['outcome']:14}"
                  f"  {row['motion_s']:6.2f} s  x past crest {row['max_x_past_crest_m']:+.3f} m",
                  flush=True)
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--heights-cm', type=float, nargs='+', help='Override finished crest heights')
    parser.add_argument('--suite', choices=[*SUITES, 'random', 'all'], default='standard')
    parser.add_argument('--seed', type=int, default=20261126, help='Seed for randomized trials')
    parser.add_argument('--samples', type=int, default=24, help='Number of randomized placements')
    parser.add_argument('--gait', choices=[*GAITS, 'both'], default='foam')
    parser.add_argument('--seconds', type=float, default=40., help='Motion time after initial settling')
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    suites = [*SUITES, 'random'] if args.suite == 'all' else [args.suite]
    gaits = list(GAITS) if args.gait == 'both' else [args.gait]
    if args.samples < 1 or args.workers < 1 or not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('Seconds, worker count and sample count must be positive.')
    cases = list(scenarios(suites, args.heights_cm, args.samples, args.seed))
    if any(not BASE_M < h < BASE_M + start.run_m for _, h, start in cases):
        parser.error('Choose finite heights above the base and below a 45-degree slope.')
    rows = sweep(cases, args.seconds, args.workers, gaits)
    for suite in suites:
        for label in gaits:
            group = [row for row in rows if row['gait'] == label and row['suite'] == suite]
            print(f"{suite} {label}: {sum(row['outcome'] == 'crossed' for row in group)}/{len(group)} crossed")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        report = dict(mujoco=mujoco.__version__, random_seed=args.seed, foam_thickness_m=FOAM_M, base_height_m=BASE_M,
                      feedback='ideal body attitude with per-trial pitch bias', steering_trim=STEER_TRIM,
                      friction_and_compliance='sensitivity assumptions, not measured',
                      settle_s=SETTLE_S, motion_budget_s=args.seconds, clearance_m=CLEARANCE_M,
                      finish_hold_s=FINISH_HOLD_S, contact_min_n=LOAD_N, max_sink_m=MAX_SINK_M,
                      max_roll_deg=MAX_ANGLE_DEG, max_heading_deg=MAX_ANGLE_DEG, max_lateral_m=MAX_LATERAL_M,
                      gaits={label: GAITS[label] for label in gaits}, trials=rows)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    evaluated = 'foam' if 'foam' in gaits else 'baseline'
    return int(any(row['outcome'] != 'crossed' for row in rows if row['gait'] == evaluated))


if __name__ == '__main__':
    raise SystemExit(main())
