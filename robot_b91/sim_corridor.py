import json
import math
from collections import deque
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from dashboard.robot_dashboard.corridor import CorridorNavigation
from robot_b91.gait import rocker_targets
from robot_b91.navigation import Navigation
from robot_b91.run import Robot
from robot_b91.paths import ROOT
from robot_b91.course import build_course


def create_robot(params, width, bend_scale=1.0):
    if width == .4 and bend_scale == 1.0:
        return Robot(params=params, ramp=True, feedback='ideal')
    if not .30 <= width <= .6:
        raise ValueError('Test corridor width must be between 0.30 and 0.60 meters.')
    root = E.parse(ROOT / 'models/robot_b98_course.xml').getroot()
    config = json.loads((ROOT / 'reference/b91_course_settings.json').read_text())
    config['corridor']['clear_width_m'] = width
    for point in config['corridor']['centerline_xy_m']:
        point[1] *= bend_scale
    assets, world = E.Element('asset'), E.Element('worldbody')
    build_course(assets, world, config)
    for mesh in assets:
        if mesh.get('name', '').startswith('corridor_wall'):
            root.find(f"asset/mesh[@name='{mesh.get('name')}']").set('vertex', mesh.get('vertex'))
    root.find('compiler').set('meshdir', str(ROOT / 'models/robot_meshes_b98'))
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'corridor.xml'
        E.ElementTree(root).write(path)
        robot = Robot(params=params, ramp=True, feedback='ideal', model_path=path)
        robot.course['corridor'] = config['corridor']
        return robot


def evaluate(seconds=120, offset=0, yaw=0, latency=.05, mismatch=0, noise=0,
             balance_gain=1.0, rate_gain=1.0, max_steering=.4, filter_s=.15,
             yaw_amplitude=.55, pitch_stance=0.0, pitch_lift=.4, corridor_gait=True,
             stop_on_contact=True, target_x=.85, width=.4, bend_scale=1.0, wall_guard=True):
    robot = create_robot(dict(yaw_amplitude=yaw_amplitude, pitch_stance=pitch_stance,
                              pitch_lift=pitch_lift, corridor=corridor_gait), width, bend_scale)
    robot.d.qpos[:2] = [3.4, -.65 - offset]
    angle = math.pi + yaw
    robot.d.qpos[3:7] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
    mujoco.mj_forward(robot.m, robot.d)
    nav = CorridorNavigation(0, balance_gain=balance_gain, rate_gain=rate_gain,
                             max_steering=max_steering, filter_s=filter_s, wall_guard=wall_guard)
    imu = Navigation()
    queue = deque()
    next_sample = gait_time = active_time = 0.0
    ready = False
    command_heading = reference_heading = 0.0
    target = robot.command_target.copy()
    wall_ids = {i for i in range(robot.m.ngeom)
                if 'corridor_wall' in (robot.m.geom(i).name or '')}
    if not wall_ids:
        raise ValueError('The course model has no corridor walls.')
    centerline = np.asarray(robot.course['corridor']['centerline_xy_m']) + robot.course['corridor']['origin_xy_m']
    max_offset = 0.0
    collision = False
    collision_pair = []
    contact_time = contact_streak = max_contact_streak = max_wall_force = 0.0
    rng = np.random.default_rng(4)
    samples = []
    while robot.d.time < seconds:
        t = robot.d.time
        imu.imu(t, robot.d.sensor('imu_acceleration').data, robot.d.sensor('imu_angular_velocity').data)
        if t >= next_sample:
            distances = [float(robot.d.sensor(n).data[0]) for n in ('ir_left', 'ir_right')]
            volts = [100 if d < 0 or d > .30 else min(3300, 120 / max(d, .001)) for d in distances]
            volts = [max(1, min(3300, v * (1 + (mismatch if i == 0 else -mismatch)) + rng.normal(0, noise)))
                     for i, v in enumerate(volts)]
            packet = dict(irMillivolts=volts, irAgeMs=0, orientationValid=imu.initialized,
                          imuAgeMs=0, orientation=[imu.roll, imu.pitch, imu.heading])
            queue.append((t + latency, packet))
            next_sample += .05
        while queue and queue[0][0] <= t:
            _, packet = queue.popleft()
            ready = nav.update(packet, t)
            command_heading, reference_heading = nav.estimate.heading, nav.imu_heading
            if len(samples) == 0 or t - samples[-1][0] >= .5:
                samples.append([round(t, 3), *robot.d.qpos[:2].tolist(), *packet['irMillivolts'], command_heading, ready, imu.heading])
        if t > 2 and ready:
            active_time += robot.m.opt.timestep
            gait_time += robot.m.opt.timestep
            target = rocker_targets(robot.p, active_time, gait_time,
                                    command_heading + imu.heading - reference_heading, nav.estimate.lateral, robot.joint_ranges)
        robot.step_targets(target)
        touching = False
        for index, contact in enumerate(robot.d.contact):
            if int(contact.geom1) in wall_ids or int(contact.geom2) in wall_ids:
                touching = collision = True
                collision_pair = [robot.m.geom(int(g)).name for g in contact.geom]
                force = np.zeros(6)
                mujoco.mj_contactForce(robot.m, robot.d, index, force)
                max_wall_force = max(max_wall_force, float(np.linalg.norm(force[:3])))
        if touching:
            contact_time += robot.m.opt.timestep
            contact_streak += robot.m.opt.timestep
            max_contact_streak = max(max_contact_streak, contact_streak)
        else:
            contact_streak = 0.0
        position = robot.d.qpos[:2]
        a, segment = centerline[:-1], np.diff(centerline, axis=0)
        u = np.clip(np.sum((position - a) * segment, axis=1) / np.sum(segment * segment, axis=1), 0, 1)
        distance = np.min(np.linalg.norm(position - (a + u[:, None] * segment), axis=1))
        max_offset = max(max_offset, float(distance))
        if (collision and stop_on_contact) or robot.tilt() > 35 or robot.d.qpos[0] < target_x:
            break
    result = dict(width_m=width, target_x=target_x, completed=bool(robot.d.qpos[0] < target_x), collision=collision, collision_pair=collision_pair,
                  contact_s=round(contact_time, 3), max_contact_streak_s=round(max_contact_streak, 3),
                  max_wall_force_n=round(max_wall_force, 2), elapsed_s=round(robot.d.time, 2), progress_m=round(3.4 - robot.d.qpos[0], 3),
                  max_offset_m=round(max_offset, 3), tilt_deg=round(robot.tilt(), 1),
                  reason=nav.reason, final_xy=robot.d.qpos[:2].tolist())
    robot.close()
    return result, samples


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name, default in [('seconds', 120), ('offset', 0), ('yaw', 0), ('latency', .05),
                          ('mismatch', 0), ('noise', 0), ('filter_s', .15), ('yaw_amplitude', .55),
                          ('pitch_stance', 0), ('pitch_lift', .4), ('balance_gain', 1),
                          ('rate_gain', 1), ('max_steering', .4), ('target_x', .85), ('width', .4), ('bend_scale', 1)]:
        ap.add_argument('--' + name.replace('_', '-'), type=float, default=default)
    ap.add_argument("--corridor-gait", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--wall-guard", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--stop-on-contact", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--output", type=Path, help="Save settings, result and trace as JSON")
    args = vars(ap.parse_args())
    output = args.pop("output")
    result, samples = evaluate(**args)
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(dict(settings=args, result=result, samples=samples), indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
