"""Run the corridor simulation."""

import argparse
from contextlib import ExitStack
import json
import math
from collections import deque
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from dashboard.robot_dashboard.corridor import CorridorNavigation
from robot.gait import rocker_targets
from robot.navigation.controller import Navigation
from robot.simulation.run import Robot
from robot.paths import ROOT
from robot.model.course import build_course
from dashboard.robot_dashboard.corridor_walls import IRWallNavigation, load_corridor_settings


def create_robot(params, width, bend_scale=1.0):
    if not math.isfinite(width) or not .30 <= width <= .6:
        raise ValueError('Test corridor width must be between 0.30 and 0.60 meters.')
    config = json.loads((ROOT / 'reference/b91_course_settings.json').read_text())
    if width == config['corridor']['clear_width_m'] and bend_scale == 1.0:
        return Robot(params=params, ramp=True, feedback='ideal')
    root = E.parse(ROOT / 'models/robot_b10_course.xml').getroot()
    config['corridor']['clear_width_m'] = width
    for point in config['corridor']['centerline_xy_m']:
        point[1] *= bend_scale
    assets, world = E.Element('asset'), E.Element('worldbody')
    build_course(assets, world, config)
    for mesh in assets:
        if mesh.get('name', '').startswith('corridor_wall'):
            root.find(f"asset/mesh[@name='{mesh.get('name')}']").set('vertex', mesh.get('vertex'))
    root.find('compiler').set('meshdir', str(ROOT / 'models/robot_meshes_b10'))
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'corridor.xml'
        E.ElementTree(root).write(path)
        robot = Robot(params=params, ramp=True, feedback='ideal', model_path=path)
        robot.course['corridor'] = config['corridor']
        return robot


def evaluate(seconds=360, offset=0, yaw=0, latency=.05, mismatch=0, noise=0,
             balance_gain=1.0, rate_gain=1.0, max_steering=.4, filter_s=.15,
             yaw_amplitude=None, pitch_stance=None, pitch_lift=None, corridor_gait=False,
             stop_on_contact=True, target_x=None, width=.4, bend_scale=1.0, wall_guard=True,
             observer=None, shared_gait=False, yaw_bias=None, wall_fit=True):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Simulation duration must be a positive number of seconds.')
    corridor_settings = load_corridor_settings()
    if not shared_gait and yaw_amplitude is None:
        yaw_amplitude = corridor_settings['gait']['yaw_amplitude']
    if yaw_bias is None:
        yaw_bias = 0 if shared_gait else corridor_settings['gait']['yaw_bias']
    params = {name: value for name, value in dict(yaw_amplitude=yaw_amplitude,
              pitch_stance=pitch_stance, pitch_lift=pitch_lift).items() if value is not None}
    if corridor_gait:
        params['corridor'] = True
    if yaw_bias:
        params['yaw_bias'] = yaw_bias
    robot = create_robot(params, width, bend_scale)
    try:
        origin = robot.course['corridor']['origin_xy_m']
        start_x = origin[0] - .15
        robot.d.qpos[:2] = [start_x, origin[1] - offset]
        angle = math.pi + yaw
        robot.d.qpos[3:7] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
        mujoco.mj_forward(robot.m, robot.d)
        nav = CorridorNavigation(0, balance_gain=balance_gain, rate_gain=rate_gain,
                                 max_steering=max_steering, filter_s=filter_s, wall_guard=wall_guard)
        if wall_fit:
            sensor_positions = [robot.m.site(name + '_site').pos[:2].copy()
                                for name in ('ir_left', 'ir_right')]
            nav = IRWallNavigation(0, **(corridor_settings['navigation'] | {'sensor_positions': sensor_positions}))
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
        if target_x is None:
            target_x = float(centerline[-1, 0] - .2)
        max_offset = 0.0
        collision = False
        collision_pair = []
        contact_time = contact_streak = max_contact_streak = max_wall_force = 0.0
        rng = np.random.default_rng(4)
        samples = []
        while robot.d.time < seconds:
            if observer is not None and not observer(robot):
                break
            t = robot.d.time
            imu.imu(t, robot.d.sensor('imu_acceleration').data, robot.d.sensor('imu_angular_velocity').data)
            if t >= next_sample:
                distances = [float(robot.d.sensor(n).data[0]) for n in ('ir_left', 'ir_right')]
                calibration = corridor_settings['navigation']
                volts = [100 if d < 0 or d > .30 else min(3300,
                         calibration['ir_offset_mv'][i] + calibration['ir_scale_mv_m'][i] / max(d, .001))
                         for i, d in enumerate(distances)]
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
            projection = np.sum((position - a) * segment, axis=1) / np.sum(segment * segment, axis=1)
            u = np.clip(projection, 0, 1)
            # Keep checking until the rear feet clear the exit.
            u[-1] = max(0, projection[-1])
            distance = np.min(np.linalg.norm(position - (a + u[:, None] * segment), axis=1))
            max_offset = max(max_offset, float(distance))
            if (collision and stop_on_contact) or robot.tilt() > 35 or robot.d.qpos[0] < target_x:
                break
        result = dict(revision=robot.revision, width_m=width, target_x=target_x, completed=bool(robot.d.qpos[0] < target_x), collision=collision, collision_pair=collision_pair,
                      contact_s=round(contact_time, 3), max_contact_streak_s=round(max_contact_streak, 3),
                      max_wall_force_n=round(max_wall_force, 2), elapsed_s=round(robot.d.time, 2), progress_m=round(start_x - robot.d.qpos[0], 3),
                      max_offset_m=round(max_offset, 3), tilt_deg=round(robot.tilt(), 1),
                      reason=nav.reason, final_xy=robot.d.qpos[:2].tolist(), gait_parameters=robot.p,
                      yaw_bias_rad=yaw_bias, guidance='IR wall fits + IMU' if wall_fit else 'IR signal balance + IMU')
        return result, samples
    finally:
        robot.close()


def run_interactive(settings, sensors=False):
    from mujoco import viewer

    with ExitStack() as stack:
        window = None
        overlay = None
        started = next_frame = 0.0

        def update(robot):
            nonlocal window, overlay, started, next_frame
            if window is None:
                window = stack.enter_context(viewer.launch_passive(robot.m, robot.d))
                corridor = robot.course['corridor']
                points = np.asarray(corridor['centerline_xy_m']) + corridor['origin_xy_m']
                window.cam.lookat[:] = [*list((points.min(0) + points.max(0)) / 2), .08]
                window.cam.distance = float(max(2.5, np.ptp(points, axis=0).max() * 1.25))
                window.cam.azimuth = 90
                window.cam.elevation = -55
                window.opt.geomgroup[3] = 0
                window.opt.sitegroup[:] = 0
                if sensors:
                    from robot.simulation.sensors import SensorOverlay
                    overlay = SensorOverlay(robot, window)
                    stack.callback(overlay.close)
                started = time.perf_counter()
                widths_cm = 100 * (corridor['clear_width_m'] + np.asarray(corridor.get('width_offsets_m', [0])))
                gait = ('shared ramp gait' if settings.get('shared_gait') else
                        'shorter ramp gait')
                if robot.p.get('corridor'):
                    gait = 'corridor gait'
                print(f"Corridor width: {widths_cm.min():.0f}–{widths_cm.max():.0f} cm. Using {gait}.", flush=True)
            if not window.is_running():
                return False
            if robot.d.time >= next_frame:
                if overlay:
                    overlay.update(time.perf_counter())
                window.sync()
                time.sleep(max(0, robot.d.time - (time.perf_counter() - started)))
                next_frame = robot.d.time + 1 / 30
            return True

        result, samples = evaluate(**settings, observer=update)
        if window is not None and window.is_running():
            print(f"Simulation paused. Finished: {result['completed']}. Wall contact: {result['collision']}. "
                  "Close the window to exit.", flush=True)
            while window.is_running():
                if overlay:
                    overlay.update(time.perf_counter())
                window.sync()
                time.sleep(1 / 30)
        return result, samples


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        epilog='Balance/rate/filter/wall-guard options apply to --no-wall-fit.')
    for name, default in [('seconds', 360), ('offset', 0), ('yaw', 0), ('latency', .05),
                          ('mismatch', 0), ('noise', 0), ('filter_s', .15), ('yaw_amplitude', None),
                          ('pitch_stance', None), ('pitch_lift', None), ('balance_gain', 1),
                          ('rate_gain', 1), ('max_steering', .4), ('target_x', None), ('width', .4), ('bend_scale', 1),
                          ('yaw_bias', None)]:
        ap.add_argument('--' + name.replace('_', '-'), type=float, default=default)
    ap.add_argument("--corridor-gait", action=argparse.BooleanOptionalAction, default=False,
                    help="Use the alternative corridor steering gait instead of the shared ramp gait")
    ap.add_argument('--shared-gait', action='store_true',
                    help='Use the unchanged ramp strides and walking stance for comparison')
    ap.add_argument('--wall-fit', action=argparse.BooleanOptionalAction, default=True,
                    help='Estimate wall direction and offset from calibrated simulated IR readings and IMU')
    ap.add_argument("--test", action="store_true", help="Run without opening the interactive viewer")
    ap.add_argument("--sensors", action="store_true", help="Show the camera, IMU and side IR readings")
    ap.add_argument("--wall-guard", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--stop-on-contact", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--output", type=Path, help="Save settings, result and trace as JSON")
    args = vars(ap.parse_args())
    output = args.pop("output")
    headless = args.pop("test")
    sensors = args.pop("sensors")
    if sensors and headless:
        ap.error("--sensors requires the interactive simulation")
    result, samples = evaluate(**args) if headless else run_interactive(args, sensors=sensors)
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(dict(settings=args, result=result, samples=samples), indent=2))
    print(f"Finished: {result['completed']}. Wall contact: {result['collision']}.")
    if headless and (not result['completed'] or result['collision']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
