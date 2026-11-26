"""B10 torque/speed-limited rocker gait experiment. No imposed base forces."""

import argparse, json, math, time
import tomllib
from pathlib import Path
import numpy as np
import mujoco
from robot_b91.paths import ROOT
from robot_b91 import servo_properties
from robot_b91.course import local_points
from robot_b91.progress import CourseProgress, corridor_entry_score
from robot_b91.gait import RAMP_SPEED_SCALE, rocker_targets
from robot_b91.camera_calibration import load_camera_scan_settings

LEGS = ["FL", "RL", "FR", "RR"]


def load_gait_parameters(path=None):
    """Load course parameters, or a custom TOML/JSON parameter file."""
    path = Path(path) if path is not None else ROOT / "reference/b91_gait.toml"
    if path.suffix == ".toml":
        with path.open("rb") as stream:
            return tomllib.load(stream)
    return json.loads(path.read_text())


class Robot:
    def __init__(self, params=None, ramp=False, feedback="sensors", model_path=None):
        if feedback not in ("sensors", "ideal"):
            raise ValueError("feedback must be sensors or ideal")
        self.feedback = feedback
        self.navigation = None
        self.p = load_gait_parameters() | (params or {})
        self.m = mujoco.MjModel.from_xml_path(
            str(model_path or ROOT / "models" / ("robot_b10_course.xml" if ramp else "robot_b10.xml"))
        )
        self.d = mujoco.MjData(self.m)
        cfg = json.loads((ROOT / "reference/b10_kinematics.json").read_text())
        self.revision = cfg["cad_revision"]
        self.camera_scan = load_camera_scan_settings()
        self.scan_enabled = bool(ramp and feedback == "sensors" and self.camera_scan["enabled"])
        self.camera_qa = self.m.joint("camera_cradle").qposadr[0]
        self.camera_da = self.m.joint("camera_cradle").dofadr[0]
        self.camera_actuator = self.m.actuator("camera_tilt").id
        self.camera_limit = cfg["servos"]["Parallax"]["stall_Nm"] / 2
        self.camera_speed = math.radians(cfg["servos"]["Parallax"]["free_speed_deg_s"] * 2)
        self.names = [l + "_" + s for l in LEGS for s in ["yaw", "pitch"]]
        js = [self.m.joint(n).id for n in self.names]
        self.qa = self.m.jnt_qposadr[js]
        self.da = self.m.jnt_dofadr[js]
        self.joint_ranges = self.m.jnt_range[js].copy()
        self.course = (
            json.loads((ROOT / "reference/b91_course.json").read_text()) if ramp else None
        )
        self.ai = [self.m.actuator(n).id for n in self.names]
        ratings = np.array([servo_properties(cfg, n) for n in self.names])
        self.limit = ratings[:, 0]
        self.speed = np.radians(ratings[:, 1])
        if not np.allclose(
            self.m.actuator_ctrlrange[self.ai],
            np.column_stack((-self.limit, self.limit)),
            rtol=1e-8,
            atol=1e-10,
        ):
            raise ValueError(
                "Hardware torque limits differ from XML; run python robot.py build"
            )
        self.command_period = 1 / cfg["servo_command_hz"]
        self.m.geom_friction[:, 0] = self.p.get("friction", 0.8)
        self.target = np.zeros(8)
        self.torque = np.zeros(8)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.qa[1::2]] = self.p["pitch_stance"]
        if self.scan_enabled:
            angle = math.radians(self.camera_scan["forward_deg"])
            for name, ratio in [("camera_cradle", 1), ("camera_input", -.5),
                                ("upper_rocker", -.5), ("camera_link", .5)]:
                self.d.qpos[self.m.joint(name).qposadr[0]] = angle * ratio
        mujoco.mj_forward(self.m, self.d)
        self.target = np.tile([0.0, self.p["pitch_stance"]], 4)
        self.command_target = self.target.copy()
        self.torque.fill(0)
        self.next_command_time = 0.0
        self.next_ir_time = 0.0
        self.ir_distances_m = np.full(2, np.nan)
        self.gait_time_origin = 2.0
        self.search_gait_time = self.search_active_time = 0.0
        self._sample_ir()
        if self.navigation:
            self.navigation.close()
        if self.feedback == "sensors" and self.course:
            from robot_b91.sim_navigation import SimNavigation
            self.navigation = SimNavigation(self.m, self.camera_scan if self.scan_enabled else None)

    def close(self):
        if self.navigation:
            self.navigation.close()
            self.navigation = None

    def step(self, walk=True):
        t = self.d.time - 2
        p = self.p
        self.target = np.tile([0.0, p["pitch_stance"]], 4)
        sensor_errors = self.navigation.update(self.d) if self.navigation else None
        if self.navigation and self.navigation.search:
            search = self.navigation.search
            if walk and self.navigation.search_ready:
                self.search_gait_time += self.m.opt.timestep * search.speed
                self.search_active_time += self.m.opt.timestep
                params = p | self.navigation.search_gait_settings
                heading, lateral = sensor_errors
                self.target = rocker_targets(params, self.search_active_time, self.search_gait_time,
                                             heading, lateral, self.joint_ranges, search.forward)
            else:
                self.target = self.command_target
            self.step_targets(self.target)
            return
        if walk and t > 0 and self.navigation and not self.navigation.estimate.ready(self.d.time):
            if self.scan_enabled:
                self.navigation.estimate.walking = False
            self.gait_time_origin += self.m.opt.timestep
            self.step_targets(self.command_target)
            return
        if walk and t > 0:
            if self.navigation:
                heading, lateral = sensor_errors
            else:
                R = self.d.xmat[self.m.body("platform").id].reshape(3, 3)
                heading = math.atan2(R[1, 0], R[0, 0])
                # Optional ideal lateral position feedback for simulation, not an implemented sensor estimator.
                lateral = self.d.qpos[1]
                if self.course:
                    lateral = local_points(self.d.qpos[:3], self.course)[1]
                    heading -= math.radians(self.course["yaw_deg"])
                    heading = math.atan2(math.sin(heading), math.cos(heading))
            pace = RAMP_SPEED_SCALE if self.course else 1.0
            self.target = rocker_targets(
                p, t * pace, (self.d.time - self.gait_time_origin) * pace,
                heading, lateral, self.joint_ranges,
            )
        if self.navigation and self.scan_enabled:
            self.navigation.estimate.walking = bool(walk and t > 0 and self.navigation.estimate.ready(self.d.time))
        self.step_targets(self.target)

    def step_targets(self, target):
        self.target = np.asarray(target, dtype=float).copy()
        p = self.p
        if self.target.shape != (8,) or not np.isfinite(self.target).all():
            raise ValueError("Expected eight finite joint targets")
        self.target = np.clip(self.target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        if self.d.time + 1e-10 >= self.next_command_time:
            self.command_target = self.target.copy()
            self.next_command_time += self.command_period
        v = self.d.qvel[self.da]
        req = p["kp"] * (self.command_target - self.d.qpos[self.qa]) - p["kd"] * v
        available = self.limit.copy()
        motoring = req * v > 0
        available[motoring] *= np.maximum(
            0, 1 - np.abs(v[motoring]) / self.speed[motoring]
        )
        self.torque = np.clip(req, -available, available)
        self.d.ctrl[self.ai] = self.torque
        camera_target = 0
        if self.scan_enabled and not (self.navigation and self.navigation.search):
            camera_target = (self.navigation.estimate.camera_target(self.d.time) if self.navigation else
                             math.radians(self.camera_scan["forward_deg"]))
        velocity = self.d.qvel[self.camera_da]
        # Servo damping is integrated implicitly in the camera joint for stability.
        requested = 2.3 * (camera_target - self.d.qpos[self.camera_qa])
        available_camera = self.camera_limit
        if requested * velocity > 0:
            available_camera *= max(0, 1 - abs(velocity) / self.camera_speed)
        self.d.ctrl[self.camera_actuator] = np.clip(requested, -available_camera, available_camera)
        mujoco.mj_step(self.m, self.d)
        self._sample_ir()

    def _sample_ir(self):
        if self.d.time + 1e-10 >= self.next_ir_time:
            # Sharp GP2Y0A41SK0F: nominal 16.5 ms period, valid between 4 and 30 cm.
            raw = np.array(
                [self.d.sensor(n).data[0] for n in ["ir_left", "ir_right"]]
            )
            self.ir_distances_m = np.where((raw >= 0.04) & (raw <= 0.30), raw, np.nan)
            self.next_ir_time += 0.0165

    def tilt(self):
        return math.degrees(
            math.acos(
                np.clip(
                    self.d.xmat[self.m.body("platform").id].reshape(3, 3)[2, 2], -1, 1
                )
            )
        )


def evaluate(p=None, seconds=15, walk=True, ramp=False, initial_y_m=0.0, initial_yaw_rad=0.0,
             observer=None, feedback="sensors", find_corridor=False):
    if not math.isfinite(seconds) or seconds <= 2:
        raise ValueError("Evaluation duration must exceed the 2-second settling period")
    if find_corridor and (not ramp or feedback != 'sensors' or not walk):
        raise ValueError('Corridor search needs --ramp, sensor feedback and walking enabled.')
    terrain = json.loads((ROOT / "reference/b91_course.json").read_text())
    surfaces = terrain.get("surface_geoms", terrain["required_surfaces"])
    obstacles = terrain["obstacle_geoms"]
    r = Robot(p, ramp, feedback)
    try:
        if initial_y_m or initial_yaw_rad:
            r.d.qpos[1] = initial_y_m
            r.d.qpos[3:7] = [math.cos(initial_yaw_rad / 2), 0, 0, math.sin(initial_yaw_rad / 2)]
            mujoco.mj_forward(r.m, r.d)
        progress = CourseProgress(r.m, terrain) if ramp else None
        start = None
        tilt = 0
        sat = 0
        n = 0
        bad = 0
        rampcontact = False
        corridor_collision = False
        wall_ids = {i for i in range(r.m.ngeom) if (r.m.geom(i).name or '').startswith('corridor_wall')}
        while r.d.time < seconds:
            r.step(walk)
            if r.d.time >= 2:
                if start is None:
                    start = r.d.qpos[:3].copy()
                tilt = max(tilt, r.tilt())
                sat += int(np.count_nonzero(np.abs(r.torque) > 0.95 * r.limit))
                n += 1
                if progress and progress.completed_at is None:
                    progress.update(r.d, r.tilt())
                for c in r.d.contact:
                    if find_corridor and (c.geom1 in wall_ids or c.geom2 in wall_ids):
                        corridor_collision = True
                    names = [
                        mujoco.mj_id2name(r.m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
                        for g in [c.geom1, c.geom2]
                    ]
                    if any(x in surfaces for x in names):
                        rampcontact = True
                    if any(x in (["floor"] + obstacles) for x in names) and not any(
                        "_tread_" in x for x in names
                    ):
                        bad += 1
                        break
            if any(w.number for w in r.d.warning) or not np.isfinite(r.d.qpos).all():
                raise RuntimeError("Invalid course physics state")
            if observer:
                observer(r, progress)
            if progress and progress.completed_at is not None:
                if not find_corridor:
                    break
                if r.navigation.search is None:
                    r.navigation.begin_corridor_search(r.d.time)
                if r.navigation.search.finished:
                    break
        delta = r.d.qpos[:3] - start
        result = dict(
            revision=r.revision,
            parameters=r.p,
            duration_s=float(r.d.time),
            requested_duration_s=seconds,
            initial_y_m=initial_y_m,
            initial_yaw_rad=initial_yaw_rad,
            walk=walk,
            ramp=ramp,
            forward_m=float(delta[0]),
            sideways_m=float(delta[1]),
            final_height_m=float(r.d.qpos[2]),
            max_tilt_deg=tilt,
            fallen=bool(tilt > 45),
            nonfoot_ground_fraction=bad / max(n, 1),
            stall_saturation_fraction=sat / max(8 * n, 1),
            ramp_contact=rampcontact,
            total_mass_kg=float(sum(r.m.body_mass)),
            torque_limits_Nm=r.limit.tolist(),
            warnings=int(sum(w.number for w in r.d.warning)),
            feedback_source="RGB camera + accelerometer/gyro" if r.navigation else "ideal simulator heading and lateral position",
            navigation=r.navigation.estimate.report() if r.navigation else None,
        )
        if progress:
            result.update(progress.report())
        if find_corridor:
            search = r.navigation.search
            score = corridor_entry_score(r.m, r.d, terrain)
            result['corridor_search'] = search.report() if search else None
            result['corridor_entry'] = score
            result['corridor_collision'] = corridor_collision
            result['corridor_found'] = bool(search and search.succeeded and score['inside_and_centered']
                                           and not corridor_collision and tilt <= 45)
            result['final_position_m'] = r.d.qpos[:3].tolist()
        return result
    finally:
        r.close()


def main():
    ap = argparse.ArgumentParser(epilog="For the IR corridor, run: python robot.py walk --corridor")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--hold", action="store_true")
    ap.add_argument("--feedback", choices=("sensors", "ideal"), default="sensors",
                    help="ramp navigation source (default: camera and IMU)")
    ap.add_argument("--ramp", action="store_true")
    ap.add_argument('--find-corridor', action='store_true',
                    help='Continue after the ramp, find the corridor with camera/IMU/IR, and stop centered')
    ap.add_argument("--seconds", type=float, help="duration (default: 300 with search, 240 on ramp, 20 on flat)")
    ap.add_argument("--render")
    ap.add_argument(
        "--sensors", action="store_true",
        help="show the onboard camera and ideal accelerometer/gyroscope readings",
    )
    ap.add_argument("--video", help="save a headless MP4 evaluation at 2x playback speed")
    ap.add_argument("--params")
    args = ap.parse_args()
    if args.video and args.render:
        ap.error("Choose either --video or --render")
    if args.sensors and (args.test or args.video or args.render):
        ap.error("--sensors requires the interactive simulation")
    if args.find_corridor and (not args.ramp or args.feedback != 'sensors' or args.hold or args.render):
        ap.error('--find-corridor needs --ramp, sensor feedback and walking enabled')
    if args.seconds is None:
        args.seconds = 300 if args.find_corridor else 240 if args.ramp else 20
    print(
        "B10: mixed servos at 5 V; provisional joint assignment/masses; programmed gait."
    )
    try:
        p = load_gait_parameters(args.params)
    except (OSError, ValueError) as error:
        ap.error(str(error))
    if args.test or args.video:
        recording = None
        if args.video:
            from robot_b91.recording import Recording
            recording = Recording(Path(args.video))
        try:
            result = evaluate(p, args.seconds, not args.hold, args.ramp, observer=recording,
                              feedback=args.feedback, find_corridor=args.find_corridor)
        finally:
            if recording:
                recording.close()
        name = "b10_" + ('ramp_to_corridor' if args.find_corridor else
                         "ramp" if args.ramp else "hold" if args.hold else "walk")
        (ROOT / "results").mkdir(exist_ok=True)
        (ROOT / "results" / f"{name}.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
        if args.ramp and not args.hold and not result["course_completed"]:
            raise SystemExit(1)
        if args.find_corridor and not result['corridor_found']:
            raise SystemExit(1)
        return
    r = Robot(p, args.ramp, args.feedback)
    course = json.loads((ROOT / "reference/b91_course.json").read_text())
    progress = CourseProgress(r.m, course) if args.ramp else None
    if args.render:
        from PIL import Image

        for _ in range(1000):
            r.step(False)
        renderer = mujoco.Renderer(r.m, height=900, width=1280)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.08]
        cam.distance = 0.6
        cam.azimuth = 135
        cam.elevation = -25
        opt = mujoco.MjvOption()
        opt.geomgroup[3] = 0
        renderer.update_scene(r.d, camera=cam, scene_option=opt)
        Path(args.render).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(args.render)
        renderer.close()
        r.close()
        return
    from mujoco import viewer

    with viewer.launch_passive(r.m, r.d) as v:
        v.cam.lookat[:] = course["camera_lookat"] if args.ramp else [0, 0, 0.08]
        v.cam.distance = course["camera_distance"] if args.ramp else 0.65
        v.cam.azimuth = 100 if args.ramp else 135
        v.cam.elevation = -38
        v.opt.geomgroup[3] = 0
        sensors = None
        try:
            if args.sensors:
                from robot_b91.sensors import SensorOverlay

                sensors = SensorOverlay(r, v)
            while v.is_running():
                t = time.perf_counter()
                search = r.navigation.search if r.navigation else None
                finished = (search.finished if search else bool(progress and progress.completed_at is not None))
                if r.d.time < args.seconds and not finished:
                    r.step(not args.hold)
                    if progress and progress.completed_at is None:
                        progress.update(r.d, r.tilt())
                        if progress.completed_at is not None:
                            if args.find_corridor:
                                r.navigation.begin_corridor_search(r.d.time)
                                print(f'Ramp completed at {r.d.time:.2f}s; searching for the corridor.')
                            else:
                                print(f"Course completed in {progress.completed_at:.2f} simulated seconds; simulation paused.")
                    if search and search.finished:
                        print(search.reason)
                if not args.ramp:
                    v.cam.lookat[:2] = r.d.qpos[:2]
                if sensors:
                    sensors.update(t)
                v.sync()
                time.sleep(max(0, r.m.opt.timestep - (time.perf_counter() - t)))
        finally:
            if sensors:
                sensors.close()
            r.close()


if __name__ == "__main__":
    main()
