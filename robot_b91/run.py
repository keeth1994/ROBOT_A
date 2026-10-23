"""B9-1 torque/speed-limited rocker gait experiment. No imposed base forces."""

import argparse, json, math, time
import tomllib
from pathlib import Path
import numpy as np
import mujoco
from robot_b91.paths import ROOT
from robot_b91 import servo_properties
from robot_b91.course import local_points
from robot_b91.progress import CourseProgress
from robot_b91.gait import rocker_targets

LEGS = ["FL", "RL", "FR", "RR"]
GAIT_PROFILES = ("flat", "slope", "transition", "course")
DEFAULT = dict(
    period=1.8,
    duty=0.75,
    yaw_amplitude=0.22,
    pitch_lift=0.65,
    pitch_stance=0.0,
    kp=1.6,
    kd=0.04,
    heading_gain=0.8,
    pattern="crawl",
)


def read_gait_config(path):
    if path.suffix == ".toml":
        with path.open("rb") as f:
            return tomllib.load(f)
    return json.loads(path.read_text())


def load_gait_parameters(path, profile=None):
    """Load one gait profile, retaining support for legacy flat JSON files."""
    if not path.exists():
        if profile not in (None, "flat"):
            raise ValueError(f"Cannot select gait profile {profile!r}: {path} is missing")
        return DEFAULT.copy(), "flat"

    config = read_gait_config(path)

    profiles = config.get("profiles")
    if profiles is None:
        if profile is not None:
            raise ValueError(
                "--gait-profile requires a configuration containing profiles"
            )
        return config, None

    selected = profile or config.get("default_profile", "flat")
    if selected not in profiles:
        available = ", ".join(sorted(profiles))
        raise ValueError(f"Unknown gait profile {selected!r}; choose from: {available}")
    return profiles[selected], selected


class GaitControlPanel:

    def __init__(self, profiles, active_profile):
        import tkinter as tk
        from tkinter import ttk

        self._tk = tk
        self._closed = False
        self._pending_profile = None
        self.root = tk.Tk()
        self.root.title("B9-1 gait controls")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(self.root, padding=12)
        frame.grid()
        ttk.Label(frame, text="Select gait profile").grid(
            row=0, column=0, columnspan=len(profiles), pady=(0, 8)
        )
        for column, profile in enumerate(profiles):
            ttk.Button(
                frame,
                text=profile.title(),
                width=12,
                command=lambda name=profile: self._select(name),
            ).grid(row=1, column=column, padx=3)

        self._active_text = tk.StringVar()
        ttk.Label(frame, textvariable=self._active_text).grid(
            row=2, column=0, columnspan=len(profiles), pady=(10, 0)
        )
        self.set_active(active_profile)

    def _select(self, profile):
        self._pending_profile = profile

    def poll(self):
        """Process UI events and return a newly requested profile, if any."""
        if self._closed:
            return None
        try:
            self.root.update_idletasks()
            self.root.update()
        except self._tk.TclError:
            self._closed = True
            return None
        requested = self._pending_profile
        self._pending_profile = None
        return requested

    def set_active(self, profile):
        self._active_text.set(f"Active: {profile.title()}")

    def close(self):
        if not self._closed:
            self._closed = True
            self.root.destroy()


class Robot:
    def __init__(self, params=None, ramp=False, feedback="sensors"):
        if feedback not in ("sensors", "ideal"):
            raise ValueError("feedback must be sensors or ideal")
        self.feedback = feedback
        self.navigation = None
        self.p = DEFAULT | (params or {})
        self.m = mujoco.MjModel.from_xml_path(
            str(ROOT / "models" / ("robot_b91_course.xml" if ramp else "robot_b91.xml"))
        )
        self.d = mujoco.MjData(self.m)
        cfg = json.loads((ROOT / "reference/b91_kinematics.json").read_text())
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
        mujoco.mj_forward(self.m, self.d)
        self.target = np.tile([0.0, self.p["pitch_stance"]], 4)
        self.command_target = self.target.copy()
        self.torque.fill(0)
        self.next_command_time = 0.0
        self.next_ir_time = 0.0
        self.ir_distances_m = np.full(3, np.nan)
        self.gait_time_origin = 2.0
        self._transition_start_time = None
        self._transition_from = None
        self._sample_ir()
        if self.navigation:
            self.navigation.close()
        if self.feedback == "sensors" and self.course:
            from robot_b91.sim_navigation import SimNavigation
            self.navigation = SimNavigation(self.m)

    def close(self):
        if self.navigation:
            self.navigation.close()
            self.navigation = None

    def set_gait_parameters(self, params, transition_seconds=0.5):
        """Change gait while preserving phase and blending away target jumps."""
        if self.d.time > 2:
            phase = (self.d.time - self.gait_time_origin) / self.p["period"]
            self.gait_time_origin = self.d.time - phase * params["period"]
        self._transition_from = self.target.copy()
        self._transition_start_time = self.d.time
        self._transition_seconds = transition_seconds
        self.p = DEFAULT | params
        self.m.geom_friction[:, 0] = self.p.get("friction", 0.8)

    def step(self, walk=True):
        t = self.d.time - 2
        p = self.p
        self.target = np.tile([0.0, p["pitch_stance"]], 4)
        sensor_errors = self.navigation.update(self.d) if self.navigation else None
        if walk and t > 0 and self.navigation and not self.navigation.estimate.ready(self.d.time):
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
            self.target = rocker_targets(
                p, t, self.d.time - self.gait_time_origin,
                heading, lateral, self.joint_ranges,
            )
        if self._transition_start_time is not None:
            progress = (self.d.time - self._transition_start_time) / max(
                self._transition_seconds, self.m.opt.timestep
            )
            if progress >= 1:
                self._transition_start_time = None
                self._transition_from = None
            else:
                blend = progress * progress * (3 - 2 * progress)
                self.target = self._transition_from + blend * (
                    self.target - self._transition_from
                )
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
        mujoco.mj_step(self.m, self.d)
        self._sample_ir()

    def _sample_ir(self):
        if self.d.time + 1e-10 >= self.next_ir_time:
            # Sharp GP2Y0A41SK0F: nominal 16.5 ms period, valid between 4 and 30 cm.
            raw = np.array(
                [self.d.sensor(n).data[0] for n in ["ir_left", "ir_right", "ir_ground"]]
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
             observer=None, feedback="sensors"):
    if not math.isfinite(seconds) or seconds <= 2:
        raise ValueError("Evaluation duration must exceed the 2-second settling period")
    terrain = json.loads((ROOT / "reference/b91_course.json").read_text())
    surfaces = terrain["required_surfaces"]
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
        while r.d.time < seconds:
            r.step(walk)
            if r.d.time >= 2:
                if start is None:
                    start = r.d.qpos[:3].copy()
                tilt = max(tilt, r.tilt())
                sat += int(np.count_nonzero(np.abs(r.torque) > 0.95 * r.limit))
                n += 1
                if progress:
                    progress.update(r.d, r.tilt())
                for c in r.d.contact:
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
            if not np.isfinite(r.d.qpos).all():
                raise RuntimeError("Non-finite physics state")
            if observer:
                observer(r, progress)
            if progress and progress.completed_at is not None:
                break
        delta = r.d.qpos[:3] - start
        result = dict(
            revision="B9-1",
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
        return result
    finally:
        r.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--hold", action="store_true")
    ap.add_argument("--feedback", choices=("sensors", "ideal"), default="sensors",
                    help="ramp navigation source (default: camera and IMU)")
    ap.add_argument("--ramp", action="store_true")
    ap.add_argument("--seconds", type=float, help="simulation duration (default: 120 on course, 20 on flat)")
    ap.add_argument("--render")
    ap.add_argument(
        "--sensors", action="store_true",
        help="show the onboard camera and ideal accelerometer/gyroscope readings",
    )
    ap.add_argument("--video", help="save a headless MP4 evaluation at 2x playback speed")
    ap.add_argument("--params")
    ap.add_argument(
        "--gait-profile",
        choices=GAIT_PROFILES,
        help="select a gait (default: course with --ramp, otherwise configuration default)",
    )
    args = ap.parse_args()
    if args.video and args.render:
        ap.error("Choose either --video or --render")
    if args.sensors and (args.test or args.video or args.render):
        ap.error("--sensors requires the interactive simulation")
    if args.seconds is None:
        args.seconds = 120 if args.ramp else 20
    print(
        "B9-1: mixed servos at 5 V; provisional joint assignment/masses; programmed gait."
    )
    pp = Path(args.params) if args.params else ROOT / "reference/b91_gait.toml"
    try:
        profile = args.gait_profile
        if profile is None and args.ramp and args.params is None:
            profile = "course"
        p, selected_profile = load_gait_parameters(pp, profile)
    except ValueError as error:
        ap.error(str(error))
    if selected_profile:
        print(f"Gait profile: {selected_profile}")
    profile_parameters = {}
    if selected_profile and pp.exists():
        profile_parameters = {
            name: parameters
            for name, parameters in read_gait_config(pp).get("profiles", {}).items()
            if name in GAIT_PROFILES
        }
    if args.test or args.video:
        recording = None
        if args.video:
            from robot_b91.recording import Recording
            recording = Recording(Path(args.video))
        try:
            result = evaluate(p, args.seconds, not args.hold, args.ramp, observer=recording, feedback=args.feedback)
        finally:
            if recording:
                recording.close()
        name = "b91_" + ("ramp" if args.ramp else "hold" if args.hold else "walk")
        (ROOT / "results" / f"{name}.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
        if args.ramp and not args.hold and not result["course_completed"]:
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
        controls = None
        if profile_parameters:
            try:
                controls = GaitControlPanel(tuple(profile_parameters), selected_profile)
            except Exception as error:
                print(f"Gait control window unavailable: {error}")
        sensors = None
        next_control_poll = 0.0
        try:
            if args.sensors:
                from robot_b91.sensors import SensorOverlay

                sensors = SensorOverlay(r, v)
            while v.is_running():
                t = time.perf_counter()
                if controls and t >= next_control_poll:
                    requested_profile = controls.poll()
                    next_control_poll = t + 1 / 30
                    if requested_profile and requested_profile != selected_profile:
                        r.set_gait_parameters(profile_parameters[requested_profile])
                        selected_profile = requested_profile
                        controls.set_active(selected_profile)
                        print(f"Gait profile: {selected_profile}")
                if r.d.time < args.seconds and not (progress and progress.completed_at is not None):
                    r.step(not args.hold)
                    if progress:
                        progress.update(r.d, r.tilt())
                        if progress.completed_at is not None:
                            print(f"Course completed in {progress.completed_at:.2f} simulated seconds; simulation paused.")
                if not args.ramp:
                    v.cam.lookat[:2] = r.d.qpos[:2]
                if sensors:
                    sensors.update(t)
                v.sync()
                time.sleep(max(0, r.m.opt.timestep - (time.perf_counter() - t)))
        finally:
            if sensors:
                sensors.close()
            if controls:
                controls.close()
            r.close()


if __name__ == "__main__":
    main()
