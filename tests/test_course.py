"""Full-physics ramp regression; --robust also checks six perturbed conditions."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from types import SimpleNamespace
import numpy as np
from robot_b91.paths import ROOT
from robot_b91.run import Robot, evaluate, load_gait_parameters
from robot_b91.progress import CourseProgress
from robot_b91.gait import rocker_targets


def check_scoring():
    r = Robot(ramp=True)
    course = r.course

    def traverse(bypass=False, reverse=False, exit_clear=True):
        tracker = CourseProgress(r.m, course)
        # Synthetic observations exercise the scorer, not robot dynamics.
        data = SimpleNamespace(qpos=np.array([0.6, 0., 0.1]), time=3., contact=[],
                               geom_xpos=r.d.geom_xpos.copy(), warning=[])
        names = list(reversed(tracker.names)) if reverse else tracker.names
        for name in names:
            surface = r.m.geom(name).id
            data.contact = [SimpleNamespace(geom1=surface, geom2=i) for i in tracker.feet]
            tracker.update(data, 0.)
        if bypass:
            data.contact = [SimpleNamespace(geom1=tracker.floor, geom2=tracker.tread_ids[0],
                                           pos=np.array([1., 0.25, 0.]))]
            tracker.update(data, 0.)
        data.contact = []
        data.qpos[0] = 3.7
        data.geom_xpos[tracker.tread_ids, 0] = 3.6 if exit_clear else 3.3
        tracker.update(data, 0.)
        return tracker.report()["course_completed"]

    assert traverse()
    assert not traverse(bypass=True), "Floor beside the ramp must not count as a pass"
    assert not traverse(reverse=True), "Every foot must visit the panels in order"
    assert not traverse(exit_clear=False), "Rear feet must clear the final descent"


def main(robust=False):
    check_scoring()
    p, _ = load_gait_parameters(ROOT / "reference/b91_gait.toml", "course")
    # The portable command generator cannot ask for travel beyond +/-45 degrees.
    for t in np.linspace(0, 2, 101):
        targets = rocker_targets(p, t, t, heading_error_rad=0.4, lateral_error_m=-0.1)
        assert targets.shape == (8,) and np.isfinite(targets).all()
        assert np.max(np.abs(targets)) <= np.pi / 4
    cases = [("nominal", {}, {})]
    if robust:
        cases += [
            ("start_left_10mm", {}, dict(initial_y_m=0.01)),
            ("start_right_10mm", {}, dict(initial_y_m=-0.01)),
            ("start_yaw_plus_0.05rad", {}, dict(initial_yaw_rad=0.05)),
            ("start_yaw_minus_0.05rad", {}, dict(initial_yaw_rad=-0.05)),
            ("friction_0.7", dict(friction=0.7), {}),
            ("friction_0.9", dict(friction=0.9), {}),
        ]
    if robust:
        with ProcessPoolExecutor(max_workers=4) as pool:
            jobs = [pool.submit(evaluate, p | changes, seconds=120, ramp=True, **start)
                    for _, changes, start in cases]
            reports = [dict(case=case[0], **job.result()) for case, job in zip(cases, jobs)]
    else:
        reports = [dict(case="nominal", **evaluate(p, seconds=120, ramp=True))]
    for result in reports:
        print(f"{result['case']}: completed={result['course_completed']}, "
              f"time={result['duration_s']:.3f}s, tilt={result['max_tilt_deg']:.2f}deg, "
              f"nonfoot contact={result['nonfoot_ground_fraction']:.1%}", flush=True)
    if robust:
        (ROOT / "results/b91_course_validation.json").write_text(json.dumps(reports, indent=2))
    for result in reports:
        assert result["course_completed"], result
        assert not result["fallen"] and result["warnings"] == 0
        assert not result["floor_bypass"] and not result["left_course"]
        assert all(feet == ["FL", "FR", "RL", "RR"] for feet in result["feet_per_surface"].values())
    print("PASS: bounded gait commands, strict scoring and full-course physics.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robust", action="store_true")
    main(parser.parse_args().robust)
