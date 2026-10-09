"""B9-1 transfer checks: provenance, links, axes, mass, contacts and motor limits."""

import json, hashlib
import numpy as np
import mujoco
from robot_b91.paths import ROOT
from robot_b91.geometry import rz
from robot_b91.run import GAIT_PROFILES, Robot, load_gait_parameters


def main():
    gait_path = ROOT / "reference/b91_gait.toml"
    default_gait, default_profile = load_gait_parameters(gait_path)
    assert default_profile == "flat" and default_gait["pattern"] == "trot"
    for profile in GAIT_PROFILES:
        gait, selected = load_gait_parameters(gait_path, profile)
        assert selected == profile and 0 < gait["duty"] < 1
    manifest = json.loads((ROOT / "reference/robot_b91_manifest.json").read_text())
    cfg = json.loads((ROOT / "reference/b91_kinematics.json").read_text())
    assert (
        hashlib.sha256((ROOT / "cad/B9_1/Robot_B9_1.f3d").read_bytes()).hexdigest()
        == manifest["source_sha256"]
    )
    r = Robot()
    assert r.m.nu == 8 and r.m.nq == 19
    slope_gait, _ = load_gait_parameters(gait_path, "slope")
    r.set_gait_parameters(slope_gait)
    assert r.p["pattern"] == "crawl" and r.p["period"] == slope_gait["period"]
    r.set_gait_parameters(default_gait, transition_seconds=0)
    r.reset()
    # Output limits include yaw gearing; pitch is direct drive at 5 V.
    np.testing.assert_allclose(
        r.limit,
        [0.2547004930555556, 0.22361580805545, 0.4862463958333334, 0.22361580805545]
        * 2,
    )
    np.testing.assert_allclose(
        r.m.actuator_ctrlrange[r.ai], np.column_stack((-r.limit, r.limit)), rtol=1e-8
    )
    assert r.d.sensor("imu_angular_velocity").data.shape == (3,)
    # Commands are held until the next 50 Hz update.
    r.step_targets(np.full(8, 0.1))
    for _ in range(9):
        r.step_targets(np.full(8, -0.1))
        np.testing.assert_allclose(r.command_target, 0.1)
    r.step_targets(np.full(8, -0.1))
    np.testing.assert_allclose(r.command_target, -0.1)
    # IR validity and sample holding, independent of the scene geometry.
    for name, value in zip(["ir_left", "ir_right", "ir_ground"], [0.04, 0.3, -1]):
        r.d.sensor(name).data[0] = value
    r.next_ir_time = r.d.time
    r._sample_ir()
    np.testing.assert_allclose(r.ir_distances_m, [0.04, 0.3, np.nan], equal_nan=True)
    r.d.sensor("ir_left").data[0] = 0.039
    r.d.sensor("ir_right").data[0] = 0.301
    r._sample_ir()
    np.testing.assert_allclose(r.ir_distances_m, [0.04, 0.3, np.nan], equal_nan=True)
    r.d.time += 0.018
    r._sample_ir()
    assert np.isnan(r.ir_distances_m).all()
    r.reset()
    assert abs(r.m.body_mass.sum() - manifest["total_mass_kg"]) < 1e-8
    assert len(manifest["parts"]) == 97
    for part in manifest["parts"]:
        if part["kind"] == "servo":
            joint = part["component"][:2] + (
                "_pitch" if "pitch servo" in part["component"] else "_yaw"
            )
            assert (
                part["mass_kg"] == cfg["servos"][cfg["joint_servos"][joint]]["mass_kg"]
            )
    for leg, k in cfg["legs"].items():
        expected = np.array(k["yaw_center_mm"]) / 1000 + rz(
            k["neutral_yaw_deg"]
        ) @ np.array([0.055, 0, 0.035])
        np.testing.assert_allclose(
            r.d.xanchor[r.m.joint(leg + "_pitch").id] - r.d.qpos[:3],
            expected,
            atol=1e-8,
        )
        assert (
            next(
                x
                for x in manifest["parts"]
                if x["component"].startswith(leg + " ")
                and "pitch servo" in x["component"]
            )["link"]
            == leg + "_pitch"
        )
        assert (
            next(
                x
                for x in manifest["parts"]
                if x["component"].startswith(leg + " ")
                and "pitch horn" in x["component"]
            )["link"]
            == leg + "_yaw"
        )
    for _ in range(1500):
        r.step(False)
    assert r.tilt() < 1 and np.isfinite(r.d.qpos).all()
    assert not sum(w.number for w in r.d.warning)
    for c in r.d.contact:
        names = [r.m.geom(g).name for g in (c.geom1, c.geom2)]
        if "floor" in names:
            assert any("_tread_" in n for n in names), names
    for _ in range(500):
        r.step_targets(np.tile([0.15, -0.2], 4))
        assert np.all(np.abs(r.torque) <= r.limit + 1e-12)
    assert np.max(np.abs(r.d.qpos[r.qa])) > 0.05
    assert np.isfinite(r.d.qpos).all() and not sum(w.number for w in r.d.warning)
    course = Robot(ramp=True)
    assert course.m.ngeom > r.m.ngeom
    print(
        "PASS: B9-1 source hash, 97 bodies, axes, mass, standing contacts, actuation, torque limits and course load."
    )


if __name__ == "__main__":
    main()
