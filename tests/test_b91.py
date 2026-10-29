"""B9-8 transfer checks: provenance, links, axes, mass, contacts and motor limits."""

import json, hashlib, gzip
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
    manifest = json.loads((ROOT / "reference/robot_b98_manifest.json").read_text())
    cfg = json.loads((ROOT / "reference/b98_kinematics.json").read_text())
    assert (
        hashlib.sha256((ROOT / "cad/B9_8/Robot_B9_8.step").read_bytes()).hexdigest()
        == manifest["source_sha256"]
    )
    with gzip.open(ROOT / "reference/b98_cad_meshes.json.gz", "rt") as stream:
        export = json.load(stream)
    assert manifest["revision"] == export["revision"] == cfg["cad_revision"] == "B9-8"
    assert manifest["source"] == export["source"] == "cad/B9_8/Robot_B9_8.step"
    assert export["source_sha256"] == manifest["source_sha256"]
    assert len(export["bodies"]) == manifest["cad_bodies"] == 225
    r = Robot()
    assert r.m.nu == 8 and r.m.nq == 19
    slope_gait, _ = load_gait_parameters(gait_path, "slope")
    r.set_gait_parameters(slope_gait)
    assert r.p["pattern"] == "crawl" and r.p["period"] == slope_gait["period"]
    r.set_gait_parameters(default_gait, transition_seconds=0)
    r.reset()
    np.testing.assert_allclose(r.m.site("imu_site").pos, [0, 0, 0.0841], atol=1e-9)
    np.testing.assert_allclose(r.m.camera("project_camera").pos, [0.092, 0, 0.09215], atol=1e-9)
    for leg, k in cfg["legs"].items():
        np.testing.assert_allclose(
            r.d.xanchor[r.m.joint(leg + "_driver").id] - r.d.qpos[:3],
            np.array(k["driver_center_mm"]) / 1000, atol=1e-8,
        )
        # Independent CAD anchor: bearing lies on the pitch axis. Its component
        # COM may be displaced along that axis, but never perpendicular to it.
        bearing = next(p for p in export["bodies"]
                       if p["component"].startswith(leg + " ") and "pitch bearing" in p["component"])
        joint = r.m.joint(leg + "_pitch").id
        offset = np.array(bearing["com_m"]) + r.d.qpos[:3] - r.d.xanchor[joint]
        assert np.linalg.norm(np.cross(offset, r.d.xaxis[joint])) < 1e-8
    for part in manifest["parts"]:
        n = part["component"]
        if any(key in n for key in ["cradle", "camera upright", "ear bracket", "bridge captive"]):
            assert part["kind"] == "PLA", n
        if "pitch M4" in n or "pitch brace M3" in n:
            assert part["kind"] == "steel" and part["link"] == n[:2] + "_pitch", n
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
    r.next_command_time = r.d.time
    r.step_targets(np.full(8, -2.0))
    np.testing.assert_allclose(r.command_target, r.joint_ranges[:, 0])
    assert not np.any(r.d.xfrc_applied) and not np.any(r.d.qfrc_applied)
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
    assert len(manifest["parts"]) == 225
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
        ) @ np.array(k["pitch_local_mm"]) / 1000
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
    assert course.navigation.estimate.camera_x == course.m.camera("project_camera").pos[0]
    course.close()
    r.close()
    print(
        "PASS: B9-8 source hash, 225 CAD solids, axes, mass, standing contacts, actuation, torque limits and course load."
    )


if __name__ == "__main__":
    main()
