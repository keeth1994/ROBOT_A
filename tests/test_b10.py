"""Check that the current CAD assembly is fully included in simulation."""

import gzip
import hashlib
import json

import mujoco
import numpy as np

from robot_b91.paths import ROOT


def main():
    design = json.loads((ROOT / "cad/current_design.json").read_text())
    manifest = json.loads((ROOT / design["simulation_manifest"]).read_text())
    config = json.loads((ROOT / "reference/b10_kinematics.json").read_text())
    with gzip.open(ROOT / "reference/b10_cad_meshes.json.gz", "rt") as stream:
        export = json.load(stream)
    assert export["revision"] == manifest["revision"] == design["cad_revision"]
    assert config["cad_revision"] == design["simulation_revision"]
    assert export["source"] == design["step"]
    assert manifest["source_sha256"] == hashlib.sha256(
        (ROOT / design["step"]).read_bytes()
    ).hexdigest()
    assert manifest["excluded_components"] == []
    assert len(export["bodies"]) == manifest["cad_bodies"] == manifest["simulated_bodies"]
    parts = {part["component"]: part for part in manifest["parts"]}
    for name in ("Drive tower", "Idler tower", "Centred camera cradle",
                 "Camera 20T output gear", "Upper 40T gear and lever", "Camera pushrod"):
        assert parts[name]["link"] == "platform", name
    assert parts["Camera Parallax 900-00005"]["servo_model"] == "Parallax"

    for path in (design["simulation_model"], design["simulation_course"]):
        model = mujoco.MjModel.from_xml_path(str(ROOT / path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        assert model.nu == 8 and model.njnt == 9
        expected_mass = json.loads((ROOT / "reference/b10_mass.json").read_text())[
            "measured_total_mass_kg"
        ]
        np.testing.assert_allclose(model.body_mass.sum(), expected_mass, atol=1e-8)
        assert sum(model.geom_group == 1) == len(export["bodies"])
        camera = model.camera("project_camera")
        lens = next(r for r in export["bodies"] if r["component"] == "Lens")
        vertices = np.array(lens["vertices"])
        # A short cylindrical lens has its optical axis along its smallest variance.
        _, axes = np.linalg.eigh(np.cov(vertices.T))
        forward = -data.cam_xmat[camera.id].reshape(3, 3)[:, 2]
        assert abs(np.dot(forward, axes[:, 0])) > .99999
        assert np.linalg.norm(camera.pos - lens["com_m"]) < .002
        assert np.isfinite(data.qacc).all()
    print("PASS: current CAD, full camera mount, camera alignment and measured mass.")


if __name__ == "__main__":
    main()
