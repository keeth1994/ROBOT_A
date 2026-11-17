"""Build B10 visuals, CAD-derived segmented collision hulls and explicit mass estimates."""

from pathlib import Path
from collections import defaultdict
import gzip, json, hashlib, math
import numpy as np
from scipy.spatial import ConvexHull, QhullError
import xml.etree.ElementTree as E
from robot_b91.paths import ROOT
from robot_b91.mesh_utils import fmt, visual_mesh
from robot_b91.geometry import rz
from robot_b91.course import build_course
from robot_b91 import servo_properties

LEGS = ["FL", "RL", "FR", "RR"]


def classify(n):
    # Match equipment envelopes specifically: their printed mounts are PLA.
    if "yaw servo fixed" in n or "pitch servo" in n or n == "Camera Parallax 900-00005":
        return "servo"
    for key, kind in [
        ("battery envelope", "battery"), ("sensor ir_", "ir"),
        ("TimerCamera-X housing", "camera"),
        ("IMU chip", "imu"), ("matrix board 76", "matrix_board"),
        ("DFR0753 carrier wiring reservation", "regulator"),
        ("rubber tread", "rubber"),
        ("bearing 4x", "bearing4"), ("circular horn", "horn"),
        ("pitch horn", "horn"), ("metal servo horn", "steel"),
        ("metal horn", "steel"), ("pivot bearing", "bearing4"),
    ]:
        if key in n:
            return kind
    # An M3/M4 mention in a printed part's name describes its interface.
    if any(k in n for k in [" axle", "washer", " bolt", " screw", " nut", "pitch M4"]):
        if not any(k in n for k in ["captive support nuts", "captive nut pocket"]):
            return "steel"
    return "PLA"


def camera_mount(record):
    if record["component"] in ("Lens", "TimerCamera-X housing"):
        return False
    return record["assembly"].startswith("Tilt ") or "camera" in record["component"].lower()


def group(record):
    # STEP assembly membership includes fasteners and parts with older names.
    assembly = record["assembly"]
    if assembly in [leg + " " + joint for leg in LEGS for joint in ("yaw", "pitch")]:
        return assembly.replace(" ", "_")
    if assembly in ("Robot frame", "Tilt input", "Tilt cradle", "Tilt link"):
        return "platform"
    raise ValueError(f"Unknown STEP assembly: {assembly}")


def neutral_records(records, cfg):
    """Rotate moving leg parts from the saved CAD pose to diagonal yaw zero."""
    result = []
    for record in records:
        link = group(record)
        if link == "platform":
            result.append(record)
            continue
        leg = cfg["legs"][link[:2]]
        rotation = rz(leg["neutral_yaw_deg"] - leg["cad_yaw_deg"])
        origin = np.array(leg["yaw_center_mm"]) / 1000
        record = record.copy()
        record["vertices"] = (np.array(record["vertices"]) - origin) @ rotation.T + origin
        record["com_m"] = rotation @ (np.array(record["com_m"]) - origin) + origin
        record["unit_inertia_m2"] = rotation @ record["unit_inertia_m2"] @ rotation.T
        result.append(record)
    return result


def split_polygons(polys, axis, cut):
    sides = [[], []]
    for poly in polys:
        for side in range(2):
            out = []
            for a, b in zip(poly, np.roll(poly, -1, axis=0)):
                ia = (a[axis] <= cut) if side == 0 else (a[axis] >= cut)
                ib = (b[axis] <= cut) if side == 0 else (b[axis] >= cut)
                if ia:
                    out.append(a)
                if ia != ib:
                    out.append(a + (b - a) * ((cut - a[axis]) / (b[axis] - a[axis])))
            if len(out) >= 3:
                sides[side].append(np.array(out))
    return sides


def hulls(v, f, steps):
    # Split actual CAD surface triangles into cells before taking each convex hull.
    cells = [[v[t] for t in f]]
    for axis, step in enumerate(steps):
        if step is None:
            continue
        for cut in np.arange(v[:, axis].min() + step, v[:, axis].max() - 1e-7, step):
            new = []
            for polys in cells:
                points = np.concatenate(polys)
                if (
                    points[:, axis].min() < cut - 1e-9
                    and points[:, axis].max() > cut + 1e-9
                ):
                    new.extend(s for s in split_polygons(polys, axis, cut) if s)
                else:
                    new.append(polys)
            cells = new
    for polys in cells:
        points = np.unique(np.round(np.concatenate(polys), 9), axis=0)
        if len(points) < 4:
            continue
        try:
            h = ConvexHull(points)
        except QhullError:
            continue
        if h.volume > 1e-13:
            yield points[h.vertices]


def main():
    ref = ROOT / "reference"
    source = ref / "b10_cad_meshes.json.gz"
    with gzip.open(source, "rt") as f:
        export = json.load(f)
    assert (
        hashlib.sha256((ROOT / export["source"].replace('\\', '/')).read_bytes()).hexdigest()
        == export["source_sha256"]
    ), "CAD STEP changed; run python cad/export_b10_sim.py again"
    cfg = json.loads((ref / "b10_kinematics.json").read_text())
    records = neutral_records(export["bodies"], cfg)
    masscfg = json.loads((ref / "b10_mass.json").read_text())
    xml = E.Element("mujoco", model="B10 CAD transfer")
    E.SubElement(
        xml,
        "compiler",
        angle="radian",
        meshdir="robot_meshes_b10",
        inertiafromgeom="false",
        autolimits="true",
    )
    E.SubElement(
        xml,
        "option",
        timestep=".002",
        integrator="implicitfast",
        cone="elliptic",
        iterations="60",
    )
    vis = E.SubElement(xml, "visual")
    E.SubElement(vis, "global", offwidth="1280", offheight="900")
    E.SubElement(vis, "headlight", ambient=".45 .45 .45")
    asset = E.SubElement(xml, "asset")
    wb = E.SubElement(xml, "worldbody")
    act = E.SubElement(xml, "actuator")
    E.SubElement(
        asset,
        "texture",
        name="checker",
        type="2d",
        builtin="checker",
        rgb1=".85 .85 .85",
        rgb2=".12 .14 .17",
        width="512",
        height="512",
    )
    E.SubElement(
        asset,
        "material",
        name="checker_floor",
        texture="checker",
        texrepeat="5 5",
        texuniform="true",
    )
    E.SubElement(wb, "light", pos="0 -1 2", dir="0 0 -1")
    E.SubElement(
        wb,
        "geom",
        name="floor",
        type="plane",
        size="10 10 .1",
        material="checker_floor",
        contype="1",
        conaffinity="2",
        friction=".8 .002 .0001",
    )
    allv = np.concatenate([r["vertices"] for r in records if not camera_mount(r)])
    height = -allv[:, 2].min() + 0.002
    root = E.SubElement(wb, "body", name="platform", pos=fmt([0, 0, height]))
    E.SubElement(root, "freejoint")
    bodies = {"platform": root}
    origins = {"platform": np.zeros(3)}
    rots = {}
    for l, k in cfg["legs"].items():
        R = rz(k["neutral_yaw_deg"])
        rots[l] = R
        y = np.array(k["yaw_center_mm"]) / 1000
        p = y + R @ np.array(k["pitch_local_mm"]) / 1000
        for suffix, origin, parent, axis, limits in [
            ("yaw", y, "platform", [0, 0, 1], cfg["yaw_requested_range_deg"]),
            ("pitch", p, l + "_yaw", R @ np.array([0, 1, 0]), cfg["pitch_requested_range_deg"]),
        ]:
            n = l + "_" + suffix
            origins[n] = origin
            bodies[n] = E.SubElement(
                bodies[parent], "body", name=n, pos=fmt(origin - origins[parent])
            )
            E.SubElement(
                bodies[n],
                "joint",
                name=n,
                axis=fmt(axis),
                range=fmt(np.radians(limits)),
                damping=".001",
                armature=".000001",
            )
            limit = servo_properties(cfg, n)[0]
            E.SubElement(
                act, "motor", name=n, joint=n, ctrlrange=fmt([-limit, limit])
            )
    folder = ROOT / "models/robot_meshes_b10"
    folder.mkdir(parents=True, exist_ok=True)
    groups = defaultdict(list)
    audit = []
    collision_count = 0
    hull_cache = {}
    visual_files = set()
    colors = {
        "PLA": ".75 .8 .87 1",
        "servo": ".08 .09 .11 1",
        "battery": ".9 .65 .1 1",
        "rubber": ".06 .065 .07 1",
        "horn": ".9 .65 .1 1",
    }
    for i, r in enumerate(records):
        if camera_mount(r):
            continue
        n = r["component"]
        g = group(r)
        kind = classify(n)
        v = np.array(r["vertices"])
        f = np.array(r["indices"]).reshape(-1, 3)
        mass = masscfg["fixed_mass_kg"].get(
            kind, r["volume_m3"] * masscfg["density_kg_m3"].get(kind, 1240)
        )
        if kind == "servo" and n.startswith("Camera "):
            mass = cfg["servos"]["Parallax"]["mass_kg"]
        elif kind == "servo":
            mass = servo_properties(
                cfg, n[:2] + ("_pitch" if "pitch servo" in n else "_yaw")
            )[2]
        groups[g].append((mass, np.array(r["com_m"]), np.array(r["unit_inertia_m2"])))
        audit.append(
            dict(component=n, link=g, kind=kind, mass_kg=mass, volume_m3=r["volume_m3"])
        )
        if kind == "servo":
            joint = n[:2] + ("_pitch" if "pitch servo" in n else "_yaw")
            audit[-1]["servo_model"] = cfg["joint_servos"][joint]
        vv, ff = visual_mesh(r, origins[g])
        name = "cad_" + str(i)
        path = folder / (name + ".obj")
        visual_files.add(path.name)
        with path.open("w") as out:
            for a in vv:
                out.write("v " + fmt(a) + "\n")
            for a in ff:
                out.write("f " + " ".join(str(int(x) + 1) for x in a) + "\n")
        E.SubElement(
            asset, "mesh", name=name, file=path.name, inertia="shell", maxhullvert="64"
        )
        E.SubElement(
            bodies[g],
            "geom",
            type="mesh",
            mesh=name,
            rgba=colors.get(kind, ".2 .5 .3 1"),
            contype="0",
            conaffinity="0",
            group="1",
            mass="0",
        )
        # Small fasteners/bearing interiors omitted from collision, retained in mass and visuals.
        if kind in ["steel", "horn", "bearing4", "bearing6"] or any(
            k in n for k in ["spacer", "spacing tube", "yaw driver"]
        ):
            continue
        R = rots.get(n[:2], np.eye(3))
        offset = origins.get(n[:2] + "_yaw", np.zeros(3))
        # 0.5 mm vertex clustering bounds the collision surface simplification.
        cells = np.floor(v / 0.0005).astype(np.int64)
        _, first, inverse = np.unique(
            cells, axis=0, return_index=True, return_inverse=True
        )
        cv = v[first]
        cf = inverse[f]
        cf = cf[
            (cf[:, 0] != cf[:, 1]) & (cf[:, 1] != cf[:, 2]) & (cf[:, 0] != cf[:, 2])
        ]
        _, keep = np.unique(np.sort(cf, axis=1), axis=0, return_index=True)
        cf = cf[keep]
        local = (cv - offset) @ R
        steps = (
            (0.004, None, None)
            if kind == "rubber"
            else ((0.016, 0.016, 0.020) if kind == "PLA" else (None, None, None))
        )
        key = hashlib.sha256(local.tobytes() + cf.tobytes()).hexdigest()
        if key not in hull_cache:
            hull_cache[key] = list(hulls(local, cf, steps))
        for j, h in enumerate(hull_cache[key]):
            name = (
                (n[:2] + "_tread_" if kind == "rubber" else "collision_")
                + str(i)
                + "_"
                + str(j)
            )
            E.SubElement(
                asset,
                "mesh",
                name=name,
                vertex=fmt(((h @ R.T) + offset - origins[g]).ravel()),
                inertia="shell",
            )
            E.SubElement(
                bodies[g],
                "geom",
                name=name,
                type="mesh",
                mesh=name,
                mass="0",
                group="3",
                rgba=".1 .8 .6 .3",
                contype="2",
                conaffinity="3",
                friction=".8 .002 .0001",
                solref=".006 1",
            )
            collision_count += 1
    for path in folder.glob("cad_*.obj"):
        if path.name not in visual_files:
            path.unlink()
    groups["platform"].append(
        (
            masscfg["extra_wiring_fasteners_kg"],
            np.array([0, 0, 0.025]),
            np.diag([0.14**2 + 0.05**2, 0.14**2 + 0.05**2, 2 * 0.14**2]) / 12,
        )
    )
    estimated_mass = sum(m for entries in groups.values() for m, _, _ in entries)
    measured_mass = masscfg["measured_total_mass_kg"]
    if not math.isfinite(measured_mass) or measured_mass <= 0:
        raise ValueError("Measured robot mass must be a positive number in kilograms.")
    mass_scale = measured_mass / estimated_mass
    for part in audit:
        part["mass_kg"] *= mass_scale
    for g, entries in groups.items():
        entries = [(m * mass_scale, c, s) for m, c, s in entries]
        total = sum(m for m, c, s in entries)
        com = sum(m * c for m, c, s in entries) / total
        I = np.zeros((3, 3))
        for m, c, unit_inertia in entries:
            delta = c - com
            I += m * unit_inertia + m * (
                np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta)
            )
        E.SubElement(
            bodies[g],
            "inertial",
            mass=fmt(total),
            pos=fmt(com - origins[g]),
            fullinertia=fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]),
        )
    # B10 has two side IR sensors; there is no ground IR sensor.
    sensors = E.SubElement(xml, "sensor")
    sc = json.loads((ref / "b10_sensor_axes.json").read_text())

    def center(key):
        rr = next(r for r in records if key in r["component"])
        v = np.array(rr["vertices"])
        return (v.min(0) + v.max(0)) / 2

    E.SubElement(
        root, "site", name="imu_site", pos=fmt(center("IMU chip")), size=".001"
    )
    E.SubElement(sensors, "accelerometer", name="imu_acceleration", site="imu_site")
    E.SubElement(sensors, "gyro", name="imu_angular_velocity", site="imu_site")
    for s in sc["ir"]:
        ax, ay, az = np.array(s["axes"])
        E.SubElement(
            root,
            "site",
            name=s["name"] + "_site",
            pos=fmt(center("sensor " + s["name"]) + ay * 0.0066),
            xyaxes=fmt(np.r_[ax, -az]),
            size=".001",
        )
        E.SubElement(sensors, "rangefinder", name=s["name"], site=s["name"] + "_site")
    # TimerCamera-X: 66.5 degree diagonal FOV converted to vertical for 4:3.
    navigation = json.loads((ref / "b91_navigation.json").read_text())
    camera_pitch = np.radians(cfg["camera"]["pitch_deg"])
    E.SubElement(
        root,
        "camera",
        name="project_camera",
        pos=fmt(cfg["camera"]["lens_xyz_m"]),
        xyaxes=fmt([0, -1, 0, np.sin(camera_pitch), 0, np.cos(camera_pitch)]),
        fovy=str(navigation["fovy_deg"]),
    )
    E.indent(xml)
    E.ElementTree(xml).write(
        ROOT / "models/robot_b10.xml", encoding="utf-8", xml_declaration=True
    )
    terrain = build_course(
        asset, wb, json.loads((ref / "b91_course_settings.json").read_text())
    )
    (ref / "b91_course.json").write_text(json.dumps(terrain, indent=2))
    E.indent(xml)
    E.ElementTree(xml).write(
        ROOT / "models/robot_b10_course.xml", encoding="utf-8", xml_declaration=True
    )
    manifest = dict(
        revision="B10",
        source=export["source"],
        source_sha256=export["source_sha256"],
        cad_bodies=len(records),
        simulated_bodies=len(audit),
        excluded_components=[r["component"] for r in records if camera_mount(r)],
        collision_hulls=collision_count,
        total_mass_kg=measured_mass,
        estimated_total_mass_kg=estimated_mass,
        mass_scale=mass_scale,
        neutral_bounds_m=np.ptp(allv, axis=0).tolist(),
        parts=audit,
        limitations=[
            "Total mass matched to the measured robot; part masses scaled proportionally from CAD estimates",
            "CAD solid inertia scaled uniformly to provisional component masses; equipment internals approximated",
            "Segmented convex CAD hulls after 0.5 mm vertex clustering; small holes/fasteners omitted",
            "Estimated 5 V direct-drive servo ratings; no backlash, thermal or supply-current model",
            "Yaw conservatively limited to +/-45 degrees; larger range not cleared",
            "Rigid terrain and assumed 0.8 friction",
            "Ideal IMU and IR rays; no calibrated sensor noise",
            "Camera mount, motor and linkage omitted; camera retained at the exported 40-degree pose",
            "The previous robot gait and course results do not validate B10",
        ],
    )
    (ref / "robot_b10_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({k: v for k, v in manifest.items() if k not in ("parts", "excluded_components")}, indent=2))


if __name__ == "__main__":
    main()
