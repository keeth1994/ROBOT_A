"""Configurable rigid plank ramp and open zigzag corridor inspired by the course photo."""

import math
import numpy as np
import xml.etree.ElementTree as E
from robot_b91.mesh_utils import fmt


def transform(points, origin, yaw):
    p = np.asarray(points, dtype=float).copy()
    a = math.radians(yaw)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    p[..., :2] = p[..., :2] @ R.T + origin
    return p


def local_points(points, course):
    p = np.asarray(points, dtype=float).copy()
    a = math.radians(course["yaw_deg"])
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    p[..., :2] = (p[..., :2] - course["origin_xy_m"]) @ R
    return p


def rounded_panels(profile, names, distance):
    starts, ends = profile[:-1].copy(), profile[1:].copy()
    curves = {}
    for i in range(1, len(profile) - 1):
        if distance == 0:
            break
        incoming, outgoing = profile[i] - profile[i - 1], profile[i + 1] - profile[i]
        before, after = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        cut = min(distance, before / 4, after / 4)
        a = profile[i] - cut * incoming / before
        b = profile[i] + cut * outgoing / after
        ends[i - 1], starts[i] = a, b
        t = np.linspace(0, 1, 17)[:, None]
        curves[i] = (1 - t) ** 2 * a + 2 * (1 - t) * t * profile[i] + t ** 2 * b
    panels = []
    for i, name in enumerate(names):
        panels.append((name, name, starts[i], ends[i]))
        curve = curves.get(i + 1)
        if curve is not None:
            for j, (a, b) in enumerate(zip(curve[:-1], curve[1:])):
                owner = name if j < (len(curve) - 1) // 2 else names[i + 1]
                panels.append((f"corner_{i + 1}_{j}", owner, a, b))
    return panels


def build_course(asset, world, cfg):
    width = cfg["width_m"]
    thick = cfg["panel_thickness_m"]
    profile = np.asarray(cfg["profile_xz_m"], float)
    names = cfg["segment_names"]
    rounding = cfg.get("corner_rounding_m", 0)
    obstacles = []
    segments = []
    bounds = []
    if (
        width <= 0
        or thick <= 0
        or len(names) != len(profile) - 1
        or np.any(np.diff(profile[:, 0]) <= 0)
        or np.any(profile[:, 1] < 0)
        or not math.isfinite(rounding)
        or rounding < 0
    ):
        raise ValueError("Invalid ramp profile, names, width or thickness")
    if len(set(names)) != len(names):
        raise ValueError("Ramp segment names must be unique")

    def prism(name, verts, color, contact=True):
        E.SubElement(
            asset, "mesh", name=name + "_mesh", vertex=fmt(np.asarray(verts).ravel())
        )
        E.SubElement(
            world,
            "geom",
            name=name,
            type="mesh",
            mesh=name + "_mesh",
            rgba=color,
            contype="1" if contact else "0",
            conaffinity="2" if contact else "0",
            friction=".8 .002 .0001",
        )
        if contact:
            obstacles.append(name)
        bounds.extend(np.asarray(verts)[:, :2].tolist())

    def ramp_transform(v):
        return transform(v, cfg["origin_xy_m"], cfg["yaw_deg"])

    panels = rounded_panels(profile, names, rounding)
    surface_geoms = {}
    bottom_edges = []
    for index, (name, owner, (x0, z0), (x1, z1)) in enumerate(panels):
        # Separate thin planks preserve the valley; no convex hull bridges its empty space.
        # 20 micrometre overlap prevents ray/contact cracks from float mesh rounding.
        slope = (z1 - z0) / (x1 - x0)
        # Measure board thickness perpendicular to the surface, with square-cut ends.
        normal_x, normal_z = -slope / math.hypot(1, slope), 1 / math.hypot(1, slope)
        left = x0 - (0.00002 if index else 0)
        right = x1 + (0.00002 if index < len(panels) - 1 else 0)
        ends = [(left, z0 + slope * (left - x0)), (right, z1 + slope * (right - x1))]
        verts = [
            [x + normal_x * depth, y, z + normal_z * depth]
            for x, z in ends
            for y in [-width / 2, width / 2]
            for depth in [-thick, 0]
        ]
        prism(name, ramp_transform(verts), ".065 .07 .075 1")
        surface_geoms[name] = owner
        bottom_edges.append(np.array([[x0, z0], [x1, z1]]) - thick * np.array([normal_x, normal_z]))
        # Small exposed plywood edge, visual only, on each side of the black surface.
        for side in [-1, 1]:
            edge = [
                [x + normal_x * depth, y, z + normal_z * depth]
                for x, z in [(x0, z0), (x1, z1)]
                for y in [side * width / 2, side * (width / 2 + 0.001)]
                for depth in [-thick, -0.001]
            ]
            prism(
                name + f"_plywood_{side}", ramp_transform(edge), ".66 .49 .28 1", False
            )
    for name, (x0, z0), (x1, z1) in zip(names, profile[:-1], profile[1:]):
        segments.append(
            dict(
                name=name,
                start_x_m=float(x0),
                end_x_m=float(x1),
                start_height_m=float(z0),
                end_height_m=float(z1),
                angle_deg=math.degrees(math.atan2(z1 - z0, x1 - x0)),
            )
        )
    for i, (x, z) in enumerate(profile[1:-1]):
        if z <= thick:
            continue
        heights = []
        for edge in bottom_edges:
            left, right = max(x - 0.015, edge[0, 0]), min(x + 0.015, edge[1, 0])
            if left <= right:
                heights.extend(np.interp([left, right], edge[:, 0], edge[:, 1]))
        support_top = min(heights)
        for side in [-1, 1]:
            v = [
                [xx, yy, zz]
                for xx in [x - 0.015, x + 0.015]
                for yy in [side * 0.10 - 0.015, side * 0.10 + 0.015]
                for zz in [0, support_top]
            ]
            prism(f"ramp_support_{i}_{side}", ramp_transform(v), ".32 .34 .35 1")
    corridor = cfg["corridor"]
    if corridor["enabled"]:
        pts = np.asarray(corridor["centerline_xy_m"], float)
        w = corridor["clear_width_m"]
        t = corridor["wall_thickness_m"]
        h = corridor["wall_height_m"]
        ds = np.diff(pts, axis=0)
        lengths = np.linalg.norm(ds, axis=1)
        if len(pts) < 2 or min(w, t, h) <= 0 or np.any(lengths < 1e-6):
            raise ValueError("Invalid corridor dimensions")
        tangents = ds / lengths[:, None]
        normals = np.c_[-tangents[:, 1], tangents[:, 0]]
        miters = [normals[0]]
        for j in range(1, len(pts) - 1):
            n = normals[j - 1] + normals[j]
            n /= np.linalg.norm(n)
            den = np.dot(n, normals[j])
            if den < 0.3:
                raise ValueError("Corridor bend too sharp for mitered walls")
            miters.append(n / den)
        miters = np.array(miters + [normals[-1]])
        for side in [-1, 1]:
            inner = pts + side * w / 2 * miters
            outer = pts + side * (w / 2 + t) * miters
            for j in range(len(pts) - 1):
                polygon = [inner[j], inner[j + 1], outer[j + 1], outer[j]]
                verts = [[p[0], p[1], z] for p in polygon for z in [0, h]]
                prism(
                    f"corridor_wall_{side}_{j}",
                    transform(verts, corridor["origin_xy_m"], corridor["yaw_deg"]),
                    ".68 .49 .29 1",
                )
    b = np.asarray(bounds + [[0, 0]])
    return dict(
        description=cfg["description"],
        width_m=width,
        origin_xy_m=cfg["origin_xy_m"],
        yaw_deg=cfg["yaw_deg"],
        approach_end_x=float(profile[0, 0]),
        downhill_end_x=float(profile[-1, 0]),
        success_base_x=float(profile[-1, 0] + 0.15),
        required_surfaces=names,
        surface_geoms=surface_geoms,
        corner_rounding_m=rounding,
        surface_profile_xz_m=[panels[0][2].tolist()] + [p[3].tolist() for p in panels],
        obstacle_geoms=obstacles,
        segments=segments,
        corridor=corridor,
        camera_lookat=[*list((b.min(0) + b.max(0)) / 2), 0.08],
        camera_distance=float(max(2.5, np.ptp(b, axis=0).max() * 1.25)),
        assumptions=[
            "Photo-inspired dimensions, not surveyed",
            "Rigid planks; no panel bending",
            "Open corridor, no roof as pictured",
            "Ramp completion is separate from corridor navigation",
        ],
    )
