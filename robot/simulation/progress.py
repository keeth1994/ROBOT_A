"""Check course progress."""

import math
import numpy as np
from robot.model.course import local_points


def corridor_entry_score(model, data, course):
    """Check corridor entry using simulation positions."""
    corridor = course['corridor']
    yaw = math.radians(corridor['yaw_deg'])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    points = np.asarray(corridor['centerline_xy_m']) @ rotation.T + corridor['origin_xy_m']
    direction = points[1] - points[0]
    length = float(np.linalg.norm(direction))
    direction /= length
    position = data.qpos[:2] - points[0]
    along = float(position @ direction)
    lateral = float(position @ np.array([-direction[1], direction[0]]))
    forward = data.xmat[model.body('platform').id].reshape(3, 3)[:2, 0]
    angle = math.atan2(forward[1], forward[0]) - math.atan2(direction[1], direction[0])
    angle = math.atan2(math.sin(angle), math.cos(angle))
    return dict(inside_and_centered=bool(.03 < along < length and abs(lateral) < .04 and abs(angle) < .20),
                entry_progress_m=along, lateral_error_m=lateral, heading_error_rad=angle)


class CourseProgress:
    """Check that all feet follow and clear the ramp."""

    def __init__(self, model, course):
        self.course = course
        self.names = course["required_surfaces"]
        surfaces = course.get("surface_geoms", {name: name for name in self.names})
        self.surface_ids = {model.geom(geom).id: name for geom, name in surfaces.items()}
        self.floor = model.geom("floor").id
        self.feet = {
            i: model.geom(i).name[:2]
            for i in range(model.ngeom)
            if "_tread_" in (model.geom(i).name or "")
        }
        self.tread_ids = np.array(list(self.feet))
        self.tread_radii = model.geom_rbound[self.tread_ids]
        self.visited = {name: set() for name in self.names}
        self.next_panel = {leg: 0 for leg in set(self.feet.values())}
        self.events = []
        self.off_course = False
        self.floor_bypass = False
        self.max_lateral = 0.0
        self.max_tilt = 0.0
        self.completed_at = None
        self.local_position = np.zeros(3)

    def update(self, data, tilt):
        c = self.course
        self.local_position = local_points(data.qpos[:3], c)
        x, y = self.local_position[:2]
        start, end = c["approach_end_x"], c["downhill_end_x"]
        if start <= x <= end:
            self.max_lateral = max(self.max_lateral, abs(float(y)))
            self.off_course |= bool(abs(y) > c["width_m"] / 2)
        self.max_tilt = max(self.max_tilt, tilt)
        for contact in data.contact:
            a, b = contact.geom1, contact.geom2
            for surface, foot in ((a, b), (b, a)):
                if surface in self.surface_ids and foot in self.feet:
                    name, leg = self.surface_ids[surface], self.feet[foot]
                    if leg not in self.visited[name]:
                        self.visited[name].add(leg)
                        self.events.append(dict(surface=name, leg=leg, time_s=float(data.time)))
                    index = self.next_panel[leg]
                    if index < len(self.names) and name == self.names[index]:
                        self.next_panel[leg] += 1
            if self.floor in (a, b):
                # Floor contact beside the ramp counts as leaving it.
                point = local_points(contact.pos, c)
                if start < point[0] < end and abs(point[1]) > c["width_m"] / 2 + 0.001:
                    self.floor_bypass = True
        all_panels = all(i == len(self.names) for i in self.next_panel.values())
        # Count the exit only after the whole foot clears the ramp.
        feet_x = local_points(data.geom_xpos[self.tread_ids], c)[:, 0]
        clear_exit = np.min(feet_x - self.tread_radii) > end
        if (self.completed_at is None and x >= c["success_base_x"] and clear_exit
                and all_panels and not self.off_course and not self.floor_bypass
                and self.max_tilt <= 45 and not sum(w.number for w in data.warning)):
            self.completed_at = float(data.time)

    def report(self):
        return dict(
            course_completed=self.completed_at is not None,
            completion_time_s=self.completed_at,
            final_course_position_m=self.local_position.tolist(),
            max_course_lateral_error_m=self.max_lateral,
            left_course=self.off_course,
            floor_bypass=self.floor_bypass,
            feet_per_surface={name: sorted(feet) for name, feet in self.visited.items()},
            contact_events=self.events,
        )
