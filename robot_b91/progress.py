"""Course scoring, independent of the gait. Never applies forces to the robot."""

import numpy as np
from robot_b91.course import local_points


class CourseProgress:
    """Require every foot on every panel, an on-course route, and a clear exit."""

    def __init__(self, model, course):
        self.course = course
        self.names = course["required_surfaces"]
        self.surface_ids = {model.geom(name).id: name for name in self.names}
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
                # Floor contact outside the sides is a bypass, including at the valley.
                point = local_points(contact.pos, c)
                if start < point[0] < end and abs(point[1]) > c["width_m"] / 2 + 0.001:
                    self.floor_bypass = True
        all_panels = all(i == len(self.names) for i in self.next_panel.values())
        # Bounding spheres are conservative: the complete rubber feet must leave the ramp.
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
