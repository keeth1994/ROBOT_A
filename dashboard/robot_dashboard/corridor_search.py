"""Find a corridor with camera and side IR readings, then stop centered."""
import math

import numpy as np
from PIL import Image, ImageDraw

from .corridor import ir_readings
from .corridor_walls import IRWallNavigation
from .navigation import CourseNavigation


def wall_view(rgb):
    """Look for substantial tan/orange walls in the lower part of the image."""
    image = Image.fromarray(rgb).convert('RGB')
    image.thumbnail((320, 240))
    rgb = np.array(image)
    hsv = np.array(image.convert('HSV'))
    h, w = hsv.shape[:2]
    hue, saturation, value = hsv.transpose(2, 0, 1)
    mask = (hue >= 8) & (hue <= 36) & (saturation >= 45) & (value >= 65)
    # Ignore the room above the maze and short strips from distant back walls.
    columns = mask[int(.40 * h):int(.90 * h)].sum(axis=0) >= .07 * h
    for x in range(1, w - 1):
        if columns[x - 1] and columns[x + 1]:
            columns[x] = True
    changes = np.diff(np.r_[False, columns, False].astype(int))
    runs = [(int(a), int(b)) for a, b in zip(np.where(changes == 1)[0],
                                            np.where(changes == -1)[0]) if b - a >= .04 * w]
    target, kind, side = None, 'none', None
    gaps = [(a[1], b[0]) for a, b in zip(runs, runs[1:]) if b[0] - a[1] >= .18 * w]
    if gaps:
        left, right = max(gaps, key=lambda gap: gap[1] - gap[0])
        target, kind = (left + right) / (2 * w), 'opening'
    elif runs:
        left, right = runs[0][0], runs[-1][1]
        if max(left, w - right) >= .25 * w:
            side = 'left' if left > w - right else 'right'
            target = max(.15, left / w - .16) if side == 'left' else min(.85, right / w + .16)
            kind = 'edge'
        else:
            kind = 'wall'
    if target is not None:
        x = int(target * w)
        floor = hsv[int(.72 * h):int(.95 * h), max(0, x - int(.06*w)):min(w, x + int(.06*w))]
        if not np.mean((floor[:, :, 1] < 90) & (floor[:, :, 2] > 12)) >= .65:
            target, kind = None, 'wall'
    blocked = bool(mask[int(.78*h):int(.96*h), int(.38*w):int(.62*w)].mean() > .12)
    return dict(rgb=rgb, mask=mask, runs=runs, target=target, kind=kind,
                side=side, blocked=blocked)


def visible_wall_ends(view, camera, roll, pitch):
    """Project visible wall bases onto the floor using measured camera geometry."""
    mask = view['mask']
    h, w = mask.shape
    columns = np.flatnonzero(mask.any(axis=0))
    if len(columns) < 12:
        return []
    bottoms = h - 1 - mask[::-1, columns].argmax(axis=0)
    f = h / (2 * math.tan(math.radians(camera['fovy_deg']) / 2))
    rays = np.column_stack([np.ones(len(columns)), -(columns - (w-1)/2)/f,
                            -(bottoms - (h-1)/2)/f])
    angle = math.radians(camera['pitch_deg']) + pitch
    cp, sp, cr, sr = math.cos(angle), math.sin(angle), math.cos(roll), math.sin(roll)
    rotation = np.array([[cp, sp*sr, sp*cr], [0, cr, -sr], [-sp, cp*sr, cp*cr]])
    rays = rays @ rotation.T
    valid = (rays[:, 2] < -.08) & (bottoms < h - 2)
    points = -camera['height_m'] * rays[:, :2] / np.minimum(rays[:, 2:3], -.001)
    points[:, 0] += camera['forward_m']
    valid &= np.linalg.norm(points, axis=1) < 2.5
    splits = np.flatnonzero((np.diff(columns) > 1) |
                           (np.linalg.norm(np.diff(points, axis=0), axis=1) > .08) |
                           ~valid[:-1] | ~valid[1:]) + 1
    candidates = []
    for indices in np.split(np.arange(len(points)), splits):
        indices = indices[valid[indices]]
        if len(indices) < 12:
            continue
        group = points[indices]
        center = group.mean(0)
        _, _, axes = np.linalg.svd(group - center, full_matrices=False)
        tangent = axes[0]
        along = (group - center) @ tangent
        normal = np.array([-tangent[1], tangent[0]])
        if np.ptp(along) < .20 or np.median(np.abs((group - center) @ normal)) > .025:
            continue
        ends = [int(np.argmin(along)), int(np.argmax(along))]
        end_index = min(ends, key=lambda i: np.linalg.norm(group[i]))
        # A wall clipped by the image border does not reveal its actual end.
        if not 4 < columns[indices[end_index]] < w - 5:
            continue
        end = center + tangent * along[end_index]
        if np.dot(tangent, center - end) < 0:
            tangent = -tangent
        if np.dot(normal, -end) < 0:
            normal = -normal
        candidates.append((float(np.linalg.norm(end)), end, tangent, normal))
    return [item[1:] for item in sorted(candidates, key=lambda item: item[0])]


class CorridorSearch:
    use_camera = use_imu = True

    def __init__(self, started, settings, camera_geometry=None):
        self.started = started
        self.orientation = CourseNavigation(started, use_camera=False, use_imu=True)
        self.estimate = self.orientation.estimate
        self.imu_heading = 0.0
        self.settings = settings
        self.stage = 'scan'
        self.stage_started = started
        self.scan_heading = None
        self.turn_degrees = 0.0
        self.forward = 0.0
        self.speed = .5
        self.view = None
        self.sequence = -1
        self.matches = 0
        self.previous_kind = None
        self.edge_side = None
        self.camera_angle = None
        self.camera_changed = started
        self.centering = None
        self.centered_since = None
        self.finished = False
        self.succeeded = False
        self.reason = 'Waiting for a fresh camera frame and IMU before searching.'
        self.camera_geometry = camera_geometry
        self.position = np.zeros(2)
        self.corner_goal = None
        self.entry_heading = None
        self.entry_goal = None
        self.both_seen_since = None
        self.last_update = started
        self.moving = False

    def set_speed(self, speed):
        self.speed = min(speed, .5)

    def finish(self, reason):
        self.finished = True
        self.forward = 0.0
        self.estimate.heading = self.estimate.lateral = 0.0
        self.reason = reason
        return False

    def hold(self, reason, reset=False):
        self.forward = 0.0
        self.estimate.heading = self.estimate.lateral = 0.0
        if reset:
            self.matches = 0
            self.both_seen_since = None
        self.centered_since = None
        self.reason = reason
        return False

    def update(self, status, now, camera_frame=None):
        dt = max(0, min(.3, now - self.last_update))
        if self.moving:
            self.position += self.settings['walk_speed'] * self.speed * self.forward * dt * np.array(
                [math.cos(self.imu_heading), math.sin(self.imu_heading)])
        self.last_update = now
        self.moving = self._update(status, now, camera_frame)
        return self.moving

    def _update(self, status, now, camera_frame):
        if self.finished:
            return False
        self.forward = 0.0
        if now - self.started > 180:
            return self.finish('Corridor search timed out. Reposition the robot and try again.')
        if not self.orientation.update(status, now, None):
            return self.hold(self.orientation.reason, reset=True)
        self.imu_heading = self.orientation.imu_heading
        self.estimate.heading = self.estimate.lateral = 0.0
        camera = status.get('cameraPitch')
        if camera is not None:
            angle = camera.get('angle')
            if (isinstance(angle, bool) or not isinstance(angle, (int, float)) or
                    not math.isfinite(angle) or not -80 <= angle <= 45):
                return self.hold('Enable the camera pitch servo before searching.', reset=True)
            target = self.orientation.camera_calibration['corridor_servo_deg']
            if angle != target:
                return self.hold(f'Waiting for camera pitch {target} degrees before corridor search.', reset=True)
            if angle != self.camera_angle:
                self.camera_angle, self.camera_changed = angle, now
                self.matches = 0
                self.sequence = -1
            if camera.get('moving') or now - self.camera_changed < .5:
                return self.hold('Waiting for the camera pitch to settle.', reset=True)
        if (camera_frame is None or not self.started <= camera_frame[1] <= now or
                now - camera_frame[1] > .35 or
                (camera is not None and camera_frame[1] < self.camera_changed + .5)):
            return self.hold('Holding steps: waiting for a fresh camera frame.', reset=True)
        sequence, _, rgb = camera_frame
        if sequence != self.sequence:
            view = wall_view(rgb)
            previous_target = self.view['target'] if self.view else None
            stable_target = (view['target'] is None or previous_target is None or
                             abs(view['target'] - previous_target) < .10)
            self.view = view
            self.sequence = sequence
            key = (self.view['kind'], self.view['side'])
            self.matches = self.matches + 1 if key == self.previous_kind and stable_target else 1
            self.previous_kind = key
        if now - self.started < 2:
            return self.hold('Settling for 2 seconds; keep the robot still.')
        try:
            volts = ir_readings(status, allow_zero=True)
        except ValueError as error:
            return self.hold(str(error), reset=True)
        distances = np.asarray(self.settings['ir_scale_mv_m']) / np.maximum(
            np.asarray(volts) - self.settings['ir_offset_mv'], .001)
        if max(volts) >= 3200 or min(distances) < .04:
            return self.finish('A side wall is too close. Reposition the robot before searching again.')

        if self.stage == 'scan':
            if self.scan_heading is None:
                self.scan_heading = self.imu_heading
                self.stage_started = now
            self.turn_degrees = math.degrees(abs(self.imu_heading - self.scan_heading))
            if self.turn_degrees >= 360 or now - self.stage_started > 90:
                return self.finish('No corridor walls found within one turn. Check the camera view and lighting.')
            if self.view['kind'] != 'none' and self.matches >= 3:
                self.stage, self.stage_started = 'approach', now
                return self.hold('Wall color found. Checking the opening before approaching.')
            self.estimate.heading = .3
            self.reason = f'Searching for tan/orange walls: {self.turn_degrees:.0f} / 360 degrees.'
            return True

        if self.stage == 'center':
            if max(distances) > .30:
                return self.finish('Lost a side wall while centering. Reposition the robot and try again.')
            self.centering.set_speed(self.speed)
            if not self.centering.update(status, now, None):
                return self.hold(self.centering.reason)
            self.estimate.heading = self.centering.estimate.heading
            self.estimate.lateral = self.centering.estimate.lateral
            centered = (abs(self.estimate.lateral) < .02 and abs(self.estimate.heading) < .10 and
                        now - self.stage_started > 2)
            self.centered_since = (self.centered_since or now) if centered else None
            if self.centered_since is not None and now - self.centered_since >= 1:
                self.succeeded = True
                return self.finish('Corridor found and centered between the side walls. Stopped.')
            if now - self.stage_started > 20 or self.view['blocked']:
                return self.finish('Stopped before the wall ahead or the centering time limit.')
            self.forward = 1.0
            self.reason = 'Centering in the corridor using both side IR sensors and IMU.'
            return True

        if self.stage in ('round_corner', 'enter'):
            if now - self.stage_started > 120:
                return self.finish('Could not enter the corridor before the time limit.')
            self.both_seen_since = ((self.both_seen_since or now) if max(distances) <= .30 else None)
            # Wait through a complete slow gait cycle; a ray can still pass the entrance edge.
            if self.both_seen_since is not None and now - self.both_seen_since >= 2:
                self.begin_centering(now)
                return self.hold('Both side walls found. Checking corridor alignment.')
            if self.stage == 'round_corner':
                # Refresh the endpoint while it remains visible; odometry bridges the final short gap.
                self.update_corner_goal()
                delta = self.corner_goal - self.position
                if np.linalg.norm(delta) < .04:
                    self.stage = 'enter'
                    return self.hold('Past the wall end. Turning toward the corridor opening.')
                desired = math.atan2(delta[1], delta[0])
            else:
                self.update_entry_goal()
                delta = None if self.entry_goal is None else self.entry_goal - self.position
                if delta is not None and np.linalg.norm(delta) < .04:
                    return self.finish('Stopped at the entrance target without stable readings from both walls.')
                desired = self.entry_heading if delta is None else math.atan2(delta[1], delta[0])
            error = math.atan2(math.sin(self.imu_heading - desired), math.cos(self.imu_heading - desired))
            self.estimate.heading = max(-.3, min(.3, error))
            self.forward = 1.0 if abs(error) < .15 else 0.0
            if self.stage == 'enter' and self.entry_goal is None:
                self.forward = 0.0
            self.reason = ('Moving around the visible wall end.' if self.stage == 'round_corner'
                           else 'Aligning with the wall direction and entering the opening.')
            return True

        if now - self.stage_started > 120:
            return self.finish('Could not confirm the corridor entrance. Reposition the robot and try again.')
        if self.view['kind'] == 'none':
            return self.hold('Lost the wall color. Stopped; waiting for the walls to reappear.')
        if self.matches < 3:
            return self.hold('Checking the wall edges across fresh camera frames.')
        if self.view['kind'] == 'opening' and max(distances) <= .30:
            self.begin_centering(now)
            return self.hold('Both side walls found. Checking corridor alignment.')
        if self.view['kind'] == 'edge':
            if self.camera_geometry and self.update_corner_goal():
                self.stage, self.stage_started = 'round_corner', now
                return self.hold('Wall end located. Checking a route around its open side.')
            if self.edge_side is not None and self.edge_side != self.view['side']:
                return self.finish('The wall edge changed sides. Stop and check the entrance before retrying.')
            self.edge_side = self.view['side']
        if self.view['target'] is None or self.view['blocked']:
            self.estimate.heading = -.25 if self.edge_side == 'left' else .25
            self.reason = 'Turning in place to look around the wall end.'
            return True
        error = (self.view['target'] - .5) * 1.1
        self.estimate.heading = max(-.3, min(.3, error))
        # Align before moving forward; keep the corner at the edge of the view.
        self.forward = 0.0 if abs(error) > .22 else 1.0
        self.reason = ('Approaching the center of the visible opening.' if self.view['kind'] == 'opening'
                       else f'Following the visible wall end toward the open {self.edge_side} side; looking for an entrance.')
        return True

    def begin_centering(self, now):
        self.stage, self.stage_started = 'center', now
        # The firmware has already settled. Fit walls using the current IMU reference.
        self.centering = IRWallNavigation(now - 2, **self.settings)
        self.centering.imu_heading = self.imu_heading

    def update_corner_goal(self):
        ends = visible_wall_ends(self.view, self.camera_geometry,
                                 self.estimate.roll, self.estimate.pitch)
        if not ends:
            return False
        end, tangent, normal = ends[0]
        rotation = np.array([[math.cos(self.imu_heading), -math.sin(self.imu_heading)],
                             [math.sin(self.imu_heading), math.cos(self.imu_heading)]])
        goal = self.position + rotation @ (end - .24*tangent - .20*normal)
        direction = rotation @ tangent
        # Reject a different corner once the approach has started.
        if self.corner_goal is not None and np.linalg.norm(goal - self.corner_goal) > .15:
            return False
        self.corner_goal = goal if self.corner_goal is None else .8*self.corner_goal + .2*goal
        self.entry_heading = math.atan2(direction[1], direction[0])
        return True

    def update_entry_goal(self):
        ends = visible_wall_ends(self.view, self.camera_geometry,
                                 self.estimate.roll, self.estimate.pitch)
        for i, (left, tangent, _) in enumerate(ends):
            for right, other_tangent, _ in ends[i+1:]:
                gap = right - left
                if (not .25 < np.linalg.norm(gap) < .60 or
                        abs(gap @ tangent) > .12 or tangent @ other_tangent < .95):
                    continue
                target = (left + right) / 2 + .20*tangent
                rotation = np.array([[math.cos(self.imu_heading), -math.sin(self.imu_heading)],
                                     [math.sin(self.imu_heading), math.cos(self.imu_heading)]])
                goal = self.position + rotation @ target
                self.entry_goal = goal if self.entry_goal is None else .7*self.entry_goal + .3*goal
                return True
        return False

    def report(self):
        return dict(guidance='corridor_search', stage=self.stage, turn_degrees=self.turn_degrees,
                    target=self.view['target'] if self.view else None, finished=self.finished,
                    succeeded=self.succeeded,
                    detection_reason=self.reason)

    def debug_image(self):
        panel = Image.new('RGB', (640, 300), (15, 18, 23))
        draw = ImageDraw.Draw(panel)
        draw.text((6, 4), 'Corridor search: tan/orange walls and opening target', fill='white')
        if self.view:
            rgb = Image.fromarray(self.view['rgb']).resize((320, 240))
            mask = Image.fromarray((self.view['mask'] * 255).astype(np.uint8)).resize((320, 240)).convert('RGB')
            target = self.view['target']
            if target is not None:
                x = round(target * 320)
                ImageDraw.Draw(rgb).line((x, 96, x, 239), fill=(60, 255, 100), width=3)
            panel.paste(rgb, (0, 22))
            panel.paste(mask, (320, 22))
        draw.text((6, 266), f'{self.stage} | {self.turn_degrees:.0f} / 360 degrees', fill='white')
        draw.text((6, 282), self.reason, fill='white')
        return np.array(panel)
