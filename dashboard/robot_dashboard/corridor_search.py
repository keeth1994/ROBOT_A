"""Find a corridor with camera and side IR readings, then stop centered."""
import math
from collections import deque

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_opening, find_objects, grey_closing, label

from .corridor import ir_readings
from .navigation import CourseNavigation


def wall_view(rgb, preferred_side=None, camera_pitch=None, fovy_deg=None):
    """Look for substantial tan/orange walls and open floor beside them."""
    image = Image.fromarray(rgb).convert('RGB')
    image.thumbnail((320, 240))
    rgb = np.array(image)
    hsv = np.array(image.convert('HSV'))
    h, w = hsv.shape[:2]
    hue, saturation, value = hsv.transpose(2, 0, 1)
    mask = (hue >= 8) & (hue <= 36) & (saturation >= 45) & (value >= 65)
    mask = binary_opening(mask, structure=np.ones((3, 3)))
    regions, _ = label(mask)
    areas = np.bincount(regions.ravel())
    areas[0] = 0
    mask &= areas[regions] >= .01 * h * w
    wall_columns = mask.sum(axis=0) >= .07 * h
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
        if preferred_side is not None:
            left, right = (gaps[-1] if preferred_side == 'left' else gaps[0])
            target = right / w - .05 if preferred_side == 'left' else left / w + .05
            side = preferred_side
    elif runs:
        left, right = runs[0][0], runs[-1][1]
        if max(left, w - right) >= .25 * w:
            side = 'left' if left > w - right else 'right'
            # Aim nearer the wall end so the approach does not pass the entrance wide.
            target = max(.15, left / w - .05) if side == 'left' else min(.85, right / w + .05)
            kind = 'edge'
        else:
            kind = 'wall'
    if target is not None:
        x = int(target * w)
        floor = hsv[int(.72 * h):int(.95 * h), max(0, x - int(.06*w)):min(w, x + int(.06*w))]
        if not np.mean((floor[:, :, 1] < 90) & (floor[:, :, 2] > 12)) >= .65:
            target, kind = None, 'wall'
    blocked = bool(mask[int(.78*h):int(.96*h), int(.38*w):int(.62*w)].mean() > .12)
    # This is the wall side, whereas `side` above describes the open floor side.
    wall_side = None
    visible_columns = np.flatnonzero(mask.any(axis=0))
    if len(visible_columns):
        left, right = visible_columns[0], visible_columns[-1] + 1
        if right >= w - 4 and left > 4:
            wall_side = 1
        elif left <= 4 and right < w - 4:
            wall_side = 0
        elif left > w / 2:
            wall_side = 1
        elif right < w / 2:
            wall_side = 0
    return dict(rgb=rgb, mask=mask, columns=wall_columns, runs=runs, target=target, kind=kind,
                side=side, wall_side=wall_side, blocked=blocked)


def visible_wall_ends(view, camera, roll, pitch):
    """Find wall ends using floor projection, or visible wall height as a fallback."""
    view['end_pixels'] = []
    mask = view['mask']
    h, w = mask.shape
    f = h / (2 * math.tan(math.radians(camera['fovy_deg']) / 2))
    angle = math.radians(camera['pitch_deg']) + pitch
    cp, sp, cr, sr = math.cos(angle), math.sin(angle), math.cos(roll), math.sin(roll)
    rotation = np.array([[cp, sp*sr, sp*cr], [0, cr, -sr], [-sp, cp*sr, cp*cr]])
    # Thin plywood ramp edges have the same color, but are not maze walls.
    horizon = max(0, min(h - 1, int(h / 2 - f * math.tan(angle))))
    # Walls below the lens should appear below the horizon. Prefer their height if the angle disagrees.
    if mask[:horizon].sum() > .5 * mask.sum():
        return wall_height_ends(view, camera, rotation, f)
    columns = np.flatnonzero(view['columns'] & (mask[horizon:int(.90*h)].sum(axis=0) >= .07*h))
    if len(columns) < 12:
        return wall_height_ends(view, camera, rotation, f)
    tops = mask[:, columns].argmax(axis=0)
    bottoms = h - 1 - mask[::-1, columns].argmax(axis=0)
    use_base = bottoms < h - 5
    rows = np.where(use_base, bottoms, tops)
    rays = np.column_stack([np.ones(len(columns)), -(columns - (w-1)/2)/f,
                            -(rows - (h-1)/2)/f])
    rays = rays @ rotation.T
    valid = (rays[:, 2] < -.02) & (use_base | ((tops > 2) & (tops < h - 2)))
    heights = np.where(use_base, camera['height_m'], camera['height_m'] - camera['wall_height_m'])
    points = -heights[:, None] * rays[:, :2] / np.minimum(rays[:, 2:3], -.001)
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
        column = int(columns[indices[end_index]])
        if not 4 < column < w - 5:
            continue
        middle = int((tops[indices[end_index]] + bottoms[indices[end_index]]) / 2)
        neighbors = mask[max(0, middle - 3):min(h, middle + 4), [column - 5, column + 5]]
        if rows[indices[end_index]] >= h - 7 and min(neighbors.mean(axis=0)) > .5:
            continue
        end = center + tangent * along[end_index]
        if np.dot(tangent, center - end) < 0:
            tangent = -tangent
        if np.dot(normal, -end) < 0:
            normal = -normal
        pixel = (int(columns[indices[end_index]]), int(rows[indices[end_index]]))
        candidates.append((float(np.linalg.norm(end)), end, tangent, normal, pixel))
    if not candidates:
        return wall_height_ends(view, camera, rotation, f)
    candidates.sort(key=lambda item: item[0])
    view['end_pixels'] = [item[4] for item in candidates]
    return [item[1:4] for item in candidates]


def wall_height_ends(view, camera, rotation, f):
    """Use visible wall height when an uncertain camera angle prevents floor projection."""
    mask = view['mask']
    h, w = mask.shape
    regions, _ = label(mask)
    candidates = []
    faces = [(region, bounds) for region, bounds in enumerate(find_objects(regions), 1)
             if np.sum(regions[bounds] == region) >= .01 * h * w]
    for region, bounds in faces:
        columns = np.arange(bounds[1].start, bounds[1].stop)
        face = regions[:, columns] == region
        tops = face.argmax(axis=0)
        bottoms = h - 1 - face[::-1].argmax(axis=0)
        # Bridge the grey support feet without treating their edges as wall ends.
        bottoms = grey_closing(bottoms, size=max(3, int(.10 * w)), mode='nearest')
        for direction in (1, -1):
            indices = np.arange(len(columns))[::direction][:max(12, int(.12 * w))]
            edge = int(columns[indices[0]])
            if not 4 < edge < w - 5:
                continue
            if any(other != region and box[1].start - .05*w <= edge <= box[1].stop + .05*w
                   for other, box in faces):
                continue
            jumps = np.flatnonzero(np.abs(np.diff(bottoms[indices])) > 5)
            if len(jumps):
                indices = indices[:jumps[0] + 1]
            indices = indices[4:]
            if len(indices) < 12:
                continue
            x = columns[indices]
            top, base = tops[indices], bottoms[indices]
            if np.any((top <= 2) | (base >= h - 2) | (base - top < .07 * h)):
                continue
            lines = [np.polyfit(x, rows, 1) for rows in (top, base)]
            fits = [np.polyval(line, x) for line in lines]
            if any(np.median(np.abs(rows - fit)) > 2 for rows, fit in zip((top, base), fits)):
                continue
            rays = []
            for rows in fits:
                ray = np.column_stack([np.ones(len(x)), -(x - (w-1)/2)/f,
                                       -(rows - (h-1)/2)/f]) @ rotation.T
                rays.append(ray / np.linalg.norm(ray[:, :2], axis=1)[:, None])
            span = rays[0][:, 2] - rays[1][:, 2]
            if np.any(span <= 0):
                continue
            distance = camera['wall_height_m'] / span
            if np.any((distance < .2) | (distance > 3)):
                continue
            points = distance[:, None] * rays[1][:, :2]
            offset = np.array([camera['forward_m'], 0.])
            points += offset
            center = points.mean(axis=0)
            _, _, axes = np.linalg.svd(points - center, full_matrices=False)
            tangent = axes[0]
            if np.ptp((points - center) @ tangent) < .08:
                continue
            normal = np.array([-tangent[1], tangent[0]])
            edge_ray = np.array([1., -(edge - (w-1)/2)/f,
                                 -(np.polyval(lines[1], edge) - (h-1)/2)/f]) @ rotation.T
            bearing = edge_ray[:2] / np.linalg.norm(edge_ray[:2])
            denominator = bearing @ normal
            if abs(denominator) < .1:
                continue
            end = offset + bearing * (((center - offset) @ normal) / denominator)
            if end[0] <= 0 or np.linalg.norm(end) > 3:
                continue
            if tangent @ (center - end) < 0:
                tangent = -tangent
            if normal @ -end < 0:
                normal = -normal
            pixel = (edge, int(bottoms[indices[0]]))
            candidates.append((float(np.linalg.norm(end)), end, tangent, normal, pixel))
    candidates.sort(key=lambda item: item[0])
    view['end_pixels'] = [item[4] for item in candidates]
    return [item[1:4] for item in candidates]


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class CorridorSearch:
    use_camera = use_imu = True

    def __init__(self, started, settings, camera_geometry=None):
        from .corridor_walls import load_corridor_settings
        self.started = self.stage_started = started
        self.settings = settings
        self.orientation = CourseNavigation(started, use_camera=False, use_imu=True)
        self.estimate = self.orientation.estimate
        self.camera_geometry = dict(camera_geometry or load_corridor_settings()['camera'])
        self.stage = 'scan'
        self.speed = self.base_speed = self.last_speed = 1.0
        self.forward = 0.0
        self.imu_heading = 0.0
        self.scan_previous_heading = None
        self.turn_degrees = 0.0
        self.view = None
        self.sequence = -1
        self.camera_angle = None
        self.camera_changed = started
        self.position = np.zeros(2)
        self.last_update = started
        self.moving = False
        self.candidates = []
        self.selected = None
        self.approach_goal = None
        self.wall_heading = None
        self.entry_heading = None
        self.approach_side = None
        self.wall_side = None
        self.wall_point = None
        self.wall_samples = deque()
        self.ir_side = None
        self.ir_since = None
        self.last_wall_seen = None
        self.both_seen_since = None
        self.centered_since = None
        self.center_filtered = None
        self.center_last_update = None
        self.finished = self.succeeded = False
        self.reason = 'Waiting for camera and IMU before the full corridor scan.'

    def set_speed(self, speed):
        self.speed = self.base_speed = min(speed, 1.0)

    def hold(self, reason):
        self.forward = 0.0
        self.estimate.heading = self.estimate.lateral = 0.0
        self.centered_since = self.center_filtered = self.center_last_update = None
        self.reason = reason
        return False

    def finish(self, reason):
        self.finished = True
        return self.hold(reason)

    def change_stage(self, stage, now):
        self.stage, self.stage_started = stage, now

    def rotation(self):
        c, s = math.cos(self.imu_heading), math.sin(self.imu_heading)
        return np.array([[c, -s], [s, c]])

    def steer_to(self, desired, forward=1.0):
        error = angle_difference(self.imu_heading, desired)
        self.estimate.heading = float(np.clip(error, -.3, .3))
        self.forward = forward if abs(error) < .12 else 0.0
        return True

    def drive_to(self, goal):
        delta = goal - self.position
        self.steer_to(math.atan2(delta[1], delta[0]))
        return np.linalg.norm(delta) < .045

    def observe_ends(self):
        ends = visible_wall_ends(self.view, self.camera_geometry,
                                 self.estimate.roll, self.estimate.pitch)
        rotation = self.rotation()
        return [(self.position + rotation @ end, rotation @ tangent, rotation @ normal)
                for end, tangent, normal in ends]

    def collect_candidates(self):
        for point, tangent, normal in self.observe_ends():
            match = next((item for item in self.candidates
                          if np.linalg.norm(item['point'] - point) < .14 and
                          item['tangent'] @ tangent > .8), None)
            if match is None:
                self.candidates.append(dict(point=point, tangent=tangent, normal=normal, samples=1))
            else:
                weight = 1 / min(20, match['samples'] + 1)
                for key, value in (('point', point), ('tangent', tangent), ('normal', normal)):
                    match[key] = (1 - weight) * match[key] + weight * value
                match['samples'] += 1

    def choose_entrance(self):
        reliable = [item for item in self.candidates if item['samples'] >= 3]
        if not reliable:
            return False
        pairs = []
        for i, left in enumerate(reliable):
            for right in reliable[i + 1:]:
                gap = right['point'] - left['point']
                if (.28 < np.linalg.norm(gap) < .60 and
                        abs(gap @ left['tangent']) < .12 and
                        left['tangent'] @ right['tangent'] > .85):
                    middle = (left['point'] + right['point']) / 2
                    pairs.append((np.linalg.norm(middle - self.position), left, right, middle))
        if pairs:
            _, left, right, middle = min(pairs, key=lambda pair: pair[0])
            self.selected = left
            tangent = left['tangent'] + right['tangent']
            tangent /= np.linalg.norm(tangent)
            self.approach_goal = middle - .25 * tangent
        else:
            self.selected = min(reliable, key=lambda item: np.linalg.norm(item['point'] - self.position) +
                                .3 / math.sqrt(item['samples']))
            tangent = self.selected['tangent'] / np.linalg.norm(self.selected['tangent'])
            normal = self.selected['normal'] / np.linalg.norm(self.selected['normal'])
            offset = self.position - self.selected['point']
            # From in front, stay on the visible face. A side approach crosses the end.
            visible_face = abs(offset @ tangent) > abs(offset @ normal)
            # Allow for wall thickness and walking drift before the side IR can see the wall.
            clearance = .25 * normal if visible_face else -.19 * normal
            self.approach_goal = self.selected['point'] - .18 * tangent + clearance
        self.wall_heading = math.atan2(tangent[1], tangent[0])
        self.entry_heading = self.wall_heading
        return True

    def sensor_point(self, side, distance):
        point = np.array(self.settings['sensor_positions'][side], dtype=float)
        point[1] += (1 if side == 0 else -1) * distance
        return self.position + self.rotation() @ point

    def wall_before_goal(self):
        mask = self.view['mask']
        h, w = mask.shape
        band = mask[:, int(.38 * w):int(.62 * w)]
        band = band[:, band.any(axis=0)]
        tops = band.argmax(axis=0)
        bottoms = h - 1 - band[::-1].argmax(axis=0)
        use_base = bottoms < h - 5
        rows = np.where(use_base, bottoms, tops)
        f = h / (2 * math.tan(math.radians(self.camera_geometry['fovy_deg']) / 2))
        pitch = math.radians(self.camera_geometry['pitch_deg']) + self.estimate.pitch
        down = pitch + np.arctan((rows - (h - 1) / 2) / f)
        if np.any(down <= 0):
            return True
        heights = np.where(use_base, self.camera_geometry['height_m'],
                           self.camera_geometry['height_m'] - self.camera_geometry['wall_height_m'])
        distance = float(np.median(heights / np.tan(down))) + self.camera_geometry['forward_m']
        return distance < np.linalg.norm(self.approach_goal - self.position) + .2

    def update_approach_side(self):
        side = self.view['wall_side']
        if side is None and self.view['columns'].any() and self.selected is not None:
            local = self.rotation().T @ (self.selected['point'] - self.position)
            if local[0] > 0 and abs(local[1]) > .02:
                side = 0 if local[1] > 0 else 1
        if side is not None and side != self.approach_side:
            self.approach_side = side
            self.ir_since = None

    def set_follow_side(self):
        point = self.wall_point if self.wall_point is not None else self.selected['point']
        local = self.rotation().T @ (point - self.position)
        self.wall_side = 0 if local[1] > 0 else 1

    def acquire_wall(self, side, distance, now):
        self.wall_side = side
        self.wall_point = self.sensor_point(side, distance)
        self.wall_samples.clear()
        self.last_wall_seen = now
        tangent = np.array([math.cos(self.wall_heading), math.sin(self.wall_heading)])
        forward = self.rotation()[:, 0]
        if abs(forward @ tangent) < .7:
            # Clear the wall thickness and the feet before turning into the corridor.
            self.approach_goal = self.wall_point - .25 * tangent + .25 * forward
            self.change_stage('round_end', now)
        else:
            self.change_stage('align_wall', now)

    def update(self, status, now, camera_frame=None):
        dt = max(0, min(.3, now - self.last_update))
        if self.moving:
            # Integrate the previous command, before applying the new walking pace.
            self.position += self.settings['walk_speed'] * self.last_speed * self.forward * dt * self.rotation()[:, 0]
        self.last_update = now
        self.moving = self._update(status, now, camera_frame)
        self.speed = self.base_speed
        if self.stage == 'approach' and self.moving and self.forward > 0 and self.ir_since is None:
            # Walk faster across open floor; slow as soon as the chosen IR sees the wall.
            self.speed *= 1.5
        self.last_speed = self.speed
        return self.moving

    def _update(self, status, now, camera_frame):
        if self.finished:
            return False
        self.forward = 0.0
        self.estimate.heading = self.estimate.lateral = 0.0
        if not self.orientation.update(status, now, None):
            self.ir_since = self.both_seen_since = None
            return self.hold(self.orientation.reason)
        self.imu_heading = self.orientation.imu_heading
        camera = status.get('cameraPitch')
        if camera is not None:
            target = self.orientation.camera_calibration['corridor_servo_deg']
            angle = camera.get('angle')
            if isinstance(angle, bool) or not isinstance(angle, (int, float)) or angle != target:
                return self.hold(f'Waiting for camera pitch {target} degrees before corridor search.')
            if angle != self.camera_angle:
                self.camera_angle, self.camera_changed = angle, now
            if camera.get('moving') or now - self.camera_changed < .5:
                return self.hold('Waiting for the camera pitch to settle.')
        if (camera_frame is None or not self.started <= camera_frame[1] <= now or
                now - camera_frame[1] > .35 or
                (camera is not None and camera_frame[1] < self.camera_changed + .5)):
            self.ir_since = self.both_seen_since = None
            return self.hold('Holding steps: waiting for a fresh camera frame.')
        try:
            volts = ir_readings(status, allow_zero=True)
        except ValueError as error:
            self.ir_since = self.both_seen_since = None
            return self.hold(str(error))
        distances = np.asarray(self.settings['ir_scale_mv_m']) / np.maximum(
            np.asarray(volts) - self.settings['ir_offset_mv'], .001)
        if max(volts) >= 3200 or min(distances) < .04:
            self.ir_since = self.both_seen_since = None
            return self.hold('Waiting for side IR readings within their measurement range.')
        fresh_frame = camera_frame[0] != self.sequence
        if fresh_frame:
            self.sequence = camera_frame[0]
            pitch = math.radians(self.camera_geometry['pitch_deg']) + self.estimate.pitch
            self.view = wall_view(camera_frame[2], camera_pitch=pitch,
                                  fovy_deg=self.camera_geometry['fovy_deg'])
        if now - self.started < 2:
            return self.hold('Settling for 2 seconds; keep the robot still.')
        if self.stage == 'scan':
            if self.scan_previous_heading is not None:
                self.turn_degrees += math.degrees(-angle_difference(self.imu_heading, self.scan_previous_heading))
            self.scan_previous_heading = self.imu_heading
            if fresh_frame:
                self.collect_candidates()
            if self.turn_degrees >= 360:
                if not self.choose_entrance():
                    self.candidates.clear()
                    self.turn_degrees = 0.
                    self.reason = 'No clear wall end in the last turn. Scanning again.'
                    self.estimate.heading = .3
                    return True
                self.change_stage('approach', now)
                return self.hold('Full turn completed. Approaching the nearest entrance candidate.')
            self.estimate.heading = .3
            self.reason = f'Scanning the full circle: {max(0, self.turn_degrees):.0f} / 360 degrees.'
            return True

        if self.stage == 'center':
            return self.center(now, distances)
        both = self.stage in ('enter', 'follow_wall') and self.wall_side is not None and max(distances) <= .30
        self.both_seen_since = (self.both_seen_since if self.both_seen_since is not None else now) if both else None
        if self.both_seen_since is not None and now - self.both_seen_since >= .6:
            self.change_stage('center', now)
            return self.hold('Both IR sensors confirm side walls. Aligning inside the maze.')

        if self.stage == 'approach':
            if fresh_frame:
                self.update_approach_side()
            side = self.approach_side
            if side is not None and distances[side] <= .30:
                if self.ir_side != side or self.ir_since is None:
                    self.ir_side, self.ir_since = side, now
                if now - self.ir_since >= .15:
                    self.acquire_wall(side, distances[side], now)
                    return self.hold('Near wall confirmed by the IR sensor on the camera wall side.')
            else:
                self.ir_since = None
            if self.drive_to(self.approach_goal):
                self.change_stage('align_wall', now)
                return self.hold('At the wall end. Aligning toward the inside of the maze.')
            if (self.forward and self.view['kind'] == 'wall' and self.view['target'] is None and
                    self.view['blocked'] and self.wall_before_goal()):
                return self.hold('The entrance target is hidden by a wall. Holding before approaching farther.')
            wall_side = ('left', 'right')[side] if side is not None else 'unconfirmed'
            self.reason = f'Approaching the wall end; checking only the {wall_side} wall side for IR handover.'
            return True

        if self.stage == 'round_end':
            if self.drive_to(self.approach_goal):
                self.change_stage('align_wall', now)
                return self.hold('Cleared the near wall end. Turning along its inside face.')
            self.reason = 'Moving just beyond the wall end detected by IR.'
            return True

        if self.stage == 'align_wall':
            self.steer_to(self.wall_heading, forward=0)
            if abs(angle_difference(self.imu_heading, self.wall_heading)) < .10:
                self.set_follow_side()
                if distances[self.wall_side] <= .30:
                    self.last_wall_seen = now
                    self.change_stage('follow_wall', now)
                else:
                    self.change_stage('enter', now)
            self.reason = 'Turning toward the wall direction recorded during the scan.'
            return True

        if self.stage == 'enter':
            if distances[self.wall_side] <= .25:
                self.last_wall_seen = now
                self.change_stage('follow_wall', now)
            elif now - self.stage_started > 15 / self.speed:
                self.change_stage('scan', now)
                self.turn_degrees = 0.
                self.scan_previous_heading = self.imu_heading
                self.candidates.clear()
                self.selected = self.approach_goal = self.approach_side = self.wall_side = None
                self.ir_side = self.ir_since = self.last_wall_seen = self.both_seen_since = None
                self.wall_samples.clear()
                return self.hold('No side wall found while entering. Starting another scan.')
            self.steer_to(self.wall_heading)
            self.reason = 'Advancing through the wall end until the chosen side IR sensor confirms the wall.'
            return True

        return self.follow_wall(now, distances)

    def follow_wall(self, now, distances):
        side = self.wall_side
        if side is not None and distances[side] <= .30:
            self.last_wall_seen = now
            point = self.sensor_point(side, distances[side])
            self.wall_samples.append((now, point))
            while self.wall_samples and now - self.wall_samples[0][0] > 2 / self.speed:
                self.wall_samples.popleft()
            if len(self.wall_samples) >= 8:
                points = np.array([p for _, p in self.wall_samples])
                _, _, axes = np.linalg.svd(points - points.mean(0), full_matrices=False)
                direction = axes[0]
                tangent = np.array([math.cos(self.wall_heading), math.sin(self.wall_heading)])
                if direction @ tangent < 0:
                    direction = -direction
                if np.ptp(points @ direction) > .03:
                    measured = math.atan2(direction[1], direction[0])
                    innovation = angle_difference(measured, self.wall_heading)
                    if abs(innovation) < .35:
                        self.wall_heading += .2 * innovation
            self.estimate.lateral = (1 if side == 1 else -1) * (distances[side] - .12)
            self.estimate.heading = angle_difference(self.imu_heading, self.wall_heading)
            self.forward = 1.0
            self.reason = 'Following the inside of the near wall with IR and IMU; waiting for the opposite IR sensor.'
            return True
        grace = now - self.stage_started < 3 or (self.last_wall_seen is not None and now - self.last_wall_seen < .6)
        if grace:
            self.steer_to(self.wall_heading, forward=.5)
            self.reason = 'Holding the wall direction across a brief gap in the side IR reading.'
            return True
        return self.hold('Near wall lost. Holding position rather than circling away from the entrance.')

    def center(self, now, distances):
        if max(distances) > .30:
            self.both_seen_since = None
            self.change_stage('follow_wall', now)
            return self.hold('Opposite wall not stable yet. Continuing to follow the near wall.')
        self.estimate.heading = angle_difference(self.imu_heading, self.entry_heading)
        sensor_y = sum(point[1] for point in self.settings['sensor_positions'])
        self.estimate.lateral = float((distances[1] - distances[0] - sensor_y) / 2)
        measurement = np.array([self.estimate.heading, self.estimate.lateral])
        if self.center_filtered is None:
            self.center_filtered = measurement
        else:
            gain = 1 - math.exp(-(now - self.center_last_update) / (.5 / self.speed))
            self.center_filtered += gain * (measurement - self.center_filtered)
        self.center_last_update = now
        centered = (abs(self.center_filtered[1]) < .015 and abs(self.center_filtered[0]) < .08 and
                    now - self.stage_started > 2 / self.speed)
        self.centered_since = (self.centered_since if self.centered_since is not None else now) if centered else None
        if (self.centered_since is not None and now - self.centered_since >= 1 / self.speed and
                abs(self.estimate.heading) < .12 and abs(self.estimate.lateral) < .025):
            self.succeeded = True
            return self.finish('Inside the maze: both side walls confirmed and robot centered. Stopped.')
        self.forward = 1.0
        self.reason = 'Both side walls confirmed. Centering with IR and IMU.'
        return True

    def report(self):
        return dict(guidance='corridor_search', stage=self.stage, turn_degrees=self.turn_degrees,
                    target=self.view['target'] if self.view else None,
                    scan_candidates=len(self.candidates), approach_side=self.approach_side, wall_side=self.wall_side,
                    speed=self.speed,
                    finished=self.finished, succeeded=self.succeeded, detection_reason=self.reason)

    def debug_image(self):
        panel = Image.new('RGB', (640, 300), (15, 18, 23))
        draw = ImageDraw.Draw(panel)
        draw.text((6, 4), 'Corridor search: full scan, wall end, IR wall following', fill='white')
        if self.view:
            rgb = Image.fromarray(self.view['rgb']).resize((320, 240))
            mask = Image.fromarray((self.view['mask'] * 255).astype(np.uint8)).resize((320, 240)).convert('RGB')
            panel.paste(rgb, (0, 22))
            panel.paste(mask, (320, 22))
            h, w = self.view['mask'].shape
            for x, y in self.view.get('end_pixels', []):
                x, y = x * 320 / w, 22 + y * 240 / h
                for offset in (0, 320):
                    draw.ellipse((offset + x - 5, y - 5, offset + x + 5, y + 5), outline='yellow', width=2)
        side = self.approach_side if self.stage == 'approach' else self.wall_side
        wall_side = ('left', 'right')[side] if side is not None else 'unconfirmed'
        draw.text((6, 266), f'{self.stage} | scan {self.turn_degrees:.0f} degrees | wall {wall_side}', fill='white')
        draw.text((6, 282), self.reason, fill='white')
        return np.asarray(panel)
