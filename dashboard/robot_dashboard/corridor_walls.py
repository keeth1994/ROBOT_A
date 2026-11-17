"""Shared corridor wall fitting from calibrated IR readings and IMU heading.
"""
from collections import deque
import json
import math

import numpy as np

from dashboard.robot_dashboard.corridor import ir_readings
from dashboard.robot_dashboard.navigation import CourseNavigation
from robot_b91.paths import ROOT


def validate_wall_settings(sensor_positions, walk_speed=.04, window_s=1.5,
                           ir_scale_mv_m=(120, 120), ir_offset_mv=(0, 0)):
    positions, scales, offsets = map(np.asarray, (sensor_positions, ir_scale_mv_m, ir_offset_mv))
    if (positions.shape != (2, 2) or scales.shape != (2,) or offsets.shape != (2,) or
            not all(np.isfinite(value).all() for value in (positions, scales, offsets)) or
            np.any(scales <= 0) or not 0 < walk_speed <= .5 or not .2 <= window_s <= 5):
        raise ValueError('Use valid corridor sensor positions, IR calibration and walking speed.')


def load_corridor_settings():
    try:
        settings = json.loads((ROOT / 'reference/corridor.json').read_text())
        gait = settings['gait']
        if not all(math.isfinite(gait[key]) and 0 <= gait[key] <= math.pi / 4
                   for key in ('yaw_amplitude', 'yaw_bias')):
            raise ValueError('Corridor yaw settings must be between 0 and 45 degrees.')
        validate_wall_settings(**settings['navigation'])
        return settings
    except (KeyError, TypeError, ValueError, OSError) as error:
        raise ValueError(f'Invalid corridor settings in reference/corridor.json: {error}') from error


class IRWallNavigation(CourseNavigation):
    def __init__(self, started, sensor_positions, walk_speed=.04, window_s=1.5,
                 ir_scale_mv_m=(120, 120), ir_offset_mv=(0, 0)):
        validate_wall_settings(sensor_positions, walk_speed, window_s, ir_scale_mv_m, ir_offset_mv)
        super().__init__(started, use_camera=False, use_imu=True)
        self.sensor_positions = np.asarray(sensor_positions, dtype=float)
        self.ir_scale_mv_m = np.asarray(ir_scale_mv_m, dtype=float)
        self.ir_offset_mv = np.asarray(ir_offset_mv, dtype=float)
        self.voltages = [None, None]
        self.walk_speed = walk_speed
        self.window_s = window_s
        self.base_walk_speed = walk_speed
        self.base_window_s = window_s
        self.motion_speed = 1.0
        self.position = np.zeros(2)
        self.history = [deque(), deque()]
        self.last_update = None
        self.wall_heading = None
        self.last_both_seen = None
        self.exit_started = None
        self.exit_complete = False
        self.exit_elapsed = 0.0
        self.exit_last_update = None

    def set_speed(self, speed):
        self.motion_speed = speed
        self.walk_speed = self.base_walk_speed * speed
        self.window_s = self.base_window_s / speed

    def reset_walls(self):
        self.history = [deque(), deque()]
        self.position[:] = 0
        self.last_update = None
        self.wall_heading = None
        self.last_both_seen = None
        self.exit_started = None

    def continue_past_walls(self, weak, now):
        if self.exit_started is None:
            if (self.wall_heading is None or self.last_both_seen is None or
                    now - self.last_both_seen > .15 or now - self.started < 3 or
                    abs(self.estimate.heading) > .2 or abs(self.estimate.lateral) > .04):
                return False
            self.exit_started = now
            self.exit_elapsed = 0.0
            self.exit_last_update = now
        self.exit_elapsed += max(0, now - self.exit_last_update) * self.motion_speed
        self.exit_last_update = now
        limit = 10 if all(weak) else 2.5
        if self.exit_elapsed > limit:
            self.exit_complete = all(weak)
            return False
        error = self.imu_heading - self.wall_heading
        self.estimate.heading = math.atan2(math.sin(error), math.cos(error))
        self.estimate.lateral = 0.0
        self.reason = 'Continuing straight briefly after the wall signals ended.'
        return True

    def update(self, status, now, camera_frame=None):
        if self.exit_complete:
            return False
        if not super().update(status, now, None):
            self.reset_walls()
            return False
        try:
            volts = ir_readings(status)
            self.voltages = volts
            distances = self.ir_scale_mv_m / np.maximum(np.asarray(volts) - self.ir_offset_mv, .001)
            weak = distances > .30
            if min(distances) < .04 or max(volts) >= 3200:
                raise ValueError('A wall is too close for a valid IR reading.')
            if any(weak) and self.continue_past_walls(weak, now):
                return True
            if self.exit_complete:
                self.reason = 'Exit movement finished after the wall signals ended.'
                return False
            if any(weak):
                raise ValueError('Both walls must be within the IR sensors\' working range.')
        except ValueError as error:
            self.reset_walls()
            self.reason = 'Holding steps: ' + str(error)
            return False

        self.last_both_seen = now
        self.exit_started = None

        heading = self.imu_heading
        dt = .05 if self.last_update is None else now - self.last_update
        self.last_update = now
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        if now - self.started > 2:
            self.position += self.walk_speed * dt * rotation[:, 0]
        for side, distance in enumerate(distances):
            point = self.sensor_positions[side].copy()
            point[1] += (1 if side == 0 else -1) * distance
            self.history[side].append((now, self.position + rotation @ point))
            while self.history[side] and now - self.history[side][0][0] > self.window_s:
                self.history[side].popleft()
        if min(map(len, self.history)) < 4:
            self.reason = 'Collecting IR readings to estimate wall direction.'
            return True

        groups = [np.array([point for _, point in samples]) for samples in self.history]
        covariance = sum((points - points.mean(0)).T @ (points - points.mean(0))
                         for points in groups)
        _, axes = np.linalg.eigh(covariance)
        direction = axes[:, -1]
        if direction @ rotation[:, 0] < 0:
            direction = -direction
        wall_heading = math.atan2(direction[1], direction[0])
        self.wall_heading = wall_heading
        normal = np.array([-direction[1], direction[0]])
        center = (groups[0].mean(0) + groups[1].mean(0)) / 2
        self.estimate.heading = math.atan2(math.sin(heading - wall_heading),
                                          math.cos(heading - wall_heading))
        self.estimate.lateral = float((self.position - center) @ normal)
        self.reason = 'Following fitted IR walls with IMU heading and estimated walking speed.'
        return True

    def report(self):
        return dict(guidance='corridor', left_mv=self.voltages[0], right_mv=self.voltages[1],
                    steering_rad=self.estimate.heading, lateral_m=self.estimate.lateral,
                    walk_speed_m_s=self.walk_speed, exit_complete=self.exit_complete,
                    detection_reason=self.reason)
