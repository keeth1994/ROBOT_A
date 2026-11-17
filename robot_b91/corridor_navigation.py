"""Fit corridor walls using calibrated simulated IR readings and IMU heading.

Walking speed is an odometry estimate, not a simulator position measurement.
The IR conversion matches sim_corridor's sensor model. Physical sensors need
their own measured calibration before this controller can be used on hardware.
"""
from collections import deque
import math

import numpy as np

from dashboard.robot_dashboard.corridor import ir_readings
from dashboard.robot_dashboard.navigation import CourseNavigation


class IRWallNavigation(CourseNavigation):
    def __init__(self, started, sensor_positions, walk_speed=.04, window_s=1.5):
        super().__init__(started, use_camera=False, use_imu=True)
        self.sensor_positions = np.asarray(sensor_positions, dtype=float)
        self.walk_speed = walk_speed
        self.window_s = window_s
        self.position = np.zeros(2)
        self.history = [deque(), deque()]
        self.last_update = None
        self.wall_heading = None
        self.last_both_seen = None
        self.exit_started = None

    def reset_walls(self):
        self.history = [deque(), deque()]
        self.position[:] = 0
        self.last_update = None
        self.wall_heading = None
        self.last_both_seen = None
        self.exit_started = None

    def continue_past_walls(self, volts, now):
        # Side sensors pass the wall ends before the rear feet have cleared them.
        # Briefly keep the last measured direction, without using the course map.
        if self.exit_started is None:
            if (self.wall_heading is None or self.last_both_seen is None or
                    now - self.last_both_seen > .15 or now - self.started < 3 or
                    abs(self.estimate.heading) > .2 or abs(self.estimate.lateral) > .04):
                return False
            self.exit_started = now
        limit = 10 if max(volts) < 150 else 2.5
        if now - self.exit_started > limit:
            return False
        error = self.imu_heading - self.wall_heading
        self.estimate.heading = math.atan2(math.sin(error), math.cos(error))
        self.estimate.lateral = 0.0
        self.reason = 'Continuing straight briefly after the wall signals ended.'
        return True

    def update(self, status, now, camera_frame=None):
        if not super().update(status, now, None):
            self.reset_walls()
            return False
        try:
            volts = ir_readings(status)
            if min(volts) < 150 and self.continue_past_walls(volts, now):
                return True
            if min(volts) < 150 or max(volts) >= 3200:
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
        for side, voltage in enumerate(volts):
            point = self.sensor_positions[side].copy()
            point[1] += (1 if side == 0 else -1) * 120 / voltage
            self.history[side].append((now, self.position + rotation @ point))
            while self.history[side] and now - self.history[side][0][0] > self.window_s:
                self.history[side].popleft()
        if min(map(len, self.history)) < 4:
            self.reason = 'Collecting IR readings to estimate wall direction.'
            return True

        # Fit one direction to the two walls, keeping their centers separate.
        # A full step cycle in the history reduces the effect of body rocking.
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
