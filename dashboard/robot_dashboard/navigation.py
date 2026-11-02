"""Physical sensor adapter for the same navigation used by the simulator."""
import json
import math

import numpy as np

from robot_b91.navigation import Navigation
from robot_b91.paths import ROOT


class CourseNavigation:
    def __init__(self, started, use_camera=True, use_imu=True):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config.update(json.loads((ROOT / 'dashboard/navigation.json').read_text()))
        self.estimate = Navigation(**config)
        self.started = started
        self.use_camera = use_camera
        self.use_imu = use_imu
        self.camera_sequence = -1
        self.reason = 'Waiting for camera and IMU.'

    def update(self, status, now, camera_frame):
        elapsed = now - self.started
        # Measured tilt tests: sensor X forward, Y left, Z up. No axis reversal.
        # Keep gravity; a single flat reading cannot separate tilt from offset.
        if self.use_imu:
            try:
                age = float(status.get('imuAgeMs', math.inf)) / 1000
                accel = np.asarray(status.get('accel'), dtype=float)
                gyro = np.asarray(status.get('gyro'), dtype=float)
                valid = (status.get('imuReady') and status.get('imuValid') and
                         0 <= age <= .05 and accel.shape == gyro.shape == (3,) and
                         np.isfinite(accel).all() and np.isfinite(gyro).all())
            except (ValueError, TypeError):
                valid = False
            if not valid:
                self.reason = 'Holding steps: waiting for fresh IMU data.'
                return False
            self.estimate.imu(max(0, elapsed - age), accel * 9.80665, np.radians(gyro))
        if not self.use_camera:
            self.reason = 'Course gait with IMU heading; no camera or side-offset correction.'
            return True
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < 1.5):
                self.camera_sequence = sequence
                self.estimate.frame(received - self.started, rgb, use_imu=self.use_imu)
        ready = (self.estimate.ready(elapsed) if self.use_imu else
                 self.estimate.last_seen is not None and elapsed - self.estimate.last_seen <= .5)
        if ready:
            self.reason = ('Following course with camera and IMU.' if self.use_imu else
                           'Following course with camera only; no tilt compensation.')
        else:
            detail = self.estimate.debug['reason'] if self.estimate.debug else 'Waiting for a visible dark course'
            self.reason = 'Holding steps: ' + detail + '.'
        return ready
