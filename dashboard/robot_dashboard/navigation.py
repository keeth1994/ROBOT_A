"""Camera navigation using orientation estimated on the ESP32."""
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
        self.imu_heading = 0.0
        self.camera_sequence = -1
        self.reason = 'Waiting for camera and IMU.'

    def update(self, status, now, camera_frame):
        elapsed = now - self.started
        if self.use_imu:
            try:
                age = float(status.get('imuAgeMs', math.inf)) / 1000
                attitude = np.asarray(status.get('orientation'), dtype=float)
                valid = (status.get('orientationValid') and
                         0 <= age <= .05 and attitude.shape == (3,) and np.isfinite(attitude).all())
            except (ValueError, TypeError):
                valid = False
            if not valid:
                self.reason = 'Holding steps: waiting for fresh ESP32 orientation.'
                return False
            roll, pitch, heading = map(float, attitude)
            # Preserve visual heading corrections while following the local gyro estimate.
            self.estimate.heading += heading - self.imu_heading
            self.imu_heading = heading
            self.estimate.roll, self.estimate.pitch = roll, pitch
            self.estimate.last_imu = max(0, elapsed - age)
        if not self.use_camera:
            self.reason = 'Walking with IMU heading; no camera or side-offset correction.'
            return True
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < 1.5):
                self.camera_sequence = sequence
                self.estimate.frame(received - self.started, rgb, use_imu=self.use_imu)
        ready = (self.estimate.ready(elapsed) if self.use_imu else
                 self.estimate.last_seen is not None and
                 elapsed - self.estimate.last_seen <= self.estimate.blind_timeout)
        if ready:
            if self.estimate.debug and not self.estimate.debug['accepted']:
                self.reason = 'Continuing briefly using the last valid ramp direction.'
            else:
                self.reason = ('Following course with camera and IMU.' if self.use_imu else
                               'Following course with camera only. No tilt compensation.')
        else:
            detail = self.estimate.debug['reason'] if self.estimate.debug else 'Waiting for a visible dark course'
            self.reason = 'Holding steps: ' + detail + '.'
        return ready
