"""Camera navigation using orientation estimated on the ESP32."""
import json
import math
from copy import copy

import numpy as np

from robot_b91.navigation import Navigation
from robot_b91.paths import ROOT


class CourseNavigation:
    def __init__(self, started, use_camera=True, use_imu=True, guidance='ramp'):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config.update(json.loads((ROOT / 'dashboard/navigation.json').read_text()))
        self.camera_servo_direction = config.pop('camera_servo_direction', 1)
        if self.camera_servo_direction not in (-1, 1):
            raise ValueError('Camera servo direction must be 1 or -1.')
        config['guidance'] = guidance
        self.estimate = Navigation(**config)
        self.camera_base_pitch = self.estimate.camera_pitch
        self.camera_angle = None
        self.camera_changed = started
        self.started = started
        self.use_camera = use_camera
        self.use_imu = use_imu
        self.imu_heading = 0.0
        self.camera_sequence = -1
        self.reason = 'Waiting for camera and IMU.'
        self.target = 'blue tape' if guidance == 'blue_tape' else 'course'

    def report(self):
        return self.estimate.report() | {'detection_reason': self.reason}

    def hold(self, reason, now, camera_frame):
        self.reason = reason
        if self.use_camera and camera_frame is not None:
            sequence, received, rgb = camera_frame
            if self.started <= received <= now and now - received < 1.5:
                preview = copy(self.estimate)
                preview.frame(received - self.started, rgb, use_imu=False)
                self.estimate.debug = preview.debug
        if self.estimate.debug is not None:
            self.estimate.debug = dict(self.estimate.debug, accepted=False, held=True, reason=reason)
        return False

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
                return self.hold('Holding steps: waiting for fresh ESP32 orientation.', now, camera_frame)
            roll, pitch, heading = map(float, attitude)
            # Preserve visual heading corrections while following the local gyro estimate.
            self.estimate.heading += heading - self.imu_heading
            self.imu_heading = heading
            self.estimate.roll, self.estimate.pitch = roll, pitch
            self.estimate.last_imu = max(0, elapsed - age)
        if not self.use_camera:
            self.reason = 'Walking with IMU heading; no camera or side-offset correction.'
            return True
        camera = status.get('cameraPitch')
        if camera is not None:
            angle = camera.get('angle')
            if (isinstance(angle, bool) or not isinstance(angle, (int, float)) or
                    not math.isfinite(angle) or not -45 <= angle <= 45):
                self.camera_angle = None
                self.estimate.last_seen = None
                return self.hold('Holding steps: enable the camera pitch servo at 0 degrees first.', now, camera_frame)
            if angle != self.camera_angle:
                self.camera_angle = angle
                self.camera_changed = now
                self.estimate.last_seen = None
            self.estimate.camera_pitch = self.camera_base_pitch + math.radians(angle * self.camera_servo_direction)
            if camera.get('moving') or now - self.camera_changed < .5:
                self.estimate.last_seen = None
                return self.hold('Holding steps while the camera pitch settles.', now, camera_frame)
            if camera_frame is not None and camera_frame[1] < self.camera_changed + .5:
                camera_frame = None
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < 1.5):
                self.camera_sequence = sequence
                self.estimate.frame(received - self.started, rgb, use_imu=self.use_imu)
        if self.estimate.debug and self.estimate.debug.get('held'):
            return self.hold('Holding steps: waiting for a fresh camera frame.', now, None)
        ready = (self.estimate.ready(elapsed) if self.use_imu else
                 self.estimate.last_seen is not None and
                 elapsed - self.estimate.last_seen <= self.estimate.blind_timeout)
        if ready:
            if self.estimate.debug and not self.estimate.debug['accepted']:
                self.reason = f'Continuing briefly using the last valid {self.target} direction.'
            else:
                self.reason = (f'Following {self.target} with camera and IMU.' if self.use_imu else
                               f'Following {self.target} with camera only. No tilt compensation.')
        else:
            detail = self.estimate.debug['reason'] if self.estimate.debug else f'Waiting for visible {self.target}'
            self.reason = 'Holding steps: ' + detail + '.'
        return ready
