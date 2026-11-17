"""Camera navigation using orientation estimated on the ESP32."""
import json
import math
from copy import copy

import numpy as np

from robot_b91.navigation import Navigation
from robot_b91.paths import ROOT
from robot_b91.camera_calibration import load_camera_scan_settings, servo_to_cradle_deg, cradle_to_servo_deg
from robot_b91.camera_scan import RampScanNavigation


class CourseNavigation:
    def __init__(self, started, use_camera=True, use_imu=True, guidance='ramp', scan_ramp=False):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config.update(json.loads((ROOT / 'dashboard/navigation.json').read_text()))
        self.camera_calibration = load_camera_scan_settings()
        config['guidance'] = guidance
        self.scan_ramp = scan_ramp and use_camera and guidance == 'ramp'
        self.estimate = (RampScanNavigation(self.camera_calibration, use_imu=use_imu,
                                          search_forward_deg=servo_to_cradle_deg(
                                              self.camera_calibration['search_forward_servo_deg'],
                                              self.camera_calibration), **config)
                         if self.scan_ramp else Navigation(**config))
        self.camera_base_pitch = self.estimate.camera_pitch
        self.camera_angle = None
        self.camera_changed = started
        self.started = started
        self.use_camera = use_camera
        self.use_imu = use_imu
        self.imu_heading = 0.0
        self.camera_sequence = -1
        self.reason = 'Waiting for camera and IMU.'
        self.target = 'blue tape' if guidance == 'blue_tape' else 'ramp'

    def camera_target(self, now):
        if not self.scan_ramp:
            return None
        angle = math.degrees(self.estimate.camera_target(now - self.started))
        return round(cradle_to_servo_deg(angle, self.camera_calibration))

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
                    not math.isfinite(angle) or not -80 <= angle <= 45):
                self.camera_angle = None
                self.estimate.last_seen = None
                return self.hold('Holding steps: enable the camera pitch servo at 0 degrees first.', now, camera_frame)
            if angle != self.camera_angle:
                self.camera_angle = angle
                self.camera_changed = now
                self.estimate.last_seen = None
            self.estimate.camera_pitch = self.camera_base_pitch + math.radians(
                servo_to_cradle_deg(angle, self.camera_calibration))
            self.estimate.view_sign = 1 if math.cos(self.estimate.camera_pitch) >= 0 else -1
            if camera.get('moving') or now - self.camera_changed < .5:
                self.estimate.last_seen = None
                return self.hold('Holding steps while the camera pitch settles.', now, camera_frame)
            if camera_frame is not None and camera_frame[1] < self.camera_changed + .5:
                camera_frame = None
            if self.scan_ramp and angle != self.camera_target(now):
                return self.hold('Holding steps while positioning the ramp camera.', now, camera_frame)
        elif self.scan_ramp:
            return self.hold('Holding steps: waiting for camera pitch control.', now, camera_frame)
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < 1.5):
                self.camera_sequence = sequence
                if self.scan_ramp:
                    self.estimate.scan_frame(received - self.started, rgb,
                                             math.radians(servo_to_cradle_deg(self.camera_angle, self.camera_calibration)),
                                             0, self.estimate.camera_x)
                else:
                    self.estimate.frame(received - self.started, rgb, use_imu=self.use_imu)
        if (self.estimate.debug and self.estimate.debug.get('held') and
                not (self.scan_ramp and self.estimate.searching)):
            return self.hold('Holding steps: waiting for a fresh camera frame.', now, None)
        ready = (self.estimate.ready(elapsed) if self.use_imu or self.scan_ramp else
                 self.estimate.last_seen is not None and
                 elapsed - self.estimate.last_seen <= self.estimate.blind_timeout)
        if self.scan_ramp and self.estimate.searching:
            self.reason = f'Walking paused; searching the {self.estimate.view} view for ramp guidance.'
        elif ready:
            if self.estimate.debug and not self.estimate.debug['accepted']:
                self.reason = f'Continuing briefly using the last valid {self.target} direction.'
            else:
                self.reason = (f'Following {self.target} with camera and IMU.' if self.use_imu else
                               f'Following {self.target} with camera only. No tilt compensation.')
        else:
            detail = self.estimate.debug['reason'] if self.estimate.debug else f'Waiting for visible {self.target}'
            self.reason = 'Holding steps: ' + detail + '.'
        return ready
