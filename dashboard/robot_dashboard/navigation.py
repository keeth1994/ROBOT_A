"""Camera navigation using orientation estimated on the ESP32."""
import json
import math
from copy import copy

import numpy as np

from robot.navigation.controller import Navigation
from robot.paths import ROOT
from robot.navigation.camera_calibration import load_camera_scan_settings, servo_to_cradle_deg, cradle_to_servo_deg
from robot.navigation.camera_scan import RampScanNavigation


class CourseNavigation:
    def __init__(self, started, use_camera=True, use_imu=True, scan_ramp=False):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config.update(json.loads((ROOT / 'dashboard/navigation.json').read_text()))
        self.camera_calibration = load_camera_scan_settings()
        self.scan_ramp = scan_ramp and use_camera
        self.estimate = (RampScanNavigation(self.camera_calibration, use_imu=use_imu, **config)
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
        self.downhill_since = self.flat_since = self.last_slope_sample = None
        self.downhill_seen = False
        self.corridor_requested = False
        self.ramp_finished = False

    def observe_slope(self, timestamp, pitch):
        if not self.scan_ramp:
            return
        if self.estimate.observe_slope(timestamp - self.started, pitch):
            self.downhill_since = self.flat_since = None
            self.downhill_seen = False
        if self.ramp_finished or self.estimate.climbs_seen < 2:
            return
        settings = self.camera_calibration
        if self.last_slope_sample is not None and not 0 < timestamp - self.last_slope_sample <= .9:
            self.downhill_since = self.flat_since = None
        self.last_slope_sample = timestamp
        if pitch >= math.radians(settings['downhill_deg']):
            if self.downhill_since is None:
                self.downhill_since = timestamp
            if timestamp - self.downhill_since >= settings['downhill_confirm_s']:
                self.downhill_seen = True
        else:
            self.downhill_since = None
        if pitch <= -math.radians(settings['downhill_deg']):
            self.downhill_seen = False
        if self.downhill_seen and abs(pitch) <= math.radians(settings['flat_deg']):
            if self.flat_since is None:
                self.flat_since = timestamp
            if timestamp - self.flat_since >= settings['flat_confirm_s']:
                self.ramp_finished = True
        else:
            self.flat_since = None

    def camera_target(self, now):
        if not self.scan_ramp:
            return None
        angle = math.degrees(self.estimate.camera_target(now - self.started))
        return round(cradle_to_servo_deg(angle, self.camera_calibration))

    def report(self):
        return self.estimate.report() | {'detection_reason': self.reason,
                                        'final_descent_seen': self.downhill_seen,
                                        'ramp_finished': self.ramp_finished,
                                        'corridor_requested': self.corridor_requested}

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
                self.downhill_since = self.flat_since = self.last_slope_sample = None
                if self.scan_ramp:
                    self.estimate.observe_slope(elapsed, None)
                return self.hold('Holding steps: waiting for fresh ESP32 orientation.', now, camera_frame)
            roll, pitch, heading = map(float, attitude)
            # Keep camera corrections while tracking gyro changes.
            self.estimate.heading += heading - self.imu_heading
            self.imu_heading = heading
            self.estimate.roll, self.estimate.pitch = roll, pitch
            self.estimate.last_imu = max(0, elapsed - age)
            if elapsed >= 2:
                self.observe_slope(now - age, pitch)
        if self.ramp_finished:
            return self.hold('Ramp finished: two climbs, final descent and flat ground.', now, None)
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
            if camera.get('moving') or now - self.camera_changed < .1:
                self.estimate.last_seen = None
                return self.hold('Holding steps while the camera pitch settles.', now, camera_frame)
            if camera_frame is not None and camera_frame[1] < self.camera_changed + .1:
                camera_frame = None
            if self.scan_ramp and angle != self.camera_target(now):
                return self.hold('Holding steps while positioning the ramp camera.', now, camera_frame)
        elif self.scan_ramp:
            return self.hold('Holding steps: waiting for camera pitch control.', now, camera_frame)
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < .35):
                self.camera_sequence = sequence
                if self.scan_ramp:
                    self.estimate.scan_frame(received - self.started, rgb,
                                             math.radians(servo_to_cradle_deg(self.camera_angle, self.camera_calibration)),
                                             0, self.estimate.camera_x, now=elapsed)
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
        elif self.scan_ramp and self.estimate.backward_retry == 'advance' and ready:
            self.reason = self.estimate.debug['reason'] + '.'
        elif ready:
            if self.estimate.debug and not self.estimate.debug['accepted']:
                self.reason = 'Continuing briefly using the last valid ramp direction.'
            else:
                if self.estimate.debug.get('edge_source') in ('left', 'right'):
                    self.reason = self.estimate.debug['reason'] + '.'
                else:
                    self.reason = ('Following ramp with camera and IMU.' if self.use_imu else
                                   'Following ramp with camera only. No tilt compensation.')
        else:
            detail = self.estimate.debug['reason'] if self.estimate.debug else 'Waiting for visible ramp'
            self.reason = 'Holding steps: ' + detail + '.'
        return ready
