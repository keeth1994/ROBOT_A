"""Camera navigation using orientation estimated on the ESP32."""
import json
import math
from copy import copy
from dataclasses import dataclass

import numpy as np

from robot_b91.navigation import Navigation
from robot_b91.paths import ROOT
from robot_b91.camera_calibration import load_camera_scan_settings, servo_to_cradle_deg, cradle_to_servo_deg
from robot_b91.camera_scan import RampScanNavigation


@dataclass
class RampObservation:
    frame: tuple
    rear: bool
    surface: tuple


class CourseNavigation:
    def __init__(self, started, use_camera=True, use_imu=True, guidance='ramp', scan_ramp=False):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config.update(json.loads((ROOT / 'dashboard/navigation.json').read_text()))
        self.camera_pivot = config.pop('camera_pivot_m', None)
        self.camera_calibration = load_camera_scan_settings()
        config['guidance'] = guidance
        self.scan_ramp = scan_ramp and use_camera and guidance == 'ramp'
        self.estimate = (RampScanNavigation(self.camera_calibration, use_imu=use_imu, **config)
                         if self.scan_ramp else Navigation(**config))
        self.camera_base_pitch = self.estimate.camera_pitch
        self.camera_lens_position = np.array([
            self.estimate.camera_x, self.estimate.camera_y, self.estimate.camera_z])
        self.camera_angle = None
        self.camera_changed = started
        self.started = started
        self.updated_at = started
        self.use_camera = use_camera
        self.use_imu = use_imu
        self.imu_heading = 0.0
        self.camera_sequence = -1
        self.reason = 'Waiting for camera and IMU.'
        self.target = 'blue tape' if guidance == 'blue_tape' else 'ramp'
        self.downhill_since = self.flat_since = self.last_slope_sample = None
        self.downhill_seen = False
        self.checking_exit = self.exit_aimed = self.corridor_requested = False
        self.exit_missing_since = None
        self.edge_anchor = None
        self.edge_relocks = 0

    def observe_slope(self, timestamp, pitch):
        if not self.scan_ramp or self.checking_exit or self.corridor_requested:
            return
        settings = self.camera_calibration
        if self.last_slope_sample is not None and timestamp - self.last_slope_sample > .35:
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
                self.checking_exit = True
        else:
            self.flat_since = None

    def camera_target(self, now):
        if not self.scan_ramp:
            return None
        angle = math.degrees(self.estimate.camera_target(now - self.started))
        return round(cradle_to_servo_deg(angle, self.camera_calibration))

    def report(self):
        return self.estimate.report() | {'detection_reason': self.reason,
                                        'edge_relocks': self.edge_relocks,
                                        'checking_ramp_exit': self.checking_exit,
                                        'corridor_requested': self.corridor_requested}

    def hold(self, reason, now, camera_frame):
        self.reason = reason
        if isinstance(camera_frame, RampObservation):
            camera_frame = None  # Do not rerun segmentation for a held control decision.
        if self.use_camera and camera_frame is not None:
            sequence, received, rgb = camera_frame
            if self.started <= received <= now and now - received < 1.5:
                preview = copy(self.estimate)
                preview.frame(received - self.started, rgb, use_imu=False)
                self.estimate.debug = preview.debug
        if self.estimate.debug is not None:
            self.estimate.debug = dict(self.estimate.debug, accepted=False, held=True, reason=reason)
        return False

    def _relock_forward_edges(self, stamp, pose):
        # Called only on distinct, fresh frames in the stationary forward exit
        # check. Reject a real turn, clipped single edge, or unstable visual fit.
        est = self.estimate
        anchor = self.edge_anchor
        self.edge_anchor = None
        if (pose is None or abs(est.heading) >= math.radians(25)
                or abs(pose[0]) > math.radians(10)
                or abs(pose[0] - est.heading) > math.radians(25)):
            return False
        yaw, offset = pose
        if (anchor is None or not 0 < stamp-anchor[1] <= .35
                or abs(yaw-anchor[2]) > math.radians(3) or abs(offset-anchor[3]) > .012
                or abs(self.imu_heading-anchor[4]) > math.radians(3)):
            self.edge_anchor = (stamp, stamp, yaw, offset, self.imu_heading, 1)
            return False
        start, _, first_yaw, first_offset, imu, count = anchor
        if count+1 < 3 or stamp-start < self.camera_calibration['reacquire_s']:
            self.edge_anchor = (start, stamp, first_yaw, first_offset, imu, count+1)
            return False
        est.heading, est.lateral = yaw, offset
        self.edge_relocks += 1
        # Do not mark this rejected frame accepted. Subsequent normal frames
        # must still pass the exit check before the gait can resume.
        return True

    def update(self, status, now, camera_frame):
        self.updated_at = now
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
                self.edge_anchor = None
                self.downhill_since = self.flat_since = self.last_slope_sample = None
                self.exit_missing_since = None
                return self.hold('Holding steps: waiting for fresh ESP32 orientation.', now, camera_frame)
            roll, pitch, heading = map(float, attitude)
            # Preserve visual heading corrections while following the local gyro estimate.
            self.estimate.heading += heading - self.imu_heading
            self.imu_heading = heading
            self.estimate.roll, self.estimate.pitch = roll, pitch
            self.estimate.last_imu = max(0, elapsed - age)
            if elapsed >= 2:
                self.observe_slope(now - age, pitch)
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
                self.edge_anchor = None
                self.camera_angle = angle
                self.camera_changed = now
                self.estimate.last_seen = None
            cradle_angle = math.radians(servo_to_cradle_deg(angle, self.camera_calibration))
            self.estimate.camera_pitch = self.camera_base_pitch + cradle_angle
            if self.camera_pivot is not None:
                # Lens and pivot are expressed in the body frame at cradle zero.
                c, s = math.cos(cradle_angle), math.sin(cradle_angle)
                displacement = self.camera_lens_position - self.camera_pivot
                position = self.camera_pivot + np.array([
                    c * displacement[0] + s * displacement[2], displacement[1],
                    -s * displacement[0] + c * displacement[2]])
                self.estimate.camera_x, self.estimate.camera_y, self.estimate.camera_z = map(float, position)
            self.estimate.view_sign = 1 if math.cos(self.estimate.camera_pitch) >= 0 else -1
            if self.checking_exit and not self.exit_aimed:
                self.estimate.start_search(elapsed, math.radians(
                    servo_to_cradle_deg(angle, self.camera_calibration)), 'forward')
                self.estimate.last_seen = None
                self.exit_aimed = True
            if camera.get('moving') or now - self.camera_changed < .1:
                self.edge_anchor = None
                self.estimate.last_seen = None
                self.exit_missing_since = None
                return self.hold('Holding steps while the camera pitch settles.', now, camera_frame)
            raw = camera_frame.frame if isinstance(camera_frame, RampObservation) else camera_frame
            if raw is not None and raw[1] < self.camera_changed + .1:
                camera_frame = None
            if self.scan_ramp and angle != self.camera_target(now):
                return self.hold('Holding steps while positioning the ramp camera.', now, camera_frame)
            if self.checking_exit and angle != self.camera_calibration['forward_servo_deg']:
                return self.hold('Holding steps while checking the ramp ahead.', now, camera_frame)
        elif self.scan_ramp:
            return self.hold('Holding steps: waiting for camera pitch control.', now, camera_frame)
        surface = None
        if isinstance(camera_frame, RampObservation):
            surface = camera_frame.surface
            camera_frame = camera_frame.frame if camera_frame.rear == (self.estimate.view_sign < 0) else None
        if camera_frame is not None:
            sequence, received, rgb = camera_frame
            if (sequence != self.camera_sequence and self.started <= received <= now and
                    now - received < .35):
                self.camera_sequence = sequence
                if self.checking_exit:
                    previous = self.estimate.debug
                    if previous is None or received - self.started - previous['timestamp'] > .35:
                        self.exit_missing_since = None
                    accepted = self.estimate.frame(received - self.started, rgb, use_imu=True, surface=surface)
                    self._relock_forward_edges(received, None if accepted else self.estimate.debug.get('edge_pose'))
                    if self.estimate.confirm_ramp(received - self.started, accepted):
                        self.checking_exit = self.exit_aimed = self.downhill_seen = False
                        self.downhill_since = self.flat_since = self.exit_missing_since = None
                        self.estimate.searching = False
                        self.estimate.move_started = None
                    elif accepted:
                        self.exit_missing_since = None
                    elif (not self.estimate.debug['surface_detected'] and
                          self.estimate.debug['reason'] != 'Invalid RGB image'):
                        if self.exit_missing_since is None:
                            self.exit_missing_since = received
                        if received - self.exit_missing_since >= self.camera_calibration['exit_check_s']:
                            self.corridor_requested = True
                    else:
                        self.exit_missing_since = None
                elif self.scan_ramp:
                    self.estimate.scan_frame(received - self.started, rgb,
                                             math.radians(servo_to_cradle_deg(self.camera_angle, self.camera_calibration)),
                                             0, self.estimate.camera_x, now=elapsed, surface=surface)
                else:
                    self.estimate.frame(received - self.started, rgb, use_imu=self.use_imu)
        if self.checking_exit:
            reason = ('Ramp ended after downhill and level readings; starting corridor search.'
                      if self.corridor_requested else
                      'Holding steps: checking the forward view after downhill and level readings.')
            return self.hold(reason, now, None)
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
