"""Align to visible ramp edges on flat floor, with no commanded forward stride."""
from collections import deque
import math
from statistics import median

import numpy as np

from robot_b91.navigation import EDGE_SAMPLE_MAX_GAP_S, EDGE_SAMPLE_WINDOW_S

from .devices import DeviceError
from .navigation import CourseNavigation


ALIGNMENT_GAIT = dict(period=2., duty=.8, pattern='crest', yaw_amplitude=.6,
                      pitch_lift=.35, pitch_stance=0., yaw_bias=0.,
                      heading_gain=3., lateral_gain=0., strafe_gain=12., turn_limit=.2)


def check_alignment(status):
    for capability in ('corridorSearchGait', 'crestWaveGait', 'boundedSteering'):
        if status.get(capability) is not True:
            raise DeviceError('Ramp alignment needs updated zero-forward gait firmware.', 409)
    age, attitude = status.get('imuAgeMs'), status.get('orientation')
    if (not status.get('orientationValid') or isinstance(age, bool) or
            not isinstance(age, (int, float)) or not 0 <= age <= 50 or
            not isinstance(attitude, (list, tuple)) or len(attitude) != 3 or
            not all(isinstance(a, (int, float)) and not isinstance(a, bool) and
                    math.isfinite(a) for a in attitude)):
        raise DeviceError('Ramp alignment lost fresh orientation.', 409)
    if max(abs(attitude[0]), abs(attitude[1])) > math.radians(8):
        raise DeviceError('Ramp alignment requires flat floor; tilt limit exceeded.', 409)


class RampAlignment(CourseNavigation):
    def __init__(self, started):
        super().__init__(started, use_camera=True, use_imu=True, scan_ramp=False)
        # Flat entrance alignment needs an independent visual heading on every
        # accepted frame. Ramp tracking's slow gyro correction can otherwise
        # turn an old heading error into a falsely centered lateral estimate.
        self.estimate.require_visual_heading = True
        self.phase = 'observe'
        self.finished = False
        self.command_heading = self.command_lateral = 0.
        # A long rolling median plus Navigation's lateral low-pass caused
        # corrections to follow a pose the robot had already moved past.
        self.samples = deque(maxlen=4)
        self.sample_stamp = None
        self.last_good = started
        self.verified_since = None
        self.best_error = None
        self.progress_at = started
        self.requested_gait = None
        self.stationary_gait = False
        self.settle_until = started + 2

    @staticmethod
    def neutral_feet(status):
        joints = {j.get('name'): j.get('angle') for j in status.get('joints', [])}
        for leg in ('FL', 'RL', 'FR', 'RR'):
            yaw, pitch = joints.get(leg+'_yaw'), joints.get(leg+'_pitch')
            if not all(isinstance(a, (int, float)) and math.isfinite(a) for a in (yaw, pitch)):
                return False
            if abs(yaw) > 2 or pitch < 42:
                return False
        return True

    def update(self, status, now, camera_frame):
        check_alignment(status)
        if now - self.started > 120:
            raise DeviceError('Ramp alignment timed out; outputs off requested.', 409)
        self.command_heading = self.command_lateral = 0.
        if self.requested_gait is not None:
            return False
        if now < self.settle_until:
            self.reason = 'Holding steps: settling the alignment stance and IMU.'
            return False
        ready = super().update(status, now, camera_frame)
        debug = self.estimate.debug or {}
        stamp = self.started + debug.get('timestamp', -math.inf)
        fresh = (ready and debug.get('accepted') and not debug.get('held') and
                 0 <= now - stamp <= .25)
        if not fresh:
            return self.wait_for_edges(now, 'fresh paired ramp edges')

        # In the calibrated forward view the ramp must end above a visible
        # foreground floor gap. A level crest is not an alignment starting area.
        camera = status.get('cameraPitch') or {}
        if camera.get('angle') != self.camera_calibration['forward_servo_deg']:
            raise DeviceError('Ramp alignment requires the calibrated forward camera view.', 409)
        mask = debug.get('mask')
        rows = np.flatnonzero(mask.any(axis=1)) if isinstance(mask, np.ndarray) and mask.ndim == 2 else []
        if not len(rows) or rows[-1] >= .8 * mask.shape[0]:
            return self.wait_for_edges(now, 'visible floor before the ramp')
        self.last_good = now
        if stamp != self.sample_stamp:
            if self.samples and stamp - self.samples[-1][0] > EDGE_SAMPLE_MAX_GAP_S:
                self.samples.clear()
                self.verified_since = None
            self.samples.append((stamp, debug.get('measured_heading_rad', self.estimate.heading),
                                 debug.get('measured_lateral_m', self.estimate.lateral)))
            self.sample_stamp = stamp
        # Keep four individually fresh observations even at slow Wi-Fi cadence.
        while self.samples and now - self.samples[0][0] > EDGE_SAMPLE_WINDOW_S:
            self.samples.popleft()
        if len(self.samples) < 4 or self.samples[-1][0]-self.samples[0][0] < .3 - 1e-9:
            self.reason = 'Holding steps: measuring ramp entrance alignment.'
            return False
        heading = median(s[1] for s in self.samples)
        lateral = median(s[2] for s in self.samples)
        if not all(math.isfinite(v) for v in (heading, lateral)) or abs(heading) > math.radians(20) or abs(lateral) > .10:
            raise DeviceError('Ramp alignment is outside the 20-degree / 10-cm capture range.', 409)

        aligned = abs(heading) <= math.radians(2) and abs(lateral) <= .008
        error = max(abs(heading)/math.radians(2), abs(lateral)/.008)
        if self.best_error is None or error < self.best_error - .2:
            self.best_error, self.progress_at = error, now
        if not aligned and now - self.progress_at > 45:
            raise DeviceError('Ramp alignment made no measurable progress; outputs off requested.', 409)

        if self.phase == 'observe':
            self.phase = 'verify' if aligned and self.neutral_feet(status) else 'correct'
        if self.phase == 'correct':
            if aligned:
                return self.request_stance('hold')
            else:
                self.command_heading, self.command_lateral = heading, lateral
                self.reason = 'Aligning with ramp: sideways and heading correction; no forward stride.'
                return True
        if self.phase == 'verify':
            self.reason = 'Holding steps: verifying centered stance before climbing.'
            if not self.neutral_feet(status):
                if self.stationary_gait:
                    raise DeviceError('Stationary alignment stance was not confirmed; outputs off requested.', 409)
                return self.request_stance('hold')
            if not aligned:
                self.verified_since = None
                if self.stationary_gait:
                    return self.request_stance('correct')
                self.phase = 'correct'
                return False
            # Do not authorize climbing from an old median while the newest
            # observation shows a different pose or the camera is still rocking.
            stable = (max(s[1] for s in self.samples)-min(s[1] for s in self.samples) <= math.radians(2) and
                      max(s[2] for s in self.samples)-min(s[2] for s in self.samples) <= .010)
            if not stable:
                self.verified_since = None
                return False
            self.verified_since = now if self.verified_since is None else self.verified_since
            if now - self.verified_since >= .8 - 1e-9:
                # The next ramp session must inherit the verified median, not
                # a possible single-frame outlier at the moment of handover.
                self.estimate.heading, self.estimate.lateral = heading, lateral
                self.finished = True
                self.phase = 'complete'
                self.reason = 'Ramp alignment complete; feet planted.'
        return False

    def request_stance(self, mode):
        self.requested_gait = mode
        self.phase = 'plant' if mode == 'hold' else 'restart_correction'
        self.command_heading = self.command_lateral = 0.
        self.samples.clear()
        self.verified_since = None
        self.reason = ('Setting a stationary all-feet-down stance for verification.' if mode == 'hold' else
                       'Restoring alignment steps for another measured correction.')
        return False

    def gait_changed(self, mode, now):
        self.requested_gait = None
        self.stationary_gait = mode == 'hold'
        self.phase = 'verify' if self.stationary_gait else 'correct'
        self.settle_until = now + 2
        self.samples.clear()
        self.sample_stamp = self.verified_since = None
        self.last_good = now
        self.imu_heading = 0.
        self.camera_angle = None
        self.estimate.last_seen = None

    def wait_for_edges(self, now, detail):
        self.samples.clear()
        self.verified_since = None
        remaining = max(0., 15. - (now - self.last_good))
        self.reason = f'Holding steps: waiting for {detail}; {remaining:.0f}s recovery grace left.'
        if remaining <= 0:
            raise DeviceError(f'Ramp alignment lost camera guidance: needs {detail} after 15s recovery grace.', 409)
        return False

    def report(self):
        return super().report() | dict(alignment_phase=self.phase, alignment_complete=self.finished,
                                      alignment_samples=len(self.samples),
                                      alignment_stationary=self.stationary_gait)
