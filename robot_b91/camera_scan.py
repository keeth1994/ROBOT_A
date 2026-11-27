"""Ramp search using RGB, camera angle and optional IMU readings."""

import math

from robot_b91.navigation import Navigation

BACKWARD_RETRY_DISTANCE_M = .10


def flip_target(elapsed, start, end, settings):
    """Pass quickly through straight down and approach each viewing angle smoothly."""
    progress = min(1, max(0, elapsed / settings['sweep_s']))
    factor = settings['downward_speed_factor']
    fraction = (1 - math.tanh(factor * math.cos(math.pi * progress)) / math.tanh(factor)) / 2
    return start + fraction * (end - start)


class RampScanNavigation(Navigation):
    def __init__(self, settings, use_imu=True, **config):
        config['max_blind_s'] = settings['cache_s']
        super().__init__(**config)
        self.settings = settings
        self.use_imu = use_imu
        self.mount_pitch = self.camera_pitch
        self.view_sign = 1
        self.walking = False
        self.view = 'forward'
        self.searching = False
        self.move_started = None
        self.move_from = math.radians(settings['forward_deg'])
        self.search_wait_since = None
        self.reacquired_since = self.last_confirmation = None
        self.climbs_seen = 0
        self.slope_stage = 'first_climb'
        self.slope_since = self.last_slope_sample = None
        self.backward_pending = False
        self.backward_retry = None
        self.retry_distance = 0.0
        self.retry_updated = None
        self.retry_gyro_heading = self.retry_last_heading = 0.0
        self.retry_using_vision = False
        self.motion_speed = 1.0

    def observe_slope(self, timestamp, pitch):
        """Recognize climb, descent, then second climb from IMU pitch."""
        if not self.use_imu or timestamp < 2 or self.climbs_seen == 2:
            return False
        if pitch is None or not math.isfinite(pitch):
            self.slope_since = self.last_slope_sample = None
            return False
        if self.last_slope_sample is not None and not 0 < timestamp - self.last_slope_sample <= .9:
            self.slope_since = None
        self.last_slope_sample = timestamp
        threshold = math.radians(self.settings['downhill_deg'])
        matches = pitch >= threshold if self.slope_stage == 'descent' else pitch <= -threshold
        if not matches:
            self.slope_since = None
            return False
        if self.slope_since is None:
            self.slope_since = timestamp
        if timestamp - self.slope_since < self.settings['downhill_confirm_s']:
            return False
        self.slope_since = None
        if self.slope_stage == 'first_climb':
            self.climbs_seen = 1
            self.slope_stage = 'descent'
        elif self.slope_stage == 'descent':
            self.slope_stage = 'second_climb'
        else:
            self.climbs_seen = 2
            self.slope_stage = 'complete'
            if self.view != 'backward':
                self.backward_pending = True
            return True
        return False

    def view_target(self):
        return math.radians(self.settings[self.view + '_deg'])

    def camera_target(self, timestamp):
        target = self.view_target()
        if self.move_started is None:
            return target
        return flip_target(timestamp - self.move_started, self.move_from, target, self.settings)

    def flip(self, timestamp, angle):
        self.start_search(timestamp, angle, 'backward' if self.view == 'forward' else 'forward')

    def start_search(self, timestamp, angle, view):
        self.view = view
        if view == 'backward':
            self.backward_pending = False
        self.searching = True
        self.walking = False
        self.move_started = timestamp
        self.move_from = angle
        self.search_wait_since = None
        self.reacquired_since = self.last_confirmation = None

    def confirm_ramp(self, now, accepted):
        if not accepted or (self.last_confirmation is not None and now - self.last_confirmation > .35):
            self.reacquired_since = None
        self.last_confirmation = now
        if not accepted:
            return False
        if self.reacquired_since is None:
            self.reacquired_since = now
        return now - self.reacquired_since >= self.settings['reacquire_s']

    def imu(self, timestamp, acceleration, angular_velocity):
        previous = self.last_imu
        super().imu(timestamp, acceleration, angular_velocity)
        if self.last_imu == timestamp:
            self.observe_slope(timestamp, self.pitch)
        else:
            self.observe_slope(timestamp, None)
        if previous is not None and self.last_imu == timestamp and self.walking:
            dt = timestamp - previous
            if 0 < dt <= .1 and self.last_seen is not None:
                # Speed is a gait estimate, not simulator position or velocity.
                self.lateral += self.settings['estimated_speed_m_s'] * math.sin(self.heading) * dt

    def scan_frame(self, timestamp, rgb, angle, angular_speed, camera_x, now=None):
        # Dashboard video can arrive before the current IMU sample. Search timing
        # uses control time while ramp observation age keeps the image timestamp.
        now = timestamp if now is None else now
        was_retrying = self.backward_retry is not None
        if was_retrying:
            turn = self.heading - self.retry_last_heading
            self.retry_gyro_heading += math.atan2(math.sin(turn), math.cos(turn))
        if self.backward_retry == 'advance':
            dt = max(0, min(.3, now - self.retry_updated))
            if self.walking:
                self.retry_distance += self.settings['estimated_speed_m_s'] * self.motion_speed * dt
            self.retry_updated = now
        self.camera_pitch = self.mount_pitch + angle
        self.camera_x = camera_x
        self.view_sign = 1 if math.cos(self.camera_pitch) >= 0 else -1
        accepted = self.frame(timestamp, rgb, use_imu=self.use_imu)
        fresh_imu = (not self.use_imu or
                     self.last_imu is not None and 0 <= now - self.last_imu <= .05)
        if fresh_imu and now >= 2:
            if self.backward_retry == 'advance':
                if self.retry_distance >= BACKWARD_RETRY_DISTANCE_M:
                    self.backward_retry = 'backward'
                    self.start_search(now, angle, 'backward')
            elif (self.backward_pending and self.view == 'forward' and accepted and
                    self.debug.get('edge_source') == 'paired' and
                    abs(angle - self.view_target()) <= math.radians(self.settings['view_tolerance_deg']) and
                    abs(angular_speed) < math.radians(5)):
                # Refresh position from both forward edges before the second-climb flip.
                self.backward_retry = 'backward'
                # Gyro changes now measure turning away from the pre-flip direction.
                self.heading = 0.0
                self.retry_gyro_heading = 0.0
                self.start_search(now, angle, 'backward')
            elif self.searching:
                target = self.view_target()
                arrived = (abs(angle - target) <= math.radians(self.settings['view_tolerance_deg']) and
                           abs(angular_speed) < math.radians(5))
                if arrived:
                    if self.search_wait_since is None:
                        self.search_wait_since = now
                    if self.backward_retry == 'backward' and not (
                            accepted and self.debug.get('edge_source') == 'paired'):
                        self.backward_retry = 'forward'
                        self.start_search(now, angle, 'forward')
                    elif self.backward_retry == 'forward':
                        self.backward_retry = 'advance'
                        self.retry_distance = 0.0
                        self.retry_updated = now
                        self.searching = False
                        self.move_started = None
                        self.search_wait_since = None
                    elif self.confirm_ramp(now, accepted):
                        self.searching = False
                        self.move_started = None
                        self.search_wait_since = None
                        self.backward_retry = None
                    if (self.backward_retry is None and self.searching and self.reacquired_since is None and
                            now - self.search_wait_since >= self.settings['search_view_s']):
                        self.flip(now, angle)
                else:
                    self.search_wait_since = None
                    self.confirm_ramp(now, False)
            elif not accepted:
                last_good = self.last_seen if self.last_seen is not None else 2.0
                if now - last_good >= self.settings['loss_timeout_s']:
                    self.flip(now, angle)
        self.retry_using_vision = (self.backward_retry == 'advance' and fresh_imu and
                                   accepted and self.debug.get('edge_source') == 'paired' and
                                   self.view_sign > 0)
        if was_retrying and self.backward_retry is not None and not self.retry_using_vision:
            # Keep the original gyro direction as fallback when the forward pair is unavailable.
            self.heading = self.retry_gyro_heading
        if self.backward_retry is not None:
            self.retry_last_heading = self.heading
        if self.searching:
            self.debug.update(held=True, reason=f'Walking paused; searching the {self.view} view for ramp guidance')
        elif self.backward_retry == 'advance':
            guidance = 'Following both forward edges' if self.retry_using_vision else 'Using the saved gyro direction'
            self.debug['reason'] = (f'{guidance} before retrying the backward view: '
                                    f'{self.retry_distance * 100:.1f} / {BACKWARD_RETRY_DISTANCE_M * 100:g} cm estimated')
        elif self.backward_pending and accepted:
            self.debug['reason'] += '; waiting for both forward edges before turning camera backward'
        elif not accepted and self.last_seen is not None:
            if now - self.last_seen <= self.settings['loss_timeout_s']:
                self.debug['reason'] += '; using recent ramp observation'
            else:
                self.debug['reason'] += '; ramp guidance lost; walking held'
        return accepted

    def ready(self, timestamp):
        if self.backward_retry == 'advance':
            fresh_imu = (not self.use_imu or self.last_imu is not None and
                         0 <= timestamp - self.last_imu <= .05)
            return (timestamp >= 2 and fresh_imu and
                    timestamp - self.retry_updated <= self.settings['loss_timeout_s'])
        fresh = (super().ready(timestamp) if self.use_imu else
                 self.last_seen is not None and timestamp - self.last_seen <= self.blind_timeout)
        return (timestamp >= 2 and fresh and not self.searching and
                timestamp - self.last_seen <= self.settings['loss_timeout_s'])

    @property
    def lateral_error(self):
        if self.backward_retry == 'advance' and not self.retry_using_vision:
            return 0.0
        return super().lateral_error

    def report(self):
        report = super().report()
        now = self.last_imu or 0
        age = None if self.last_seen is None else max(0, now - self.last_seen)
        state = 'tracking' if self.last_seen is not None else 'waiting_for_ramp'
        if self.searching:
            state = 'turning' if self.search_wait_since is None else 'searching'
        elif self.backward_retry == 'advance':
            state = 'advancing_before_retry'
        report.update(camera_pitch_deg=math.degrees(self.camera_pitch),
                      camera_view=self.view, camera_search_state=state,
                      uphill_slopes_seen=self.climbs_seen, slope_stage=self.slope_stage,
                      cache_age_s=age,
                      cache_confidence=0 if age is None else max(0, 1 - age / self.blind_timeout),
                      using_cached_ramp=bool(not self.searching and self.debug and not self.debug['accepted'] and
                                             age is not None and age <= self.settings['loss_timeout_s']))
        return report
