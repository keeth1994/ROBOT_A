"""Ramp search using RGB, camera angle and optional IMU readings."""

import math

from robot_b91.navigation import Navigation


def flip_target(elapsed, start, end, settings):
    """Pass quickly through straight down and approach each viewing angle smoothly."""
    progress = min(1, max(0, elapsed / settings['sweep_s']))
    factor = settings['downward_speed_factor']
    fraction = (1 - math.tanh(factor * math.cos(math.pi * progress)) / math.tanh(factor)) / 2
    return start + fraction * (end - start)


class RampScanNavigation(Navigation):
    def __init__(self, settings, use_imu=True, search_forward_deg=None, **config):
        config['max_blind_s'] = settings['cache_s']
        super().__init__(**config)
        self.settings = settings
        self.use_imu = use_imu
        self.mount_pitch = self.camera_pitch
        self.view_sign = 1
        self.walking = False
        self.view = 'forward'
        self.forward_angle = math.radians(settings['forward_deg'])
        self.search_forward_deg = search_forward_deg
        self.searching = False
        self.move_started = None
        self.move_from = self.forward_angle
        self.search_wait_since = None
        self.reacquired_since = None

    def view_target(self):
        return self.forward_angle if self.view == 'forward' else math.radians(self.settings['backward_deg'])

    def camera_target(self, timestamp):
        target = self.view_target()
        if self.move_started is None:
            return target
        return flip_target(timestamp - self.move_started, self.move_from, target, self.settings)

    def flip(self, timestamp, angle):
        self.start_search(timestamp, angle, 'backward' if self.view == 'forward' else 'forward')

    def start_search(self, timestamp, angle, view):
        self.view = view
        self.searching = True
        self.walking = False
        self.move_started = timestamp
        self.move_from = angle
        self.search_wait_since = self.reacquired_since = None

    def imu(self, timestamp, acceleration, angular_velocity):
        previous = self.last_imu
        super().imu(timestamp, acceleration, angular_velocity)
        if previous is not None and self.last_imu == timestamp and self.walking:
            dt = timestamp - previous
            if 0 < dt <= .1 and self.last_seen is not None:
                # Speed is a gait estimate, not simulator position or velocity.
                self.lateral += self.settings['estimated_speed_m_s'] * math.sin(self.heading) * dt

    def scan_frame(self, timestamp, rgb, angle, angular_speed, camera_x):
        self.camera_pitch = self.mount_pitch + angle
        self.camera_x = camera_x
        self.view_sign = 1 if math.cos(self.camera_pitch) >= 0 else -1
        accepted = self.frame(timestamp, rgb, use_imu=self.use_imu)
        fresh_imu = (not self.use_imu or
                     self.last_imu is not None and 0 <= timestamp - self.last_imu <= .05)
        if fresh_imu and timestamp >= 2:
            if self.searching:
                target = self.view_target()
                arrived = (timestamp - self.move_started >= self.settings['sweep_s'] and
                           abs(angle - target) <= math.radians(self.settings['view_tolerance_deg']) and
                           abs(angular_speed) < math.radians(5))
                if arrived:
                    if self.search_wait_since is None:
                        self.search_wait_since = timestamp
                    if accepted:
                        if self.reacquired_since is None:
                            self.reacquired_since = timestamp
                        if timestamp - self.reacquired_since >= self.settings['reacquire_s']:
                            self.searching = False
                            self.move_started = None
                            self.search_wait_since = self.reacquired_since = None
                    else:
                        self.reacquired_since = None
                    if (self.searching and
                            timestamp - self.search_wait_since >= self.settings['search_view_s']):
                        self.flip(timestamp, angle)
                else:
                    self.search_wait_since = self.reacquired_since = None
            elif not accepted:
                last_good = self.last_seen if self.last_seen is not None else 2.0
                if timestamp - last_good >= self.settings['loss_timeout_s']:
                    if self.search_forward_deg is None:
                        self.flip(timestamp, angle)
                    else:
                        # Search the forward view first after losing the ramp.
                        self.forward_angle = math.radians(self.search_forward_deg)
                        self.start_search(timestamp, angle, 'forward')
        if self.searching:
            self.debug.update(held=True, reason=f'Walking paused; searching the {self.view} view for ramp guidance')
        elif not accepted and self.last_seen is not None:
            if timestamp - self.last_seen <= self.settings['loss_timeout_s']:
                self.debug['reason'] += '; using recent ramp observation'
            else:
                self.debug['reason'] += '; ramp guidance lost; walking held'
        return accepted

    def ready(self, timestamp):
        fresh = (super().ready(timestamp) if self.use_imu else
                 self.last_seen is not None and timestamp - self.last_seen <= self.blind_timeout)
        return (fresh and not self.searching and
                timestamp - self.last_seen <= self.settings['loss_timeout_s'])

    def report(self):
        report = super().report()
        now = self.last_imu or 0
        age = None if self.last_seen is None else max(0, now - self.last_seen)
        state = 'tracking' if self.last_seen is not None else 'waiting_for_ramp'
        if self.searching:
            state = 'turning' if self.search_wait_since is None else 'searching'
        report.update(camera_pitch_deg=math.degrees(self.camera_pitch),
                      camera_view=self.view, camera_search_state=state,
                      cache_age_s=age,
                      cache_confidence=0 if age is None else max(0, 1 - age / self.blind_timeout),
                      using_cached_ramp=bool(not self.searching and self.debug and not self.debug['accepted'] and
                                             age is not None and age <= self.settings['loss_timeout_s']))
        return report
