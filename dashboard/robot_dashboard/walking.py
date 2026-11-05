import math
from pathlib import Path
import tomllib

from .devices import DeviceError
from .navigation import CourseNavigation

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
PROFILES = {'flat': (), 'course': ('camera', 'imu'), 'course_camera': ('camera',),
            'course_imu': ('imu',), 'course_open': ()}
WALK_ACTIONS = ('start', *(name for name in PROFILES if name != 'flat'))


class Walking:
    def __init__(self):
        with (ROOT / 'dashboard/walking.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_confirmed_frame = 0.0
        self.profile = 'flat'
        self.navigation = None
        self.speed = 1.0
        self.message = 'Stopped. Walking is not started automatically.'

    def state(self):
        return dict(active=self.active, message=self.message,
                    profile=self.profile, mode='straight' if self.profile == 'flat' else 'course',
                    controller='ESP32',
                    speed=self.speed, period_s=self.params['period'] / self.speed,
                    navigation=self.navigation.estimate.report() if self.navigation else None)

    def set_speed(self, speed):
        if (isinstance(speed, bool) or not isinstance(speed, (int, float)) or
                not math.isfinite(speed) or not .25 <= speed <= 2):
            raise ValueError('Walking speed must be between 25% and 200%.')
        self.speed = float(speed)

    def controller_check(self, status):
        if (not status.get('servoReady') or status.get('faulted') or
                status.get('motionProtocol') != 2):
            raise DeviceError('Upload the local-gait ESP32 firmware and check servo status.', 409)

    def prepare(self, status):
        self.controller_check(status)
        joints = status.get('joints', [])
        mapping = {j['name']: j['channel'] for j in joints}
        if (set(mapping) != set(JOINTS) or len(joints) != 8 or
                len(set(mapping.values())) != 8 or
                any(not isinstance(ch, int) or isinstance(ch, bool) or not 0 <= ch <= 15
                    for ch in mapping.values())):
            raise DeviceError('ESP32 joint mapping does not match the eight gait joints.', 409)

    def start_settings(self, profile):
        if profile not in PROFILES:
            raise ValueError('Unknown walking profile.')
        path = 'dashboard/walking.toml' if profile == 'flat' else 'reference/b91_gait.toml'
        with (ROOT / path).open('rb') as stream:
            self.params = tomllib.load(stream)
        defaults = {'pitch_min': -math.pi / 4, 'lateral_gain': 0, 'strafe_gain': 0}
        names = ('period', 'duty', 'yaw_amplitude', 'pitch_lift', 'pitch_stance',
                 'pitch_min', 'heading_gain', 'lateral_gain', 'strafe_gain', 'pattern')
        settings = defaults | self.params
        return {'profile': profile, **{name: settings[name] for name in names}}

    def begin(self, status, session, now, profile='flat'):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        if profile not in PROFILES:
            raise ValueError('Unknown walking profile.')
        self.profile = profile
        sensors = PROFILES[profile]
        self.navigation = (CourseNavigation(now, use_camera='camera' in sensors, use_imu='imu' in sensors)
                           if sensors else None)
        self.session, self.sequence = session, 0
        self.started = self.last_confirmed_frame = now
        self.active = True
        self.message = ('Settling for 2 seconds; keep the robot still.' if profile != 'flat' else
                        'Walking straight. Gait runs on the ESP32.')

    def frame(self, status, now, camera_frame=None):
        if not self.active:
            raise DeviceError('Walking is stopped.', 409)
        self.controller_check(status)
        if now - self.last_confirmed_frame > 1.0:
            raise DeviceError('PC/network control overrun; restart walking explicitly.', 409)
        if not status.get('externalActive') or status.get('motionSession') != self.session:
            raise DeviceError('ESP32 motion session ended; restart explicitly.', 409)
        if status.get('gaitStopping'):
            raise DeviceError('ESP32 is stopping after losing navigation; restart explicitly.', 409)
        ready = self.navigation.update(status, now, camera_frame) if self.navigation else True
        if self.profile != 'flat' and now - self.started < 2:
            self.message = 'Settling for 2 seconds; keep the robot still.'
        elif self.navigation:
            self.message = self.navigation.reason
        else:
            self.message = 'Walking locally on ESP32.'
        self.sequence += 1
        estimate = self.navigation.estimate if self.navigation else None
        frame = {'session': self.session, 'sequence': self.sequence,
                 'speed': self.speed, 'ready': int(ready),
                 'heading': estimate.heading if estimate else 0,
                 'heading_reference': self.navigation.imu_heading if self.navigation else 0,
                 'lateral': estimate.lateral if estimate else 0}
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
