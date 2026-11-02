import math
from pathlib import Path
import tomllib

from robot_b91.gait import rocker_targets
from .devices import DeviceError
from .navigation import CourseNavigation

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
# Physical +yaw turns the rear legs clockwise; simulation +yaw is counterclockwise.
YAW_DIRECTION = {'FL_yaw': 1, 'RL_yaw': -1, 'FR_yaw': 1, 'RR_yaw': -1}
PROFILES = {'flat': (), 'course': ('camera', 'imu'), 'course_camera': ('camera',),
            'course_imu': ('imu',), 'course_open': ()}
WALK_ACTIONS = ('start', *(name for name in PROFILES if name != 'flat'))


class Walking:
    def __init__(self):
        with (ROOT / 'dashboard/walking.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_frame = self.last_confirmed_frame = 0.0
        self.channels = {}
        self.profile = 'flat'
        self.navigation = None
        self.speed = 1.0
        self.message = 'Stopped. PC walking is not started automatically.'

    def state(self):
        return dict(active=self.active, message=self.message,
                    profile=self.profile, mode='straight' if self.profile == 'flat' else 'course',
                    controller='PC',
                    speed=self.speed, period_s=self.params['period'] / self.speed,
                    navigation=self.navigation.estimate.report() if self.navigation else None)

    def set_speed(self, speed):
        if (isinstance(speed, bool) or not isinstance(speed, (int, float)) or
                not math.isfinite(speed) or not .25 <= speed <= 2):
            raise ValueError('Walking speed must be between 25% and 200%.')
        self.speed = float(speed)

    def controller_check(self, status):
        if (not status.get('servoReady') or status.get('faulted') or
                status.get('motionProtocol') != 1):
            raise DeviceError('Upload the PC-controlled ESP32 firmware and check servo status.', 409)

    def prepare(self, status):
        self.controller_check(status)
        joints = status.get('joints', [])
        mapping = {j['name']: j['channel'] for j in joints}
        if (set(mapping) != set(JOINTS) or len(joints) != 8 or
                len(set(mapping.values())) != 8 or
                any(not isinstance(ch, int) or isinstance(ch, bool) or not 0 <= ch <= 15
                    for ch in mapping.values())):
            raise DeviceError('ESP32 joint mapping does not match the eight gait joints.', 409)
        self.channels = mapping

    def begin(self, status, session, now, profile='flat'):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or session <= 0:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        if profile not in PROFILES:
            raise DeviceError('Unknown walking profile.', 409)
        path = 'dashboard/walking.toml' if profile == 'flat' else 'reference/b91_gait.toml'
        with (ROOT / path).open('rb') as stream:
            self.params = tomllib.load(stream)
        self.profile = profile
        sensors = PROFILES[profile]
        self.navigation = (CourseNavigation(now, use_camera='camera' in sensors, use_imu='imu' in sensors)
                           if sensors else None)
        self.gait_time = 0.0
        self.targets = rocker_targets(self.params, 0, 0)
        self.session, self.sequence = session, 0
        self.started = self.last_frame = self.last_confirmed_frame = now
        self.active = True
        self.message = ('Settling for 2 seconds; keep the robot still.' if profile != 'flat' else
                        'Walking straight. Gait is calculated on this PC.')

    def frame(self, status, now, camera_frame=None):
        if not self.active:
            raise DeviceError('Walking is stopped.', 409)
        self.controller_check(status)
        if now - self.last_confirmed_frame > 1.0:
            raise DeviceError('PC/network control overrun; restart walking explicitly.', 409)
        if not status.get('externalActive') or status.get('motionSession') != self.session:
            raise DeviceError('ESP32 motion session ended; restart explicitly.', 409)
        settling = 0 if self.profile == 'flat' else 2
        elapsed = max(0, now - self.started - settling)
        ready = self.navigation.update(status, now, camera_frame) if self.navigation else True
        if settling and elapsed == 0:
            self.message = 'Settling for 2 seconds; keep the robot still.'
        elif ready:
            self.gait_time += max(0, now - max(self.last_frame, self.started + settling)) * self.speed
            heading = self.navigation.estimate.heading if self.navigation else 0
            lateral = self.navigation.estimate.lateral if self.navigation else 0
            self.targets = rocker_targets(self.params, elapsed, self.gait_time, heading, lateral)
            if self.navigation:
                self.message = self.navigation.reason
            elif settling:
                self.message = 'Course gait without camera or IMU corrections.'
        else:
            # Hold targets and step phase until sensor observations recover.
            self.message = self.navigation.reason
        self.sequence += 1
        frame = {'session': self.session, 'sequence': self.sequence}
        for name, target in zip(JOINTS, self.targets):
            angle = math.degrees(float(target)) * YAW_DIRECTION.get(name, 1)
            if name.endswith('_pitch'):
                angle += 45
            frame[f'a{self.channels[name]}'] = f'{angle:.5f}'
        self.last_frame = now
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
