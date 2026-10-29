import math
from pathlib import Path
import tomllib

from robot_b91.gait import rocker_targets
from .devices import DeviceError

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))


class Walking:
    def __init__(self):
        with (ROOT / 'reference/b91_gait.toml').open('rb') as stream:
            self.params = tomllib.load(stream)['profiles']['flat']
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_frame = 0.0
        self.tilt = None
        self.channels = {}
        self.message = 'Stopped. PC walking is not started automatically.'

    def state(self):
        return dict(active=self.active, message=self.message, tilt_deg=self.tilt,
                    profile='flat', mode='straight', controller='PC')

    def sensor_check(self, status, starting=False):
        if (not status.get('servoReady') or status.get('faulted') or
                status.get('motionProtocol') != 1):
            raise DeviceError('Upload the PC-controlled ESP32 firmware and check servo status.', 409)
        if (not status.get('imuValid') or status.get('imuAgeMs', 10000) > 100):
            raise DeviceError('Fresh BMI270 readings are required; walking stopped.', 409)
        try:
            a, g = status['accel'], status['gyro']
            if len(a) != 3 or len(g) != 3 or not all(math.isfinite(v) for v in (*a, *g)):
                raise ValueError()
            norm = math.sqrt(sum(v*v for v in a))
            if not .5 <= norm <= 1.5:
                raise ValueError()
            self.tilt = math.degrees(math.acos(max(-1, min(1, a[2] / norm))))
        except (KeyError, TypeError, ValueError):
            raise DeviceError('Invalid IMU sample; walking stopped.', 409) from None
        if self.tilt > (30 if starting else 45):
            raise DeviceError('Body tilt too high. Support and level the robot; IMU Z must point up.', 409)

    def prepare(self, status):
        self.sensor_check(status, starting=True)
        joints = status.get('joints', [])
        mapping = {j['name']: j['channel'] for j in joints}
        if (set(mapping) != set(JOINTS) or len(joints) != 8 or
                len(set(mapping.values())) != 8 or
                any(not isinstance(ch, int) or isinstance(ch, bool) or not 0 <= ch <= 15
                    for ch in mapping.values())):
            raise DeviceError('ESP32 joint mapping does not match the eight gait joints.', 409)
        self.channels = mapping

    def begin(self, status, session, now):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or session <= 0:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        self.session, self.sequence = session, 0
        self.started = self.last_frame = now
        self.active = True
        self.message = 'Walking straight. Gait is calculated on this PC.'

    def frame(self, status, now):
        if not self.active:
            raise DeviceError('Walking is stopped.', 409)
        self.sensor_check(status)
        if now - self.last_frame > .25:
            raise DeviceError('PC/network control overrun; restart walking explicitly.', 409)
        if not status.get('externalActive') or status.get('motionSession') != self.session:
            raise DeviceError('ESP32 motion session ended; restart explicitly.', 409)
        elapsed = max(0, now - self.started)
        targets = rocker_targets(self.params, elapsed, elapsed, 0, 0)
        self.sequence += 1
        frame = {'session': self.session, 'sequence': self.sequence}
        for name, target in zip(JOINTS, targets):
            angle = math.degrees(float(target)) + (45 if name.endswith('_pitch') else 0)
            frame[f'a{self.channels[name]}'] = f'{angle:.5f}'
        self.last_frame = now
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
