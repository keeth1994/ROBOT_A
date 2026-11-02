import math
from pathlib import Path
import tomllib

from robot_b91.gait import rocker_targets
from .devices import DeviceError

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
# Physical +yaw turns the rear legs clockwise; simulation +yaw is counterclockwise.
YAW_DIRECTION = {'FL_yaw': 1, 'RL_yaw': -1, 'FR_yaw': 1, 'RR_yaw': -1}


class Walking:
    def __init__(self):
        with (ROOT / 'dashboard/walking.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_frame = self.last_confirmed_frame = 0.0
        self.channels = {}
        self.message = 'Stopped. PC walking is not started automatically.'

    def state(self):
        return dict(active=self.active, message=self.message,
                    profile='flat', mode='straight', controller='PC')

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

    def begin(self, status, session, now):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or session <= 0:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        self.session, self.sequence = session, 0
        self.started = self.last_frame = self.last_confirmed_frame = now
        self.active = True
        self.message = 'Walking straight. Gait is calculated on this PC.'

    def frame(self, status, now):
        if not self.active:
            raise DeviceError('Walking is stopped.', 409)
        self.controller_check(status)
        if now - self.last_confirmed_frame > 1.0:
            raise DeviceError('PC/network control overrun; restart walking explicitly.', 409)
        if not status.get('externalActive') or status.get('motionSession') != self.session:
            raise DeviceError('ESP32 motion session ended; restart explicitly.', 409)
        elapsed = max(0, now - self.started)
        targets = rocker_targets(self.params, elapsed, elapsed, 0, 0)
        self.sequence += 1
        frame = {'session': self.session, 'sequence': self.sequence}
        for name, target in zip(JOINTS, targets):
            angle = math.degrees(float(target)) * YAW_DIRECTION.get(name, 1)
            if name.endswith('_pitch'):
                angle += 45
            frame[f'a{self.channels[name]}'] = f'{angle:.5f}'
        self.last_frame = now
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
