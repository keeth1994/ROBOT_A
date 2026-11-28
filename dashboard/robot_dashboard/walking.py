import math
from pathlib import Path
import tomllib

from .devices import DeviceError
from .navigation import CourseNavigation
from .corridor_walls import IRWallNavigation, load_corridor_settings
from .corridor_search import CorridorSearch
from .crest import CrestSequence, CREST_WAVE_GAIT
from robot.navigation.camera_calibration import load_camera_scan_settings
from robot.gait import RAMP_SPEED_SCALE

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
PROFILES = {'walk': (), 'course': ('camera', 'imu'), 'course_camera': ('camera',),
            'course_imu': ('imu',),
            'corridor': ('ir', 'imu'), 'corridor_search': ('camera', 'ir', 'imu'), 'dance': ()}
WALK_ACTIONS = (*PROFILES, 'run_course')
RAMP_PROFILES = ('course', 'course_camera')


class Walking:
    def __init__(self, *, crest_enabled=True, crest_position='first', on_message=None):
        self.on_message = on_message
        if crest_position not in ('first', 'last'):
            raise ValueError('Crest position must be first or last.')
        self.crest_position = crest_position
        with (ROOT / 'reference/b91_gait.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_confirmed_frame = 0.0
        self.profile = 'walk'
        self.navigation = None
        self.corridor_settings = None
        self.speed = 1.0
        self.crest_sequence = None
        self.crest_enabled = crest_enabled
        self.base_params = None
        self.resume_until = None
        self.camera_settings = load_camera_scan_settings()
        self.message = 'Stopped. Walking is not started automatically.'

    @property
    def message(self):
        return self._message

    @message.setter
    def message(self, text):
        changed = text != getattr(self, '_message', None)
        self._message = text
        if changed and self.on_message:
            self.on_message(text)

    def state(self):
        period = self.params['period']
        if self.profile in RAMP_PROFILES:
            period /= RAMP_SPEED_SCALE
        return dict(active=self.active, message=self.message,
                    profile=self.profile, mode='dance' if self.profile == 'dance' else 'walk',
                    controller='ESP32',
                    speed=self.speed, period_s=(1.0 if self.profile == 'dance' else period) / self.speed,
                    camera_controlled=self.active and self.camera_start_angle(self.profile) is not None,
                    ramp_gait_phase=self.crest_sequence.phase if self.crest_sequence else None,
                    crest_enabled=self.crest_enabled,
                    crest_position=self.crest_position,
                    navigation=self.navigation.report() if self.navigation else None)

    def camera_start_angle(self, profile):
        if profile == 'corridor_search':
            return self.camera_settings['corridor_servo_deg']
        if profile in RAMP_PROFILES:
            return self.camera_settings['forward_servo_deg']
        return None

    def camera_target(self, now):
        if not self.active:
            return None
        if self.profile in ('course', 'course_camera'):
            return self.navigation.camera_target(now)
        return self.camera_start_angle(self.profile)

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
        self.camera_settings = load_camera_scan_settings()
        with (ROOT / 'reference/b91_gait.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.base_params = dict(self.params)
        return self.controller_settings(profile, self.params)

    def controller_settings(self, profile, params):
        defaults = {'pitch_min': -125 * math.pi / 180, 'lateral_gain': 0, 'strafe_gain': 0, 'yaw_bias': 0}
        names = ('period', 'duty', 'yaw_amplitude', 'pitch_lift', 'pitch_stance',
                 'pitch_min', 'heading_gain', 'lateral_gain', 'strafe_gain', 'pattern', 'yaw_bias')
        settings = defaults | params
        if profile in RAMP_PROFILES:
            settings['period'] /= RAMP_SPEED_SCALE
        if profile in ('corridor', 'corridor_search'):
            self.corridor_settings = load_corridor_settings()
            settings.update(self.corridor_settings['gait'])
        if profile == 'dance':
            settings['pitch_stance'] = 0.0
        # The PC calculates guidance; the ESP32 runs the gait.
        controller_profile = 'course' if profile == 'corridor' else profile
        result = {'profile': controller_profile, **{name: settings[name] for name in names}}
        if 'rear_stride_scale' in settings:
            result['rear_stride_scale'] = settings['rear_stride_scale']
        if 'front_pitch_lift' in settings:
            result['front_pitch_lift'] = settings['front_pitch_lift']
        return result

    def begin(self, status, session, now, profile='walk'):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        if profile not in PROFILES:
            raise ValueError('Unknown walking profile.')
        self.profile = profile
        self.crest_sequence = (CrestSequence(self.crest_position)
                               if profile == 'course' and self.crest_enabled else None)
        self.base_params = dict(self.params)
        self.resume_until = None
        sensors = PROFILES[profile]
        if profile in ('corridor', 'corridor_search'):
            self.corridor_settings = self.corridor_settings or load_corridor_settings()
            self.navigation = (CorridorSearch(now, self.corridor_settings['navigation'])
                               if profile == 'corridor_search' else
                               IRWallNavigation(now, **self.corridor_settings['navigation']))
        else:
            self.navigation = (CourseNavigation(now, use_camera='camera' in sensors, use_imu='imu' in sensors,
                                                scan_ramp=profile in ('course', 'course_camera'))
                               if sensors else None)
        self.session, self.sequence = session, 0
        self.started = self.last_confirmed_frame = now
        self.active = True
        self.message = ('Settling for 2 seconds; keep the robot still.' if sensors or profile == 'dance' else
                        'Walking on the ESP32.')

    def transition_settings(self):
        phase = self.crest_sequence.requested if self.crest_sequence else None
        if phase not in ('crest', 'descent', 'plateau') or self.base_params is None:
            raise DeviceError('No ramp gait change is pending.', 409)
        params = self.base_params | (CREST_WAVE_GAIT if phase == 'crest' else {})
        return params, self.controller_settings(self.profile, params)

    def resume_gait(self, session, now, params):
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        self.session, self.sequence = session, 0
        self.params = params
        self.last_confirmed_frame = now
        self.resume_until = now + 2
        self.crest_sequence.switched(now)
        # Keep heading and offset when orientation resets.
        self.navigation.camera_angle = None
        self.navigation.estimate.last_seen = None
        self.navigation.downhill_since = self.navigation.flat_since = None
        self.navigation.last_slope_sample = None
        self.message = 'Settling after ramp gait change; preserving navigation heading.'

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
        if self.profile in ('corridor', 'corridor_search'):
            self.navigation.set_speed(self.speed)
        elif self.navigation and getattr(self.navigation, 'scan_ramp', False):
            self.navigation.estimate.motion_speed = self.speed
        if self.resume_until is not None:
            if now < self.resume_until:
                self.sequence += 1
                return dict(session=self.session, sequence=self.sequence, speed=self.speed,
                            ready=0, heading=0, heading_reference=0, lateral=0)
            attitude = status.get('orientation')
            if (not status.get('orientationValid') or not 0 <= status.get('imuAgeMs', 999) <= 50 or
                    not isinstance(attitude, (list, tuple)) or len(attitude) != 3 or
                    not all(isinstance(a, (int, float)) and math.isfinite(a) for a in attitude)):
                raise DeviceError('Fresh orientation required after ramp gait change.', 409)
            self.navigation.imu_heading = attitude[2]
            self.resume_until = None
        ready = self.navigation.update(status, now, camera_frame) if self.navigation else True
        if self.crest_sequence:
            self.crest_sequence.observe(now, self.navigation.estimate.pitch, ready and now - self.started >= 2)
            if self.crest_sequence.requested:
                ready = False
        if self.navigation and getattr(self.navigation, 'scan_ramp', False):
            self.navigation.estimate.walking = ready
        if (PROFILES[self.profile] or self.profile == 'dance') and now - self.started < 2:
            self.message = 'Settling for 2 seconds; keep the robot still.'
        elif self.profile == 'dance' and status.get('danceTurnDegrees') is not None:
            self.message = f"Turning while dancing: {abs(status['danceTurnDegrees']):.0f}° / 360°."
        elif self.navigation:
            self.message = self.navigation.reason
        else:
            self.message = 'Dancing on the ESP32.' if self.profile == 'dance' else 'Walking on the ESP32.'
        self.sequence += 1
        estimate = self.navigation.estimate if self.navigation else None
        lateral = estimate.lateral if estimate else 0
        if self.profile in RAMP_PROFILES:
            lateral = estimate.lateral_error
        frame = {'session': self.session, 'sequence': self.sequence,
                 'speed': self.speed, 'ready': int(ready),
                 'heading': estimate.heading if estimate else 0,
                 'heading_reference': self.navigation.imu_heading if self.navigation else 0,
                 'lateral': lateral}
        if self.profile == 'corridor_search':
            frame.update(speed=self.navigation.speed, forward=self.navigation.forward)
            if self.navigation.finished:
                self.stop(self.navigation.reason + ' Outputs off requested.')
        if self.profile == 'corridor' and self.navigation.exit_complete:
            self.stop(self.navigation.reason + ' Outputs off requested.')
        if getattr(self.navigation, 'ramp_finished', False):
            self.stop(self.navigation.reason + ' Outputs off requested.')
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
