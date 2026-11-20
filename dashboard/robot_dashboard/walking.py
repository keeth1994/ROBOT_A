import math
from pathlib import Path
import tomllib

from .devices import DeviceError
from .navigation import CourseNavigation
from .corridor_walls import IRWallNavigation, load_corridor_settings
from .corridor_search import CorridorSearch
from .crest import CrestSequence, CREST_GAIT, CREST_WAVE_GAIT, RampSpeedGovernor
from .sidestep import SIDESTEP_PROFILES, SIDESTEP_GAIT, check_sidestep
from robot_b91.camera_calibration import load_camera_scan_settings
from robot_b91.gait import RAMP_SPEED_SCALE, RAMP_STEERING

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
PROFILES = {'walk': (), 'course': ('camera', 'imu'), 'course_camera': ('camera',),
            'course_tape': ('camera', 'imu'), 'course_imu': ('imu',),
            'corridor': ('ir', 'imu'), 'corridor_search': ('camera', 'ir', 'imu'), 'dance': ()}
PROFILES.update({name: ('imu',) for name in SIDESTEP_PROFILES})
WALK_ACTIONS = tuple(PROFILES)
RAMP_PROFILES = ('course', 'course_camera')


class Walking:
    def __init__(self, *, crest_assist=False, exit_clearance=False, adaptive_ramp_speed=False,
                 crest_wave=False, start_at_crest=False):
        if start_at_crest and not (crest_assist or crest_wave):
            raise ValueError('Assisted crest start requires a selected crest gait.')
        with (ROOT / 'reference/b91_gait.toml').open('rb') as stream:
            self.params = tomllib.load(stream)
        self.active = False
        self.session = self.sequence = 0
        self.started = self.last_confirmed_frame = 0.0
        self.profile = 'walk'
        self.navigation = None
        self.corridor_settings = None
        self.speed = 1.0
        self.camera_settings = load_camera_scan_settings()
        self.message = ('Stopped. Assisted crest test: place robot at the first crest.'
                        if start_at_crest else 'Stopped. Walking is not started automatically.')
        self.crest_wave = crest_wave
        self.start_at_crest = start_at_crest
        self.crest_assist = crest_assist or crest_wave
        self.exit_clearance_enabled = exit_clearance or self.crest_assist
        self.crest_sequence = None
        self.resume_until = None
        self.base_params = None
        self.exit_cycles = self.exit_started = self.exit_updated = None
        self.adaptive_ramp_speed = adaptive_ramp_speed or crest_wave
        self.speed_governor = None
        self.effective_speed = self.speed

    def state(self):
        period = self.params['period']
        if self.profile in RAMP_PROFILES:
            period /= RAMP_SPEED_SCALE
        return dict(active=self.active, message=self.message,
                    profile=self.profile, mode='dance' if self.profile == 'dance' else 'walk',
                    controller='ESP32',
                    speed=self.speed, period_s=(1.0 if self.profile == 'dance' else period) /
                    (self.effective_speed if self.active else self.speed),
                    effective_speed=self.effective_speed,
                    ramp_gait_phase=self.crest_sequence.phase if self.crest_sequence else None,
                    crest_wave_enabled=self.crest_wave,
                    assisted_crest_start=self.start_at_crest,
                    exit_clearance_cycles=self.exit_cycles,
                    camera_controlled=self.active and self.camera_start_angle(self.profile) is not None,
                    navigation=self.navigation.report() if self.navigation else None)

    def camera_start_angle(self, profile):
        if profile == 'corridor_search':
            return self.camera_settings['corridor_servo_deg']
        if profile in ('course', 'course_camera', 'course_tape'):
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
        if self.active and self.profile in SIDESTEP_PROFILES:
            self.effective_speed = min(self.speed, .65)
        elif not self.active or not self.speed_governor:
            self.effective_speed = self.speed

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
        if profile in SIDESTEP_PROFILES:
            self.params.update(SIDESTEP_GAIT)
        elif profile in RAMP_PROFILES:
            self.params.update(RAMP_STEERING)
        self.base_params = dict(self.params)
        if self.start_at_crest and profile == 'course':
            self.params.update(CREST_WAVE_GAIT if self.crest_wave else CREST_GAIT)
        elif self.crest_assist and profile == 'course':
            # Trials 41/42 drifted on the first incline with gain 1.2. Test
            # the earlier gain before yaw builds, retaining the .2 turn cap.
            # Keep base_params unchanged for the proven crest/descent setup.
            self.params['heading_gain'] = 3.0
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
        # Guidance runs on the PC; these modes share the ESP32's existing IMU gait.
        controller_profile = 'course' if profile in ('course_tape', 'corridor') else profile
        result = {'profile': controller_profile, **{name: settings[name] for name in names}}
        if 'turn_limit' in params:
            result['turn_limit'] = params['turn_limit']
        if 'rear_stride_scale' in params:
            result['rear_stride_scale'] = params['rear_stride_scale']
        if profile in SIDESTEP_PROFILES:
            result.update(profile='sidestep', sideways=SIDESTEP_PROFILES[profile])
        return result

    def begin(self, status, session, now, profile='walk'):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        if profile not in PROFILES:
            raise ValueError('Unknown walking profile.')
        self.profile = profile
        self.crest_sequence = CrestSequence() if self.crest_assist and profile == 'course' else None
        if self.start_at_crest and self.crest_sequence:
            self.crest_sequence.requested = 'crest'
            self.crest_sequence.switched(now)
        self.resume_until = None
        self.exit_cycles = self.exit_started = self.exit_updated = None
        self.speed_governor = RampSpeedGovernor() if self.adaptive_ramp_speed and profile == 'course' else None
        self.effective_speed = self.speed
        if profile in SIDESTEP_PROFILES:
            self.effective_speed = min(self.speed, .65)
        sensors = PROFILES[profile]
        if profile in ('corridor', 'corridor_search'):
            self.corridor_settings = self.corridor_settings or load_corridor_settings()
            self.navigation = (CorridorSearch(now, self.corridor_settings['navigation'])
                               if profile == 'corridor_search' else
                               IRWallNavigation(now, **self.corridor_settings['navigation']))
        else:
            self.navigation = (CourseNavigation(now, use_camera='camera' in sensors, use_imu='imu' in sensors,
                                                guidance='blue_tape' if profile == 'course_tape' else 'ramp',
                                                scan_ramp=profile in ('course', 'course_camera'))
                               if sensors else None)
        self.session, self.sequence = session, 0
        self.started = self.last_confirmed_frame = now
        self.active = True
        self.message = ('Settling for 2 seconds; keep the robot still.' if sensors or profile == 'dance' else
                        'Walking on the ESP32.')

    def transition_settings(self):
        phase = self.crest_sequence.requested if self.crest_sequence else None
        if phase not in ('crest', 'descent') or self.base_params is None:
            raise DeviceError('No ramp gait change is pending.', 409)
        crest_params = CREST_WAVE_GAIT if self.crest_wave else CREST_GAIT
        params = self.base_params | (crest_params if phase == 'crest' else {})
        return params, self.controller_settings(self.profile, params)

    def resume_gait(self, session, now, params):
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        self.session, self.sequence = session, 0
        self.params = params
        self.last_confirmed_frame = now
        self.resume_until = now + 2
        self.crest_sequence.switched(now)
        # Keep the established heading, offset and route state across the MCU
        # orientation reset. Discard camera freshness and slope debounce times.
        self.navigation.camera_angle = None
        self.navigation.estimate.last_seen = None
        self.navigation.downhill_since = self.navigation.flat_since = None
        self.navigation.last_slope_sample = None
        self.message = 'Settling after ramp gait change; preserving navigation heading.'

    def ramp_speed(self, now, ready, *, exiting=False):
        speed = self.speed
        # Keep the first approach steady. Trial41 repeatedly sped up briefly
        # on the incline before drifting; earlier full traversals used50%.
        # The pitch-triggered crest phase can still use the requested speed.
        if self.crest_sequence and self.crest_sequence.phase == 'approach':
            speed = min(speed, .5)
        if self.profile in SIDESTEP_PROFILES:
            speed = min(speed, .65)
        if self.speed_governor:
            estimate = self.navigation.estimate
            speed = self.speed_governor.update(now, speed, estimate.roll, estimate.heading,
                        0 if exiting else estimate.lateral, ready=ready)
        self.effective_speed = speed
        return speed

    def clear_exit_frame(self, status, now):
        """Advance just beyond visual ramp loss; require fresh, nearly level IMU."""
        attitude = status.get('orientation')
        if (not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50 or
                not isinstance(attitude, (list, tuple)) or len(attitude) != 3 or
                not all(math.isfinite(a) for a in attitude)):
            raise DeviceError('Ramp exit clearance lost fresh orientation.', 409)
        roll, pitch, heading = attitude
        nav = self.navigation
        nav.estimate.heading += heading - nav.imu_heading
        nav.imu_heading = heading
        nav.estimate.roll, nav.estimate.pitch = roll, pitch
        nav.estimate.last_imu = now - nav.started - status['imuAgeMs'] / 1000
        nav.updated_at = now
        dt = now - self.exit_updated
        if (not 0 <= dt <= .35 or now - self.exit_started > 12 or
                abs(roll) > math.radians(10) or abs(pitch) > math.radians(10) or
                abs(nav.estimate.heading) > math.radians(15)):
            raise DeviceError('Ramp exit clearance exceeded its timing or attitude limit.', 409)
        self.exit_updated = now
        period = self.params['period'] / RAMP_SPEED_SCALE
        previous_speed = self.effective_speed if self.speed_governor else self.speed
        self.exit_cycles = max(0, self.exit_cycles - dt * previous_speed / period)
        # Stop with all feet commanded near the support stance, not mid-swing.
        pitches = [j.get('angle') for j in status['joints'] if j['name'].endswith('_pitch')]
        down = len(pitches) == 4 and all(isinstance(a, (int, float)) and a >= 30 for a in pitches)
        nav.corridor_requested = self.exit_cycles <= 1e-9 and down
        self.message = ('Ramp exit clearance finished.' if nav.corridor_requested else
                        'Clearing ramp exit with IMU heading; waiting for rear feet to pass.')
        nav.reason = self.message
        self.sequence += 1
        return dict(session=self.session, sequence=self.sequence, speed=self.ramp_speed(now, True, exiting=True),
                    ready=int(not nav.corridor_requested), heading=nav.estimate.heading,
                    heading_reference=heading, lateral=0)

    def finish_sidestep(self, status):
        """Confirm this step's MCU completion, including the already-idle state."""
        if (self.active and self.profile in SIDESTEP_PROFILES and
                status.get('sidestepGait') is True and status.get('sidestepComplete') is True and
                not status.get('running') and not status.get('manualActive') and
                (status.get('motionSession') == self.session or
                 (not status.get('externalActive') and status.get('motionSession') == 0))):
            self.stop('Side step complete. Outputs off requested.')
            return True
        return False

    def frame(self, status, now, camera_frame=None):
        if not self.active:
            raise DeviceError('Walking is stopped.', 409)
        self.controller_check(status)
        if now - self.last_confirmed_frame > 1.0:
            raise DeviceError('PC/network control overrun; restart walking explicitly.', 409)
        if self.profile in SIDESTEP_PROFILES:
            if self.finish_sidestep(status):
                return {}
            check_sidestep(status)
            if now - self.started > 12:
                raise DeviceError('Side-step trial timed out; outputs off requested.', 409)
        if not status.get('externalActive') or status.get('motionSession') != self.session:
            raise DeviceError('ESP32 motion session ended; restart explicitly.', 409)
        if status.get('gaitStopping'):
            raise DeviceError('ESP32 is stopping after losing navigation; restart explicitly.', 409)
        if self.profile == 'course' and self.exit_clearance_enabled and status.get('orientationValid'):
            roll, pitch, _ = status['orientation']
            if abs(roll) > math.radians(12) or abs(pitch) > math.radians(35):
                raise DeviceError(f'Ramp tilt limit: roll {math.degrees(roll):.1f}, pitch {math.degrees(pitch):.1f} degrees; outputs off requested.', 409)
        if self.exit_cycles is not None:
            return self.clear_exit_frame(status, now)
        if self.profile in ('corridor', 'corridor_search'):
            self.navigation.set_speed(self.speed)
        if self.resume_until is not None:
            if now < self.resume_until:
                self.sequence += 1
                return dict(session=self.session, sequence=self.sequence, speed=self.ramp_speed(now, False),
                            ready=0, heading=0, heading_reference=0, lateral=0)
            attitude = status.get('orientation')
            if (not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50 or
                    not isinstance(attitude, (list, tuple)) or len(attitude) != 3 or
                    not all(math.isfinite(a) for a in attitude)):
                raise DeviceError('Fresh orientation required after ramp gait change.', 409)
            self.navigation.imu_heading = attitude[2]
            self.resume_until = None
        ready = self.navigation.update(status, now, camera_frame) if self.navigation else True
        if (self.profile in RAMP_PROFILES and ready and
                abs(self.navigation.estimate.heading) > math.radians(12)):
            raise DeviceError('Ramp heading exceeded 12 degrees; stop and recenter before restarting.', 409)
        if self.profile in SIDESTEP_PROFILES and abs(self.navigation.estimate.heading) > math.radians(10):
            raise DeviceError('Side step exceeded heading limit; outputs off requested.', 409)
        if (self.exit_clearance_enabled and self.profile == 'course' and self.navigation.corridor_requested):
            # Trial 23 left one rear foot on the ramp. Trial 24 cleared it with
            # roughly 1.125 gait cycles at 50%; count cycles so other speeds scale.
            self.exit_cycles = 1.125
            self.exit_started = self.exit_updated = now
            return self.clear_exit_frame(status, now)
        if self.crest_sequence:
            self.crest_sequence.observe(now, self.navigation.estimate.pitch, ready)
            if self.crest_sequence.requested:
                ready = False
        if (PROFILES[self.profile] or self.profile == 'dance') and now - self.started < 2:
            self.message = 'Settling for 2 seconds; keep the robot still.'
        elif self.profile == 'dance' and status.get('danceTurnDegrees') is not None:
            self.message = f"Turning while dancing: {abs(status['danceTurnDegrees']):.0f}° / 360°."
        elif self.navigation:
            self.message = self.navigation.reason
            if self.profile in SIDESTEP_PROFILES:
                direction = 'left' if SIDESTEP_PROFILES[self.profile] > 0 else 'right'
                self.message = f'Side step {direction}: holding heading; stops after one cycle.'
        else:
            self.message = 'Dancing on the ESP32.' if self.profile == 'dance' else 'Walking on the ESP32.'
        self.sequence += 1
        estimate = self.navigation.estimate if self.navigation else None
        frame = {'session': self.session, 'sequence': self.sequence,
                 'speed': self.ramp_speed(now, ready), 'ready': int(ready),
                 'heading': estimate.heading if estimate else 0,
                 'heading_reference': self.navigation.imu_heading if self.navigation else 0,
                 'lateral': estimate.lateral if estimate else 0}
        if self.profile == 'corridor_search':
            frame.update(speed=self.navigation.speed, forward=self.navigation.forward)
            if self.navigation.finished:
                self.stop(self.navigation.reason + ' Outputs off requested.')
        if self.profile == 'corridor' and self.navigation.exit_complete:
            self.stop('Corridor exit movement finished. Outputs off requested.')
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.message = message
