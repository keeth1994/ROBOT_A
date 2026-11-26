import math
from pathlib import Path
import tomllib

from .devices import DeviceError
from .navigation import CourseNavigation
from .corridor_walls import IRWallNavigation, load_corridor_settings
from .corridor_search import CorridorSearch
from .crest import CrestSequence, CREST_WAVE_GAIT, RampSpeedGovernor
from .ramp_alignment import ALIGNMENT_GAIT, RampAlignment
from .ramp_recovery import RampHeadingRecovery
from robot_b91.camera_calibration import load_camera_scan_settings
from robot_b91.gait import RAMP_SPEED_SCALE, RAMP_STEERING, ramp_forward_scale

ROOT = Path(__file__).resolve().parents[2]
JOINTS = tuple(f'{leg}_{axis}' for leg in ('FL', 'RL', 'FR', 'RR') for axis in ('yaw', 'pitch'))
PROFILES = {'walk': (), 'course': ('camera', 'imu'), 'course_camera': ('camera',),
            'course_tape': ('camera', 'imu'), 'course_imu': ('imu',), 'crest_test': ('imu',),
            'corridor': ('ir', 'imu'), 'corridor_search': ('camera', 'ir', 'imu'), 'dance': ()}
PROFILES['ramp_align'] = ('camera', 'imu')
WALK_ACTIONS = tuple(PROFILES)
RAMP_PROFILES = ('course', 'course_camera')


class Walking:
    def __init__(self, *, exit_clearance=False, adaptive_ramp_speed=False,
                 crest_wave=True, start_at_crest=False, ramp_drift_control=False):
        if start_at_crest and not crest_wave:
            raise ValueError('Assisted crest start requires the crest wave gait.')
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
        self.exit_clearance_enabled = exit_clearance or self.crest_wave
        self.crest_sequence = None
        self.resume_until = None
        self.base_params = None
        self.exit_cycles = self.exit_started = self.exit_updated = None
        self.adaptive_ramp_speed = adaptive_ramp_speed or crest_wave
        self.speed_governor = None
        self.effective_speed = self.speed
        self.alignment_then_course = False
        self.ramp_drift_control = ramp_drift_control
        self.forward_scale = 1.
        self.heading_recovery = RampHeadingRecovery()

    def state(self):
        period = self.params['period']
        if self.profile in RAMP_PROFILES or self.profile == 'crest_test':
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
                    ramp_drift_control=self.ramp_drift_control,
                    forward_scale=self.forward_scale,
                    ramp_heading_recovery=self.heading_recovery.active,
                    exit_clearance_cycles=self.exit_cycles,
                    camera_controlled=self.active and self.camera_start_angle(self.profile) is not None,
                    navigation=self.navigation.report() if self.navigation else None)

    def camera_start_angle(self, profile):
        if profile == 'corridor_search':
            return self.camera_settings['corridor_servo_deg']
        if profile in ('course', 'course_camera', 'course_tape', 'ramp_align'):
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
        if self.active and self.profile == 'ramp_align':
            self.effective_speed = min(self.speed, .5)
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
        if profile == 'ramp_align':
            self.params.update(ALIGNMENT_GAIT)
        elif profile in RAMP_PROFILES or profile == 'crest_test':
            self.params.update(RAMP_STEERING)
        if profile == 'course' and self.ramp_drift_control:
            self.params['strafe_gain'] = 12.
        self.base_params = dict(self.params)
        if profile == 'crest_test' or (self.start_at_crest and profile == 'course'):
            self.params.update(CREST_WAVE_GAIT)
        return self.controller_settings(profile, self.params)

    def controller_settings(self, profile, params):
        defaults = {'pitch_min': -125 * math.pi / 180, 'lateral_gain': 0, 'strafe_gain': 0, 'yaw_bias': 0}
        names = ('period', 'duty', 'yaw_amplitude', 'pitch_lift', 'pitch_stance',
                 'pitch_min', 'heading_gain', 'lateral_gain', 'strafe_gain', 'pattern', 'yaw_bias')
        settings = defaults | params
        if profile in RAMP_PROFILES or profile == 'crest_test':
            settings['period'] /= RAMP_SPEED_SCALE
        if profile in ('corridor', 'corridor_search'):
            self.corridor_settings = load_corridor_settings()
            settings.update(self.corridor_settings['gait'])
        if profile == 'dance':
            settings['pitch_stance'] = 0.0
        # Guidance runs on the PC; these modes share the ESP32's existing IMU gait.
        controller_profile = 'course' if profile in ('course_tape', 'corridor') else profile
        if profile == 'crest_test':
            controller_profile = 'course_imu'
        elif profile == 'ramp_align':
            # Existing firmware supports independent strafe/yaw with forward=0.
            controller_profile = 'corridor_search'
        elif profile == 'course' and self.ramp_drift_control:
            # The installed search protocol also supports continuous forward
            # scaling. Perception and ramp gait phases remain on the PC.
            controller_profile = 'corridor_search'
        result = {'profile': controller_profile, **{name: settings[name] for name in names}}
        if 'turn_limit' in params:
            result['turn_limit'] = params['turn_limit']
        if 'rear_stride_scale' in params:
            result['rear_stride_scale'] = params['rear_stride_scale']
        return result

    def begin(self, status, session, now, profile='walk'):
        self.prepare(status)
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        if profile not in PROFILES:
            raise ValueError('Unknown walking profile.')
        self.profile = profile
        self.forward_scale = 1.
        self.heading_recovery = RampHeadingRecovery()
        self.alignment_then_course = False
        self.crest_sequence = CrestSequence() if self.crest_wave and profile == 'course' else None
        if self.start_at_crest and self.crest_sequence:
            self.crest_sequence.requested = 'crest'
            self.crest_sequence.switched(now)
        self.resume_until = None
        self.exit_cycles = self.exit_started = self.exit_updated = None
        self.speed_governor = RampSpeedGovernor() if self.adaptive_ramp_speed and profile == 'course' else None
        self.effective_speed = self.speed
        if profile == 'ramp_align':
            self.effective_speed = min(self.speed, .5)
        sensors = PROFILES[profile]
        if profile == 'ramp_align':
            self.navigation = RampAlignment(now)
        elif profile in ('corridor', 'corridor_search'):
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

    def alignment_transition_settings(self):
        mode = self.navigation.requested_gait if self.profile == 'ramp_align' else None
        if mode not in ('hold', 'correct') or self.base_params is None:
            raise DeviceError('No alignment stance change is pending.', 409)
        params = dict(self.base_params)
        if mode == 'hold':
            # A zero-lift, zero-yaw session stays planted at every gait phase.
            # A delayed ready=0 can no longer freeze a newly lifted foot.
            params.update(pitch_lift=0., yaw_amplitude=0., yaw_bias=0.)
        return mode, params, self.controller_settings('ramp_align', params)

    def resume_alignment(self, session, now, params, mode):
        if not isinstance(session, int) or isinstance(session, bool) or not 0 < session <= 0xffffffff:
            raise DeviceError('Invalid ESP32 motion session.', 409)
        self.session, self.sequence = session, 0
        self.params = params
        self.last_confirmed_frame = now
        self.navigation.gait_changed(mode, now)

    def transition_settings(self):
        phase = self.crest_sequence.requested if self.crest_sequence else None
        if phase not in ('crest', 'descent') or self.base_params is None:
            raise DeviceError('No ramp gait change is pending.', 409)
        crest_params = CREST_WAVE_GAIT
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
        if self.speed_governor:
            self.speed_governor.require_stability()
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
        if self.profile == 'ramp_align':
            speed = min(speed, .5)
        if self.speed_governor:
            estimate = self.navigation.estimate
            period = self.params['period'] / RAMP_SPEED_SCALE
            cycle_s = period / min(speed, .5) if self.speed_governor.slow else period / speed
            speed = self.speed_governor.update(now, speed, estimate.roll, estimate.heading,
                        0 if exiting else estimate.lateral, ready=ready, stable_cycle_s=cycle_s)
        self.effective_speed = speed
        return speed

    def clear_exit_frame(self, status, now):
        """Advance just beyond visual ramp loss using fresh IMU readings."""
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
        if not 0 <= dt <= .35:
            raise DeviceError('Ramp exit clearance lost timely control updates.', 409)
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
        self.forward_scale = 1.
        return dict(session=self.session, sequence=self.sequence, speed=self.ramp_speed(now, True, exiting=True),
                    ready=int(not nav.corridor_requested), heading=nav.estimate.heading,
                    heading_reference=heading, lateral=0, forward=1.)

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
        if self.exit_cycles is not None:
            return self.clear_exit_frame(status, now)
        if self.profile in ('corridor', 'corridor_search'):
            self.navigation.set_speed(self.speed)
        if self.resume_until is not None:
            if now < self.resume_until:
                self.sequence += 1
                return dict(session=self.session, sequence=self.sequence, speed=self.ramp_speed(now, False),
                            ready=0, heading=0, heading_reference=0, lateral=0, forward=0.)
            attitude = status.get('orientation')
            if (not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50 or
                    not isinstance(attitude, (list, tuple)) or len(attitude) != 3 or
                    not all(math.isfinite(a) for a in attitude)):
                raise DeviceError('Fresh orientation required after ramp gait change.', 409)
            self.navigation.imu_heading = attitude[2]
            self.resume_until = None
        ready = self.navigation.update(status, now, camera_frame) if self.navigation else True
        recovering = False
        if self.profile == 'course' and self.ramp_drift_control:
            estimate = self.navigation.estimate
            debug = estimate.debug or {}
            fresh_edges = (debug.get('accepted') and not debug.get('held') and
                           debug.get('lateral_source') == 'paired_edges' and
                           0 <= now-self.navigation.started-debug.get('timestamp', -math.inf) <= .35)
            # A sudden edge-fit excursion holds correction for confirmation.
            # Sustained displacement still reaches the filtered offset guard.
            raw_lateral = debug.get('measured_lateral_m', estimate.lateral)
            fresh_edges = fresh_edges and math.isfinite(raw_lateral) and abs(raw_lateral) < .05
            ready = self.heading_recovery.update(now, estimate.heading, estimate.lateral,
                estimate.roll, estimate.pitch, ready=ready, fresh_edges=fresh_edges)
            recovering = self.heading_recovery.active
            if recovering:
                self.navigation.reason = self.heading_recovery.message
                if self.speed_governor:
                    self.speed_governor.require_stability()
        if (self.exit_clearance_enabled and self.profile == 'course' and
                self.navigation.corridor_requested and not recovering):
            # Trial 23 left one rear foot on the ramp. Trial 24 cleared it with
            # roughly 1.125 gait cycles at 50%; count cycles so other speeds scale.
            self.exit_cycles = 1.125
            self.exit_started = self.exit_updated = now
            return self.clear_exit_frame(status, now)
        if self.crest_sequence:
            self.crest_sequence.observe(now, self.navigation.estimate.pitch, ready and not recovering)
            if self.crest_sequence.requested:
                ready = False
        if (PROFILES[self.profile] or self.profile == 'dance') and now - self.started < 2:
            self.message = 'Settling for 2 seconds; keep the robot still.'
        elif self.profile == 'dance' and status.get('danceTurnDegrees') is not None:
            self.message = f"Turning while dancing: {abs(status['danceTurnDegrees']):.0f}° / 360°."
        elif self.profile == 'crest_test':
            self.message = ('Testing crest gait with IMU heading. Press Stop to finish.' if ready else
                            self.navigation.reason)
        elif self.navigation:
            self.message = self.navigation.reason
        else:
            self.message = 'Dancing on the ESP32.' if self.profile == 'dance' else 'Walking on the ESP32.'
        self.sequence += 1
        estimate = self.navigation.estimate if self.navigation else None
        frame = {'session': self.session, 'sequence': self.sequence,
                 'speed': self.ramp_speed(now, ready), 'ready': int(ready),
                 'heading': estimate.heading if estimate else 0,
                 'heading_reference': self.navigation.imu_heading if self.navigation else 0,
                 'lateral': estimate.lateral if estimate else 0}
        if self.profile == 'course' and self.ramp_drift_control:
            self.forward_scale = ramp_forward_scale(estimate.lateral)
            if recovering:
                self.forward_scale = min(self.forward_scale, .35)
                frame['speed'] = min(frame['speed'], .5)
                self.effective_speed = frame['speed']
            frame['forward'] = self.forward_scale
        if self.profile == 'corridor_search':
            frame.update(speed=self.navigation.speed, forward=self.navigation.forward)
            if self.navigation.finished:
                self.stop(self.navigation.reason + ' Outputs off requested.')
        if self.profile == 'ramp_align':
            frame.update(forward=0., heading=self.navigation.command_heading,
                         lateral=self.navigation.command_lateral)
        if self.profile == 'corridor' and self.navigation.exit_complete:
            self.stop('Corridor exit movement finished. Outputs off requested.')
        return frame

    def stop(self, message='Stopped. Outputs off requested.'):
        self.active = False
        self.heading_recovery = RampHeadingRecovery()
        self.alignment_then_course = False
        self.message = message
