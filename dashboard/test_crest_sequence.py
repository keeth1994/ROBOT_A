"""Crest detection and gait transitions without hardware commands."""
import math
import time
import unittest
from unittest.mock import patch

from dashboard.robot_dashboard.crest import CrestSequence
from dashboard.robot_dashboard.devices import DeviceError, DeviceConnectionError
from dashboard.robot_dashboard.runtime import Dashboard
from dashboard.robot_dashboard.walking import Walking, JOINTS


def status(session=1, heading=0):
    return dict(servoReady=True, faulted=False, motionProtocol=2,
                externalActive=True, motionSession=session,
                signedYawBias=True, crestWaveGait=True, rearStrideScale=True, frontPitchLift=True,
                orientationValid=True, orientation=[0., 0., heading], imuAgeMs=10,
                joints=[dict(name=name, channel=i) for i, name in enumerate(JOINTS)])


class CrestSequenceTests(unittest.TestCase):
    def walking(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(), 1, 0, 'course')
        return walking

    def test_transition_restores_original_gait_and_preserves_offset(self):
        walking = self.walking()
        baseline = walking.start_settings('course')
        nav = walking.navigation
        nav.estimate.heading = -.12
        nav.estimate.lateral = -nav.estimate.ramp_offset_right_m
        for phase in ('crest', 'descent', 'plateau'):
            nav.estimate.heading = -.12
            walking.crest_sequence.requested = phase
            params, settings = walking.transition_settings()
            if phase == 'crest':
                self.assertEqual(settings['pattern'], 'crest')
                self.assertEqual(settings['rear_stride_scale'], .3)
                self.assertEqual(settings['yaw_bias'], -.25)
                self.assertAlmostEqual(settings['front_pitch_lift'], math.radians(50))
            else:
                self.assertEqual(settings, baseline)
            walking.resume_gait(2, 10, params)
            self.assertIs(nav, walking.navigation)
            self.assertAlmostEqual(nav.estimate.lateral_error, 0)
            self.assertEqual(walking.crest_sequence.phase, phase)
            frame = walking.frame(status(2), 10.1)
            self.assertEqual(frame['ready'], 0)
            walking.last_confirmed_frame = 12.05
            walking.frame(status(2, .02), 12.1)
            self.assertAlmostEqual(nav.estimate.heading, -.12)
            walking.last_confirmed_frame = 12.15
            walking.frame(status(2, .04), 12.2)
            self.assertAlmostEqual(nav.estimate.heading, -.10)

    def test_only_imu_and_camera_ramp_mode_enables_crest_detection(self):
        for profile in ('walk', 'course_camera', 'course_imu', 'course_tape'):
            walking = Walking()
            walking.start_settings(profile)
            walking.begin(status(), 1, 0, profile)
            self.assertIsNone(walking.crest_sequence)

    def test_large_gaps_do_not_confirm_a_climb(self):
        sequence = CrestSequence()
        for i in range(15):
            sequence.observe(i, math.radians(-22), True)
        self.assertFalse(sequence.climbed)

    def test_walking_requests_crest_and_holds_before_switching(self):
        walking = self.walking()
        with patch.object(walking.navigation, 'update', return_value=True):
            for i in range(30):
                now = 2 + i * .1
                walking.navigation.estimate.pitch = math.radians(-22 if i < 20 else -5)
                walking.last_confirmed_frame = now
                frame = walking.frame(status(), now)
            self.assertEqual(walking.crest_sequence.requested, 'crest')
            self.assertEqual(frame['ready'], 0)

    def test_old_firmware_is_rejected_before_start(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        old = status() | dict(externalActive=False, crestWaveGait=False)
        with patch('dashboard.robot_dashboard.runtime.devices.request', return_value=old) as request:
            with self.assertRaisesRegex(DeviceError, 'updated controller firmware'):
                app.walk_command('course')
        self.assertNotIn('/motion/start', [call.args[1] for call in request.call_args_list])

    def test_switch_stops_confirms_idle_then_restarts(self):
        app = Dashboard()
        app.walking = self.walking()
        app.walking.crest_sequence.requested = 'crest'
        idle = status() | dict(externalActive=False)
        with patch('dashboard.robot_dashboard.runtime.devices.request',
                   side_effect=[{}, idle, dict(session=2)]) as request, \
                patch.object(app, '_aim_camera'):
            app._change_ramp_gait('http://robot.test')
        self.assertEqual([call.args[1] for call in request.call_args_list],
                         ['/stop', '/status', '/motion/start'])
        self.assertEqual(app.walking.session, 2)
        self.assertEqual(app.walking.crest_sequence.phase, 'crest')

    def test_failed_or_unconfirmed_stop_cannot_restart(self):
        for responses in ([DeviceConnectionError('offline'), {}], [{}, status(), {}],
                          [{}, status() | dict(externalActive=False, crestWaveGait=False), {}]):
            app = Dashboard()
            app.walking = self.walking()
            app.walking.crest_sequence.requested = 'crest'
            with patch('dashboard.robot_dashboard.runtime.devices.request',
                       side_effect=responses) as request:
                with self.assertRaises(DeviceError):
                    app._change_ramp_gait('http://robot.test')
            self.assertFalse(app.walking.active)
            self.assertNotIn('/motion/start', [call.args[1] for call in request.call_args_list])

    def feed(self, seq, start, degrees, count, ready=True):
        for i in range(count):
            seq.observe(start + i * .1, math.radians(degrees), ready)


    def test_climb_then_crest_then_sustained_descent(self):
        seq = CrestSequence()
        self.feed(seq, 0, 20, 20)  # A downhill assisted start must not arm it.
        self.assertIsNone(seq.requested)
        self.feed(seq, 2, -22, 20)
        self.assertTrue(seq.climbed)
        self.feed(seq, 4, -5, 3)  # Short pitch spike is not a crest.
        self.feed(seq, 4.3, -18, 3)
        self.assertIsNone(seq.requested)
        self.feed(seq, 4.6, -5, 8)
        self.assertEqual(seq.requested, 'crest')
        seq.switched(6)
        self.feed(seq, 6, 20, 20)
        self.assertIsNone(seq.requested)  # Must perform at least one gait cycle.
        self.feed(seq, 8, 20, 31)
        self.assertEqual(seq.requested, 'descent')
        seq.switched(12)
        self.feed(seq, 12, -22, 20)
        self.assertEqual(seq.phase, 'descent')  # Only first crest is assisted.


    def test_crest_detection_uses_slope_sequence_at_different_times(self):
        for slope in (20, 25, 30):
            for delay in (0, 25, 90):
                with self.subTest(slope=slope, delay=delay):
                    seq = CrestSequence()
                    self.feed(seq, delay, 0, 10)
                    self.assertFalse(seq.climbed)
                    self.feed(seq, delay+1, -slope, 20)
                    self.assertTrue(seq.climbed)
                    self.assertIsNone(seq.requested)
                    self.feed(seq, delay+3, -5, 8)
                    self.assertEqual(seq.requested, 'crest')


    def test_missing_guidance_holds_crest_without_timeout(self):
        seq = CrestSequence()
        self.feed(seq, 0, -22, 20)
        self.feed(seq, 2, -5, 20, ready=False)
        self.assertIsNone(seq.requested)
        self.feed(seq, 4, -5, 8)
        seq.switched(5)
        seq.observe(46, .3, False)
        self.assertEqual(seq.phase, 'crest')
        self.assertIsNone(seq.requested)

    def enter_crest(self):
        seq = CrestSequence()
        self.feed(seq, 0, -22, 20)
        self.feed(seq, 2, -5, 10)
        self.assertEqual(seq.requested, 'crest')
        seq.switched(3)
        return seq

    def test_sustained_level_pitch_returns_to_normal_gait(self):
        seq = self.enter_crest()
        self.feed(seq, 3, 0, 81)
        self.assertIsNone(seq.requested)
        self.feed(seq, 11.1, 0, 14)
        self.assertEqual(seq.requested, 'plateau')
        seq.switched(12.5)
        self.feed(seq, 13, -22, 20)
        self.assertEqual(seq.phase, 'plateau')
        self.assertIsNone(seq.requested)

    def test_short_level_pitch_does_not_end_crest_gait(self):
        seq = self.enter_crest()
        self.feed(seq, 3, 0, 70)
        self.feed(seq, 10, 7, 10)
        self.assertIsNone(seq.requested)
        self.assertIsNone(seq.exit_candidate)

    def test_pauses_and_sample_gaps_reset_plateau_confirmation(self):
        for missing in ('paused', 'gap'):
            with self.subTest(missing=missing):
                seq = self.enter_crest()
                self.feed(seq, 3, 0, 81)
                if missing == 'paused':
                    seq.observe(11.1, 0, False)
                self.feed(seq, 13, 0, 40)
                self.assertIsNone(seq.requested)
                self.feed(seq, 17, 0, 20)
                self.assertEqual(seq.requested, 'plateau')

    def test_switching_from_plateau_to_descent_restarts_confirmation(self):
        seq = self.enter_crest()
        self.feed(seq, 3, 0, 81)
        self.feed(seq, 11.1, 20, 4)
        self.assertIsNone(seq.requested)
        self.feed(seq, 11.5, 20, 6)
        self.assertEqual(seq.requested, 'descent')


