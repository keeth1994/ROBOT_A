"""Checks for physical ramp gait changes, without connecting to hardware."""
import math
import time
import unittest
from unittest.mock import patch

from robot_dashboard.crest import CrestSequence, RampSpeedGovernor
from robot_dashboard.devices import DeviceError, DeviceConnectionError
from robot_dashboard.runtime import Dashboard
from robot_dashboard.walking import JOINTS, Walking
from robot_b91.camera_scan import RampScanNavigation
from robot_b91.camera_calibration import load_camera_scan_settings


def status(session=1, yaw=0):
    return dict(servoReady=True, faulted=False, motionProtocol=2, signedYawBias=True, boundedSteering=True,
                externalActive=True, motionSession=session, gaitStopping=False,
                orientationValid=True, imuAgeMs=3, orientation=[0., .35, yaw],
                joints=[dict(name=name, channel=i) for i, name in enumerate(JOINTS)],
                cameraPitch=dict(angle=-65, moving=False, minAngle=-80, maxAngle=45))


class CrestSequenceTests(unittest.TestCase):
    def test_ground_test_uses_same_crest_settings_and_period(self):
        ramp = Walking(start_at_crest=True)
        ramp_settings = ramp.start_settings('course')
        ground = Walking()
        ground_settings = ground.start_settings('crest_test')
        self.assertEqual(ground_settings, ramp_settings | {'profile': 'course_imu'})
        ground.begin(status(), 1, 0, 'crest_test')
        self.assertEqual(ground.state()['period_s'], ground_settings['period'])
        self.assertIsNone(ground.crest_sequence)
        self.assertIsNone(ground.speed_governor)
        self.assertIsNone(ground.camera_target(0))

    def test_ground_test_runs_without_camera_and_holds_on_stale_imu(self):
        walking = Walking()
        walking.start_settings('crest_test')
        walking.begin(status(), 1, 0, 'crest_test')
        walking.set_speed(.5)
        sample = status() | {'orientation': [0., 0., 0.]}
        sample.pop('cameraPitch')
        for now in (2.1, 3., 5.):
            walking.last_confirmed_frame = now
            frame = walking.frame(sample, now)
            self.assertEqual(frame['ready'], 1)
            self.assertEqual(frame['speed'], .5)
            self.assertEqual(frame['lateral'], 0)
            self.assertFalse(walking.navigation.corridor_requested)
        self.assertIn('Press Stop', walking.message)
        frame = walking.frame(sample | {'imuAgeMs': 200}, 5.1)
        self.assertEqual(frame['ready'], 0)

    def test_ground_test_starts_existing_firmware_profile_without_camera_requests(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        sample = status() | {'externalActive': False, 'crestWaveGait': True, 'rearStrideScale': True}
        sample.pop('cameraPitch')
        def request(url, path, *args, **kwargs):
            return sample if path == '/status' else {'session': 1}
        with patch('robot_dashboard.runtime.devices.request', side_effect=request) as requests:
            with patch.object(app.camera, 'enable') as enable_camera:
                app.walk_command('crest_test')
        self.assertEqual([call.args[1] for call in requests.call_args_list], ['/status', '/motion/start'])
        self.assertEqual(requests.call_args_list[1].args[2]['profile'], 'course_imu')
        enable_camera.assert_not_called()
        self.assertTrue(app.walking.active)

    def test_ground_test_checks_required_firmware_features(self):
        for feature in ('signedYawBias', 'crestWaveGait', 'boundedSteering', 'rearStrideScale'):
            app = Dashboard()
            app.browser_heartbeat = time.monotonic()
            sample = status() | {'externalActive': False, 'crestWaveGait': True, 'rearStrideScale': True,
                                 feature: False}
            with patch('robot_dashboard.runtime.devices.request', return_value=sample) as requests:
                with self.assertRaises(DeviceError):
                    app.walk_command('crest_test')
            self.assertNotIn('/motion/start', [call.args[1] for call in requests.call_args_list])
            self.assertFalse(app.walking.active)

    def test_assisted_crest_start_keeps_descent_gait_without_timeout(self):
        walking = Walking(crest_wave=True, start_at_crest=True)
        settings = walking.start_settings('course')
        self.assertEqual(settings['pattern'], 'crest')
        self.assertEqual(settings['turn_limit'], .2)
        self.assertEqual(walking.base_params['pattern'], 'trot')
        walking.begin(status(), 1, 10, 'course')
        self.assertTrue(walking.state()['assisted_crest_start'])
        self.assertEqual(walking.crest_sequence.phase, 'crest')
        walking.crest_sequence.observe(51, 0, False)
        self.assertEqual(walking.crest_sequence.phase, 'crest')
        walking.crest_sequence.requested = 'descent'
        params, settings = walking.transition_settings()
        self.assertEqual(params, walking.base_params)
        self.assertEqual(settings['pattern'], 'trot')

    def test_assisted_crest_start_requires_ramp_only(self):
        with self.assertRaises(ValueError):
            Dashboard(start_at_crest=True)
        app = Dashboard(start_at_crest=True, auto_corridor=False)
        self.assertEqual(app.walking.start_settings('walk')['pattern'], 'trot')
        self.assertEqual(app.walking.start_settings('course_camera')['pattern'], 'trot')

    def test_wave_uses_requested_crest_speed_with_adaptive_fallback(self):
        walking = Walking()
        self.assertTrue(walking.crest_wave)
        self.assertTrue(walking.exit_clearance_enabled)
        self.assertTrue(walking.adaptive_ramp_speed)
        walking.start_settings('course')
        walking.begin(status(), 1, 0, 'course')
        walking.set_speed(.65)
        for now in (0, .2, 1.4, 2.5, 10):
            self.assertEqual(walking.ramp_speed(now, True), .5)
        walking.set_speed(.4)
        self.assertEqual(walking.ramp_speed(10.1, True), .4)
        walking.set_speed(.65)
        walking.crest_sequence.requested = 'crest'
        params, settings = walking.transition_settings()
        self.assertEqual(settings['pattern'], 'crest')
        self.assertEqual(settings['duty'], .8)
        walking.resume_gait(2, 1, params)
        self.assertEqual(walking.ramp_speed(3, True), .5)
        walking.navigation.estimate.roll = math.radians(6)
        self.assertEqual(walking.ramp_speed(3.1, True), .5)
        walking.set_speed(.4)
        self.assertEqual(walking.ramp_speed(3.2, True), .4)
        walking.set_speed(.65)
        walking.navigation.estimate.roll = 0
        self.assertEqual(walking.ramp_speed(3.3, True), .5)
        for tick in range(34, 73):
            self.assertEqual(walking.ramp_speed(tick/10, True), .5)
        self.assertEqual(walking.ramp_speed(7.4, True), .65)
        walking.crest_sequence.requested = 'descent'
        params, settings = walking.transition_settings()
        self.assertEqual(params, walking.base_params)
        self.assertEqual(settings['pattern'], 'trot')
        walking.resume_gait(3, 10, params)
        walking.set_speed(.65)
        for tick in range(120, 160):
            self.assertEqual(walking.ramp_speed(tick/10, True), .5)
        self.assertEqual(walking.ramp_speed(16.1, True), .65)

    def test_wave_rejects_old_firmware_before_climbing_or_switching(self):
        for initial in (True, False):
            app = Dashboard()
            app.browser_heartbeat = time.monotonic()
            old = status() | {'externalActive': False}
            calls = []
            def request(url, path, *args, **kwargs):
                calls.append(path)
                return old if path == '/status' else {}
            with patch('robot_dashboard.runtime.devices.request', side_effect=request):
                with self.assertRaisesRegex(DeviceError, 'updated controller firmware'):
                    if initial:
                        app.walk_command('course')
                    else:
                        app.walking.start_settings('course')
                        app.walking.begin(old, 1, 0, 'course')
                        app.walking.crest_sequence.requested = 'crest'
                        app._change_ramp_gait('http://robot.test')
            self.assertNotIn('/motion/start', calls)
            self.assertFalse(app.walking.active)

    def test_speed_slows_before_edge_and_requires_sustained_centering(self):
        governor = RampSpeedGovernor()
        def speed(t, roll=0, heading=0, lateral=0, ready=True, requested=.6):
            return governor.update(t, requested, math.radians(roll), math.radians(heading),
                                   lateral, ready=ready)
        self.assertEqual(speed(0), .6)
        self.assertEqual(speed(.1, lateral=.025), .5)
        for tick in range(2, 12):
            self.assertEqual(speed(tick/10), .5)
        speed(1.2)
        self.assertEqual(speed(1.3), .6)
        self.assertEqual(speed(1.4, roll=5.1), .5)
        self.assertEqual(speed(3, ready=False), .5)
        for tick in range(40, 50):
            self.assertEqual(speed(tick/10), .5)
        speed(5)
        self.assertEqual(speed(5.1), .6)
        self.assertEqual(speed(6, heading=9), .5)
        self.assertEqual(speed(6.1, requested=.4), .4)

    def test_full_cycle_and_fresh_samples_required_after_a_disturbance(self):
        governor = RampSpeedGovernor()
        governor.require_stability()
        def stable(t):
            return governor.update(t, .65, 0, 0, 0, ready=True, stable_cycle_s=4)
        for tick in range(40):
            self.assertEqual(stable(tick/10), .5)
        self.assertEqual(stable(4), .65)
        governor.update(4.1, .65, math.radians(6), 0, 0, ready=True, stable_cycle_s=4)
        for tick in range(42, 57):
            self.assertEqual(stable(tick/10), .5)  # Quiet support between lifts is insufficient.
        self.assertEqual(stable(10), .5)  # A telemetry gap does not count as stability.
        for tick in range(101, 140):
            self.assertEqual(stable(tick/10), .5)
        self.assertEqual(stable(14), .65)

    def test_search_alternates_views_without_changing_heading(self):
        settings = load_camera_scan_settings()
        nav = RampScanNavigation(settings)
        nav.heading = -.07
        nav.heading_initialized = True
        nav.flip(3, math.radians(settings['forward_deg']))
        self.assertEqual(nav.view, 'backward')
        self.assertAlmostEqual(math.degrees(nav.view_target()), settings['backward_deg'])
        self.assertFalse(nav.ready(3))
        nav.flip(7, nav.view_target())
        self.assertEqual(nav.view, 'forward')
        self.assertAlmostEqual(math.degrees(nav.view_target()), settings['forward_deg'])
        self.assertFalse(nav.ready(7))
        self.assertAlmostEqual(nav.heading, -.07)
        self.assertTrue(nav.heading_initialized)

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

    def test_crest_and_speed_confirmation_at_real_wifi_cadence(self):
        for gap in (.4, .6, .85):
            seq = CrestSequence()
            for i in range(6):
                seq.observe(i*gap, math.radians(-22), True)
            self.assertTrue(seq.climbed)
            for i in range(6, 9):
                seq.observe(i*gap, math.radians(-5), True)
            self.assertEqual(seq.requested, 'crest')
            governor = RampSpeedGovernor()
            governor.require_stability()
            for i in range(math.ceil(4/gap)+1):
                speed = governor.update(i*gap, .7, 0, 0, 0, ready=True, stable_cycle_s=4.)
                if i*gap < 4:
                    self.assertEqual(speed, .5)
            self.assertEqual(speed, .7)

    def test_large_control_gaps_do_not_arm_crest_or_restore_speed(self):
        seq = CrestSequence()
        governor = RampSpeedGovernor()
        governor.require_stability()
        for i in range(15):
            seq.observe(i, math.radians(-22), True)
            self.assertEqual(governor.update(i, .7, 0, 0, 0, ready=True), .5)
        self.assertFalse(seq.climbed)

    def test_slope_exit_confirmation_accepts_slow_fresh_imu_samples(self):
        from robot_dashboard.navigation import CourseNavigation
        nav = CourseNavigation(0, scan_ramp=True)
        for t in (2., 2.4, 2.8):
            nav.observe_slope(t, math.radians(15))
        self.assertTrue(nav.downhill_seen)
        for t in (3.2, 3.6):
            nav.observe_slope(t, 0.)
        self.assertTrue(nav.checking_exit)

    def test_restart_preserves_estimate_across_imu_zero_reset(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(), 1, 0, 'course')
        nav = walking.navigation
        nav.estimate.heading = -.12
        nav.estimate.heading_initialized = True
        nav.estimate.lateral = .027
        nav.imu_heading = 1.3
        walking.crest_sequence.requested = 'crest'
        params, settings = walking.transition_settings()
        self.assertEqual(settings['yaw_bias'], -.25)
        walking.resume_gait(2, 10, params)
        frame = walking.frame(status(2, .02), 10.1)
        self.assertEqual(frame['ready'], 0)
        self.assertAlmostEqual(nav.estimate.heading, -.12)
        walking.last_confirmed_frame = 12.05
        walking.frame(status(2, .025), 12.1)
        self.assertAlmostEqual(nav.estimate.heading, -.12)
        walking.last_confirmed_frame = 12.15
        walking.frame(status(2, .045), 12.2)
        self.assertAlmostEqual(nav.estimate.heading, -.10)
        self.assertAlmostEqual(nav.estimate.lateral, .027)
        self.assertTrue(nav.estimate.heading_initialized)

    def test_failed_stop_or_unconfirmed_idle_cannot_start_next_gait(self):
        for failed_stop in (True, False):
            app = Dashboard()
            app.walking.start_settings('course')
            app.walking.begin(status(), 1, 0, 'course')
            app.walking.crest_sequence.requested = 'crest'
            calls = []
            def request(url, path, *args, **kwargs):
                calls.append(path)
                if path == '/stop' and failed_stop:
                    raise DeviceConnectionError('offline')
                if path == '/status':
                    return status()  # Still active; cannot confirm idle.
                return {}
            with patch('robot_dashboard.runtime.devices.request', side_effect=request):
                with self.assertRaises(DeviceError):
                    app._change_ramp_gait('http://robot.test')
            self.assertNotIn('/motion/start', calls)
            self.assertFalse(app.walking.active)

    def test_ramp_tilt_does_not_stop_gait_restart(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(), 1, 0, 'course')
        walking.crest_sequence.requested = 'crest'
        params, _ = walking.transition_settings()
        walking.resume_gait(2, 10, params)
        tilted = status(2)
        tilted['orientation'][:2] = [math.radians(20), math.radians(40)]
        frame = walking.frame(tilted, 10.1)
        self.assertTrue(walking.active)
        self.assertEqual(frame['ready'], 0)

    def test_ramp_exit_continues_beyond_duration_and_attitude_limits(self):
        walking, sample = self.clearance(.5)
        sample['orientation'] = [math.radians(20), math.radians(40), math.radians(25)]
        walking.exit_cycles = 20
        for tick in range(1, 131):
            now = tick * .1
            walking.last_confirmed_frame = now
            frame = walking.frame(sample, now)
        self.assertTrue(walking.active)
        self.assertEqual(frame['ready'], 1)
        self.assertAlmostEqual(frame['heading'], math.radians(25))

    def clearance(self, speed):
        walking = Walking()
        walking.start_settings('course')
        sample = status()
        sample['orientation'] = [0., 0., 0.]
        for joint in sample['joints']:
            joint['angle'] = 33.5 if joint['name'].endswith('_pitch') else 0
        walking.begin(sample, 1, 0, 'course')
        walking.set_speed(speed)
        walking.effective_speed = speed
        walking.crest_sequence.phase = 'descent'
        walking.exit_cycles = 1.125
        walking.exit_started = walking.exit_updated = 0
        return walking, sample

    def test_exit_counts_cycles_at_both_tested_and_requested_speeds(self):
        for speed, end in ((.5, 4.5), (.6, 3.75)):
            walking, sample = self.clearance(speed)
            for tick in range(1, round(end/.05)+1):
                now = tick*.05
                walking.last_confirmed_frame = now
                frame = walking.frame(sample, now)
                if now < end - .001:
                    self.assertEqual(frame['ready'], 1)
                    self.assertFalse(walking.navigation.corridor_requested)
            self.assertEqual(frame['ready'], 0)
            self.assertTrue(walking.navigation.corridor_requested)

    def test_exit_waits_for_feet_down_and_stops_on_sensor_loss(self):
        walking, sample = self.clearance(.5)
        walking.exit_cycles = 0
        sample['joints'][1]['angle'] = 10
        self.assertEqual(walking.frame(sample, .1)['ready'], 1)
        sample['joints'][1]['angle'] = 33.5
        self.assertEqual(walking.frame(sample, .2)['ready'], 0)
        for invalid in ('stale', 'overrun'):
            walking, sample = self.clearance(.5)
            now = .1
            if invalid == 'stale':
                sample['imuAgeMs'] = 100
            else:
                now = .5
            with self.assertRaises(DeviceError):
                walking.frame(sample, now)

    def test_exit_distance_counts_the_speed_actually_commanded(self):
        walking, sample = self.clearance(.6)
        walking.speed_governor = RampSpeedGovernor()
        walking.effective_speed = .6
        sample['orientation'][0] = math.radians(6)
        self.assertEqual(walking.frame(sample, .1)['speed'], .5)
        self.assertAlmostEqual(walking.exit_cycles, 1.125-.1*.6/2)
        self.assertEqual(walking.frame(sample, .3)['speed'], .5)
        self.assertAlmostEqual(walking.exit_cycles, 1.125-.1*.6/2-.2*.5/2)


if __name__ == '__main__':
    unittest.main()
