"""Check physical dashboard camera control without sending commands to hardware."""
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from dashboard.robot_dashboard.devices import DeviceError, DeviceConnectionError
from dashboard.robot_dashboard.runtime import Dashboard
from dashboard.robot_dashboard.walking import JOINTS, Walking
from robot.navigation.camera_calibration import servo_to_cradle_deg
from robot.navigation.debug import navigation_status
from tests.test_camera_scan import CONFIG, ramp_image


def status(angle=-65, moving=False, minimum=-80):
    return dict(servoReady=True, calibratedAngles=True, motionProtocol=2, faulted=False,
                externalActive=True, motionSession=1, corridorSearchGait=True,
                signedYawBias=True, crestWaveGait=True, rearStrideScale=True, frontPitchLift=True,
                orientationValid=True, orientation=[0, 0, 0], imuAgeMs=10,
                irAgeMs=10, irMillivolts=[100, 100],
                cameraPitch=dict(angle=angle, moving=moving, minAngle=minimum, maxAngle=45),
                joints=[dict(name=name, channel=i) for i, name in enumerate(JOINTS)])


class DashboardNavigationTests(unittest.TestCase):
    def setUp(self):
        self.walking = Walking()
        self.sequence = 0

    def frame(self, elapsed, angle=-65, blank=False, missing=False, delay=0, **changes):
        now = 100 + elapsed
        packet = status(angle) | changes
        self.walking.last_confirmed_frame = now
        self.sequence += 1
        pitch = 40 + servo_to_cradle_deg(angle, self.walking.camera_settings)
        rgb = (np.full((240, 320, 3), 180, dtype=np.uint8) if blank else
               ramp_image(math.radians(pitch)))
        camera = None if missing else (self.sequence, now - delay, rgb)
        return self.walking.frame(packet, now, camera)

    def start_ramp(self, profile='course'):
        self.walking.begin(status(), 1, 100, profile)
        self.assertEqual(self.walking.camera_target(100), -65)
        self.assertEqual(self.frame(0)['ready'], 0)
        self.assertEqual(self.frame(2)['ready'], 1)

    def test_ramp_modes_use_half_speed_without_slowing_corridor_or_plain_walking(self):
        base_period = self.walking.params['period']
        for profile in ('course', 'course_camera', 'corridor_search', 'corridor', 'walk'):
            with self.subTest(profile=profile):
                settings = self.walking.start_settings(profile)
                expected_period = 2 * base_period if profile in ('course', 'course_camera') else base_period
                self.assertEqual(settings['period'], expected_period)
                self.walking.begin(status(), 1, 100, profile)
                self.assertEqual(self.walking.state()['period_s'], expected_period)
                self.walking.set_speed(.5)
                self.assertEqual(self.walking.state()['period_s'], expected_period / .5)
                self.walking.set_speed(1)

    def test_both_ramp_buttons_pause_after_one_second_and_search_then_resume(self):
        for profile in ('course', 'course_camera'):
            with self.subTest(profile=profile):
                self.start_ramp(profile)
                self.assertEqual(self.frame(2.9, blank=True)['ready'], 1)
                self.assertEqual(self.frame(3.1, blank=True)['ready'], 0)
                self.assertTrue(self.walking.navigation.estimate.searching)
                self.assertEqual(self.walking.navigation.estimate.view, 'backward')
                for now in np.arange(103.1, 106, .1):
                    self.assertTrue(-65 <= self.walking.camera_target(now) <= -4)
                self.assertEqual(self.frame(4.1, angle=-34)['ready'], 0)
                self.assertEqual(self.frame(5.2, angle=-4, cameraPitch=status(-4, moving=True)['cameraPitch'])['ready'], 0)
                self.assertEqual(self.frame(5.31, angle=-4)['ready'], 0)
                self.assertEqual(self.frame(5.51, angle=-4)['ready'], 0)
                self.assertEqual(self.frame(5.62, angle=-4)['ready'], 1)
                self.assertEqual(self.walking.camera_target(105.62), -4)
                self.assertEqual(self.frame(6.8, angle=-4, blank=True)['ready'], 0)
                self.assertEqual(self.walking.navigation.estimate.view, 'forward')
                self.assertEqual(self.walking.camera_target(109), -65)

    def test_camera_only_ramp_does_not_require_imu(self):
        self.start_ramp('course_camera')
        self.assertEqual(self.frame(2.1, orientationValid=False, imuAgeMs=500)['ready'], 1)

    def test_video_arriving_before_current_imu_does_not_delay_reacquiring(self):
        self.start_ramp()
        self.assertEqual(self.frame(3.1, blank=True)['ready'], 0)
        self.assertEqual(self.frame(5.2, angle=-4)['ready'], 0)
        self.assertEqual(self.frame(5.5, angle=-4, delay=.12)['ready'], 0)
        self.assertAlmostEqual(self.walking.navigation.estimate.last_seen, 5.38)
        self.assertEqual(self.frame(5.7, angle=-4, delay=.12)['ready'], 0)
        self.assertEqual(self.frame(5.81, angle=-4, delay=.12)['ready'], 1)

    def test_adaptive_detector_preserves_offset_sign_in_both_camera_views(self):
        for offset in (-.025, .025):
            measured = []
            for angle in (-65, -4):
                self.walking.begin(status(angle), 1, 100, 'course_camera')
                nav = self.walking.navigation
                if angle == -4:
                    nav.estimate.view = 'backward'
                pitch = 40 + servo_to_cradle_deg(angle, self.walking.camera_settings)
                rgb = ramp_image(math.radians(pitch), offset)
                for elapsed in (0, 2, 2.1, 2.2):
                    nav.update(status(angle), 100 + elapsed, (elapsed, 100 + elapsed, rgb))
                self.assertTrue(nav.estimate.debug['accepted'])
                measured.append(nav.estimate.lateral)
            self.assertGreater(measured[0] * offset, 0)
            np.testing.assert_allclose(measured[0], measured[1], atol=.002)

    def test_missing_imu_holds_without_turning_camera(self):
        self.start_ramp()
        self.assertEqual(self.frame(3.1, blank=True, imuAgeMs=500)['ready'], 0)
        self.assertFalse(self.walking.navigation.estimate.searching)
        self.assertEqual(self.walking.camera_target(103.1), -65)

    def test_ramp_steering_follows_configured_offset_in_both_camera_views(self):
        target = CONFIG['ramp_offset_right_m']
        for profile in ('course', 'course_camera'):
            for angle in (-65, -4):
                for offset in (0, -target, -target - .025):
                    with self.subTest(profile=profile, angle=angle, offset=offset):
                        self.walking.begin(status(angle), 1, 100, profile)
                        nav = self.walking.navigation
                        if angle == -4:
                            nav.estimate.view = 'backward'
                        pitch = 40 + servo_to_cradle_deg(angle, self.walking.camera_settings)
                        rgb = ramp_image(math.radians(pitch), offset)
                        for i in range(25):
                            now = 102 + i * .1
                            self.walking.last_confirmed_frame = now
                            frame = self.walking.frame(status(angle), now, (i, now, rgb))
                        self.assertEqual(frame['ready'], 1)
                        self.assertAlmostEqual(nav.estimate.lateral, offset, delta=.003)
                        self.assertAlmostEqual(frame['lateral'], offset + target, delta=.003)

    def test_missing_video_holds_after_one_second(self):
        self.start_ramp()
        self.assertEqual(self.frame(3.1, missing=True)['ready'], 0)
        self.assertFalse(self.walking.navigation.estimate.searching)
        self.assertEqual(navigation_status(self.walking.navigation.estimate, 3.1)[0], 'HOLD REQUIRED')

    def request_corridor(self):
        # Exercise the runtime handoff separately from ramp completion detection.
        self.start_ramp()
        self.walking.navigation.corridor_requested = True
        self.walking.last_confirmed_frame = 106.34

    def test_one_climb_then_downhill_and_flat_does_not_finish_ramp(self):
        self.start_ramp()
        nav = self.walking.navigation
        for start, pitch in ((102, -15), (103, 15), (104, 0)):
            for i in range(8):
                nav.observe_slope(start + i * .1, math.radians(pitch))
        self.assertEqual(nav.estimate.climbs_seen, 1)
        self.assertFalse(nav.ramp_finished)
        self.assertFalse(nav.corridor_requested)

    def test_flat_uphill_and_short_downhill_readings_do_not_trigger_ramp_exit(self):
        for pitches in ((0,) * 12, (-12,) * 6 + (0,) * 6, (0,) * 5 + (12,) + (0,) * 6):
            self.start_ramp()
            for i, pitch in enumerate(pitches):
                self.frame(2.1 + i * .1, orientation=[0, math.radians(pitch), 0])
            self.assertFalse(self.walking.navigation.ramp_finished)
            self.assertFalse(self.walking.navigation.corridor_requested)

    def test_stale_imu_samples_cannot_confirm_flat_ground(self):
        self.start_ramp()
        self.walking.navigation.estimate.climbs_seen = 2
        for elapsed in (3, 3.1, 3.2, 3.3, 3.4):
            self.frame(elapsed, orientation=[0, math.radians(12), 0])
        self.assertTrue(self.walking.navigation.downhill_seen)
        for elapsed in (3.5, 3.6, 3.7, 3.8):
            self.assertEqual(self.frame(elapsed, imuAgeMs=500)['ready'], 0)
        self.assertFalse(self.walking.navigation.ramp_finished)
        self.frame(3.9)
        self.assertFalse(self.walking.navigation.ramp_finished)

    def test_camera_only_does_not_use_imu_for_automatic_ramp_completion(self):
        self.start_ramp('course_camera')
        for i, pitch in enumerate((-15,) * 6 + (15,) * 6 + (-15,) * 6 + (15,) * 6 + (0,) * 6):
            self.frame(2.1 + .1 * i, orientation=[0, math.radians(pitch), 0])
        self.assertFalse(self.walking.navigation.ramp_finished)
        self.assertFalse(self.walking.navigation.corridor_requested)

    def test_camera_only_ramp_start_does_not_require_minus_80_support(self):
        with patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=100):
            app = Dashboard()
            app.browser_heartbeat = 100
            calls = []

            def request(url, path, values=None, **kwargs):
                calls.append(path)
                if path == '/status':
                    return status(minimum=-70) | dict(externalActive=False)
                return dict(session=1) if path == '/motion/start' else {}

            with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                app.walk_command('course_camera')
            self.assertEqual(calls, ['/status', '/motion/start'])
            self.assertEqual(app.walking.camera_target(100), -65)

    def test_control_loop_stops_ramp_before_starting_corridor_with_new_gait_and_session(self):
        self.request_corridor()
        with patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=106.34):
            app = Dashboard()
            app.walking = self.walking
            app.browser_heartbeat = 106.34
            app.camera.navigation_frame = Mock(return_value=None)
            app.stop_event = Mock()
            app.stop_event.wait.side_effect = [False, True]
            calls = []

            def request(url, path, values=None, **kwargs):
                calls.append((path, values))
                if path == '/status':
                    stopped = any(item[0] == '/stop' for item in calls)
                    return status(angle=None if stopped else -65) | dict(externalActive=not stopped)
                return dict(session=2) if path == '/motion/start' else {}

            with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                app._walk_loop()
            self.assertEqual([item[0] for item in calls],
                             ['/status', '/motion/adjust', '/stop', '/status',
                              '/camera-pitch', '/camera-pitch', '/motion/start'])
            self.assertEqual(calls[1][1]['ready'], 0)
            self.assertEqual(calls[5][1], dict(angle=-80))
            self.assertEqual(calls[-1][1]['profile'], 'corridor_search')
            self.assertTrue(app.walking.active)
            self.assertEqual(app.walking.profile, 'corridor_search')
            self.assertEqual(app.walking.session, 2)
            self.assertEqual(app.walking.camera_target(106.34), -80)
            self.assertEqual(app.walking.frame(status(-80) | dict(motionSession=2), 106.34)['ready'], 0)

    def test_failed_corridor_restart_stops_outputs_without_retrying_start(self):
        self.request_corridor()
        with patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=106.34):
            app = Dashboard()
            app.walking = self.walking
            app.browser_heartbeat = 106.34
            app.camera.navigation_frame = Mock(return_value=None)
            app.stop_event = Mock()
            app.stop_event.wait.side_effect = [False, True]
            calls = []

            def request(url, path, values=None, **kwargs):
                calls.append(path)
                if path == '/status':
                    return status() | dict(externalActive='/stop' not in calls)
                if path == '/motion/start':
                    raise DeviceConnectionError('Start response was lost.')
                return {}

            with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                app._walk_loop()
            self.assertFalse(app.walking.active)
            self.assertEqual(calls.count('/motion/start'), 1)
            self.assertEqual(calls[-1], '/stop')

    def test_no_ramp_keeps_walking_paused_and_searches_both_views(self):
        self.start_ramp()
        self.assertEqual(self.frame(3.1, blank=True)['ready'], 0)
        settings = self.walking.navigation.camera_calibration
        rear_departure = 5.4 + settings['search_view_s'] + .1
        forward_arrival = rear_departure + settings['sweep_s'] + .1
        forward_departure = forward_arrival + .2 + settings['search_view_s'] + .1
        for elapsed, angle in ((5.2, -4), (5.4, -4), (rear_departure, -4),
                               (forward_arrival, -65), (forward_arrival + .2, -65),
                               (forward_departure, -65)):
            self.assertEqual(self.frame(elapsed, angle=angle, blank=True)['ready'], 0)
        self.assertEqual(self.walking.navigation.estimate.view, 'backward')
        self.assertEqual(self.walking.camera_target(100 + forward_departure + settings['sweep_s'] + .1), -4)

    def test_corridor_waits_for_minus_80_and_a_frame_after_settling(self):
        self.walking.begin(status(), 1, 100, 'corridor_search')
        self.assertEqual(self.walking.camera_target(100), -80)
        self.assertEqual(self.frame(2, angle=-65, blank=True)['ready'], 0)
        self.assertIn('-80', self.walking.message)
        self.assertEqual(self.frame(2.1, angle=-80, blank=True)['ready'], 0)
        self.assertEqual(self.frame(2.7, angle=-80, blank=True)['ready'], 1)

    def test_corridor_approach_sends_faster_speed_then_slows_on_the_chosen_ir(self):
        self.walking.begin(status(), 1, 100, 'corridor_search')
        nav = self.walking.navigation
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_walk_time = 10
        nav.approach_goal = np.array([1., 0.])
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[100:210, 192:] = (180, 120, 50)
        packet = status(angle=-80)
        for now in (100, 102):
            self.walking.last_confirmed_frame = now
            frame = self.walking.frame(packet, now, (now, now, image))
        self.assertEqual(frame['ready'], 1)
        self.assertEqual(frame['speed'], 1.5)
        packet['irMillivolts'][1] = 800
        self.walking.last_confirmed_frame = 102.1
        frame = self.walking.frame(packet, 102.1, (102.1, 102.1, image))
        self.assertEqual(frame['speed'], 1)

    def test_start_enables_and_aims_camera_before_starting_gait(self):
        for profile, angle in (('course', -65), ('course_camera', -65), ('corridor_search', -80)):
            with self.subTest(profile=profile), patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=100):
                app = Dashboard()
                app.browser_heartbeat = 100
                calls = []

                def request(url, path, values=None, **kwargs):
                    calls.append((path, values))
                    if path == '/status':
                        return status(angle=None) | dict(externalActive=False)
                    return dict(session=1) if path == '/motion/start' else {}

                with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                    app.walk_command(profile)
                self.assertEqual([item[0] for item in calls],
                                 ['/status', '/camera-pitch', '/camera-pitch', '/motion/start'])
                self.assertEqual(calls[1][1], dict(angle=0))
                self.assertEqual(calls[2][1], dict(angle=angle))
                self.assertTrue(app.camera.enabled)
                self.assertTrue(app.walking.state()['camera_controlled'])

    def test_old_firmware_cannot_start_ramp_or_corridor_search(self):
        for profile in ('course', 'corridor_search'):
            with self.subTest(profile=profile), patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=100):
                app = Dashboard()
                app.browser_heartbeat = 100
                calls = []

                def request(url, path, values=None, **kwargs):
                    calls.append(path)
                    return status(minimum=-70) | dict(externalActive=False)

                with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                    with self.assertRaisesRegex(DeviceError, 'firmware.*-80'):
                        app.walk_command(profile)
                self.assertNotIn('/motion/start', calls)
                self.assertNotIn('/camera-pitch', calls)
                self.assertFalse(app.walking.active)

    def test_control_loop_sends_pause_before_turning_camera(self):
        with patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=100):
            app = Dashboard()
            app.browser_heartbeat = 100
            app.walking.begin(status(), 1, 97, 'course')
            app.walking.last_confirmed_frame = 100
            nav = app.walking.navigation
            nav.estimate.start_search(2, math.radians(servo_to_cradle_deg(-65, nav.camera_calibration)), 'backward')
            app.stop_event = Mock()
            app.stop_event.wait.side_effect = [False, True]
            calls = []

            def request(url, path, values=None, **kwargs):
                calls.append((path, values))
                return status() if path == '/status' else {}

            with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                app._walk_loop()
            self.assertEqual([item[0] for item in calls], ['/status', '/motion/adjust', '/camera-pitch'])
            self.assertEqual(calls[1][1]['ready'], 0)
            self.assertEqual(calls[2][1], dict(angle=-35))

    def test_camera_command_failure_stops_walking(self):
        with patch('dashboard.robot_dashboard.runtime.time.monotonic', return_value=100):
            app = Dashboard()
            app.browser_heartbeat = 100
            app.walking.begin(status(), 1, 97, 'course')
            app.walking.last_confirmed_frame = 100
            nav = app.walking.navigation
            nav.estimate.start_search(2, math.radians(servo_to_cradle_deg(-65, nav.camera_calibration)), 'backward')
            app.stop_event = Mock()
            app.stop_event.wait.side_effect = [False, True]
            calls = []

            def request(url, path, values=None, **kwargs):
                calls.append(path)
                if path == '/camera-pitch':
                    raise DeviceError('Camera command failed.')
                return status() if path == '/status' else {}

            with patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
                app._walk_loop()
            self.assertFalse(app.walking.active)
            self.assertEqual(calls[-1], '/stop')

    def test_manual_camera_control_is_blocked_during_automatic_camera_control(self):
        app = Dashboard()
        app.walking.begin(status(), 1, 100, 'course')
        with self.assertRaisesRegex(DeviceError, 'adjusting the camera manually'):
            app.servo_command('/camera-pitch', dict(angle=0))

    def test_imu_only_and_plain_walking_do_not_take_camera_control(self):
        for profile in ('course_imu', 'walk', 'corridor', 'dance'):
            self.walking.begin(status(), 1, 100, profile)
            self.assertIsNone(self.walking.camera_target(100))
            self.assertFalse(self.walking.state()['camera_controlled'])


if __name__ == '__main__':
    unittest.main()
