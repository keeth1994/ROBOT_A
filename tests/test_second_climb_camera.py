"""Test camera changes on the second climb."""
import math
import unittest

from dashboard.robot_dashboard.navigation import CourseNavigation
from dashboard.robot_dashboard.walking import Walking
from tests.test_crest_sequence import status
from tests.test_ramp_edges import ramp_frame
from robot.navigation.camera_calibration import load_camera_scan_settings
from robot.navigation.camera_scan import RampScanNavigation


def forward_frame(nav, now, offset=0, angular_speed=0):
    angle = math.radians(nav.settings['forward_deg'])
    image = ramp_frame(math.degrees(nav.mount_pitch + angle), offset=offset, body_pitch=nav.pitch)
    nav.last_imu = now
    return nav.scan_frame(now, image, angle, angular_speed, 0)


def backward_frame(nav, now, offset=0):
    angle = math.radians(nav.settings['backward_deg'])
    image = ramp_frame(math.degrees(nav.mount_pitch + angle), offset=offset, body_pitch=nav.pitch)
    nav.last_imu = now
    return nav.scan_frame(now, image, angle, 0, 0)


class SecondClimbCameraTests(unittest.TestCase):
    def navigation(self, use_imu=True):
        nav = RampScanNavigation(load_camera_scan_settings(), use_imu=use_imu,
                                 adaptive_surface=True, dark_max=125, camera_pitch_deg=40)
        nav.camera_pitch = nav.mount_pitch + math.radians(nav.settings['forward_deg'])
        return nav

    def feed(self, nav, start, pitch, count=8, step=.1):
        changes = []
        for i in range(count):
            changes.append(nav.observe_slope(start + i * step, math.radians(pitch)))
        return changes

    def second_climb(self, nav):
        self.feed(nav, 2, -15)
        self.feed(nav, 3, 15)
        return self.feed(nav, 4, -15)

    def test_second_sustained_climb_switches_to_rear_once(self):
        nav = self.navigation()
        changes = self.second_climb(nav)
        self.assertEqual(sum(changes), 1)
        self.assertEqual(nav.climbs_seen, 2)
        self.assertEqual(nav.view, 'forward')
        self.assertTrue(nav.backward_pending)
        self.assertTrue(forward_frame(nav, 5), nav.debug)
        self.assertEqual(nav.view, 'backward')
        self.assertTrue(nav.searching)
        self.assertAlmostEqual(nav.camera_target(10), math.radians(nav.settings['backward_deg']))
        started = nav.move_started
        self.second_climb(nav)
        self.assertEqual(nav.move_started, started)
        self.assertEqual(nav.report()['uphill_slopes_seen'], 2)

    def test_flat_spots_and_brief_downhill_spikes_do_not_count_twice(self):
        nav = self.navigation()
        self.feed(nav, 2, -15)
        self.feed(nav, 3, 0)
        self.feed(nav, 4, -15)
        self.feed(nav, 5, 15, count=2)
        self.feed(nav, 5.2, -15)
        self.assertEqual(nav.climbs_seen, 1)
        self.assertEqual(nav.view, 'forward')

    def test_second_climb_waits_for_both_forward_edges_before_flipping(self):
        nav = self.navigation()
        self.second_climb(nav)
        nav.mount_pitch = math.radians(70)
        self.assertTrue(forward_frame(nav, 5, offset=.09), nav.debug)
        self.assertNotEqual(nav.debug['edge_source'], 'paired')
        self.assertEqual(nav.view, 'forward')
        self.assertTrue(nav.backward_pending)
        self.assertTrue(forward_frame(nav, 5.1), nav.debug)
        self.assertEqual(nav.debug['edge_source'], 'paired')
        self.assertEqual(nav.view, 'backward')

    def test_camera_must_be_still_before_the_pending_backward_flip(self):
        nav = self.navigation()
        self.second_climb(nav)
        forward_frame(nav, 5, angular_speed=math.radians(10))
        self.assertEqual(nav.view, 'forward')
        forward_frame(nav, 5.1)
        self.assertEqual(nav.view, 'backward')

    def retry_navigation(self):
        nav = self.navigation()
        self.second_climb(nav)
        forward_frame(nav, 5)
        self.assertTrue(backward_frame(nav, 7.1, offset=.12), nav.debug)
        self.assertNotEqual(nav.debug['edge_source'], 'paired')
        self.assertEqual(nav.view, 'forward')
        self.assertEqual(nav.backward_retry, 'forward')
        self.assertFalse(nav.ready(7.1))
        forward_frame(nav, 9.2)
        self.assertEqual(nav.backward_retry, 'advance')
        self.assertTrue(nav.ready(9.2))
        return nav

    def test_single_backward_edge_returns_forward_advances_and_retries_at_both_speeds(self):
        durations = []
        for speed in (1, .5):
            nav = self.retry_navigation()
            nav.motion_speed = speed
            nav.walking = True
            for i in range(1, 100):
                now = 9.2 + i * .1
                forward_frame(nav, now)
                if nav.backward_retry == 'backward':
                    break
                self.assertTrue(nav.ready(now))
            self.assertEqual(nav.view, 'backward')
            self.assertEqual(nav.backward_retry, 'backward')
            self.assertAlmostEqual(nav.retry_distance, .10, delta=.0031)
            durations.append(now - 9.2)
            for elapsed in (2.1, 2.3, 2.5):
                backward_frame(nav, now + elapsed)
            self.assertIsNone(nav.backward_retry)
            self.assertTrue(nav.ready(now + 2.5))
        self.assertAlmostEqual(durations[1], 2 * durations[0], delta=.11)

    def test_forward_retry_uses_both_edges_to_correct_heading_and_centering(self):
        nav = self.retry_navigation()
        nav.walking = True
        nav.imu(9.25, [0, 0, 9.81], [0, 0, .2])
        heading = nav.heading
        self.assertAlmostEqual(heading, .01)
        nav.lateral = .08
        forward_frame(nav, 9.3, offset=.04)
        self.assertLess(abs(nav.heading), abs(heading))
        self.assertGreater(nav.lateral_error, 0)
        self.assertLess(nav.lateral, .08)
        self.assertTrue(nav.retry_using_vision)

    def test_single_or_missing_forward_edge_uses_original_gyro_direction_as_fallback(self):
        nav = self.retry_navigation()
        nav.heading += .05
        forward_frame(nav, 9.3, offset=.04)
        self.assertTrue(nav.retry_using_vision)
        nav.heading += .02
        nav.mount_pitch = math.radians(70)
        forward_frame(nav, 9.4, offset=.09)
        self.assertNotEqual(nav.debug['edge_source'], 'paired')
        self.assertFalse(nav.retry_using_vision)
        self.assertAlmostEqual(nav.heading, .07)
        self.assertEqual(nav.lateral_error, 0)
        nav.heading -= .01
        image = ramp_frame(60)
        image[:] = 190
        nav.last_imu = 9.5
        nav.scan_frame(9.5, image, math.radians(nav.settings['forward_deg']), 0, 0)
        self.assertFalse(nav.retry_using_vision)
        self.assertAlmostEqual(nav.heading, .06)
        self.assertEqual(nav.lateral_error, 0)
        self.assertTrue(nav.ready(9.5))
        nav.mount_pitch = math.radians(40)
        forward_frame(nav, 9.6, offset=.04)
        self.assertTrue(nav.retry_using_vision)
        self.assertNotEqual(nav.lateral_error, 0)

    def test_paused_walking_does_not_count_toward_the_forward_retry_distance(self):
        nav = self.retry_navigation()
        for now in (9.3, 9.4, 9.5):
            forward_frame(nav, now)
        self.assertEqual(nav.retry_distance, 0)
        self.assertEqual(nav.backward_retry, 'advance')

    def test_missing_backward_edges_return_forward_without_waiting_for_both(self):
        nav = self.navigation()
        self.second_climb(nav)
        forward_frame(nav, 5)
        angle = math.radians(nav.settings['backward_deg'])
        nav.last_imu = 7.1
        image = ramp_frame(158)
        image[:] = 190
        nav.scan_frame(7.1, image, angle, 0, 0)
        self.assertEqual(nav.view, 'forward')
        self.assertEqual(nav.backward_retry, 'forward')

    def test_dashboard_retry_uses_forward_heading_and_centering_at_slider_speed(self):
        walking = Walking(crest_enabled=False)
        walking.begin(status(), 1, 100, 'course')
        walking.set_speed(.5)
        nav = walking.navigation
        for start, pitch in ((102, -15), (103, 15), (104, -15)):
            self.feed(nav, start, pitch)
        forward_frame(nav.estimate, 5)
        backward_frame(nav.estimate, 7.1, offset=.12)
        forward_frame(nav.estimate, 9.2)
        self.assertEqual(nav.estimate.backward_retry, 'advance')
        nav.camera_angle, nav.camera_changed = -65, 100
        nav.imu_heading = 1
        image = ramp_frame(30, offset=.04)
        packet = status(heading=1.04) | dict(cameraPitch=dict(angle=-65, moving=False))
        for sequence, now in enumerate((109.3, 109.4), 1):
            walking.last_confirmed_frame = now
            frame = walking.frame(packet, now, (sequence, now, image))
            self.assertEqual(frame['ready'], 1, walking.message)
            self.assertEqual(frame['speed'], .5)
            self.assertLess(abs(frame['heading']), .04)
            self.assertNotEqual(frame['lateral'], 0)
            self.assertTrue(nav.estimate.retry_using_vision)
        self.assertTrue(nav.estimate.walking)
        self.assertAlmostEqual(nav.estimate.retry_distance, .0015)

    def test_gaps_and_missing_imu_break_confirmation(self):
        nav = self.navigation()
        self.feed(nav, 2, -15, count=2)
        self.feed(nav, 4, -15, count=2)
        self.assertEqual(nav.climbs_seen, 0)
        nav.observe_slope(4.2, None)
        self.feed(nav, 4.3, -15, count=2)
        self.assertEqual(nav.climbs_seen, 0)

    def test_slow_fresh_samples_can_confirm_slopes(self):
        nav = self.navigation()
        self.feed(nav, 2, -15, count=3, step=.6)
        self.feed(nav, 4, 15, count=3, step=.6)
        self.feed(nav, 6, -15, count=3, step=.6)
        self.assertEqual(nav.climbs_seen, 2)
        self.assertEqual(nav.view, 'forward')
        forward_frame(nav, 8)
        self.assertEqual(nav.view, 'backward')

    def test_camera_only_mode_does_not_count_slopes(self):
        nav = self.navigation(use_imu=False)
        self.assertFalse(any(self.second_climb(nav)))
        self.assertEqual(nav.view, 'forward')

    def test_simulation_imu_uses_the_same_second_climb_trigger(self):
        nav = self.navigation()
        for start, degrees in ((2, -15), (7, 15), (12, -15)):
            pitch = math.radians(degrees)
            acceleration = [-9.81 * math.sin(pitch), 0, 9.81 * math.cos(pitch)]
            for i in range(100):
                nav.imu(start + i * .05, acceleration, [0, 0, 0])
        self.assertEqual(nav.climbs_seen, 2)
        forward_frame(nav, 17)
        self.assertEqual(nav.view, 'backward')

    def test_existing_rear_view_is_kept_without_another_sweep(self):
        nav = self.navigation()
        nav.view = 'backward'
        self.second_climb(nav)
        self.assertEqual(nav.climbs_seen, 2)
        self.assertIsNone(nav.move_started)
        self.assertFalse(nav.searching)

    def test_dashboard_uses_elapsed_time_for_second_climb(self):
        nav = CourseNavigation(100, scan_ramp=True)
        self.feed(nav, 102, -15)
        self.feed(nav, 103, 15)
        self.feed(nav, 104, -15)
        self.assertEqual(nav.camera_target(105), nav.camera_calibration['forward_servo_deg'])
        forward_frame(nav.estimate, 5)
        self.assertEqual(nav.camera_target(110), nav.camera_calibration['backward_servo_deg'])

    def test_dashboard_reacquires_rear_image_and_resumes(self):
        nav = CourseNavigation(100, scan_ramp=True)
        nav.camera_angle = -65
        nav.estimate.camera_pitch = math.radians(30)
        self.feed(nav, 102, -15)
        self.feed(nav, 103, 15)
        self.feed(nav, 104, -15)
        forward_frame(nav.estimate, 5)
        image = ramp_frame(158, body_pitch=math.radians(-15))
        packet = dict(orientationValid=True, orientation=[0, math.radians(-15), 0], imuAgeMs=0,
                      cameraPitch=dict(angle=-4, moving=False))
        self.assertFalse(nav.update(packet, 107, (1, 107, image)))
        for i, now in enumerate((107.2, 107.4, 107.6), 2):
            ready = nav.update(packet, now, (i, now, image))
        self.assertTrue(ready, nav.reason)
        self.assertEqual(nav.estimate.view, 'backward')
        self.assertFalse(nav.estimate.searching)


if __name__ == '__main__':
    unittest.main()
