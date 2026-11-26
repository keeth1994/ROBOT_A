"""Second-climb camera changes from IMU observations, without hardware commands."""
import math
import unittest

from dashboard.robot_dashboard.navigation import CourseNavigation
from dashboard.test_ramp_edges import ramp_frame
from robot_b91.camera_calibration import load_camera_scan_settings
from robot_b91.camera_scan import RampScanNavigation


class SecondClimbCameraTests(unittest.TestCase):
    def navigation(self, use_imu=True):
        nav = RampScanNavigation(load_camera_scan_settings(), use_imu=use_imu)
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
        self.assertEqual(nav.view, 'backward')

    def test_existing_rear_view_is_kept_without_another_sweep(self):
        nav = self.navigation()
        nav.view = 'backward'
        self.second_climb(nav)
        self.assertEqual(nav.climbs_seen, 2)
        self.assertIsNone(nav.move_started)
        self.assertFalse(nav.searching)

    def test_dashboard_uses_elapsed_time_and_cancels_early_exit_check(self):
        nav = CourseNavigation(100, scan_ramp=True)
        self.feed(nav, 102, -15)
        self.feed(nav, 103, 15)
        nav.checking_exit = nav.exit_aimed = True
        self.feed(nav, 104, -15)
        self.assertFalse(nav.checking_exit)
        self.assertFalse(nav.exit_aimed)
        self.assertEqual(nav.camera_target(110), nav.camera_calibration['backward_servo_deg'])

    def test_dashboard_reacquires_rear_image_and_resumes(self):
        nav = CourseNavigation(100, scan_ramp=True)
        nav.camera_angle = -65
        nav.estimate.camera_pitch = math.radians(30)
        self.feed(nav, 102, -15)
        self.feed(nav, 103, 15)
        self.feed(nav, 104, -15)
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
