"""Top platform, descent and finished-ramp checks using synthetic camera frames."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from dashboard.robot_dashboard.navigation import CourseNavigation
from dashboard.robot_dashboard.walking import Walking
from dashboard.test_crest_sequence import status
from dashboard.test_ramp_edges import ramp_frame
from robot_b91.camera_calibration import servo_to_cradle_deg


class RampFlatSectionTests(unittest.TestCase):
    def setUp(self):
        self.nav = CourseNavigation(100, scan_ramp=True)
        self.sequence = 0

    def slope(self, start, degrees):
        for i in range(8):
            self.nav.observe_slope(100 + start + i * .1, math.radians(degrees))

    def frame(self, time, angle=-4, pitch=0, ramp=True, missing=False, clipped=False):
        self.sequence += 1
        packet = status() | dict(imuAgeMs=0, orientation=[0, math.radians(pitch), 0],
                                cameraPitch=dict(angle=angle, moving=False))
        camera_pitch = 40 + servo_to_cradle_deg(angle, self.nav.camera_calibration)
        image = (ramp_frame(camera_pitch, body_pitch=math.radians(pitch)) if ramp else
                 np.full((240, 320, 3), 180, dtype=np.uint8))
        if clipped:
            image[:] = 70
            image[:60] = 180
        camera = None if missing else (self.sequence, 100 + time, image)
        return self.nav.update(packet, 100 + time, camera)

    def top_check(self):
        self.slope(2, -15)
        self.slope(3, 15)
        self.slope(4, -15)
        self.assertEqual(self.nav.estimate.climbs_seen, 2)
        for time in (5, 5.1, 5.2, 5.3):
            self.frame(time)
        self.assertTrue(self.nav.checking_exit)
        self.assertEqual(self.nav.camera_target(108), -65)

    def cross_platform(self, clipped=False):
        self.top_check()
        for time in (7.5, 7.7, 7.9, 8.1):
            self.frame(time, angle=-65, clipped=clipped)
        self.assertTrue(self.nav.top_checked)
        self.assertFalse(self.nav.checking_exit)
        self.assertFalse(self.nav.ramp_finished)
        self.assertEqual(self.nav.camera_target(111), -4)
        for time in (10.3, 10.5, 10.7, 10.9):
            ready = self.frame(time)
        self.assertTrue(ready, self.nav.reason)

    def ground_check(self):
        self.cross_platform()
        for time in (11, 11.1, 11.2, 11.3, 11.4, 11.5):
            self.frame(time, pitch=15)
        for time in (11.6, 11.7, 11.8, 11.9):
            self.frame(time)
        self.assertTrue(self.nav.checking_exit)
        self.assertEqual(self.nav.camera_target(114), -65)

    def test_platform_then_descent_then_flat_ground_finishes(self):
        self.ground_check()
        for time in (14.1, 14.3, 14.5, 14.7):
            ready = self.frame(time, angle=-65, ramp=False)
        self.assertFalse(ready)
        self.assertTrue(self.nav.ramp_finished)
        self.assertFalse(self.nav.corridor_requested)
        self.assertIn('Ramp finished', self.nav.reason)

    def test_staying_on_same_platform_does_not_flip_forward_again(self):
        self.cross_platform()
        for i in range(12):
            self.assertTrue(self.frame(11 + i * .1))
        self.assertFalse(self.nav.checking_exit)
        self.assertEqual(self.nav.estimate.view, 'backward')

    def test_visible_mat_without_edges_is_still_a_platform(self):
        self.cross_platform(clipped=True)

    def test_no_platform_after_sharp_crest_can_finish_at_first_flat(self):
        self.top_check()
        for time in (7.5, 7.7, 7.9, 8.1):
            self.frame(time, angle=-65, ramp=False)
        self.assertTrue(self.nav.ramp_finished)

    def test_empty_forward_view_cannot_finish_while_robot_is_tilted(self):
        self.top_check()
        for time in (7.5, 7.7, 7.9, 8.1):
            self.frame(time, angle=-65, pitch=15, ramp=False)
        self.assertFalse(self.nav.ramp_finished)
        self.assertIsNone(self.nav.exit_missing_since)

    def test_missing_images_and_rear_images_cannot_finish_forward_check(self):
        self.ground_check()
        for time in (14.1, 14.3, 14.5, 14.7):
            self.frame(time, angle=-4, ramp=False)
        self.assertFalse(self.nav.ramp_finished)
        self.frame(14.9, angle=-65, ramp=False)
        for time in (15.1, 15.3, 15.5):
            self.frame(time, angle=-65, missing=True)
        self.assertFalse(self.nav.ramp_finished)

    def test_ramp_visible_on_later_flat_returns_to_rear_view(self):
        self.ground_check()
        for time in (14.1, 14.3, 14.5, 14.7):
            self.frame(time, angle=-65)
        self.assertFalse(self.nav.ramp_finished)
        self.assertFalse(self.nav.checking_exit)
        self.assertEqual(self.nav.camera_target(118), -4)

    def test_finished_ramp_stops_walking(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(), 1, 100, 'course')
        walking.navigation.ramp_finished = True
        walking.navigation.reason = 'Ramp finished: flat ground and no ramp ahead.'
        with patch.object(walking.navigation, 'update', return_value=False):
            frame = walking.frame(status(), 100.1)
        self.assertEqual(frame['ready'], 0)
        self.assertFalse(walking.active)
        self.assertFalse(walking.navigation.corridor_requested)


if __name__ == '__main__':
    unittest.main()
