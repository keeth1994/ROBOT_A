"""Ramp completion from two climbs, a final descent and flat IMU readings."""
import math
import unittest

import numpy as np

from dashboard.robot_dashboard.navigation import CourseNavigation
from dashboard.robot_dashboard.walking import Walking
from dashboard.test_crest_sequence import status
from dashboard.test_ramp_edges import ramp_frame


class RampFlatSectionTests(unittest.TestCase):
    def setUp(self):
        self.nav = CourseNavigation(100, scan_ramp=True)

    def sample(self, time, degrees, age=0, image=None):
        packet = status() | dict(imuAgeMs=age, orientation=[0, math.radians(degrees), 0],
                                cameraPitch=dict(angle=-4, moving=False))
        camera = None if image is None else (time, 100 + time, image)
        return self.nav.update(packet, 100 + time, camera)

    def slope(self, start, degrees, count=8, step=.1, **kwargs):
        for i in range(count):
            ready = self.sample(start + i * step, degrees, **kwargs)
        return ready

    def two_climbs(self, **kwargs):
        self.slope(2, -15, **kwargs)
        self.slope(3, 15, **kwargs)
        self.slope(4, -15, **kwargs)
        self.assertEqual(self.nav.estimate.climbs_seen, 2)
        self.assertFalse(self.nav.downhill_seen)

    def test_final_descent_and_flat_finish_without_camera_images(self):
        self.two_climbs()
        self.slope(5, 0)
        self.assertFalse(self.nav.ramp_finished)
        self.slope(6, 15)
        self.assertTrue(self.nav.downhill_seen)
        self.assertFalse(self.nav.ramp_finished)
        self.assertFalse(self.slope(7, 0))
        self.assertTrue(self.nav.ramp_finished)
        self.assertFalse(self.nav.corridor_requested)
        self.assertIn('two climbs, final descent and flat ground', self.nav.reason)

    def test_top_platform_is_not_finished_and_keeps_rear_view(self):
        self.two_climbs()
        self.slope(5, 0, count=80)
        self.assertFalse(self.nav.ramp_finished)
        self.assertFalse(self.nav.downhill_seen)
        self.assertEqual(self.nav.camera_target(113), -4)

    def test_first_descent_cannot_count_as_final_descent(self):
        self.slope(2, -15)
        self.slope(3, 15)
        self.slope(4, 0)
        self.assertEqual(self.nav.estimate.climbs_seen, 1)
        self.assertFalse(self.nav.ramp_finished)
        self.slope(5, -15)
        self.slope(6, 0)
        self.assertFalse(self.nav.ramp_finished)

    def test_sharp_crest_needs_no_platform_before_final_descent(self):
        self.two_climbs()
        self.slope(5, 15)
        self.slope(6, 0)
        self.assertTrue(self.nav.ramp_finished)

    def test_short_downhill_spike_does_not_finish(self):
        self.two_climbs()
        self.slope(5, 15, count=2)
        self.slope(5.2, 0)
        self.assertFalse(self.nav.ramp_finished)

    def test_stale_imu_cannot_confirm_flat_ground(self):
        self.two_climbs()
        self.slope(5, 15)
        self.slope(6, 0, age=500)
        self.assertFalse(self.nav.ramp_finished)
        self.sample(7, 0)
        self.assertFalse(self.nav.ramp_finished)
        self.slope(7.1, 0)
        self.assertTrue(self.nav.ramp_finished)

    def test_gap_in_downhill_readings_resets_confirmation(self):
        self.two_climbs()
        self.slope(5, 15, count=2)
        self.slope(7, 15, count=2)
        self.slope(7.2, 0)
        self.assertFalse(self.nav.ramp_finished)

    def test_visible_ramp_or_empty_camera_does_not_change_completion(self):
        for image in (ramp_frame(158), np.full((240, 320, 3), 180, dtype=np.uint8)):
            with self.subTest(ramp_visible=bool(image.min() < 180)):
                self.nav = CourseNavigation(100, scan_ramp=True)
                self.two_climbs(image=image)
                self.slope(5, 15, image=image)
                self.slope(6, 0, image=image)
                self.assertTrue(self.nav.ramp_finished)

    def test_slow_fresh_imu_samples_can_complete_sequence(self):
        for start, pitch in ((2, -15), (4, 15), (6, -15), (8, 0), (10, 15), (12, 0)):
            self.slope(start, pitch, count=3, step=.6)
        self.assertTrue(self.nav.ramp_finished)

    def test_finished_ramp_stops_walking_without_starting_corridor(self):
        walking = Walking(crest_enabled=False)
        walking.start_settings('course')
        walking.begin(status(), 1, 100, 'course')
        for start, pitch in ((2, -15), (3, 15), (4, -15), (5, 0), (6, 15), (7, 0)):
            for i in range(8):
                now = 100 + start + i * .1
                walking.last_confirmed_frame = now
                packet = status() | dict(imuAgeMs=0, orientation=[0, math.radians(pitch), 0])
                frame = walking.frame(packet, now)
                if not walking.active:
                    break
            if not walking.active:
                break
        self.assertEqual(frame['ready'], 0)
        self.assertFalse(walking.active)
        self.assertTrue(walking.navigation.ramp_finished)
        self.assertFalse(walking.navigation.corridor_requested)


if __name__ == '__main__':
    unittest.main()
