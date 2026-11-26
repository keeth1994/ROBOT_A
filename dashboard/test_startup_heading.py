"""Startup must not retain a heading fitted while the MCU attitude settles."""
import math
import unittest

import numpy as np

from robot_dashboard.navigation import CourseNavigation
from robot_dashboard.walking import Walking
from test_camera_body_offset import ramp_image
from test_ramp_alignment import status


def observe(nav, now, *, heading=0., pitch=0., sequence=None):
    state = status(orientation=[0., math.radians(pitch), 0.])
    lens = np.array([nav.estimate.camera_x, nav.estimate.camera_y, nav.estimate.camera_z])
    image = ramp_image(0., math.radians(pitch), math.radians(heading), 0., lens)
    return nav.update(state, now, (round(now*100) if sequence is None else sequence, now, image))


class StartupHeadingTests(unittest.TestCase):
    def navigation(self):
        nav = CourseNavigation(0., scan_ramp=True)
        nav.update(status(), 2., None)  # Establish the camera geometry; let it settle.
        return nav

    def test_pre_settle_frames_cannot_initialize_heading(self):
        nav = CourseNavigation(0., scan_ramp=True)
        self.assertFalse(observe(nav, .5, heading=-19))
        self.assertFalse(nav.estimate.heading_initialized)
        self.assertIsNone(nav.estimate.last_seen)
        nav.update(status(), 2., None)
        for now in (2.2, 2.35, 2.5, 2.65):
            self.assertFalse(observe(nav, now))
        self.assertTrue(observe(nav, 2.85))
        self.assertFalse(nav.initial_pose_pending)
        self.assertAlmostEqual(nav.estimate.heading, 0., delta=math.radians(.5))
        self.assertFalse(nav.estimate.require_visual_heading)

    def test_first_post_settle_outlier_does_not_lock_in_heading(self):
        nav = self.navigation()
        self.assertFalse(observe(nav, 2.2, heading=-19))
        for now in np.arange(2.35, 5.11, .15):
            self.assertFalse(observe(nav, float(now)))
        self.assertTrue(observe(nav, 5.35))
        self.assertAlmostEqual(nav.estimate.heading, 0., delta=math.radians(.5))

    def test_changing_attitude_and_duplicate_frames_cannot_release_steps(self):
        nav = self.navigation()
        for i in range(12):
            self.assertFalse(observe(nav, 2.2+i*.15, pitch=0 if i%2 else 2))
        nav = self.navigation()
        for i in range(12):
            self.assertFalse(observe(nav, 2.2+i*.15, sequence=1))
        self.assertTrue(nav.initial_pose_pending)

    def test_alignment_handover_keeps_verified_heading(self):
        nav = CourseNavigation(0., scan_ramp=True)
        nav.estimate.heading = math.radians(3)
        nav.estimate.heading_initialized = True
        nav.update(status(), .2, None)
        self.assertFalse(nav.initial_pose_pending)
        self.assertFalse(nav.estimate.require_visual_heading)
        self.assertAlmostEqual(nav.estimate.heading, math.radians(3))

    def test_startup_accepts_fresh_images_with_slow_acknowledgements(self):
        for gap in (.6, .85):
            nav = self.navigation()
            for i in range(3):
                self.assertFalse(observe(nav, 2.2+i*gap))
            self.assertTrue(observe(nav, 2.2+3*gap))
            self.assertFalse(nav.initial_pose_pending)

    def test_edge_search_recovers_at_observed_wifi_cadence(self):
        nav = self.navigation()
        nav.initial_pose_pending = False
        nav.estimate.heading_initialized = True
        nav.estimate.searching = True
        self.assertFalse(observe(nav, 2.2))
        self.assertTrue(observe(nav, 2.8))
        self.assertFalse(nav.estimate.searching)

    def test_stale_images_and_interrupted_edges_cannot_confirm_search(self):
        nav = self.navigation()
        nav.initial_pose_pending = False
        nav.estimate.heading_initialized = True
        nav.estimate.searching = True
        self.assertFalse(observe(nav, 2.2))
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        self.assertFalse(nav.update(status(), 2.8, (999, 2.8, image)))
        self.assertFalse(observe(nav, 3.4))
        self.assertTrue(observe(nav, 4.0))
        nav.estimate.searching = True
        nav.estimate.reacquired_since = nav.estimate.last_confirmation = None
        for now in (5., 5.6, 6.2):
            self.assertFalse(nav.update(status(), now, (1001, 4., image)))
        self.assertTrue(nav.estimate.searching)

    def test_confirmed_startup_heading_reaches_ramp_steering(self):
        walking = Walking(exit_clearance=True)
        walking.begin(status(), 8, 0., 'course')
        nav = walking.navigation
        nav.update(status(), 2., None)
        for now in (2.2, 2.35, 2.5, 2.65, 2.85):
            observe(nav, now, heading=14)
        self.assertFalse(nav.initial_pose_pending)
        walking.last_confirmed_frame = 2.9
        frame = walking.frame(status(), 2.9)
        self.assertAlmostEqual(frame['heading'], math.radians(14), delta=.01)


if __name__ == '__main__':
    unittest.main()
