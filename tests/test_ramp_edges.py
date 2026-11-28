"""Test ramp edge detection."""
import math
import unittest

import numpy as np

from robot.navigation.controller import Navigation
from robot.navigation.debug import navigation_debug_image
from robot.navigation.ramp_detection import detect_ramp_edges


def ramp_frame(camera_pitch=30, heading=0, offset=0, grade=0, body_pitch=0, roll=0, far_grade=None):
    h, w = 240, 320
    y, x = np.mgrid[:h, :w]
    focal = h / (2 * math.tan(math.radians(42.9470936) / 2))
    rays = np.stack([np.ones_like(x), -(x - (w - 1) / 2) / focal,
                     -(y - (h - 1) / 2) / focal], axis=-1)
    def pitch_rotation(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    c, s = math.cos(heading), math.sin(heading)
    yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    c, s = math.cos(roll), math.sin(roll)
    bank = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    rotation = yaw @ pitch_rotation(body_pitch) @ bank @ pitch_rotation(math.radians(camera_pitch))
    rays = rays @ rotation.T
    denominator = rays[:, :, 2] - grade * rays[:, :, 0]
    distance = -.24 / np.where(abs(denominator) > 1e-8, denominator, 1e-8)
    points = rays * distance[:, :, None]
    surface = (distance > 0) & (abs(points[:, :, 1] + offset) < .145)
    if far_grade is not None:
        surface &= points[:, :, 0] < .45
        denominator = rays[:, :, 2] - far_grade * rays[:, :, 0]
        distance = ((grade - far_grade) * .45 - .24) / np.where(abs(denominator) > 1e-8, denominator, 1e-8)
        points = rays * distance[:, :, None]
        surface |= (distance > 0) & (points[:, :, 0] >= .45) & (abs(points[:, :, 1] + offset) < .145)
    return np.repeat(np.where(surface, 80, 190)[:, :, None], 3, axis=2).astype(np.uint8)


class RampEdgeTests(unittest.TestCase):
    def navigation(self, pitch=30):
        nav = Navigation(width_m=.29, adaptive_surface=True, camera_pitch_deg=pitch,
                         camera_x_m=0, ramp_offset_right_m=.03)
        nav.view_sign = 1 if pitch < 90 else -1
        return nav

    def test_clipped_side_is_missing_rather_than_an_image_border(self):
        for offset in (-.09, .09):
            image = ramp_frame(camera_pitch=60, offset=offset)
            _, rows = detect_ramp_edges(image)
            self.assertGreater(len(rows), 100)
            self.assertTrue(all((left is None) != (right is None) for _, left, right in rows))
            self.assertTrue(all(x is None or 2 < x < 317 for _, l, r in rows for x in (l, r)))

    def test_single_edge_corrects_heading_without_changing_measured_position(self):
        for rear in (False, True):
            for offset in (-.09, .09):
                for heading in (-.08, .08):
                    with self.subTest(rear=rear, offset=offset, heading=heading):
                        pitch = 150 if rear else 30
                        nav = self.navigation(pitch)
                        for t in range(10):
                            self.assertTrue(nav.frame(t, ramp_frame(pitch, offset=offset), False))
                        lateral = nav.lateral
                        nav.camera_pitch = math.radians(120 if rear else 60)
                        image = ramp_frame(120 if rear else 60, heading, offset)
                        for t in range(10, 30):
                            self.assertTrue(nav.frame(t, image, False), nav.debug['reason'])
                        self.assertAlmostEqual(nav.heading, heading, delta=.012)
                        self.assertAlmostEqual(nav.lateral, lateral)
                        self.assertEqual(nav.debug['lateral_source'], 'held')
                        self.assertEqual(nav.debug['inliers'], [])
                        self.assertIn(nav.debug['edge_source'], ('left', 'right'))
                        self.assertEqual(navigation_debug_image(nav, 29, False).shape, (318, 640, 3))

    def test_inclined_ramp_learns_grade_before_using_one_edge(self):
        for grade in (-.25, .25):
            nav = self.navigation()
            for t in range(10):
                self.assertTrue(nav.frame(t, ramp_frame(offset=.09, grade=grade), False))
            self.assertAlmostEqual(nav.ramp_slopes[1], grade, delta=.02)
            nav.camera_pitch = math.radians(60)
            for t in range(10, 30):
                self.assertTrue(nav.frame(t, ramp_frame(60, .08, .09, grade), False))
            self.assertAlmostEqual(nav.heading, .08, delta=.02)

    def test_body_tilt_does_not_turn_a_straight_ramp_into_a_heading_error(self):
        nav = self.navigation()
        nav.pitch, nav.roll = -.15, .08
        image = ramp_frame(30, .06, .03, .15, body_pitch=nav.pitch, roll=nav.roll)
        for t in range(10):
            self.assertTrue(nav.frame(t, image), nav.debug['reason'])
        self.assertAlmostEqual(nav.heading, .06, delta=.015)
        self.assertAlmostEqual(nav.lateral, .03, delta=.008)

    def test_different_slopes_share_the_same_heading_and_center(self):
        nav = self.navigation()
        image = ramp_frame(30, .08, -.025, .1, far_grade=-.2)
        for t in range(10):
            self.assertTrue(nav.frame(t, image, False), nav.debug['reason'])
        self.assertEqual(nav.debug['edge_source'], 'paired')
        self.assertAlmostEqual(nav.heading, .08, delta=.015)
        self.assertAlmostEqual(nav.lateral, -.025, delta=.008)

    def test_single_edge_can_start_using_imu_slope(self):
        nav = self.navigation(60)
        nav.pitch = -math.atan(.25)
        image = ramp_frame(60, .08, .09, .25, body_pitch=nav.pitch)
        for t in range(20):
            self.assertTrue(nav.frame(t, image), nav.debug['reason'])
        self.assertAlmostEqual(nav.heading, .08, delta=.02)
        self.assertEqual(nav.debug['grade_source'], 'body_pitch')

    def test_single_edge_handles_a_slope_change_after_a_paired_view(self):
        nav = self.navigation()
        for t in range(10):
            self.assertTrue(nav.frame(t, ramp_frame(offset=.09)))
        nav.camera_pitch = math.radians(60)
        nav.pitch = -math.atan(.25)
        image = ramp_frame(60, .08, .09, .25, body_pitch=nav.pitch)
        for t in range(10, 30):
            self.assertTrue(nav.frame(t, image), nav.debug['reason'])
        self.assertAlmostEqual(nav.heading, .08, delta=.02)
        self.assertEqual(nav.debug['grade_source'], 'paired_edges_and_pitch')

    def test_uniform_surface_does_not_invent_a_side(self):
        nav = self.navigation()
        self.assertFalse(nav.frame(2, np.full((240, 320, 3), 90, dtype=np.uint8)))
        self.assertEqual(nav.debug['edge_source'], 'none')


if __name__ == '__main__':
    unittest.main()
