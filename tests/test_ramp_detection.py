"""Check ramp edges against known outlines, clutter and changes in brightness."""
import math
from pathlib import Path
import unittest

import numpy as np
from PIL import Image, ImageDraw

from robot.navigation.ramp_detection import detect_ramp
from robot.navigation.controller import Navigation
from robot.navigation.camera_scan import RampScanNavigation
from tests.test_camera_scan import BACKWARD, CONFIG, SETTINGS, ramp_image


FIXTURES = Path(__file__).parent / 'fixtures' / 'ramp'


def ramp_scene(brightness=90, near_border=False, supports=False):
    image = Image.new('RGB', (640, 480), (190, 190, 190))
    draw = ImageDraw.Draw(image)
    outline = [(10, 20), (629, 20), (629, 440), (10, 440)] if near_border else [
        (230, 50), (410, 50), (590, 440), (50, 440)]
    draw.polygon(outline, fill=(brightness,) * 3)
    if supports:
        draw.rectangle((100, 290, 180, 320), fill=(brightness,) * 3)
        draw.rectangle((460, 290, 540, 320), fill=(brightness,) * 3)
    # Screws and a foreground robot obscure parts of the ramp.
    for y in (100, 180, 260):
        draw.ellipse((300, y, 304, y + 4), fill='white')
    draw.rectangle((200, 410, 440, 479), fill=(180, 30, 50))
    return np.asarray(image)


class RampDetectionTests(unittest.TestCase):
    def test_perspective_edges_remain_centered_across_brightness_and_supports(self):
        for brightness in (45, 90, 140):
            for supports in (False, True):
                with self.subTest(brightness=brightness, supports=supports):
                    mask, rows = detect_ramp(ramp_scene(brightness, supports=supports))
                    self.assertTrue(mask.any())
                    self.assertGreater(len(rows), 20)
                    for y, left, right in rows:
                        expected_left = 230 - (y - 50) * 180 / 390
                        expected_right = 410 + (y - 50) * 180 / 390
                        self.assertAlmostEqual(left, expected_left, delta=5)
                        self.assertAlmostEqual(right, expected_right, delta=5)

    def test_visible_edges_close_to_borders_are_kept(self):
        _, rows = detect_ramp(ramp_scene(near_border=True))
        self.assertGreater(len(rows), 20)
        self.assertTrue(all(abs((left + right) / 2 - 319.5) < 2 for _, left, right in rows))

    def test_clipped_surface_does_not_invent_a_center(self):
        image = np.full((480, 640, 3), 90, dtype=np.uint8)
        image[:50] = 190
        _, rows = detect_ramp(image)
        self.assertEqual(rows, [])

    def test_uniform_wall_is_not_a_ramp(self):
        for brightness in (45, 90, 140):
            mask, rows = detect_ramp(np.full((480, 640, 3), brightness, dtype=np.uint8))
            self.assertFalse(mask.any())
            self.assertEqual(rows, [])

    def test_edges_map_back_to_odd_source_dimensions(self):
        image = Image.fromarray(ramp_scene()).resize((601, 451))
        mask, rows = detect_ramp(np.asarray(image))
        self.assertEqual(mask.shape, (451, 601))
        self.assertGreater(len(rows), 20)
        self.assertTrue(all(abs((left + right) / 2 - 300) < 3 for _, left, right in rows))

    def test_forward_and_rear_views_measure_known_side_offsets(self):
        for pitch in (30, 158):
            for offset in (-.03, 0, .03):
                with self.subTest(pitch=pitch, offset=offset):
                    nav = Navigation(**(CONFIG | {
                        'camera_pitch_deg': pitch, 'adaptive_surface': True, 'dark_max': 115}))
                    nav.view_sign = 1 if pitch == 30 else -1
                    image = ramp_image(math.radians(pitch), offset)
                    # Let the existing offset filter settle on a stationary image.
                    for timestamp in range(2, 12):
                        self.assertTrue(nav.frame(timestamp, image, use_imu=False), nav.debug['reason'])
                    self.assertAlmostEqual(nav.lateral, offset, delta=.002)


class RampPhotoTests(unittest.TestCase):
    def photo(self, name, thumbnail=False):
        with Image.open(FIXTURES / (name + '.png')) as source:
            image = source.convert('RGB')
        if thumbnail:
            image.thumbnail((320, 240))
        rgb = np.asarray(image)
        return rgb[::-1, ::-1].copy() if name == 'rear' else rgb

    def test_centers_follow_the_photographed_ramp(self):
        # Approximate midlines marked from the photographs, in original pixels.
        references = {'entrance': (364, -.17), 'rear': (275, 0), 'supports': (190, -.18)}
        for name, (intercept, slope) in references.items():
            for thumbnail in (False, True):
                with self.subTest(name=name, thumbnail=thumbnail):
                    rgb = self.photo(name, thumbnail)
                    mask, rows = detect_ramp(rgb)
                    self.assertTrue(mask.any())
                    self.assertGreaterEqual(len(rows), 12)
                    y, left, right = np.asarray(rows).T.astype(float)
                    y = (y + .5) * 452 / rgb.shape[0] - .5
                    centers = ((left + right) / 2 + .5) * 601 / rgb.shape[1] - .5
                    error = np.abs(centers - (intercept + slope * y))
                    self.assertLess(np.percentile(error, 95), 12)
                    if name == 'rear':
                        # The floor between the supports is above the mat, not its center.
                        self.assertGreater(y.min(), 350)

    def test_close_up_reports_surface_without_using_the_image_border_as_edges(self):
        for thumbnail in (False, True):
            with self.subTest(thumbnail=thumbnail):
                mask, rows = detect_ramp(self.photo('close_up', thumbnail))
                self.assertGreater(mask.mean(), .65)
                self.assertEqual(rows, [])

    def test_photos_reach_navigation_including_the_rotated_rear_view(self):
        for name in ('entrance', 'rear', 'supports', 'close_up'):
            with self.subTest(name=name):
                rgb = self.photo(name, thumbnail=True)
                rear = name == 'rear'
                nav = Navigation(**(CONFIG | {
                    'camera_pitch_deg': 158 if rear else 30,
                    'adaptive_surface': True, 'dark_max': 115}))
                nav.view_sign = -1 if rear else 1
                # Navigation receives the raw rear view and rotates it itself.
                if rear:
                    rgb = rgb[::-1, ::-1].copy()
                accepted = nav.frame(2, rgb, use_imu=False)
                self.assertTrue(accepted, nav.debug['reason'])
                if name == 'close_up':
                    self.assertIn(nav.debug['edge_source'], ('left', 'right'))
                    self.assertEqual(nav.debug['lateral_source'], 'held')
                self.assertTrue(nav.debug['surface_detected'])

    def test_rear_tilted_photo_establishes_the_ramp_heading(self):
        for thumbnail in (False, True):
            for use_imu in (False, True):
                with self.subTest(thumbnail=thumbnail, use_imu=use_imu):
                    rgb = self.photo('rear_tilted', thumbnail)
                    nav = Navigation(**(CONFIG | {
                        'camera_pitch_deg': 158, 'adaptive_surface': True, 'dark_max': 115}))
                    nav.view_sign = -1
                    # A gyro heading of zero need not point along the ramp.
                    nav.heading = .3
                    self.assertTrue(nav.frame(2, rgb, use_imu=use_imu), nav.debug['reason'])
                    rows = np.asarray(nav.debug['rows'])[nav.debug['inliers']]
                    self.assertGreater(len(rows), 30)
                    y, left, right = rows.T.astype(float)
                    y = (y + .5) * 905 / rgb.shape[0] - .5
                    centers = ((left + right) / 2 + .5) * 1200 / rgb.shape[1] - .5
                    # Midline marked visually in the original, upside-down photograph.
                    self.assertLess(np.percentile(abs(centers - (620 + .13 * y)), 95), 30)

    def test_rear_tilted_photo_reacquires_after_heading_drift(self):
        rgb = self.photo('rear_tilted', thumbnail=True)
        nav = RampScanNavigation(SETTINGS, **(CONFIG | {
            'camera_pitch_deg': 40, 'adaptive_surface': True, 'dark_max': 115}))
        nav.last_seen = 2
        nav.accepted = 1
        nav.heading = .3
        angle = math.radians(BACKWARD)
        nav.start_search(3.2, angle, 'backward')
        for timestamp in (5.3, 5.5, 5.61):
            nav.last_imu = timestamp
            self.assertTrue(nav.scan_frame(timestamp, rgb, angle, 0, CONFIG['camera_x_m']),
                            nav.debug['reason'])
        self.assertFalse(nav.searching)
        self.assertTrue(nav.ready(5.61))


if __name__ == '__main__':
    unittest.main()
