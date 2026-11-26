"""Stopped camera diagnostics use IMU compensation without commanding motion."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from robot_dashboard.devices import DeviceConnectionError
from robot_dashboard.navigation import CourseNavigation
from robot_dashboard.runtime import Dashboard
from robot_b91.navigation_debug import navigation_status
from test_camera_body_offset import ramp_image


class DebugPreviewTests(unittest.TestCase):
    def setUp(self):
        self.app = Dashboard()
        self.app.debug_preview = CourseNavigation(0)
        self.status = dict(orientationValid=True, imuAgeMs=2,
                           orientation=[math.radians(6), math.radians(8), 0],
                           cameraPitch=dict(angle=-65, moving=False))
        self.app.debug_preview.update(self.status, .1, None)
        estimate = self.app.debug_preview.estimate
        lens = np.array([estimate.camera_x, estimate.camera_y, estimate.camera_z])
        rgb = ramp_image(*self.status['orientation'], .02, lens,
                         math.degrees(estimate.camera_pitch))
        self.frame = (1, 3, rgb)

    def render(self, **request_args):
        with patch('robot_dashboard.runtime.time.monotonic', return_value=3):
            with patch.object(self.app.camera, 'navigation_frame', return_value=self.frame):
                with patch('robot_dashboard.runtime.devices.request', **request_args) as request:
                    with patch('robot_dashboard.runtime.navigation_debug_image',
                               return_value=np.zeros((4, 4, 3), dtype=np.uint8)) as draw:
                        jpeg = self.app.navigation_debug_frame()
        self.assertTrue(jpeg.startswith(b'\xff\xd8'))
        self.assertTrue(draw.call_args.kwargs['use_imu'])
        return draw.call_args.args[0], request

    def test_preview_matches_ramp_perception_with_body_tilt(self):
        ramp = CourseNavigation(0, scan_ramp=True)
        ramp.update(self.status, .1, None)
        ramp.update(self.status, 3, self.frame)
        preview, request = self.render(return_value=self.status)
        request.assert_called_once_with(self.app.config['servo'], '/status', timeout=.15)
        self.assertFalse(self.app.walking.active)
        self.assertTrue(preview.debug['accepted'])
        self.assertAlmostEqual(preview.lateral, ramp.estimate.lateral)
        self.assertAlmostEqual(preview.heading, ramp.estimate.heading)
        self.assertAlmostEqual(preview.roll, self.status['orientation'][0])
        self.assertAlmostEqual(preview.pitch, self.status['orientation'][1])
        self.assertAlmostEqual(preview.lateral/.3, .02, delta=.004)

    def test_stale_imu_holds_preview_even_if_cached_status_is_fresh(self):
        self.app.cache['servo']['data'] = self.status
        preview, request = self.render(return_value=self.status | {'imuAgeMs': 200})
        request.assert_called_once()
        self.assertFalse(preview.debug['accepted'])
        self.assertTrue(preview.debug['held'])
        self.assertEqual(navigation_status(preview, 3)[0], 'HOLD REQUIRED')
        self.assertFalse(self.app.walking.active)

    def test_unavailable_controller_holds_preview(self):
        preview, request = self.render(side_effect=DeviceConnectionError('Controller unavailable'))
        request.assert_called_once()
        self.assertTrue(preview.debug['held'])
        self.assertFalse(preview.debug['accepted'])
        self.assertFalse(self.app.walking.active)

    def test_live_debug_uses_controller_estimate_without_extra_status_request(self):
        ramp = CourseNavigation(0, scan_ramp=True)
        ramp.update(self.status, .1, None)
        ramp.update(self.status, 3, self.frame)
        self.app.walking.navigation = ramp
        self.app.walking.active = True
        self.app.walking.profile = 'course'
        preview, request = self.render()
        request.assert_not_called()
        self.assertAlmostEqual(preview.lateral, ramp.estimate.lateral)
        self.assertTrue(self.app.walking.active)


if __name__ == '__main__':
    unittest.main()
