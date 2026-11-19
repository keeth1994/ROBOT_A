"""Hardware-free checks for the boundary between ramp and corridor trials."""
import unittest
import math
from unittest.mock import Mock, patch

from robot_dashboard.runtime import Dashboard
from robot_b91.gait import rocker_targets
from robot_b91.camera_scan import RampScanNavigation
from robot_b91.camera_calibration import load_camera_scan_settings
from robot_b91.navigation_debug import navigation_status


class RampOnlyTests(unittest.TestCase):
    def test_debug_uses_control_decision_time_but_flags_stale_snapshots(self):
        nav = RampScanNavigation(load_camera_scan_settings())
        nav.last_imu = 3.99
        nav.last_seen = 3.98
        nav.debug = {'accepted': True, 'timestamp': 3.98, 'reason': 'Ramp found'}
        self.assertEqual(navigation_status(nav, 4.1, decision_time=4)[0], 'VALID CAMERA')
        self.assertEqual(navigation_status(nav, 4.5, decision_time=4)[0], 'STALE CONTROL SNAPSHOT')
        nav.last_imu = 3.8
        self.assertEqual(navigation_status(nav, 4.1, decision_time=4)[0], 'HOLD REQUIRED')

    def test_crawl_has_only_one_swing_leg_and_preserves_swing_duration(self):
        import numpy as np
        params = Dashboard(auto_corridor=False).walking.params | {
            'period': 1.6, 'duty': .75, 'pattern': 'crawl', 'pitch_stance': 0, 'pitch_lift': .9}
        targets = np.array([rocker_targets(params, 5, t) for t in np.linspace(0, 3.2, 401)])
        self.assertTrue(np.all(np.count_nonzero(targets[:, 1::2] < -.001, axis=1) <= 1))
        self.assertAlmostEqual(params['period'] * (1 - params['duty']), .4)

    def test_raised_stance_preserves_lift_endpoint_and_servo_limits(self):
        import numpy as np
        params = Dashboard(auto_corridor=False).walking.params | {'pitch_stance': 0, 'pitch_lift': .9}
        targets = np.array([rocker_targets(params, 5, t) for t in np.linspace(0, 2, 201)])
        self.assertAlmostEqual(float(targets[:, 1::2].max()), 0)
        self.assertAlmostEqual(float(targets[:, 1::2].min()), -.9)
        physical_pitch = np.degrees(targets[:, 1::2]) + 45
        self.assertTrue(np.all(physical_pitch <= 45))
        self.assertTrue(np.all(physical_pitch >= -90))
        self.assertTrue(np.all(np.abs(targets[:, ::2]) <= math.pi / 4))

    def test_ramp_exit_stops_without_starting_corridor(self):
        app = Dashboard(auto_corridor=False)
        app.walking.active = True
        with patch('robot_dashboard.runtime.devices.request', return_value={}) as request:
            app._corridor_after_ramp('http://robot.test')
        request.assert_called_once_with('http://robot.test', '/stop', {}, timeout=.15)
        self.assertFalse(app.walking.active)
        self.assertIn('verify completion visually', app.walking.message)

    def test_failed_stop_cannot_start_corridor(self):
        app = Dashboard(auto_corridor=False)
        app.walking.start_settings = Mock()
        with patch('robot_dashboard.runtime.devices.request', side_effect=OSError('offline')):
            with self.assertRaises(OSError):
                app._corridor_after_ramp('http://robot.test')
        app.walking.start_settings.assert_not_called()


if __name__ == '__main__':
    unittest.main()
