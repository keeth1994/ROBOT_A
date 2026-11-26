"""Disabling the crest gait keeps normal walking and ramp camera navigation."""
import math
import time
import unittest
from unittest.mock import patch

from dashboard.robot_dashboard.runtime import Dashboard
from dashboard.robot_dashboard.walking import Walking
from dashboard.test_crest_sequence import status


class NoCrestTests(unittest.TestCase):
    def test_default_still_enables_crest_sequence(self):
        walking = Walking()
        walking.begin(status(), 1, 100, 'course')
        self.assertIsNotNone(walking.crest_sequence)
        self.assertTrue(walking.state()['crest_enabled'])

    def test_disabled_crest_keeps_normal_gait_through_pitch_changes(self):
        walking = Walking(crest_enabled=False)
        self.assertEqual(walking.start_settings('course'), Walking().start_settings('course'))
        walking.begin(status(), 1, 100, 'course')
        with patch.object(walking.navigation, 'update', return_value=True):
            for i in range(40):
                now = 102 + i * .1
                walking.last_confirmed_frame = now
                walking.navigation.estimate.pitch = math.radians(-20 if i < 20 else 0)
                self.assertEqual(walking.frame(status(), now)['ready'], 1)
        self.assertIsNone(walking.crest_sequence)
        self.assertIsNone(walking.resume_until)
        self.assertFalse(walking.state()['crest_enabled'])

    def test_disabled_crest_preserves_second_climb_camera_and_offset(self):
        walking = Walking(crest_enabled=False)
        walking.begin(status(), 1, 100, 'course')
        for start, pitch in ((102, -15), (103, 15), (104, -15)):
            for i in range(8):
                walking.navigation.observe_slope(start + i * .1, math.radians(pitch))
        self.assertEqual(walking.navigation.estimate.climbs_seen, 2)
        self.assertEqual(walking.camera_target(110), -4)
        enabled = Walking()
        enabled.begin(status(), 1, 100, 'course')
        self.assertAlmostEqual(walking.navigation.estimate.lateral_error,
                               enabled.navigation.estimate.lateral_error)

    def test_disabled_crest_does_not_require_crest_firmware_capabilities(self):
        app = Dashboard(crest_enabled=False)
        app.browser_heartbeat = time.monotonic()
        old = status() | dict(externalActive=False)
        for key in ('crestWaveGait', 'signedYawBias', 'rearStrideScale', 'frontPitchLift'):
            old.pop(key)
        with patch('dashboard.robot_dashboard.runtime.devices.request',
                   side_effect=[old, dict(session=2)]) as request, \
                patch.object(app, '_aim_camera'):
            app.walk_command('course')
        self.assertEqual([call.args[1] for call in request.call_args_list], ['/status', '/motion/start'])
        self.assertIsNone(app.walking.crest_sequence)
        self.assertTrue(app.walking.active)


if __name__ == '__main__':
    unittest.main()
