"""Ramp offset corrections retain direction, limits, and MCU trajectory parity."""
import math
import os
import subprocess
import time
import unittest
from unittest.mock import patch

import numpy as np

from robot_b91.gait import ramp_forward_scale, rocker_targets
from robot_dashboard.devices import DeviceError
from robot_dashboard.runtime import Dashboard
from robot_dashboard.walking import Walking
from test_ramp_alignment import status


class RampDriftTests(unittest.TestCase):
    def test_forward_stride_is_symmetric_bounded_and_reduces_smoothly(self):
        offsets = np.linspace(0, .10, 1000)
        forward = np.array([ramp_forward_scale(x) for x in offsets])
        self.assertTrue(np.all(np.diff(forward) <= 1e-12))
        self.assertGreaterEqual(forward.min(), .5)
        self.assertLessEqual(forward.max(), 1)
        self.assertEqual(ramp_forward_scale(.010), 1)
        self.assertAlmostEqual(ramp_forward_scale(.03), .5)
        np.testing.assert_allclose(forward, [ramp_forward_scale(-x) for x in offsets])
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                ramp_forward_scale(value)

    def test_commands_preserve_heading_and_strafe_sign_with_less_forward_motion(self):
        walking = Walking(ramp_drift_control=True)
        settings = walking.start_settings('course')
        self.assertEqual(settings['profile'], 'corridor_search')
        self.assertEqual(settings['turn_limit'], .2)
        self.assertEqual(settings['lateral_gain'], 0)
        self.assertEqual(settings['strafe_gain'], 12)
        walking.begin(status(), 8, 0., 'course')
        walking.set_speed(.5)
        for lateral in (-.05, 0, .05):
            walking.navigation.estimate.heading = .02
            walking.navigation.estimate.lateral = lateral
            walking.last_confirmed_frame = 3
            with patch.object(walking.navigation, 'update', return_value=True):
                frame = walking.frame(status(), 3)
            self.assertEqual(frame['heading'], .02)
            self.assertEqual(frame['lateral'], lateral)
            self.assertEqual(frame['speed'], .5)
            self.assertEqual(frame['ready'], 1)
            self.assertAlmostEqual(frame['forward'], .5 if lateral else 1)
        with patch.object(walking.navigation, 'update', return_value=False):
            self.assertEqual(walking.frame(status(), 3)['ready'], 0)

    def test_option_preserves_gait_phases_and_does_not_change_other_profiles(self):
        walking = Walking(ramp_drift_control=True)
        approach = walking.start_settings('course')
        walking.begin(status(), 8, 0., 'course')
        for phase in ('crest', 'descent'):
            walking.crest_sequence.requested = phase
            params, settings = walking.transition_settings()
            self.assertEqual(settings['profile'], 'corridor_search')
            self.assertEqual(settings['strafe_gain'], 12)
            self.assertEqual(settings['turn_limit'], .2)
            self.assertEqual(settings['pitch_stance'], 0 if phase == 'crest' else -.2)
        self.assertEqual(approach['heading_gain'], 1.2)
        self.assertEqual(Walking().start_settings('course')['profile'], 'course')
        self.assertEqual(walking.start_settings('course_camera')['profile'], 'course_camera')

    def test_missing_capability_refused_before_any_motion(self):
        app = Dashboard(ramp_drift_control=True, auto_corridor=False)
        app.browser_heartbeat = time.monotonic()
        old = status(externalActive=False, corridorSearchGait=False)
        calls = []
        def request(url, path, *args, **kwargs):
            calls.append(path)
            return old if path == '/status' else {}
        with patch('robot_dashboard.runtime.devices.request', side_effect=request):
            with self.assertRaisesRegex(DeviceError, 'variable-forward'):
                app.walk_command('course')
        self.assertNotIn('/motion/start', calls)

    def test_restart_holds_and_exit_clearance_also_include_required_forward_field(self):
        walking = Walking(ramp_drift_control=True)
        walking.start_settings('course')
        walking.begin(status(), 8, 0., 'course')
        walking.crest_sequence.requested = 'crest'
        params, _ = walking.transition_settings()
        walking.resume_gait(8, 10., params)
        held = walking.frame(status(), 10.1)
        self.assertEqual(held['ready'], 0)
        self.assertEqual(held['forward'], 0)
        walking.resume_until = None
        walking.exit_cycles = 1.125
        walking.exit_started = walking.exit_updated = 12.
        walking.last_confirmed_frame = 12.1
        walking.navigation.estimate.heading = 0.
        frame = walking.frame(status(), 12.1)
        self.assertEqual(frame['ready'], 1)
        self.assertEqual(frame['forward'], 1)
        self.assertEqual(frame['lateral'], 0)

    @unittest.skipUnless(os.environ.get('CREST_GAIT_TEST_EXE'), 'Compiled MCU gait harness required')
    def test_variable_forward_trajectories_match_installed_firmware_math(self):
        rng = np.random.default_rng(54)
        walking = Walking(ramp_drift_control=True)
        walking.start_settings('course')
        lines, expected = [], []
        for pattern in ('trot', 'crest'):
            for _ in range(100):
                p = walking.params | dict(pattern=pattern, duty=.85, search=True, corridor=True,
                                          yaw_bias=-.25 if pattern == 'crest' else 0)
                elapsed, phase, heading, lateral = rng.uniform([0,0,-.2,-.06],[10,15,.2,.06])
                forward = ramp_forward_scale(lateral)
                keys = ('period','duty','yaw_amplitude','pitch_lift','pitch_stance','yaw_bias')
                values = [p[k] for k in keys] + [math.radians(-125),p['heading_gain'],
                    p['lateral_gain'],p['strafe_gain'],elapsed,phase,heading,lateral,.2,1,forward]
                lines.append(('ramp_crest' if pattern == 'crest' else 'ramp')+' '+' '.join(map(str,values)))
                expected.append(rocker_targets(p,elapsed,phase,heading,lateral,forward_scale=forward))
        result = subprocess.run([os.environ['CREST_GAIT_TEST_EXE']], input='\n'.join(lines)+'\n',
                                text=True,capture_output=True,check=True)
        actual = np.array([np.fromstring(row,sep=' ') for row in result.stdout.splitlines()])
        np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=1e-12)


if __name__ == '__main__':
    unittest.main()
