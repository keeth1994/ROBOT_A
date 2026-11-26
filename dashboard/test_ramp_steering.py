"""Steering limits after a reported hard-turn fall; no hardware commands."""
import math
import time
import unittest
from unittest.mock import patch

import numpy as np

from robot_b91.gait import RAMP_STEERING, rocker_targets
from robot_b91.run import load_gait_parameters
from robot_dashboard.devices import DeviceError
from robot_dashboard.runtime import Dashboard
from robot_dashboard.walking import Walking
from test_crest_sequence import status


class RampSteeringTests(unittest.TestCase):
    def test_offset_does_not_add_turn_and_yaw_differential_is_bounded(self):
        p = load_gait_parameters() | RAMP_STEERING | dict(yaw_bias=0, pattern='trot', yaw_amplitude=.4)
        # At t=0, the FL and RR diagonal have identical phase. Their sum
        # cancels forward and strafe components, isolating the turn command.
        for gain in (1.2, 3.0):
            for heading in (-1., -.1, 0., .1, 1.):
                for lateral in (-.05, 0., .05):
                    target = rocker_targets(p | {'heading_gain': gain}, 3, 0, heading, lateral)
                    turn = -(target[0]+target[6])/(2*p['yaw_amplitude'])
                    self.assertAlmostEqual(turn, float(np.clip(gain*heading,-.2,.2)))
                    self.assertLessEqual(abs(turn), .2+1e-12)
        forward = rocker_targets(p,3,0,0,0)
        shifted = rocker_targets(p,3,0,0,.02)
        np.testing.assert_array_equal(forward, shifted)

    def test_settings_survive_crest_switch_and_do_not_change_other_modes(self):
        walking = Walking(crest_wave=True)
        settings = walking.start_settings('course')
        self.assertEqual(settings['turn_limit'],.2)
        self.assertEqual(settings['lateral_gain'],0)
        self.assertEqual(settings['heading_gain'],1.2)
        walking.begin(status(),1,0,'course')
        for phase in ('crest','descent'):
            walking.crest_sequence.requested = phase
            params, settings = walking.transition_settings()
            self.assertEqual(settings['turn_limit'],.2)
            self.assertEqual(settings['heading_gain'],1.2)
        self.assertNotIn('turn_limit',walking.start_settings('walk'))
        self.assertEqual(walking.start_settings('course_camera')['heading_gain'], 1.2)

    def test_ramp_strokes_ignore_lateral_offset_through_crest_and_descent(self):
        walking = Walking()
        for profile in ('course', 'course_camera'):
            settings = walking.start_settings(profile)
            walking.begin(status(), 1, 0, profile)
            phases = ('approach', 'crest', 'descent') if profile == 'course' else ('approach',)
            for phase in phases:
                if phase != 'approach':
                    walking.crest_sequence.requested = phase
                    params, settings = walking.transition_settings()
                    walking.resume_gait(2, 3, params)
                self.assertEqual(settings['strafe_gain'], 0)
                self.assertEqual(settings['lateral_gain'], 0)
                for t in np.linspace(0, 2, 41):
                    for offset in (-.1, .02, .1):
                        np.testing.assert_array_equal(
                            rocker_targets(settings, 4, t, .08, offset),
                            rocker_targets(settings, 4, t, .08, 0))

    def test_side_step_profiles_are_not_available(self):
        for profile in ('sidestep_left', 'sidestep_right'):
            with self.assertRaisesRegex(ValueError, 'Unknown walking profile'):
                Walking().start_settings(profile)

    def test_old_firmware_cannot_silently_ignore_the_cap(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        with patch('robot_dashboard.runtime.devices.request',return_value=status() | {
                'boundedSteering':False,'externalActive':False,'crestWaveGait':True}) as request:
            with self.assertRaisesRegex(DeviceError,'Gentle ramp steering'):
                app.walk_command('course')
        self.assertNotIn('/motion/start',[call.args[1] for call in request.call_args_list])

    def test_large_heading_error_keeps_ramp_steering_active(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(),1,0,'course')
        walking.navigation.estimate.heading = math.radians(13)
        walking.last_confirmed_frame = 3
        with patch.object(walking.navigation,'update',return_value=True):
            frame = walking.frame(status(),3)
        self.assertTrue(walking.active)
        self.assertEqual(frame['ready'], 1)
        self.assertAlmostEqual(frame['heading'], math.radians(13))


if __name__ == '__main__':
    unittest.main()
