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
        for heading in (-1., -.1, 0., .1, 1.):
            for lateral in (-.05, 0., .05):
                target = rocker_targets(p, 3, 0, heading, lateral)
                turn = -(target[0]+target[6])/(2*p['yaw_amplitude'])
                self.assertAlmostEqual(turn, float(np.clip(1.2*heading,-.2,.2)))
                self.assertLessEqual(abs(turn), .2+1e-12)
        forward = rocker_targets(p,3,0,0,0)
        shifted = rocker_targets(p,3,0,0,.02)
        self.assertFalse(np.allclose(forward,shifted)) # Strafe remains active.

    def test_settings_survive_crest_switch_and_do_not_change_other_modes(self):
        walking = Walking(crest_wave=True)
        settings = walking.start_settings('course')
        self.assertEqual(settings['turn_limit'],.2)
        self.assertEqual(settings['lateral_gain'],0)
        walking.begin(status(),1,0,'course')
        for phase in ('crest','descent'):
            walking.crest_sequence.requested = phase
            params, settings = walking.transition_settings()
            self.assertEqual(settings['turn_limit'],.2)
            self.assertEqual(settings['heading_gain'],1.2)
        self.assertNotIn('turn_limit',walking.start_settings('walk'))
        self.assertNotIn('turn_limit',walking.start_settings('sidestep_left'))

    def test_old_firmware_cannot_silently_ignore_the_cap(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        with patch('robot_dashboard.runtime.devices.request',return_value=status() | {
                'boundedSteering':False,'externalActive':False}) as request:
            with self.assertRaisesRegex(DeviceError,'Gentle ramp steering'):
                app.walk_command('course')
        self.assertNotIn('/motion/start',[call.args[1] for call in request.call_args_list])

    def test_large_heading_error_stops_instead_of_requesting_recovery_turn(self):
        walking = Walking()
        walking.start_settings('course')
        walking.begin(status(),1,0,'course')
        walking.navigation.estimate.heading = math.radians(13)
        walking.last_confirmed_frame = 3
        with patch.object(walking.navigation,'update',return_value=True):
            with self.assertRaisesRegex(DeviceError,'12 degrees'):
                walking.frame(status(),3)


if __name__ == '__main__':
    unittest.main()
