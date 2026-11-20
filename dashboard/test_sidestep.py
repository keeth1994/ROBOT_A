import math
import time
import unittest
from unittest.mock import patch

import numpy as np

from robot_b91.gait import rocker_targets
from robot_b91.run import load_gait_parameters
from robot_dashboard.devices import DeviceError
from robot_dashboard.runtime import Dashboard
from robot_dashboard.sidestep import SIDESTEP_GAIT, check_sidestep
from robot_dashboard.walking import JOINTS, Walking


def status(**changes):
    return dict(servoReady=True, faulted=False, motionProtocol=2, sidestepGait=True,
        sidestepComplete=False, externalActive=True, motionSession=8, gaitStopping=False,
        orientationValid=True, imuAgeMs=4, orientation=[0.,0.,0.],
        joints=[dict(name=n,channel=i) for i,n in enumerate(JOINTS)]) | changes


class SidestepTests(unittest.TestCase):
    def test_lateral_command_has_no_forward_stride_or_position_turn(self):
        p = load_gait_parameters() | SIDESTEP_GAIT | {'sideways': True}
        for t in np.linspace(0, 2, 100):
            left = rocker_targets(p, 3, t, sideways_scale=1)
            right = rocker_targets(p, 3, t, sideways_scale=-1)
            np.testing.assert_allclose(left[::2], -right[::2], atol=1e-12)
            np.testing.assert_allclose(left[1::2], right[1::2], atol=1e-12)
            zero = rocker_targets(p, 3, t, sideways_scale=0)
            np.testing.assert_allclose(zero[::2], 0, atol=1e-12)
            # Even nonzero legacy gains and lateral error must not turn this mode.
            shifted = rocker_targets(p | {'lateral_gain': 20, 'strafe_gain': 20},
                                      3, t, lateral_error_m=.2, sideways_scale=1)
            np.testing.assert_allclose(shifted, left, atol=1e-12)
        self.assertFalse(np.allclose(rocker_targets(p,3,.3,heading_error_rad=.1,sideways_scale=1),
                                    rocker_targets(p,3,.3,sideways_scale=1)))

    def test_profiles_send_explicit_direction_and_hold_heading(self):
        for profile, direction in [('sidestep_left',1),('sidestep_right',-1)]:
            walking = Walking(crest_wave=True)
            settings = walking.start_settings(profile)
            self.assertEqual(settings['profile'], 'sidestep')
            self.assertEqual(settings['sideways'], direction)
            self.assertEqual(settings['lateral_gain'], 0)
            self.assertEqual(settings['period'], 2)
            walking.set_speed(.7)
            walking.begin(status(), 8, 0, profile)
            self.assertIsNone(walking.crest_sequence)
            walking.last_confirmed_frame = 2.1
            frame = walking.frame(status(orientation=[0.,0.,.02]), 2.1)
            self.assertEqual(frame['speed'], .65)
            self.assertAlmostEqual(frame['heading'], .02)
            self.assertEqual(frame['lateral'], 0)
            walking.set_speed(1.)
            self.assertEqual(walking.state()['effective_speed'], .65)
            walking.set_speed(.4)
            self.assertEqual(walking.state()['effective_speed'], .4)

    def test_old_firmware_refused_before_motion(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        with patch('robot_dashboard.runtime.devices.request', return_value=status(
                sidestepGait=False,externalActive=False)) as request:
            with self.assertRaisesRegex(DeviceError, 'updated controller firmware'):
                app.walk_command('sidestep_left')
        self.assertNotIn('/motion/start', [call.args[1] for call in request.call_args_list])

    def test_freshness_tilt_heading_and_duration_guards(self):
        for invalid in [dict(imuAgeMs=51),dict(imuAgeMs=float('nan')),
                        dict(orientation=[0.,math.radians(9),0.]),
                        dict(orientation=[float('nan'),0.,0.])]:
            with self.assertRaises(DeviceError):
                check_sidestep(status(**invalid))
        for now, attitude in [(2.1,[0.,0.,math.radians(11)]),(12.1,[0.,0.,0.])]:
            walking = Walking()
            walking.start_settings('sidestep_left')
            walking.begin(status(), 8, 0, 'sidestep_left')
            walking.last_confirmed_frame = now
            with self.assertRaises(DeviceError):
                walking.frame(status(orientation=attitude),now)

    def test_completion_stops_without_another_adjustment(self):
        for active, session in [(True,8),(False,0)]:
            walking = Walking()
            walking.start_settings('sidestep_left')
            walking.begin(status(), 8, 0, 'sidestep_left')
            walking.last_confirmed_frame = 5.1
            frame = walking.frame(status(sidestepComplete=True,externalActive=active,
                                          motionSession=session),5.1)
            self.assertEqual(frame,{})
            self.assertFalse(walking.active)
            self.assertIn('complete',walking.message)

    def test_completion_between_status_and_adjustment_is_confirmed_not_retried(self):
        for active, session in ((True, 8), (False, 0)):
            app = Dashboard()
            app.walking.start_settings('sidestep_right')
            app.walking.begin(status(), 8, 0, 'sidestep_right')
            completed = status(sidestepComplete=True, externalActive=active, motionSession=session)
            with patch('robot_dashboard.runtime.devices.request', side_effect=[
                    DeviceError('Inactive, stale or mismatched walking session.', 409), completed]) as request:
                app._adjust_walking('http://robot.test', {'session': 8})
            self.assertEqual([c.args[1] for c in request.call_args_list], ['/motion/adjust', '/status'])
            self.assertFalse(app.walking.active)
            self.assertIn('complete', app.walking.message)

    def test_unconfirmed_or_unrelated_step_does_not_hide_adjustment_error(self):
        for changes in (dict(sidestepComplete=False), dict(motionSession=9),
                        dict(manualActive=True), dict(running=True)):
            app = Dashboard()
            app.walking.start_settings('sidestep_right')
            app.walking.begin(status(), 8, 0, 'sidestep_right')
            completed = status(sidestepComplete=True) | changes
            with patch('robot_dashboard.runtime.devices.request', side_effect=[
                    DeviceError('session mismatch', 409), completed]) as request:
                with self.assertRaisesRegex(DeviceError, 'session mismatch'):
                    app._adjust_walking('http://robot.test', {'session': 8})
            self.assertEqual(request.call_count, 2)
            self.assertTrue(app.walking.active)


if __name__ == '__main__':
    unittest.main()
