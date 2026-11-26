"""Entrance alignment guards, zero-forward commands and confirmed handover."""
import math
import os
import subprocess
import time
import unittest
from unittest.mock import patch

import numpy as np

from robot_b91.gait import rocker_targets
from robot_b91.navigation import Navigation
from robot_dashboard.devices import DeviceError
from robot_dashboard.navigation import CourseNavigation
from robot_dashboard.ramp_alignment import ALIGNMENT_GAIT, RampAlignment, check_alignment
from robot_dashboard.runtime import Dashboard
from robot_dashboard.walking import JOINTS, Walking
from test_camera_body_offset import ramp_image


def status(**changes):
    return dict(servoReady=True, faulted=False, motionProtocol=2, signedYawBias=True,
                corridorSearchGait=True, crestWaveGait=True, rearStrideScale=True, boundedSteering=True,
                externalActive=True, motionSession=8, gaitStopping=False,
                orientationValid=True, imuAgeMs=5, orientation=[0., 0., 0.],
                cameraPitch=dict(angle=-65, moving=False, minAngle=-80, maxAngle=45),
                joints=[dict(name=n, channel=i, angle=45 if n.endswith('pitch') else 0)
                        for i, n in enumerate(JOINTS)]) | changes


def observe(nav, now, *, lateral=0., heading=0., fresh=True, gap=True, state=None):
    nav.estimate.heading, nav.estimate.lateral = heading, lateral
    mask = np.zeros((240, 320), dtype=bool)
    mask[5:130 if gap else 230, 30:290] = True
    nav.estimate.debug = dict(timestamp=now, accepted=fresh, mask=mask)
    with patch.object(CourseNavigation, 'update', return_value=fresh):
        return nav.update(state or status(), now, None)


class RampAlignmentTests(unittest.TestCase):
    def test_fresh_slow_control_updates_can_finish_alignment(self):
        nav = RampAlignment(0)
        for now in (2., 2.6, 3.2, 3.8):
            self.assertFalse(observe(nav, now))
        self.assertEqual(nav.phase, 'verify')
        self.assertFalse(nav.finished)
        for now in (4.4, 5.0):
            observe(nav, now)
        self.assertTrue(nav.finished)

    def test_missing_measurement_clears_slow_alignment_window(self):
        nav = RampAlignment(0)
        for now in (2., 2.38, 2.76):
            observe(nav, now)
        observe(nav, 3.14, fresh=False)
        self.assertEqual(len(nav.samples), 0)
        observe(nav, 3.52)
        self.assertEqual(nav.phase, 'observe')
        self.assertFalse(nav.finished)

    def test_alignment_replaces_biased_heading_using_tilt_corrected_edges(self):
        self.assertTrue(RampAlignment(0).estimate.require_visual_heading)
        lens = np.array([-.013, .015, .244])
        roll, pitch, yaw = np.radians([3, -2, 2])
        nav = Navigation(width_m=.29, camera_pitch_deg=30,
                         camera_x_m=lens[0], camera_y_m=lens[1], camera_z_m=lens[2])
        nav.require_visual_heading = True
        nav.heading_initialized = True
        nav.heading = math.radians(-7)
        nav.roll, nav.pitch = roll, pitch
        self.assertTrue(nav.frame(5, ramp_image(roll, pitch, yaw, -.025, lens)))
        self.assertAlmostEqual(nav.heading, yaw, delta=math.radians(.5))
        self.assertAlmostEqual(nav.lateral/.3, -.025, delta=.004)

    def test_alignment_rejects_edges_that_only_support_gyro_based_offset(self):
        image = np.full((240, 320, 3), 200, dtype=np.uint8)
        image[80:110, 100:220] = 15
        nav = Navigation(camera_pitch_deg=0)
        self.assertTrue(nav.frame(3, image), nav.debug['reason'])
        nav.require_visual_heading = True
        self.assertFalse(nav.frame(3.1, image))
        self.assertIn('Too little edge depth', nav.debug['reason'])

    def test_controller_profile_and_frames_never_command_forward_motion(self):
        w = Walking()
        settings = w.start_settings('ramp_align')
        self.assertEqual(settings['profile'], 'corridor_search')
        self.assertEqual(settings['lateral_gain'], 0)
        self.assertEqual(settings['turn_limit'], .2)
        w.set_speed(.65)
        w.begin(status(), 8, 0, 'ramp_align')
        self.assertIsNone(w.crest_sequence)
        w.last_confirmed_frame = 3
        w.navigation.command_heading, w.navigation.command_lateral = .04, -.03
        with patch.object(w.navigation, 'update', return_value=True):
            frame = w.frame(status(), 3)
        self.assertEqual(frame['forward'], 0)
        self.assertEqual(frame['speed'], .5)
        self.assertEqual(frame['heading'], .04)
        self.assertEqual(frame['lateral'], -.03)
        w.alignment_then_course = True
        w.stop()
        self.assertFalse(w.alignment_then_course)

    def test_misalignment_commands_both_directions_after_observation(self):
        for sign in (-1, 1):
            nav = RampAlignment(0)
            for t in np.arange(.1, 2.7, .1):
                ready = observe(nav, float(t), lateral=.05*sign, heading=math.radians(8)*sign)
                if t < 2:
                    self.assertFalse(ready)
            self.assertTrue(ready)
            self.assertEqual(nav.phase, 'correct')
            self.assertEqual(nav.command_lateral, .05*sign)
            self.assertAlmostEqual(nav.command_heading, math.radians(8)*sign)

    def test_already_aligned_completes_without_stepping(self):
        nav = RampAlignment(0)
        for t in np.arange(.1, 3.2, .1):
            self.assertFalse(observe(nav, float(t), lateral=.005, heading=.01))
        self.assertTrue(nav.finished)

    def test_completion_waits_for_stable_new_measurements_after_outlier(self):
        nav = RampAlignment(0)
        for t in np.arange(.1, 2.8, .1):
            observe(nav, float(t), lateral=.005, heading=.01)
        self.assertFalse(nav.finished)
        observe(nav, 2.9, lateral=.05, heading=math.radians(10))
        self.assertFalse(nav.finished)
        for t in np.arange(3., 4.6, .1):
            observe(nav, float(t), lateral=.005, heading=.01)
        self.assertTrue(nav.finished)
        self.assertAlmostEqual(nav.estimate.lateral, .005)
        self.assertAlmostEqual(nav.estimate.heading, .01)

    def test_success_requires_planting_and_stationary_reverification(self):
        nav = RampAlignment(0)
        for t in np.arange(.1, 3., .1):
            observe(nav, float(t), lateral=.04)
        lifted = status()
        lifted['joints'][1]['angle'] = 30
        for t in np.arange(3., 4.6, .1):
            observe(nav, float(t), state=lifted)
        self.assertFalse(nav.finished)
        self.assertEqual(nav.phase, 'plant')
        self.assertEqual(nav.command_lateral, 0)
        self.assertEqual(nav.requested_gait, 'hold')
        nav.gait_changed('hold', 4.6)
        for t in np.arange(4.6, 8.2, .1):
            observe(nav, float(t))
        self.assertTrue(nav.finished)

    def test_lifted_foot_during_verification_requests_stationary_gait_once(self):
        nav = RampAlignment(0)
        for t in (2., 2.2, 2.4, 2.6):
            observe(nav, t)
        lifted = status()
        lifted['joints'][1]['angle'] = 24.95  # Observed Wi-Fi pause race.
        self.assertFalse(observe(nav, 2.8, state=lifted))
        self.assertEqual(nav.requested_gait, 'hold')
        for t in (3., 3.4, 3.8):
            self.assertFalse(observe(nav, t, state=lifted))
            self.assertEqual(nav.phase, 'plant')
        nav.gait_changed('hold', 4.)
        for t in (6., 6.4, 6.8):
            self.assertFalse(observe(nav, t, state=lifted))
        with self.assertRaisesRegex(DeviceError, 'Stationary alignment stance'):
            observe(nav, 7.2, state=lifted)

    def test_alignment_uses_raw_edge_offset_instead_of_twice_filtered_pose(self):
        nav = RampAlignment(0)
        mask = np.zeros((240, 320), dtype=bool)
        mask[5:130, 30:290] = True
        for now in (2., 2.2, 2.4, 2.6):
            nav.estimate.lateral = 0.  # Previously looked centered at startup.
            nav.estimate.debug = dict(timestamp=now, accepted=True, mask=mask,
                                      measured_heading_rad=0., measured_lateral_m=.03)
            with patch.object(CourseNavigation, 'update', return_value=True):
                ready = nav.update(status(), now, None)
        self.assertTrue(ready)
        self.assertEqual(nav.command_lateral, .03)

    def test_stationary_recheck_can_request_further_correction(self):
        nav = RampAlignment(0)
        nav.gait_changed('hold', 2.)
        for now in (4., 4.4, 4.8, 5.2):
            self.assertFalse(observe(nav, now, heading=math.radians(4)))
        self.assertEqual(nav.requested_gait, 'correct')
        nav.gait_changed('correct', 5.5)
        for now in (7.5, 7.9, 8.3, 8.7):
            ready = observe(nav, now, heading=math.radians(4))
        self.assertTrue(ready)
        self.assertAlmostEqual(nav.command_heading, math.radians(4))

    def test_camera_loss_holds_then_stops_without_blind_correction(self):
        nav = RampAlignment(0)
        for t in np.arange(.1, 2.6, .1):
            observe(nav, float(t), lateral=.04)
        self.assertFalse(observe(nav, 2.7, fresh=False))
        self.assertEqual(nav.command_lateral, 0)
        self.assertFalse(observe(nav, 10., fresh=False))
        with self.assertRaisesRegex(DeviceError, 'lost camera'):
            observe(nav, 17.6, fresh=False)

    def test_floor_visibility_recovers_during_grace_without_blind_steps(self):
        nav = RampAlignment(0)
        for now in (2., 2.6, 3.2, 3.8):
            observe(nav, now, lateral=.04)
        self.assertFalse(observe(nav, 4.4, gap=False))
        self.assertEqual(nav.command_lateral, 0)
        self.assertFalse(observe(nav, 10., gap=False))
        for now in (10.6, 11.2, 11.8, 12.4):
            ready = observe(nav, now, lateral=.04)
        self.assertTrue(ready)

    def test_floor_gap_range_timeout_and_no_progress_guards(self):
        for camera in (None, dict(angle=-20, moving=False)):
            with self.assertRaisesRegex(DeviceError, 'forward camera view'):
                observe(RampAlignment(0), 2.2, state=status(cameraPitch=camera))
        nav = RampAlignment(0)
        self.assertFalse(observe(nav, 2.2, gap=False))
        with self.assertRaisesRegex(DeviceError, 'visible floor'):
            observe(nav, 15.1, gap=False)
        with self.assertRaisesRegex(DeviceError, 'timed out'):
            observe(RampAlignment(0), 121)
        for lateral, heading in ((.11, 0), (0, math.radians(21)), (float('nan'), 0)):
            nav = RampAlignment(0)
            with self.assertRaisesRegex(DeviceError, 'capture range'):
                for t in np.arange(.1, 2.8, .1):
                    observe(nav, float(t), lateral=lateral, heading=heading)
        nav = RampAlignment(0)
        with self.assertRaisesRegex(DeviceError, 'no measurable progress'):
            for t in np.arange(.1, 49., .1):
                observe(nav, float(t), lateral=.04)

    def test_bad_imu_tilt_and_missing_firmware_refused_before_start(self):
        for changes in (dict(imuAgeMs=51), dict(orientation=[0, float('nan'), 0]),
                        dict(orientation=[math.radians(9), 0, 0]),
                        dict(corridorSearchGait=False), dict(boundedSteering=False)):
            with self.assertRaises(DeviceError):
                check_alignment(status(**changes))
            app = Dashboard(align_ramp=True)
            app.browser_heartbeat = time.monotonic()
            with patch('robot_dashboard.runtime.devices.request', return_value=status(
                    externalActive=False, **changes)) as request:
                with self.assertRaises(DeviceError):
                    app.walk_command('course')
            self.assertNotIn('/motion/start', [c.args[1] for c in request.call_args_list])

    def test_handover_requires_confirmed_stop_and_keeps_estimate(self):
        for continue_course in (False, True):
            app = Dashboard(align_ramp=True)
            w = app.walking
            w.start_settings('ramp_align'); w.begin(status(), 8, 0, 'ramp_align')
            w.alignment_then_course = continue_course
            w.navigation.estimate.heading, w.navigation.estimate.lateral = .02, .008
            replies = [{}, status(externalActive=False), {'session': 9}]
            with patch('robot_dashboard.runtime.devices.request', side_effect=replies) as request:
                app._after_ramp_alignment('http://robot.test')
            self.assertEqual([c.args[1] for c in request.call_args_list],
                             ['/stop', '/status'] + (['/motion/start'] if continue_course else []))
            self.assertEqual(w.active, continue_course)
            if continue_course:
                self.assertEqual(w.profile, 'course')
                self.assertEqual(w.crest_sequence.phase, 'approach')
                self.assertAlmostEqual(w.navigation.estimate.heading, .02)
                self.assertAlmostEqual(w.navigation.estimate.lateral, .008)
        for reply in (DeviceError('stop failed'), status(externalActive=True)):
            app = Dashboard(align_ramp=True)
            w = app.walking
            w.start_settings('ramp_align'); w.begin(status(), 8, 0, 'ramp_align')
            w.alignment_then_course = True
            replies = [reply] if isinstance(reply, Exception) else [{}, reply]
            with patch('robot_dashboard.runtime.devices.request', side_effect=replies) as request:
                with self.assertRaises(DeviceError):
                    app._after_ramp_alignment('http://robot.test')
            self.assertFalse(w.active)
            self.assertNotIn('/motion/start', [c.args[1] for c in request.call_args_list])

    def test_stance_switch_preserves_run_and_restores_correction_gait(self):
        app = Dashboard(align_ramp=True)
        w = app.walking
        w.start_settings('ramp_align'); w.begin(status(), 8, 0, 'ramp_align')
        w.alignment_then_course = True
        original_navigation = w.navigation
        for mode, next_session in (('hold', 9), ('correct', 10)):
            w.navigation.request_stance(mode)
            with patch('robot_dashboard.runtime.devices.request', side_effect=[
                    {}, status(externalActive=False), {'session': next_session}]) as request, \
                    patch('robot_dashboard.runtime.time.monotonic', return_value=next_session*2.):
                app._change_alignment_stance('http://robot.test')
            self.assertEqual([c.args[1] for c in request.call_args_list],
                             ['/stop', '/status', '/motion/start'])
            settings = request.call_args_list[-1].args[2]
            self.assertEqual(settings['pitch_lift'], 0 if mode == 'hold' else ALIGNMENT_GAIT['pitch_lift'])
            self.assertEqual(settings['yaw_amplitude'], 0 if mode == 'hold' else ALIGNMENT_GAIT['yaw_amplitude'])
            self.assertEqual(settings['profile'], 'corridor_search')
            self.assertTrue(w.active)
            self.assertTrue(w.alignment_then_course)
            self.assertIs(w.navigation, original_navigation)
            self.assertEqual(w.navigation.started, 0)  # Overall timeout cannot reset.
            self.assertEqual(w.session, next_session)
            now = w.last_confirmed_frame + .1
            frame = w.frame(status(motionSession=next_session), now)
            self.assertEqual((frame['ready'], frame['forward'], frame['heading'], frame['lateral']),
                             (0, 0, 0, 0))

    def test_stance_switch_failure_cannot_start_or_continue_alignment(self):
        for reply in (DeviceError('stop failed'), status(externalActive=True),
                      status(externalActive=False, orientation=[0, .2, 0])):
            app = Dashboard(align_ramp=True)
            w = app.walking
            w.start_settings('ramp_align'); w.begin(status(), 8, 0, 'ramp_align')
            w.alignment_then_course = True
            w.navigation.request_stance('hold')
            replies = [reply, {}] if isinstance(reply, Exception) else [{}, reply, {}]
            with patch('robot_dashboard.runtime.devices.request', side_effect=replies) as request:
                with self.assertRaises(DeviceError):
                    app._change_alignment_stance('http://robot.test')
            self.assertFalse(w.active)
            self.assertFalse(w.alignment_then_course)
            self.assertNotIn('/motion/start', [c.args[1] for c in request.call_args_list])

    def test_invalid_stance_session_requests_stop_after_partial_start(self):
        app = Dashboard(align_ramp=True)
        w = app.walking
        w.start_settings('ramp_align'); w.begin(status(), 8, 0, 'ramp_align')
        w.navigation.request_stance('hold')
        with patch('robot_dashboard.runtime.devices.request', side_effect=[
                {}, status(externalActive=False), {'session': 0}, {}]) as request:
            with self.assertRaisesRegex(DeviceError, 'Invalid ESP32'):
                app._change_alignment_stance('http://robot.test')
        self.assertFalse(w.active)
        self.assertEqual(request.call_args_list[-1].args[1], '/stop')

    def test_auto_alignment_is_explicit_and_cannot_run_at_assisted_crest(self):
        with self.assertRaises(ValueError):
            Dashboard(align_ramp=True, start_at_crest=True, auto_corridor=False)
        app = Dashboard(align_ramp=True)
        app.browser_heartbeat = time.monotonic()
        with patch('robot_dashboard.runtime.devices.request', side_effect=[
                status(externalActive=False), {'session': 8}]):
            app.walk_command('course')
        self.assertEqual(app.walking.profile, 'ramp_align')
        self.assertTrue(app.walking.alignment_then_course)

    @unittest.skipUnless(os.environ.get('CREST_GAIT_TEST_EXE'), 'Compiled gait harness required')
    def test_zero_forward_alignment_matches_firmware_for_both_axes(self):
        rng = np.random.default_rng(46)
        p = ALIGNMENT_GAIT | dict(search=True, corridor=True, pitch_min=math.radians(-125))
        lines, expected = [], []
        for _ in range(100):
            elapsed, phase, heading, lateral = rng.uniform([2, 0, -.3, -.09], [10, 6, .3, .09])
            keys = ('period','duty','yaw_amplitude','pitch_lift','pitch_stance','yaw_bias',
                    'pitch_min','heading_gain','lateral_gain','strafe_gain')
            values = [p[k] for k in keys] + [elapsed, phase, heading, lateral, p['turn_limit'], 1]
            lines.append('align '+' '.join(map(str, values)))
            expected.append(rocker_targets(p, elapsed, phase, heading, lateral, forward_scale=0))
        result = subprocess.run([os.environ['CREST_GAIT_TEST_EXE']], input='\n'.join(lines)+'\n',
                                text=True, capture_output=True, check=True)
        actual = np.array([np.fromstring(line, sep=' ') for line in result.stdout.splitlines()])
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)


if __name__ == '__main__':
    unittest.main()
