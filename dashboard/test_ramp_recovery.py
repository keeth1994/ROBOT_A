"""Course excursions can recover without ending a healthy motion session."""
import math
import unittest
from unittest.mock import patch

from robot_dashboard.devices import DeviceError
from robot_dashboard.ramp_recovery import RampHeadingRecovery
from robot_dashboard.walking import Walking
from test_course_control import status


def update(recovery, now, heading=14, lateral=.01, roll=4, pitch=-20,
           ready=True, fresh_edges=True):
    return recovery.update(now, math.radians(heading), lateral,
                           math.radians(roll), math.radians(pitch),
                           ready=ready, fresh_edges=fresh_edges)


class RampRecoveryTests(unittest.TestCase):
    def test_recorded_moderate_errors_enter_recovery_on_either_side(self):
        for heading, offset, roll in ((12.4, -.0039, 3.6), (16., -.0145, 6.9), (14.1, .0126, 8.7)):
            for sign in (-1, 1):
                recovery = RampHeadingRecovery()
                self.assertTrue(update(recovery, 3., sign*heading, offset, roll))
                self.assertTrue(recovery.active)

    def test_recovery_needs_sustained_centering_and_heading(self):
        recovery = RampHeadingRecovery()
        update(recovery, 0)
        update(recovery, .4, heading=6)
        update(recovery, .8, heading=10)  # One good reading is insufficient.
        update(recovery, 1.2, heading=6, lateral=.03)
        self.assertTrue(recovery.active)
        for now in (1.6, 2., 2.4, 2.8):
            update(recovery, now, heading=6)
        self.assertFalse(recovery.active)

    def test_missing_guidance_holds_and_recovers_even_after_a_long_delay(self):
        for changes in (dict(ready=False), dict(fresh_edges=False),
                        dict(fresh_edges=None)):
            recovery = RampHeadingRecovery()
            update(recovery, 0)
            self.assertFalse(update(recovery, 1., **changes))
            self.assertTrue(recovery.active)
            self.assertTrue(update(recovery, 2.))
            self.assertFalse(update(recovery, 90., **changes))
            self.assertTrue(update(recovery, 91.))

    def test_large_finite_errors_keep_walking_but_invalid_telemetry_is_rejected(self):
        for changes in (dict(heading=20), dict(heading=-21), dict(lateral=.051),
                        dict(lateral=-.051), dict(roll=15), dict(pitch=-46)):
            recovery = RampHeadingRecovery()
            with self.subTest(changes=changes):
                self.assertTrue(update(recovery, 0, **changes))
                self.assertTrue(update(recovery, 120, **changes))
        with self.assertRaises(DeviceError):
            update(RampHeadingRecovery(), 0., heading=math.nan)

    def test_control_gap_does_not_count_as_recovered(self):
        recovery = RampHeadingRecovery()
        update(recovery, 0)
        update(recovery, .4, heading=6)
        update(recovery, 3., heading=6)
        self.assertTrue(recovery.active)

    def walking(self):
        w = Walking(ramp_drift_control=True, adaptive_ramp_speed=True)
        w.start_settings('course')
        w.begin(status(), 8, 0., 'course')
        w.set_speed(.8)
        e = w.navigation.estimate
        e.heading, e.lateral, e.roll, e.pitch = math.radians(-14), .01, .05, -.3
        e.debug = dict(accepted=True, held=False, timestamp=3., lateral_source='paired_edges',
                       measured_lateral_m=.01, surface_detected=True, reason='Camera measurement accepted')
        w.last_confirmed_frame = 3.
        return w

    def test_recovery_commands_are_bounded_and_operator_stop_still_works(self):
        w = self.walking()
        w.crest_sequence.climbed = True
        w.navigation.estimate.pitch = 0
        with patch.object(w.navigation, 'update', return_value=True):
            f = w.frame(status(), 3.)
        self.assertEqual(f['ready'], 1)
        self.assertEqual(f['speed'], .5)
        self.assertEqual(f['forward'], .35)
        self.assertEqual(f['heading'], math.radians(-14))
        self.assertTrue(w.state()['ramp_heading_recovery'])
        self.assertIsNone(w.crest_sequence.requested)
        w.stop()
        self.assertFalse(w.heading_recovery.active)
        with self.assertRaisesRegex(DeviceError, 'Walking is stopped'):
            w.frame(status(), 3.1)

    def test_stale_edges_hold_but_fresh_large_raw_and_filtered_offsets_do_not_stop(self):
        w = self.walking()
        w.navigation.estimate.debug['timestamp'] = 2.
        with patch.object(w.navigation, 'update', return_value=True):
            self.assertEqual(w.frame(status(), 3.)['ready'], 0)
        w.navigation.estimate.debug.update(timestamp=3.1, measured_lateral_m=.06)
        with patch.object(w.navigation, 'update', return_value=True):
            self.assertEqual(w.frame(status(), 3.1)['ready'], 1)
        w.navigation.estimate.lateral = .06
        with patch.object(w.navigation, 'update', return_value=True):
            self.assertEqual(w.frame(status(), 3.2)['ready'], 1)
        self.assertTrue(w.active)

    def test_recovery_preserves_link_and_session_checks(self):
        for changes in (dict(externalActive=False), dict(motionSession=99),
                        dict(gaitStopping=True)):
            w = self.walking()
            with self.assertRaises(DeviceError):
                w.frame(status(**changes), 3.)
        w = self.walking()
        with self.assertRaisesRegex(DeviceError, 'control overrun'):
            w.frame(status(), 4.1)
        w = self.walking()
        w.ramp_drift_control = False
        with patch.object(w.navigation, 'update', return_value=True):
            frame = w.frame(status(), 3.)
            self.assertAlmostEqual(frame['heading'], math.radians(-14))
            self.assertFalse(w.heading_recovery.active)


if __name__ == '__main__':
    unittest.main()
