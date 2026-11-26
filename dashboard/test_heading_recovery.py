"""Paused recovery must agree across forward views before changing heading."""
import math
import unittest

from robot_dashboard.heading_recovery import PausedHeadingRecovery


def window(recovery, start, angle, heading=10, lateral=.004, attitude=(0, .35, 0)):
    result = None
    for i in range(6):
        action = recovery.observe(start+i*.15, angle, math.radians(heading), lateral,
                                  attitude, math.radians(-6))
        result = action or result
    return result


class HeadingRecoveryTests(unittest.TestCase):
    def test_two_forward_views_required_before_updating_pose(self):
        recovery = PausedHeadingRecovery((-65, -75))
        self.assertEqual(window(recovery, 0, -75), {'target_angle': -65})
        self.assertEqual(recovery.count, 0)
        self.assertIsNone(window(recovery, 1, -75))
        result = window(recovery, 3, -65, heading=11, lateral=-.003)
        self.assertAlmostEqual(math.degrees(result['heading']), 10.5)
        self.assertAlmostEqual(result['lateral'], .0005)
        self.assertEqual(recovery.count, 1)
        self.assertIsNone(window(recovery, 4, -65))

    def test_disagreement_or_movement_between_views_cannot_change_heading(self):
        for change in (dict(heading=6), dict(lateral=.025), dict(attitude=(0,.35,.06))):
            recovery = PausedHeadingRecovery((-65, -75))
            window(recovery, 0, -75)
            self.assertIsNone(window(recovery, 3, -65, **change))
            self.assertTrue(recovery.attempted)
            self.assertEqual(recovery.count, 0)

    def test_large_errors_rear_views_and_missing_second_view_are_refused(self):
        for angle, heading, lateral in ((-4,10,0),(-20,10,0),(-65,13,0),(-65,-30,0),(-65,0,.051)):
            recovery = PausedHeadingRecovery((-65,-75))
            self.assertIsNone(window(recovery, 0, angle, heading, lateral))
            self.assertIsNone(recovery.anchor)
        self.assertIsNone(window(PausedHeadingRecovery((-65,-65)), 0, -65))

    def test_stale_missing_duplicate_or_unstable_samples_cannot_confirm(self):
        recovery = PausedHeadingRecovery((-65,-75))
        for i in range(20):
            self.assertIsNone(recovery.observe(0,-65,.1,0,(0,0,0),0))
        recovery.reset()
        for i in range(20):
            self.assertIsNone(recovery.observe(i*1.,-65,.1,0,(0,0,0),0))
        recovery.reset()
        for i in range(20):
            self.assertIsNone(recovery.observe(i*.1,-65,.1 if i%2 else .17,0,(0,0,0),0))
        recovery.reset()
        for i in range(20):
            self.assertIsNone(recovery.observe(i*.1,-65,.1,0,(0,0,.06 if i%2 else 0),0))
        recovery.reset()
        for i in range(20):
            self.assertIsNone(recovery.observe(i*.1,-65,.1,0,(0,0,0),0))
            recovery.missing()

    def test_second_view_expires_and_successful_tracking_resets_attempt(self):
        recovery = PausedHeadingRecovery((-65,-75))
        window(recovery,0,-75)
        self.assertIsNone(window(recovery,8,-65))
        self.assertTrue(recovery.attempted)
        recovery.reset()
        self.assertEqual(window(recovery,10,-75),{'target_angle':-65})
        self.assertIsNotNone(window(recovery,13,-65))
        recovery.reset()
        self.assertEqual(recovery.count,1)

    def test_slow_fresh_views_can_recover_heading(self):
        recovery = PausedHeadingRecovery((-65, -75))
        for now in (0., .6, 1.2, 1.8):
            result = recovery.observe(now, -65, .1, .004, (0, 0, 0), 0)
        self.assertEqual(result, {'target_angle': -75})
        for now in (4., 4.6, 5.2, 5.8):
            result = recovery.observe(now, -75, .11, .005, (0, 0, 0), 0)
        self.assertAlmostEqual(result['heading'], .105)
        self.assertEqual(recovery.count, 1)


if __name__ == '__main__':
    unittest.main()
