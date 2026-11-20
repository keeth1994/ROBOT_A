"""Rear-only placement and lift retain front trajectories and joint limits."""
import math
import unittest

import numpy as np

from robot_b91.gait import RAMP_STEERING
from robot_b91.rear_flare_experiment import rear_targets as rocker_targets
from robot_b91.run import load_gait_parameters
from rear_support_probe import plan


class RearPlacementTests(unittest.TestCase):
    def test_support_probe_flares_only_lifted_rear_feet_and_keeps_front_fixed(self):
        stages = plan()
        self.assertLess(sum(s[1] for s in stages), 16)
        self.assertEqual(stages[0][2], [0, 45, 0, 45, 0, 45, 0, 45])
        self.assertEqual(stages[1][2], [14, 45, 14, 45, -14, 45, -14, 45])
        previous = stages[1][2]
        for _, duration, pose in stages[2:]:
            self.assertGreater(duration, 0)
            self.assertEqual(pose[:2] + pose[4:6], [14, 45, -14, 45])
            self.assertLessEqual(sum(p < 45 for p in pose[1::2]), 1)
            self.assertTrue(all(31 <= p <= 45 for p in pose[1::2]))
            for leg in (1, 3):
                if previous[2*leg] != pose[2*leg]:
                    self.assertEqual(previous[2*leg+1], 31)
                    self.assertEqual(pose[2*leg+1], 31)
            previous = pose
        self.assertEqual(stages[-1][2], [14, 45, 23, 45, -14, 45, -23, 45])

    def test_flare_and_lift_leave_front_action_and_support_schedule_unchanged(self):
        base = load_gait_parameters() | RAMP_STEERING | dict(
            pattern='crest', duty=.8, yaw_amplitude=.45, rear_stride_scale=.35,
            yaw_bias=-.25, pitch_stance=0, pitch_lift=.25)
        for t in np.linspace(0, 1, 501):
            old = rocker_targets(base, 4, t, .1, .02)
            explicit = rocker_targets(base | dict(rear_yaw_bias=-.25, rear_pitch_lift=.25), 4, t, .1, .02)
            np.testing.assert_array_equal(old, explicit)
            new = rocker_targets(base | dict(rear_yaw_bias=-.5, rear_pitch_lift=.45), 4, t, .1, .02)
            np.testing.assert_array_equal(old[[0, 1, 4, 5]], new[[0, 1, 4, 5]])
            np.testing.assert_allclose(new[[2, 6]], old[[2, 6]] + [-.25, .25])
            np.testing.assert_allclose(new[[3, 7]], old[[3, 7]] * .45/.25)
            self.assertLessEqual(np.count_nonzero(new[1::2] < -1e-9), 1)
            self.assertLessEqual(np.abs(new[::2]).max(), math.pi/4)
        np.testing.assert_array_equal(rocker_targets(base | dict(rear_yaw_bias=-.5,
            rear_pitch_lift=.45), 0, 0), np.zeros(8))

    def test_invalid_rear_placement_is_rejected(self):
        base = load_gait_parameters()
        for key, values in dict(rear_yaw_bias=(-.8, .8, float('nan')),
                                rear_pitch_lift=(-.1, 2.2, float('nan'))).items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    rocker_targets(base | {key: value}, 4, .5)


if __name__ == '__main__':
    unittest.main()
