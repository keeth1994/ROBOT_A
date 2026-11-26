"""Mechanical trajectory invariants and optional compiled ESP32-math parity."""
import math
import os
import subprocess
import unittest

import numpy as np

from robot_b91.gait import rocker_targets
from robot_b91.run import load_gait_parameters
from dashboard.robot_dashboard.crest import CREST_WAVE_GAIT


class CrestWaveTests(unittest.TestCase):
    def setUp(self):
        self.params = load_gait_parameters() | CREST_WAVE_GAIT

    def test_single_swing_with_all_down_intervals(self):
        for duty in (.8, .85, .9):
            params = self.params | {'duty': duty}
            lifted = [np.count_nonzero(rocker_targets(params, 4, t)[1::2] < -1e-9)
                      for t in np.linspace(0, 1, 2000, endpoint=False)]
            self.assertEqual(max(lifted), 1)
            self.assertAlmostEqual(lifted.count(0)/len(lifted), 4*duty-3, delta=.004)

    def test_lift_before_sweep_and_plant_after_sweep(self):
        params = self.params
        sample = lambda v: rocker_targets(params, 4, params['duty'] + v*(1-params['duty']))[:2]
        # Front-left foot: yaw remains fixed throughout lift and lowering.
        self.assertAlmostEqual(sample(.02)[0], sample(.18)[0])
        self.assertAlmostEqual(sample(.82)[0], sample(.98)[0])
        for v in (.2, .4, .6, .8):
            self.assertAlmostEqual(sample(v)[1], -params['front_pitch_lift'])
        self.assertGreater(abs(sample(.2)[0]-sample(.8)[0]), .5)

    def test_extra_lift_only_changes_front_feet_in_crest_gait(self):
        baseline = dict(self.params)
        baseline.pop('front_pitch_lift')
        for pattern in ('crest', 'trot', 'crawl'):
            for time in np.linspace(0, 1, 101):
                before = rocker_targets(baseline | dict(pattern=pattern), 4, time)
                after = rocker_targets(self.params | dict(pattern=pattern), 4, time)
                if pattern == 'crest':
                    np.testing.assert_allclose(after[[3, 7]], before[[3, 7]])
                    ratio = self.params['front_pitch_lift'] / self.params['pitch_lift']
                    np.testing.assert_allclose(after[[1, 5]], ratio * before[[1, 5]])
                    np.testing.assert_allclose(after[::2], before[::2])
                else:
                    np.testing.assert_array_equal(after, before)

    def test_limits_continuity_and_startup(self):
        np.testing.assert_array_equal(rocker_targets(self.params, 0, 0), np.zeros(8))
        for heading in (-.5, 0, .5):
            for lateral in (-.06, 0, .06):
                samples = np.array([rocker_targets(self.params, 4, t, heading, lateral)
                                    for t in np.linspace(0, 1, 2001)])
                self.assertTrue(np.isfinite(samples).all())
                self.assertLessEqual(np.abs(samples[:, ::2]).max(), math.pi/4)
                self.assertLessEqual(samples[:, 1::2].max(), 0)
                self.assertGreaterEqual(samples[:, 1::2].min(), -math.radians(125))
                np.testing.assert_allclose(samples[0], samples[-1], atol=1e-12)
                for offset in (0, .25, .5, .75):
                    for phase in (0, .8, .84, .96):
                        boundary = (phase-offset) % 1 + 1
                        before = rocker_targets(self.params, 4, boundary-1e-8, heading, lateral)
                        after = rocker_targets(self.params, 4, boundary+1e-8, heading, lateral)
                        np.testing.assert_allclose(before, after, atol=1e-5, rtol=0)
        with self.assertRaises(ValueError):
            rocker_targets(self.params | {'duty': .79}, 1, 1)

    @unittest.skipUnless(os.environ.get('CREST_GAIT_TEST_EXE'), 'Set CREST_GAIT_TEST_EXE to the compiled host harness')
    def test_firmware_parity_and_pause_stop(self):
        rng = np.random.default_rng(61)
        lines, expected = [], []
        for pattern in ('trot', 'crawl', 'crest'):
            for _ in range(250):
                p = self.params | dict(pattern=pattern, duty=float(rng.uniform(.8, .9)),
                    yaw_amplitude=float(rng.uniform(.1, .78)), yaw_bias=float(rng.uniform(-.7, .5)),
                    pitch_lift=float(rng.uniform(.1, 1.1)), period=float(rng.uniform(.8, 2.5)))
                if _ % 2:
                    p['rear_stride_scale'] = float(rng.uniform(0, 1))
                    p['front_pitch_lift'] = float(rng.uniform(0, 1.1))
                else:
                    p.pop('front_pitch_lift')
                elapsed, time, heading, lateral = rng.uniform([0, 0, -.4, -.05], [8, 15, .4, .05])
                keys = ('period','duty','yaw_amplitude','pitch_lift','pitch_stance','yaw_bias')
                values = [p[k] for k in keys] + [p.get('pitch_min', math.radians(-125)),
                    p['heading_gain'], p.get('lateral_gain',0), p.get('strafe_gain',0),
                    elapsed, time, heading, lateral,
                    p.get('rear_stride_scale', 1), p.get('front_pitch_lift', -1)]
                lines.append(pattern+' '+' '.join(map(str,values)))
                expected.append(rocker_targets(p, elapsed, time, heading, lateral))
        result = subprocess.run([os.environ['CREST_GAIT_TEST_EXE']], input='\n'.join(lines)+'\n',
                                text=True, capture_output=True, check=True)
        actual = np.array([np.fromstring(line, sep=' ') for line in result.stdout.splitlines()])
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)


if __name__ == '__main__':
    unittest.main()
