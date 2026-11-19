"""Recorded-manual-recovery failure cases, without hardware access."""
import math
import unittest

from crest_recovery import fit_heading_drift,heading_change,plan


class CrestRecoveryTests(unittest.TestCase):
    def samples(self):
        return [dict(imuTimestampMs=100000+i*100,orientation=[.01,-.14,
                math.radians(179.5-.324*i*.1)]) for i in range(41)]

    def test_idle_drift_does_not_hide_a_real_turn(self):
        samples=self.samples()
        ref=fit_heading_drift(samples)
        self.assertAlmostEqual(math.degrees(ref['rate_rad_s']),-.324,places=5)
        state=dict(imuTimestampMs=ref['time_ms']+30000,orientation=[0,0,
                   ref['yaw_rad']+math.radians(-.324*30+11)])
        self.assertAlmostEqual(math.degrees(heading_change(ref,state)),11,places=5)
        state['orientation'][2] -= math.radians(11)
        self.assertAlmostEqual(heading_change(ref,state),0,places=6)

    def test_moving_calibration_and_reset_clock_are_rejected(self):
        samples=self.samples()
        samples[20]['orientation'][0] += math.radians(2)
        with self.assertRaises(ValueError):
            fit_heading_drift(samples)
        ref=fit_heading_drift(self.samples())
        with self.assertRaises(ValueError):
            heading_change(ref,dict(imuTimestampMs=100,orientation=[0,0,0]))

    def test_compact_plan_keeps_three_feet_down_and_bounded_angles(self):
        stages=plan()
        self.assertLess(sum(s[1] for s in stages),25)
        for _,_,angles in stages:
            self.assertLessEqual(sum(p<45 for p in angles[1::2]),1)
            self.assertTrue(all(-45<=a<=45 for a in angles[::2]))
            self.assertTrue(all(15<=a<=45 for a in angles[1::2]))
        with self.assertRaises(ValueError):
            plan(40,10)


if __name__=='__main__':
    unittest.main()
