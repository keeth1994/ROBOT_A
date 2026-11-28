"""Check generated terrain limits across different random courses."""
import math
import unittest

from robot.randomize_ramp import make_profile


class RandomRampTests(unittest.TestCase):
    def test_generated_courses_respect_geometry_limits(self):
        for seed in (0, 1, 42, 20261120):
            with self.subTest(seed=seed):
                profile, names = make_profile(8.4, seed)
                self.assertEqual(len(names), len(set(names)))
                self.assertEqual(len(names), len(profile) - 1)
                self.assertAlmostEqual(profile[-1][0], 8.4)
                self.assertAlmostEqual(max(z for x, z in profile), .225)
                self.assertEqual(profile[0][1], profile[-1][1])
                for (x0, z0), (x1, z1) in zip(profile, profile[1:]):
                    self.assertGreater(x1, x0)
                    self.assertGreaterEqual(z1, .01)
                    if z1 != z0:
                        angle = abs(math.degrees(math.atan2(z1 - z0, x1 - x0)))
                        self.assertGreaterEqual(angle, 20 - 1e-10)
                        self.assertLessEqual(angle, 27 + 1e-10)

    def test_seed_is_repeatable_and_changes_the_course(self):
        self.assertEqual(make_profile(8.4, 42), make_profile(8.4, 42))
        self.assertNotEqual(make_profile(8.4, 42), make_profile(8.4, 43))

    def test_short_course_rejects_too_many_hills(self):
        with self.assertRaisesRegex(ValueError, 'too short'):
            make_profile(2, 42)

    def test_exactly_half_the_tops_are_crests(self):
        for hills in (2, 4, 6):
            for seed in (0, 1, 42, 20261120):
                with self.subTest(hills=hills, seed=seed):
                    profile, names = make_profile(8.4, seed, hills=hills)
                    crests = sum(b[1] > a[1] and b[1] > c[1]
                                 for a, b, c in zip(profile, profile[1:], profile[2:]))
                    self.assertEqual(crests, hills // 2)
                    self.assertEqual(sum(name.startswith('platform_') for name in names), hills // 2)

    def test_odd_hill_counts_cannot_have_exactly_half_crests(self):
        for hills in (1, 3, 5):
            with self.subTest(hills=hills):
                with self.assertRaisesRegex(ValueError, 'even number'):
                    make_profile(8.4, 42, hills=hills)


if __name__ == '__main__':
    unittest.main()
