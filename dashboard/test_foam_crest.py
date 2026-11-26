"""Controller, contact geometry and scoring for the headless foam crest trials."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from robot_b91.gait import FOAM_CREST_GAIT, RAMP_SPEED_SCALE, RAMP_STEERING, FoamCrestController, rocker_targets
from robot_b91.run import load_gait_parameters
from robot_b91.sim_crest import (BASE_M, CLEARANCE_M, FOAM_M, PLACEMENTS, make_model,
                                 run_trial, scenarios)


class FoamCrestTests(unittest.TestCase):
    def setUp(self):
        self.climb = load_gait_parameters() | RAMP_STEERING
        self.wave = self.climb | FOAM_CREST_GAIT
        self.wave['period'] /= RAMP_SPEED_SCALE

    def test_transition_shares_phase_with_climb_and_limits_stride_as_heading_grows(self):
        for pitch, mix, bias in ((-30, 1., 0.), (-14, .5, 0.), (-10, 0., 0.),
                                 (-2, 0., -.2), (6, 0., -.4), (30, 0., -.4)):
            for yaw, stride in ((0, 1.), (8, .75), (-16, .5)):
                with self.subTest(pitch=pitch, yaw=yaw):
                    controller = FoamCrestController(self.wave, self.climb)
                    heading = math.radians(yaw)
                    wave = rocker_targets(self.wave | dict(yaw_bias=bias, search=True), 4, .67,
                                          heading, forward_scale=stride)
                    climb = rocker_targets(self.climb | dict(period=self.wave['period'], search=True),
                                           4, .67, heading, forward_scale=stride)
                    actual = controller.targets(4, .67, math.radians(pitch), heading)
                    np.testing.assert_allclose(actual, wave + (climb - wave) * mix, atol=1e-12)

    def test_prolonged_level_recovery_is_bounded_and_resets_between_trials(self):
        controller = FoamCrestController(self.wave, self.climb)
        for t, bias in ((0, -.25), (19, -.25), (21.5, -.485), (24, -.72), (40, -.72)):
            target = controller.targets(t, .6, 0., 0.)
            expected = rocker_targets(self.wave | {'yaw_bias': bias}, t, .6)
            np.testing.assert_allclose(target, expected)
        self.assertEqual(self.wave['yaw_bias'], 0.)
        fresh = FoamCrestController(self.wave, self.climb)
        np.testing.assert_array_equal(fresh.targets(0, 0, -.3, .1), np.zeros(8))

    def test_invalid_attitude_and_clock_do_not_issue_targets(self):
        controller = FoamCrestController(self.wave, self.climb)
        controller.targets(1., .5, 0., 0.)
        for t, pitch, heading in ((1., math.nan, 0.), (1., 0., math.inf), (.5, 0., 0.)):
            with self.assertRaises(ValueError):
                controller.targets(t, .5, pitch, heading)

    def test_random_cases_are_repeatable_and_respect_height_overrides(self):
        a = list(scenarios(['random'], [16.], 8, 17))
        self.assertEqual(a, list(scenarios(['random'], [16.], 8, 17)))
        self.assertNotEqual(a, list(scenarios(['random'], [16.], 8, 18)))
        self.assertEqual([h for _, h, _ in a], [.16] * 8)

    def test_foam_and_backing_are_separated_normal_to_each_panel(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'crest.xml'
            for height, run in ((.10, .50), (.225, .40)):
                make_model(path, height, run)
                xml = ET.parse(path)
                self.assertFalse(any('_tread_' in g.get('name', '') for g in xml.iter('geom')))
                self.assertIsNotNone(xml.find(".//body[@name='RL_pitch']/geom[@contype='2']"))
                cos_slope = math.cos(math.atan2(height - BASE_M, run))
                for name in ('uphill', 'downhill'):
                    skin = xml.find(f"./worldbody/geom[@name='{name}']")
                    board = xml.find(f"./worldbody/geom[@name='{name}_board']")
                    self.assertEqual(skin.get('mesh'), board.get('mesh'))
                    vertical_offset = float(board.get('pos').split()[2])
                    self.assertAlmostEqual(-vertical_offset * cos_slope, FOAM_M)

    def test_a_settled_stance_is_not_a_crossing_but_the_centered_gait_crosses(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'crest.xml'
            make_model(path, .10)
            short = run_trial(path, .10, PLACEMENTS[0], .05, 'foam')
            self.assertEqual(short['outcome'], 'timeout')
            complete = run_trial(path, .10, PLACEMENTS[0], 25., 'foam')
            self.assertEqual(complete['outcome'], 'crossed', complete)
            self.assertGreaterEqual(complete['max_x_past_crest_m'], CLEARANCE_M)
