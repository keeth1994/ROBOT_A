"""Corridor selection and approach using the layout beside the ramp."""
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from dashboard.robot_dashboard.corridor_search import CorridorSearch, visible_wall_ends, wall_view
from dashboard.robot_dashboard.corridor_walls import load_corridor_settings
from dashboard.test_crest_sequence import status


def endpoint(point, direction=(-1., 0.), samples=10):
    tangent = np.array(direction, dtype=float)
    tangent /= np.linalg.norm(tangent)
    return dict(point=np.array(point, dtype=float), tangent=tangent,
                normal=np.array([-tangent[1], tangent[0]]), samples=samples)


class CorridorEntranceTests(unittest.TestCase):
    def navigation(self):
        return CorridorSearch(0, load_corridor_settings()['navigation'])

    def test_prefers_a_side_entrance_leading_back_on_either_side_at_any_heading(self):
        for heading in (0, 1.3, -2.5):
            rotation = np.array([[math.cos(heading), -math.sin(heading)],
                                 [math.sin(heading), math.cos(heading)]])
            for side in (-1, 1):
                with self.subTest(heading=heading, side=side):
                    nav = self.navigation()
                    nav.search_heading = heading
                    correct = endpoint(rotation @ [0., side * .9], rotation @ [-1., 0.])
                    wrong_end = endpoint(rotation @ [0., side * .6], rotation @ [1., 0.])
                    wrong_place = endpoint(rotation @ [.6, 0.], rotation @ [-1., 0.])
                    nav.candidates = [wrong_end, wrong_place, correct]
                    self.assertTrue(nav.choose_entrance())
                    self.assertIs(nav.selected, correct)
                    self.assertAlmostEqual(math.cos(nav.entry_heading - heading), -1)

    def test_correct_single_end_beats_a_pair_at_the_wrong_end(self):
        nav = self.navigation()
        nav.search_heading = 0
        correct = endpoint([0., -.9])
        nav.candidates = [endpoint([.5, -.2], (1, 0)),
                          endpoint([.5, .2], (1, 0)), correct]
        nav.choose_entrance()
        self.assertIs(nav.selected, correct)

    def test_pair_is_preferred_when_both_entrance_ends_are_visible(self):
        nav = self.navigation()
        nav.search_heading = 0
        nav.candidates = [endpoint([0., -.7]), endpoint([0., -1.1])]
        nav.choose_entrance()
        self.assertIsNotNone(nav.selected_pair)
        np.testing.assert_allclose(nav.approach_goal, [.25, -.9])

    def test_pair_uses_both_wall_directions_when_one_estimate_is_noisy(self):
        nav = self.navigation()
        nav.search_heading = 0
        near = endpoint([-.287, -.556], (-1., .01), samples=4)
        far = endpoint([-.363, -1.083], (-.88, .36), samples=16)
        for ends in ((near, far), (far, near)):
            nav.candidates = list(ends)
            nav.choose_entrance()
            self.assertIsNotNone(nav.selected_pair)

    def test_layout_is_a_preference_not_a_required_angle(self):
        nav = self.navigation()
        nav.search_heading = 0
        only = endpoint([.8, .2], (.7, .7))
        nav.candidates = [only]
        self.assertTrue(nav.choose_entrance())
        self.assertIs(nav.selected, only)

    def test_tracking_corrects_the_goal_without_switching_to_another_end(self):
        nav = self.navigation()
        target = endpoint([1., .2])
        nav.candidates = [target]
        nav.choose_entrance()
        old_goal = nav.approach_goal.copy()
        nav.view = dict(end_pixels=[(100, 150), (220, 120)])
        observation = (np.array([1.1, .25]), target['tangent'], target['normal'])
        decoy = (np.array([1., .2]), -target['tangent'], target['normal'])
        with patch.object(nav, 'observe_ends', return_value=[observation, decoy]):
            nav.track_entrance()
        self.assertIs(nav.selected, target)
        np.testing.assert_allclose(nav.approach_goal - old_goal, [.03, .015])
        self.assertEqual(nav.tracked_frames, 1)
        self.assertEqual(nav.view['tracked_pixels'], [(100, 150)])

    def test_tracking_a_pair_does_not_match_both_targets_to_the_same_end(self):
        nav = self.navigation()
        first, second = endpoint([0., -.7]), endpoint([0., -1.1])
        nav.candidates = [first, second]
        nav.choose_entrance()
        old = second['point'].copy()
        nav.view = {}
        observation = (np.array([0., -.8]), first['tangent'], first['normal'])
        with patch.object(nav, 'observe_ends', return_value=[observation]):
            nav.track_entrance()
        np.testing.assert_allclose(second['point'], old)

    def test_close_endpoint_is_not_replaced_when_it_leaves_the_image(self):
        nav = self.navigation()
        target = endpoint([.3, 0.])
        nav.candidates = [target]
        nav.choose_entrance()
        nav.view = {}
        observation = (np.array([.45, 0.]), target['tangent'], target['normal'])
        with patch.object(nav, 'observe_ends', return_value=[observation]):
            nav.track_entrance()
        np.testing.assert_allclose(target['point'], [.3, 0.])
        self.assertEqual(nav.tracked_frames, 0)

    def test_missing_or_unrelated_endpoint_keeps_the_original_target(self):
        for observations in ([], [(np.array([1., 2.]), np.array([-1., 0.]), np.array([0., 1.]))]):
            nav = self.navigation()
            nav.candidates = [endpoint([0., -.9])]
            nav.choose_entrance()
            old_goal = nav.approach_goal.copy()
            nav.view = {}
            with patch.object(nav, 'observe_ends', return_value=observations):
                nav.track_entrance()
            np.testing.assert_allclose(nav.approach_goal, old_goal)
            self.assertEqual(nav.tracked_frames, 0)

    def test_ir_handover_keeps_the_center_when_both_ends_were_seen(self):
        nav = self.navigation()
        nav.candidates = [endpoint([0., -.7]), endpoint([0., -1.1])]
        nav.choose_entrance()
        nav.imu_heading = -math.pi / 2
        goal = nav.approach_goal.copy()
        nav.acquire_wall(1, .15, 3)
        self.assertEqual(nav.stage, 'clear_end')
        np.testing.assert_allclose(nav.approach_goal, goal)

    def test_search_heading_is_captured_before_the_scan_and_not_replaced(self):
        nav = self.navigation()
        packet = status() | dict(irAgeMs=0, irMillivolts=[100, 100])
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        for now, heading in ((2, .4), (2.1, .3), (2.2, .2)):
            packet['orientation'][2] = heading
            nav.update(packet, now, (now, now, image))
        self.assertAlmostEqual(nav.search_heading, .4)

    def test_recorded_near_wall_end_survives_pale_color_and_overlap(self):
        # Camera crop at 20 seconds in the supplied corridor search recording.
        image = np.asarray(Image.open(Path(__file__).with_name('testdata') / 'corridor_scan_overlap.png'))
        view = wall_view(image)
        ends = visible_wall_ends(view, load_corridor_settings()['camera'], 0, 0)
        self.assertTrue(any(100 < x < 120 and y > 140 for x, y in view['end_pixels']))
        self.assertTrue(any(np.linalg.norm(point) < 1 for point, _, _ in ends))
        nav = self.navigation()
        nav.search_heading = 0
        nav.imu_heading = math.radians(-246)  # Scan angle displayed in this frame.
        nav.view = view
        for _ in range(5):
            nav.collect_candidates()
        self.assertTrue(nav.choose_entrance())
        self.assertIsNotNone(nav.selected_pair)
        self.assertGreater(math.cos(nav.entry_heading - math.pi), .9)

    def test_pale_wall_extends_a_colored_patch_but_plain_floor_does_not(self):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[100:210, 100:250] = (190, 178, 164)
        self.assertFalse(wall_view(image)['mask'].any())
        image[120:150, 120:150] = (180, 120, 50)
        self.assertTrue(wall_view(image)['mask'][180, 200])


if __name__ == '__main__':
    unittest.main()
