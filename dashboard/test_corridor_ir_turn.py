"""IR handover and wall clearance during the corridor entrance turn."""
import math
from pathlib import Path
import unittest

import numpy as np
from PIL import Image

from dashboard.test_corridor_entrance import endpoint
from dashboard.test_crest_sequence import status
from dashboard.robot_dashboard.corridor_search import CorridorSearch, RAMP_CLEARANCE_M
from dashboard.robot_dashboard.corridor_walls import IRWallNavigation, load_corridor_settings


class CorridorIRTurnTests(unittest.TestCase):
    def navigation(self):
        nav = CorridorSearch(0, load_corridor_settings()['navigation'])
        nav.candidates = [endpoint([1.5, .8], (0, 1))]
        nav.choose_entrance()
        nav.stage = 'approach'
        nav.approach_walk_time = 15
        return nav

    def recorded_image(self):
        # Camera crop at 65 seconds in the second corridor search recording.
        path = Path(__file__).with_name('testdata') / 'corridor_close_left.png'
        if not path.exists():
            self.skipTest("Recorded corridor image is missing.")
        return np.asarray(Image.open(path))

    def frame(self, nav, now, readings=(1260, 256), heading=0, image=None):
        packet = status() | dict(irAgeMs=0, irMillivolts=list(readings))
        packet['orientation'][2] = heading
        if image is None:
            image = np.full((240, 320, 3), 180, dtype=np.uint8)
            image[80:210, :150] = (180, 120, 50)
        return nav.update(packet, now, (now, now, image))

    def test_recorded_ir_starts_turn_despite_the_wrong_camera_distance(self):
        for side in (0, 1):
            with self.subTest(side=side):
                nav = self.navigation()
                nav.wall_heading = math.pi / 2 if side == 0 else -math.pi / 2
                image = self.recorded_image()
                readings = (1260, 256)
                if side == 1:
                    image = image[:, ::-1].copy()
                    readings = readings[::-1]
                for now in (2, 2.1, 2.2):
                    self.frame(nav, now, readings=readings, image=image)
                self.assertEqual(nav.stage, 'align_wall')
                self.assertEqual(nav.wall_side, side)
                self.assertLess(np.linalg.norm(nav.wall_point - nav.position), .25)
                self.assertGreater(np.linalg.norm(nav.wall_point - nav.selected['point']), 1)
                self.assertNotEqual(nav.estimate.heading, 0)

    def test_ir_handover_waits_for_fifteen_seconds_of_forward_walking(self):
        nav = self.navigation()
        nav.change_stage('approach', 2)
        for i in range(110):
            self.frame(nav, 2 + i * .1, heading=math.pi)
            self.assertEqual(nav.stage, 'approach')
            self.assertIsNone(nav.ir_since)
            self.assertIsNone(nav.wall_point)
        self.assertEqual(nav.approach_walk_time, 0)
        for i in range(180):
            delta = nav.approach_goal - nav.position
            self.frame(nav, 13 + i * .1, heading=math.atan2(delta[1], delta[0]))
            if nav.stage == 'align_wall':
                break
            self.assertIsNone(nav.wall_point)
        self.assertEqual(nav.stage, 'align_wall')
        self.assertGreaterEqual(nav.approach_walk_time, 15)
        self.assertLess(nav.approach_walk_time, 15.4)

    def test_pausing_does_not_count_toward_ir_handover_delay(self):
        nav = self.navigation()
        nav.change_stage('approach', 2)
        for now in (2, 5, 15, 25):
            nav.update(status(), now, None)
        self.assertEqual(nav.approach_walk_time, 0)
        self.assertIsNone(nav.ir_since)

    def test_short_ir_pulse_does_not_start_turn(self):
        nav = self.navigation()
        self.frame(nav, 2)
        self.frame(nav, 2.1, readings=(100, 100))
        self.assertEqual(nav.stage, 'approach')
        self.assertIsNone(nav.wall_point)

    def test_entrance_steps_run_at_twice_the_selected_speed(self):
        for stage in ('align_wall', 'enter', 'follow_wall'):
            nav = self.navigation()
            nav.set_speed(.5)
            nav.acquire_wall(0, .16, 2)
            nav.stage = stage
            self.frame(nav, 2.1, readings=(750, 100))
            self.assertEqual(nav.speed, 1, stage)

    def test_entry_stops_and_finishes_after_both_walls_are_seen_for_three_seconds(self):
        nav = self.navigation()
        nav.stage = 'enter'
        nav.wall_side = 0
        nav.wall_heading = 0
        self.assertFalse(self.frame(nav, 2, readings=(750, 750)))
        self.assertEqual(nav.forward, 0)
        self.assertFalse(nav.finished)
        self.assertFalse(self.frame(nav, 4.9, readings=(750, 750)))
        self.assertFalse(nav.finished)
        self.assertEqual(nav.estimate.heading, 0)
        self.assertEqual(nav.estimate.lateral, 0)
        self.frame(nav, 5, readings=(750, 750))
        self.assertTrue(nav.succeeded)
        self.assertTrue(nav.finished)
        self.assertEqual(nav.forward, 0)

    def test_losing_either_wall_restarts_the_three_second_confirmation(self):
        for readings in ((100, 750), (750, 100)):
            with self.subTest(readings=readings):
                nav = self.navigation()
                nav.stage = 'follow_wall'
                nav.wall_side = 0
                nav.wall_heading = 0
                self.frame(nav, 2, readings=(750, 750))
                self.frame(nav, 4, readings=readings)
                self.assertIsNone(nav.both_seen_since)
                self.frame(nav, 4.1, readings=(750, 750))
                self.frame(nav, 7, readings=(750, 750))
                self.assertFalse(nav.finished)
                self.frame(nav, 7.2, readings=(750, 750))
                self.assertTrue(nav.succeeded)

    def test_missing_sensor_data_restarts_wall_confirmation(self):
        nav = self.navigation()
        nav.stage = 'enter'
        nav.wall_side = 0
        nav.wall_heading = 0
        self.frame(nav, 2, readings=(750, 750))
        nav.update(status(), 4, None)
        self.assertIsNone(nav.both_seen_since)
        self.frame(nav, 4.1, readings=(750, 750))
        self.frame(nav, 7, readings=(750, 750))
        self.assertFalse(nav.finished)
        self.frame(nav, 7.2, readings=(750, 750))
        self.assertTrue(nav.succeeded)

    def test_ramp_clearance_moves_forward_before_starting_scan(self):
        nav = self.navigation()
        nav.change_stage('clear_ramp', 2)
        self.frame(nav, 2, readings=(100, 100), heading=.6)
        self.assertEqual(nav.stage, 'clear_ramp')
        self.assertEqual(nav.estimate.heading, 0)
        self.assertEqual(nav.forward, 1)
        nav.position = (RAMP_CLEARANCE_M + .001) * np.array([math.cos(.6), math.sin(.6)])
        self.frame(nav, 2.1, readings=(100, 100), heading=.6)
        self.assertEqual(nav.stage, 'scan')
        self.assertEqual(nav.turn_degrees, 0)

    def test_course_missing_walls_start_clearance_after_ten_seconds_at_half_speed(self):
        for readings in ((100, 750), (750, 100), (100, 100)):
            nav = IRWallNavigation(0, **load_corridor_settings()['navigation'])
            nav.set_speed(.5)
            nav.finish_on_missing_walls = True
            for now in (2, 5, 11.9):
                self.frame(nav, now, readings=readings)
                self.assertFalse(nav.exit_complete)
            self.frame(nav, 12, readings=readings)
            self.assertFalse(nav.exit_complete)
            self.assertIsNotNone(nav.exit_clearance_heading)
            self.assertEqual(nav.exit_clearance_distance, 0)

    def test_both_walls_reset_course_missing_wall_timer(self):
        nav = IRWallNavigation(0, **load_corridor_settings()['navigation'])
        nav.finish_on_missing_walls = True
        self.frame(nav, 2, readings=(100, 750))
        self.frame(nav, 7, readings=(750, 750))
        self.assertIsNone(nav.missing_walls_since)
        for now in (8, 17.9):
            self.frame(nav, now, readings=(100, 750))
            self.assertFalse(nav.exit_complete)
        self.frame(nav, 18, readings=(100, 750))
        self.assertFalse(nav.exit_complete)
        self.assertIsNotNone(nav.exit_clearance_heading)

    def test_course_walks_twenty_cm_straight_before_finishing_at_both_speeds(self):
        for speed in (1, .5):
            nav = IRWallNavigation(0, **load_corridor_settings()['navigation'])
            nav.set_speed(speed)
            nav.finish_on_missing_walls = True
            self.frame(nav, 2, readings=(100, 750), heading=.6)
            self.assertTrue(self.frame(nav, 12, readings=(100, 750), heading=.6))
            steps = round(.20 / (nav.walk_speed * .1))
            for i in range(1, steps):
                self.assertTrue(self.frame(nav, 12 + i * .1, readings=(750, 750), heading=.7))
                self.assertFalse(nav.exit_complete)
                self.assertAlmostEqual(nav.estimate.heading, .1)
                self.assertEqual(nav.estimate.lateral, 0)
            self.assertFalse(self.frame(nav, 12 + steps * .1, readings=(750, 750), heading=.7))
            self.assertTrue(nav.exit_complete)
            self.assertAlmostEqual(nav.exit_clearance_distance, .20)

    def test_lost_imu_does_not_count_toward_exit_clearance(self):
        nav = IRWallNavigation(0, **load_corridor_settings()['navigation'])
        nav.finish_on_missing_walls = True
        self.frame(nav, 2, readings=(100, 750))
        self.frame(nav, 12, readings=(100, 750))
        packet = status() | dict(orientationValid=False)
        nav.update(packet, 12.1)
        nav.update(packet, 17)
        self.frame(nav, 17.1)
        self.assertEqual(nav.exit_clearance_distance, 0)

    def test_close_wall_does_not_finish_corridor_search(self):
        nav = self.navigation()
        for now in (2, 2.1, 2.2, 2.3):
            self.frame(nav, now, readings=(3300, 750))
        self.assertFalse(nav.finished)
        self.assertTrue(nav.moving)
        self.assertEqual(nav.stage, 'align_wall')

    def test_close_wall_does_not_stop_corridor_navigation(self):
        for readings in ((3300, 750), (750, 3300)):
            nav = IRWallNavigation(0, **load_corridor_settings()['navigation'])
            for i in range(8):
                self.assertTrue(self.frame(nav, 2 + i * .1, readings=readings))

    def test_opposite_ir_does_not_start_turn(self):
        nav = self.navigation()
        for now in (2, 2.1, 2.2):
            self.frame(nav, now, readings=(100, 1260))
        self.assertEqual(nav.stage, 'approach')
        self.assertIsNone(nav.wall_point)

    def test_lost_imu_or_camera_cannot_confirm_handover(self):
        nav = self.navigation()
        self.frame(nav, 2)
        packet = status() | dict(irAgeMs=0, irMillivolts=[1260, 256])
        nav.update(packet, 2.1, None)
        self.frame(nav, 2.2)
        self.assertEqual(nav.stage, 'approach')
        packet['orientationValid'] = False
        nav.update(packet, 2.3, (2.3, 2.3, self.recorded_image()))
        self.frame(nav, 2.4)
        self.assertEqual(nav.stage, 'approach')

    def test_handover_turns_immediately_without_changing_the_approach_goal(self):
        nav = self.navigation()
        goal = nav.approach_goal.copy()
        nav.acquire_wall(0, .24, 2)
        self.assertEqual(nav.stage, 'align_wall')
        np.testing.assert_allclose(nav.approach_goal, goal)
        self.frame(nav, 2.1, readings=(500, 100))
        self.assertNotEqual(nav.estimate.heading, 0)

    def test_turn_uses_opposite_ramp_heading_and_cannot_command_past_it(self):
        for start in (0, 2.8, -2.8):
            for side, direction in ((0, 1), (1, -1)):
                nav = self.navigation()
                nav.search_heading = start
                nav.imu_heading = start + direction * math.pi / 2
                nav.acquire_wall(side, .16, 2)
                self.assertAlmostEqual(math.cos(nav.wall_heading - start), -1)
                self.assertLessEqual(nav.turn_angle, math.pi)
                # Put the estimated corner where the old arc would keep circling.
                nav.imu_heading = start + direction * (math.pi - .2)
                nav.wall_point = nav.position + np.array([math.cos(start), math.sin(start)])
                nav.turn_at_wall(2.1, np.array([.16, .16]))
                self.assertGreaterEqual(direction * nav.estimate.heading, -.200001)
                self.frame(nav, 2.2, readings=(750, 100), heading=start + direction * math.pi)
                self.assertIn(nav.stage, ('enter', 'follow_wall'))
                self.assertEqual(nav.estimate.heading, 0)

    def test_reaching_and_passing_the_camera_target_cannot_start_the_turn(self):
        nav = self.navigation()
        nav.position = nav.approach_goal.copy()
        self.frame(nav, 2, readings=(100, 100))
        self.assertEqual(nav.stage, 'approach')
        self.assertIsNone(nav.wall_point)
        nav.position[0] += .1
        self.frame(nav, 2.1, readings=(100, 100))
        self.assertEqual(nav.stage, 'approach')
        self.assertEqual(nav.estimate.heading, 0)
        self.assertEqual(nav.forward, 1)

    def test_ir_confirmation_starts_the_turn_after_passing_the_camera_target(self):
        nav = self.navigation()
        nav.position = nav.approach_goal.copy()
        self.frame(nav, 2, readings=(100, 100))
        self.assertIsNone(nav.wall_point)
        for now in (2.1, 2.2, 2.3):
            self.frame(nav, now)
        self.assertEqual(nav.stage, 'align_wall')
        self.assertIsNotNone(nav.wall_point)
        self.assertEqual(nav.wall_side, 0)
        self.assertIsNotNone(nav.turn_previous_heading)

    def test_turn_responds_to_clearance_on_either_side(self):
        for side, sign in ((0, -1), (1, 1)):
            nav = self.navigation()
            nav.wall_heading = -sign * math.pi / 2
            nav.acquire_wall(side, .24, 2)
            commands = []
            for distance in (.20, .24, .28):
                ranges = [1., 1.]
                ranges[side] = distance
                nav.turn_at_wall(2.1, np.array(ranges))
                commands.append(sign * nav.estimate.heading)
            self.assertLess(commands[0], commands[1])
            self.assertLess(commands[1], commands[2])

    def test_entry_follows_either_wall_at_twenty_four_cm(self):
        for side, sign in ((0, -1), (1, 1)):
            nav = self.navigation()
            nav.acquire_wall(side, .24, 2)
            for distance in (.20, .24, .28):
                ranges = np.ones(2)
                ranges[side] = distance
                self.assertTrue(nav.follow_wall(2.1, ranges))
                self.assertAlmostEqual(nav.estimate.lateral, sign * (distance - .24))

    def test_brief_loss_uses_the_last_wall_point_to_complete_the_turn(self):
        nav = self.navigation()
        nav.acquire_wall(0, .12, 2)
        nav.stage = 'align_wall'
        original = nav.wall_point.copy()
        self.frame(nav, 2.1, readings=(100, 100))
        self.assertEqual(nav.stage, 'align_wall')
        self.assertGreater(nav.forward, 0)
        np.testing.assert_allclose(nav.wall_point, original)

    def test_turn_finishes_from_imu_heading_then_follows_the_wall(self):
        nav = self.navigation()
        nav.acquire_wall(0, .12, 2)
        nav.stage = 'align_wall'
        nav.wall_heading = .5
        nav.wall_point = np.array([-.2, .2])
        self.frame(nav, 2.1, readings=(1000, 100), heading=.5)
        self.assertEqual(nav.stage, 'follow_wall')
        self.assertEqual(nav.forward, 0)
        self.frame(nav, 2.2, readings=(1000, 100), heading=.5)
        self.assertEqual(nav.forward, 1)

    def test_matching_corridor_heading_finishes_even_if_clearance_estimate_is_wrong(self):
        nav = self.navigation()
        nav.acquire_wall(0, .16, 2)
        self.frame(nav, 2.1, readings=(750, 100), heading=nav.wall_heading)
        self.assertIn(nav.stage, ('enter', 'follow_wall'))

    def test_entry_keeps_short_steps_until_the_side_sensor_sees_the_wall(self):
        nav = self.navigation()
        nav.stage = 'enter'
        nav.stage_started = 2
        nav.wall_heading = 0
        nav.wall_side = 0
        self.frame(nav, 2.1, readings=(100, 100))
        self.assertEqual(nav.stage, 'enter')
        self.assertEqual(nav.forward, .5)

    def test_half_speed_entry_allows_time_for_the_short_steps(self):
        nav = self.navigation()
        nav.set_speed(.5)
        nav.stage = 'enter'
        nav.stage_started = 2
        nav.wall_heading = 0
        nav.wall_side = 0
        self.frame(nav, 67, readings=(100, 100))
        self.assertEqual(nav.stage, 'enter')
        self.assertTrue(nav.moving)
        self.assertEqual(nav.forward, .5)



if __name__ == '__main__':
    unittest.main()
