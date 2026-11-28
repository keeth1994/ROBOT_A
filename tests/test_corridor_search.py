"""Corridor acquisition: full scan, near-wall handover and persistent IR entry."""
import math
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image, ImageDraw

from dashboard.robot_dashboard.corridor_search import CorridorSearch, visible_wall_ends, wall_view
from dashboard.robot_dashboard.corridor_walls import load_corridor_settings
from tests.test_dashboard_navigation import status
from robot.navigation.camera_calibration import load_camera_scan_settings, servo_to_cradle_deg


def wall_image(start=192, end=320):
    image = np.full((240, 320, 3), 180, dtype=np.uint8)
    image[100:210, start:end] = (180, 120, 50)
    return image


def candidate(point, samples=5):
    return dict(point=np.array(point, dtype=float), tangent=np.array([1., 0.]),
                normal=np.array([0., 1.]), samples=samples)


class CorridorSearchTests(unittest.TestCase):
    def navigation(self):
        nav = CorridorSearch(0, load_corridor_settings()['navigation'])
        nav.approach_walk_time = 10
        packet = status(angle=-80)
        packet['imuAgeMs'] = packet['irAgeMs'] = 0
        nav.update(packet, 0, (0, 0, wall_image()))
        return nav, packet

    def frame(self, nav, packet, now):
        return nav.update(packet, now, (now, now, wall_image()))

    def test_scan_completes_a_full_circle_before_approaching_any_wall(self):
        nav, packet = self.navigation()
        end = (np.array([1., .2]), np.array([1., 0.]), np.array([0., 1.]))
        with patch.object(nav, 'observe_ends', return_value=[end]):
            for i, yaw in enumerate((0, -1, -2, -3, -4, -5, -6)):
                packet['orientation'][2] = yaw
                self.frame(nav, packet, 2 + i * .1)
                self.assertEqual(nav.stage, 'scan')
                self.assertEqual(nav.forward, 0)
            packet['orientation'][2] = -6.3
            self.frame(nav, packet, 2.8)
        self.assertGreaterEqual(nav.turn_degrees, 360)
        self.assertEqual(nav.stage, 'approach')

    def test_scan_handles_wrapped_imu_angles_without_counting_backtracking(self):
        nav, packet = self.navigation()
        nav.candidates = [candidate([1., .2])]
        for i, yaw in enumerate((0, -2, 2.28, .28)):
            packet['orientation'][2] = yaw
            self.frame(nav, packet, 2 + i * .1)
        self.assertAlmostEqual(nav.turn_degrees, math.degrees(2 * math.pi - .28), places=1)
        packet['orientation'][2] = .38
        self.frame(nav, packet, 2.4)
        self.assertLess(nav.turn_degrees, 360)
        packet['orientation'][2] = -.05
        self.frame(nav, packet, 2.5)
        self.assertEqual(nav.stage, 'approach')

    def test_nearest_wall_is_selected_even_when_farther_wall_was_seen_first(self):
        nav, _ = self.navigation()
        far, near = candidate([1.5, .8]), candidate([.5, .2])
        nav.candidates = [far, near]
        self.assertTrue(nav.choose_entrance())
        self.assertIs(nav.selected, near)

    def test_stable_wall_end_wins_over_a_slightly_closer_intermittent_end(self):
        nav, _ = self.navigation()
        intermittent = candidate([1., .1], samples=4)
        stable = candidate([1.04, .1], samples=20)
        nav.candidates = [intermittent, stable]
        self.assertTrue(nav.choose_entrance())
        self.assertIs(nav.selected, stable)

    def test_parallel_wall_pair_targets_the_entrance_instead_of_either_outer_face(self):
        nav, _ = self.navigation()
        nav.candidates = [candidate([1., .2]), candidate([1., -.2])]
        self.assertTrue(nav.choose_entrance())
        np.testing.assert_allclose(nav.approach_goal, [.75, 0])

    def test_front_approach_stays_on_the_visible_inside_face(self):
        nav, _ = self.navigation()
        wall = candidate([1., .2])
        wall['normal'] = np.array([0., -1.])
        nav.candidates = [wall]
        nav.choose_entrance()
        np.testing.assert_allclose(nav.approach_goal, [.82, -.05], atol=1e-8)

    def test_wall_top_recovers_a_known_end_when_the_base_is_below_the_image(self):
        camera = load_corridor_settings()['camera']
        f = 240 / (2 * math.tan(math.radians(camera['fovy_deg']) / 2))
        angle = math.radians(camera['pitch_deg'])
        c, s = math.cos(angle), math.sin(angle)
        mount = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        points = np.array([[.5, -.08, camera['wall_height_m']], [1., -.08, camera['wall_height_m']],
                           [1., -.08, 0], [.5, -.08, 0]])
        rays = (points - [camera['forward_m'], 0, camera['height_m']]) @ mount
        pixels = np.column_stack([159.5 - f * rays[:, 1] / rays[:, 0],
                                  119.5 - f * rays[:, 2] / rays[:, 0]])
        self.assertGreater(pixels[-1, 1], 240)
        image = Image.new('RGB', (320, 240), (180, 180, 180))
        ImageDraw.Draw(image).polygon([tuple(p) for p in pixels], fill=(180, 120, 50))
        for border in (0, 2):
            with self.subTest(bottom_border=border):
                rgb = np.array(image)
                if border:
                    rgb[-border:] = 0
                ends = visible_wall_ends(wall_view(rgb), camera, 0, 0)
                self.assertTrue(ends)
                # Very short columns at the endpoint are trimmed by the wall-height filter.
                np.testing.assert_allclose(ends[0][0], [.5, -.08], atol=.04)

    def test_thin_plywood_strip_is_not_a_wall_endpoint(self):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[170:173, 50:270] = (180, 120, 50)
        self.assertEqual(visible_wall_ends(wall_view(image), load_corridor_settings()['camera'], 0, 0), [])

    def test_visible_height_recovers_wall_with_an_incorrect_camera_pitch(self):
        camera = load_corridor_settings()['camera']
        f = 240 / (2 * math.tan(math.radians(camera['fovy_deg']) / 2))
        angle = math.radians(20)
        c, s = math.cos(angle), math.sin(angle)
        mount = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        points = np.array([[1., -.12, .17], [1., -.6, .17], [1., -.6, 0], [1., -.12, 0]])
        rays = (points - [camera['forward_m'], 0, camera['height_m']]) @ mount
        pixels = np.column_stack([159.5 - f * rays[:, 1] / rays[:, 0],
                                  119.5 - f * rays[:, 2] / rays[:, 0]])
        image = Image.new('RGB', (320, 240), (180, 180, 180))
        ImageDraw.Draw(image).polygon([tuple(p) for p in pixels], fill=(180, 120, 50))
        view = wall_view(np.asarray(image))
        ends = visible_wall_ends(view, camera, 0, 0)
        self.assertEqual(len(ends), 1)
        np.testing.assert_allclose(ends[0][0], [1., -.12], atol=.15)
        self.assertGreater(ends[0][1] @ [0, -1], .8)
        self.assertEqual(view['wall_side'], 1)

    def test_grey_support_does_not_become_a_wall_end_above_the_horizon(self):
        image = Image.new('RGB', (320, 240), (180, 180, 180))
        draw = ImageDraw.Draw(image)
        draw.rectangle((192, 60, 319, 110), fill=(180, 120, 50))
        draw.polygon(((205, 110), (215, 85), (225, 110)), fill=(100, 100, 100))
        view = wall_view(np.asarray(image))
        self.assertEqual(len(visible_wall_ends(view, load_corridor_settings()['camera'], 0, 0)), 1)
        self.assertEqual(view['end_pixels'][0][0], 192)

    def test_wall_clipped_on_both_sides_does_not_invent_an_entrance(self):
        image = wall_image(0, 320)
        image[60:110] = (180, 120, 50)
        self.assertEqual(visible_wall_ends(wall_view(image), load_corridor_settings()['camera'], 0, 0), [])

    def test_approach_accepts_stable_ir_at_the_edge_of_its_working_range(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([1., 0.])
        packet['irMillivolts'][1] = 120 / .295
        for now in (2, 2.1, 2.2):
            self.frame(nav, packet, now)
        self.assertEqual(nav.stage, 'clear_end')
        self.assertEqual(nav.wall_side, 1)

    def test_slanted_plywood_borders_on_the_ramp_are_not_wall_ends(self):
        camera = load_corridor_settings()['camera']
        image = Image.new('RGB', (320, 240), (180, 180, 180))
        draw = ImageDraw.Draw(image)
        for points in (((130, 0), (30, 239)), ((210, 0), (290, 239))):
            draw.line(points, fill=(180, 120, 50), width=2)
        view = wall_view(np.asarray(image), camera_pitch=math.radians(camera['pitch_deg']),
                         fovy_deg=camera['fovy_deg'])
        self.assertEqual(visible_wall_ends(view, camera, 0, 0), [])

    def test_downward_camera_detects_wall_ends_high_in_the_image(self):
        camera = load_corridor_settings()['camera'] | {'pitch_deg': 30}
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[20:75, 192:] = (180, 120, 50)
        view = wall_view(image, camera_pitch=math.radians(camera['pitch_deg']),
                         fovy_deg=camera['fovy_deg'])
        self.assertEqual(view['wall_side'], 1)
        self.assertTrue(visible_wall_ends(view, camera, 0, 0))

    def test_distant_wall_above_an_opening_does_not_hide_the_entrance(self):
        camera = load_corridor_settings()['camera']
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[20:75] = (180, 120, 50)
        image[100:220, :80] = image[100:220, 240:] = (180, 120, 50)
        view = wall_view(image, camera_pitch=math.radians(camera['pitch_deg']),
                         fovy_deg=camera['fovy_deg'])
        self.assertEqual(view['kind'], 'opening')
        self.assertAlmostEqual(view['target'], .5)

    def test_camera_side_ir_starts_turn_despite_an_inaccurate_visual_position(self):
        for side in (0, 1):
            with self.subTest(side=side):
                nav, packet = self.navigation()
                nav.candidates = [candidate([1.5, .8])]
                nav.choose_entrance()
                nav.stage = 'approach'
                nav.approach_goal = np.array([1., 0.])
                packet['irMillivolts'][side] = 800
                image = wall_image(0, 128) if side == 0 else wall_image()
                for now in (2, 2.1, 2.2):
                    nav.update(packet, now, (now, now, image))
                self.assertEqual(nav.wall_side, side)
                self.assertEqual(nav.stage, 'clear_end')
                self.assertIsNotNone(nav.wall_point)

    def test_wall_extending_out_of_one_image_edge_selects_that_side_ir(self):
        for start, end, side in ((192, 320, 1), (0, 128, 0)):
            with self.subTest(side=side):
                view = wall_view(wall_image(start, end))
                self.assertEqual(view['wall_side'], side)

    def test_stronger_opposite_ir_cannot_trigger_the_approach_handover(self):
        for side in (0, 1):
            with self.subTest(side=side):
                nav, packet = self.navigation()
                nav.stage, nav.wall_heading = 'approach', 0
                nav.approach_goal = np.array([1., 0.])
                packet['irMillivolts'][1 - side] = 1500
                image = wall_image(0, 128) if side == 0 else wall_image()
                for now in (2, 2.2, 2.5, 3):
                    nav.update(packet, now, (now, now, image))
                    self.assertEqual(nav.stage, 'approach')
                    self.assertIsNone(nav.wall_side)
                self.assertEqual(nav.approach_side, side)
                packet['irMillivolts'][side] = 800
                for now in (3.1, 3.3):
                    nav.update(packet, now, (now, now, image))
                self.assertEqual(nav.wall_side, side)
                self.assertEqual(nav.stage, 'clear_end')

    def test_camera_wall_side_is_remembered_when_wall_leaves_the_frame(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([1., 0.])
        self.frame(nav, packet, 2)
        blank = np.full((240, 320, 3), 180, dtype=np.uint8)
        packet['irMillivolts'] = [1500, 100]
        for now in (2.1, 2.3):
            nav.update(packet, now, (now, now, blank))
        self.assertEqual(nav.approach_side, 1)
        self.assertEqual(nav.stage, 'approach')
        packet['irMillivolts'][1] = 800
        for now in (2.4, 2.6):
            nav.update(packet, now, (now, now, blank))
        self.assertEqual(nav.wall_side, 1)
        self.assertEqual(nav.stage, 'clear_end')

    def test_two_ir_readings_cannot_confirm_entry_before_the_wall_end_turn(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'align_wall', 2
        nav.wall_side, nav.wall_heading = 1, math.pi / 2
        nav.approach_goal = np.array([1., 0.])
        packet['irMillivolts'] = [1000, 1000]
        for now in (2, 2.3, 2.7):
            self.frame(nav, packet, now)
        self.assertEqual(nav.stage, 'align_wall')
        self.assertIsNone(nav.both_seen_since)

    def test_turning_around_wall_end_changes_sensor_using_wall_position(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'align_wall', -math.pi / 2
        nav.wall_point = np.array([0., -.2])
        nav.position = np.array([.2, -.3])
        nav.wall_side = 1
        packet['orientation'][2] = -math.pi / 2
        packet['irMillivolts'] = [800, 1500]
        self.frame(nav, packet, 2)
        self.assertEqual(nav.stage, 'follow_wall')
        self.assertEqual(nav.wall_side, 1)
        # Crossing to the other face of this wall changes its side after the turn.
        nav.stage = 'align_wall'
        nav.position[0] = -.2
        self.frame(nav, packet, 2.1)
        self.assertEqual(nav.wall_side, 0)

    def test_entering_does_not_follow_an_unrelated_opposite_ir_reading(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'enter', 2
        nav.wall_side, nav.wall_heading = 1, 0
        packet['irMillivolts'] = [1500, 100]
        for now in (2, 2.2, 2.5):
            self.frame(nav, packet, now)
            self.assertEqual(nav.stage, 'enter')
            self.assertEqual(nav.wall_side, 1)

    def test_side_ir_starts_the_turn_before_continuing_across_the_entrance(self):
        nav, packet = self.navigation()
        nav.stage = 'approach'
        nav.wall_heading = math.pi / 2
        nav.approach_goal = np.array([1., 0.])
        packet['irMillivolts'] = [100, 800]
        for now in (2, 2.1, 2.2):
            self.frame(nav, packet, now)
        self.assertEqual(nav.stage, 'clear_end')
        np.testing.assert_allclose(nav.approach_goal, [1., 0.])

    def test_wall_following_uses_clearance_and_does_not_switch_back_to_camera_target(self):
        for side, sign in ((0, -1), (1, 1)):
            with self.subTest(side=side):
                nav, packet = self.navigation()
                nav.stage, nav.stage_started = 'follow_wall', 2
                nav.wall_side, nav.wall_heading = side, 0
                for now, volts, expected in ((2, 600, sign), (2.1, 1000, 0), (2.2, 1500, -sign)):
                    packet['irMillivolts'][side] = volts
                    self.frame(nav, packet, now)
                    self.assertEqual(np.sign(nav.estimate.lateral), expected)
                    self.assertEqual(nav.forward, 1)
                    self.assertEqual(nav.wall_side, side)

    def test_short_opposite_ir_pulse_does_not_confirm_entry(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'follow_wall', 2
        nav.wall_side, nav.wall_heading = 1, 0
        packet['irMillivolts'] = [1000, 1000]
        for now in (2, 2.2, 2.4):
            self.frame(nav, packet, now)
            self.assertEqual(nav.stage, 'follow_wall')
        packet['irMillivolts'] = [100, 1000]
        self.frame(nav, packet, 2.5)
        self.assertIsNone(nav.both_seen_since)
        packet['irMillivolts'] = [1000, 1000]
        for now in (2.6, 2.8, 3, 3.21):
            self.frame(nav, packet, now)
        self.assertEqual(nav.stage, 'center')

    def test_lost_ir_holds_direction_briefly_then_stops_instead_of_circling(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'follow_wall', 2
        nav.wall_side, nav.wall_heading = 1, 0
        nav.last_wall_seen = 2
        self.assertTrue(self.frame(nav, packet, 2.2))
        self.assertEqual(nav.estimate.heading, 0)
        self.assertFalse(self.frame(nav, packet, 5.2))
        self.assertEqual(nav.forward, 0)
        self.assertEqual(nav.estimate.heading, 0)

    def test_search_uses_requested_speed_up_to_normal_walking_speed(self):
        nav, _ = self.navigation()
        for requested, expected in ((.3, .3), (.8, .8), (1, 1), (2, 1)):
            nav.set_speed(requested)
            self.assertEqual(nav.speed, expected)

    def test_straight_approach_is_faster_until_the_camera_side_ir_sees_a_wall(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([1., 0.])
        self.frame(nav, packet, 2)
        self.assertEqual(nav.speed, 1.5)
        packet['irMillivolts'] = [1500, 100]
        self.frame(nav, packet, 2.1)
        self.assertEqual(nav.speed, 1.5)
        packet['irMillivolts'][1] = 800
        self.frame(nav, packet, 2.2)
        self.assertEqual(nav.stage, 'approach')
        self.assertEqual(nav.speed, 1)
        self.frame(nav, packet, 2.4)
        self.assertEqual(nav.stage, 'clear_end')
        self.assertEqual(nav.speed, 2)

    def test_approach_turns_and_stale_frames_do_not_use_the_faster_pace(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([0., 1.])
        self.frame(nav, packet, 2)
        self.assertEqual(nav.forward, 0)
        self.assertEqual(nav.speed, 1)
        nav.approach_goal = np.array([1., 0.])
        self.frame(nav, packet, 2.1)
        self.assertEqual(nav.speed, 1.5)
        self.assertFalse(nav.update(packet, 2.2, None))
        self.assertEqual(nav.speed, 1)

    def test_approach_odometry_uses_the_previous_faster_command(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([1., 0.])
        self.frame(nav, packet, 2)
        nav.set_speed(1)
        self.frame(nav, packet, 2.1)
        self.assertAlmostEqual(nav.position[0], nav.settings['walk_speed'] * 1.5 * .1)
        self.assertAlmostEqual(nav.position[1], 0)

    def test_missing_or_stale_sensors_hold_steps(self):
        nav, packet = self.navigation()
        self.assertFalse(nav.update(packet, 2, None))
        packet['irAgeMs'] = 500
        self.assertFalse(self.frame(nav, packet, 2.1))

    def test_occluded_visual_target_cannot_drive_into_a_wall(self):
        nav, packet = self.navigation()
        nav.stage = 'approach'
        nav.wall_heading = 0
        nav.approach_goal = np.array([1., 0.])
        image = wall_image(0, 320)
        self.assertFalse(nav.update(packet, 2, (2, 2, image)))
        self.assertEqual(nav.forward, 0)

    def test_distant_wall_does_not_stop_an_approach_over_clear_floor(self):
        nav, packet = self.navigation()
        nav.stage, nav.wall_heading = 'approach', 0
        nav.approach_goal = np.array([1., 0.])
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[100:175] = (180, 120, 50)
        self.assertTrue(nav.update(packet, 2, (2, 2, image)))
        self.assertEqual(nav.forward, 1)

    def test_level_camera_stops_only_for_a_wall_before_the_approach_goal(self):
        for distance, expected in ((.7, False), (1.3, True)):
            with self.subTest(wall_distance=distance):
                nav, packet = self.navigation()
                nav.stage, nav.wall_heading = 'approach', 0
                nav.approach_goal = np.array([1., 0.])
                camera = nav.camera_geometry
                f = 240 / (2 * math.tan(math.radians(camera['fovy_deg']) / 2))
                pitch = math.radians(camera['pitch_deg'])
                depth = distance - camera['forward_m']
                top = 119.5 + f * math.tan(math.atan2(camera['height_m'] - camera['wall_height_m'], depth) - pitch)
                bottom = 119.5 + f * math.tan(math.atan2(camera['height_m'], depth) - pitch)
                image = np.full((240, 320, 3), 180, dtype=np.uint8)
                image[max(0, round(top)):min(240, round(bottom))] = (180, 120, 50)
                self.assertEqual(nav.update(packet, 2, (2, 2, image)), expected)

    def test_search_projection_matches_the_physical_camera_command(self):
        nav, _ = self.navigation()
        calibration = load_camera_scan_settings()
        commanded_pitch = (math.degrees(nav.orientation.camera_base_pitch) +
                           servo_to_cradle_deg(calibration['corridor_servo_deg'], calibration))
        self.assertAlmostEqual(nav.camera_geometry['pitch_deg'], commanded_pitch, places=6)

    def test_centering_cannot_report_success_while_gyro_heading_is_off_the_wall_direction(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'center', 2
        nav.entry_heading = 0
        packet['irMillivolts'] = [1000, 1000]
        packet['orientation'][2] = .22
        for now in (2, 3, 4, 5, 6):
            self.frame(nav, packet, now)
            self.assertFalse(nav.succeeded)
            self.assertAlmostEqual(nav.estimate.heading, .22)

    def test_centering_confirms_a_balanced_trot_despite_normal_heading_oscillation(self):
        nav, packet = self.navigation()
        nav.stage, nav.stage_started = 'center', 2
        nav.entry_heading = 0
        packet['irMillivolts'] = [1000, 1000]
        for now in np.arange(2, 7, .05):
            packet['orientation'][2] = .10 * math.sin(2 * math.pi * now)
            self.frame(nav, packet, float(now))
            if nav.finished:
                break
        self.assertTrue(nav.succeeded)
        packet['irAgeMs'] = 0
        packet['orientationValid'] = False
        self.assertFalse(self.frame(nav, packet, 2.2))
        self.assertEqual(nav.forward, 0)


if __name__ == '__main__':
    unittest.main()
