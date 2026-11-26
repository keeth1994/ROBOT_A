"""Project a known ramp through a tilted, offset camera and recover body position."""
import math
import unittest

import numpy as np
from PIL import Image, ImageDraw

from robot_b91.navigation import Navigation


def rotation(roll, pitch, yaw=0):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def ramp_image(roll, pitch, yaw, body_y, lens, camera_pitch_deg=30, slope=0):
    body_rotation = rotation(roll, pitch, yaw)
    camera_rotation = body_rotation @ rotation(0, math.radians(camera_pitch_deg))
    camera_position = np.array([0., body_y, .3 if lens[2] == 0 else .05]) + body_rotation @ lens
    focal = 240 / (2*math.tan(math.radians(42.9470936)/2))
    points = []
    sign = 1 if math.cos(math.radians(camera_pitch_deg)) >= 0 else -1
    for x, y in ((sign*.25, .145), (sign*4, .145), (sign*4, -.145), (sign*.25, -.145)):
        ray = camera_rotation.T @ (np.array([x, y, slope*x])-camera_position)
        points.append((159.5-focal*ray[1]/ray[0], 119.5-focal*ray[2]/ray[0]))
    image = Image.new('RGB', (320,240), (200,200,200))
    ImageDraw.Draw(image).polygon(points, fill=(15,15,15))
    return np.array(image)


class CameraBodyOffsetTests(unittest.TestCase):
    def test_roll_and_pitch_do_not_become_false_body_drift(self):
        lens = np.array([-.013, .015, .244])
        for roll_deg, pitch_deg, yaw_deg in ((-10,0,0), (10,0,0), (8,15,5), (-8,-15,-5), (0,0,0)):
            for body_y in (-.03, 0, .03):
                with self.subTest(attitude=(roll_deg,pitch_deg,yaw_deg), body_y=body_y):
                    roll, pitch, yaw = np.radians([roll_deg,pitch_deg,yaw_deg])
                    nav = Navigation(width_m=.29, camera_pitch_deg=30,
                                     camera_x_m=lens[0], camera_y_m=lens[1], camera_z_m=lens[2])
                    nav.roll, nav.pitch = roll, pitch
                    rgb = ramp_image(roll,pitch,yaw,body_y,lens)
                    self.assertTrue(nav.frame(3,rgb), nav.debug['reason'])
                    self.assertAlmostEqual(nav.lateral/.3, body_y, delta=.004)

    def test_default_zero_height_preserves_existing_flat_camera_behavior(self):
        lens = np.array([.0626, 0, 0])
        nav = Navigation(width_m=.29, camera_pitch_deg=30)
        self.assertTrue(nav.frame(3, ramp_image(0,0,0,.02,lens)))
        self.assertAlmostEqual(nav.lateral/.3, .02, delta=.004)

    def test_forward_and_backward_views_on_slopes(self):
        for camera_pitch in (30, 158):
            lens = np.array([-.017, 0, .247] if camera_pitch == 30 else [-.021, 0, .193])
            for roll_deg, pitch_deg, yaw_deg, slope in ((6,10,4,.2), (-6,-10,-4,-.2)):
                with self.subTest(camera_pitch=camera_pitch, slope=slope):
                    roll, pitch, yaw = np.radians([roll_deg,pitch_deg,yaw_deg])
                    nav = Navigation(width_m=.29, camera_pitch_deg=camera_pitch,
                                     camera_x_m=lens[0], camera_z_m=lens[2])
                    nav.view_sign = 1 if camera_pitch == 30 else -1
                    nav.roll, nav.pitch = roll, pitch
                    rgb = ramp_image(roll,pitch,yaw,.025,lens,camera_pitch,slope)
                    self.assertTrue(nav.frame(3,rgb), nav.debug['reason'])
                    self.assertAlmostEqual(nav.lateral/.3, .025, delta=.004)

    def test_camera_search_does_not_discard_established_imu_heading(self):
        lens = np.array([-.013, 0, .244])
        nav = Navigation(width_m=.29, camera_pitch_deg=30, camera_x_m=lens[0], camera_z_m=lens[2])
        self.assertTrue(nav.frame(3,ramp_image(0,0,0,0,lens)))
        self.assertTrue(nav.heading_initialized)
        established = nav.heading
        nav.last_seen = None  # The camera servo moved to another view.
        nav.searching = True
        nav.frame(5,ramp_image(0,0,math.radians(10),0,lens))
        self.assertAlmostEqual(nav.heading, established)
        # A real turn reported by the gyro remains usable after the view change.
        nav.heading = math.radians(10)
        self.assertTrue(nav.frame(5.2,ramp_image(0,0,math.radians(10),0,lens)), nav.debug['reason'])
        self.assertAlmostEqual(nav.heading, math.radians(10), delta=.005)

    def test_heading_bias_cannot_reverse_edge_based_centering(self):
        lens = np.array([-.017, 0, .247])
        for view in (30, 158):
            for body_y in (-.04, .04):
                for bias_deg in (-8, 8):
                    with self.subTest(view=view, offset=body_y, bias=bias_deg):
                        roll, pitch, yaw = np.radians([4, -15, 2])
                        nav = Navigation(width_m=.29, camera_pitch_deg=view,
                                         camera_x_m=lens[0], camera_z_m=lens[2])
                        nav.view_sign = 1 if view == 30 else -1
                        nav.heading_initialized = True
                        nav.heading = yaw + math.radians(bias_deg)
                        nav.roll, nav.pitch = roll, pitch
                        rgb = ramp_image(roll, pitch, yaw, body_y, lens, view, -.2)
                        self.assertTrue(nav.frame(5, rgb), nav.debug['reason'])
                        self.assertEqual(nav.debug['lateral_source'], 'paired_edges')
                        self.assertAlmostEqual(nav.debug['measured_lateral_m'], body_y, delta=.004)
                        self.assertAlmostEqual(nav.heading, yaw+math.radians(bias_deg))

    def test_narrow_edge_view_reports_heading_dependent_fallback(self):
        image = np.full((240, 320, 3), 200, dtype=np.uint8)
        image[80:110, 100:220] = 15
        nav = Navigation(camera_pitch_deg=0)
        self.assertTrue(nav.frame(3, image))
        self.assertEqual(nav.debug['lateral_source'], 'tracked_heading')
        self.assertIsNone(nav.debug['visual_heading_rad'])


if __name__ == '__main__':
    unittest.main()
