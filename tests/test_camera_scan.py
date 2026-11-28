"""Check scan geometry, ramp memory and the level/no-ramp stop rule."""
import json
import math
import unittest

import numpy as np

from dashboard.robot_dashboard.navigation import CourseNavigation
from robot.navigation.camera_scan import RampScanNavigation, flip_target
from robot.navigation.camera_calibration import load_camera_scan_settings, servo_to_cradle_deg
from robot.paths import ROOT


SETTINGS = load_camera_scan_settings()
CONFIG = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
FORWARD, BACKWARD = SETTINGS['forward_deg'], SETTINGS['backward_deg']


def ramp_image(pitch, lateral=0):
    """Project a known-width strip into the camera; no detector internals used."""
    h, w = 240, 320
    y, x = np.mgrid[:h, :w]
    f = h / (2 * math.tan(math.radians(CONFIG['fovy_deg']) / 2))
    rays = np.stack([np.ones_like(x), -(x-(w-1)/2)/f, -(y-(h-1)/2)/f], axis=-1)
    c, s = math.cos(pitch), math.sin(pitch)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    rays = rays @ rotation.T
    scale = -.24 / np.where(abs(rays[:,:,2]) > 1e-8, rays[:,:,2], 1e-8)
    points = rays * scale[:,:,None]
    ramp = ((scale > 0) & (abs(points[:,:,0]) < 2) &
            (abs(points[:,:,1] + lateral) < CONFIG['width_m']/2))
    return np.repeat(np.where(ramp, 15, 180)[:,:,None], 3, axis=2).astype(np.uint8)


class CameraScanTests(unittest.TestCase):
    def test_measured_servo_commands_map_to_camera_views(self):
        self.assertEqual(SETTINGS['backward_servo_deg'], -4)
        self.assertEqual(SETTINGS['forward_servo_deg'], -65)
        self.assertEqual(SETTINGS['corridor_servo_deg'], -80)
        self.assertEqual(servo_to_cradle_deg(-4, SETTINGS), 118)
        self.assertEqual(servo_to_cradle_deg(-65, SETTINGS), -10)
        self.assertEqual(servo_to_cradle_deg(-34.5, SETTINGS), 54)

    def test_dashboard_accepts_forward_view_and_recognizes_backward_view(self):
        nav = CourseNavigation(0, use_imu=False)
        nav.update({'cameraPitch': {'angle': -65}}, 2, None)
        self.assertEqual(nav.camera_angle, -65)
        self.assertAlmostEqual(math.degrees(nav.estimate.camera_pitch), 30)
        self.assertEqual(nav.estimate.view_sign, 1)
        nav.update({'cameraPitch': {'angle': -4}}, 3, None)
        self.assertAlmostEqual(math.degrees(nav.estimate.camera_pitch), 158)
        self.assertEqual(nav.estimate.view_sign, -1)
        nav.update({'cameraPitch': {'angle': -81}}, 4, None)
        self.assertIsNone(nav.camera_angle)
        nav.update({'cameraPitch': {'angle': 55}}, 5, None)
        self.assertIsNone(nav.camera_angle)

    def navigation(self):
        return RampScanNavigation(SETTINGS, **(CONFIG | {'camera_pitch_deg':40}))

    def frame(self, nav, time, angle, image=None, speed=0):
        nav.imu(time, [0,0,9.81], [0,0,0])
        return nav.scan_frame(time, ramp_image(math.radians(40+angle)) if image is None else image,
                              math.radians(angle), speed, 0)

    def test_flip_crosses_downward_region_quickly_and_stops_at_endpoint(self):
        sweep = SETTINGS['sweep_s']
        step = .0001
        for start, end in ((FORWARD, BACKWARD), (BACKWARD, FORWARD)):
            start, end = math.radians(start), math.radians(end)
            target = lambda t: flip_target(t, start, end, SETTINGS)
            self.assertAlmostEqual(target(0), start)
            self.assertAlmostEqual(target(sweep+10), end)
            rate = abs(target(sweep/2+step)-target(sweep/2-step))/(2*step)
            self.assertGreater(rate, 2.9*math.radians(abs(BACKWARD-FORWARD))*math.pi/(2*sweep))
            self.assertAlmostEqual(target(sweep/2), math.radians((FORWARD+BACKWARD)/2))
            self.assertLess(abs(target(.001)-target(0)),1e-5)

    def test_starts_forward_and_stays_there_while_ramp_is_visible(self):
        nav=self.navigation()
        for t in np.arange(2,12,.1):
            self.assertTrue(self.frame(nav,t,FORWARD))
            self.assertTrue(nav.ready(t))
            self.assertAlmostEqual(nav.camera_target(t),math.radians(FORWARD))
        self.assertEqual(nav.view,'forward')
        self.assertFalse(nav.searching)

    def test_forward_and_backward_observations_agree(self):
        offsets=[]
        for angle in (FORWARD,BACKWARD):
            nav=self.navigation()
            for t in (2,2.1,2.2):
                self.assertTrue(self.frame(nav,t,angle,ramp_image(math.radians(40+angle),.025)))
            offsets.append(nav.lateral)
        np.testing.assert_allclose(offsets[0],offsets[1],atol=.001)
        self.assertGreater(offsets[0],.01)

    def lose_ramp(self,nav):
        self.frame(nav,2,FORWARD)
        blank=np.full((240,320,3),180,dtype=np.uint8)
        self.frame(nav,3.1,FORWARD,blank)
        return blank

    def test_short_loss_uses_cache_without_flipping(self):
        nav=self.navigation();self.frame(nav,2,FORWARD)
        blank=np.full((240,320,3),180,dtype=np.uint8)
        self.assertFalse(self.frame(nav,2.4,FORWARD,blank))
        self.assertTrue(nav.ready(2.4))
        self.assertTrue(nav.report()['using_cached_ramp'])
        self.frame(nav,2.8,FORWARD)
        self.assertFalse(nav.searching)
        self.assertEqual(nav.view,'forward')

    def test_loss_stops_and_briefly_confirmed_rear_detection_resumes(self):
        nav=self.navigation();self.lose_ramp(nav)
        self.assertTrue(nav.searching)
        self.assertFalse(nav.ready(3.1))
        self.assertEqual(nav.view,'backward')
        self.assertAlmostEqual(nav.camera_target(3.1),math.radians(FORWARD))
        self.assertAlmostEqual(nav.camera_target(4.1),math.radians((FORWARD+BACKWARD)/2))
        self.frame(nav,4.1,50)
        self.assertFalse(nav.ready(4.1))
        self.frame(nav,5.2,BACKWARD)
        self.assertFalse(nav.ready(5.2))
        self.frame(nav,5.4,BACKWARD)
        self.assertFalse(nav.ready(5.4))
        self.frame(nav,5.51,BACKWARD)
        self.assertFalse(nav.searching)
        self.assertTrue(nav.ready(5.51))
        self.assertAlmostEqual(nav.camera_target(20),math.radians(BACKWARD))

    def test_no_ramp_keeps_walking_stopped_and_searches_both_views(self):
        nav = self.navigation()
        blank = self.lose_ramp(nav)
        arrival = 5.2
        for view, angle in (('backward', BACKWARD), ('forward', FORWARD)):
            for t in (arrival, arrival + 1.0, arrival + SETTINGS['search_view_s'] - .01):
                self.frame(nav, t, angle, blank)
                self.assertEqual(nav.view, view)
                self.assertFalse(nav.ready(t))
            departure = arrival + SETTINGS['search_view_s'] + .01
            self.frame(nav, departure, angle, blank)
            self.assertNotEqual(nav.view, view)
            self.assertFalse(nav.ready(departure))
            arrival = departure + SETTINGS['sweep_s'] + .1

    def test_arrived_camera_does_not_wait_for_the_full_planned_sweep(self):
        nav = self.navigation()
        self.lose_ramp(nav)
        self.frame(nav, 3.6, BACKWARD, speed=math.radians(10))
        self.assertFalse(nav.ready(3.6))
        self.frame(nav, 3.7, BACKWARD)
        self.assertFalse(nav.ready(3.7))
        self.frame(nav, 3.9, BACKWARD)
        self.assertFalse(nav.ready(3.9))
        self.frame(nav, 4.01, BACKWARD)
        self.assertTrue(nav.ready(4.01))
        self.assertAlmostEqual(nav.camera_target(4), math.radians(BACKWARD))

    def test_ramp_seen_during_flip_cannot_resume_walking(self):
        nav=self.navigation();self.lose_ramp(nav)
        self.assertTrue(self.frame(nav,4,25))
        self.assertFalse(nav.ready(4))
        self.assertTrue(nav.searching)

    def test_lost_detection_resets_the_short_confirmation(self):
        nav = self.navigation()
        blank = self.lose_ramp(nav)
        self.frame(nav, 5.2, BACKWARD)
        self.assertFalse(nav.ready(5.2))
        self.frame(nav, 5.3, BACKWARD, blank)
        for t in (5.4, 5.6):
            self.frame(nav, t, BACKWARD)
            self.assertFalse(nav.ready(t))
        self.frame(nav, 5.71, BACKWARD)
        self.assertTrue(nav.ready(5.71))

    def test_view_does_not_flip_while_a_valid_ramp_is_being_confirmed(self):
        nav = self.navigation()
        blank = self.lose_ramp(nav)
        for t in (5.2, 5.8):
            self.frame(nav, t, BACKWARD, blank)
        for t in (5.9, 6.0):
            self.frame(nav, t, BACKWARD)
            self.assertFalse(nav.ready(t))
            self.assertEqual(nav.view, 'backward')
        self.frame(nav, 6.21, BACKWARD)
        self.assertTrue(nav.ready(6.21))

    def test_search_applies_on_a_slope_too(self):
        nav=self.navigation();self.frame(nav,2,FORWARD)
        nav.pitch=math.radians(18)
        nav.last_imu=3.2
        blank=np.full((240,320,3),180,dtype=np.uint8)
        nav.scan_frame(3.2,blank,math.radians(FORWARD),0,0)
        self.assertTrue(nav.searching)
        self.assertFalse(nav.ready(3.2))

    def test_clipped_edges_pause_and_search_after_guidance_timeout(self):
        nav=self.navigation();self.frame(nav,2,FORWARD)
        closeup=np.full((240,320,3),15,dtype=np.uint8)
        self.assertFalse(self.frame(nav,3.2,FORWARD,closeup))
        self.assertTrue(nav.debug['surface_detected'])
        self.assertTrue(nav.searching)
        self.assertFalse(nav.ready(3.2))

    def test_never_seen_ramp_stays_stopped_and_starts_search_after_startup(self):
        nav=self.navigation()
        blank=np.full((240,320,3),180,dtype=np.uint8)
        self.frame(nav,1,FORWARD,blank)
        self.assertFalse(nav.ready(1))
        self.assertFalse(nav.searching)
        self.frame(nav,3.2,FORWARD,blank)
        self.assertTrue(nav.searching)
        self.assertFalse(nav.ready(3.2))

    def test_stale_imu_holds_walking_without_triggering_flip(self):
        nav=self.navigation();self.frame(nav,2,FORWARD)
        blank=np.full((240,320,3),180,dtype=np.uint8)
        for t in (3.2,3.5):nav.scan_frame(t,blank,math.radians(FORWARD),0,0)
        self.assertFalse(nav.searching)
        self.assertFalse(nav.ready(3.5))


if __name__=='__main__':
    unittest.main()
