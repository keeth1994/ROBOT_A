"""Rendered-camera regressions for holding a view and recovering after ramp loss."""
import math
import unittest

import mujoco
import numpy as np

from robot.simulation.run import Robot, load_gait_parameters


class CameraScanSimulationTests(unittest.TestCase):
    def test_visible_ramp_keeps_camera_forward(self):
        robot = Robot(ramp=True, feedback='sensors')
        try:
            self.assertAlmostEqual(robot.d.qpos[robot.camera_qa], math.radians(robot.camera_scan["forward_deg"]))
            while robot.d.time < 4:
                robot.step(walk=False)
            nav = robot.navigation.estimate
            self.assertEqual(nav.view, 'forward')
            self.assertFalse(nav.searching)
            self.assertTrue(nav.ready(robot.d.time))
            self.assertAlmostEqual(robot.d.qpos[robot.camera_qa], math.radians(robot.camera_scan["forward_deg"]), delta=math.radians(2))
            self.assertEqual(sum(w.number for w in robot.d.warning), 0)
        finally:
            robot.close()

    def test_endpoint_views_do_not_show_robot(self):
        robot = Robot(ramp=True, feedback='sensors')
        renderer = robot.navigation.renderer
        try:
            renderer.enable_segmentation_rendering()
            for view in ('forward', 'backward'):
                angle = math.radians(robot.camera_scan[view + '_deg'])
                for name, ratio in [('camera_cradle',1), ('camera_input',-.5),
                                    ('upper_rocker',-.5), ('camera_link',.5)]:
                    robot.d.qpos[robot.m.joint(name).qposadr[0]] = angle * ratio
                mujoco.mj_forward(robot.m, robot.d)
                renderer.update_scene(robot.d, camera='project_camera',
                                      scene_option=robot.navigation.options)
                image = renderer.render()
                geometry = (image[:,:,1] == mujoco.mjtObj.mjOBJ_GEOM) & (image[:,:,0] >= 0)
                ids = np.where(geometry, image[:,:,0], 0)
                robot_pixels = geometry & (robot.m.geom_bodyid[ids] > 0)
                self.assertEqual(int(robot_pixels.sum()), 0, view + ' view must clear the robot')
        finally:
            renderer.disable_segmentation_rendering()
            robot.close()

    def test_forward_loss_pauses_flips_and_recovers_from_rear_view(self):
        robot = Robot(ramp=True, feedback='sensors')
        try:
            # Test placement only; navigation never receives this course position.
            robot.d.qpos[0] = robot.course['origin_xy_m'][0] + robot.course['downhill_end_x'] + .35
            mujoco.mj_forward(robot.m, robot.d)
            was_searching = saw_pause = saw_resume = False
            held_targets = None
            angles = []
            # Allow the forward search view and rear-view settling/reacquisition.
            while robot.d.time < 12:
                robot.step()
                angles.append(float(robot.d.qpos[robot.camera_qa]))
                nav = robot.navigation.estimate
                if nav.searching:
                    saw_pause = True
                    if not was_searching:
                        held_targets = robot.command_target.copy()
                    np.testing.assert_allclose(robot.command_target, held_targets)
                    self.assertFalse(nav.ready(robot.d.time))
                elif was_searching and nav.ready(robot.d.time):
                    saw_resume = True
                    self.assertEqual(nav.view, 'backward')
                was_searching = nav.searching
            self.assertTrue(saw_pause, 'Ramp loss must pause walking and start a search')
            self.assertTrue(saw_resume, 'Valid rear-view guidance must allow walking to resume')
            self.assertLess(min(angles), math.radians(robot.camera_scan["forward_deg"]+5))
            self.assertGreater(max(angles), math.radians(robot.camera_scan["backward_deg"]-5))
            self.assertEqual(robot.p, load_gait_parameters())
            self.assertEqual(sum(w.number for w in robot.d.warning), 0)
        finally:
            robot.close()


if __name__ == '__main__':
    unittest.main()
