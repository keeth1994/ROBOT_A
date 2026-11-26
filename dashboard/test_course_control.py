"""Ramp startup and the stop paths that must remain independent of course conditions."""
import time
import unittest
from unittest.mock import patch

import numpy as np

from robot_dashboard.corridor_search import CorridorSearch
from robot_dashboard.corridor_walls import load_corridor_settings
from robot_dashboard.devices import DeviceConnectionError
from robot_dashboard.runtime import Dashboard
from robot_dashboard.walking import JOINTS, WALK_ACTIONS


def status(**changes):
    return dict(servoReady=True, faulted=False, motionProtocol=2, signedYawBias=True,
                corridorSearchGait=True, crestWaveGait=True, rearStrideScale=True, boundedSteering=True,
                externalActive=True, motionSession=8, gaitStopping=False,
                orientationValid=True, imuAgeMs=5, orientation=[0., 0., 0.],
                cameraPitch=dict(angle=-65, moving=False, minAngle=-80, maxAngle=45),
                joints=[dict(name=n, channel=i, angle=45 if n.endswith('pitch') else 0)
                        for i, n in enumerate(JOINTS)]) | changes


class CourseControlTests(unittest.TestCase):
    def test_course_starts_in_approach_without_an_alignment_session(self):
        app = Dashboard()
        app.browser_heartbeat = time.monotonic()
        with patch('robot_dashboard.runtime.devices.request', side_effect=[
                status(externalActive=False), {'session': 8}]) as request:
            app.walk_command('course')
        self.assertEqual([call.args[1] for call in request.call_args_list], ['/status', '/motion/start'])
        self.assertEqual(request.call_args_list[-1].args[2]['profile'], 'course')
        self.assertEqual(app.walking.crest_sequence.phase, 'approach')
        self.assertTrue(app.walking.active)
        self.assertNotIn('ramp_align', WALK_ACTIONS)
        with patch('robot_dashboard.runtime.devices.request') as request, self.assertRaises(ValueError):
            app.walk_command('ramp_align')
        request.assert_not_called()

    def test_operator_stop_works_without_a_live_heartbeat(self):
        app = Dashboard()
        app.walking.begin(status(), 8, 0., 'course')
        with patch('robot_dashboard.runtime.devices.request', return_value={}) as request:
            app.servo_command('/stop', {})
        self.assertFalse(app.walking.active)
        self.assertEqual([call.args[1] for call in request.call_args_list], ['/stop'])

    def test_lost_operator_or_servo_link_ends_session_without_restarting(self):
        for operator_present in (False, True):
            with self.subTest(operator_present=operator_present):
                app = Dashboard()
                now = time.monotonic()
                app.walking.begin(status(), 8, now - 5., 'course')
                app.browser_heartbeat = now if operator_present else now - 5.
                replies = [DeviceConnectionError('servo link lost'), {}] if operator_present else [{}]
                with patch.object(app.stop_event, 'wait', side_effect=[False, True]), \
                     patch('robot_dashboard.runtime.devices.request', side_effect=replies) as request:
                    app._walk_loop()
                self.assertFalse(app.walking.active)
                self.assertEqual([c.args[1] for c in request.call_args_list],
                                 (['/status'] if operator_present else []) + ['/stop'])

    def test_corridor_search_can_retry_an_empty_scan_and_resume_after_invalid_ir(self):
        nav = CorridorSearch(0., load_corridor_settings()['navigation'])
        nav.camera_angle = nav.orientation.camera_calibration['corridor_servo_deg']
        state = status(cameraPitch=dict(angle=nav.camera_angle, moving=False), irAgeMs=5,
                       irMillivolts=[3250, 100])
        image = np.zeros((32, 32, 3), dtype=np.uint8)
        nav.turn_degrees = 361.
        with patch.object(nav, 'collect_candidates'), patch.object(nav, 'choose_entrance', return_value=False):
            self.assertFalse(nav.update(state, 301., (1, 301., image)))
            self.assertFalse(nav.finished)
            state['irMillivolts'] = [100, 100]
            self.assertTrue(nav.update(state, 302., (2, 302., image)))
        self.assertEqual(nav.turn_degrees, 0.)
        self.assertFalse(nav.finished)
        nav.change_stage('enter', 310.)
        nav.wall_side = 0
        self.assertFalse(nav.update(state, 330., (3, 330., image)))
        self.assertEqual(nav.stage, 'scan')
        self.assertFalse(nav.finished)
        nav.change_stage('center', 340.)
        nav.entry_heading = nav.imu_heading
        self.assertTrue(nav.center(370., [.07, .20]))
        self.assertFalse(nav.finished)
