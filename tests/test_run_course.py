"""Test course stages."""
import math
import sys
import time
import unittest
from unittest.mock import Mock, patch

from dashboard import run
from dashboard.robot_dashboard.crest import CrestSequence
from dashboard.robot_dashboard.devices import DeviceConnectionError
from dashboard.robot_dashboard.runtime import Dashboard
from tests.test_crest_sequence import status


class RunCourseTests(unittest.TestCase):
    def test_startup_flags_default_to_first_and_allow_last(self):
        for flags, enabled, position in (([], True, 'first'),
                                         (['--crest-first'], True, 'first'),
                                         (['--crest-last'], True, 'last'),
                                         (['--crest-last', '--no-crest'], False, 'last')):
            with self.subTest(flags=flags):
                runtime = Mock()
                server = Mock()
                with patch.dict(sys.modules, {'robot_dashboard.runtime': runtime,
                                               'robot_dashboard.server': server}), \
                        patch.object(sys, 'argv', ['dashboard/run.py', *flags]), \
                        patch('builtins.print'):
                    run.main()
                self.assertEqual(runtime.Dashboard.call_args.kwargs,
                                 dict(crest_enabled=enabled, crest_position=position))

    def feed(self, sequence, start, pitch, count=20, ready=True):
        for i in range(count):
            sequence.observe(start + i * .1, math.radians(pitch), ready)

    def test_last_crest_skips_first_top_and_waits_for_descent_then_second_climb(self):
        sequence = CrestSequence('last')
        self.feed(sequence, 0, -22)
        self.feed(sequence, 2, -5)
        self.assertIsNone(sequence.requested)
        self.feed(sequence, 4, 0, 60)
        self.feed(sequence, 10, -22)
        self.feed(sequence, 12, -5)
        self.assertEqual(sequence.climbs_seen, 1)
        self.assertIsNone(sequence.requested)
        self.feed(sequence, 14, 20)
        self.feed(sequence, 16, -22)
        self.feed(sequence, 18, -5)
        self.assertEqual(sequence.climbs_seen, 2)
        self.assertEqual(sequence.requested, 'crest')
        sequence.switched(20)
        self.feed(sequence, 20, 20, 60)
        self.assertEqual(sequence.requested, 'descent')
        sequence.switched(26)
        self.feed(sequence, 26, -22)
        self.feed(sequence, 28, -5)
        self.assertIsNone(sequence.requested)

    def test_brief_or_unready_descent_cannot_arm_last_crest(self):
        sequence = CrestSequence('last')
        self.feed(sequence, 0, -22)
        self.feed(sequence, 2, -5)
        self.feed(sequence, 4, 20, 4)
        self.feed(sequence, 4.4, 0)
        self.feed(sequence, 6.4, 20, ready=False)
        self.feed(sequence, 8.4, -22)
        self.feed(sequence, 10.4, -5)
        self.assertEqual(sequence.climbs_seen, 1)
        self.assertIsNone(sequence.requested)

    def start(self, action='run_course'):
        app = Dashboard(crest_position='last')
        app.browser_heartbeat = time.monotonic()
        idle = status() | dict(externalActive=False, corridorSearchGait=True, corridorWallGait=True)
        with patch('dashboard.robot_dashboard.runtime.devices.request',
                   side_effect=[idle, dict(session=1)]), patch.object(app, '_aim_camera'):
            app.walk_command(action)
        return app

    def finish_stage(self, app, fail_start=False, **completion):
        now = time.monotonic()
        app.walking.started = now - 3
        app.walking.last_confirmed_frame = app.browser_heartbeat = now
        navigation = app.walking.navigation

        def update(*args):
            for name, value in completion.items():
                setattr(navigation, name, value)
            return False

        app.stop_event = Mock()
        app.stop_event.wait.side_effect = [False, True]
        active = status(app.walking.session)
        idle = active | dict(externalActive=False, corridorSearchGait=True, corridorWallGait=True)
        next_session = app.walking.session + 1
        calls = []

        def request(url, path, values=None, **kwargs):
            calls.append((path, values))
            if path == '/status':
                return idle if any(p == '/stop' for p, _ in calls) else active
            if path == '/motion/start':
                if fail_start:
                    raise DeviceConnectionError('Start response was lost.')
                return dict(session=next_session)
            return {}

        with patch.object(navigation, 'update', side_effect=update), \
                patch.object(app, '_aim_camera'), \
                patch('dashboard.robot_dashboard.runtime.devices.request', side_effect=request):
            app._walk_loop()
        return calls

    def test_full_course_runs_ramp_search_corridor_then_dance(self):
        app = self.start()
        self.assertTrue(app.course_running)
        self.assertEqual(app.walking.crest_sequence.target_climb, 2)
        for completion, profile, controller_profile in (
                (dict(ramp_finished=True), 'corridor_search', 'corridor_search'),
                (dict(finished=True, succeeded=True), 'corridor', 'course'),
                (dict(exit_complete=True), 'dance', 'dance')):
            calls = self.finish_stage(app, **completion)
            self.assertEqual([p for p, _ in calls], ['/status', '/stop', '/status', '/motion/start'])
            self.assertEqual(calls[-1][1]['profile'], controller_profile)
            self.assertEqual(app.walking.profile, profile)
            self.assertTrue(app.walking.active)
        self.assertFalse(app.course_running)

    def test_ramp_button_still_stops_at_ramp_end(self):
        app = self.start('course')
        calls = self.finish_stage(app, ramp_finished=True)
        self.assertEqual([p for p, _ in calls], ['/status', '/stop'])
        self.assertFalse(app.walking.active)

    def test_course_clears_ramp_and_enables_missing_wall_finish(self):
        app = self.start()
        self.finish_stage(app, ramp_finished=True)
        self.assertEqual(app.walking.navigation.stage, 'clear_ramp')
        self.finish_stage(app, finished=True, succeeded=True)
        self.assertTrue(app.walking.navigation.finish_on_missing_walls)

    def test_unsuccessful_search_does_not_start_corridor(self):
        app = self.start()
        self.finish_stage(app, ramp_finished=True)
        calls = self.finish_stage(app, finished=True, succeeded=False)
        self.assertEqual([p for p, _ in calls], ['/status', '/stop'])
        self.assertFalse(app.walking.active)
        self.assertFalse(app.course_running)

    def test_manual_stop_cancels_course(self):
        app = self.start()
        with patch('dashboard.robot_dashboard.runtime.devices.request', return_value={}):
            app.servo_command('/stop', {})
        self.assertFalse(app.course_running)
        self.assertFalse(app.walking.active)

    def test_failed_transition_is_not_retried(self):
        app = self.start()
        calls = self.finish_stage(app, ramp_finished=True, fail_start=True)
        self.assertEqual([p for p, _ in calls],
                         ['/status', '/stop', '/status', '/motion/start', '/stop'])
        self.assertFalse(app.walking.active)
        self.assertFalse(app.course_running)


if __name__ == '__main__':
    unittest.main()
