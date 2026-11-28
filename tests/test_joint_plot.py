"""Test joint target history."""
import unittest

from dashboard.robot_dashboard.runtime import Dashboard


def joint_status():
    return dict(externalActive=True, joints=[
        dict(name=f'{leg}_{axis}', angle=angle)
        for leg, angles in zip(('FL', 'RL', 'FR', 'RR'), ((10, 34), (-20, -5), (30, 20), (-40, 0)))
        for axis, angle in zip(('yaw', 'pitch'), angles)])


class JointPlotTests(unittest.TestCase):
    def test_reported_servo_angles_become_gait_joint_targets_in_leg_order(self):
        app = Dashboard()
        app.record_joint_sample(joint_status(), 100)
        sample = app.state()['joint_samples'][0]
        self.assertEqual(sample['yaw'], [10, 20, 30, 40])
        self.assertEqual(sample['pitch'], [-11, -50, -25, -45])
        self.assertEqual(sample['time_s'], 0)

    def test_samples_between_browser_refreshes_are_retained_and_sent_once(self):
        app = Dashboard()
        for now in (100, 100.1, 100.2):
            app.record_joint_sample(joint_status(), now)
        state = app.state()
        self.assertEqual(len(state['joint_samples']), 3)
        self.assertAlmostEqual(state['joint_samples'][-1]['time_s'], .2)
        state = app.state(message_session=state['message_session'], after_joint=2)
        self.assertEqual([sample['id'] for sample in state['joint_samples']], [3])

    def test_inactive_and_missing_targets_do_not_create_fake_samples(self):
        app = Dashboard()
        app.record_joint_sample(joint_status() | dict(externalActive=False), 100)
        for value in (None, True, float('nan')):
            status = joint_status()
            status['joints'][0]['angle'] = value
            app.record_joint_sample(status, 100)
        self.assertEqual(app.state()['joint_samples'], [])


if __name__ == '__main__':
    unittest.main()
