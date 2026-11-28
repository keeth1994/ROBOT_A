"""Test console messages."""
import unittest

from dashboard.robot_dashboard.runtime import Dashboard


class ConsoleTests(unittest.TestCase):
    def test_messages_between_polls_are_kept_in_order_without_duplicates(self):
        app = Dashboard()
        initial = app.state()
        cursor = initial['messages'][-1]['id']
        app.walking.message = 'Turning the camera backward.'
        app.walking.message = 'Turning the camera backward.'
        app.walking.message = 'Both edges found. Continuing.'
        state = app.state(cursor, initial['message_session'])
        self.assertEqual([entry['message'] for entry in state['messages']],
                         ['Turning the camera backward.', 'Both edges found. Continuing.'])
        self.assertTrue(all(entry['source'] == 'Motion' for entry in state['messages']))
        self.assertEqual(app.state(state['messages'][-1]['id'], initial['message_session'])['messages'], [])

    def test_reload_and_server_restart_return_available_history(self):
        app = Dashboard()
        app.walking.message = 'Starting corridor navigation.'
        state = app.state()
        self.assertIn('Starting corridor navigation.', [entry['message'] for entry in state['messages']])
        restarted = Dashboard()
        state = restarted.state(9999, state['message_session'])
        self.assertTrue(state['messages'])
        self.assertEqual(state['messages'][0]['source'], 'Motion')

    def test_sources_keep_independent_histories(self):
        app = Dashboard()
        app.log_message('Controller', 'Walking locally on ESP32.')
        app.log_message('Camera', 'Camera connected.')
        app.log_message('Controller', 'Walking locally on ESP32.')
        entries = [entry for entry in app.state()['messages'] if entry['source'] in ('Camera', 'Controller')]
        self.assertEqual([entry['source'] for entry in entries], ['Controller', 'Camera'])


if __name__ == '__main__':
    unittest.main()
