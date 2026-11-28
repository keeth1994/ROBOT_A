"""Test dashboard video recording."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from dashboard.robot_dashboard.recording import DashboardRecording


class DashboardRecordingTests(unittest.TestCase):
    def recording(self, root):
        factory = MagicMock()
        module = SimpleNamespace(sync_playwright=factory)
        with patch.dict('sys.modules', {'playwright.sync_api': module}), \
                patch('dashboard.robot_dashboard.recording.ROOT', Path(root)):
            recorder = DashboardRecording()
        return recorder, factory

    def test_records_the_read_only_page_and_finishes_video_before_saving(self):
        with tempfile.TemporaryDirectory() as root:
            recorder, factory = self.recording(root)
            browser = factory.return_value.__enter__.return_value.chromium.launch.return_value
            context = browser.new_context.return_value
            page = context.new_page.return_value
            page.wait_for_timeout.side_effect = lambda _: recorder.stopped.set()
            page.video.save_as.side_effect = lambda _: self.assertTrue(context.close.called)
            with redirect_stdout(StringIO()):
                recorder.record('http://127.0.0.1:8765')
            page.goto.assert_called_once_with('http://127.0.0.1:8765/?recording=1',
                                              wait_until='domcontentloaded')
            options = browser.new_context.call_args.kwargs
            self.assertEqual(options['viewport'], options['record_video_size'])
            self.assertEqual(options['record_video_dir'], str(recorder.output))
            page.video.save_as.assert_called_once_with(str(recorder.output / 'dashboard.webm'))
            page.video.delete.assert_called_once()
            context.close.assert_called_once()
            browser.close.assert_called_once()

    def test_navigation_failure_still_closes_browser_and_reports_error(self):
        with tempfile.TemporaryDirectory() as root:
            recorder, factory = self.recording(root)
            browser = factory.return_value.__enter__.return_value.chromium.launch.return_value
            context = browser.new_context.return_value
            context.new_page.return_value.goto.side_effect = RuntimeError('Server unavailable')
            output = StringIO()
            with redirect_stdout(output):
                recorder.record('http://127.0.0.1:8765')
            context.close.assert_called_once()
            browser.close.assert_called_once()
            self.assertIn('Dashboard recording failed: Server unavailable', output.getvalue())

    def test_close_waits_for_video_to_finish(self):
        with tempfile.TemporaryDirectory() as root:
            recorder, _ = self.recording(root)
            recorder.thread = MagicMock()
            recorder.close()
            self.assertTrue(recorder.stopped.is_set())
            recorder.thread.join.assert_called_once()


if __name__ == '__main__':
    unittest.main()
