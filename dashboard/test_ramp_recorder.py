"""Exercise the recorder clock, telemetry and operator Stop without any device I/O."""
from contextlib import ExitStack, redirect_stdout
from io import StringIO
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

import ramp_trial


class RampRecorderTests(unittest.TestCase):
    def record(self, output, *args):
        clock = Mock()
        now = [0.]
        clock.monotonic.side_effect = lambda: now[0]
        clock.time.return_value = 0.
        clock.sleep.side_effect = lambda _: now.__setitem__(0, now[0] + 1.)
        client = Mock()
        client.camera_frame.return_value = b'frame'
        calls = []

        def request(path, values=None, *, binary=False):
            calls.append(path)
            if path == '/api/state':
                if now[0] >= 65:
                    (output / 'STOP').touch()
                return dict(token='test', auto_corridor=False, camera=dict(online=True),
                            servo=dict(online=True, data=dict(orientationValid=True,
                                      orientation=[math.radians(25), math.radians(50), 0.])),
                            pipeline=dict(fresh=True), walking=dict(active='/api/walk/course' in calls,
                                      message='Holding steps for guidance.'))
            return b'panel' if binary else {}

        def camera(*args, **kwargs):
            (output / 'webcam.jpg').write_bytes(b'frame')
            return Mock(**{'poll.return_value': None})

        client.request.side_effect = request
        with ExitStack() as stack:
            stack.enter_context(patch.object(ramp_trial, 'Client', return_value=client))
            stack.enter_context(patch.object(ramp_trial, 'time', clock))
            stack.enter_context(patch.object(ramp_trial.subprocess, 'Popen', side_effect=camera))
            stack.enter_context(patch.object(ramp_trial.imageio_ffmpeg, 'get_ffmpeg_exe', return_value='fake'))
            stack.enter_context(patch('sys.argv', ['ramp_trial.py', '--output', str(output), *args]))
            stack.enter_context(redirect_stdout(StringIO()))
            ramp_trial.main()
        self.assertEqual(calls[-1], '/api/servo/stop')
        return json.loads((output / 'summary.json').read_text())

    def test_no_default_duration_tilt_or_guidance_timeout_but_stop_file_still_stops(self):
        with TemporaryDirectory() as directory:
            summary = self.record(Path(directory))
        self.assertEqual(summary['reason'], 'Operator stop file.')
        self.assertGreaterEqual(summary['duration_s'], 65.)
        self.assertEqual(summary['peak_roll_deg'], 25.)
        self.assertEqual(summary['peak_pitch_deg'], 50.)
        self.assertIsNone(summary['requested_duration_s'])

    def test_explicit_recording_duration_is_honored(self):
        with TemporaryDirectory() as directory:
            summary = self.record(Path(directory), '--seconds', '3')
        self.assertEqual(summary['reason'], 'Trial time limit reached.')
        self.assertEqual(summary['duration_s'], 3.)
