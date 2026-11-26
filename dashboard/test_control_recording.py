"""Preserve final diagnostics even when control stops producing samples."""
import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from robot_dashboard.control_recording import ControlRecording


class ControlRecordingTests(unittest.TestCase):
    def test_final_sample_flushes_while_idle_and_frames_keep_their_sequence(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=root/'results') as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(root))
            path = Path(directory)/'control.jsonl'
            recording = ControlRecording(path)
            try:
                frame = (42, 1., np.zeros((32,32,3), dtype=np.uint8))
                recording.submit({'event':'adjusted'}, frame)
                recording.submit({'event':'control_error','error':'heading stop'})
                deadline = time.monotonic()+3
                records = []
                while time.monotonic()<deadline:
                    records = [json.loads(line) for line in path.read_text().splitlines()]
                    if len(records)>=2:
                        break
                    time.sleep(.05)
                self.assertEqual(records[-1]['event'], 'control_error')
                self.assertEqual(records[0]['camera_sequence'], 42)
                self.assertTrue((path.parent/records[0]['camera_image']).exists())
                self.assertIsNone(recording.error)
            finally:
                recording.close()
            self.assertEqual(json.loads(path.read_text().splitlines()[-1])['dropped'], 0)


if __name__ == '__main__':
    unittest.main()
