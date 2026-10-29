"""Exercise saving the extra pitch point without a display or real serial port."""
import json
from pathlib import Path
import tempfile
import unittest

from gui import Session


class StatusConnection:
    def request(self, command):
        if command != 'STATUS':
            raise AssertionError('Recording must only read status, never move the servo')
        return 'OK STATUS ready=1 channel=3 pulse=2200'


class RaisedPitchTests(unittest.TestCase):
    def test_add_endpoints_preserves_existing_measurements(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'measurements.json'
            original = {'FL_pitch': {'channel': 3, 'points': {'-45': 950, '0': 1500, '45': 1850}},
                        'FR_yaw': {'channel': 0, 'points': {'0': 1415}}}
            path.write_text(json.dumps(original))
            session = Session(path)
            session.connection = StatusConnection()
            session.active = ('FL_pitch', 3, 2200)
            session.record(90)
            session.record(-90)
            saved = json.loads(path.read_text())
            self.assertEqual(saved['FL_pitch']['points']['90'], 2200)
            self.assertEqual(saved['FL_pitch']['points']['-90'], 2200)
            del saved['FL_pitch']['points']['90']
            del saved['FL_pitch']['points']['-90']
            self.assertEqual(saved, original)

    def test_yaw_cannot_record_90(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Session(Path(directory) / 'measurements.json')
            session.connection = StatusConnection()
            session.active = ('FL_yaw', 8, 1500)
            for angle in (-90, 90):
                with self.assertRaises(ValueError):
                    session.record(angle)


if __name__ == '__main__':
    unittest.main()
