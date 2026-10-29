import unittest
from calibrate import JOINTS, export_table


class ExportTests(unittest.TestCase):
    def measurements(self):
        return {name: {'channel': channel,
                       'points': {'-90': 2400, '-45': 2000, '0': 1500, '45': 1000, '90': 600}}
                for name, channel in zip(JOINTS, (0, 1, 2, 3, 8, 9, 10, 11))}

    def test_reversed_servo_is_valid(self):
        self.assertIn('{0, 2000, 1500, 1000}', export_table(self.measurements()))

    def test_duplicate_channel_rejected(self):
        data = self.measurements()
        data['RR_pitch']['channel'] = 0
        with self.assertRaises(ValueError):
            export_table(data)

    def test_nonmonotonic_rejected(self):
        data = self.measurements()
        data['FL_yaw']['points']['45'] = 2100
        with self.assertRaises(ValueError):
            export_table(data)

    def test_missing_measurement_rejected(self):
        data = self.measurements()
        del data['FL_yaw']['points']['45']
        with self.assertRaises(KeyError):
            export_table(data)

    def test_pitch_requires_fourth_measurement(self):
        data = self.measurements()
        del data['FL_pitch']['points']['90']
        with self.assertRaises(KeyError):
            export_table(data)

    def test_export_includes_fourth_pitch_pulse(self):
        self.assertIn('{1, 2000, 1500, 1000, 600, 2400}', export_table(self.measurements()))

    def test_pitch_fourth_measurement_must_follow_direction(self):
        data = self.measurements()
        data['FL_pitch']['points']['90'] = 1100
        with self.assertRaises(ValueError):
            export_table(data)

    def test_pitch_requires_minus90_measurement(self):
        data = self.measurements()
        del data['FL_pitch']['points']['-90']
        with self.assertRaises(KeyError):
            export_table(data)

    def test_minus90_measurement_must_follow_direction(self):
        data = self.measurements()
        data['FL_pitch']['points']['-90'] = 1900
        with self.assertRaises(ValueError):
            export_table(data)

    def test_yaw_only_needs_original_three_points(self):
        data = self.measurements()
        for name, entry in data.items():
            if name.endswith('_yaw'):
                del entry['points']['-90']
                del entry['points']['90']
        self.assertIn('{0, 2000, 1500, 1000}', export_table(data))


if __name__ == '__main__':
    unittest.main()
