import unittest
from calibrate import JOINTS, export_table


class ExportTests(unittest.TestCase):
    def measurements(self):
        return {name: {'channel': channel,
                       'points': {'-45': 2000, '0': 1500, '45': 1000}}
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


if __name__ == '__main__':
    unittest.main()
