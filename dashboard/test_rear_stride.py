"""Reduced rear strokes preserve front action, stance and single-foot support."""
import unittest
from unittest.mock import patch

import numpy as np

from robot_b91.gait import rocker_targets, RAMP_STEERING
from robot_b91.run import load_gait_parameters
from robot_dashboard.devices import DeviceError
from robot_dashboard.runtime import Dashboard
from test_crest_sequence import status


class RearStrideTests(unittest.TestCase):
    def test_only_rear_yaw_stroke_changes_and_bias_is_preserved(self):
        base = load_gait_parameters() | RAMP_STEERING | dict(
            pattern='crest', duty=.8, yaw_amplitude=.4, yaw_bias=-.2,
            pitch_lift=.25, pitch_stance=0)
        for t in np.linspace(0, 1, 201):
            original = rocker_targets(base, 4, t, .05, .01)
            np.testing.assert_array_equal(original, rocker_targets(base | {'rear_stride_scale': 1}, 4, t, .05, .01))
            for scale in (0, .35, .5):
                reduced = rocker_targets(base | {'rear_stride_scale': scale}, 4, t, .05, .01)
                np.testing.assert_array_equal(reduced[[0, 1, 3, 4, 5, 7]], original[[0, 1, 3, 4, 5, 7]])
                # Rear yaw bias has opposing signs and does not shrink.
                np.testing.assert_allclose(reduced[[2, 6]],
                    np.array([-.2, .2]) + scale * (original[[2, 6]] - [-.2, .2]))
                self.assertLessEqual(np.count_nonzero(reduced[1::2] < -1e-9), 1)
        for value in (-.1, 1.1, float('nan')):
            with self.assertRaises(ValueError):
                rocker_targets(base | {'rear_stride_scale': value}, 4, 0)

    def test_old_firmware_cannot_silently_ignore_rear_reduction(self):
        for mode in ('assisted', 'approach', 'transition'):
            app = Dashboard(crest_wave=True, start_at_crest=mode == 'assisted', auto_corridor=False)
            import time
            app.browser_heartbeat = time.monotonic()
            old = status() | dict(externalActive=False, crestWaveGait=True)
            with patch.dict('robot_dashboard.walking.CREST_WAVE_GAIT', {'rear_stride_scale': .35}):
                with patch('robot_dashboard.runtime.devices.request', return_value=old) as request:
                    with self.assertRaisesRegex(DeviceError, 'Reduced rear stride'):
                        if mode != 'transition':
                            app.walk_command('course')
                        else:
                            app.walking.start_settings('course')
                            app.walking.begin(status(), 1, 0, 'course')
                            app.walking.crest_sequence.requested = 'crest'
                            app._change_ramp_gait('http://robot.test')
                self.assertNotIn('/motion/start', [call.args[1] for call in request.call_args_list])


if __name__ == '__main__':
    unittest.main()
