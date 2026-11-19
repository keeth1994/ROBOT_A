"""Bounded experimental side steps; no automatic ramp integration yet."""
import math

from .devices import DeviceError

SIDESTEP_PROFILES = {'sidestep_left': 1, 'sidestep_right': -1}
SIDESTEP_GAIT = dict(period=2., duty=.8, pattern='crest', yaw_amplitude=.45,
                     pitch_lift=.35, pitch_stance=0., yaw_bias=0.,
                     lateral_gain=0., strafe_gain=0., heading_gain=3.)


def check_sidestep(status):
    if status.get('sidestepGait') is not True:
        raise DeviceError('Side steps need the updated controller firmware.', 409)
    attitude = status.get('orientation')
    age = status.get('imuAgeMs')
    if (not status.get('orientationValid') or not isinstance(age, (int, float)) or
            not 0 <= age <= 50 or not isinstance(attitude, (list, tuple)) or
            len(attitude) != 3 or not all(isinstance(a, (int, float)) and math.isfinite(a) for a in attitude)):
        raise DeviceError('Side steps need fresh IMU readings.', 409)
    if abs(attitude[0]) > math.radians(8) or abs(attitude[1]) > math.radians(8):
        raise DeviceError('Side-step trial needs a level stance; outputs off requested.', 409)
