"""Rocker gait calculation without MuJoCo or access to simulator state.

An adapter supplies heading error (radians) and cross-track error (metres).
These can come from localization or operator input; the IMU alone cannot
provide a drift-free cross-track measurement. Output order is FL, RL, FR, RR,
with yaw then pitch for each leg. Servo mounting signs/zeroes still need
physical calibration before converting these joint angles to PWM.
"""

import math
import numpy as np

LEGS = ("FL", "RL", "FR", "RR")


def rocker_targets(params, elapsed_s, gait_time_s, heading_error_rad=0.0,
                   lateral_error_m=0.0, joint_ranges=None):
    """Return eight bounded joint angles; call at the existing 50 Hz command rate.

    elapsed_s is time since walking began; gait_time_s preserves phase when
    changing period. Positive lateral error means left of the desired path.
    Heading correction changes left/right strides; strafe correction changes
    front/rear strides, so correcting drift need not turn the body off the ramp.
    """
    p = params
    target = np.tile([0.0, p["pitch_stance"]], 4)
    if elapsed_s > 0:
        offsets = [0, 0.5, 0.25, 0.75] if p["pattern"] == "crawl" else [0, 0.5, 0.5, 0]
        error = heading_error_rad + math.atan(p.get("lateral_gain", 0) * lateral_error_m)
        correction = np.clip(p["heading_gain"] * error, -0.5, 0.5)
        strafe = np.clip(p.get("strafe_gain", 0) * lateral_error_m, -0.5, 0.5)
        for i, leg in enumerate(LEGS):
            u = (gait_time_s / p["period"] + offsets[i]) % 1
            duty = p["duty"]
            ramp = min(1, elapsed_s / p["period"])
            if u < duty:
                yaw = p["yaw_amplitude"] * (2 * u / duty - 1)
                lift = 0
            else:
                v = (u - duty) / (1 - duty)
                yaw = p["yaw_amplitude"] * (1 - 2 * v * v * (3 - 2 * v))
                lift = -p["pitch_lift"] * math.sin(math.pi * v) ** 2
            sign = 1 if i < 2 else -1
            fore = 1 if leg.startswith("F") else -1
            target[2 * i] = yaw * ramp * (sign + correction + fore * strafe)
            target[2 * i + 1] = p["pitch_stance"] + lift * ramp
    target[1::2] = np.clip(target[1::2], p.get("pitch_min", -math.pi / 4), 0)
    if joint_ranges is None:
        joint_ranges = np.tile([-math.pi / 4, math.pi / 4], (8, 1))
        joint_ranges[1::2, 0] = -3 * math.pi / 4
        joint_ranges[1::2, 1] = 0
    return np.clip(target, joint_ranges[:, 0], joint_ranges[:, 1])
