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
RAMP_SPEED_SCALE = 0.5


def rocker_targets(params, elapsed_s, gait_time_s, heading_error_rad=0.0,
                   lateral_error_m=0.0, joint_ranges=None, forward_scale=1.0):
    """Return eight bounded joint angles; call at the existing 50 Hz command rate.

    elapsed_s is time since walking began; gait_time_s preserves phase when
    changing period. Positive lateral error means left of the desired path.
    Heading correction changes left/right strides; strafe correction changes
    front/rear strides, so correcting drift need not turn the body off the ramp.
    """
    p = params
    front_lift = p.get('front_pitch_lift', p['pitch_lift'])
    if not math.isfinite(front_lift) or not 0 <= front_lift <= math.radians(125):
        raise ValueError('Front foot lift must be between zero and 125 degrees in radians.')
    rear_stride = p.get('rear_stride_scale', 1.)
    if not math.isfinite(rear_stride) or not 0 <= rear_stride <= 1:
        raise ValueError('Rear stride scale must be between zero and one.')
    target = np.tile([0.0, p["pitch_stance"]], 4)
    if elapsed_s > 0:
        crest = p["pattern"] == "crest"
        if crest and not .8 <= p["duty"] <= .9:
            raise ValueError('Crest gait duty must be between .8 and .9.')
        # Rear-right, front-right, rear-left, front-left: one swing at a time.
        offsets = ([0, .25, .5, .75] if crest else
                   [0, .5, .25, .75] if p["pattern"] == "crawl" else [0, .5, .5, 0])
        error = heading_error_rad + math.atan(p.get("lateral_gain", 0) * lateral_error_m)
        turn_limit = .95 if p.get('corridor', False) else .5
        correction = np.clip(p["heading_gain"] * error, -turn_limit, turn_limit)
        forward = max(.25, 1 - abs(correction)) if p.get('corridor', False) else 1
        if p.get('search', False):
            forward = np.clip(forward_scale, 0, 1)
        strafe = np.clip(p.get("strafe_gain", 0) * lateral_error_m, -0.3, 0.3)
        for i, leg in enumerate(LEGS):
            u = (gait_time_s / p["period"] + offsets[i]) % 1
            duty = p["duty"]
            ramp = min(1, elapsed_s / p["period"])
            if u < duty:
                yaw = p["yaw_amplitude"] * (2 * u / duty - 1)
                lift = 0
            else:
                v = (u - duty) / (1 - duty)
                # Clear the surface before returning the foot, then lower it
                # without sweeping. Ordinary trot/crawl retain their trajectory.
                sweep = np.clip((v - .2) / .6, 0, 1) if crest else v
                yaw = p["yaw_amplitude"] * (1 - 2 * sweep * sweep * (3 - 2 * sweep))
                edge = .2 if crest else .15
                height = min(1, v / edge, (1 - v) / edge)
                pitch_lift = front_lift if crest and leg.startswith('F') else p['pitch_lift']
                lift = -pitch_lift * height * height * (3 - 2 * height)
            sign = 1 if i < 2 else -1
            fore = 1 if leg.startswith("F") else -1
            stride_scale = rear_stride if fore < 0 else 1.
            target[2 * i] = yaw * ramp * stride_scale * (sign * forward + correction + fore * strafe)
            target[2 * i + 1] = p["pitch_stance"] + lift * ramp
    target[1::2] = np.clip(target[1::2], p.get("pitch_min", math.radians(-125)), 0)
    if joint_ranges is None:
        joint_ranges = np.tile([-math.pi / 4, math.pi / 4], (8, 1))
        joint_ranges[1::2, 0] = -3 * math.pi / 4
        joint_ranges[1::2, 1] = 0
    target = np.clip(target, joint_ranges[:, 0], joint_ranges[:, 1])
    ramp = max(0, min(1, elapsed_s / p['period']))
    target[::2] += ramp * p.get('yaw_bias', 0) * np.array([-1, 1, 1, -1])
    return np.clip(target, joint_ranges[:, 0], joint_ranges[:, 1])
