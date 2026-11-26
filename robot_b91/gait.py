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
# Correct lateral offset with the configured strafe gain rather than a sharper turn.
RAMP_STEERING = dict(heading_gain=1.2, lateral_gain=0., turn_limit=.2)


def ramp_forward_scale(lateral_error_m):
    """Give sideways correction more time as offset grows, without sharper turns."""
    if not math.isfinite(lateral_error_m):
        raise ValueError('Ramp offset must be finite.')
    error = min(1., max(0., (abs(lateral_error_m) - .010) / .020))
    return 1. - .50 * error * error * (3. - 2. * error)


def rocker_targets(params, elapsed_s, gait_time_s, heading_error_rad=0.0,
                   lateral_error_m=0.0, joint_ranges=None, forward_scale=1.0):
    """Return eight bounded joint angles; call at the existing 50 Hz command rate.

    elapsed_s is time since walking began; gait_time_s preserves phase when
    changing period. Positive lateral error means left of the desired path.
    Heading correction changes left/right strides; strafe correction changes
    front/rear strides, so correcting drift need not turn the body off the ramp.
    """
    p = params
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
        turn_limit = p.get('turn_limit', .95 if p.get('corridor', False) else .5)
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
                lift = -p["pitch_lift"] * height * height * (3 - 2 * height)
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


FOAM_CREST_GAIT = dict(pattern='crest', period=.8, duty=.8, yaw_amplitude=.75,
                       rear_stride_scale=1., pitch_lift=.5, pitch_stance=0., yaw_bias=0.)


class FoamCrestController:
    """Simulation candidate. Start a fresh instance when entering a crest trial."""

    def __init__(self, wave, climb):
        # 'search' lets the existing target generator scale forward stride separately from turn.
        self.wave = wave | {'search': True}
        self.climb = climb | {'search': True, 'period': wave['period']}
        self.previous_s = 0.
        self.filtered_pitch_deg = None
        self.level_s = 0.

    def targets(self, elapsed_s, gait_time_s, pitch_rad, heading_rad, joint_ranges=None, *, trim=0.):
        if not all(map(math.isfinite, (elapsed_s, gait_time_s, pitch_rad, heading_rad, trim))):
            raise ValueError('Crest gait needs finite time, pitch, heading and trim.')
        if elapsed_s < self.previous_s:
            raise ValueError('Crest gait time went backwards. Start a new controller for a new trial.')
        dt = elapsed_s - self.previous_s
        self.previous_s = elapsed_s
        degrees = math.degrees(pitch_rad)
        if self.filtered_pitch_deg is None:
            self.filtered_pitch_deg = degrees
        self.filtered_pitch_deg += dt / (dt + .5) * (degrees - self.filtered_pitch_deg)

        # Move the feet inward as the chassis tips downhill. If it remains near
        # level for many cycles, gradually tuck further to free a grounded belly.
        self.level_s = max(0., self.level_s + dt * (1 if -8 <= self.filtered_pitch_deg <= 12 else -2))
        recovery = max(0., min(1., (self.level_s - 20.) / 3.))
        bias = -.4 * max(0., min(1., (degrees + 10.) / 16.))
        self.wave['yaw_bias'] = bias + (-.72 - bias) * recovery
        correction = max(0., min(1., (abs(math.degrees(heading_rad)) - 6.) / 4.))
        forward = 1. - .5 * correction
        heading = heading_rad + trim / self.wave['heading_gain']
        wave = rocker_targets(self.wave, elapsed_s, gait_time_s, heading, 0., joint_ranges, forward)

        # Negative pitch is nose-up. Keep climbing on a steep approach and
        # crossfade into the one-foot-at-a-time wave without restarting the clock.
        mix = max(0., min(1., (-self.filtered_pitch_deg - 10.) / 8.)) * min(1., elapsed_s / .5)
        if not mix:
            return wave
        climb = rocker_targets(self.climb, elapsed_s, gait_time_s, heading_rad + trim / self.climb['heading_gain'],
                               0., joint_ranges, forward)
        return wave + (climb - wave) * mix
