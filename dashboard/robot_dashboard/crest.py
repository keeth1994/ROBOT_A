"""First-crest wave gait, triggered only by onboard observations."""
import math

MAX_SAMPLE_GAP_S = .9


# Lift one foot at a time, with shorter rear strokes and an inward stance.
CREST_WAVE_GAIT = dict(duty=.8, yaw_amplitude=.55, rear_stride_scale=.30, pitch_lift=.25,
                       front_pitch_lift=.50, pitch_stance=0., yaw_bias=-.25, pattern='crest')


class CrestSequence:
    def __init__(self):
        self.phase = 'approach'
        self.climbed = False
        self.since = self.previous = None
        self.moving_seconds = 0.
        self.requested = None

    def observe(self, now, pitch, ready):
        dt = 0 if self.previous is None else now - self.previous
        self.previous = now
        if self.requested or self.phase == 'descent':
            return
        if not ready or not 0 < dt <= MAX_SAMPLE_GAP_S:
            self.since = None
            return
        self.moving_seconds += dt
        degrees = math.degrees(pitch)
        if self.phase == 'approach':
            condition = degrees <= -15 if not self.climbed else degrees >= -10
            duration = 1.0 if not self.climbed else .4
        else:
            # Require sustained downhill pitch and at least four seconds of
            # movement before returning to the normal gait.
            condition = degrees >= 14 and self.moving_seconds >= 4
            duration = .6
        if not condition:
            self.since = None
            return
        self.since = now if self.since is None else self.since
        if now - self.since < duration:
            return
        self.since = None
        if self.phase == 'approach' and not self.climbed:
            self.climbed = True
        else:
            self.requested = 'crest' if self.phase == 'approach' else 'descent'

    def switched(self, now):
        if self.requested not in ('crest', 'descent'):
            raise ValueError('No slope-triggered gait change is pending.')
        self.phase = self.requested
        self.requested = None
        self.moving_seconds = 0.
        self.since = self.previous = None
