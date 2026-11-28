"""Handle the selected crest using IMU readings."""
import math

MAX_SAMPLE_GAP_S = .9


# Lift one foot at a time.
CREST_WAVE_GAIT = dict(duty=.8, yaw_amplitude=.55, rear_stride_scale=.30, pitch_lift=.25,
                       front_pitch_lift=math.radians(50), pitch_stance=0., yaw_bias=-.25, pattern='crest')


class CrestSequence:
    def __init__(self, position='first'):
        if position not in ('first', 'last'):
            raise ValueError('Crest position must be first or last.')
        self.target_climb = 1 if position == 'first' else 2
        self.climbs_seen = 0
        self.waiting_descent = False
        self.phase = 'approach'
        self.climbed = False
        self.since = self.previous = None
        self.moving_seconds = 0.
        self.requested = None
        self.exit_candidate = None

    def observe(self, now, pitch, ready):
        dt = 0 if self.previous is None else now - self.previous
        self.previous = now
        if self.requested or self.phase in ('descent', 'plateau'):
            return
        if not ready or not 0 < dt <= MAX_SAMPLE_GAP_S:
            self.since = None
            return
        self.moving_seconds += dt
        degrees = math.degrees(pitch)
        if self.waiting_descent:
            condition = degrees >= 14
            duration = .6
        elif self.phase == 'approach':
            condition = degrees <= -15 if not self.climbed else degrees >= -10
            duration = 1.0 if not self.climbed else .4
        else:
            candidate = None
            if self.moving_seconds >= 4:
                if degrees >= 14:
                    candidate = 'descent'
                elif self.climbed and abs(degrees) <= 4:
                    candidate = 'plateau'
            if candidate != self.exit_candidate:
                self.since = None
            self.exit_candidate = candidate
            condition = candidate is not None
            # Wait for level pitch through a full gait cycle.
            # Count walking time, not distance.
            duration = 5.0 if candidate == 'plateau' else .6
        if not condition:
            self.since = None
            return
        self.since = now if self.since is None else self.since
        if now - self.since < duration:
            return
        self.since = None
        if self.waiting_descent:
            self.waiting_descent = False
            self.climbed = False
        elif self.phase == 'approach' and not self.climbed:
            self.climbed = True
            self.climbs_seen += 1
        elif self.phase == 'approach' and self.climbs_seen < self.target_climb:
            self.waiting_descent = True
        else:
            self.requested = 'crest' if self.phase == 'approach' else self.exit_candidate

    def switched(self, now):
        if self.requested not in ('crest', 'descent', 'plateau'):
            raise ValueError('No slope-triggered gait change is pending.')
        self.phase = self.requested
        self.requested = None
        self.moving_seconds = 0.
        self.since = self.previous = None
