"""First-crest wave gait, triggered only by onboard observations."""
import math

from robot_b91.navigation import EDGE_SAMPLE_MAX_GAP_S


# Trial 34's front .45/rear 35% stayed grounded. Try a wider .55 front stroke
# with rear 30%: front excursion increases 22%, rear stays near .16 rad.
# Six 25-degree model cases crossed; five stayed within the 12-degree heading
# bound. Steeper 30-degree cases remain blocked; contact is uncalibrated.
CREST_WAVE_GAIT = dict(duty=.8, yaw_amplitude=.55, rear_stride_scale=.30, pitch_lift=.25,
                       pitch_stance=0., yaw_bias=-.25, pattern='crest')


class RampSpeedGovernor:
    """Slow early; observe an entire stable gait cycle before speeding up."""
    def __init__(self):
        self.slow = False
        self.centered_since = None
        self.previous = None

    def require_stability(self):
        self.slow = True
        self.centered_since = self.previous = None

    def update(self, now, requested, roll, heading, lateral, *, ready, stable_cycle_s=1.):
        if self.previous is not None and not 0 <= now - self.previous <= EDGE_SAMPLE_MAX_GAP_S:
            self.centered_since = None
        self.previous = now
        if abs(roll) >= math.radians(5) or abs(heading) >= math.radians(8) or abs(lateral) >= .02:
            self.slow = True
            self.centered_since = None
        elif (ready and abs(roll) < math.radians(3) and abs(heading) < math.radians(4) and
              abs(lateral) < .012):
            self.centered_since = now if self.centered_since is None else self.centered_since
            if now - self.centered_since >= max(1., stable_cycle_s):
                self.slow = False
        else:
            self.centered_since = None
        return min(requested, .5) if self.slow else requested


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
        if not ready or not 0 < dt <= EDGE_SAMPLE_MAX_GAP_S:
            self.since = None
            return
        self.moving_seconds += dt
        degrees = math.degrees(pitch)
        if self.phase == 'approach':
            condition = degrees <= -15 if not self.climbed else degrees >= -10
            duration = 1.0 if not self.climbed else .4
        else:
            # Trial 21 established descent near +20 degrees. Require at least
            # one full 50%-speed gait cycle before switching back to baseline.
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
