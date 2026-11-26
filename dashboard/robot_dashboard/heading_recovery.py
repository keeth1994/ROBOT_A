"""Re-establish heading only from agreeing forward views while steps are held."""
from collections import deque
import math
from statistics import median

from robot_b91.navigation import EDGE_SAMPLE_MAX_GAP_S, EDGE_SAMPLE_WINDOW_S


def difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class PausedHeadingRecovery:
    def __init__(self, angles):
        self.angles = tuple(dict.fromkeys(angles))
        self.samples = deque()
        self.anchor = None
        self.attempted = False
        self.count = 0

    def reset(self):
        self.samples.clear()
        self.anchor = None
        self.attempted = False

    def missing(self):
        self.samples.clear()

    def observe(self, now, angle, heading, lateral, attitude, tracked_heading):
        if self.attempted or len(self.angles) != 2:
            return None
        if self.anchor and now-self.anchor['time'] > 6:
            self.attempted = True
            return None
        if (angle not in self.angles or
                not all(math.isfinite(v) for v in (now, heading, lateral, tracked_heading, *attitude)) or
                abs(heading) > math.radians(12) or abs(lateral) > .05 or
                abs(difference(heading, tracked_heading)) > math.radians(20)):
            self.missing()
            return None
        if self.anchor and angle == self.anchor['angle']:
            return None  # Wait for the second camera angle, with the legs held.
        if self.samples and (angle != self.samples[-1][1] or
                             not 0 < now-self.samples[-1][0] <= EDGE_SAMPLE_MAX_GAP_S):
            self.missing()
        self.samples.append((now, angle, heading, lateral, tuple(attitude)))
        while self.samples and now-self.samples[0][0] > EDGE_SAMPLE_WINDOW_S:
            self.samples.popleft()
        if len(self.samples) < 4 or now-self.samples[0][0] < .6:
            return None
        headings = [s[2] for s in self.samples]
        offsets = [s[3] for s in self.samples]
        stationary = all(abs(difference(s[4][axis], self.samples[0][4][axis])) <= math.radians(2)
                         for s in self.samples for axis in range(3))
        if not stationary or max(headings)-min(headings) > math.radians(2) or max(offsets)-min(offsets) > .015:
            return None
        pose = dict(time=now, angle=angle, heading=median(headings), lateral=median(offsets),
                    attitude=tuple(attitude))
        self.samples.clear()
        if self.anchor is None:
            self.anchor = pose
            return {'target_angle': next(a for a in self.angles if a != angle)}
        anchor = self.anchor
        self.attempted = True
        if (any(abs(difference(attitude[i], anchor['attitude'][i])) > math.radians(2) for i in range(3)) or
                abs(difference(pose['heading'], anchor['heading'])) > math.radians(2.5) or
                abs(pose['lateral']-anchor['lateral']) > .015):
            return None
        self.count += 1
        return dict(heading=(pose['heading']+anchor['heading'])/2,
                    lateral=(pose['lateral']+anchor['lateral'])/2)
