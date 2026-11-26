"""Bounded heading correction while paired ramp edges remain observable."""
import math

from .devices import DeviceError


class RampHeadingRecovery:
    def __init__(self):
        self.started = None
        self.centered_since = None
        self.previous = None
        self.message = ''

    @property
    def active(self):
        return self.started is not None

    def update(self, now, heading, lateral, roll, pitch, *, ready, fresh_edges):
        if (not all(math.isfinite(v) for v in (heading, lateral, roll, pitch)) or
                abs(roll) >= math.radians(12) or abs(pitch) >= math.radians(35)):
            raise DeviceError('Ramp drift control lost a valid attitude or exceeded its tilt limit.', 409)
        if not self.active:
            if not ready or abs(heading) <= math.radians(12):
                return ready
            self.started = now
        if abs(heading) >= math.radians(20) or abs(lateral) >= .05:
            raise DeviceError('Ramp recovery exceeded its heading or edge-offset limit; outputs off requested.', 409)
        if now - self.started >= 15:
            raise DeviceError('Ramp heading did not recover within 15 seconds; outputs off requested.', 409)
        # Missing guidance consumes the recovery budget, but never permits blind
        # correction. Avoid motion near the roll stop while waiting to settle.
        guided = bool(ready and fresh_edges and abs(roll) < math.radians(10))
        if self.previous is not None and not 0 < now-self.previous <= .9:
            self.centered_since = None
        self.previous = now
        if guided and abs(heading) <= math.radians(7) and abs(lateral) <= .025:
            self.centered_since = now if self.centered_since is None else self.centered_since
            if now-self.centered_since >= .8:
                self.started = self.centered_since = self.previous = None
                self.message = 'Ramp heading recovered; resuming normal guidance.'
                return ready
        else:
            self.centered_since = None
        remaining = max(0, 15-(now-self.started))
        self.message = (f'Recovering ramp heading at reduced forward stride; {remaining:.0f}s remaining.' if guided else
                        f'Holding steps for fresh ramp edges and stable tilt; {remaining:.0f}s recovery remaining.')
        return guided
