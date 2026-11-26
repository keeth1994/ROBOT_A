"""Slow down to correct heading while paired ramp edges remain observable."""
import math

from .devices import DeviceError


class RampHeadingRecovery:
    def __init__(self):
        self.active = False
        self.centered_since = None
        self.previous = None
        self.message = ''

    def update(self, now, heading, lateral, roll, pitch, *, ready, fresh_edges):
        if not all(math.isfinite(v) for v in (heading, lateral, roll, pitch)):
            raise DeviceError('Ramp drift control received non-finite telemetry.', 409)
        if not self.active:
            if not ready or abs(heading) <= math.radians(12):
                return ready
            self.active = True
        guided = bool(ready and fresh_edges)
        if self.previous is not None and not 0 < now-self.previous <= .9:
            self.centered_since = None
        self.previous = now
        if guided and abs(heading) <= math.radians(7) and abs(lateral) <= .025:
            self.centered_since = now if self.centered_since is None else self.centered_since
            if now-self.centered_since >= .8:
                self.active = False
                self.centered_since = self.previous = None
                self.message = 'Ramp heading recovered; resuming normal guidance.'
                return ready
        else:
            self.centered_since = None
        self.message = ('Recovering ramp heading at reduced forward stride.' if guided else
                        'Holding steps for fresh ramp guidance.')
        return guided
