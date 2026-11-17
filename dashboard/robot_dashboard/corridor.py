"""Center between walls using side IR voltages, without distance calibration."""
import math

from .navigation import CourseNavigation

# A low signal suggests a distant/missing wall; this is not a distance measurement.
WEAK_SIGNAL_MV = 150


def ir_readings(status, allow_zero=False):
    age, values = status.get('irAgeMs'), status.get('irMillivolts')
    if (isinstance(age, bool) or not isinstance(age, (int, float)) or not 0 <= age <= 150 or
            not isinstance(values, list) or len(values) != 2 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) or not 0 <= v <= 3300 or (v == 0 and not allow_zero)
                for v in values)):
        raise ValueError('Waiting for fresh readings from both IR sensors.')
    return values



class CorridorNavigation(CourseNavigation):
    def __init__(self, started, balance_gain=1.0, rate_gain=1.0, max_steering=.4, filter_s=.15,
                 wall_guard=True):
        super().__init__(started, use_camera=False, use_imu=True)
        self.filter_s = filter_s
        self.balance_gain = balance_gain
        self.rate_gain = rate_gain
        self.max_steering = max_steering
        self.wall_guard = wall_guard
        self.voltages = [None, None]
        self.balance = 0.0
        self.balance_rate = 0.0
        self.last_update = None
        self.last_both_seen = None
        self.last_both_heading = 0.0
        self.steering = 0.0
        self.reason = 'Waiting for side IR signals and fresh IMU readings.'

    def update(self, status, now, camera_frame=None):
        self.voltages = [None, None]
        if not super().update(status, now, None):
            self.last_update = None
            return False
        try:
            left, right = ir_readings(status)
            self.voltages = [left, right]
            if max(left, right) >= 3200:
                raise ValueError('An IR signal is saturated. Check wall clearance.')
            weak = [left < WEAK_SIGNAL_MV, right < WEAK_SIGNAL_MV]
            if all(weak):
                raise ValueError('Both IR signals are too weak to follow the corridor.')
            previous_balance = self.balance
            raw_balance = (left - right) / (left + right)
            dt = None if self.last_update is None else now - self.last_update
            gain = 1 if dt is None or self.filter_s <= 0 else 1 - math.exp(-max(0, dt) / self.filter_s)
            self.balance = raw_balance if self.filter_s <= 0 else self.balance + gain * (raw_balance - self.balance)
            if dt is None or not .02 <= dt <= .3:
                self.balance_rate = 0.0
            else:
                rate = max(-2.0, min(2.0, (self.balance - previous_balance) / dt))
                self.balance_rate += (1 - math.exp(-dt / .12)) * (rate - self.balance_rate)
            if any(weak):
                if (self.last_both_seen is None or now - self.last_both_seen > 1.0 or
                        abs(self.imu_heading - self.last_both_heading) > math.radians(10)):
                    raise ValueError('Could not regain both wall signals. Reposition the robot.')
                target = -.035 if weak[0] else .035
                self.balance_rate = 0.0
                self.reason = f'Turning toward the {"left" if weak[0] else "right"} wall to regain its signal.'
            else:
                self.last_both_seen = now
                self.last_both_heading = self.imu_heading
                # Higher left voltage means a nearer left wall: positive steering turns right.
                error = math.copysign(max(0.0, abs(self.balance) - .02), self.balance)
                # Ease off before reaching center; countersteer if crossing it quickly.
                target = max(-self.max_steering, min(self.max_steering, self.balance_gain * error + self.rate_gain * self.balance_rate))
                self.reason = 'Centering between walls using IR signal balance and IMU.'
                if self.wall_guard and abs(raw_balance) > .12 and target * raw_balance < 0:
                    # A filter or braking term must not keep turning toward the nearer wall.
                    target = max(-.10, min(.10, .3 * raw_balance))
                    self.reason = 'Easing out of the turn to keep clear of the nearer wall.'
        except ValueError as error:
            self.reason = 'Holding steps: ' + str(error)
            self.last_update = None
            return False
        # Smooth the IR signal and its rate, rather than delaying the steering output.
        self.steering = target
        self.last_update = now
        # Send a steering correction, not a fabricated distance in meters.
        # The ESP32 gyro steadies this direction between PC updates.
        self.estimate.heading = self.steering
        self.estimate.lateral = 0.0
        return True

    def report(self):
        return dict(guidance='corridor', left_mv=self.voltages[0], right_mv=self.voltages[1],
                    signal_balance=self.balance, steering_rad=self.steering,
                    detection_reason=self.reason)
