import json
import math

import numpy as np

from robot_b91.paths import ROOT
from .navigation import CourseNavigation

CALIBRATION_PATH = ROOT / 'dashboard/ir_calibration.local.json'


def load_calibration(path=CALIBRATION_PATH):
    return json.loads(path.read_text()) if path.exists() else {'left': [], 'right': []}


def validate_points(points):
    if not isinstance(points, list):
        raise ValueError('IR calibration must contain distance and voltage pairs.')
    previous = None
    for point in points:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError('Each IR calibration point needs distance in meters and millivolts.')
        distance, voltage = point
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
            raise ValueError('IR calibration values must be finite numbers.')
        if not .04 <= distance <= .30 or not 0 < voltage <= 3300:
            raise ValueError('Use distances from 4 to 30 cm and voltages from 1 to 3300 mV.')
        if previous and (distance <= previous[0] or voltage >= previous[1]):
            raise ValueError('IR voltage must decrease as wall distance increases. Check the measurements.')
        previous = point


def ir_readings(status):
    age, values = status.get('irAgeMs'), status.get('irMillivolts')
    if (isinstance(age, bool) or not isinstance(age, (int, float)) or not 0 <= age <= 150 or
            not isinstance(values, list) or len(values) != 2 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) or not 0 < v <= 3300 for v in values)):
        raise ValueError('Waiting for fresh readings from both IR sensors.')
    return values


def save_calibration_point(side, distance_cm, status, path=CALIBRATION_PATH):
    if side not in ('left', 'right'):
        raise ValueError('Choose the left or right IR sensor.')
    if (isinstance(distance_cm, bool) or not isinstance(distance_cm, (int, float)) or
            not math.isfinite(distance_cm) or not 4 <= distance_cm <= 30):
        raise ValueError('Enter a measured wall distance from 4 to 30 cm.')
    voltage = ir_readings(status)[0 if side == 'left' else 1]
    calibration = load_calibration(path)
    distance = round(distance_cm / 100, 5)
    points = [point for point in calibration[side] if point[0] != distance]
    points.append([distance, voltage])
    points.sort()
    validate_points(points)
    calibration[side] = points
    path.write_text(json.dumps(calibration, indent=2) + '\n')
    return {'message': f'Saved {side} IR reading at {distance_cm:g} cm ({len(points)} points).'}


class CorridorNavigation(CourseNavigation):
    def __init__(self, started, calibration=None):
        super().__init__(started, use_camera=False, use_imu=True)
        self.calibration = load_calibration() if calibration is None else calibration
        for side in ('left', 'right'):
            points = self.calibration.get(side, [])
            validate_points(points)
            if len(points) < 2:
                raise ValueError(f'Calibrate the {side} IR sensor at two or more wall distances first.')
        self.distances = [None, None]
        self.last_update = None
        self.reason = 'Waiting for both walls and fresh IMU readings.'

    def update(self, status, now, camera_frame=None):
        self.distances = [None, None]
        if not super().update(status, now, None):
            self.last_update = None
            return False
        self.estimate.heading = 0.0
        try:
            voltages = ir_readings(status)
            distances = []
            for side, voltage in zip(('left', 'right'), voltages):
                points = self.calibration[side]
                if not points[-1][1] <= voltage <= points[0][1]:
                    raise ValueError(f'{side.capitalize()} wall is outside the calibrated IR range.')
                distances.append(float(np.interp(voltage, [p[1] for p in reversed(points)],
                                                  [p[0] for p in reversed(points)])))
        except ValueError as error:
            self.reason = 'Holding steps: ' + str(error)
            self.last_update = None
            return False
        self.distances = distances
        # Positive offset means the robot is left of center, so steer right.
        offset = max(-.08, min(.08, (distances[1] - distances[0]) / 2))
        gain = 1.0 if self.last_update is None else 1 - math.exp(-max(0, now - self.last_update) / .2)
        self.estimate.lateral += gain * (offset - self.estimate.lateral)
        self.last_update = now
        self.reason = 'Following corridor with side IR sensors and IMU.'
        return True

    def report(self):
        return dict(guidance='corridor', lateral_m=self.estimate.lateral,
                    left_distance_m=self.distances[0], right_distance_m=self.distances[1],
                    detection_reason=self.reason)
