"""Portable navigation: SI IMU samples and RGB frames, never simulator state.

Body axes: X forward, Y left, Z up. Camera is pitched down about body Y.
Calibration must be measured again on the physical robot.
"""
import math
import numpy as np


class Navigation:
    def __init__(self, width_m=0.30, fovy_deg=42.9470936, camera_x_m=0.0626,
                 camera_pitch_deg=15.0, dark_min=3, dark_max=25, max_chroma=.28, max_blind_s=8.0):
        self.width_m = width_m
        self.fovy = math.radians(fovy_deg)
        self.camera_x = camera_x_m
        self.camera_pitch = math.radians(camera_pitch_deg)
        self.dark_min, self.dark_max = dark_min, dark_max
        self.max_chroma = max_chroma
        self.blind_timeout = max_blind_s
        self.roll = self.pitch = self.heading = self.lateral = 0.0
        self.last_imu = None
        self.last_seen = None
        self.frames = self.accepted = 0
        self.bias = np.zeros(3)
        self.bias_samples = []
        self.initialized = False
        self.max_blind_s = 0.0
        self.debug = None

    def imu(self, timestamp, acceleration, angular_velocity):
        a, g = np.asarray(acceleration), np.asarray(angular_velocity)
        if a.shape != (3,) or g.shape != (3,) or not np.isfinite(a).all() or not np.isfinite(g).all():
            return
        dt = 0.0 if self.last_imu is None else timestamp - self.last_imu
        if dt < 0:
            return
        self.last_imu = timestamp
        if dt > .1:
            dt = 0.0  # Resume without integrating across a missing packet interval.
        # Use the stationary end of the startup settling interval for bias.
        if 1.0 < timestamp < 1.9 and np.linalg.norm(g) < 0.05:
            self.bias_samples.append(g.copy())
            self.bias = np.mean(self.bias_samples, axis=0)
        gx, gy, gz = g - self.bias
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        cp = max(0.3, math.cos(self.pitch))
        self.heading += (gy * sr + gz * cr) / cp * dt
        self.roll += (gx + math.tan(self.pitch) * (gy * sr + gz * cr)) * dt
        self.pitch += (gy * cr - gz * sr) * dt
        if 8.0 < np.linalg.norm(a) < 11.5:
            roll = math.atan2(a[1], a[2])
            pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
            gain = 1.0 if not self.initialized else 1 - math.exp(-dt / 1.5)
            self.roll += gain * (roll - self.roll)
            self.pitch += gain * (pitch - self.pitch)
            self.initialized = True

    def frame(self, timestamp, rgb):
        self.frames += 1
        # A fresh diagnostic snapshot for this exact input frame, including failures.
        # Display code must never re-run perception or annotate the sensor input.
        self.debug = dict(timestamp=timestamp, rgb=rgb.copy(), mask=None,
                          rows=[], inliers=[], accepted=False, reason="Invalid RGB image")
        def reject(reason):
            self.debug["reason"] = reason
            return False

        if rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 32:
            return reject("Invalid RGB image")
        h, w = rgb.shape[:2]
        f = h / (2 * math.tan(self.fovy / 2))
        # Threshold black ramp against lighter surroundings. Reject unlit sky.
        gray = rgb.astype(float).mean(axis=2)
        chroma = np.ptp(rgb.astype(float), axis=2) / np.maximum(gray, 1)
        dark = (gray > self.dark_min) & (gray < self.dark_max) & (chroma < self.max_chroma)
        self.debug["mask"] = dark.copy()
        column = np.flatnonzero(dark[:int(h * .98), w // 2])
        if not len(column):
            return reject("No dark surface at image center")
        bands = np.split(column, np.flatnonzero(np.diff(column) > 1) + 1)
        band = max(bands, key=len)
        if len(band) < 12:
            return reject("Dark region too short")
        top, bottom = int(band[0]), int(band[-1])
        rows = []
        for y in range(top + 3, bottom, 2):
            indices = np.flatnonzero(dark[y])
            if len(indices) < 12:
                continue
            runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
            runs = [r for r in runs if r[0] <= w / 2 <= r[-1]]
            if not runs:
                continue
            run = runs[0]
            if len(run) < 12 or run[0] < 3 or run[-1] > w - 4:
                continue
            # Rotate camera rays into a level frame using only IMU attitude.
            rays = np.array([[1., -(x - (w - 1) / 2) / f,
                              -(y - (h - 1) / 2) / f] for x in [run[0], run[-1]]])
            cr, sr = math.cos(self.roll), math.sin(self.roll)
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            rotation = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr],
                                 [-sp, cp * sr, cp * cr]])
            mount = self.camera_pitch
            cm, sm = math.cos(mount), math.sin(mount)
            rays = rays @ np.array([[cm, 0, sm], [0, 1, 0], [-sm, 0, cm]]).T
            rays = rays @ rotation.T
            if np.min(rays[:, 0]) < .2:
                continue
            bearings = rays[:, 1] / rays[:, 0]
            rows.append([bearings[0] - bearings[1], np.mean(bearings)])
            self.debug["rows"].append((y, int(run[0]), int(run[-1])))
        if len(rows) < 3:
            return reject("Too few visible edge pairs (edges may be clipped)")
        widths, centers = np.asarray(rows).T
        keep = np.ones(len(widths), dtype=bool)
        # A narrow visible strip still measures offset using the gyro heading.
        # Only use vision to correct yaw when there is enough depth variation.
        heading = self.heading
        if len(rows) >= 8 and np.ptp(widths) > .12:
            A = np.column_stack([widths, np.ones(len(widths))])
            for _ in range(3):
                slope, intercept = np.linalg.lstsq(A[keep], centers[keep], rcond=None)[0]
                residual = np.abs(centers - A @ [slope, intercept])
                candidate = residual < max(.008, 2.5 * np.median(residual))
                if candidate.sum() < 3:
                    break
                keep = candidate
            visual_heading = -math.atan(intercept)
            if abs(visual_heading - self.heading) < .12:
                heading += (.2 if timestamp < 2 else .025) * (visual_heading - heading)
        offsets = -(centers[keep] + math.tan(heading)) / widths[keep] * self.width_m
        self.debug["inliers"] = keep.tolist()
        lateral = np.median(offsets) - self.camera_x * math.sin(heading)
        if abs(lateral) > self.width_m * .4 or np.median(np.abs(offsets-np.median(offsets))) > .015:
            return reject("Offset out of range or inconsistent edge pairs")
        self.heading = heading
        self.lateral += .3 * (lateral - self.lateral)
        self.last_seen = timestamp
        self.accepted += 1
        self.debug.update(accepted=True, reason="Camera measurement accepted")
        return True

    def ready(self, timestamp):
        """Brief occlusions coast on gyro and held offset; long losses hold gait.

        A bounded grace interval permits clearing the ramp after its end leaves the image.
        This is a configurable deployment policy, not autonomous exit detection.
        """
        age = timestamp - (self.last_seen if self.last_seen is not None else 0.0)
        self.max_blind_s = max(self.max_blind_s, age)
        return (self.last_seen is not None and age <= self.blind_timeout
                and self.last_imu is not None and timestamp - self.last_imu <= .05)

    def report(self):
        return dict(camera_frames=self.frames, accepted_frames=self.accepted,
                    last_seen_s=self.last_seen, heading_rad=self.heading,
                    lateral_m=self.lateral, roll_rad=self.roll, pitch_rad=self.pitch,
                    max_blind_s=self.max_blind_s)
