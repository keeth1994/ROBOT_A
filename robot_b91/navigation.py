"""Portable navigation: SI IMU samples and RGB frames, never simulator state.

Body axes: X forward, Y left, Z up. Camera is pitched down about body Y.
Calibration must be measured again on the physical robot.
"""
import math
import numpy as np


class Navigation:
    def __init__(self, width_m=0.30, fovy_deg=42.9470936, camera_x_m=0.0626,
                 camera_pitch_deg=15.0, dark_min=3, dark_max=25, max_chroma=.28, max_blind_s=8.0,
                 adaptive_surface=False, guidance='ramp', tape_width_m=0.025,
                 camera_y_m=0.0, camera_z_m=0.0):
        if guidance not in ('ramp', 'blue_tape'):
            raise ValueError('Guidance must be ramp or blue_tape.')
        if not math.isfinite(tape_width_m) or tape_width_m <= 0:
            raise ValueError('Tape width must be a positive number in meters.')
        self.guidance = guidance
        self.tape_width_m = tape_width_m
        self.width_m = width_m
        self.fovy = math.radians(fovy_deg)
        self.camera_x = camera_x_m
        self.camera_y = camera_y_m
        self.camera_z = camera_z_m
        self.camera_pitch = math.radians(camera_pitch_deg)
        self.dark_min, self.dark_max = dark_min, dark_max
        self.max_chroma = max_chroma
        self.blind_timeout = max_blind_s
        self.adaptive_surface = adaptive_surface
        if adaptive_surface:
            from .ramp_detection import detect_ramp
            self.surface_detector = detect_ramp
        self.roll = self.pitch = self.heading = self.lateral = 0.0
        self.last_imu = None
        self.last_seen = None
        self.frames = self.accepted = 0
        self.bias = np.zeros(3)
        self.bias_samples = []
        self.initialized = False
        self.heading_initialized = False
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

    def frame(self, timestamp, rgb, use_imu=True, *, surface=None):
        self.frames += 1
        # A fresh diagnostic snapshot for this exact input frame, including failures.
        # Display code must never re-run perception or annotate the sensor input.
        self.debug = dict(timestamp=timestamp, rgb=rgb.copy(), mask=None,
                          rows=[], inliers=[], accepted=False, surface_detected=False,
                          reason="Invalid RGB image")
        def reject(reason):
            self.debug["reason"] = reason
            return False

        if rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 32:
            return reject("Invalid RGB image")
        h, w = rgb.shape[:2]
        f = h / (2 * math.tan(self.fovy / 2))
        if self.guidance == 'blue_tape':
            from .tape_detection import detect_blue_tape
            dark, edges = detect_blue_tape(rgb)
            self.debug['mask'] = dark
            self.debug['surface_detected'] = bool(edges)
            if not edges:
                return reject("No clear blue tape found")
        elif self.adaptive_surface:
            # Pitching over the top inverts the rear image. Detect on an upright view,
            # then put the edges back in raw camera coordinates for ray projection.
            rear = getattr(self, 'view_sign', 1) < 0
            upright = rgb[::-1, ::-1] if rear else rgb
            dark, edges = (surface if surface is not None else
                           self.surface_detector(upright, self.dark_min, self.dark_max, self.max_chroma))
            if rear:
                dark = dark[::-1, ::-1]
                edges = sorted((h - 1 - y, w - 1 - right, w - 1 - left) for y, left, right in edges)
            self.debug['mask'] = dark
            self.debug['surface_detected'] = bool(dark.any())
            if not dark.any():
                return reject("No coherent ramp surface found")
            if not edges:
                return reject("Ramp detected; both side edges are not sufficiently visible")
        else:
            # Preserve the simulator's threshold detector.
            gray = rgb.astype(float).mean(axis=2)
            chroma = np.ptp(rgb.astype(float), axis=2) / np.maximum(gray, 1)
            dark = (gray > self.dark_min) & (gray < self.dark_max) & (chroma < self.max_chroma)
            self.debug['mask'] = dark.copy()
            column = np.flatnonzero(dark[:int(h * .98), w // 2])
            if not len(column):
                return reject("No dark surface at image center")
            bands = np.split(column, np.flatnonzero(np.diff(column) > 1) + 1)
            band = max(bands, key=len)
            if len(band) < 12:
                return reject("Dark region too short")
            self.debug['surface_detected'] = True
            top, bottom = int(band[0]), int(band[-1])
            edges = []
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
                edges.append((y, int(run[0]), int(run[-1])))
        # Camera mounting and IMU attitude are the same for every edge row.
        cr, sr = (math.cos(self.roll), math.sin(self.roll)) if use_imu else (1, 0)
        cp, sp = (math.cos(self.pitch), math.sin(self.pitch)) if use_imu else (1, 0)
        rotation = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])
        cm, sm = math.cos(self.camera_pitch), math.sin(self.camera_pitch)
        mount = np.array([[cm, 0, sm], [0, 1, 0], [-sm, 0, cm]])
        ray_pairs = []
        for y, left, right in edges:
            rays = np.array([[1., -(x - (w - 1) / 2) / f,
                              -(y - (h - 1) / 2) / f] for x in [left, right]])
            rays = rays @ mount.T
            rays = rays @ rotation.T
            if np.min(rays[:, 0] * getattr(self, "view_sign", 1)) < .2:
                continue
            ray_pairs.append(rays)
            self.debug["rows"].append((y, left, right))
        if len(ray_pairs) < 3:
            return reject("Too few visible edge pairs (edges may be clipped)")
        # Equal image rows cease to represent equal elevations when the body
        # rolls. Pair the two edges after attitude correction, without
        # extrapolating beyond the part visible on both sides.
        rays = np.asarray(ray_pairs)
        elevation = rays[:, :, 2] / rays[:, :, 0]
        bearing = rays[:, :, 1] / rays[:, :, 0]
        right_order = np.argsort(elevation[:, 1])
        right_elevation, unique = np.unique(elevation[right_order, 1], return_index=True)
        if len(right_elevation) < 3:
            return reject("Too little edge elevation to pair both sides")
        right_bearing = bearing[right_order[unique], 1]
        paired = np.flatnonzero((elevation[:, 0] >= right_elevation[0]) &
                               (elevation[:, 0] <= right_elevation[-1]))
        if len(paired) < 3:
            return reject("Too little overlapping edge elevation after tilt correction")
        left = bearing[paired, 0]
        right = np.interp(elevation[paired, 0], right_elevation, right_bearing)
        widths, centers = left - right, (left + right) / 2
        if np.any(np.abs(widths) < 1e-6):
            return reject("Degenerate edge pair after tilt correction")
        self.debug['rows'] = [self.debug['rows'][i] for i in paired]
        keep = np.ones(len(widths), dtype=bool)
        # A narrow visible strip still measures offset using the gyro heading.
        # Only use vision to correct yaw when there is enough depth variation.
        heading = self.heading
        heading_observed = False
        depth_threshold = .01 if self.guidance == 'blue_tape' else (.025 if self.adaptive_surface else .12)
        if len(paired) >= 8 and np.ptp(widths) > depth_threshold:
            A = np.column_stack([widths, np.ones(len(widths))])
            for _ in range(3):
                slope, intercept = np.linalg.lstsq(A[keep], centers[keep], rcond=None)[0]
                residual = np.abs(centers - A @ [slope, intercept])
                candidate = residual < max(.008, 2.5 * np.median(residual))
                if candidate.sum() < 3:
                    break
                keep = candidate
            visual_heading = -math.atan(intercept)
            heading_observed = True
            # Camera turns clear last_seen and enter search, but neither event
            # invalidates the heading already established and propagated by IMU.
            # A partial rear view must not reset it to an unrelated visual fit.
            difference = math.atan2(math.sin(visual_heading - heading), math.cos(visual_heading - heading))
            if not use_imu or not self.heading_initialized:
                heading = visual_heading
            elif abs(difference) < .12:
                heading += (.2 if timestamp < 2 else .025) * difference
        elif not use_imu:
            return reject("Too little edge depth to estimate heading without IMU")
        measured_width = self.tape_width_m if self.guidance == 'blue_tape' else self.width_m
        offsets = -(centers[keep] + math.tan(heading)) / widths[keep] * measured_width
        self.debug["inliers"] = keep.tolist()
        # Edges locate the lens, while gait corrections act on the body origin.
        # A high camera moves sideways as the robot rolls and forward as it
        # pitches; removing only camera_x*sin(heading) mistakes that motion for
        # body drift. Use the same body attitude as the image-ray projection.
        camera_offset = rotation @ np.array([self.camera_x, self.camera_y, self.camera_z])
        lateral = np.median(offsets) - (
            camera_offset[0] * math.sin(heading) + camera_offset[1] * math.cos(heading))
        if abs(lateral) > self.width_m * .4 or np.median(np.abs(offsets-np.median(offsets))) > .015:
            # Diagnostic only: a stale yaw prior can put two good edges outside
            # the ramp. The dashboard may verify the independent fit while held.
            if (self.adaptive_surface and self.guidance == 'ramp' and use_imu and heading_observed
                    and keep.sum() >= 12 and keep.mean() >= .75 and np.ptp(widths[keep]) > depth_threshold):
                alt = -(centers[keep] + math.tan(visual_heading)) / widths[keep] * measured_width
                mid = np.median(alt)
                body = mid - (camera_offset[0]*math.sin(visual_heading)
                              + camera_offset[1]*math.cos(visual_heading))
                if np.median(np.abs(alt-mid)) <= .006 and abs(body) <= .2*self.width_m:
                    self.debug['edge_pose'] = (visual_heading, float(body))
            return reject("Offset out of range or inconsistent edge pairs")
        self.heading = heading
        self.heading_initialized = self.heading_initialized or heading_observed
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
                    guidance=self.guidance,
                    ramp_detected=bool(self.debug and self.debug['surface_detected']),
                    detection_reason=self.debug['reason'] if self.debug else 'Waiting for camera',
                    last_seen_s=self.last_seen, heading_rad=self.heading,
                    lateral_m=self.lateral, roll_rad=self.roll, pitch_rad=self.pitch,
                    max_blind_s=self.max_blind_s)
