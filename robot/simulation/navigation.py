"""Use simulated camera and IMU readings for navigation."""
import mujoco
import json
import math
import numpy as np
from robot.navigation.controller import Navigation
from robot.paths import ROOT


class SimNavigation:
    def __init__(self, model, scan_settings=None):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        self.scan_settings = scan_settings
        self.camera = model.camera('project_camera')
        self.pivot = model.body('camera_cradle').pos.copy()
        config['camera_x_m'] = float(self.pivot[0] + self.camera.pos[0])
        camera = model.camera('project_camera')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, camera.quat)
        forward = -rotation.reshape(3, 3)[:, 2]
        config['camera_pitch_deg'] = math.degrees(math.atan2(-forward[2], forward[0]))
        if scan_settings:
            from robot.navigation.camera_scan import RampScanNavigation
            self.estimate = RampScanNavigation(scan_settings, **config)
            self.angle_address = model.joint('camera_cradle').qposadr[0]
            self.speed_address = model.joint('camera_cradle').dofadr[0]
        else:
            self.estimate = Navigation(**config)
        self.renderer = mujoco.Renderer(model, height=240, width=320)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0
        self.options.sitegroup[:] = 0
        self.options.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.next_frame = self.next_imu = 0.0
        self.model = model
        self.imu = Navigation()
        self.search = None
        self.search_ready = False
        self.frame_sequence = 0
        self.camera_frame = None

    def begin_corridor_search(self, now):
        from dashboard.robot_dashboard.corridor_search import CorridorSearch
        from dashboard.robot_dashboard.corridor_walls import load_corridor_settings

        corridor = load_corridor_settings()
        self.search_gait_settings = corridor['gait'] | dict(corridor=True, search=True)
        settings = corridor['navigation']
        settings['sensor_positions'] = [self.model.site(name + '_site').pos[:2].tolist()
                                        for name in ('ir_left', 'ir_right')]
        # Estimate movement from gait speed.
        settings['walk_speed'] = .032
        camera = self.model.camera('project_camera')
        pivot = self.model.body('camera_cradle').pos
        geometry = dict(corridor['camera'], height_m=float(pivot[2] + camera.pos[2]) + .047,
                        forward_m=float(pivot[0] + camera.pos[0]),
                        fovy_deg=float(camera.fovy[0]))
        self.search = CorridorSearch(now, settings, camera_geometry=geometry)
        # Tilt the camera view without moving the CAD mount.
        self.set_search_pitch(geometry['pitch_deg'])
        self.next_frame = now
        self.next_search_update = now

    def set_search_pitch(self, degrees):
        self.search.camera_geometry['pitch_deg'] = degrees
        pitch = math.radians(degrees)
        axes = np.array([[0, math.sin(pitch), -math.cos(pitch)],
                         [-1, 0, 0], [0, math.cos(pitch), math.sin(pitch)]])
        mujoco.mju_mat2Quat(self.model.cam_quat[self.model.camera('project_camera').id], axes.ravel())

    def update(self, data):
        if data.time >= self.next_imu:
            acceleration = data.sensor('imu_acceleration').data.copy()
            gyro = data.sensor('imu_angular_velocity').data.copy()
            self.imu.imu(data.time, acceleration, gyro)
            if not self.search:
                self.estimate.imu(data.time, acceleration, gyro)
            self.next_imu = data.time + .005
        if data.time >= self.next_frame:
            self.renderer.update_scene(data, camera='project_camera', scene_option=self.options)
            rgb = self.renderer.render()
            self.frame_sequence += 1
            self.camera_frame = (self.frame_sequence, data.time, rgb)
            if not self.search:
                if self.scan_settings:
                    angle = float(data.qpos[self.angle_address])
                    camera_x = (self.pivot[0] + math.cos(angle) * self.camera.pos[0]
                                + math.sin(angle) * self.camera.pos[2])
                    self.estimate.scan_frame(data.time, rgb, angle,
                                             float(data.qvel[self.speed_address]), camera_x)
                else:
                    self.estimate.frame(data.time, rgb)
            self.next_frame = data.time + .1
        if self.search and data.time >= self.next_search_update:
            settings = self.search.settings
            distances = [float(data.sensor(name).data[0]) for name in ('ir_left', 'ir_right')]
            volts = [100 if d < 0 or d > .30 else min(3300, settings['ir_offset_mv'][i] +
                     settings['ir_scale_mv_m'][i] / max(d, .001)) for i, d in enumerate(distances)]
            status = dict(irMillivolts=volts, irAgeMs=0, orientationValid=self.imu.initialized,
                          imuAgeMs=(data.time - self.imu.last_imu) * 1000,
                          orientation=[self.imu.roll, self.imu.pitch, self.imu.heading])
            self.search_ready = self.search.update(status, data.time, self.camera_frame)
            self.next_search_update = data.time + .05
        if self.search:
            return (self.search.estimate.heading + self.imu.heading - self.search.imu_heading,
                    self.search.estimate.lateral)
        return self.estimate.heading, self.estimate.lateral_error

    def close(self):
        self.renderer.close()
