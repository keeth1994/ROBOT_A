"""Sensor acquisition adapter. Only RGB, accelerometer and gyro cross the boundary."""
import mujoco
import json
from robot_b91.navigation import Navigation
from robot_b91.paths import ROOT


class SimNavigation:
    def __init__(self, model):
        config = json.loads((ROOT / 'reference/b91_navigation.json').read_text())
        config['camera_x_m'] = float(model.camera('project_camera').pos[0])
        self.estimate = Navigation(**config)
        self.renderer = mujoco.Renderer(model, height=240, width=320)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0
        self.options.sitegroup[:] = 0
        self.options.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.next_frame = self.next_imu = 0.0

    def update(self, data):
        if data.time >= self.next_imu:
            self.estimate.imu(data.time, data.sensor('imu_acceleration').data.copy(),
                              data.sensor('imu_angular_velocity').data.copy())
            self.next_imu = data.time + .005
        if data.time >= self.next_frame:
            self.renderer.update_scene(data, camera='project_camera', scene_option=self.options)
            self.estimate.frame(data.time, self.renderer.render())
            self.next_frame = data.time + .1
        return self.estimate.heading, self.estimate.lateral

    def close(self):
        self.renderer.close()
