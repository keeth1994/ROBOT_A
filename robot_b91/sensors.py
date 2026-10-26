import mujoco
from robot_b91.navigation_debug import navigation_debug_image, navigation_status


class SensorOverlay:
    def __init__(self, robot, viewer):
        self.robot = robot
        self.viewer = viewer
        self.renderer = None if robot.navigation else mujoco.Renderer(robot.m, height=240, width=320)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0  # Hide collision hulls, retain CAD visuals.
        self.options.sitegroup[:] = 0
        self.options.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.next_frame = 0.0

    def update(self, now):
        if now < self.next_frame:
            return
        self.next_frame = now + 1 / 15
        if self.robot.navigation:
            panel = navigation_debug_image(self.robot.navigation.estimate, self.robot.d.time)
            self.viewer.set_images((mujoco.MjrRect(10, 10, 640, 318), panel))
        else:
            self.renderer.update_scene(
                self.robot.d, camera="project_camera", scene_option=self.options
            )
            self.viewer.set_images((
                mujoco.MjrRect(10, 10, 320, 240), self.renderer.render()
            ))
        acceleration = self.robot.d.sensor("imu_acceleration").data
        gyro = self.robot.d.sensor("imu_angular_velocity").data
        vector = lambda values: "  ".join(f"{v:+8.3f}" for v in values)
        labels = "Ideal IMU (local X Y Z)\nAcceleration (m/s^2)\nGyroscope (rad/s)"
        values = f"t = {self.robot.d.time:.2f} s\n{vector(acceleration)}\n{vector(gyro)}"
        if self.robot.navigation:
            import math
            n = self.robot.navigation.estimate
            labels += "\nEstimated heading (deg)\nEstimated lateral (mm)\nCamera age (s)"
            age = self.robot.d.time - n.last_seen if n.last_seen is not None else float('inf')
            values += f"\n{math.degrees(n.heading):+.2f}\n{n.lateral * 1000:+.1f}\n{age:.2f}"
            labels += "\nNavigation status"
            values += "\n" + navigation_status(n, self.robot.d.time)[0]
        self.viewer.set_texts((
            mujoco.mjtFontScale.mjFONTSCALE_100,
            mujoco.mjtGridPos.mjGRID_TOPRIGHT,
            labels,
            values,
        ))

    def close(self):
        if self.renderer:
            self.renderer.close()
