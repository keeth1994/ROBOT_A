import mujoco


class SensorOverlay:
    def __init__(self, robot, viewer):
        self.robot = robot
        self.viewer = viewer
        self.renderer = mujoco.Renderer(robot.m, height=240, width=320)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0  # Hide collision hulls, retain CAD visuals.
        self.next_frame = 0.0

    def update(self, now):
        if now < self.next_frame:
            return
        self.next_frame = now + 1 / 15
        self.renderer.update_scene(
            self.robot.d, camera="project_camera", scene_option=self.options
        )
        self.viewer.set_images((
            mujoco.MjrRect(10, 10, 320, 240), self.renderer.render()
        ))
        acceleration = self.robot.d.sensor("imu_acceleration").data
        gyro = self.robot.d.sensor("imu_angular_velocity").data
        vector = lambda values: "  ".join(f"{v:+8.3f}" for v in values)
        self.viewer.set_texts((
            mujoco.mjtFontScale.mjFONTSCALE_100,
            mujoco.mjtGridPos.mjGRID_TOPRIGHT,
            "Ideal IMU (local X Y Z)\nAcceleration (m/s^2)\nGyroscope (rad/s)",
            f"t = {self.robot.d.time:.2f} s\n{vector(acceleration)}\n{vector(gyro)}",
        ))

    def close(self):
        self.renderer.close()
