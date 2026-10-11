"""Optional headless recording of the same physics used by evaluation."""

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


class Recording:
    """10 samples per simulated second, played at 20 fps (2x speed)."""

    def __init__(self, path):
        self.writer = imageio.get_writer(str(path), fps=20, codec="libx264", quality=7)
        self.renderer = None
        self.next_frame = 0.0
        self.font = ImageFont.load_default(size=20)
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 0.7
        self.camera.azimuth = 100
        self.camera.elevation = -28
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0

    def __call__(self, robot, progress):
        if robot.d.time < self.next_frame and not (progress and progress.completed_at):
            return
        self.next_frame += 0.1
        if self.renderer is None:
            self.renderer = mujoco.Renderer(robot.m, height=640, width=960)
        self.camera.lookat[:] = robot.d.qpos[:3] + [0, 0, 0.035]
        self.renderer.update_scene(robot.d, camera=self.camera, scene_option=self.options)
        frame = Image.fromarray(self.renderer.render())
        draw = ImageDraw.Draw(frame)
        draw.rectangle((0, 0, 960, 38), fill=(20, 25, 30))
        status = "COMPLETE" if progress and progress.completed_at else "SIMULATION | 2x playback"
        draw.text((12, 8), f"B9-1 | {robot.d.time:.1f} s | {status}", fill="white", font=self.font)
        if progress:
            c = progress.course
            draw.rectangle((0, 570, 960, 640), fill=(20, 25, 30))
            points = [(s["start_x_m"], s["start_height_m"]) for s in c["segments"]]
            points.append((c["segments"][-1]["end_x_m"], c["segments"][-1]["end_height_m"]))
            pixel = lambda x, z: (80 + x / c["downhill_end_x"] * 800, 627 - z * 200)
            draw.line([pixel(x, z) for x, z in points], fill="white", width=3)
            x, _, z = progress.local_position
            px, py = pixel(float(np.clip(x, 0, c["downhill_end_x"])), z)
            draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(70, 220, 160))
        self.writer.append_data(np.asarray(frame))

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
        self.writer.close()
