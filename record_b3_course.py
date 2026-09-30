"""Record the real torque-limited B3 run at 2x playback; no pose animation."""
import argparse,json
import imageio.v2 as imageio
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from run_robot_b3 import Robot,ROOT

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=100);a=ap.parse_args()
 course=json.loads((ROOT/'reference/b3_course.json').read_text())
 r=Robot(json.loads((ROOT/'reference/b3_gait.json').read_text()),True)
 renderer=mujoco.Renderer(r.m,height=720,width=1280);cam=mujoco.MjvCamera();cam.distance=course['camera_distance'];cam.azimuth=100;cam.elevation=-38;cam.lookat[:]=course['camera_lookat']
 opt=mujoco.MjvOption();opt.geomgroup[3]=0
 out=ROOT/'results/b3_flat_up_down.mp4'
 renderer.update_scene(r.d,camera=cam,scene_option=opt);Image.fromarray(renderer.render()).save(ROOT/'results/b3_course_overview.png')
 with imageio.get_writer(out,fps=20,codec='libx264',quality=7) as writer:
  for f in range(round(a.seconds*10)):
   for _ in range(50):r.step()
   renderer.update_scene(r.d,camera=cam,scene_option=opt);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
   draw.rectangle((0,0,1280,58),fill=(20,24,32));draw.text((16,9),f"B3 | PROGRAMMED GAIT | PHOTO COURSE: TWO CLIMBS / VALLEY / PLATFORM / DESCENT | {course['width_m']*1000:.0f} mm ramp | 2x playback",fill='white')
   draw.text((16,32),f't = {r.d.time:.1f} s   x = {r.d.qpos[0]:.3f} m   y = {r.d.qpos[1]:.3f} m   mass = {sum(r.m.body_mass):.3f} kg',fill='white');writer.append_data(np.array(im))
   if f%100==0:print(f'{r.d.time:.1f}/{a.seconds}s',flush=True)
   if f==round(a.seconds*10)-1:im.save(ROOT/'results/b3_course_final.png')
 renderer.close();print(out)
if __name__=='__main__':main()
