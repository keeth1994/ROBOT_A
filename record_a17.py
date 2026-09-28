"""Record the real torque-limited simulation, not a kinematic animation."""
from run_robot_a17 import Robot, ROOT
import json,mujoco,imageio.v2 as imageio

def main():
 r=Robot(json.loads((ROOT/'reference/a17_gait.json').read_text()))
 renderer=mujoco.Renderer(r.m,height=720,width=960)
 cam=mujoco.MjvCamera();cam.lookat[:]=[.28,0,.12];cam.distance=1.25;cam.azimuth=135;cam.elevation=-25
 options=mujoco.MjvOption();options.geomgroup[3]=0
 path=ROOT/'results/a17_forward.mp4'
 with imageio.get_writer(path,fps=30,codec='libx264',quality=7) as writer:
  for frame in range(360):
   while r.d.time<(frame+1)/30:r.step()
   renderer.update_scene(r.d,camera=cam,scene_option=options);writer.append_data(renderer.render())
 renderer.close();print(path)
if __name__=='__main__':main()
