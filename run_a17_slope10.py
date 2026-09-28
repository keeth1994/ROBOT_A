"""A17 on a 10-degree uphill plane. Same gait, gravity, mass and servo limits as flat test.
Robot starts aligned with the slope; this does not test the flat-to-ramp transition.
"""
from run_robot_a17 import Robot, ROOT
import argparse,csv,json,math,time
import xml.etree.ElementTree as ET
import mujoco,numpy as np
ANGLE=math.radians(10)
NORMAL=np.array([-math.sin(ANGLE),0,math.cos(ANGLE)])
UPHILL=np.array([math.cos(ANGLE),0,math.sin(ANGLE)])
ROT=np.array([[math.cos(ANGLE),0,-math.sin(ANGLE)],[0,1,0],[math.sin(ANGLE),0,math.cos(ANGLE)]])
MODEL=ROOT/'models/robot_a17_slope10.xml'
def build():
 tree=ET.parse(ROOT/'models/robot_a17.xml');root=tree.getroot();root.set('model','A17 - 10 degree uphill')
 floor=root.find("worldbody/geom[@name='floor']");floor.set('quat',f'{math.cos(ANGLE/2)} 0 {-math.sin(ANGLE/2)} 0')
 ET.indent(tree);tree.write(MODEL,encoding='utf-8',xml_declaration=True)
class SlopeRobot(Robot):
 def __init__(self):
  super().__init__(json.loads((ROOT/'reference/a17_gait.json').read_text()),MODEL)
 def reset(self):
  super().reset()
  self.d.qpos[:3]=ROT@self.d.qpos[:3]
  self.d.qpos[3:7]=[math.cos(ANGLE/2),0,-math.sin(ANGLE/2),0]
  mujoco.mj_forward(self.m,self.d)
def evaluate(seconds=30):
 r=SlopeRobot();rows=[];start=None;max_tilt=0;badcount=0;count=0;sat=0;fallen=False;min_clear=1.;backslide=0.;previous=None
 while r.d.time<seconds:
  r.step();tilt,heading,bad,nfeet=r.metrics()
  up=r.d.xmat[r.m.body('platform').id].reshape(3,3)[:,2]
  relative=math.degrees(math.acos(np.clip(up@NORMAL,-1,1)));clear=float(r.d.qpos[:3]@NORMAL);progress=float(r.d.qpos[:3]@UPHILL)
  if r.d.time>=2:
   if start is None:start=r.d.qpos[:3].copy()
   max_tilt=max(max_tilt,relative);min_clear=min(min_clear,clear);badcount+=bad;count+=1;sat+=np.count_nonzero(abs(r.torque)>.95*r.stall)
   fallen|=relative>45 or clear<.055
   if previous is not None:backslide+=max(0,previous-progress)
   previous=progress
  if int(r.d.time/r.m.opt.timestep)%10==0:rows.append([r.d.time,*r.d.qpos[:3],progress,clear,relative,heading,nfeet,*r.torque])
 delta=r.d.qpos[:3]-start
 result=dict(slope_deg=10,starts_on_slope=True,transition_tested=False,duration_s=seconds,active_gait_s=seconds-2,uphill_distance_m=float(delta@UPHILL),horizontal_forward_m=float(delta[0]),height_gain_m=float(delta[2]),sideways_m=float(delta[1]),max_tilt_relative_to_slope_deg=max_tilt,min_base_normal_clearance_m=min_clear,fallen=bool(fallen),nonfoot_ground_fraction=float(badcount/max(count,1)),stall_saturation_fraction=sat/max(8*count,1),accumulated_backward_motion_m=backslide,final_heading_deg=math.degrees(heading),mass_kg=float(sum(r.m.body_mass)),servo_stall_Nm=r.stall,servo_free_speed_deg_s=math.degrees(r.speed),friction=float(r.m.geom_friction[r.floor,0]),warnings=int(sum(w.number for w in r.d.warning)))
 dest=ROOT/'results/a17_slope10';dest.with_suffix('.json').write_text(json.dumps(result,indent=2))
 with dest.with_suffix('.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['t','x','y','z','uphill_position','normal_clearance','relative_tilt_deg','heading_rad','feet_in_contact']+[n+'_torque' for n in r.names]);w.writerows(rows)
 return result

def video():
 import imageio.v2 as imageio
 r=SlopeRobot();renderer=mujoco.Renderer(r.m,height=720,width=960)
 cam=mujoco.MjvCamera();cam.distance=1.25;cam.azimuth=100;cam.elevation=-15
 opt=mujoco.MjvOption();opt.geomgroup[3]=0
 with imageio.get_writer(ROOT/'results/a17_slope10.mp4',fps=30,codec='libx264',quality=7) as writer:
  for f in range(360):
   while r.d.time<(f+1)/30:r.step()
   cam.lookat[:]=r.d.qpos[:3]+[0,0,.025]
   renderer.update_scene(r.d,camera=cam,scene_option=opt);writer.append_data(renderer.render())
 renderer.close()
def view():
 from mujoco import viewer as mv
 r=SlopeRobot();state={'pause':False,'reset':False}
 def key(k):
  if k==32:state['pause']=not state['pause']
  if k in [82,114]:state['reset']=True
 with mv.launch_passive(r.m,r.d,key_callback=key) as viewer:
  viewer.cam.distance=1.;viewer.cam.azimuth=100;viewer.cam.elevation=-15;viewer.opt.geomgroup[3]=0
  start=time.perf_counter()
  while viewer.is_running():
   if state['reset']:r.reset();start=time.perf_counter();state['reset']=False
   if state['pause']:start=time.perf_counter()-r.d.time;viewer.sync();time.sleep(.02);continue
   for _ in range(5):r.step()
   viewer.cam.lookat[:]=r.d.qpos[:3]+[0,0,.025];viewer.sync()
   delay=r.d.time-(time.perf_counter()-start)
   if delay>0:time.sleep(min(delay,.02))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--test',action='store_true');ap.add_argument('--video',action='store_true');ap.add_argument('--seconds',type=float,default=30);args=ap.parse_args();build()
 if args.test:print(json.dumps(evaluate(args.seconds),indent=2))
 elif args.video:video()
 else:view()
