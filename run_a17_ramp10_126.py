"""A17: flat approach -> 10 degree ramp, 1.26 Nm per servo.
Same mass and no-load speed as A17. A hypothetical torque upgrade, not a specific motor.
"""
from run_robot_a17 import Robot,ROOT
import argparse,json,csv,math,time
import numpy as np,mujoco
import xml.etree.ElementTree as ET
START=.65
END=6.65
WIDTH=3.
ANGLE=math.radians(10)
MODEL=ROOT/'models/robot_a17_ramp10_126.xml'
def build():
 tree=ET.parse(ROOT/'models/robot_a17.xml');root=tree.getroot();root.set('model','A17 - flat to 10 degree ramp - 1.26 Nm')
 height=(END-START)*math.tan(ANGLE)
 vertices=[[x,y,z] for x,top in [(START,0),(END,height)] for y in [-WIDTH/2,WIDTH/2] for z in [-.1,top]]
 ET.SubElement(root.find('asset'),'mesh',name='ramp_wedge',vertex=' '.join(str(v) for row in vertices for v in row))
 ET.SubElement(root.find('worldbody'),'geom',name='ramp',type='mesh',mesh='ramp_wedge',mass='0',rgba='.52 .48 .30 1',contype='1',conaffinity='2',friction='.8 .002 .0001',solref='.006 1')
 for a in root.find('actuator'):a.set('ctrlrange','-1.26 1.26')
 ET.indent(tree);tree.write(MODEL,encoding='utf-8',xml_declaration=True)
 params=json.loads((ROOT/'reference/a17_gait.json').read_text());params['servo_stall_Nm']=1.26
 (ROOT/'reference/a17_ramp10_126_gait.json').write_text(json.dumps(params,indent=2))
class RampRobot(Robot):
 def __init__(self):
  super().__init__(json.loads((ROOT/'reference/a17_ramp10_126_gait.json').read_text()),MODEL)
  self.ramp=self.m.geom('ramp').id
 def ground_height(self,x,y):return (x-START)*math.tan(ANGLE) if START<x<END and abs(y)<WIDTH/2 else 0.
 def contact_state(self):
  rampfeet=set();feet=set();bad=False
  for c in self.d.contact:
   terrains={self.floor,self.ramp}
   if c.geom1 in terrains:g=c.geom2;t=c.geom1
   elif c.geom2 in terrains:g=c.geom1;t=c.geom2
   else:continue
   if g in self.feet:
    feet.add(g)
    if t==self.ramp:rampfeet.add(g)
   elif c.dist<0:bad=True
  return len(feet),len(rampfeet),bad
 def stats(self):
  up=self.d.xmat[self.m.body('platform').id].reshape(3,3)[:,2]
  x,y,z=self.d.qpos[:3];normal=np.array([-math.sin(ANGLE),0,math.cos(ANGLE)]) if x>START else np.array([0,0,1])
  tilt=math.degrees(math.acos(np.clip(up@normal,-1,1)))
  clearance=z-self.ground_height(x,y)
  return tilt,clearance

def evaluate(seconds):
 r=RampRobot();rows=[];start=None;first_ramp=None;all_on=None;fall=False;maxtilt=0.;minclear=1.;badcount=0;count=0;maxforce=0.;maxctrl=0.
 for i in range(round(seconds/r.m.opt.timestep)):
  r.step();feet,rampfeet,bad=r.contact_state();tilt,clear=r.stats()
  maxforce=max(maxforce,float(np.max(abs(r.d.actuator_force))));maxctrl=max(maxctrl,float(np.max(abs(r.d.ctrl))))
  if rampfeet and first_ramp is None:first_ramp=r.d.time
  sites=np.array([r.d.site_xpos[r.m.site(l+'_foot').id] for l in ['FL','RL','FR','RR']])
  if np.all(sites[:,0]>START) and all_on is None:all_on=r.d.time
  if r.d.time>=2:
   if start is None:start=r.d.qpos[:3].copy()
   maxtilt=max(maxtilt,tilt);minclear=min(minclear,clear);fall|=tilt>45 or clear<.055;badcount+=bad;count+=1
  if i%10==0:rows.append([r.d.time,*r.d.qpos[:3],tilt,clear,feet,rampfeet,*r.d.actuator_force])
 result=dict(duration_s=seconds,servo_stall_Nm=r.stall,servo_free_speed_deg_s=math.degrees(r.speed),mass_kg=float(sum(r.m.body_mass)),ramp_start_x_m=START,ramp_angle_deg=10,first_ramp_contact_s=first_ramp,all_feet_past_ramp_start_s=all_on,forward_m=float(r.d.qpos[0]-start[0]),sideways_m=float(r.d.qpos[1]-start[1]),final_position_m=r.d.qpos[:3].tolist(),max_relative_tilt_deg=maxtilt,min_vertical_clearance_m=minclear,fallen=bool(fall),nonfoot_ground_fraction=badcount/max(count,1),max_actual_actuator_torque_Nm=maxforce,max_commanded_torque_Nm=maxctrl,warnings=int(sum(w.number for w in r.d.warning)))
 assert abs(r.stall-1.26)<1e-10 and np.allclose(r.m.actuator_ctrlrange,[-1.26,1.26])
 assert maxforce<=1.2600001 and maxctrl<=1.2600001
 dest=ROOT/'results/a17_ramp10_126';dest.with_suffix('.json').write_text(json.dumps(result,indent=2))
 with dest.with_suffix('.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['time','x','y','z','relative_tilt_deg','clearance','contact_feet','ramp_contact_feet']+[n+'_torque' for n in r.names]);w.writerows(rows)
 return result

def show(record=False):
 r=RampRobot()
 if record:
  import imageio.v2 as imageio
  renderer=mujoco.Renderer(r.m,height=720,width=960);cam=mujoco.MjvCamera();cam.distance=1.6;cam.azimuth=90;cam.elevation=-15
  opt=mujoco.MjvOption();opt.geomgroup[3]=0
  # 45 seconds of actual simulation at 2x playback.
  with imageio.get_writer(ROOT/'results/a17_ramp10_126.mp4',fps=30,codec='libx264',quality=7) as writer:
   for f in range(675):
    while r.d.time<(f+1)*2/30:r.step()
    cam.lookat[:]=r.d.qpos[:3]+[.15,0,.02];renderer.update_scene(r.d,camera=cam,scene_option=opt);writer.append_data(renderer.render())
  renderer.close();return
 from mujoco import viewer as mv
 state={'pause':False,'reset':False}
 def key(k):
  if k==32:state['pause']=not state['pause']
  if k in [82,114]:state['reset']=True
 with mv.launch_passive(r.m,r.d,key_callback=key) as viewer:
  viewer.cam.distance=1.5;viewer.cam.azimuth=90;viewer.cam.elevation=-15;viewer.opt.geomgroup[3]=0;start=time.perf_counter()
  while viewer.is_running():
   if state['reset']:r.reset();start=time.perf_counter();state['reset']=False
   if state['pause']:start=time.perf_counter()-r.d.time;viewer.sync();time.sleep(.02);continue
   for _ in range(5):r.step()
   viewer.cam.lookat[:]=r.d.qpos[:3]+[.15,0,.02];viewer.sync()
   delay=r.d.time-(time.perf_counter()-start)
   if delay>0:time.sleep(min(delay,.02))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--test',action='store_true');ap.add_argument('--video',action='store_true');ap.add_argument('--seconds',type=float,default=60);args=ap.parse_args();build()
 if args.test:print(json.dumps(evaluate(args.seconds),indent=2))
 else:show(args.video)
