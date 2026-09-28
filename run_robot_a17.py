"""Torque-limited A17 forward gait experiment. +X is forward; no base forces or pose animation.
Run --test for measured physics; --viewer for the same controller in real time.
"""
import argparse,json,math,time,csv
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
LEGS=['FL','RL','FR','RR']
DEFAULT=dict(period=2.4,duty=.75,yaw_amplitude=.10,pitch_lift=.40,pitch_stance=0.,pattern='crawl',kp=3.0,kd=.09,heading_gain=.8)
class Robot:
 def __init__(self,params=None,model_path=None):
  self.m=mujoco.MjModel.from_xml_path(str(model_path or ROOT/'models/robot_a17.xml'));self.d=mujoco.MjData(self.m)
  self.p=DEFAULT| (params or {})
  self.manifest=json.loads((ROOT/'reference/robot_a17_manifest.json').read_text());cfg=self.manifest['assumptions']
  self.stall=self.p.get('servo_stall_Nm',cfg['servo_stall_torque_Nm'])*self.p.get('torque_scale',1.);self.speed=math.radians(cfg['servo_free_speed_deg_s'])
  self.m.body_mass[:]*=self.p.get('mass_scale',1.);self.m.body_inertia[:]*=self.p.get('mass_scale',1.)
  self.m.geom_friction[:,0]=self.p.get('friction',cfg['ground_friction'])
  self.names=[l+'_'+p for l in LEGS for p in ['yaw','pitch']]
  self.jids=np.array([self.m.joint(n).id for n in self.names]);self.qa=self.m.jnt_qposadr[self.jids];self.da=self.m.jnt_dofadr[self.jids]
  self.aids=np.array([self.m.actuator(n).id for n in self.names]);self.m.actuator_ctrlrange[self.aids]=[-self.stall,self.stall];self.target=np.zeros(8);self.torque=np.zeros(8)
  self.floor=self.m.geom('floor').id;self.feet=[self.m.geom(l+'_foot_contact').id for l in LEGS]
  self.reset()
 def reset(self):
  mujoco.mj_resetData(self.m,self.d)
  self.d.qpos[self.qa[1::2]]=self.p['pitch_stance']
  self.d.qpos[2]-=.158*(1-math.cos(self.p['pitch_stance']))
  mujoco.mj_forward(self.m,self.d)
 def command(self,t):
  p=self.p
  if t<0:return np.tile([0.,p['pitch_stance']],4)
  offsets=[0,.5,.25,.75] if p['pattern']=='crawl' else [0,.5,.5,0]
  q=[];ramp=min(1,t/p['period'])
  for i,leg in enumerate(LEGS):
   u=(t/p['period']+offsets[i])%1;duty=p['duty']
   if u<duty:
    yaw=-p['yaw_amplitude']+2*p['yaw_amplitude']*u/duty;pitch=p['pitch_stance']
   else:
    v=(u-duty)/(1-duty);smooth=v*v*(3-2*v)
    yaw=p['yaw_amplitude']*(1-2*smooth);pitch=p['pitch_stance']+p['pitch_lift']*math.sin(math.pi*v)**2
   q.extend([yaw*(1 if leg in ['FL','RL'] else -1)*ramp,p['pitch_stance']+(pitch-p['pitch_stance'])*ramp])
  return np.array(q)
 def step(self,walk=True):
  self.target=self.command(self.d.time-2) if walk else np.zeros(8)
  if walk and self.d.time>2:
   R=self.d.xmat[self.m.body('platform').id].reshape(3,3)
   heading=math.atan2(R[1,0],R[0,0])
   correction=np.clip(self.p.get('heading_gain',.8)*heading,-.5,.5)
   self.target[0::2]*=1+correction*np.array([1,1,-1,-1])
  v=self.d.qvel[self.da];req=self.p['kp']*(self.target-self.d.qpos[self.qa])-self.p['kd']*v
  available=np.full(8,self.stall);motoring=req*v>0;available[motoring]*=np.maximum(0,1-np.abs(v[motoring])/self.speed)
  self.torque=np.clip(req,-available,available);self.d.ctrl[self.aids]=self.torque
  mujoco.mj_step(self.m,self.d)
 def metrics(self):
  R=self.d.xmat[self.m.body('platform').id].reshape(3,3)
  tilt=math.degrees(math.acos(np.clip(R[2,2],-1,1)));heading=math.atan2(R[1,0],R[0,0])
  bad=False;feet=set()
  for c in self.d.contact:
   if self.floor in (c.geom1,c.geom2):
    g=c.geom2 if c.geom1==self.floor else c.geom1
    if g in self.feet:feet.add(g)
    elif c.dist<0:bad=True
  return tilt,heading,bad,len(feet)
def evaluate(params=None,seconds=14,walk=True,log=None):
 r=Robot(params);rows=[];x0=None;tiltmax=0;badsteps=0;sat=0;steps=0;maxspeed=0;fallen=False
 while r.d.time<seconds:
  r.step(walk);tilt,heading,bad,nfeet=r.metrics()
  if r.d.time>=2:
   if x0 is None:x0=r.d.qpos[:3].copy()
   tiltmax=max(tiltmax,tilt);badsteps+=int(bad);sat+=np.count_nonzero(np.abs(r.torque)>.95*r.stall);steps+=1
   maxspeed=max(maxspeed,float(np.max(np.abs(r.d.qvel[r.da]))))
   fallen|=tilt>45 or r.d.qpos[2]<.055
  if log and int(r.d.time/r.m.opt.timestep)%10==0:rows.append([r.d.time,*r.d.qpos[:3],tilt,heading,nfeet,*r.target,*r.d.qpos[r.qa],*r.torque])
  if not np.isfinite(r.d.qpos).all():raise RuntimeError('Nonfinite state')
 dx=r.d.qpos[:3]-x0
 result=dict(parameters=r.p,duration_s=seconds,walking_duration_s=seconds-2,forward_m=float(dx[0]),sideways_m=float(dx[1]),final_base_height_m=float(r.d.qpos[2]),max_tilt_deg=tiltmax,final_heading_deg=math.degrees(heading),fallen=bool(fallen),nonfoot_floor_fraction=badsteps/max(steps,1),stall_saturation_fraction=sat/max(8*steps,1),max_joint_speed_deg_s=math.degrees(maxspeed),mass_kg=float(sum(r.m.body_mass)),stall_torque_Nm=r.stall,warning_count=int(sum(w.number for w in r.d.warning)))
 if log:
  log=Path(log);log.parent.mkdir(exist_ok=True,parents=True)
  with log.with_suffix('.csv').open('w',newline='') as f:
   w=csv.writer(f);w.writerow(['time','x','y','z','tilt_deg','heading_rad','feet_in_contact']+[n+'_target' for n in r.names]+[n+'_actual' for n in r.names]+[n+'_torque' for n in r.names]);w.writerows(rows)
  log.with_suffix('.json').write_text(json.dumps(result,indent=2))
 return result

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--test',action='store_true');ap.add_argument('--viewer',action='store_true');ap.add_argument('--hold',action='store_true');ap.add_argument('--seconds',type=float,default=20);ap.add_argument('--params');ap.add_argument('--render');args=ap.parse_args()
 pp=Path(args.params) if args.params else ROOT/'reference/a17_gait.json';p=json.loads(pp.read_text()) if pp.exists() else DEFAULT
 if args.test:
  print(json.dumps(evaluate(p,args.seconds,not args.hold,ROOT/'results'/('a17_hold' if args.hold else 'a17_walk')),indent=2));return
 r=Robot(p)
 if args.render:
  from PIL import Image
  for _ in range(1000):r.step(False)
  renderer=mujoco.Renderer(r.m,height=900,width=1280);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.13];cam.distance=.9;cam.azimuth=135;cam.elevation=-25
  renderer.update_scene(r.d,camera=cam);Image.fromarray(renderer.render()).save(args.render);renderer.close();return
 from mujoco import viewer as mjviewer
 state={'paused':False,'reset':False}
 def key(k):
  if k==32:state['paused']=not state['paused']
  if k in [82,114]:state['reset']=True
 with mjviewer.launch_passive(r.m,r.d,key_callback=key) as viewer:
  viewer.cam.distance=.95;viewer.cam.azimuth=135;viewer.cam.elevation=-25
  viewer.opt.geomgroup[1]=1;viewer.opt.geomgroup[3]=0
  start=time.perf_counter()
  while viewer.is_running():
   if state['reset']:r.reset();start=time.perf_counter();state['reset']=False
   if state['paused']:
    start=time.perf_counter()-r.d.time;viewer.sync();time.sleep(.02);continue
   for _ in range(5):r.step(not args.hold)
   viewer.cam.lookat[:]=r.d.qpos[:3]+[0,0,.03];viewer.sync()
   wait=r.d.time-(time.perf_counter()-start)
   if wait>0:time.sleep(min(wait,.02))
if __name__=='__main__':main()
