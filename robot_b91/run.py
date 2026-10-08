"""B9-1 torque/speed-limited rocker gait experiment. No imposed base forces."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import mujoco
from robot_b91.paths import ROOT
from robot_b91 import servo_properties
LEGS=['FL','RL','FR','RR']
DEFAULT=dict(period=1.8,duty=.75,yaw_amplitude=.22,pitch_lift=.65,pitch_stance=0.,kp=1.6,kd=.04,heading_gain=.8,pattern='crawl')
class Robot:
 def __init__(self,params=None,ramp=False):
  self.p=DEFAULT|(params or {});self.m=mujoco.MjModel.from_xml_path(str(ROOT/'models'/('robot_b91_course.xml' if ramp else 'robot_b91.xml')));self.d=mujoco.MjData(self.m)
  cfg=json.loads((ROOT/'reference/b91_kinematics.json').read_text())
  self.names=[l+'_'+s for l in LEGS for s in ['yaw','pitch']];js=[self.m.joint(n).id for n in self.names];self.qa=self.m.jnt_qposadr[js];self.da=self.m.jnt_dofadr[js];self.ai=[self.m.actuator(n).id for n in self.names]
  ratings=np.array([servo_properties(cfg,n) for n in self.names]);self.limit=ratings[:,0];self.speed=np.radians(ratings[:,1])
  if not np.allclose(self.m.actuator_ctrlrange[self.ai],np.column_stack((-self.limit,self.limit)),rtol=1e-8,atol=1e-10):raise ValueError('Hardware torque limits differ from XML; run python robot.py build')
  self.command_period=1/cfg['servo_command_hz']
  self.m.geom_friction[:,0]=self.p.get('friction',.8);self.target=np.zeros(8);self.torque=np.zeros(8);self.reset()
 def reset(self):
  mujoco.mj_resetData(self.m,self.d);self.d.qpos[self.qa[1::2]]=self.p['pitch_stance'];mujoco.mj_forward(self.m,self.d)
  self.target=np.tile([0.,self.p['pitch_stance']],4);self.command_target=self.target.copy();self.torque.fill(0);self.next_command_time=0.;self.next_ir_time=0.;self.ir_distances_m=np.full(3,np.nan);self._sample_ir()
 def step(self,walk=True):
  t=self.d.time-2;p=self.p;self.target=np.tile([0.,p['pitch_stance']],4)
  if walk and t>0:
   offsets=[0,.5,.25,.75] if p['pattern']=='crawl' else [0,.5,.5,0]
   R=self.d.xmat[self.m.body('platform').id].reshape(3,3);heading=math.atan2(R[1,0],R[0,0])
   # Optional ideal lateral position feedback for simulation, not an implemented sensor estimator.
   error=heading+math.atan(p.get('lateral_gain',0)*self.d.qpos[1])
   correction=np.clip(p['heading_gain']*error,-.5,.5)
   for i,l in enumerate(LEGS):
    u=(t/p['period']+offsets[i])%1;duty=p['duty'];ramp=min(1,t/p['period'])
    if u<duty:yaw=p['yaw_amplitude']*(2*u/duty-1);lift=0
    else:
     v=(u-duty)/(1-duty);yaw=p['yaw_amplitude']*(1-2*v*v*(3-2*v));lift=-p['pitch_lift']*math.sin(math.pi*v)**2
    sign=1 if i<2 else -1
    self.target[2*i]=yaw*sign*ramp*(1+sign*correction);self.target[2*i+1]=p['pitch_stance']+lift*ramp
  self.step_targets(self.target)
 def step_targets(self,target):
  self.target=np.asarray(target,dtype=float).copy();p=self.p
  if self.target.shape!=(8,) or not np.isfinite(self.target).all():raise ValueError('Expected eight finite joint targets')
  if self.d.time+1e-10>=self.next_command_time:
   self.command_target=self.target.copy();self.next_command_time+=self.command_period
  v=self.d.qvel[self.da];req=p['kp']*(self.command_target-self.d.qpos[self.qa])-p['kd']*v;available=self.limit.copy();motoring=req*v>0;available[motoring]*=np.maximum(0,1-np.abs(v[motoring])/self.speed[motoring]);self.torque=np.clip(req,-available,available);self.d.ctrl[self.ai]=self.torque;mujoco.mj_step(self.m,self.d);self._sample_ir()
 def _sample_ir(self):
  if self.d.time+1e-10>=self.next_ir_time:
   # Sharp GP2Y0A41SK0F: nominal 16.5 ms period, valid between 4 and 30 cm.
   raw=np.array([self.d.sensor(n).data[0] for n in ['ir_left','ir_right','ir_ground']])
   self.ir_distances_m=np.where((raw>=.04)&(raw<=.30),raw,np.nan);self.next_ir_time+=.0165
 def tilt(self):
  return math.degrees(math.acos(np.clip(self.d.xmat[self.m.body('platform').id].reshape(3,3)[2,2],-1,1)))
def evaluate(p=None,seconds=15,walk=True,ramp=False):
 if not math.isfinite(seconds) or seconds<=2:raise ValueError('Evaluation duration must exceed the 2-second settling period')
 terrain=json.loads((ROOT/'reference/b91_course.json').read_text());surfaces=terrain['required_surfaces'];obstacles=terrain['obstacle_geoms']
 r=Robot(p,ramp);start=None;tilt=0;sat=0;n=0;bad=0;rampcontact=False
 while r.d.time<seconds:
  r.step(walk)
  if r.d.time>=2:
   if start is None:start=r.d.qpos[:3].copy()
   tilt=max(tilt,r.tilt());sat+=int(np.count_nonzero(np.abs(r.torque)>.95*r.limit));n+=1
   for c in r.d.contact:
    names=[mujoco.mj_id2name(r.m,mujoco.mjtObj.mjOBJ_GEOM,g) or '' for g in [c.geom1,c.geom2]]
    if any(x in surfaces for x in names):rampcontact=True
    if any(x in (['floor']+obstacles) for x in names) and not any('_tread_' in x for x in names):bad+=1;break
  if not np.isfinite(r.d.qpos).all():raise RuntimeError('Non-finite physics state')
 delta=r.d.qpos[:3]-start
 return dict(revision="B9-1",parameters=r.p,duration_s=seconds,walk=walk,ramp=ramp,forward_m=float(delta[0]),sideways_m=float(delta[1]),final_height_m=float(r.d.qpos[2]),max_tilt_deg=tilt,fallen=bool(tilt>45),nonfoot_ground_fraction=bad/max(n,1),stall_saturation_fraction=sat/max(8*n,1),ramp_contact=rampcontact,total_mass_kg=float(sum(r.m.body_mass)),torque_limits_Nm=r.limit.tolist(),warnings=int(sum(w.number for w in r.d.warning)))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--test',action='store_true');ap.add_argument('--hold',action='store_true');ap.add_argument('--ramp',action='store_true');ap.add_argument('--seconds',type=float,default=20);ap.add_argument('--render');ap.add_argument('--params');args=ap.parse_args()
 print('B9-1: mixed servos at 5 V; provisional joint assignment/masses; programmed gait.')
 pp=Path(args.params) if args.params else ROOT/'reference/b91_gait.json';p=json.loads(pp.read_text()) if pp.exists() else DEFAULT
 if args.test:
  result=evaluate(p,args.seconds,not args.hold,args.ramp);name='b91_'+('ramp' if args.ramp else 'hold' if args.hold else 'walk');(ROOT/'results'/f'{name}.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return
 r=Robot(p,args.ramp)
 course=json.loads((ROOT/'reference/b91_course.json').read_text())
 if args.render:
  from PIL import Image
  for _ in range(1000):r.step(False)
  renderer=mujoco.Renderer(r.m,height=900,width=1280);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.08];cam.distance=.6;cam.azimuth=135;cam.elevation=-25;opt=mujoco.MjvOption();opt.geomgroup[3]=0
  renderer.update_scene(r.d,camera=cam,scene_option=opt);Image.fromarray(renderer.render()).save(args.render);renderer.close();return
 from mujoco import viewer
 with viewer.launch_passive(r.m,r.d) as v:
  v.cam.lookat[:]=course['camera_lookat'] if args.ramp else [0,0,.08];v.cam.distance=course['camera_distance'] if args.ramp else .65;v.cam.azimuth=100 if args.ramp else 135;v.cam.elevation=-38;v.opt.geomgroup[3]=0
  while v.is_running():
   t=time.perf_counter()
   if r.d.time<args.seconds:r.step(not args.hold)
   if not args.ramp:v.cam.lookat[:2]=r.d.qpos[:2]
   v.sync();time.sleep(max(0,r.m.opt.timestep-(time.perf_counter()-t)))
if __name__=='__main__':main()
