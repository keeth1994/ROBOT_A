"""Flat-ground RL: learned joint targets, real simulated motor limits, no gait/path."""
import math
import xml.etree.ElementTree as ET
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from run_robot_b3 import Robot, ROOT

class StraightLineEnv(gym.Env):
    metadata = {'render_modes': []}
    def __init__(self, seconds=10., visuals=False, lateral_weight=4., heading_weight=.3):
        # Headless training omits only non-colliding visual CAD meshes, not physics.
        self.lateral_weight=lateral_weight;self.heading_weight=heading_weight
        self.robot = Robot.__new__(Robot)
        r = self.robot
        import json
        cfg = json.loads((ROOT/'reference/b3_kinematics.json').read_text())
        tree = ET.parse(ROOT/'models/robot_b3.xml')
        if not visuals:
            for parent in tree.iter():
                for child in list(parent):
                    if child.tag == 'geom' and child.get('contype') == '0' and child.get('conaffinity') == '0' and child.get('mesh', '').startswith('cad_'): parent.remove(child)
            asset = tree.find('asset')
            for mesh in list(asset):
                if mesh.tag == 'mesh' and mesh.get('file'): asset.remove(mesh)
        self.configure_terrain(tree)
        tree.find('compiler').set('meshdir', str(ROOT/'models/robot_meshes_b3'))
        r.m = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
        r.d = mujoco.MjData(r.m)
        r.p = dict(kp=1.6,kd=.04,pitch_stance=0.)
        r.limit=np.tile([cfg['servo_stall_Nm']*.85/1.5,cfg['servo_stall_Nm']],4)
        r.speed=np.tile(np.radians([cfg['servo_free_speed_deg_s']*1.5,cfg['servo_free_speed_deg_s']]),4)
        r.names=[leg+'_'+joint for leg in ['FL','RL','FR','RR'] for joint in ['yaw','pitch']]
        js=[r.m.joint(n).id for n in r.names]
        r.qa=r.m.jnt_qposadr[js];r.da=r.m.jnt_dofadr[js];r.ai=[r.m.actuator(n).id for n in r.names]
        r.target=np.zeros(8);r.torque=np.zeros(8)
        self.joint_range=r.m.jnt_range[js].copy()
        self.action_space=spaces.Box(-1.,1.,(8,),dtype=np.float32)
        self.observation_space=spaces.Box(-np.inf,np.inf,(37,),dtype=np.float32)
        self.frame_skip=10;self.dt=r.m.opt.timestep*self.frame_skip
        self.horizon=round(seconds/self.dt)
        self.platform=r.m.body('platform').id
        self.floor=r.m.geom('floor').id
        self.ground_geoms={self.floor}|{i for i in range(r.m.ngeom) if (r.m.geom(i).name or '').startswith(('lesson_','ramp_support_'))}
        self.treads={i for i in range(r.m.ngeom) if '_tread_' in (r.m.geom(i).name or '')}
        self.previous=np.zeros(8);self.steps=0
    def configure_terrain(self,tree):
        pass
    def _obs(self):
        r=self.robot;R=r.d.xmat[self.platform].reshape(3,3)
        heading=math.atan2(R[1,0],R[0,0])
        return np.concatenate([r.d.qpos[r.qa]/(math.pi/4),r.d.qvel[r.da]/r.speed,
            r.d.qvel[:3],r.d.qvel[3:6],R[2,:],
            [math.sin(heading),math.cos(heading),r.d.qpos[1],r.d.qpos[2]],self.previous]).astype(np.float32)
    def reset(self,seed=None,options=None):
        super().reset(seed=seed);r=self.robot;r.reset()
        r.d.qpos[r.qa]=self.np_random.uniform(-.015,.015,8)
        mujoco.mj_forward(r.m,r.d)
        for _ in range(round(.3/r.m.opt.timestep)):r.step_targets(np.zeros(8))
        self.start=r.d.qpos[:3].copy();self.previous=np.zeros(8);self.steps=0
        self.max_lateral=0.;self.max_tilt=0.;self.max_heading=0.;self.bad_steps=0
        return self._obs(),{}
    def step(self,action):
        r=self.robot;action=np.clip(np.asarray(action,dtype=float),-1,1)
        target=self.joint_range[:,0]+(action+1)*.5*np.diff(self.joint_range,axis=1).ravel()
        before=r.d.qpos[:3].copy();bad=False;power=0.
        for _ in range(self.frame_skip):
            r.step_targets(target)
            power+=float(np.sum(np.abs(r.torque*r.d.qvel[r.da])))
            for c in r.d.contact:
                if c.geom1 in self.ground_geoms or c.geom2 in self.ground_geoms:
                    other=c.geom2 if c.geom1 in self.ground_geoms else c.geom1
                    if other not in self.treads:bad=True
        self.steps+=1;R=r.d.xmat[self.platform].reshape(3,3)
        heading=math.atan2(R[1,0],R[0,0]);tilt=r.tilt();y=float(r.d.qpos[1]-self.start[1])
        vx=float((r.d.qpos[0]-before[0])/self.dt);vy=float((r.d.qpos[1]-before[1])/self.dt)
        finite=bool(np.isfinite(r.d.qpos).all() and np.isfinite(r.d.qvel).all())
        fallen=tilt>45 or r.d.qpos[2]<.016
        terminated=fallen or not finite or abs(y)>.25 or abs(heading)>math.pi/2 or bool(sum(w.number for w in r.d.warning))
        # Progress must come from feet. Penalize turning, drift, energy and command chatter.
        reward=10*np.clip(vx,-.2,.2)-2*abs(vy)-self.lateral_weight*abs(y)-self.heading_weight*abs(heading)-.4*(tilt/45)**2
        reward-=.003*np.mean((action-self.previous)**2)+.003*power/self.frame_skip+.5*bad
        if terminated:reward-=5
        self.previous=action.copy();self.max_lateral=max(self.max_lateral,abs(y));self.max_tilt=max(self.max_tilt,tilt);self.max_heading=max(self.max_heading,abs(heading));self.bad_steps+=int(bad)
        info=dict(forward_m=float(r.d.qpos[0]-self.start[0]),sideways_m=y,
            elapsed_s=self.steps*self.dt,max_lateral_m=self.max_lateral,max_tilt_deg=self.max_tilt,
            max_heading_deg=math.degrees(self.max_heading),fallen=bool(fallen),
            nonfoot_fraction=self.bad_steps/self.steps,warnings=int(sum(w.number for w in r.d.warning)))
        info['success']=bool(info['forward_m']>=.30 and self.max_lateral<=.04 and self.max_heading<=math.radians(15) and not terminated and self.bad_steps==0)
        return self._obs(),float(reward),bool(terminated),self.steps>=self.horizon,info
