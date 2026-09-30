"""Curriculum: narrow straight-line corridor, then a physical 3/5-degree ramp."""
import math
import numpy as np
from rl_b3_env import StraightLineEnv
from course_b3 import build_course

OBS_SCALE=np.ones(37,dtype=np.float32)
OBS_SCALE[16:19]=5;OBS_SCALE[22:24]=3;OBS_SCALE[25]=3;OBS_SCALE[27]=20;OBS_SCALE[28]=10

class CurriculumEnv(StraightLineEnv):
    def __init__(self,slope=0.,visuals=False,seconds=None,normalized=False,ramp_width=.3):
        self.ramp_width=float(ramp_width)
        self.normalized=normalized
        self.slope=float(slope)
        super().__init__(seconds=seconds or (25 if slope else 10),visuals=visuals)
    def _obs(self):
        obs=super()._obs()
        return obs*OBS_SCALE if self.normalized else obs
    def configure_terrain(self,tree):
        if not self.slope:return
        height=.4*math.tan(math.radians(self.slope))
        self.course=build_course(tree.find('asset'),tree.find('worldbody'),dict(
            description='RL gentle slope lesson',width_m=self.ramp_width,panel_thickness_m=.006,
            origin_xy_m=[.3,0],yaw_deg=0,profile_xz_m=[[0,0],[.4,height],[.6,height],[1.,0]],
            segment_names=['lesson_up','lesson_top','lesson_down'],corridor=dict(enabled=False)))
    def reset(self,seed=None,options=None):
        obs,info=super().reset(seed=seed,options=options)
        self.touched=set();self.edge=False;self.total_bad=0;self.max_path_error=0.
        return obs,info
    def step(self,action):
        r=self.robot;before=r.d.qpos[:3].copy();prev=self.previous.copy()
        obs,_,term,trunc,info=super().step(action)
        vx=(r.d.qpos[0]-before[0])/self.dt;vy=(r.d.qpos[1]-before[1])/self.dt
        y=float(r.d.qpos[1]);heading=math.atan2(r.d.xmat[self.platform].reshape(3,3)[1,0],r.d.xmat[self.platform].reshape(3,3)[0,0])
        self.max_path_error=max(self.max_path_error,abs(y))
        # Saturating progress removes incentive to trade steering accuracy for speed.
        alignment=math.exp(-(y/.035)**2-(heading/.20)**2)
        reward=15*np.clip(vx,-.10,.08)*alignment-10*abs(y)-abs(heading)-2*abs(vy)
        reward-=.003*np.mean((np.asarray(action)-prev)**2)+.003*np.sum(np.abs(r.torque*r.d.qvel[r.da]))
        reward-=.2*(info['max_tilt_deg']/45)**2
        bad_count=round(info['nonfoot_fraction']*self.steps)
        reward-=3*(bad_count-self.total_bad);self.total_bad=bad_count
        for c in r.d.contact:
            for gid in (c.geom1,c.geom2):
                name=r.m.geom(gid).name or ''
                if name.startswith('lesson_'):self.touched.add(name)
            if self.slope and (c.geom1 in self.ground_geoms or c.geom2 in self.ground_geoms):
                if .3<c.pos[0]<1.3 and abs(c.pos[1])>self.ramp_width/2:self.edge=True
        # Narrow training corridor is a failure condition, not a physical guide rail.
        term=term or abs(y)>.10 or abs(heading)>math.radians(40) or self.edge
        if term:reward-=10
        valid=not term and info['nonfoot_fraction']==0 and self.max_path_error<=.03 and info['max_heading_deg']<=10
        complete=bool(r.d.qpos[0]>1.45 and {'lesson_up','lesson_top','lesson_down'}<=self.touched) if self.slope else info['forward_m']>=.5
        info.update(success=bool(valid and complete and (trunc or self.slope)),max_lateral_m=self.max_path_error,
            slope_deg=self.slope,touched=sorted(self.touched),edge_violation=self.edge,course_complete=complete)
        if self.slope and complete:
            trunc=True
            if valid:reward+=10
        return obs,float(reward),bool(term),bool(trunc),info
