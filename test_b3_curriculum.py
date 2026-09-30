"""Physical lesson ramp geometry and curriculum regression checks."""
import math
import numpy as np
import mujoco
from rl_b3_curriculum import CurriculumEnv

def main():
    for angle in [3,5]:
        env=CurriculumEnv(slope=angle,normalized=True);full=CurriculumEnv(slope=angle,visuals=True,normalized=True)
        obs,_=env.reset(seed=13);other,_=full.reset(seed=13);np.testing.assert_allclose(obs,other,atol=1e-7)
        r=env.robot;h=.4*math.tan(math.radians(angle));groups=np.array([1,0,0,0,0,0],dtype=np.uint8)
        for x,z in [(.25,0),(.5,h/2),(.8,h),(1.1,h/2),(1.4,0)]:
            gid=np.array([-1],dtype=np.int32);distance=mujoco.mj_ray(r.m,r.d,np.array([x,0,1.]),np.array([0.,0.,-1]),groups,1,-1,gid)
            assert abs(1-distance-z)<1e-5,(angle,x,z,1-distance)
        for x in [.5,.8,1.1]:
            gid=np.array([-1],dtype=np.int32);distance=mujoco.mj_ray(r.m,r.d,np.array([x,.16,1.]),np.array([0.,0.,-1]),groups,1,-1,gid)
            assert abs(1-distance)<1e-5,'Outside ramp should be floor'
        rng=np.random.default_rng(4)
        for _ in range(40):
            action=rng.uniform(-.3,.3,8);a=env.step(action);b=full.step(action)
            np.testing.assert_allclose(a[0],b[0],atol=1e-6);np.testing.assert_allclose(a[1],b[1],atol=1e-6)
        assert not env.touched,'Holding at the approach cannot count as ramp completion'
        assert r.m.geom('ramp_support_1_1').id in env.ground_geoms
    flat=CurriculumEnv();flat.reset(seed=1)
    for _ in range(flat.horizon):obs,reward,term,trunc,info=flat.step(np.zeros(8))
    assert trunc and not info['success'],'Standing still is not success'
    print('PASS: slope angles/heights, 30cm width, physical floor outside edges, full/headless parity, contact accounting, no false completion.')
if __name__=='__main__':main()
