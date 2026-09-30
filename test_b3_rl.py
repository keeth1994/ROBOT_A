"""Check RL physics parity, repeatability, servo limits and headless model fidelity."""
import numpy as np
from stable_baselines3.common.env_checker import check_env
from rl_b3_env import StraightLineEnv

def main():
    lite=StraightLineEnv();full=StraightLineEnv(visuals=True)
    check_env(lite)
    a,_=lite.reset(seed=123);b,_=full.reset(seed=123)
    np.testing.assert_allclose(a,b,atol=1e-7)
    np.testing.assert_allclose(lite.robot.m.body_mass,full.robot.m.body_mass,atol=0)
    np.testing.assert_allclose(lite.robot.m.body_inertia,full.robot.m.body_inertia,atol=0)
    assert np.count_nonzero(lite.robot.m.geom_contype)==np.count_nonzero(full.robot.m.geom_contype)
    rng=np.random.default_rng(44)
    actions=rng.uniform(-.4,.4,(80,8))
    observations=[]
    for action in actions:
        a,ra,ta,xa,ia=lite.step(action);b,rb,tb,xb,ib=full.step(action)
        np.testing.assert_allclose(a,b,atol=1e-6);np.testing.assert_allclose(ra,rb,atol=1e-6)
        assert (ta,xa)==(tb,xb)
        assert np.all(np.abs(lite.robot.torque)<=lite.robot.limit+1e-12)
        assert not lite.robot.d.xfrc_applied.any() and not lite.robot.d.qfrc_applied.any()
        observations.append(a.copy())
    lite.reset(seed=123)
    for action,expected in zip(actions,observations):np.testing.assert_allclose(lite.step(action)[0],expected,atol=1e-7)
    print('PASS: Gym API, deterministic seeds, full/headless dynamics parity, motor limits, no external forces.')
if __name__=='__main__':main()
