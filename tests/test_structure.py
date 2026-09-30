"""Repository entry points and Windows RL worker import checks."""
from functools import partial
import json
import subprocess
import sys
from pathlib import Path
from robot_b3.paths import ROOT

def main():
    import robot
    for command in robot.COMMANDS:
        run = subprocess.run([sys.executable, str(ROOT/'robot.py'), command, '--help'],
                             cwd=ROOT.parent, capture_output=True, text=True, timeout=30)
        assert run.returncode == 0, (command, run.stderr)
    for task in json.loads((ROOT/'.vscode/tasks.json').read_text())['tasks']:
        args=task.get('args',[])
        if args and args[0].endswith('robot.py'):
            assert args[1] in robot.COMMANDS or args[1]=='check'
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from robot_b3.train_b3_curriculum import make_env
    env=SubprocVecEnv([partial(make_env,0,True),partial(make_env,3,True)],start_method='spawn')
    try:
        obs=env.reset();assert obs.shape==(2,37)
        import numpy as np
        assert env.step(np.zeros((2,8)))[0].shape==(2,37)
    finally:
        env.close()
    print('PASS: all command help screens, tasks, outside-repo launch and spawned RL workers.')

if __name__=='__main__':main()
