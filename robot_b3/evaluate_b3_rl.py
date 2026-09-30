"""Evaluate a saved RL policy on new seeds, optionally a finer physics timestep."""
import argparse,json
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from robot_b3.rl_b3_env import StraightLineEnv

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',required=True);ap.add_argument('--episodes',type=int,default=10);ap.add_argument('--seed',type=int,default=2000);ap.add_argument('--fine',action='store_true');ap.add_argument('--out',required=True);args=ap.parse_args()
    torch.set_num_threads(1);policy=PPO.load(args.model,device='cpu');env=StraightLineEnv();rows=[]
    if args.fine:
        env.robot.m.opt.timestep/=2;env.frame_skip*=2
    for seed in range(args.seed,args.seed+args.episodes):
        obs,_=env.reset(seed=seed);done=False;total=0;power=0;max_joint_speed=0
        while not done:
            action,_=policy.predict(obs,deterministic=True);obs,reward,term,trunc,info=env.step(action);done=term or trunc;total+=reward
            max_joint_speed=max(max_joint_speed,float(np.max(np.abs(env.robot.d.qvel[env.robot.da]))))
        rows.append(dict(seed=seed,reward=total,max_joint_speed_rad_s=max_joint_speed,**info))
    report=dict(model=args.model,physics_timestep_s=float(env.robot.m.opt.timestep),episodes=rows,mean_forward_m=float(np.mean([x['forward_m'] for x in rows])),success_rate=float(np.mean([x['success'] for x in rows])),fall_rate=float(np.mean([x['fallen'] for x in rows])))
    Path(args.out).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));env.close()
if __name__=='__main__':main()
