"""Train/evaluate a PPO joint-target policy for B3 on flat ground."""
import argparse,json,time,hashlib
from functools import partial
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from robot_b3.rl_b3_env import StraightLineEnv
from robot_b3.run_robot_b3 import ROOT

def make_env(env_kwargs=None):
    return Monitor(StraightLineEnv(**(env_kwargs or {})))

def evaluate(model,episodes=3,start_seed=1000,env_kwargs=None):
    env=StraightLineEnv(**(env_kwargs or {}));rows=[]
    for seed in range(start_seed,start_seed+episodes):
        obs,_=env.reset(seed=seed);done=False;total=0
        while not done:
            action=np.zeros(8,dtype=np.float32) if model is None else model.predict(obs,deterministic=True)[0]
            obs,reward,term,trunc,info=env.step(action);total+=reward;done=term or trunc
        rows.append(dict(seed=seed,reward=total,**info))
    env.close()
    return dict(episodes=rows,mean_forward_m=float(np.mean([x['forward_m'] for x in rows])),
        mean_reward=float(np.mean([x['reward'] for x in rows])),success_rate=float(np.mean([x['success'] for x in rows])))

class Progress(BaseCallback):
    def __init__(self,out,budget_seconds,env_kwargs=None):
        super().__init__();self.env_kwargs=env_kwargs;self.out=out;self.budget_seconds=budget_seconds;self.started=time.perf_counter();self.next_eval=32768;self.best=-np.inf;self.history=[]
    def _on_step(self):
        return time.perf_counter()-self.started<self.budget_seconds
    def _on_rollout_end(self):
        if self.num_timesteps>=self.next_eval:
            result=evaluate(self.model,2,env_kwargs=self.env_kwargs);elapsed=time.perf_counter()-self.started
            row=dict(timesteps=self.num_timesteps,wall_seconds=elapsed,**result);self.history.append(row)
            (self.out/'learning_curve.json').write_text(json.dumps(self.history,indent=2))
            self.model.save(self.out/'latest')
            if result['mean_reward']>self.best:self.best=result['mean_reward'];self.model.save(self.out/'best')
            print(f"EVAL steps={self.num_timesteps} wall={elapsed:.1f}s forward={result['mean_forward_m']:.3f}m success={result['success_rate']:.0%}",flush=True)
            self.next_eval=self.num_timesteps+32768

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--steps',type=int,default=262144);ap.add_argument('--envs',type=int,default=8);ap.add_argument('--minutes',type=float,default=10);ap.add_argument('--seed',type=int,default=42);ap.add_argument('--resume');ap.add_argument('--strict',action='store_true',help='Stronger lateral and heading penalties');ap.add_argument('--out',default='results/rl_flat');args=ap.parse_args()
    torch.set_num_threads(1);out=ROOT/args.out;out.mkdir(parents=True,exist_ok=True)
    env_kwargs=dict(lateral_weight=15.,heading_weight=1.5) if args.strict else {}
    started=time.perf_counter();baseline=evaluate(None,env_kwargs=env_kwargs);(out/'hold_baseline.json').write_text(json.dumps(baseline,indent=2))
    print('Standing baseline:',baseline['mean_forward_m'],flush=True)
    env=SubprocVecEnv([partial(make_env,env_kwargs)]*args.envs,start_method='spawn') if args.envs>1 else DummyVecEnv([partial(make_env,env_kwargs)])
    if args.resume:model=PPO.load(args.resume,env=env,device='cpu')
    else:model=PPO('MlpPolicy',env,learning_rate=3e-4,n_steps=512,batch_size=256,n_epochs=5,gamma=.99,gae_lambda=.95,ent_coef=.005,policy_kwargs=dict(net_arch=[64,64],log_std_init=-.8),seed=args.seed,device='cpu',verbose=1)
    model.set_logger(configure(str(out),['stdout','csv']))
    initial=evaluate(model,env_kwargs=env_kwargs);(out/'untrained_baseline.json').write_text(json.dumps(initial,indent=2))
    cb=Progress(out,args.minutes*60,env_kwargs)
    try:
        model.learn(total_timesteps=args.steps,callback=cb,reset_num_timesteps=not bool(args.resume))
        model.save(out/'latest')
        final=evaluate(model,5,env_kwargs=env_kwargs)
        if final['mean_reward']>cb.best or not (out/'best.zip').exists():model.save(out/'best')
        best=evaluate(PPO.load(out/'best',device='cpu'),5,env_kwargs=env_kwargs)
        metadata=dict(reward_settings=env_kwargs or dict(lateral_weight=4.,heading_weight=.3),seed=args.seed,timesteps=model.num_timesteps,wall_seconds=time.perf_counter()-started,
            training_wall_seconds=time.perf_counter()-cb.started,environments=args.envs,final=final,best=best,
            model_sha256=hashlib.sha256((ROOT/'models/robot_b3.xml').read_bytes()).hexdigest(),
            cad_revision=json.loads((ROOT/'reference/b3_kinematics.json').read_text())['cad_revision'],
            policy='PPO direct absolute joint targets; no gait generator',observations='Ideal simulator state, not hardware-ready sensor estimates',
            motor_limits_Nm=[.152058749477706,.26833896966654],success='At least 0.30 m forward in 10 s, max lateral 0.04 m, heading 15 degrees, no fall/nonfoot contact/warnings')
        (out/'summary.json').write_text(json.dumps(metadata,indent=2))
        (ROOT/'reference/b3_rl_policy.json').write_text(json.dumps(dict(model=str((out/'best.zip').relative_to(ROOT)),summary=str((out/'summary.json').relative_to(ROOT)),model_sha256=metadata['model_sha256']),indent=2))
        print(json.dumps(metadata,indent=2))
    finally:env.close()
if __name__=='__main__':main()
