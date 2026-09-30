"""Train a curriculum stage; save checkpoints by task success before reward."""
import argparse,json,time,hashlib
from rl_b3_curriculum import OBS_SCALE
from functools import partial
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from rl_b3_curriculum import CurriculumEnv
from run_robot_b3 import ROOT

def make_env(slope,normalized=False,ramp_width=.3):return Monitor(CurriculumEnv(slope=slope,normalized=normalized,ramp_width=ramp_width))
def evaluate(policy,slope,seeds,normalized=False):
    env=CurriculumEnv(slope=slope,normalized=normalized);rows=[]
    for seed in seeds:
        obs,_=env.reset(seed=seed);done=False;total=0
        while not done:
            act,_=policy.predict(obs,deterministic=True);obs,reward,term,trunc,info=env.step(act);total+=reward;done=term or trunc
        rows.append(dict(seed=seed,reward=total,**info))
    env.close()
    return dict(completion_rate=float(np.mean([x['course_complete'] and not x['edge_violation'] and not x['fallen'] for x in rows])),episodes=rows,success_rate=float(np.mean([x['success'] for x in rows])),
        mean_forward_m=float(np.mean([x['forward_m'] for x in rows])),mean_reward=float(np.mean([x['reward'] for x in rows])),
        mean_max_lateral_m=float(np.mean([x['max_lateral_m'] for x in rows])))
def score(result):return (result['success_rate'],result['completion_rate'],result['mean_reward'])
class Progress(BaseCallback):
    def __init__(self,out,slope,start_steps,budget,initial,normalized=False):
        super().__init__();self.normalized=normalized;self.out=out;self.slope=slope;self.next_eval=start_steps+65536;self.start=time.perf_counter();self.budget=budget;self.best=score(initial);self.history=[]
    def _on_step(self):return time.perf_counter()-self.start<self.budget
    def _on_rollout_end(self):
        if self.num_timesteps>=self.next_eval:
            result=evaluate(self.model,self.slope,range(3000,3003),self.normalized);self.history.append(dict(steps=self.num_timesteps,wall_seconds=time.perf_counter()-self.start,**result))
            (self.out/'learning_curve.json').write_text(json.dumps(self.history,indent=2));self.model.save(self.out/'latest')
            if score(result)>self.best:self.best=score(result);self.model.save(self.out/'best')
            print(f"EVAL {self.num_timesteps}: success={result['success_rate']:.0%}, distance={result['mean_forward_m']:.3f}m, drift={result['mean_max_lateral_m']:.3f}m",flush=True)
            self.next_eval=self.num_timesteps+65536

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--slope',type=float,default=0,choices=[0,3,5]);ap.add_argument('--resume',required=True);ap.add_argument('--out',required=True);ap.add_argument('--steps',type=int,default=262144);ap.add_argument('--minutes',type=float,default=6);ap.add_argument('--seed',type=int,default=84);ap.add_argument('--evaluate-only',action='store_true');ap.add_argument('--evaluation-seed',type=int,default=4000);ap.add_argument('--normalized',action='store_true');ap.add_argument('--exploration-std',type=float);args=ap.parse_args()
    torch.set_num_threads(1);out=ROOT/args.out;out.mkdir(parents=True,exist_ok=True);t=time.perf_counter()
    source_summary=Path(args.resume).parent/'summary.json'
    if not source_summary.exists():source_summary=Path(args.resume).parent/'run_config.json'
    source_normalized=json.loads(source_summary.read_text()).get('normalized_observations',False) if source_summary.exists() else False
    args.normalized=args.normalized or source_normalized
    if not args.evaluate_only and (out/'best.zip').exists():
        raise SystemExit('Output already contains a checkpoint. Choose a new --out folder to preserve the experiment.')
    if args.evaluate_only and args.normalized and not source_normalized:
        raise SystemExit('Evaluation must use the observation convention of the saved policy.')
    config=dict(slope_deg=args.slope,normalized_observations=args.normalized,resume=args.resume,seed=args.seed,learning_rate=1e-4,exploration_std=args.exploration_std,requested_additional_steps=args.steps)
    (out/'run_config.json').write_text(json.dumps(config,indent=2))
    if args.evaluate_only:
        r=evaluate(PPO.load(args.resume,device='cpu'),args.slope,range(args.evaluation_seed,args.evaluation_seed+20),args.normalized);(out/'heldout.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));return
    practice=[(0.,.3)]*8 if not args.slope else [(0.,.3)]*2+[(min(args.slope,3.),.5)]*2+[(args.slope,.5)]*2+[(args.slope,.3)]*2
    env=SubprocVecEnv([partial(make_env,s,args.normalized,w) for s,w in practice],start_method='spawn')
    try:
        policy=PPO.load(args.resume,env=env,device='cpu');policy.set_random_seed(args.seed)
        if args.normalized and not source_normalized:
            # Preserve the pretrained function while improving gradient scaling.
            with torch.no_grad():
                scale=torch.as_tensor(OBS_SCALE)
                policy.policy.mlp_extractor.policy_net[0].weight.div_(scale)
                policy.policy.mlp_extractor.value_net[0].weight.div_(scale)
            policy.policy.optimizer.state.clear()
        if args.exploration_std is not None:
            with torch.no_grad():policy.policy.log_std.fill_(float(np.log(args.exploration_std)))
            policy.policy.optimizer.state.clear()
            policy.ent_coef=.001
        policy.learning_rate=1e-4;policy.lr_schedule=lambda _:1e-4
        policy.set_logger(configure(str(out),['stdout','csv']))
        initial=evaluate(policy,args.slope,range(3000,3003),args.normalized);policy.save(out/'best');(out/'initial.json').write_text(json.dumps(initial,indent=2))
        cb=Progress(out,args.slope,policy.num_timesteps,args.minutes*60,initial,args.normalized)
        policy.learn(total_timesteps=args.steps,reset_num_timesteps=False,callback=cb);policy.save(out/'latest')
        final=evaluate(policy,args.slope,range(3000,3003),args.normalized)
        if score(final)>cb.best:policy.save(out/'best')
        best=PPO.load(out/'best',device='cpu');heldout=evaluate(best,args.slope,range(args.evaluation_seed,args.evaluation_seed+20),args.normalized)
        regression=evaluate(best,0,range(5000,5020),args.normalized) if args.slope else None
        result=dict(training_slope_width_pairs=practice,flat_regression=regression,normalized_observations=args.normalized,slope_deg=args.slope,total_steps=policy.num_timesteps,selected_steps=best.num_timesteps,
            wall_seconds=time.perf_counter()-t,initial=initial,final=final,heldout=heldout,
            gate_passed=heldout['success_rate']>=.9 and (regression is None or regression['success_rate']>=.9),seed=args.seed,
            model_sha256=hashlib.sha256((ROOT/'models/robot_b3.xml').read_bytes()).hexdigest(),
            environment_sha256=hashlib.sha256((ROOT/'rl_b3_curriculum.py').read_bytes()).hexdigest(),
            success='Flat: 0.5 m in 10s; slopes: all three surfaces and exit cleared in 25s; <=3cm drift, <=10deg heading, no nonfoot contact, fall or edge violation')
        (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    finally:env.close()
if __name__=='__main__':main()
