"""Watch a learned B3 policy with the full CAD visuals."""
import argparse,time,json,hashlib
from pathlib import Path
import torch
from stable_baselines3 import PPO
from mujoco import viewer
from rl_b3_env import StraightLineEnv
from run_robot_b3 import ROOT

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',default=None);ap.add_argument('--seconds',type=float,default=None);ap.add_argument('--curriculum',action='store_true');ap.add_argument('--slope',type=float,default=None,choices=[0,3,5]);ap.add_argument('--seed',type=int,default=1000);ap.add_argument('--record',help='Write an MP4 instead of opening the viewer');args=ap.parse_args()
    if args.model is None:
        selected=json.loads((ROOT/'reference/b3_rl_policy.json').read_text())
        if selected['model_sha256']!=hashlib.sha256((ROOT/'models/robot_b3.xml').read_bytes()).hexdigest():
            raise SystemExit('Robot model changed since training. Retrain or explicitly select a policy for revalidation with --model.')
        args.model=str(ROOT/selected['model'])
        args.curriculum=args.curriculum or selected.get('curriculum',False)
        if args.slope is None:args.slope=selected.get('slope_deg')
    run_info=Path(args.model).parent/'summary.json'
    if not run_info.exists():run_info=Path(args.model).parent/'run_config.json'
    run_meta=json.loads(run_info.read_text()) if run_info.exists() else {}
    args.curriculum=args.curriculum or ('slope_deg' in run_meta and 'normalized_observations' in run_meta)
    if args.slope is None:args.slope=run_meta.get('slope_deg',0)
    torch.set_num_threads(1);policy=PPO.load(args.model,device='cpu')
    if args.curriculum:
        from rl_b3_curriculum import CurriculumEnv
        normalized=run_meta.get('normalized_observations',False)
        env=CurriculumEnv(slope=args.slope,seconds=args.seconds,visuals=True,normalized=normalized)
    else:env=StraightLineEnv(seconds=args.seconds or 10,visuals=True)
    obs,_=env.reset(seed=args.seed);r=env.robot
    if args.record:
        import mujoco,imageio.v2 as imageio
        dest=Path(args.record);dest.parent.mkdir(parents=True,exist_ok=True)
        renderer=mujoco.Renderer(r.m,height=720 if args.slope else 480,width=1280 if args.slope else 640);camera=mujoco.MjvCamera()
        camera.distance=1.8 if args.slope else .65;camera.azimuth=110 if args.slope else 135;camera.elevation=-30
        opt=mujoco.MjvOption();opt.geomgroup[3]=0
        writer=imageio.get_writer(str(dest),fps=25);step=0
        try:
            while True:
                action,_=policy.predict(obs,deterministic=True);obs,_,term,trunc,info=env.step(action)
                if step%2==0:
                    camera.lookat[:]=[.75,0,.04] if args.slope else [r.d.qpos[0],r.d.qpos[1],.06]
                    renderer.update_scene(r.d,camera=camera,scene_option=opt);writer.append_data(renderer.render())
                step+=1
                if term or trunc:print(info);break
        finally:writer.close();renderer.close();env.close()
        return
    with viewer.launch_passive(r.m,r.d) as v:
        v.cam.distance=1.8 if args.slope else .65;v.cam.azimuth=110 if args.slope else 135;v.cam.elevation=-30;v.opt.geomgroup[3]=0
        while v.is_running():
            started=time.perf_counter();action,_=policy.predict(obs,deterministic=True)
            obs,reward,term,trunc,info=env.step(action)
            v.cam.lookat[:]=[.75,0,.04] if args.slope else [r.d.qpos[0],r.d.qpos[1],.06];v.sync()
            if term or trunc:
                print(info,flush=True);obs,_=env.reset(seed=args.seed)
            time.sleep(max(0,env.dt-(time.perf_counter()-started)))
if __name__=='__main__':main()
