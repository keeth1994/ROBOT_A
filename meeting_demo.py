"""Meeting demos: real motor-driven physics, live or recorded at 1x speed."""
import argparse
import json
import time
from pathlib import Path
import mujoco
import numpy as np
from run_robot_b3 import ROOT, Robot

MODES = {
    'normal': ('PROGRAMMED WALK | FLAT', None, 0),
    'rl-flat': ('LEARNED PPO | FLAT', 'results/curriculum_selected/flat/policy.zip', 0),
    'rl-ramp': ('LEARNED PPO | 3 DEGREE LESSON', 'results/my_3deg_run_02/best.zip', 3),
}

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('demo', choices=MODES)
    ap.add_argument('--record', action='store_true', help='Save a labelled MP4 instead of opening a live viewer')
    ap.add_argument('--seconds', type=float, default=12)
    ap.add_argument('--seed', type=int, default=4000)
    args = ap.parse_args()
    title, checkpoint, slope = MODES[args.demo]
    env = None
    if checkpoint:
        import torch
        from stable_baselines3 import PPO
        from rl_b3_curriculum import CurriculumEnv
        torch.set_num_threads(1)
        path = ROOT / checkpoint
        meta = json.loads((path.parent / 'summary.json').read_text())
        env = CurriculumEnv(slope=slope, visuals=True, normalized=meta['normalized_observations'])
        policy = PPO.load(path, device='cpu')
        obs, _ = env.reset(seed=args.seed)
        robot = env.robot
    else:
        robot = Robot(json.loads((ROOT / 'reference/b3_gait.json').read_text()))
    start_x = float(robot.d.qpos[0])
    elapsed = 0.0
    info = {}
    done = False
    def step():
        nonlocal obs, elapsed, info, done
        if env:
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(action)
            done = term or trunc
        else:
            for _ in range(10):
                robot.step()
        elapsed += .02
        done = done or elapsed >= args.seconds
    def camera(cam):
        cam.distance = 1.8 if slope else 1.05
        cam.azimuth = 110
        cam.elevation = -30
        cam.lookat[:] = [.75, 0, .04] if slope else [robot.d.qpos[0] + .15, 0, .04]
    def status():
        return f't={elapsed:.2f}s | forward={robot.d.qpos[0]-start_x:.3f}m | sideways={robot.d.qpos[1]:+.3f}m | tilt={robot.tilt():.1f}deg'
    if args.record:
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw
        out = ROOT / 'results/meeting'
        out.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(robot.m, height=720, width=1280)
        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()
        opt.geomgroup[3] = 0
        frames = 0
        try:
            with imageio.get_writer(str(out / (args.demo + '.mp4')), fps=25, codec='libx264') as writer:
                while not done:
                    step()
                    if not done:
                        step()
                    camera(cam)
                    renderer.update_scene(robot.d, camera=cam, scene_option=opt)
                    im = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(im)
                    draw.rectangle((0, 0, 1280, 80), fill=(18, 24, 35))
                    draw.text((18, 10), title + ' | 1x playback | original servo limits', fill='white')
                    draw.text((18, 34), status(), fill='white')
                    draw.text((18, 57), 'Motor-driven simulation; ideal feedback; not a hardware validation', fill='#c4cedd')
                    writer.append_data(np.asarray(im))
                    if frames == 50:
                        im.save(out / (args.demo + '.png'))
                    frames += 1
                im.save(out / (args.demo + '_final.png'))
        finally:
            renderer.close()
        report = {'demo': args.demo, 'checkpoint': checkpoint, 'seed': args.seed, 'elapsed_s': elapsed,
                  'forward_m': float(robot.d.qpos[0])-start_x, 'sideways_m': float(robot.d.qpos[1]),
                  'final_tilt_deg': robot.tilt(), 'episode': info}
        (out / (args.demo + '.json')).write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print(out / (args.demo + '.mp4'))
    else:
        from mujoco import viewer
        print(title + '\nClose the viewer to exit. Episodes repeat; original motor limits apply.')
        with viewer.launch_passive(robot.m, robot.d) as v:
            v.opt.geomgroup[3] = 0
            while v.is_running():
                started = time.perf_counter()
                step()
                camera(v.cam)
                v.sync()
                if done:
                    print(status(), info, flush=True)
                    if env:
                        obs, _ = env.reset(seed=args.seed)
                    else:
                        robot.reset()
                    elapsed = 0
                    start_x = float(robot.d.qpos[0])
                    done = False
                time.sleep(max(0, .02 - (time.perf_counter() - started)))
    if env:
        env.close()

if __name__ == '__main__':
    main()
