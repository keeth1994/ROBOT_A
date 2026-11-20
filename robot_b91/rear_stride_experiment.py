"""Compare rear-leg stroke reductions across assumed sharp-crest layouts.

Local mechanics only: ideal feedback and unmeasured friction/contact parameters.
No network or robot commands. Success requires clearance and heading <=12deg;
the model also stops on roll >12deg or lateral displacement >4cm.
"""
from concurrent.futures import ProcessPoolExecutor
import json

from robot_b91.crest_experiment import ROOT, build, simulate
from robot_b91.gait import RAMP_STEERING


def main():
    output = ROOT / 'results/ramp_trials/rear_stride_sim'
    common = dict(pattern='crest', duty=.8, pitch_lift=.25,
                  pitch_stance=0, yaw_bias=-.25) | RAMP_STEERING
    profiles = {
        'short_equal': dict(yaw_amplitude=.35, rear_stride_scale=1),
        'front045_rear35pct': dict(yaw_amplitude=.45, rear_stride_scale=.35),
        'front055_rear35pct': dict(yaw_amplitude=.55, rear_stride_scale=.35),
    }
    # Sample changed layouts, rather than assume the current ramp dimensions.
    # Offsets are simulation initial conditions, never robot control inputs.
    scenarios = [(20, .45, -.02), (25, .60, 0), (30, .75, -.02)]
    jobspecs = []
    for slope, crest, offset in scenarios:
        folder = output / f'slope{slope}_crest{crest}'
        build(slope, crest, folder)
        for name, profile in profiles.items():
            for speed in (.5, .65):
                for friction in (.5, .8, 1.1):
                    params = common | profile | dict(initial_x=crest+offset, speed=speed)
                    case = (f'{name}_speed{speed}', params, friction)
                    jobspecs.append((case, 40, 'crest', folder))
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs = [pool.submit(simulate, *args) for args in jobspecs]
        results = [job.result() for job in jobs]
    (output / 'comparison.json').write_text(json.dumps(results, indent=2))
    for name in profiles:
        group = [r for r in results if r['case'].startswith(name + '_speed')]
        print(json.dumps(dict(profile=name, cases=len(group),
            crossed=sum(r['reason']=='body passed crest by 22 cm' for r in group),
            crossed_within_heading_limit=sum(r['reason']=='body passed crest by 22 cm'
                and r['max_heading_deg'] <= 12 for r in group),
            unstable_settling=sum(r['seconds'] <= 2.01 for r in group))), flush=True)


if __name__ == '__main__':
    main()
