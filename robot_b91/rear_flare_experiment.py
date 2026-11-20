"""Offline rear flare/lift comparison; no network or physical robot commands.

Rear feet lift and replant alternately, keeping the crest wave's single swing.
The contact model is approximate and cannot predict grip on the real foam.
"""
from concurrent.futures import ProcessPoolExecutor
import argparse
import json
import math

from robot_b91.crest_experiment import ROOT, build, simulate
from robot_b91.gait import RAMP_STEERING, rocker_targets


def rear_targets(params, *args, **kwargs):
    """Simulator-only rear placement override; not sent to live gait firmware."""
    bias = params.get('rear_yaw_bias', params.get('yaw_bias', 0))
    lift = params.get('rear_pitch_lift', params['pitch_lift'])
    if not math.isfinite(bias) or abs(bias) > math.pi/4:
        raise ValueError('Rear yaw bias must be within +/-45 degrees.')
    if not math.isfinite(lift) or not 0 <= lift <= math.radians(125):
        raise ValueError('Rear lift must be between zero and 125 degrees.')
    target = rocker_targets(params, *args, **kwargs)
    rear = rocker_targets(params | dict(yaw_bias=bias, pitch_lift=lift), *args, **kwargs)
    target[[2, 3, 6, 7]] = rear[[2, 3, 6, 7]]
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refine', action='store_true')
    args = parser.parse_args()
    output = ROOT / 'results/ramp_trials/rear_flare_sim'
    output.mkdir(parents=True, exist_ok=True)
    common = dict(pattern='crest', duty=.8, yaw_amplitude=.45,
                  rear_stride_scale=.35, pitch_lift=.25, pitch_stance=0,
                  yaw_bias=-.25) | RAMP_STEERING
    profiles = {'control': {},
                'flare050': dict(rear_yaw_bias=-.50),
                'flare060': dict(rear_yaw_bias=-.60),
                'flare050_lift045': dict(rear_yaw_bias=-.50, rear_pitch_lift=.45),
                'flare060_lift045': dict(rear_yaw_bias=-.60, rear_pitch_lift=.45),
                'flare050_lift065': dict(rear_yaw_bias=-.50, rear_pitch_lift=.65)}
    if args.refine:
        profiles = {'control': {},
                    'flare035': dict(rear_yaw_bias=-.35),
                    'flare040_rear15': dict(rear_yaw_bias=-.40, rear_stride_scale=.15),
                    'flare050_rear0': dict(rear_yaw_bias=-.50, rear_stride_scale=0),
                    'flare040_rear0_lift015': dict(rear_yaw_bias=-.40,
                        rear_stride_scale=0, rear_pitch_lift=.15)}
    specs = []
    for slope, crest in ((25, .6), (30, .75)):
        model_dir = ROOT / f'results/ramp_trials/rear_stride_sim/slope{slope}_crest{crest}'
        if not (model_dir/'model.xml').exists():
            build(slope, crest, model_dir)
        for name, profile in profiles.items():
            for speed in ((.5, .65) if args.refine else (.65,)):
                for friction in (.5, .8, 1.1):
                    params = common | profile | dict(initial_x=crest-.02, speed=speed)
                    specs.append(((f'rear_{name}_speed{speed}', params, friction), 35, 'crest', model_dir, rear_targets))
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs = [pool.submit(simulate, *spec) for spec in specs]
        results = [job.result() for job in jobs]
    (output / ('refined.json' if args.refine else 'comparison.json')).write_text(json.dumps(results, indent=2))
    for name in profiles:
        for slope in (25, 30):
            group = [r for r in results if r['case'].startswith(f'rear_{name}_speed')
                     and r['slope_deg'] == slope]
            print(json.dumps(dict(profile=name, slope=slope, cases=len(group),
                crossed=sum(r['reason']=='body passed crest by 22 cm' for r in group),
                crossed_within_heading_limit=sum(r['reason']=='body passed crest by 22 cm'
                    and r['max_heading_deg'] <= 12 for r in group),
                max_roll=max(r['max_roll_deg'] for r in group))), flush=True)


if __name__ == '__main__':
    main()
