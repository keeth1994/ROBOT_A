"""Generate a repeatable extended ramp using the physical width and height limit.

Run python -m robot_b91.randomize_ramp, then python robot.py build.
The saved ramp length and hill count are reused unless overridden. This is a
stress-test course, not a reconstruction of the shorter physical ramp.
"""
import argparse
import json
import math
import random

from robot_b91.paths import ROOT

WIDTH = .29
MAX_HEIGHT = .225
BASE_HEIGHT = .01
ANGLE_RANGE = (20., 27.)


def make_profile(length, seed, hills=4):
    if not math.isfinite(length) or length <= 0 or hills < 2 or hills % 2:
        raise ValueError('Use a positive ramp length and an even number of hills (at least two).')
    rng = random.Random(seed)
    heights = [rng.uniform(.16, MAX_HEIGHT) for _ in range(hills)]
    heights[0] = MAX_HEIGHT
    climbs = [rng.uniform(*ANGLE_RANGE) for _ in range(hills)]
    descents = [rng.uniform(*ANGLE_RANGE) for _ in range(hills)]
    climbs[0], descents[-1] = ANGLE_RANGE
    # Keep the first top flat at the height limit; exactly half are crests.
    crests = set(rng.sample(range(1, hills), hills // 2))
    platforms = [i not in crests for i in range(hills)]
    up = [(h - BASE_HEIGHT) / math.tan(math.radians(a)) for h, a in zip(heights, climbs)]
    down = [(h - BASE_HEIGHT) / math.tan(math.radians(a)) for h, a in zip(heights, descents)]
    flat_length = length - sum(up) - sum(down)
    flat_count = sum(platforms) + hills - 1
    if flat_length < .10 * flat_count:
        raise ValueError('The ramp is too short for these hills. Use fewer hills or a longer ramp.')
    weights = [rng.uniform(.7, 1.3) for _ in range(flat_count)]
    flats = iter(flat_length * weight / sum(weights) for weight in weights)
    profile, names = [[0., BASE_HEIGHT]], []

    def add(name, distance, height):
        profile.append([profile[-1][0] + distance, height])
        names.append(name)

    for i in range(hills):
        add(f'climb_{i + 1}', up[i], heights[i])
        if platforms[i]:
            add(f'platform_{i + 1}', next(flats), heights[i])
        add(f'descent_{i + 1}', down[i], BASE_HEIGHT)
        if i < hills - 1:
            add(f'valley_{i + 1}', next(flats), BASE_HEIGHT)
    return profile, names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=20261120)
    parser.add_argument('--hills', type=int, help='Even number of tops; half are crests and half have flat platforms.')
    parser.add_argument('--length', type=float, help='Horizontal ramp length in metres.')
    args = parser.parse_args()
    path = ROOT / 'reference/b91_course_settings.json'
    settings = json.loads(path.read_text())
    hills = args.hills if args.hills is not None else settings['ramp_generation']['hills']
    length = args.length if args.length is not None else settings['profile_xz_m'][-1][0]
    try:
        profile, names = make_profile(length, args.seed, hills)
    except ValueError as error:
        parser.error(str(error))
    old_end = settings['profile_xz_m'][-1][0]
    shift = profile[-1][0] - old_end
    angle = math.radians(settings['yaw_deg'])
    settings['corridor']['origin_xy_m'][0] += shift * math.cos(angle)
    settings['corridor']['origin_xy_m'][1] += shift * math.sin(angle)
    settings.update(
        description='Extended randomized simulation course, not the 2-3 m physical ramp; width 29 cm, height at most 22.5 cm, assumed slopes 20-27 degrees.',
        width_m=WIDTH, profile_xz_m=profile, segment_names=names,
        ramp_generation=dict(seed=args.seed, hills=hills,
                             crests=hills // 2,
                             target_length_m=length, max_height_m=MAX_HEIGHT,
                             slope_range_deg=list(ANGLE_RANGE)),
    )
    path.write_text(json.dumps(settings, indent=2) + '\n')
    print(f'Saved {len(names)} sections across {length:.2f} m; seed {args.seed}.')
    print('Run python robot.py build to rebuild the simulation.')


if __name__ == '__main__':
    main()
