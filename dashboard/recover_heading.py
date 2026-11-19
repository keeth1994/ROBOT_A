"""Bounded IMU-controlled pivot to reacquire a ramp; no forward stride."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_dashboard import devices
from robot_dashboard.walking import Walking
from ramp_trial import Client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--degrees', type=float, required=True, help='Positive turns left.')
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.degrees) or not 1 <= abs(args.degrees) <= 15 or not 1 <= args.seconds <= 20:
        parser.error('Use a 1–15 degree turn lasting at most 20 seconds.')
    args.output.mkdir(parents=True, exist_ok=False)
    client = Client(8765)
    state = client.request('/api/state')
    client.token = state['token']
    url = state['config']['servo']
    if state['walking']['active']:
        raise RuntimeError('Stop dashboard walking before a recovery pivot.')
    status = devices.request(url, '/status')
    walking = Walking()
    walking.prepare(status)
    if (status.get('externalActive') or status.get('running') or status.get('manualActive') or
            not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50):
        raise RuntimeError('Recovery needs idle servos and fresh orientation.')
    settings = walking.start_settings('course')
    # This MCU profile exposes forward=0; camera/corridor search runs nowhere here.
    settings['profile'] = 'corridor_search'
    started = time.monotonic()
    sequence = 0
    target = None
    within_since = None
    reason = 'Pivot time limit reached.'
    result = None
    try:
        result = devices.request(url, '/motion/start', settings)
        with (args.output / 'telemetry.jsonl').open('w') as log:
            while time.monotonic() - started < args.seconds:
                now = time.monotonic()
                if (args.output / 'STOP').exists():
                    reason = 'Operator stopped pivot.'
                    break
                status = devices.request(url, '/status', timeout=.2)
                if not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50:
                    raise RuntimeError('Fresh orientation lost.')
                roll, pitch, heading = status['orientation']
                if abs(roll) > math.radians(20) or abs(pitch) > math.radians(35):
                    raise RuntimeError('Tilt exceeded pivot limits.')
                elapsed = now - started
                if target is None and elapsed >= 1.8:
                    target = heading + math.radians(args.degrees)
                error = 0 if target is None else math.atan2(math.sin(heading - target), math.cos(heading - target))
                sequence += 1
                devices.request(url, '/motion/adjust',
                                {'session': result['session'], 'sequence': sequence,
                                 'speed': .5, 'forward': 0, 'ready': int(target is not None),
                                 'heading': error, 'heading_reference': heading, 'lateral': 0}, timeout=.2)
                log.write(json.dumps({'seconds': elapsed, 'error_deg': math.degrees(error), 'status': status}) + '\n')
                log.flush()
                if target is not None:
                    if abs(error) < math.radians(2):
                        within_since = now if within_since is None else within_since
                        if now - within_since >= .3:
                            reason = 'Target heading reached.'
                            break
                    else:
                        within_since = None
                if sequence % 10 == 0:
                    print(json.dumps({'seconds': round(elapsed, 1), 'remaining_deg': round(-math.degrees(error), 1)}), flush=True)
                time.sleep(max(0, .1 - (time.monotonic() - now)))
    finally:
        stopped = devices.request(url, '/stop', {}, timeout=.8)
        summary = {'reason': reason, 'requested_degrees': args.degrees,
                   'duration_s': time.monotonic() - started, 'stop': stopped}
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
