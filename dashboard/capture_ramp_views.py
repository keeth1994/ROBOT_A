"""Capture stationary camera-angle checks; never enable a leg servo."""
import argparse
import json
from pathlib import Path
import threading
import time

from ramp_trial import Client
from robot_dashboard import devices


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--angles', type=int, nargs='+', default=[-80, -75, -70, -65, -60, -55])
    args = parser.parse_args()
    if any(not -80 <= angle <= 45 for angle in args.angles):
        parser.error('Camera angles must stay within calibrated -80 to +45 degrees.')
    args.output.mkdir(parents=True, exist_ok=False)
    client = Client(8765)
    state = client.request('/api/state')
    client.token = state['token']
    url = state['config']['servo']
    if state['walking']['active']:
        raise RuntimeError('Stop walking before stationary camera measurements.')
    status = devices.request(url, '/status')
    if any(status.get(key) for key in ('running', 'externalActive', 'manualActive')):
        raise RuntimeError('Leg controls must be idle.')
    stopped = threading.Event()
    errors = []
    records = []

    def heartbeat():
        while not stopped.wait(.2):
            try:
                client.request('/api/heartbeat', {})
            except Exception as error:
                errors.append(str(error))
                return

    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    try:
        client.request('/api/heartbeat', {})
        client.request('/api/video', {'enabled': True})
        client.request('/api/servo/camera-pitch', {'angle': 0})
        for angle in args.angles:
            if errors:
                raise RuntimeError(errors[-1])
            client.request('/api/servo/camera-pitch', {'angle': angle})
            time.sleep(.6)
            for sample in range(5):
                if errors:
                    raise RuntimeError(errors[-1])
                before = time.monotonic()
                jpeg = client.camera_frame()
                status = devices.request(url, '/status', timeout=.3)
                if (any(status.get(key) for key in ('running', 'externalActive', 'manualActive'))
                        or status['cameraPitch']['angle'] != angle
                        or status['cameraPitch']['moving']):
                    raise RuntimeError('Robot control or camera angle changed during capture.')
                if not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50:
                    raise RuntimeError('Fresh orientation required for calibration.')
                name = f'angle_{angle}_frame_{sample}.jpg'
                (args.output / name).write_bytes(jpeg)
                records.append({'image': name, 'angle': angle, 'status': status,
                                'capture_start': before, 'status_received': time.monotonic()})
                time.sleep(.12)
            print(f'Captured 5 frames at camera servo {angle} degrees; legs idle.', flush=True)
    finally:
        stopped.set()
        worker.join(timeout=2)
        try:
            print(client.request('/api/servo/stop', {}), flush=True)
        finally:
            (args.output / 'capture.json').write_text(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()
