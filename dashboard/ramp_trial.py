"""Record a ramp run through the dashboard API until completion or operator stop.

The laptop webcam is recorded for the operator; it supplies no steering input.
Create OUTPUT/STOP to end an attempt immediately. Every exit requests servo stop.
"""
import argparse
from datetime import datetime
import http.client
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import imageio_ffmpeg
from robot_dashboard.devices import mjpeg_frames


class Client:
    def __init__(self, port):
        self.port = port
        self.token = ''

    def request(self, path, values=None, *, binary=False):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        try:
            body = None if values is None else json.dumps(values)
            headers = {'X-Dashboard-Token': self.token, 'Content-Type': 'application/json'}
            conn.request('GET' if body is None else 'POST', path, body, headers)
            response = conn.getresponse()
            payload = response.read()
            if response.status != 200:
                raise RuntimeError(f'{path}: HTTP {response.status}: {payload.decode()}')
            return payload if binary else json.loads(payload)
        finally:
            conn.close()

    def camera_frame(self):
        """Save unannotated onboard input for later perception diagnosis."""
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=1)
        try:
            conn.request('GET', '/api/video.mjpeg')
            response = conn.getresponse()
            if response.status != 200:
                raise RuntimeError(f'Camera stream: HTTP {response.status}')
            return next(mjpeg_frames(response))
        finally:
            conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float,
                        help='Optional operator-selected duration. Default: run until the course ends or Stop.')
    parser.add_argument('--speed', type=float, default=.5)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--profile', choices=('course', 'course_imu'), default='course')
    parser.add_argument('--assisted-crest-start', action='store_true',
                        help='Require dashboard --start-at-crest; record assisted placement explicitly.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error('Duration must be finite and positive.')
    if not .25 <= args.speed <= 1:
        parser.error('Use speed 0.25 to 1.0.')
    if args.assisted_crest_start and args.profile != 'course':
        parser.error('Assisted crest trials require the course profile.')
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'telemetry.jsonl').exists() or (args.output / 'STOP').exists():
        parser.error('Choose a new output directory for each attempt.')
    root = Path(__file__).resolve().parents[1]
    for source in ('reference/b91_gait.toml', 'reference/b10_camera_scan.json',
                   'dashboard/navigation.json'):
        shutil.copyfile(root / source, args.output / Path(source).name)
    client = Client(args.port)
    state = client.request('/api/state')
    client.token = state['token']
    if state.get('auto_corridor') is not False:
        raise RuntimeError('Start the dashboard with --ramp-only before a ramp trial.')
    if bool(state['walking'].get('assisted_crest_start')) != args.assisted_crest_start:
        raise RuntimeError('Assisted crest start differs between dashboard and recorder; movement was not started.')
    if state['walking']['active'] or (state['servo'].get('data') or {}).get('externalActive'):
        raise RuntimeError('An existing motion session is active; this trial did not take over.')
    if not state['servo']['online']:
        raise RuntimeError('The servo controller must be online before a trial.')
    if not state['camera']['online']:
        raise RuntimeError('The onboard camera must be online before a ramp trial.')
    latest = args.output / 'webcam.jpg'
    capture_log = (args.output / 'webcam.log').open('w')
    capture = subprocess.Popen([
        imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error',
        '-f', 'dshow', '-i', 'video=Integrated Webcam',
        '-vf', 'fps=2,scale=1280:-2', '-update', '1', '-atomic_writing', '1', '-y', str(latest),
    ], stdout=subprocess.DEVNULL, stderr=capture_log)
    started = None
    stop_reason = 'Trial did not start.'
    peak_roll = peak_pitch = 0.0
    try:
        deadline = time.monotonic() + 8
        while not latest.exists() and time.monotonic() < deadline and capture.poll() is None:
            time.sleep(.1)
        if not latest.exists():
            raise RuntimeError('No laptop webcam frame; movement was not started.')
        client.request('/api/walk/speed', {'speed': args.speed})
        client.request('/api/video', {'enabled': True})
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            client.request('/api/heartbeat', {})
            state = client.request('/api/state')
            if state['pipeline']['fresh']:
                break
            time.sleep(.2)
        if not state['pipeline']['fresh']:
            raise RuntimeError('No fresh onboard camera frame; movement was not started.')
        client.request('/api/heartbeat', {})
        client.request('/api/walk/' + args.profile, {})
        started = time.monotonic()
        duration = f'{args.seconds:g} s' if args.seconds is not None else 'until course completion or Stop'
        print(f'Started {args.profile} at {args.speed:.0%}, {duration}.', flush=True)
        next_sample = next_image = next_report = 0.0
        stop_reason = 'Trial time limit reached.'
        with (args.output / 'telemetry.jsonl').open('w', encoding='utf-8') as log:
            while args.seconds is None or time.monotonic() - started < args.seconds:
                loop_started = time.monotonic()
                elapsed = loop_started - started
                if (args.output / 'STOP').exists():
                    stop_reason = 'Operator stop file.'
                    break
                if capture.poll() is not None or time.time() - latest.stat().st_mtime > 3:
                    stop_reason = 'Laptop observation camera stopped updating.'
                    break
                client.request('/api/heartbeat', {})
                if elapsed >= next_sample:
                    state = client.request('/api/state')
                    state.pop('token', None)
                    record = {'elapsed_s': round(elapsed, 3), **state}
                    log.write(json.dumps(record) + '\n')
                    log.flush()
                    temporary = args.output / 'state.tmp'
                    temporary.write_text(json.dumps(record, indent=2), encoding='utf-8')
                    try:
                        temporary.replace(args.output / 'state.json')
                    except PermissionError:
                        # Windows viewers may briefly hold the previous snapshot open.
                        # The append-only telemetry is already flushed; skip this preview.
                        pass
                    next_sample = elapsed + .4
                    data = state['servo'].get('data') or {}
                    attitude = data.get('orientation')
                    if attitude and data.get('orientationValid'):
                        roll, pitch = [abs(math.degrees(float(a))) for a in attitude[:2]]
                        peak_roll, peak_pitch = max(peak_roll, roll), max(peak_pitch, pitch)
                    if not state['walking']['active']:
                        stop_reason = state['walking']['message']
                        break
                if elapsed >= next_image:
                    stamp = f'{elapsed:06.1f}'
                    shutil.copyfile(latest, args.output / f'webcam_{stamp}.jpg')
                    (args.output / f'camera_{stamp}.jpg').write_bytes(client.camera_frame())
                    panel = client.request('/api/navigation-debug.jpg', binary=True)
                    (args.output / 'navigation.jpg').write_bytes(panel)
                    (args.output / f'navigation_{stamp}.jpg').write_bytes(panel)
                    next_image = elapsed + 2
                if elapsed >= next_report:
                    nav = state['walking'].get('navigation') or {}
                    print(json.dumps({'seconds': round(elapsed, 1),
                                      'message': state['walking']['message'],
                                      'navigation': nav}), flush=True)
                    next_report = elapsed + 5
                time.sleep(max(0, .2 - (time.monotonic() - loop_started)))
    except BaseException as error:
        stop_reason = f'{type(error).__name__}: {error}'
        raise
    finally:
        stop_result = None
        for _ in range(2):
            try:
                stop_result = client.request('/api/servo/stop', {})
                break
            except Exception as error:
                stop_result = {'error': str(error)}
        capture.terminate()
        try:
            capture.wait(timeout=3)
        except subprocess.TimeoutExpired:
            capture.kill()
            capture.wait()
        capture_log.close()
        summary = {'timestamp': datetime.now().astimezone().isoformat(),
                   'duration_s': round(time.monotonic() - started, 2) if started is not None else 0,
                   'speed': args.speed, 'reason': stop_reason,
                   'requested_duration_s': args.seconds,
                   'profile': args.profile,
                   'assisted_crest_start': args.assisted_crest_start,
                   'peak_roll_deg': round(peak_roll, 2), 'peak_pitch_deg': round(peak_pitch, 2),
                   'stop_response': stop_result}
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
