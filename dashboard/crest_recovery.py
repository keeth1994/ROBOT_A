"""One bounded compact-stance recovery using calibrated manual joint commands.

This is an operator-observed experiment, not automatic ramp navigation.
The original gait is unchanged. Use --dry-run to inspect the joint plan.
Trials 19 and 20 remained upright but did not clear the first crest; this is not a
validated recovery gait and is not enabled by the dashboard.
"""
import argparse
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

import imageio_ffmpeg
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ramp_trial import Client
from robot_dashboard import devices
from robot_dashboard.walking import Walking, JOINTS
from robot_dashboard.navigation import CourseNavigation


def plan(compact_deg=40, stride_deg=5):
    if not 30 <= compact_deg <= 40 or not 1 <= stride_deg <= 10 or compact_deg+stride_deg > 45:
        raise ValueError('Compact yaw must remain inside calibrated +/-45 degree limits')
    neutral = [0, 45, 0, 45, 0, 45, 0, 45]
    # Physical servo angles. Rear yaw mounting is opposite to model joint yaw.
    low, high = compact_deg-stride_deg, compact_deg+stride_deg
    ahead = [low, 45, high, 45, -low, 45, -high, 45]
    behind = [high, 45, low, 45, -high, 45, -low, 45]
    stages = [('neutral', 1., neutral), ('compact stance', 3., ahead)]
    for cycle in range(2):
        stages.append((f'push {cycle+1}', 1.5, behind.copy()))
        pose = behind.copy()
        for leg in (0, 3, 2, 1):
            pose[2*leg+1] = 15
            stages.append((f'return {cycle+1} leg {leg}: lift', .6, pose.copy()))
            pose[2*leg] = ahead[2*leg]
            stages.append((f'return {cycle+1} leg {leg}: place', .5, pose.copy()))
            pose[2*leg+1] = 45
            stages.append((f'return {cycle+1} leg {leg}: lower', .6, pose.copy()))
    stages.append(('final push', 1.5, behind.copy()))
    return stages


def fit_heading_drift(samples):
    """Estimate idle MCU yaw drift; refuse a noisy or visibly tilting sample."""
    if len(samples) < 20:
        raise ValueError('Insufficient stationary orientation samples')
    times = np.array([s['imuTimestampMs']/1000 for s in samples])
    attitudes = np.array([s['orientation'] for s in samples])
    if times[-1]-times[0] < 2 or np.any(np.diff(times) <= 0):
        raise ValueError('Stationary samples must span two seconds on a monotonic MCU clock')
    if np.max(np.ptp(attitudes[:,:2],axis=0)) > math.radians(1):
        raise ValueError('Robot moved during stationary heading calibration')
    yaw = np.unwrap(attitudes[:,2])
    rate, intercept = np.polyfit(times-times[0],yaw,1)
    residual = float(np.std(yaw-(rate*(times-times[0])+intercept)))
    if abs(rate) > math.radians(.75) or residual > math.radians(.1):
        raise ValueError('Heading calibration is not sufficiently stationary')
    return dict(rate_rad_s=float(rate), yaw_rad=float(attitudes[-1,2]),
                time_ms=int(samples[-1]['imuTimestampMs']), residual_rad=residual)


def heading_change(reference, status):
    elapsed = (status['imuTimestampMs']-reference['time_ms'])/1000
    if not 0 <= elapsed <= 60:
        raise ValueError('Stationary heading reference expired or controller clock reset')
    angle = status['orientation'][2]-reference['yaw_rad']-reference['rate_rad_s']*elapsed
    return math.atan2(math.sin(angle),math.cos(angle))


def visual_pose(status, jpeg):
    """An independent ramp fit for a fixed camera, using measured roll/pitch."""
    observer = CourseNavigation(0,scan_ramp=False)
    sample = dict(status,orientation=[*status['orientation'][:2],0])
    if sample['cameraPitch'].get('moving') or sample['cameraPitch'].get('angle') != -65:
        raise RuntimeError('Fixed recovery camera is not settled')
    observer.update(sample,3,None)
    observer.update(sample,3.2,(1,3.2,np.array(Image.open(BytesIO(jpeg)).convert('RGB'))))
    estimate = observer.estimate
    if not estimate.debug or not estimate.debug['accepted']:
        raise RuntimeError('Recovery lost visual ramp guidance')
    # A fresh estimate uses one 0.3-gain lateral update from zero.
    return dict(heading_deg=math.degrees(estimate.heading), lateral_m=estimate.lateral/.3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    stages = plan()
    for _, duration, pose in stages:
        assert duration > 0 and all(-45 <= a <= 45 for a in pose[::2])
        assert all(-90 <= a <= 45 for a in pose[1::2])
        assert sum(a < 45 for a in pose[1::2]) <= 1
    if args.dry_run:
        print(json.dumps({'duration_s': sum(s[1] for s in stages), 'stages': stages}, indent=2))
        return
    if args.output is None:
        parser.error('--output is required for a physical test')
    args.output.mkdir(parents=True, exist_ok=False)
    client = Client(8765)
    state = client.request('/api/state')
    client.token = state['token']
    url = state['config']['servo']
    status = devices.request(url, '/status')
    Walking().prepare(status)
    if state['walking']['active'] or any(status.get(k) for k in ('externalActive','manualActive','running')):
        raise RuntimeError('Existing motion must be stopped; no takeover was attempted')
    if not status.get('orientationValid') or status.get('imuAgeMs', 999) > 50:
        raise RuntimeError('Fresh onboard orientation required')
    if abs(status['orientation'][0]) > math.radians(8) or abs(status['orientation'][1]) > math.radians(20):
        raise RuntimeError('Initial crest pose is outside the recovery envelope')
    mapping = {j['name']: j['channel'] for j in status['joints']}
    (args.output/'plan.json').write_text(json.dumps(stages, indent=2))
    (args.output/'initial_status.json').write_text(json.dumps(status, indent=2))
    webcam = args.output/'webcam.jpg'
    capture_log = (args.output/'webcam.log').open('w')
    capture = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner','-loglevel','error',
        '-f','dshow','-i','video=Integrated Webcam','-vf','fps=2,scale=1280:-2',
        '-update','1','-atomic_writing','1','-y',str(webcam)],
        stdout=subprocess.DEVNULL,stderr=capture_log)
    started = None
    reason = 'Not started'
    last_commands = [None]*8
    last_image = -10
    peak_roll = 0
    stop_result = None
    try:
        deadline = time.monotonic()+8
        while not webcam.exists() and time.monotonic() < deadline and capture.poll() is None:
            time.sleep(.1)
        if not webcam.exists():
            raise RuntimeError('No laptop camera feedback; motion not started')
        client.request('/api/video', {'enabled': True})
        client.request('/api/heartbeat', {})
        client.request('/api/servo/camera-pitch', {'angle': 0})
        client.request('/api/servo/camera-pitch', {'angle': -65})
        deadline = time.monotonic()+5
        while True:
            client.request('/api/heartbeat', {})
            sample = devices.request(url,'/status',timeout=.2)
            if sample['cameraPitch'].get('angle') == -65 and not sample['cameraPitch'].get('moving'):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Camera did not settle; legs not enabled')
            time.sleep(.1)
        samples=[]
        calibration_start=time.monotonic()
        while time.monotonic()-calibration_start < 4:
            client.request('/api/heartbeat', {})
            sample=devices.request(url,'/status',timeout=.2)
            if (any(sample.get(k) for k in ('manualActive','externalActive','running'))
                    or not sample.get('orientationValid') or sample.get('imuAgeMs',999)>50):
                raise RuntimeError('Stationary calibration requires idle legs and fresh orientation')
            samples.append(sample)
            time.sleep(.1)
        reference=fit_heading_drift(samples)
        (args.output/'heading_calibration.json').write_text(json.dumps(dict(reference=reference,samples=samples),indent=2))
        print(json.dumps({'idle_heading_drift_deg_s':math.degrees(reference['rate_rad_s'])}),flush=True)
        started = time.monotonic()
        with (args.output/'telemetry.jsonl').open('w') as log:
            def tick(pose, stage):
                nonlocal last_image, peak_roll
                now = time.monotonic()
                elapsed = now-started
                if elapsed > 35 or (args.output/'STOP').exists():
                    raise RuntimeError('Recovery time limit or operator stop')
                if capture.poll() is not None or time.time()-webcam.stat().st_mtime > 3:
                    raise RuntimeError('Observation camera stopped updating')
                client.request('/api/heartbeat', {})
                sample = devices.request(url, '/status', timeout=.2)
                if (not sample.get('orientationValid') or sample.get('imuAgeMs', 999)>50
                        or sample.get('faulted') or sample.get('externalActive') or sample.get('running')):
                    raise RuntimeError('Fresh orientation or exclusive manual control lost')
                if any(a is not None for a in last_commands) and not sample.get('manualActive'):
                    raise RuntimeError('Manual outputs were stopped; recovery will not restart them')
                roll,pitch,yaw = sample['orientation']
                error = heading_change(reference,sample)
                peak_roll = max(peak_roll, abs(math.degrees(roll)))
                if abs(roll)>math.radians(12) or abs(pitch)>math.radians(30) or abs(error)>math.radians(10):
                    raise RuntimeError('Recovery limit reached: '
                        f'roll {math.degrees(roll):.2f}, pitch {math.degrees(pitch):.2f}, '
                        f'heading change {math.degrees(error):.2f} degrees')
                for index, angle in enumerate(pose):
                    angle = round(angle)
                    if last_commands[index] != angle:
                        client.request('/api/servo/servo', {'channel': mapping[JOINTS[index]],'angle': angle})
                        last_commands[index] = angle
                log.write(json.dumps({'elapsed_s':elapsed,'stage':stage,'commands':last_commands,
                                      'corrected_heading_change_deg':math.degrees(error),'status':sample})+'\n')
                log.flush()
                if elapsed-last_image >= 2:
                    stamp = f'{elapsed:06.1f}'
                    shutil.copyfile(webcam,args.output/f'webcam_{stamp}.jpg')
                    jpeg=client.camera_frame()
                    (args.output/f'camera_{stamp}.jpg').write_bytes(jpeg)
                    observed=visual_pose(sample,jpeg)
                    with (args.output/'vision.jsonl').open('a') as vision:
                        vision.write(json.dumps({'elapsed_s':elapsed,**observed})+'\n')
                    if abs(observed['heading_deg'])>8 or abs(observed['lateral_m'])>.015:
                        raise RuntimeError(f'Recovery visual alignment limit reached: {observed}')
                    last_image = elapsed
                time.sleep(max(0,.1-(time.monotonic()-now)))
            previous = stages[0][2]
            for label, duration, target in stages:
                print(json.dumps({'stage':label,'elapsed_s':round(time.monotonic()-started,1)}),flush=True)
                stage_start = time.monotonic()
                while time.monotonic()-stage_start < duration:
                    fraction = min(1,(time.monotonic()-stage_start)/duration)
                    fraction = fraction*fraction*(3-2*fraction)
                    tick([a+(b-a)*fraction for a,b in zip(previous,target)],label)
                tick(target,label)
                previous = target
            reason = 'Completed two compact returns and three short pushes; physical outcome requires observation'
    except BaseException as error:
        reason = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            for attempt in range(2):
                try:
                    stop_result = client.request('/api/servo/stop', {})
                    break
                except Exception as error:
                    stop_result = {'error':str(error)}
        finally:
            capture.terminate()
            try:
                capture.wait(timeout=3)
            except subprocess.TimeoutExpired:
                capture.kill()
                capture.wait()
            capture_log.close()
            summary = {'reason':reason,'duration_s':time.monotonic()-started if started else 0,
                       'peak_roll_deg':peak_roll,'stop':stop_result}
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
            print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    main()
