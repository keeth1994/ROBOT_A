"""Project sensor observations. Ideal physics channels, not calibrated hardware signals."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
CONFIG=json.loads((ROOT/'reference/b3_sensors.json').read_text())

def observe(model,data):
    ranges={}
    for sensor in CONFIG['ir']:
        raw=float(data.sensor(sensor['name']).data[0])
        valid=CONFIG['range_m'][0]<=raw<=CONFIG['range_m'][1]
        ranges[sensor['name']]={'distance_m':raw if valid else None,'valid':valid,
            'status':'in_range' if valid else 'unknown_outside_calibrated_range'}
    return {'time_s':float(data.time),'ir':ranges,
        'acceleration_m_s2':data.sensor('mc6470_acceleration').data.tolist(),
        'magnetic_field_model_units':data.sensor('mc6470_magnetic').data.tolist(),
        'gyroscope_available':False}

if __name__=='__main__':
    import argparse,mujoco
    from run_robot_b3 import Robot
    ap=argparse.ArgumentParser();ap.add_argument('--ramp',action='store_true');ap.add_argument('--walk',action='store_true');ap.add_argument('--seconds',type=float,default=3);a=ap.parse_args()
    r=Robot(json.loads((ROOT/'reference/b3_gait.json').read_text()),a.ramp);samples=[];next_sample=0
    while r.d.time<a.seconds:
        r.step(a.walk)
        if r.d.time>=next_sample:
            samples.append(observe(r.m,r.d));next_sample+=CONFIG['sample_period_s']
    (ROOT/'results/b3_sensor_samples.json').write_text(json.dumps({'ideal_sensor_approximation':True,'samples':samples},indent=2))
    from PIL import Image
    renderer=mujoco.Renderer(r.m,height=240,width=320)
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    renderer.update_scene(r.d,camera='project_camera',scene_option=opt)
    Image.fromarray(renderer.render()).save(ROOT/'results/b3_camera_view.png');renderer.close()
    print(json.dumps(samples[-1],indent=2))
