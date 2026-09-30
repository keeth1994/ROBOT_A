"""Check sensor directions, range validity and absence of a fictitious gyroscope."""
import json,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
import mujoco
from robot_b3.b3_sensors import observe
from robot_b3.paths import ROOT

def main():
    tree=E.parse(ROOT/'models/robot_b3.xml');xml=tree.getroot()
    xml.find('compiler').set('meshdir',str(ROOT/'models/robot_meshes_b3'))
    wb=xml.find('worldbody')
    for sign in [-1,1]:
        E.SubElement(wb,'geom',name=f'test_wall_{sign}',type='box',pos=f'0 {sign*.15} .15',size='.3 .002 .15')
    m=mujoco.MjModel.from_xml_string(E.tostring(xml,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    obs=observe(m,d)
    for name in ['ir_left','ir_right']:
        assert obs['ir'][name]['valid'],obs
        assert abs(obs['ir'][name]['distance_m']-.0764)<1e-5,obs
    ground_site=m.site('ir_ground_site').id
    expected=-d.site_xpos[ground_site,2]/d.site_xmat[ground_site].reshape(3,3)[2,2]
    assert abs(obs['ir']['ir_ground']['distance_m']-expected)<1e-5,obs
    assert not any(m.sensor_type==mujoco.mjtSensor.mjSENS_GYRO)
    # Move only a test wall, not the robot: readings below 4 cm must be invalid.
    m.geom_pos[m.geom('test_wall_1').id,1]=.1;mujoco.mj_forward(m,d)
    assert not observe(m,d)['ir']['ir_left']['valid']
    m.geom_pos[m.geom('test_wall_1').id,1]=.8;mujoco.mj_forward(m,d)
    assert not observe(m,d)['ir']['ir_left']['valid']
    assert m.camera('project_camera').id>=0
    result={'passed':True,'initial_observation':obs,'checks':['left/right wall distances','ground beam direction','near/far invalid range','no gyro','camera present']}
    (ROOT/'results/b3_sensor_test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
