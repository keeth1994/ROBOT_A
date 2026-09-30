"""Validate plank heights, the valley and editable corridor/ramp placement."""
from pathlib import Path
import copy,json,math
import xml.etree.ElementTree as E
import numpy as np
import mujoco
from robot_b3.course_b3 import build_course,transform,local_points
from robot_b3.paths import ROOT

def check(cfg):
    xml=E.parse(ROOT/'models/robot_b3.xml').getroot()
    xml.find('compiler').set('meshdir',str(ROOT/'models/robot_meshes_b3'))
    info=build_course(xml.find('asset'),xml.find('worldbody'),cfg)
    model=mujoco.MjModel.from_xml_string(E.tostring(xml,encoding='unicode'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    groups=np.array([1,0,0,0,0,0],dtype=np.uint8)
    def ray(point,direction):
        gid=np.array([-1],dtype=np.int32)
        distance=mujoco.mj_ray(model,data,np.array(point,dtype=float),np.array(direction,dtype=float),groups,1,-1,gid)
        assert distance>=0,'Ray missed terrain'
        return distance
    profile=np.array(cfg['profile_xz_m'])
    for x,z in list(profile)+list((profile[:-1]+profile[1:])/2):
        point=transform([x,0,1],cfg['origin_xy_m'],cfg['yaw_deg'])
        assert abs((1-ray(point,[0,0,-1]))-z)<1e-5,(x,z)
        assert np.allclose(local_points(point,info),[x,0,1])
    c=cfg['corridor'];pts=np.array(c['centerline_xy_m']);mid=(pts[0]+pts[1])/2
    tangent=pts[1]-pts[0];tangent/=np.linalg.norm(tangent);normal=np.array([-tangent[1],tangent[0]])
    origin=transform([*mid,c['wall_height_m']/2],c['origin_xy_m'],c['yaw_deg'])
    for sign in [-1,1]:
        direction=transform([*(sign*normal),0],[0,0],c['yaw_deg'])
        assert abs(ray(origin,direction)-c['clear_width_m']/2)<1e-6
    return info['segments']

if __name__=='__main__':
    cfg=json.loads((ROOT/'reference/b3_course_settings.json').read_text());segments=check(cfg)
    moved=copy.deepcopy(cfg);moved['origin_xy_m']=[.6,1.0];moved['yaw_deg']=23;moved['corridor']['origin_xy_m']=[3,-2];moved['corridor']['yaw_deg']=17;moved['corridor']['clear_width_m']=.5
    check(moved)
    result={'passed':True,'checks':['all profile vertices and section midpoints','valley not bridged by collision geometry','wall clear width','independent translation and rotation','wider corridor'],'segments':segments}
    (ROOT/'results/b3_terrain_test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
