"""Offline sharp-crest comparison, separate from the provisional course assets.

Uses CAD camera mass and removes rubber tread collision geometry because the
physical robot has bare plastic feet. Friction, slope and compliance are assumed.
Contact softness is only a sensitivity test, not a calibrated foam simulation.
Ideal heading/position are used only to isolate gait mechanics in simulation.
No networking, robot commands, or changes to production model files.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import gzip
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as E

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_b91.build import neutral_records
from robot_b91.course import build_course
from robot_b91.gait import rocker_targets
from robot_b91.run import Robot,load_gait_parameters

OUT=ROOT/'results/ramp_trials/crest_wave_sim'


def fmt(v):
    return ' '.join(f'{float(x):.12g}' for x in np.asarray(v).ravel())


def build(slope_deg=25):
    OUT.mkdir(parents=True,exist_ok=True)
    tree=E.parse(ROOT/'models/robot_b10.xml')
    xml=tree.getroot()
    xml.find('compiler').set('meshdir',str(ROOT/'models/robot_meshes_b10'))
    base=mujoco.MjModel.from_xml_path(str(ROOT/'models/robot_b10.xml'))
    data=mujoco.MjData(base)
    data.qpos[2]=0
    mujoco.mj_forward(base,data)
    manifest=json.loads((ROOT/'reference/robot_b10_manifest.json').read_text())
    cfg=json.loads((ROOT/'reference/b10_kinematics.json').read_text())
    with gzip.open(ROOT/'reference/b10_cad_meshes.json.gz','rt') as stream:
        records=neutral_records(json.load(stream)['bodies'],cfg)
    bodies={b.get('name'):b for b in xml.iter('body')}
    entries={}
    for name,b in bodies.items():
        i=base.body(name).id
        rotation=np.zeros(9)
        mujoco.mju_quat2Mat(rotation,base.body_iquat[i])
        rotation=rotation.reshape(3,3)
        inertia=rotation@np.diag(base.body_inertia[i])@rotation.T
        entries[name]=[] if base.body_mass[i]<1e-5 else [(base.body_mass[i],base.body_ipos[i].copy(),inertia)]
    restored_mass=removed_mass=0
    for index,(part,record) in enumerate(zip(manifest['parts'],records,strict=True)):
        assert part['component']==record['component']
        name=part['link']; body_id=base.body(name).id
        rotation=data.xmat[body_id].reshape(3,3)
        com=rotation.T@(np.array(record['com_m'])-data.xpos[body_id])
        if part['camera_assembly']:
            mass=part['reference_mass_kg']
            inertia=rotation.T@(mass*np.array(record['unit_inertia_m2']))@rotation
            entries[name].append((mass,com,inertia))
            restored_mass+=mass
        if part['kind']=='rubber':
            mass=-part['mass_kg']
            inertia=rotation.T@(mass*np.array(record['unit_inertia_m2']))@rotation
            entries[name].append((mass,com,inertia))
            removed_mass-=mass
            for parent in xml.iter():
                for geom in list(parent.findall('geom')):
                    if geom.get('mesh')==f'cad_{index}' or '_tread_' in geom.get('name',''):
                        parent.remove(geom)
    for name,parts in entries.items():
        total=sum(m for m,_,_ in parts)
        com=sum(m*c for m,c,_ in parts)/total
        inertia=np.zeros((3,3))
        for mass,c,I in parts:
            delta=c-com
            inertia+=I+mass*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
        if np.min(np.linalg.eigvalsh(inertia))<=0:
            raise ValueError(f'Non-positive inertia on {name}')
        inertial=bodies[name].find('inertial')
        inertial.attrib.clear()
        inertial.set('mass',fmt(total)); inertial.set('pos',fmt(com))
        inertial.set('fullinertia',fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
    height=.6*math.tan(math.radians(slope_deg))+.01
    course=build_course(xml.find('asset'),xml.find('worldbody'),dict(
        description='User-confirmed sharp first crest, 25-degree assumed slopes; local mechanics experiment',
        width_m=.29,panel_thickness_m=.01,corner_rounding_m=0,
        profile_xz_m=[[0,.01],[.6,height],[1.2,.01]],segment_names=['uphill','downhill'],
        origin_xy_m=[0,0],yaw_deg=0,corridor={'enabled':False}))
    tree.write(OUT/'model.xml',encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(OUT/'model.xml'))
    metadata=dict(course=course,slope_deg=slope_deg,crest_x_m=.6,crest_z_m=height,
                  restored_camera_mass_kg=restored_mass,removed_rubber_mass_kg=removed_mass,
                  total_mass_kg=float(model.body_mass.sum()),
                  assumptions=['Ideal steering isolates mechanics; not a sensor validation',
                               'Slope 25 degrees approximated from physical IMU; dimensions not surveyed',
                               'Friction is swept, not measured; CAD and servo models remain approximate',
                               'Contact softness is a sensitivity test, not measured foam deformation'])
    (OUT/'model.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps({k:v for k,v in metadata.items() if k not in ('course','assumptions')}),flush=True)


def simulate(case,seconds,start_pose):
    name,changes,friction=case
    metadata=json.loads((OUT/'model.json').read_text())
    changes=dict(changes)
    initial_x=changes.pop('initial_x',.58)
    softness=changes.pop('softness',.02)
    speed=changes.pop('speed',.5)
    params=load_gait_parameters()|changes|{'friction':friction}
    params['period']/=.5  # Ramp profile period, before the dashboard 50% speed.
    robot=Robot(params,feedback='ideal',model_path=OUT/'model.xml')
    model,data=robot.m,robot.d
    # Higher-priority terrain parameters override geom mixing. Same contact
    # assumptions apply to feet AND underside. No added adhesion or grip pads.
    # https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters
    for surface in ('uphill','downhill'):
        geom=model.geom(surface).id
        model.geom_priority[geom]=1
        model.geom_solref[geom]=[softness,1]
        model.geom_solimp[geom]=[.9,.95,.001,.5,2]
    slope=math.radians(metadata['slope_deg'])
    if start_pose=='crest':
        data.qpos[:3]=[initial_x,0,metadata['crest_z_m']+.055]
        data.qpos[3:7]=[1,0,0,0]
    else:
        data.qpos[:3]=[.43,0,.01+.43*math.tan(slope)+.047/math.cos(slope)]
        data.qpos[3:7]=[math.cos(slope/2),0,-math.sin(slope/2),0]
    mujoco.mj_forward(model,data)
    maximum_x=float(data.qpos[0]); max_roll=0; max_lateral=0; max_heading=0; contacts=0; frames=0; settled=None
    max_penetration=0.
    started=time.monotonic(); last_log=-1; history=[]
    reason='time limit'
    while data.time<seconds:
        t=data.time-2
        rotation=data.xmat[model.body('platform').id].reshape(3,3)
        heading=math.atan2(rotation[1,0],rotation[0,0])
        roll=math.atan2(rotation[2,1],rotation[2,2])
        if t<=0:
            target=np.tile([0,params['pitch_stance']],4)
        else:
            target=rocker_targets(params,t,t*speed,heading+.01/max(params['heading_gain'],.01),float(data.qpos[1]),robot.joint_ranges)
        robot.step_targets(target)
        if t>=0:
            if settled is None:
                settled=data.qpos[:3].copy()
            max_roll=max(max_roll,abs(math.degrees(roll)))
            max_lateral=max(max_lateral,abs(float(data.qpos[1])))
            max_heading=max(max_heading,abs(math.degrees(heading)))
            maximum_x=max(maximum_x,float(data.qpos[0]))
            frames+=1
            contacts+=int(any(model.geom_bodyid[c.geom1]==model.body('platform').id or
                             model.geom_bodyid[c.geom2]==model.body('platform').id for c in data.contact))
            max_penetration=max(max_penetration,max((-c.dist for c in data.contact),default=0))
            if abs(roll)>math.radians(12) or abs(data.qpos[1])>.04:
                reason='roll or lateral limit'; break
            if data.qpos[0]>.82:
                reason='body passed crest by 22 cm'; break
        if data.time-last_log>=.5:
            history.append(dict(t=float(data.time),xyz=data.qpos[:3].tolist(),roll_deg=math.degrees(roll)))
            last_log=data.time
    result=dict(case=name,friction=friction,start_pose=start_pose,reason=reason,seconds=float(data.time),
                softness_s=softness,speed=speed,initial_x=initial_x,max_penetration_m=max_penetration,
                final_xyz=data.qpos[:3].tolist(),max_x=maximum_x,max_roll_deg=max_roll,
                max_lateral_m=max_lateral,max_heading_deg=max_heading,
                platform_contact_fraction=contacts/max(1,frames),wall_seconds=time.monotonic()-started,
                changes=changes,params=params,warnings=int(sum(w.number for w in data.warning)))
    (OUT/(name+'_'+str(friction)+'_'+start_pose+'.json')).write_text(json.dumps(dict(result=result,history=history,final_qpos=data.qpos.tolist()),indent=2))
    print(json.dumps({k:result[k] for k in ('case','friction','softness_s','reason','seconds','max_x','max_roll_deg','platform_contact_fraction')}),flush=True)
    robot.close()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',action='store_true')
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--start',choices=['crest','incline'],default='crest')
    parser.add_argument('--search',type=int,default=0)
    parser.add_argument('--compare',action='store_true',help='Compare the selected crest wave with the existing diagonal crest gait.')
    parser.add_argument('--validate',type=Path,action='append',help='Saved case JSON; repeat to compare candidates against the existing crest gait.')
    args=parser.parse_args()
    if args.build or not (OUT/'model.xml').exists():
        build()
    current=dict(duty=.85,yaw_amplitude=.75,pitch_lift=.7,pitch_stance=0,yaw_bias=-.25,pattern='trot')
    cases=[('current',current,.8),('wave',current|{'pattern':'crest'},.8)]
    if args.search:
        rng=np.random.default_rng(20261119)
        for i in range(args.search):
            p=dict(yaw_amplitude=float(rng.choice([.25,.35,.55,.75])),
                   pitch_lift=float(rng.choice([.25,.45,.7])),pitch_stance=0,
                   duty=float(rng.choice([.8,.85,.9])),pattern='crest',
                   yaw_bias=float(rng.choice([-.15,-.25,-.4,-.55])))
            cases.append((f'wave_{i:03}',p,.8))
    if args.validate:
        selected=[json.loads(path.read_text())['result'] for path in args.validate]
    elif args.compare:
        selected=[dict(case='crest_wave',changes=dict(pattern='crest',duty=.8,
            yaw_amplitude=.55,pitch_lift=.25,pitch_stance=0,yaw_bias=-.25))]
    if args.validate or args.compare:
        cases=[(f'{name}_x{x:.2f}_soft{soft}',p|{'initial_x':x,'softness':soft},f)
               for name,p in [('control',current)]+[(r['case'],r['changes']) for r in selected]
               for x in (.56,.58,.60) for soft in (.02,.04) for f in (.5,.8,1.1)]
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs=[pool.submit(simulate,case,args.seconds,args.start) for case in cases]
        results=[job.result() for job in jobs]
    (OUT/'latest_results.json').write_text(json.dumps(results,indent=2))
    for label in sorted({r['case'].split('_x')[0] for r in results}):
        group=[r for r in results if r['case'].split('_x')[0]==label]
        # The two-second neutral settling interval precedes any gait motion.
        invalid=sum(r['seconds']<=2.01 and r['reason']=='roll or lateral limit' for r in group)
        print(json.dumps(dict(gait=label,cases=len(group),
            crossed=sum(r['reason']=='body passed crest by 22 cm' for r in group),
            unstable_initial_settling=invalid)),flush=True)


if __name__=='__main__':
    main()
