"""A17 CAD geometry -> eight-axis MuJoCo prototype. Mass/contact assumptions explicit."""
from pathlib import Path
from collections import defaultdict
import gzip,json,math,re
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parent
LEGS=['FL','RL','FR','RR']
def fmt(x):
    if np.isscalar(x): return f'{x:.9g}'
    return ' '.join(f'{v:.9g}' for v in x)

def visual_mesh(record,origin):
    """Cluster overly dense tessellation per CAD solid, retaining real vertices.

    0.25 mm grid. Contacts/inertia never use this simplified visual geometry.
    Every retained vertex is from CAD; collapsed/duplicate faces are discarded.
    """
    v=np.array(record['vertices'],dtype=np.float64)-origin
    f=np.array(record['indices'],dtype=np.int64).reshape(-1,3)
    if len(v)>1200:
        cells=np.floor(v/.00025).astype(np.int64)
        _,first,inverse=np.unique(cells,axis=0,return_index=True,return_inverse=True)
        v=v[first];f=inverse[f]
        f=f[(f[:,0]!=f[:,1])&(f[:,0]!=f[:,2])&(f[:,1]!=f[:,2])]
        _,keep=np.unique(np.sort(f,axis=1),axis=0,return_index=True);f=f[keep]
        cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        f=f[np.linalg.norm(cross,axis=1)>1e-14]
        used=np.unique(f);mapping=np.full(len(v),-1,dtype=np.int64);mapping[used]=np.arange(len(used))
        v=v[used];f=mapping[f]
    return v,f

def main():
    ref=ROOT/'reference'
    cfg=json.loads((ref/'robot_parameters_a16.json').read_text())
    cfg.update(hip_servo_count=0,notes='A17: 8 single servos; old parts use matched A16 slicer mass. New hollow feet use conservative solid PLA density. Electronics/battery 200g unmeasured. Inertias approximate. Servo torque and speed optimistic instantaneous ceilings; no thermal or supply sag model.')
    with gzip.open(ref/'a17_cad_meshes.json.gz','rt') as f:source=json.load(f)
    records=source['bodies']
    assert source['source']=='Robot_A17_Yaw_Pitch.f3d'
    groups=defaultdict(list);assignment=[]
    for i,r in enumerate(records):
        c=r['component'];leg=c[:2]
        if leg not in LEGS or ('01 YAW' in c and 'FAST' in c):group='platform'
        elif '01 YAW' in c or ('02 PITCH' in c and 'FAST' in c):group=leg+'_yaw'
        else:group=leg+'_pitch'
        groups[group].append((i,r));assignment.append(dict(index=i,component=c,body=r['body'],link=group))
    xml=ET.Element('mujoco',model='A17 yaw pitch quadruped')
    ET.SubElement(xml,'compiler',angle='radian',meshdir='robot_meshes_a17',inertiafromgeom='false',autolimits='true')
    ET.SubElement(xml,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='50')
    vis=ET.SubElement(xml,'visual');ET.SubElement(vis,'global',offwidth='1280',offheight='900')
    ET.SubElement(vis,'headlight',ambient='.4 .4 .4',diffuse='.7 .7 .7')
    ET.SubElement(xml,'statistic',center='0 0 .15',extent='.9')
    asset=ET.SubElement(xml,'asset')
    ET.SubElement(asset,'texture',name='grid',type='2d',builtin='checker',width='512',height='512',rgb1='.15 .18 .22',rgb2='.23 .27 .3')
    ET.SubElement(asset,'material',name='floor_mat',texture='grid',texrepeat='15 15',texuniform='true')
    wb=ET.SubElement(xml,'worldbody');ET.SubElement(wb,'light',pos='0 -1 2',dir='0 0 -1')
    ET.SubElement(wb,'geom',name='floor',type='plane',size='20 20 .1',material='floor_mat',contype='1',conaffinity='2',friction='.8 .002 .0001')
    root=ET.SubElement(wb,'body',name='platform',pos='0 0 .114')
    ET.SubElement(root,'freejoint',name='floating_base')
    ET.SubElement(root,'site',name='imu',pos='0 0 .09',size='.003',group='4')
    bodies={'platform':root};origins={'platform':np.zeros(3)};joint_info=[]
    kin=json.loads((ref/'A17_kinematic_reference.json').read_text())['legs']
    for leg in LEGS:
        parent='platform'
        for part in ['yaw','pitch']:
            name=leg+'_'+part;origin=np.array(kin[leg][part+'_axis_origin_mm'])/1000;axis=kin[leg][part+'_axis']
            limits=[-math.pi/2,math.pi/2] if part=='yaw' else [0,math.pi/2]
            b=ET.SubElement(bodies[parent],'body',name=name,pos=fmt(origin-origins[parent]))
            ET.SubElement(b,'joint',name=name,type='hinge',axis=fmt(axis),range=fmt(limits),damping='.012',armature='.000005')
            bodies[name]=b;origins[name]=origin
            joint_info.append(dict(name=name,parent=parent,origin_cad_m=origin.tolist(),axis=axis,limits_rad=limits))
            parent=name
    folder=ROOT/'models/robot_meshes_a17';folder.mkdir(parents=True,exist_ok=True)
    part_mass={}
    electronics=[];servos=[]
    for i,r in enumerate(records):
        n,c=r['body'],r['component']
        metal=any(k in n.lower() for k in ['metall','staal','stael','stål','608','61805','lagerkule','skive','endeskrue','styrepinne','51101'])
        part_mass[i]=max(1e-7,r['volume_m3']*(cfg['metal_density_kg_m3'] if metal else cfg['effective_print_density_kg_m3']))
        if 'rubber foot pad' in n:part_mass[i]=r['volume_m3']*1100
        if 'hollow shank' in n:part_mass[i]=r['volume_m3']*1240
        if 'Parallax' in n:part_mass[i]=cfg['servo_mass_kg'];servos.append(i)
        if n=='A7 fremre lager 25x37x7':part_mass[i]=cfg['bearing_25_37_7_mass_kg']
        if n=='A7 bakre lager 8x16x5':part_mass[i]=cfg['bearing_8_16_5_mass_kg']
        if c.startswith(('Plattform E','Plattform S','Plattform A16 E')) or ('A16' in c and ('fixed hip optical' in c or 'camera package' in c)):electronics.append(i)
    emass=sum(part_mass[i] for i in electronics)
    for i in electronics:part_mass[i]*=cfg['electronics_and_battery_mass_kg']/emass
    import re
    rules=json.loads((ref/'a16_print_mass_rules.json').read_text())
    matched=0
    for i,r in enumerate(records):
        n=re.sub(r' \(\d+\)$','',r['body'])
        candidates=[x for x in rules if x['name']==n and abs(x['volume_m3']-r['volume_m3'])<1e-10]
        if candidates:
            part_mass[i]=candidates[0]['mass_kg'];matched+=1
    assert len(servos)==8
    print("Slicer-matched parts:",matched)
    colors={'White':(.78,.8,.83,1),'Gray':(.42,.47,.53,1),'Black':(.065,.075,.09,1),'Yellow':(.88,.57,.10,1),'Green':(.12,.55,.32,1),'Blue':(.13,.38,.75,1),'Red':(.8,.16,.1,1)}
    audit=[]
    for group,parts in groups.items():
        total=sum(part_mass[i] for i,r in parts)
        com=sum(part_mass[i]*np.array(r['com_m']) for i,r in parts)/total
        inertia=np.zeros((3,3));bycolor=defaultdict(list)
        for i,r in parts:
            vs=np.array(r['vertices']);size=np.ptp(vs,axis=0);mass=part_mass[i]
            diag=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])
            delta=np.array(r['com_m'])-com
            inertia+=np.diag(np.maximum(diag,1e-12))+mass*((delta@delta)*np.eye(3)-np.outer(delta,delta))
            color=next((k for k in colors if k.lower() in r['appearance'].lower()),'Gray')
            bycolor[color].append(r)
        ET.SubElement(bodies[group],'inertial',mass=fmt(total),pos=fmt(com-origins[group]),fullinertia=fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
        for color,rs in bycolor.items():
            vertices=[];faces=[]
            for r in rs:
                v,f=visual_mesh(r,origins[group]);f=f+len(vertices)
                vertices.extend(v.tolist());faces.extend(f.tolist())
            meshname=group+'_'+color;path=folder/(meshname+'.obj')
            with path.open('w',encoding='ascii') as f:
                f.write('# CAD visual only; metres; body-local coordinates\n')
                for v in vertices:f.write('v '+fmt(v)+'\n')
                for face in faces:f.write('f '+' '.join(str(x+1) for x in face)+'\n')
            ET.SubElement(asset,'mesh',name=meshname,file=path.name,inertia='shell',maxhullvert='32')
            ET.SubElement(bodies[group],'geom',name='visual_'+meshname,type='mesh',mesh=meshname,rgba=fmt(colors[color]),contype='0',conaffinity='0',group='1',mass='0')
        audit.append({'link':group,'parts':len(parts),'mass_kg':total,'com_cad_m':com.tolist(),'inertia_approx_kg_m2':inertia.tolist()})
    def proxy(group,name,kind,**kw):
        ET.SubElement(bodies[group],'geom',name=name,type=kind,contype='2',conaffinity='3',group='3',rgba='.1 .8 .6 .35',mass='0',friction='.8 .002 .0001',solref='.006 1',**kw)
    proxy('platform','centre_contact','box',pos='0 0 .042',size='.061 .061 .045')
    proxy('platform','battery_contact','box',pos='0 0 -.0115',size='.038 .022 .0075')
    for leg in LEGS:
        y=origins[leg+'_yaw'];p=origins[leg+'_pitch'];f=np.array(kin[leg]['foot_contact_mm'])/1000
        proxy('platform',leg+'_yaw_fixed','cylinder',pos=fmt(y-np.array([0,0,.01])),size='.028 .033')
        proxy(leg+'_yaw',leg+'_upper_contact','capsule',fromto=fmt(np.r_[np.zeros(3),p-y]),size='.025')
        proxy(leg+'_pitch',leg+'_shank_contact','capsule',fromto=fmt(np.r_[np.array([0,0,-.047]),f-p+[0,0,.020]]),size='.012')
        proxy(leg+'_pitch',leg+'_foot_contact','cylinder',pos=fmt(f-p+[0,0,.0045]),size='.015 .0045')
        ET.SubElement(bodies[leg+'_pitch'],'site',name=leg+'_foot',pos=fmt(f-p),size='.003',group='4')
    act=ET.SubElement(xml,'actuator')
    for j in joint_info:ET.SubElement(act,'motor',name=j['name'],joint=j['name'],gear='1',ctrlrange=fmt([-cfg['servo_stall_torque_Nm'],cfg['servo_stall_torque_Nm']]))
    sens=ET.SubElement(xml,'sensor');ET.SubElement(sens,'gyro',site='imu',name='gyro');ET.SubElement(sens,'accelerometer',site='imu',name='accel')
    ET.indent(xml);ET.ElementTree(xml).write(ROOT/'models/robot_a17.xml',encoding='utf-8',xml_declaration=True)
    report=dict(source=source['source'],cad_body_count=len(records),servo_envelopes=len(servos),total_mass_assumed_kg=sum(part_mass.values()),joints=joint_info,links=audit,assignments=assignment,assumptions=cfg,limitations=['CAD visuals; simplified collision proxies','Bounding-box approximate inertia','No physical load, servo travel or battery validation','Open-loop gait is not reinforcement learning'])
    (ref/'robot_a17_manifest.json').write_text(json.dumps(report,indent=2))
    print('A17 mass kg:',report['total_mass_assumed_kg'])
if __name__=='__main__':main()
