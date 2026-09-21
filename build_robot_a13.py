"""Convert the actual A9 Fusion mesh export to an articulated MuJoCo model.

Visual geometry is CAD. Contact proxies, mass and motor properties are hypotheses.
Run again after changing reference/robot_parameters.json; never edits Fusion files.
"""
from pathlib import Path
from collections import defaultdict
import gzip, json, math, hashlib
import xml.etree.ElementTree as ET
import numpy as np
from cad_source_a13 import load_cad_source

ROOT = Path(__file__).resolve().parent
LEGS = ['FL', 'RL', 'FR', 'RR']
PORTS = [(115,100,45),(-115,100,135),(115,-100,-45),(-115,-100,-135)]

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
    ref = ROOT/'reference'
    config = ref/'robot_parameters_a13.json'
    if not config.exists():
        config.write_text(json.dumps({
            'effective_print_density_kg_m3':650,
            'metal_density_kg_m3':7800,
            'servo_mass_kg':0.044,
            'wheel_rotating_assembly_mass_kg':0.120,
            'electronics_and_battery_mass_kg':0.200,
            'servo_stall_torque_Nm':0.27,
            'servo_free_speed_deg_s':230,
            'brake_torque_Nm':0.15,
            'ground_friction':0.8,
            'notes':'All unmeasured assumptions. 120 g per rotating wheel is user starting estimate. Print density is an effective bulk estimate, not an infill calculation. Brake is ideal joint friction; no cam/contact validation.'
        },indent=2),encoding='utf8')
    cfg=json.loads(config.read_text(encoding='utf8'))
    raw=ref/'a7_1_cad_meshes.json.gz'
    source=load_cad_source()
    records=source['bodies']
    assert source['source']=='Robot_A13_Kompakt_Hjul.f3d'
    axes=json.loads((ref/'Robot_A9_simulering.json').read_text())['new_hinges']
    axes={a['name']:a for a in axes}
    for name,a in axes.items():
        if name.endswith('_yaw'):a['limits_from_export_pose_rad']=[-math.pi/2,math.pi/2]
    groups=defaultdict(list)
    assignment=[]
    # Each CAD body belongs to exactly one physical link.
    for i,r in enumerate(records):
        c,n=r['component'],r['body']
        prefix=c[:2]
        if prefix not in LEGS: group='platform'
        else:
            tag=c[3:]
            if tag.startswith('01 '): group='platform' if 'FAST' in tag else prefix+'_hip_pitch'
            elif tag.startswith('02 '): group=prefix+('_hip_pitch' if 'FAST' in tag else '_yaw')
            elif tag.startswith('03 '): group=prefix+('_yaw' if 'FAST' in tag else '_distal_pitch')
            elif tag.startswith('05 A7.1'):
                rolling=('hjul D65' in n or 'lagerlokk M3' in n or (n.startswith('608') and 'indre ring' not in n))
                group=prefix+('_wheel_roll' if rolling else '_steering')
            elif tag.startswith('04 BEVART 02 '): group=prefix+'_steering'
            elif tag.startswith('04 BEVART 04 '): group=prefix+'_steering'
            elif tag.startswith('04 BEVART 05 '): group=prefix+'_pinion'
            elif tag.startswith('04 BEVART 11 '): raise ValueError('Hidden spring placeholder must not be exported')
            else: group=prefix+'_distal_pitch'
        groups[group].append((i,r));assignment.append({'index':i,'component':c,'body':n,'link':group})
    expected={'platform'}|{l+'_'+j for l in LEGS for j in ['hip_pitch','yaw','distal_pitch','steering','wheel_roll','pinion']}
    assert set(groups)==expected, (set(groups)^expected)
    assert sum(len(g) for g in groups.values())==len(records)
    xml=ET.Element('mujoco',model='Robot A13 - yaw 180 - CAD prototype')
    ET.SubElement(xml,'compiler',angle='radian',meshdir='robot_meshes_a13',inertiafromgeom='false',autolimits='true')
    ET.SubElement(xml,'option',timestep='0.002',integrator='implicitfast',cone='elliptic',iterations='50')
    visual=ET.SubElement(xml,'visual');ET.SubElement(visual,'global',offwidth='1440',offheight='1000')
    ET.SubElement(visual,'headlight',diffuse='.7 .7 .7',ambient='.35 .35 .35')
    ET.SubElement(xml,'statistic',center='0 0 .19',extent='1.1')
    asset=ET.SubElement(xml,'asset')
    ET.SubElement(asset,'texture',name='grid',type='2d',builtin='checker',width='512',height='512',rgb1='.18 .21 .25',rgb2='.24 .28 .32')
    ET.SubElement(asset,'material',name='floor',texture='grid',texrepeat='8 8',texuniform='true',reflectance='.05')
    wb=ET.SubElement(xml,'worldbody')
    ET.SubElement(wb,'light',pos='0 -1 2',dir='0 0 -1',diffuse='.8 .8 .8')
    ET.SubElement(wb,'geom',name='floor',type='plane',size='3 3 .1',material='floor',contype='1',conaffinity='2',solref='.006 1',solimp='.95 .99 .001',friction=f"{cfg['ground_friction']} .002 .0001")
    root=ET.SubElement(wb,'body',name='platform',pos='0 0 .1915')
    ET.SubElement(root,'freejoint',name='floating_base')
    ET.SubElement(root,'site',name='imu',pos='0 0 .1',size='.004',rgba='0 .8 .8 1',group='4')
    bodies={'platform':root};origins={'platform':np.zeros(3)}
    joint_info=[]
    for leg,(px,py,angle) in zip(LEGS,PORTS):
        a=math.radians(angle);rot=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        def world(p):return rot@np.array(p)+np.array([px/1000,py/1000,0])
        parent='platform'
        for part in ['hip_pitch','yaw','distal_pitch','steering','wheel_roll']:
            name=leg+'_'+part
            if name in axes:
                ja=axes[name];origin=np.array(ja['origin_world_m']);axis=np.array(ja['axis_world']);limits=ja['limits_from_export_pose_rad']
            elif part=='steering':origin=world([.241,0,-.106]);axis=np.array([0,0,1]);limits=[-math.pi/2,math.pi/2]
            else:origin=world([.241,0,-.1555]);axis=rot@np.array([0,1,0]);limits=None
            b=ET.SubElement(bodies[parent],'body',name=name,pos=fmt(origin-origins[parent]))
            kwargs=dict(name=name,type='hinge',axis=fmt(axis),damping='0.00002' if part=='wheel_roll' else '0.012',armature='0.000005')
            if limits is not None:kwargs.update(range=fmt(limits),limited='true')
            else:kwargs.update(limited='false',frictionloss='.0002')
            ET.SubElement(b,'joint',**kwargs)
            bodies[name]=b;origins[name]=origin
            joint_info.append({'name':name,'parent':parent,'origin_cad_m':origin.tolist(),'axis':axis.tolist(),'limits_rad':limits,'actuated':part!='wheel_roll'})
            parent=name
        # 40T/40T steering pinion: coupled opposite to the swivel, preserving the CAD mesh motion.
        name=leg+'_pinion';parent=leg+'_distal_pitch'
        pin=next(r for _,r in groups[name] if 'Servohjul 40T' in r['body'])
        origin=np.array(pin['com_m'])
        bodies[name]=ET.SubElement(bodies[parent],'body',name=name,pos=fmt(origin-origins[parent]));origins[name]=origin
        ET.SubElement(bodies[name],'joint',name=name,type='hinge',axis='0 0 1',limited='false',armature='.000001',damping='.0001')
    equality=ET.SubElement(xml,'equality')
    for l in LEGS:ET.SubElement(equality,'joint',joint1=l+'_pinion',joint2=l+'_steering',polycoef='0 -1 0 0 0')
    folder=ROOT/'models/robot_meshes_a13';folder.mkdir(parents=True,exist_ok=True)
    # Approximate inertias from each CAD part's mass and bounding box about its CAD COM.
    # Explicitly not deriving mass from Fusion's decorative plastic appearances.
    part_mass={}
    electronics=[];servos=[]
    for i,r in enumerate(records):
        n,c=r['body'],r['component']
        metal=any(k in n.lower() for k in ['metall','staal','stål','608','61805','lagerkule','skive','endeskrue','styrepinne','51101'])
        part_mass[i]=max(1e-7,r['volume_m3']*(cfg['metal_density_kg_m3'] if metal else cfg['effective_print_density_kg_m3']))
        if 'Parallax' in n:part_mass[i]=cfg['servo_mass_kg'];servos.append(i)
        if n=='A7 fremre lager 25x37x7':part_mass[i]=cfg['bearing_25_37_7_mass_kg']
        if n=='A7 bakre lager 8x16x5':part_mass[i]=cfg['bearing_8_16_5_mass_kg']
        if c.startswith(('Plattform E','Plattform S','Plattform A9 IR','Plattform A9 S06')):electronics.append(i)
    emass=sum(part_mass[i] for i in electronics)
    for i in electronics:part_mass[i]*=cfg['electronics_and_battery_mass_kg']/emass
    for leg in LEGS:
        g=groups[leg+'_wheel_roll'];m=sum(part_mass[i] for i,r in g)
        for i,r in g:part_mass[i]*=cfg['wheel_rotating_assembly_mass_kg']/m
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
        ET.SubElement(bodies[group],'geom',name=name,type=kind,contype='2',conaffinity='3',group='3',rgba='.1 .8 .6 .35',mass='0',solref='.006 1',solimp='.95 .99 .001',friction=f"{cfg['ground_friction']} .002 .0001",**kw)
    outline=[(.155,.06),(.075,.14),(-.075,.14),(-.155,.06),(-.155,-.06),(-.075,-.14),(.075,-.14),(.155,-.06)]
    vertices=[[x,y,z] for z in [0,.083] for x,y in outline]
    ET.SubElement(asset,'mesh',name='platform_collision_hull',vertex=fmt(np.array(vertices).ravel()))
    proxy('platform','platform_contact','mesh',mesh='platform_collision_hull')
    for leg,(px,py,angle) in zip(LEGS,PORTS):
        h,y,d,s,w=[origins[leg+'_'+p] for p in ['hip_pitch','yaw','distal_pitch','steering','wheel_roll']]
        for p,a,b,r in [('hip_pitch',h,y,.028),('yaw',y,d,.028),('distal_pitch',d,s,.026)]:
            end=b+np.array([0,0,.026]) if p=='distal_pitch' else b
            proxy(leg+'_'+p,leg+'_'+p+'_contact','capsule',fromto=fmt(np.r_[a-origins[leg+'_'+p],end-origins[leg+'_'+p]]),size=fmt(r))
        a=math.radians(angle);axis=np.array([-math.sin(a),math.cos(a),0])
        for side in [-1,1]:
            top=s+side*.025*axis;bottom=w+side*.025*axis
            proxy(leg+'_steering',leg+'_fork_'+str(side),'capsule',fromto=fmt(np.r_[top-s,bottom-s]),size='.008')
        proxy(leg+'_wheel_roll',leg+'_wheel_contact','cylinder',size='.0325 .015',zaxis=fmt(axis))
        ET.SubElement(bodies[leg+'_wheel_roll'],'site',name=leg+'_wheel_center',size='.004',group='4',rgba='1 .4 .1 1')
    # Do not collide internal steering gears or bearing races; CAD meshes are visual only.
    actuator=ET.SubElement(xml,'actuator')
    for j in joint_info:
        if j['actuated']:ET.SubElement(actuator,'motor',name=j['name'],joint=j['name'],gear='1',ctrllimited='true',ctrlrange=fmt([-cfg['servo_stall_torque_Nm'],cfg['servo_stall_torque_Nm']]))
    sensor=ET.SubElement(xml,'sensor')
    ET.SubElement(sensor,'accelerometer',name='ideal_accelerometer',site='imu')
    ET.SubElement(sensor,'gyro',name='ideal_gyro',site='imu')
    ET.SubElement(sensor,'framequat',name='ideal_orientation',objtype='site',objname='imu')
    for j in joint_info:
        ET.SubElement(sensor,'jointpos',name=j['name']+'_q',joint=j['name'])
        ET.SubElement(sensor,'jointvel',name=j['name']+'_dq',joint=j['name'])
    ET.indent(xml)
    target=ROOT/'models/robot_a13.xml';ET.ElementTree(xml).write(target,encoding='utf-8',xml_declaration=True)
    report={'source':source['source'],'cad_body_count':len(records),'servo_envelopes':len(servos),'total_mass_assumed_kg':sum(x['mass_kg'] for x in audit),'joints':joint_info,'links':audit,'assignments':assignment,'assumptions':cfg,'limitations':['CAD visual meshes simplified with a 0.25 mm vertex grid; not manufacturing geometry','Bounding-box inertias; no calibrated mass properties','Capsule/box/cylinder collision proxies; not CAD interference validation','Ideal IMU and joint feedback, not evidence of sensors on real servos','Brake friction torque surrogate, no cam deformation, spring or pad model','Steering 40T/40T ideal joint equality; no backlash','No walking/skating policy yet']}
    report['source_base_sha256']=source['base_sha256'];report['source_patch_sha256']=source['patch_sha256'];report['cad_com_validation_error_m']=source['a9_com_error_m']
    (ref/'robot_a13_manifest.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps({k:report[k] for k in ['cad_body_count','servo_envelopes','total_mass_assumed_kg']},indent=2))

if __name__=='__main__':main()
