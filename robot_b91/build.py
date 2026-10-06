"""Build B9-1 visuals, CAD-derived segmented collision hulls and explicit mass estimates."""
from pathlib import Path
from collections import defaultdict
import gzip,json,hashlib,math
import numpy as np
from scipy.spatial import ConvexHull,QhullError
import xml.etree.ElementTree as E
from robot_b91.paths import ROOT
from robot_b91.mesh_utils import fmt,visual_mesh
from robot_b91.geometry import rz
from robot_b91.course import build_course
LEGS=['FL','RL','FR','RR']

def classify(n):
    if 'servo' in n.lower() and 'SERVO2' not in n:return 'servo'
    if 'battery' in n:return 'battery'
    if 'sensor ir_' in n:return 'ir'
    if 'sensor camera' in n:return 'camera'
    if 'IMU reservation' in n:return 'imu'
    if 'SERVO2' in n:return 'driver_board'
    if 'ESP32' in n:return 'esp32'
    if 'DFR0753' in n:return 'regulator'
    if 'rubber tread' in n:return 'rubber'
    if 'bearing 6x' in n:return 'bearing6'
    if 'bearing 4x' in n:return 'bearing4'
    if any(k in n for k in ['steel axle','M4','washer']):return 'steel'
    if any(k in n for k in ['original yaw cross','pitch horn']):return 'horn'
    return 'PLA'

def group(n):
    l=n[:2]
    if l not in LEGS:return 'platform'
    if any(k in n for k in ['yaw driver','original yaw cross']):return l+'_driver'
    if any(k in n for k in ['rocker foot','rubber tread','pitch servo','pitch steel axle','pitch M4','pitch inner clamp']):return l+'_pitch'
    if any(k in n for k in ['yaw turret','pitch horn','pitch bearing','pitch inner ring','yaw steel axle']):return l+'_yaw'
    return 'platform'

def split_polygons(polys,axis,cut):
    sides=[[],[]]
    for poly in polys:
        for side in range(2):
            out=[]
            for a,b in zip(poly,np.roll(poly,-1,axis=0)):
                ia=(a[axis]<=cut) if side==0 else (a[axis]>=cut)
                ib=(b[axis]<=cut) if side==0 else (b[axis]>=cut)
                if ia:out.append(a)
                if ia!=ib:out.append(a+(b-a)*((cut-a[axis])/(b[axis]-a[axis])))
            if len(out)>=3:sides[side].append(np.array(out))
    return sides

def hulls(v,f,steps):
    # Split actual CAD surface triangles into cells before taking each convex hull.
    cells=[[v[t] for t in f]]
    for axis,step in enumerate(steps):
        if step is None:continue
        for cut in np.arange(v[:,axis].min()+step,v[:,axis].max()-1e-7,step):
            new=[]
            for polys in cells:
                points=np.concatenate(polys)
                if points[:,axis].min()<cut-1e-9 and points[:,axis].max()>cut+1e-9:
                    new.extend(s for s in split_polygons(polys,axis,cut) if s)
                else:new.append(polys)
            cells=new
    for polys in cells:
        points=np.unique(np.round(np.concatenate(polys),9),axis=0)
        if len(points)<4:continue
        try:h=ConvexHull(points)
        except QhullError:continue
        if h.volume>1e-13:yield points[h.vertices]

def main():
    ref=ROOT/'reference';source=ref/'b91_cad_meshes.json.gz'
    with gzip.open(source,'rt') as f:export=json.load(f)
    assert hashlib.sha256((ROOT/export['source']).read_bytes()).hexdigest()==export['source_sha256'],'CAD archive changed; export it again'
    records=export['bodies'];cfg=json.loads((ref/'b91_kinematics.json').read_text());masscfg=json.loads((ref/'b91_mass.json').read_text())
    xml=E.Element('mujoco',model='B9-1 CAD transfer');E.SubElement(xml,'compiler',angle='radian',meshdir='robot_meshes_b91',inertiafromgeom='false',autolimits='true')
    E.SubElement(xml,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='60')
    vis=E.SubElement(xml,'visual');E.SubElement(vis,'global',offwidth='1280',offheight='900');E.SubElement(vis,'headlight',ambient='.45 .45 .45')
    asset=E.SubElement(xml,'asset');wb=E.SubElement(xml,'worldbody');act=E.SubElement(xml,'actuator');eq=E.SubElement(xml,'equality')
    E.SubElement(asset,'texture',name='checker',type='2d',builtin='checker',rgb1='.85 .85 .85',rgb2='.12 .14 .17',width='512',height='512');E.SubElement(asset,'material',name='checker_floor',texture='checker',texrepeat='5 5',texuniform='true')
    E.SubElement(wb,'light',pos='0 -1 2',dir='0 0 -1');E.SubElement(wb,'geom',name='floor',type='plane',size='10 10 .1',material='checker_floor',contype='1',conaffinity='2',friction='.8 .002 .0001')
    allv=np.concatenate([r['vertices'] for r in records]);height=-allv[:,2].min()+.002
    root=E.SubElement(wb,'body',name='platform',pos=fmt([0,0,height]));E.SubElement(root,'freejoint')
    bodies={'platform':root};origins={'platform':np.zeros(3)};rots={}
    for l,k in cfg['legs'].items():
        R=rz(k['neutral_yaw_deg']);rots[l]=R;y=np.array(k['yaw_center_mm'])/1000;p=y+R@np.array(k['pitch_local_mm'])/1000;d=y+R@np.array([-.025,0,0])
        for suffix,origin,parent,axis,limits in [('yaw',y,'platform',[0,0,1],[-45,45]),('pitch',p,l+'_yaw',R@np.array([0,1,0]),[-45,45]),('driver',d,'platform',[0,0,1],[-90,90])]:
            n=l+'_'+suffix;origins[n]=origin;bodies[n]=E.SubElement(bodies[parent],'body',name=n,pos=fmt(origin-origins[parent]));E.SubElement(bodies[n],'joint',name=n,axis=fmt(axis),range=fmt(np.radians(limits)),damping='.001',armature='.000001')
            if suffix!='driver':
                limit=cfg['servo_stall_Nm']*(cfg['gear_efficiency_assumed']/cfg['gear_ratio_speed'] if suffix=='yaw' else 1)
                E.SubElement(act,'motor',name=n,joint=n,ctrlrange=fmt([-limit,limit]))
        E.SubElement(eq,'joint',joint1=l+'_driver',joint2=l+'_yaw',polycoef=fmt([0,-1/cfg['gear_ratio_speed'],0,0,0]),solref='.004 1')
    folder=ROOT/'models/robot_meshes_b91';folder.mkdir(parents=True,exist_ok=True)
    groups=defaultdict(list);audit=[];collision_count=0;hull_cache={}
    colors={'PLA':'.75 .8 .87 1','servo':'.08 .09 .11 1','battery':'.9 .65 .1 1','rubber':'.06 .065 .07 1','horn':'.9 .65 .1 1'}
    for i,r in enumerate(records):
        n=r['component'];g=group(n);kind=classify(n);v=np.array(r['vertices']);f=np.array(r['indices']).reshape(-1,3)
        mass=masscfg['fixed_mass_kg'].get(kind,r['volume_m3']*masscfg['density_kg_m3'].get(kind,1240));groups[g].append((mass,np.array(r['com_m']),np.ptp(v,axis=0)))
        audit.append(dict(component=n,link=g,kind=kind,mass_kg=mass,volume_m3=r['volume_m3']))
        vv,ff=visual_mesh(r,origins[g]);name='cad_'+str(i);path=folder/(name+'.obj')
        with path.open('w') as out:
            for a in vv:out.write('v '+fmt(a)+'\n')
            for a in ff:out.write('f '+' '.join(str(int(x)+1) for x in a)+'\n')
        E.SubElement(asset,'mesh',name=name,file=path.name,inertia='shell',maxhullvert='64');E.SubElement(bodies[g],'geom',type='mesh',mesh=name,rgba=colors.get(kind,'.2 .5 .3 1'),contype='0',conaffinity='0',group='1',mass='0')
        # Small fasteners/bearing interiors omitted from collision, retained in mass and visuals.
        if kind in ['steel','horn','bearing4','bearing6'] or any(k in n for k in ['spacer','spacing tube','yaw driver']):continue
        R=rots.get(n[:2],np.eye(3));offset=origins.get(n[:2]+'_yaw',np.zeros(3))
        # 0.5 mm vertex clustering bounds the collision surface simplification.
        cells=np.floor(v/.0005).astype(np.int64);_,first,inverse=np.unique(cells,axis=0,return_index=True,return_inverse=True)
        cv=v[first];cf=inverse[f];cf=cf[(cf[:,0]!=cf[:,1])&(cf[:,1]!=cf[:,2])&(cf[:,0]!=cf[:,2])]
        _,keep=np.unique(np.sort(cf,axis=1),axis=0,return_index=True);cf=cf[keep];local=(cv-offset)@R
        steps=(.004,None,None) if kind=='rubber' else ((.016,.016,.020) if kind=='PLA' else (None,None,None))
        key=n[3:] if n[:2] in LEGS else n
        if key not in hull_cache:hull_cache[key]=list(hulls(local,cf,steps))
        for j,h in enumerate(hull_cache[key]):
            name=(n[:2]+'_tread_' if kind=='rubber' else 'collision_')+str(i)+'_'+str(j)
            E.SubElement(asset,'mesh',name=name,vertex=fmt(((h@R.T)+offset-origins[g]).ravel()),inertia='shell')
            E.SubElement(bodies[g],'geom',name=name,type='mesh',mesh=name,mass='0',group='3',rgba='.1 .8 .6 .3',contype='2',conaffinity='3',friction='.8 .002 .0001',solref='.006 1');collision_count+=1
    groups['platform'].append((masscfg['extra_wiring_fasteners_kg'],np.array([0,0,.025]),np.array([.14,.14,.05])))
    for g,entries in groups.items():
        total=sum(m for m,c,s in entries);com=sum(m*c for m,c,s in entries)/total;I=np.zeros((3,3))
        for m,c,s in entries:
            delta=c-com;I+=np.diag(m/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2]))+m*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
        E.SubElement(bodies[g],'inertial',mass=fmt(total),pos=fmt(com-origins[g]),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
    # Sensor locations follow the B9-1 component bounds; IR axes inherited from mounting orientation.
    sensors=E.SubElement(xml,'sensor');sc=json.loads((ref/'b91_sensor_axes.json').read_text())
    def center(key):
        rr=next(r for r in records if key in r['component']);v=np.array(rr['vertices']);return (v.min(0)+v.max(0))/2
    E.SubElement(root,'site',name='imu_site',pos=fmt(center('IMU reservation')),size='.001');E.SubElement(sensors,'accelerometer',name='imu_acceleration',site='imu_site')
    for s in sc['ir']:
        ax,ay,az=np.array(s['axes']);E.SubElement(root,'site',name=s['name']+'_site',pos=fmt(center('sensor '+s['name'])+ay*.0066),xyaxes=fmt(np.r_[ax,-az]),size='.001');E.SubElement(sensors,'rangefinder',name=s['name'],site=s['name']+'_site')
    E.SubElement(root,'camera',name='project_camera',pos=fmt(center('sensor camera')+[.0076,0,0]),xyaxes='0 -1 0 0 0 1',fovy='43')
    E.indent(xml);E.ElementTree(xml).write(ROOT/'models/robot_b91.xml',encoding='utf-8',xml_declaration=True)
    terrain=build_course(asset,wb,json.loads((ref/'b91_course_settings.json').read_text()));(ref/'b91_course.json').write_text(json.dumps(terrain,indent=2));E.indent(xml);E.ElementTree(xml).write(ROOT/'models/robot_b91_course.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(revision='B9-1',source_sha256=export['source_sha256'],cad_bodies=len(records),collision_hulls=collision_count,total_mass_kg=sum(x['mass_kg'] for x in audit)+masscfg['extra_wiring_fasteners_kg'],neutral_bounds_m=np.ptp(allv,axis=0).tolist(),parts=audit,limitations=['Solid-material CAD mass plus provisional component allowances; not measured','Bounding-box inertia approximation','Segmented convex CAD hulls after 0.5 mm vertex clustering; small holes/fasteners omitted','Original servo torque/speed model, 85% gear efficiency; no backlash/thermal model','Yaw conservatively limited to +/-45 degrees; larger range not cleared','Rigid terrain and assumed 0.8 friction','IR rays and camera pose approximate; IMU exposes acceleration only pending exact board confirmation','No learned policy is validated for this model'])
    (ref/'robot_b91_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({k:v for k,v in manifest.items() if k!='parts'},indent=2))
if __name__=='__main__':main()

