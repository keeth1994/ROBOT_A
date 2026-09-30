"""B3 CAD export -> MuJoCo prototype. Conservative solid PLA mass; approximate inertia."""
from pathlib import Path
from collections import defaultdict
import json,gzip,math
import numpy as np
import xml.etree.ElementTree as E
from mesh_utils import visual_mesh,fmt
ROOT=Path(__file__).resolve().parent
LEGS=['FL','RL','FR','RR']
def rz(a):
 a=math.radians(a);return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
def main():
 sensor_cfg=json.loads((ROOT/'reference/b3_sensors.json').read_text())
 foot_cfg=json.loads((ROOT/'reference/b3_foot_geometry.json').read_text())
 ref=ROOT/'reference';cfg=json.loads((ref/'b3_kinematics.json').read_text())
 with gzip.open(ref/'b3_cad_meshes.json.gz','rt') as f:records=json.load(f)['bodies']
 groups=defaultdict(list);mass={};totals=defaultdict(float)
 for i,r in enumerate(records):
  c=r['component'];l=c[:2];k=r['kind']
  group='platform'
  if l in LEGS:
   if 'yaw driver' in c:group=l+'_driver'
   elif any(x in c for x in ['rocker foot','rubber tread','pitch servo','pitch pin']):group=l+'_pitch'
   elif any(x in c for x in ['yaw turret','pitch bushing','pitch horn']):group=l+'_yaw'
  groups[group].append((i,r))
  mass[i]={'servo':.044,'battery':.180,'electronics':.090,'imu':sensor_cfg['imu']['mass_kg'],'ir_sensor':sensor_cfg['ir_mass_kg'],'camera_sensor':sensor_cfg['camera']['mass_kg'],'bearing':.010}.get(k,r['volume_m3']*{'PLA':1240,'rubber':1100,'hardware':7800}.get(k,1240))
  totals[k]+=mass[i]
 xml=E.Element('mujoco',model='Design B3 compact rockers')
 E.SubElement(xml,'compiler',angle='radian',meshdir='robot_meshes_b3',inertiafromgeom='false',autolimits='true')
 E.SubElement(xml,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='60',magnetic='0.2 0 -0.45')
 vis=E.SubElement(xml,'visual');E.SubElement(vis,'global',offwidth='1280',offheight='900');E.SubElement(vis,'headlight',ambient='.45 .45 .45')
 asset=E.SubElement(xml,'asset');wb=E.SubElement(xml,'worldbody');eq=E.SubElement(xml,'equality');act=E.SubElement(xml,'actuator')
 E.SubElement(asset,'texture',name='checker',type='2d',builtin='checker',rgb1='.85 .85 .85',rgb2='.12 .14 .17',width='512',height='512')
 E.SubElement(asset,'material',name='checker_floor',texture='checker',texrepeat='5 5',texuniform='true',reflectance='0')
 E.SubElement(wb,'light',pos='0 -1 2',dir='0 0 -1')
 E.SubElement(wb,'geom',name='floor',type='plane',size='10 10 .1',material='checker_floor',contype='1',conaffinity='2',friction='.8 .002 .0001')
 root=E.SubElement(wb,'body',name='platform',pos=fmt([0,0,(foot_cfg['tread_radius_mm']-foot_cfg['center_z_mm'])/1000+.002]));E.SubElement(root,'freejoint')
 bodies={'platform':root};origins={'platform':np.zeros(3)};rots={}
 for l in LEGS:
  k=cfg['legs'][l];R=rz(k['neutral_yaw_deg']);rots[l]=R;y=np.array(k['yaw_center_mm'])/1000;p=y+R@np.array(k['pitch_local_mm'])/1000;d=y+R@np.array([-.025,0,0])
  for suffix,origin,parent,axis,limits in [('yaw',y,'platform',[0,0,1],[-45,45]),('pitch',p,l+'_yaw',R@np.array([0,1,0]),[-45,45]),('driver',d,'platform',[0,0,1],[-90,90])]:
   n=l+'_'+suffix;origins[n]=origin;bodies[n]=E.SubElement(bodies[parent],'body',name=n,pos=fmt(origin-origins[parent]))
   E.SubElement(bodies[n],'joint',name=n,axis=fmt(axis),range=fmt(np.radians(limits)),damping='.001',armature='.000001')
   if suffix!='driver':
    limit=cfg['servo_stall_Nm']*(cfg['gear_efficiency_assumed']/1.5 if suffix=='yaw' else 1)
    E.SubElement(act,'motor',name=n,joint=n,ctrlrange=fmt([-limit,limit]))
  E.SubElement(eq,'joint',joint1=l+'_driver',joint2=l+'_yaw',polycoef='0 -0.666666666667 0 0 0',solref='.004 1')
 folder=ROOT/'models/robot_meshes_b3';folder.mkdir(exist_ok=True,parents=True);audit=[]
 colors={'PLA':'.75 .8 .87 1','rubber':'.06 .065 .07 1','servo':'.08 .09 .11 1','battery':'.9 .65 .1 1','electronics':'.15 .5 .23 1','imu':'.2 .65 .3 1','bearing':'.5 .52 .55 1','hardware':'.45 .46 .48 1','ir_sensor':'.12 .12 .15 1','camera_sensor':'.12 .12 .15 1'}
 for group,parts in groups.items():
  entries=[(mass[i],np.array(r['com_m']),np.ptp(np.array(r['vertices']),axis=0)) for i,r in parts]
  if group=='platform':entries.append((.080,np.array([0,0,.02]),np.array([.14,.14,.04])))
  total=sum(m for m,c,s in entries);com=sum(m*c for m,c,s in entries)/total;I=np.zeros((3,3))
  for m,c,s in entries:
   delta=c-com;diag=m/12*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2]);I+=np.diag(diag)+m*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
  E.SubElement(bodies[group],'inertial',mass=fmt(total),pos=fmt(com-origins[group]),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
  audit.append(dict(link=group,mass_kg=total))
  for i,r in parts:
   v,f=visual_mesh(r,origins[group]);name='cad_'+str(i);path=folder/(name+'.obj')
   with path.open('w') as out:
    for a in v:out.write('v '+fmt(a)+'\n')
    for a in f:out.write('f '+' '.join(str(int(x)+1) for x in a)+'\n')
   E.SubElement(asset,'mesh',name=name,file=path.name,inertia='shell',maxhullvert='32')
   E.SubElement(bodies[group],'geom',type='mesh',mesh=name,rgba=colors[r['kind']],contype='0',conaffinity='0',group='1',mass='0')
 def proxy(group,name,kind,**kw):
  return E.SubElement(bodies[group],'geom',name=name,type=kind,contype='2',conaffinity='3',group='3',rgba='.1 .8 .6 .3',mass='0',friction='.8 .002 .0001',solref='.006 1',**kw)
 # Collision proxies preserve the open space between rockers and the moving motor.
 proxy('platform','electronics_contact','box',pos='0 .0075 .0515',size='.040 .035 .0115')
 proxy('platform','chassis_core','box',pos='0 0 .0025',size='.05 .05 .0015')
 for l in LEGS:
  R=rots[l];y=origins[l+'_yaw'];p=origins[l+'_pitch'];a=math.radians(cfg['legs'][l]['neutral_yaw_deg']);quat=fmt([math.cos(a/2),0,0,math.sin(a/2)])
  def localbox(group,name,center,half):
   proxy(group,name,'box',pos=fmt(y+R@np.array(center)-origins[group]),size=fmt(half),quat=quat)
  def localrod(group,name,aa,bb,r):
   proxy(group,name,'capsule',fromto=fmt(np.r_[y+R@np.array(aa)-origins[group],y+R@np.array(bb)-origins[group]]),size=str(r))
  localbox('platform',l+'_yaw_motor',[-.03485,0,-.00505],[.02025,.01,.01805])
  localbox(l+'_pitch',l+'_pitch_motor',[.038,.00305,.01715],[.010,.01805,.02025])
  for sy in [-1,1]:
   localrod(l+'_yaw',l+f'_fork_branch_{sy}',[0,0,.027],[0,sy*.0135,.027],.004)
   for j in range(12):
    aa=math.pi-j*math.pi/24;bb=math.pi-(j+1)*math.pi/24
    pa=[.012+.012*math.cos(aa),sy*(.0135+.012*math.sin(aa)),.027]
    pb=[.012+.012*math.cos(bb),sy*(.0135+.012*math.sin(bb)),.027]
    localrod(l+'_yaw',l+f'_fork_curve_{sy}_{j}',pa,pb,.004)
   localrod(l+'_yaw',l+f'_fork_arm_{sy}',[.012,sy*.0255,.027],[.038,sy*.0255,.027],.004)
  for yy in [-.018,.032]:
   for n,(ex,ez) in enumerate(foot_cfg['strut_endpoints_mm']):
    end=[ex/1000,yy,ez/1000]
    localrod(l+'_pitch',l+f'_foot_strut_{yy}_{n}',[.038,yy,.027],end,.0035)
  cz=foot_cfg['crossbar_z_mm']/1000
  localrod(l+'_pitch',l+'_foot_crossbar',[.036,-.018,cz],[.036,.032,cz],.004)
  # Two quarter-circle tips and a genuinely flat central tread segment.
  profile=[]
  for tip,(cx,start) in enumerate(zip(foot_cfg['centers_x_mm'],[math.pi,1.5*math.pi])):
   extra=foot_cfg.get('outer_radius_extra_mm',0) if tip else 0
   for j in range(9):profile.append((cx/1000,start+j*math.pi/16,extra))
  for strip,(y0,y1) in enumerate(np.array(foot_cfg['tread_y_ranges_mm'])/1000):
   for j in range(len(profile)-1):
    vertices=[]
    for cx,theta,extra in profile[j:j+2]:
     for rad_mm in [foot_cfg['plastic_radius_mm'],foot_cfg['tread_radius_mm']]:
      rad=(rad_mm+extra)/1000
      for yy in [y0,y1]:vertices.append(y+R@np.array([cx+rad*math.cos(theta),yy,(foot_cfg['center_z_mm']+extra)/1000+rad*math.sin(theta)])-p)
    name=f'{l}_tread_{strip}_{j}';E.SubElement(asset,'mesh',name=name,vertex=fmt(np.array(vertices).ravel()))
    proxy(l+'_pitch',name,'mesh',mesh=name)
  E.SubElement(bodies[l+'_pitch'],'site',name=l+'_foot',pos=fmt(R@np.array([-.002,0,(foot_cfg['center_z_mm']-foot_cfg['tread_radius_mm']-27)/1000])),size='.002')
 # Project sensors: native acceleration/magnetic channels, range rays, RGB camera.
 sensors=E.SubElement(xml,'sensor')
 E.SubElement(root,'site',name='mc6470_site',pos=fmt(np.array(sensor_cfg['imu']['center_mm'])/1000),size='.001')
 E.SubElement(sensors,'accelerometer',name='mc6470_acceleration',site='mc6470_site')
 E.SubElement(sensors,'magnetometer',name='mc6470_magnetic',site='mc6470_site')
 for sensor in sensor_cfg['ir']:
  center=np.array(sensor['center_mm'])/1000;axes=np.array(sensor['axes']);ax,ay,az=axes
  proxy('platform',sensor['name']+'_case','box',pos=fmt(center),size=fmt(np.array(sensor_cfg['ir_size_mm'])/2000),xyaxes=fmt(np.r_[ax,ay]))
  # Site +Z points outward from the optical face (package-local +Y).
  E.SubElement(root,'site',name=sensor['name']+'_site',pos=fmt(center+ay*.0066),xyaxes=fmt(np.r_[ax,-az]),size='.001',rgba='1 .2 .1 1')
  E.SubElement(sensors,'rangefinder',name=sensor['name'],site=sensor['name']+'_site')
 for name,key in [('camera_case','camera'),('imu_case','imu')]:
  item=sensor_cfg[key];proxy('platform',name,'box',pos=fmt(np.array(item['center_mm'])/1000),size=fmt(np.array(item['size_mm'])/2000))
 cam=sensor_cfg['camera'];fovy=math.degrees(2*math.atan(math.tan(math.radians(cam['diagonal_fov_deg']/2))*.6))
 E.SubElement(root,'camera',name='project_camera',pos=fmt((np.array(cam['center_mm'])+np.array([7.6,0,0]))/1000),xyaxes='0 -1 0 0 0 1',fovy=str(fovy))
 E.indent(xml);E.ElementTree(xml).write(ROOT/'models/robot_b3.xml',encoding='utf-8',xml_declaration=True)
 from course_b3 import build_course
 course=json.loads((ref/'b3_course_settings.json').read_text())
 metadata=build_course(asset,wb,course)
 E.indent(xml);E.ElementTree(xml).write(ROOT/'models/robot_b3_course.xml',encoding='utf-8',xml_declaration=True)
 (ref/'b3_course.json').write_text(json.dumps(metadata,indent=2))
 totals['unmodeled_horns_fasteners_wiring_straps']=.080
 manifest=dict(foot_geometry=foot_cfg,sensor_layout=sensor_cfg,total_mass_kg=sum(totals.values()),mass_by_kind_kg=dict(totals),links=audit,servo=cfg,limitations=['Solid PLA mass, not slicer verified','Bounding-box approximate inertia','Battery/electronics are packaging reservations with mass allowances','Rigid ramp, no flexible panel model','Friction 0.8 assumed, not measured','Gear efficiency 85% assumed, no backlash or thermal model','No native CAD joints; MuJoCo kinematic tree supplied separately','Collision proxies approximate CAD; mounting details omitted','Yaw restricted to +/-45 degrees; combined pitch/yaw CAD clearance not certified','Pitch motor mass moves with the rocker, opposite to B1'])
 (ref/'robot_b3_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
