from pathlib import Path
import gzip,math,json,traceback
import adsk.core as C,adsk.fusion as F,adsk
BASE=Path(__file__).resolve().parent.parent
OUT=BASE/'cad'
foot_cfg=json.loads((BASE/'reference/b3_foot_geometry.json').read_text())
sensor_cfg=json.loads((BASE/'reference/b3_sensors.json').read_text())
app=C.Application.get();tm=F.TemporaryBRepManager.get()
def p(x,y,z):return C.Point3D.create(x/10,y/10,z/10)
def v(x,y,z):return C.Vector3D.create(x,y,z)
def box(x0,y0,z0,x1,y1,z1):return tm.createBox(C.OrientedBoundingBox3D.create(p((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),v(1,0,0),v(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cyl(x,y,z0,z1,r):return tm.createCylinderOrCone(p(x,y,z0),r/10,p(x,y,z1),r/10)
def cut(b,t):
 assert tm.booleanOperation(b,t,F.BooleanTypes.DifferenceBooleanType)
 return b
def tx(b,m):
 q=tm.copy(b);assert tm.transform(q,m);return q
def frame(origin,ax=(1,0,0),ay=(0,1,0),az=(0,0,1)):
 m=C.Matrix3D.create();m.setWithCoordinateSystem(p(*origin),v(*ax),v(*ay),v(*az));return m
def rot(deg,origin=(0,0,0),axis=(0,0,1)):
 m=C.Matrix3D.create();m.setToRotation(math.radians(deg),v(*axis),p(*origin));return m
def load(filename,title):
 doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(OUT/filename)));doc.name=title
 d=F.Design.cast(app.activeProduct);return doc,d,d.rootComponent
def overlap(a,b):
 aa=a.boundingBox;bb=b.boundingBox
 if any(aa.maxPoint.asArray()[k]<=bb.minPoint.asArray()[k]+1e-6 or bb.maxPoint.asArray()[k]<=aa.minPoint.asArray()[k]+1e-6 for k in range(3)):return 0
 q=tm.copy(a)
 assert tm.booleanOperation(q,b,F.BooleanTypes.IntersectionBooleanType)
 return q.volume*1000 if q.isSolid else 0
def save(d,root,name):
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/(name+'.f3d'))))
 assert d.exportManager.execute(d.exportManager.createSTEPExportOptions(str(OUT/(name+'.step')),root))
def render(name,eye,target):
 cam=app.activeViewport.camera;cam.eye=p(*eye);cam.target=p(*target);cam.upVector=v(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam
 app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/(name+'.png')),1600,1200)


def union(b,t):
 assert tm.booleanOperation(b,t,F.BooleanTypes.UnionBooleanType);return b
def intersect(b,t):
 assert tm.booleanOperation(b,t,F.BooleanTypes.IntersectionBooleanType);return b
def inv(m):
 q=m.copy();assert q.invert();return q
def rod(a,b,r):return tm.createCylinderOrCone(p(*a),r/10,p(*b),r/10)
logfile=BASE/'logs/b3_cad.log'
logfile.parent.mkdir(exist_ok=True)
def note(s):
 with logfile.open('a') as f:f.write(s+'\n')
 adsk.doEvents()
logfile.write_text('START\n')
# Existing B3 archive supplies appearances only; geometry is rebuilt below.
source_doc,source_design,src=load('Robot_B3_Compact_Rockers.f3d','B3 appearance source')
aps=[b.appearance for o in src.occurrences for b in o.component.bRepBodies]
white=next((a for a in aps if 'white' in a.name.lower()),aps[0]);black=next((a for a in aps if 'black' in a.name.lower()),white);gold=next((a for a in aps if 'yellow' in a.name.lower()),white);green=next((a for a in aps if 'green' in a.name.lower()),white)
nd=app.documents.add(C.DocumentTypes.FusionDesignDocumentType);nd.name='Design B3 - short soles and rounded outer toes';d=F.Design.cast(app.activeProduct);d.designType=F.DesignTypes.DirectDesignType;root=d.rootComponent
parts=[]
def comp(n):
 o=root.occurrences.addNewComponent(C.Matrix3D.create());o.component.name=n;return o
def add(o,q,n,kind='PLA',ap=None):
 assert q.isSolid and q.volume>0,n
 body=o.component.bRepBodies.add(q);body.name=n;body.appearance=ap or white
 parts.append((o,body,kind));return body
ports={}
for leg,x,y,a in [('FL',52,52,45),('RL',-52,52,135),('FR',52,-52,-45),('RR',-52,-52,-135)]:
 pm=rot(a);pm.translation=v(x/10,y/10,0);ports[leg]=pm
# Parallax 900-00005 casing envelope.
servo=box(-10.4,-10,-42.1,30.1,10,-6);union(servo,box(-17.9,-10,-15,37.6,10,-13));union(servo,cyl(0,0,-6,0,6))
for x in [-15.4,35.1]:
 for y in [-5,5]:cut(servo,cyl(x,y,-16,-12,2.25))
yaw_servo_frame=frame((-25,0,19),(-1,0,0),(0,-1,0),(0,0,1))
yawservo=tx(servo,yaw_servo_frame)
pitchframe=frame((38,-21,27),(0,0,-1),(1,0,0),(0,-1,0));pitchservo=tx(servo,pitchframe)
# One structural print houses all four yaw servos and their output bearings.
q=box(-72,-72,1,72,72,4)
for pm in ports.values():
 cut(q,tx(box(-55.7,-10.6,0,-14,10.6,5),pm))
 for x in [-25-35.1,-25+15.4]:
  for y in [-5,5]:cut(q,tx(cyl(x,y,0,5,2.25),pm))
 boss=cyl(0,0,0,9,14);cut(boss,cyl(0,0,-1,10,10.55));union(q,tx(boss,pm))
# Raised perimeter lip, cross ribs and integrated deck posts.
for a in [0,90,180,270]:
 union(q,tx(box(-72,-72,2,72,-69,7),rot(a)))
for x,y in [(0,62),(0,-62),(62,0),(-62,0)]:
 union(q,cyl(x,y,3,40,3.5));cut(q,cyl(x,y,0,42,1.3))
# Central opening and weight-relief slots leave connected ribs.
cut(q,box(-11,-11,0,11,11,8))
for a in [0,90,180,270]:cut(q,tx(box(-5,18,0,5,50,8),rot(a)))
for pm in ports.values():cut(q,tx(cyl(0,0,-2,10,10.55),pm))
for pm in ports.values():cut(q,tx(box(16,-55,-2,85,55,10),pm))
assert q.lumps.count==1
add(comp('B3 00 integrated chassis'),q,'B3 single chassis - four yaw pockets',ap=white)
q=box(-66,-66,40,66,66,43)
for x in [-1,1]:
 for y in [-1,1]:cut(q,box(min(x*12,x*52),min(y*12,y*52),39,max(x*12,x*52),max(y*12,y*52),44))
for x,y in [(0,62),(0,-62),(62,0),(-62,0)]:cut(q,cyl(x,y,39,44,1.7))
# Strap slots through the central rails; electronics are packaging reservations.
for x in [-8,8]:cut(q,box(x-1,-30,39,x+1,30,44))
add(comp('B3 01 removable electronics deck'),q,'B3 open deck with battery strap slots')
add(comp('B3 02 battery reservation'),box(-40,-28,43,40,2,63),'Battery envelope 80x30x20 - pack not selected','battery',gold)
add(comp('B3 03 electronics reservation'),box(-40,7,43,40,42,61),'Controller driver regulator envelope - layout pending','electronics',green)
# Supplied project sensor envelopes and light, open strap cradles.
imu=sensor_cfg['imu'];cx,cy,cz=imu['center_mm'];sx,sy,sz=imu['size_mm']
add(comp('B3 04 MC6470 accel magnetometer'),box(cx-sx/2,cy-sy/2,cz-sz/2,cx+sx/2,cy+sy/2,cz+sz/2),'MIKROE-4228 envelope - no gyroscope','imu',green)
q=box(-66,3,43,-40.2,37,45.7)
for yy in [7,33]:cut(q,box(-62,yy-1,42,-44,yy+1,47))
add(comp('B3 sensor IMU cradle'),q,'Rigid adhesive pad with cable tie slots')
cam=sensor_cfg['camera'];cx,cy,cz=cam['center_mm'];sx,sy,sz=cam['size_mm']
add(comp('B3 sensor camera'),box(cx-sx/2,cy-sy/2,cz-sz/2,cx+sx/2,cy+sy/2,cz+sz/2),'M5Stack TimerCamera-X 48x24x15 envelope','camera_sensor',black)
q=box(45,-26,46,65,26,48.7)
union(q,box(44,-26,46,47.2,26,70))
for yy in [-24,24]:union(q,box(59,yy-2,42.8,65,yy+2,47))
for yy in [-18,18]:cut(q,box(49,yy-1,45,61,yy+1,50))
add(comp('B3 sensor camera cradle'),q,'Open camera tray and strap slots')
for sensor in sensor_cfg['ir']:
 name=sensor['name'];mm=frame(sensor['center_mm'],*sensor['axes'])
 sx,sy,sz=sensor_cfg['ir_size_mm']
 q=box(-sx/2,-sy/2,-sz/2,sx/2,sy/2,sz/2)
 add(comp('B3 sensor '+name),tx(q,mm),'Sharp GP2Y0A41SK0F envelope - optics face local +Y','ir_sensor',black)
 q=box(-17,-9.5,-9,17,-6.8,7.5)
 union(q,box(-17,-9.5,-9,17,6.5,-7))
 for xx in [-16,16]:union(q,box(xx-1,-9.5,-9,xx+1,3,-2))
 for xx in [-10,10]:cut(q,box(xx-1,-10,-5,xx+1,-6,3))
 q=tx(q,mm)
 if name=='ir_ground':
  union(q,rod((65,0,44),(67,0,55),2))
  union(q,box(64,-6,42.8,70,6,46))
 else:
  side=1 if name=='ir_left' else -1
  union(q,box(-17,min(side*54,side*68),42.8,17,max(side*54,side*68),46.5))
 cut(q,tx(box(-sx/2-.2,-sy/2-.2,-sz/2-.2,sx/2+.2,sy/2+.2,sz/2+.2),mm))
 for po,pb,pk in parts:
  if pk in ['camera_sensor','imu'] and overlap(q,pb)>.0001:cut(q,tm.copy(pb))
 assert q.lumps.count==1,'Sensor cradle disconnected '+name
 add(comp('B3 cradle '+name),q,'Open Sharp cradle with strap slots')
# Temporary feature document for proper involute tooth geometry, copied into direct bodies.
proto=comp('B3 temporary gear construction')
def gear(N,phase):
 rp=N/2;rb=rp*math.cos(math.radians(20));ra=rp+1;rf=rp-1.25;invp=math.tan(math.radians(20))-math.radians(20);half=math.pi/(2*N)-.08/(2*rp);pts=[]
 for n in range(N):
  a=phase+n*math.tau/N;ab=half+invp;pts.append((rf*math.cos(a-ab),rf*math.sin(a-ab)))
  for side,js in [(-1,range(5)),(1,range(4,-1,-1))]:
   for j in js:
    rr=rb+(ra-rb)*j/4;t=math.sqrt(max(0,(rr/rb)**2-1));ang=a+side*(half+invp-(t-math.atan(t)));pts.append((rr*math.cos(ang),rr*math.sin(ang)))
  pts.append((rf*math.cos(a+ab),rf*math.sin(a+ab)))
 sk=proto.component.sketches.add(proto.component.xYConstructionPlane);sk.isComputeDeferred=True
 for i in range(len(pts)):sk.sketchCurves.sketchLines.addByTwoPoints(p(*pts[i],0),p(*pts[(i+1)%len(pts)],0))
 sk.isComputeDeferred=False;pro=max(list(sk.profiles),key=lambda z:z.areaProperties().area)
 ei=proto.component.features.extrudeFeatures.createInput(pro,F.FeatureOperations.NewBodyFeatureOperation);ei.setDistanceExtent(False,C.ValueInput.createByString('5 mm'));ext=proto.component.features.extrudeFeatures.add(ei)
 b=tx(ext.bodies.item(0),frame((0,0,-7)));sk.isVisible=False;return b
gear30=gear(30,0);gear20=gear(20,math.pi/20);proto.isLightBulbOn=False
note('Gears created')
# Top-facing yaw gears and a two-sided Y fork. Pitch servo travels WITH the foot.
gear20top=tx(gear20,frame((0,0,20)))
turret=cyl(0,0,4,22,6);union(turret,gear20top)
union(turret,cyl(0,0,19,27,12))
for sy in [-1,1]:
 # True tangent quarter-circle bend: 12 mm centreline radius, 8 mm tube diameter.
 # Inner bend radius is 8 mm. No intersecting straight tubes at the elbow.
 bend=tx(tm.createTorus(p(0,0,0),v(0,0,1),1.2,.4),frame((12,sy*13.5,27)))
 intersect(bend,box(-5,13.5 if sy>0 else -31,22,12,31 if sy>0 else -13.5,32))
 union(turret,rod((0,0,27),(0,sy*13.52,27),4))
 union(turret,bend)
 union(turret,rod((11.98,sy*25.5,27),(38,sy*25.5,27),4))
 union(turret,rod((38,sy*25.5-2.5,27),(38,sy*25.5+2.5,27),10))
cut(turret,rod((38,-30,27),(38,-20,27),2))
for z in [21,33]:cut(turret,rod((38,-30,z),(38,-20,z),1.25))
cut(turret,rod((38,21,27),(38,30,27),4.05))
cut(turret,cyl(0,0,2,27,1.3))
assert turret.lumps.count==1
# Compact flat-bottom runners with tangent rounded ends; dimensions shared with MuJoCo.
def arc(y0,y1,inner,outer):
 left,right=foot_cfg['centers_x_mm'];z=foot_cfg['center_z_mm']
 extra=foot_cfg.get('outer_radius_extra_mm',0)
 b=box(left,y0,z-outer,right,y1,z-inner)
 for cx,cz,ri,ro,is_left in [(left,z,inner,outer,True),(right,z+extra,inner+extra,outer+extra,False)]:
  q=rod((cx,y0,cz),(cx,y1,cz),ro)
  cut(q,rod((cx,y0-1,cz),(cx,y1+1,cz),ri))
  intersect(q,box(cx-ro-1 if is_left else cx-.01,y0-1,cz-ro-1,cx+.01 if is_left else cx+ro+1,y1+1,cz))
  union(b,q)
 return b
foot=None
for y0,y1 in foot_cfg['rail_y_ranges_mm']:
 q=arc(y0,y1,foot_cfg['inner_radius_mm'],foot_cfg['plastic_radius_mm']);yc=(y0+y1)/2
 union(q,rod((38,y0,27),(38,y1,27),9))
 for ex,ez in foot_cfg['strut_endpoints_mm']:
  end=(ex,yc,ez)
  union(q,rod((38,yc,27),end,3.5))
 cut(q,rod((38,y0-1,27),(38,y1+1,27),6.4 if y0<0 else 2.1))
 if foot is None:foot=q
 else:union(foot,q)
union(foot,rod((36,-18,foot_cfg['crossbar_z_mm']),(36,32,foot_cfg['crossbar_z_mm']),4))
# Four-hole flange plate stays with the moving rocker, not the yaw fork.
plate=box(23,-10,-13,53,-8,48)
cut(plate,box(27.4,-11,-3.7,48.6,-7,38))
for x in [33,43]:
 for z in [-8.1,42.4]:cut(plate,rod((x,-11,z),(x,-7,z),2.25))
union(foot,plate)
for z in [-9,44]:
 union(foot,box(23,-18,z-2,27,-8,z+2))
 union(foot,rod((25,-18,z),(38,-18,27),2))
# Clearance around the moving motor casing, including reinforcing webs.
cut(foot,box(27.5,-15.5,-3.6,48.5,21.6,37.9))
# Relieve mounting webs around the actual casing/flanges with 0.3 mm clearance.
for off in [(0,0,0),(.3,0,0),(-.3,0,0),(0,.3,0),(0,-.3,0),(0,0,.3),(0,0,-.3)]:
 cut(foot,tx(pitchservo,frame(off)))
assert foot.lumps.count==1,'Foot disconnected'
# Small local fork reliefs for the moving motor and its mounting frame.
for angle in range(-46,47,2):
 m=rot(angle,(38,0,27),(0,1,0))
 for geo in [foot,pitchservo]:
  swept=tx(geo,m)
  for off in [(0,0,0),(.4,0,0),(-.4,0,0),(0,.4,0),(0,-.4,0),(0,0,.4),(0,0,-.4)]:
   tool=tx(swept,frame(off))
   if overlap(turret,tool)>.0001:cut(turret,tool)
assert turret.lumps.count==1,'Fork relief disconnected structure'

driver=tx(gear30,frame((-25,0,20)));union(driver,cyl(-25,0,18,21,9))
cut(driver,cyl(-25,0,12,19.3,6.3));cut(driver,cyl(-25,0,12,23,1.7))
for y in [-6,6]:cut(driver,cyl(-25,y,12,23,1.25))
# Relieve the chassis lip for side-facing and forward-facing neutral leg poses.
co,cb,ck=parts[0];cq=tm.copy(cb)
for port in ports.values():
 for angle in range(-45,46,5):
  tool=tx(tx(foot,rot(angle)),port)
  for off in [(0,0,0),(.5,0,0),(-.5,0,0),(0,.5,0),(0,-.5,0)]:
   tt=tx(tool,frame(off))
   if overlap(cq,tt)>.0001:cut(cq,tt)
assert cq.lumps.count==1
cb.deleteMe();nb=co.component.bRepBodies.add(cq);nb.name='B3 relieved integrated chassis';nb.appearance=white;parts[0]=(co,nb,ck)
leg_templates={}
for leg,pm in ports.items():
 add(comp(leg+' B3 yaw servo fixed'),tx(yawservo,pm),'Parallax 900-00005 yaw','servo',black)
 add(comp(leg+' B3 yaw driver'),tx(driver,pm),'B3 30T yaw driver - stock horn attachment pending','PLA',gold)
 q=cyl(0,0,4,9,10.5);cut(q,cyl(0,0,3,10,6));add(comp(leg+' B3 yaw bearing'),tx(q,pm),'Yaw bearing envelope 12x21x5','bearing',white)
 q=cyl(0,0,1,3,8);cut(q,cyl(0,0,0,4,1.7));add(comp(leg+' B3 retention washer'),tx(q,pm),'Yaw retention washer and screw allowance','hardware',white)
 add(comp(leg+' B3 yaw turret'),tx(turret,pm),'B3 smooth R12 Y fork - 8 mm section')
 add(comp(leg+' B3 pitch servo'),tx(pitchservo,pm),'Parallax 900-00005 pitch','servo',black)
 q=rod((38,24,27),(38,28,27),4);cut(q,rod((38,23,27),(38,29,27),2.05));add(comp(leg+' B3 pitch bushing'),tx(q,pm),'Pitch passive bushing 4x8x4','hardware')
 q=rod((38,24,27),(38,34,27),2);add(comp(leg+' B3 pitch pin'),tx(q,pm),'Passive M4 axle placeholder','hardware')
 q=rod((38,-23,27),(38,-21,27),9);cut(q,rod((38,-24,27),(38,-20,27),1.7))
 for z in [21,33]:cut(q,rod((38,-24,z),(38,-20,z),1.25))
 add(comp(leg+' B3 pitch horn'),tx(q,pm),'Stock horn envelope - measure before printing','hardware',gold)
 add(comp(leg+' B3 rocker foot'),tx(foot,pm),'B3 compact flat runner and moving servo flange - one print')
 for y0,y1 in foot_cfg['tread_y_ranges_mm']:add(comp(leg+' B3 rubber tread'),tx(arc(y0,y1,foot_cfg['plastic_radius_mm'],foot_cfg['tread_radius_mm']),pm),'B3 replaceable rubber strip 2mm','rubber',black)
 leg_templates[leg]=dict(yaw_center_mm=[pm.translation.x*10,pm.translation.y*10,0],neutral_yaw_deg={'FL':45,'RL':135,'FR':-45,'RR':-135}[leg],pitch_local_mm=[38,0,27])
 note(leg+' built')
# Conservative bound valid for any yaw: max radius of all pitch-swept vertices about yaw axis.
# Actual sampled meshes refine it below; no claims about simultaneous leg-to-leg clearance.
records=[];inventory=[]
for o,body,kind in parts:
 calc=body.meshManager.createMeshCalculator();calc.surfaceTolerance=.025;calc.maxNormalDeviation=.3;m=calc.calculate();vv=m.nodeCoordinatesAsDouble
 record=dict(component=o.component.name,body=body.name,kind=kind,vertices=[[vv[k]/100,vv[k+1]/100,vv[k+2]/100] for k in range(0,len(vv),3)],indices=list(m.nodeIndices),appearance=body.appearance.name,volume_m3=body.volume/1e6,com_m=[x/100 for x in body.physicalProperties.centerOfMass.asArray()]);records.append(record)
 bb=body.boundingBox;inventory.append(dict(component=o.component.name,body=body.name,kind=kind,volume_cm3=body.volume,min_mm=[x*10 for x in bb.minPoint.asArray()],max_mm=[x*10 for x in bb.maxPoint.asArray()]))
(OUT/'B3_inventory.json').write_text(json.dumps(inventory,indent=2))
ref=BASE/'reference'
with gzip.open(ref/'b3_cad_meshes.json.gz','wt') as f:json.dump(dict(source='Robot_B3_Compact_Rockers.f3d',revision=foot_cfg['revision'],bodies=records),f)
(ref/'b3_kinematics.json').write_text(json.dumps(dict(cad_revision=foot_cfg['revision'],legs=leg_templates,yaw_requested_range_deg=[-135,135],pitch_requested_range_deg=[-45,45],clearance_status='See B3 clearance audit; requested travel is not clearance certified',pitch_servo_moves_with_foot=True,gear_ratio_speed=1.5,gear_efficiency_assumed=.85,servo_stall_Nm=.26833896966654,servo_free_speed_deg_s=315.7894736842,servo_mass_kg=.044),indent=2))
# Exclude construction solids from manufacturing exports.
proto.deleteMe()
save(d,root,'Robot_B3_Compact_Rockers');note('Saved F3D and STEP')
cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=p(450,-570,370);cam.target=p(0,0,-5);cam.upVector=v(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam;app.activeViewport.refresh();adsk.doEvents()
app.activeViewport.saveAsImageFile(str(OUT/'Robot_B3_Compact_Rockers.png'),1600,1200)
note('BUILT')
exec(compile((BASE/'cad/audit_b3_fusion.py').read_text(), 'audit_b3.py','exec'))
note('DONE')

