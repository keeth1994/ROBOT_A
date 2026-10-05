"""Fusion: build B5 from preserved B4 solids and the user's edited rocker."""
from pathlib import Path
import adsk.core as C, adsk.fusion as F
import math,json,traceback
BASE=Path(__file__).resolve().parents[2];OUT=BASE/'cad/B5_plain_pivots'
def run(context):
 OUT.mkdir(parents=True,exist_ok=True)
 for marker in ['complete.txt','error.txt']:
  (OUT/marker).unlink(missing_ok=True)
 try:
  app=C.Application.get();tm=F.TemporaryBRepManager.get()
  def p(x,y,z):return C.Point3D.create(x/10,y/10,z/10)
  def cyl(x,y,z0,z1,r):return tm.createCylinderOrCone(p(x,y,z0),r/10,p(x,y,z1),r/10)
  def rod(a,b,r):return tm.createCylinderOrCone(p(*a),r/10,p(*b),r/10)
  def boolean(a,b,op):assert tm.booleanOperation(a,b,op);return a
  def cut(a,b):return boolean(a,b,F.BooleanTypes.DifferenceBooleanType)
  def union(a,b):return boolean(a,b,F.BooleanTypes.UnionBooleanType)
  def tx(b,m):q=tm.copy(b);assert tm.transform(q,m);return q
  def rot(a,origin=(0,0,0),axis=(0,0,1)):
   m=C.Matrix3D.create();m.setToRotation(math.radians(a),C.Vector3D.create(*axis),p(*origin));return m
  def overlap(a,b):
   aa=a.boundingBox;bb=b.boundingBox
   if any(aa.maxPoint.asArray()[k]<=bb.minPoint.asArray()[k] or bb.maxPoint.asArray()[k]<=aa.minPoint.asArray()[k] for k in range(3)):return 0
   q=boolean(tm.copy(a),b,F.BooleanTypes.IntersectionBooleanType);return q.volume*1000 if q.isSolid else 0
  def load(path):
   doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)));return doc,F.Design.cast(app.activeProduct)
  u,ud=load(BASE/'cad/fit_audit_20261005/User_updated_rocker.f3d');foot=tm.copy(ud.rootComponent.bRepBodies.item(0));u.close(False)
  src,sd=load(BASE/'cad/B4_reinforced_harness/Robot_B4_Reinforced_Harness.f3d')
  source=[(o.component.name,b.name,tm.copy(b),b.appearance) for o in sd.rootComponent.occurrences for b in o.component.bRepBodies]
  chassis=tm.copy(next(b for n,_,b,_ in source if 'integrated chassis' in n))
  ports={}
  for leg,x,y,a in [('FL',52,52,45),('RL',-52,52,135),('FR',52,-52,-45),('RR',-52,-52,-135)]:
   m=rot(a);m.translation=C.Vector3D.create(x/10,y/10,0);ports[leg]=m
  inverse=ports['FL'].copy();inverse.invert()
  fork=tx(next(b for n,_,b,_ in source if n=='FL B3 yaw turret'),inverse)
  # Replace all four bearing seats with integral 12.4 mm plain bores, 12.7 mm long.
  for m in ports.values():
   sleeve=cyl(0,0,0,12.7,10.65);cut(sleeve,cyl(0,0,-1,14,6.2));union(chassis,tx(sleeve,m))
  # Rotating journal extends to the underside; gear underside is the upper thrust shoulder.
  union(fork,cyl(0,0,-.3,13.1,6));cut(fork,cyl(0,0,-1,24,1.7))
  # Captive M3 hex nut; top access counterbore allows insertion and screwdriver access.
  # Hex pocket is 5.7 mm across flats, 2.7 mm deep, positioned below an access well.
  vertices=[p(3.291*math.cos(i*math.pi/3),3.291*math.sin(i*math.pi/3),19) for i in range(6)]
  # Temporary prism is generated in the new design below.
  cut(fork,cyl(0,0,21.7,33,3.4))
  # Pitch bushings become integral fork bosses, clearance for a 4 mm axle.
  union(fork,rod((38,23.8,27),(38,28,27),4.15));cut(fork,rod((38,21,27),(38,30,27),2.2))
  # Relieve inherited axle and horn collisions without rebuilding the user's rocker.
  original_foot_volume=foot.volume*1000
  cut(foot,rod((38,23,27),(38,36,27),2.2))
  cut(foot,rod((38,-23.3,27),(38,-20.7,27),9.3))
  assert foot.isSolid and foot.lumps.count==1
  nd=app.documents.add(C.DocumentTypes.FusionDesignDocumentType);nd.name='B5 - integrated plain pivots - user rocker';d=F.Design.cast(app.activeProduct);d.designType=F.DesignTypes.DirectDesignType;root=d.rootComponent
  temp=root.occurrences.addNewComponent(C.Matrix3D.create());sk=temp.component.sketches.add(temp.component.xYConstructionPlane)
  for i in range(6):sk.sketchCurves.sketchLines.addByTwoPoints(p(vertices[i].x*10,vertices[i].y*10,0),p(vertices[(i+1)%6].x*10,vertices[(i+1)%6].y*10,0))
  ei=temp.component.features.extrudeFeatures.createInput(sk.profiles.item(0),F.FeatureOperations.NewBodyFeatureOperation);ei.setDistanceExtent(False,C.ValueInput.createByString('2.7 mm'));ex=temp.component.features.extrudeFeatures.add(ei)
  m=C.Matrix3D.create();m.translation=C.Vector3D.create(0,0,1.9);hexp=tx(ex.bodies.item(0),m);cut(fork,hexp);temp.deleteMe()
  assert fork.isSolid and fork.lumps.count==1
  washer=cyl(0,0,-2.3,-.3,9);cut(washer,cyl(0,0,-3,0,1.7))
  records=[];bodies=[]
  def add(name,q,ap):
   assert q.isSolid and q.lumps.count==1,name
   o=root.occurrences.addNewComponent(C.Matrix3D.create());o.component.name=name;b=o.component.bRepBodies.add(q);b.name=name;b.appearance=ap;bodies.append((name,q));records.append(dict(name=name,volume_mm3=q.volume*1000))
  for n,bn,b,ap in source:
   leg=n[:2]
   if 'yaw bearing' in n or 'pitch bushing' in n:continue
   if 'integrated chassis' in n:b=chassis
   elif 'yaw turret' in n:b=tx(fork,ports[leg])
   elif 'rocker foot' in n:b=tx(foot,ports[leg])
   elif 'retention washer' in n:b=tx(washer,ports[leg])
   add(n.replace('B3','B5'),b,ap)
  tests=[]
  for angle in range(-45,46,5):
   q=tx(foot,rot(angle,(38,0,27),(0,1,0)));tests.append(dict(test='pitch_rocker_fork',angle=angle,intersection_mm3=overlap(q,fork)))
  for leg,m in ports.items():
   for angle in range(-45,46,5):
    tests.append(dict(test='yaw_fork_chassis',leg=leg,angle=angle,intersection_mm3=overlap(tx(tx(fork,rot(angle)),m),chassis)))
   tests.append(dict(test='retainer_chassis',leg=leg,intersection_mm3=overlap(tx(washer,m),chassis)))
  report=dict(tests=tests,issues=[t for t in tests if t['intersection_mm3']>.1],parts=records,removed_separate_bearings=4,removed_separate_pitch_bushings=4,journal_diameter_mm=12,bore_diameter_mm=12.4,radial_clearance_mm=.2,upper_axial_gap_mm=.3,lower_axial_gap_mm=.3,nut='M3 hex nut, 5.5 mm AF and <=2.7 mm thick; pocket 5.7 AF',retainer_screw='M3 x 25 mm, measured from under head; use 2 mm thick retainer',original_user_rocker_mm3=original_foot_volume,revised_rocker_mm3=foot.volume*1000,scope='CAD intersections sampled at 5 degrees; friction, wear, loads and full combined leg motions not validated. Stock horn spacing remains awaiting measurement.')
  (OUT/'fit_report.json').write_text(json.dumps(report,indent=2))
  assert not report['issues'],str(report['issues'][:4])
  assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'Robot_B5_Plain_Pivots.f3d')))
  assert d.exportManager.execute(d.exportManager.createSTEPExportOptions(str(OUT/'Robot_B5_Plain_Pivots.step'),root))
  cam=app.activeViewport.camera;cam.eye=p(330,-420,320);cam.target=p(0,0,15);cam.upVector=C.Vector3D.create(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam;app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/'Robot_B5_Plain_Pivots.png'),1600,1200)
  (OUT/'complete.txt').write_text('CAD export and sampled fit checks passed. Physical validation pending.')
 except:(OUT/'error.txt').write_text(traceback.format_exc())
