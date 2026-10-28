import adsk.core as C,adsk.fusion as F,json,math,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'cad/B9_8'
def run(context):
 try:
  d=F.Design.cast(C.Application.get().activeProduct);tm=F.TemporaryBRepManager.get();parts=[(o.component.name,tm.copy(b)) for o in d.rootComponent.allOccurrences for b in o.component.bRepBodies];checks=[];issues=[]
  def p(x,y,z):return C.Point3D.create(x/10,y/10,z/10)
  def ov(a,b):
   if not a.boundingBox.intersects(b.boundingBox):return 0
   q=tm.copy(a);tm.booleanOperation(q,tm.copy(b),F.BooleanTypes.IntersectionBooleanType);return q.volume*1000 if q.isSolid else 0
  # Straight 5.7 mm diameter driver corridor, upper matrix assembly removed.
  skip=['matrix board','IMU chip','matrix bolt','pillar upper bolt']
  staged=[(n,b) for n,b in parts if not any(k in n for k in skip)]
  for n,b in parts:
   if n.startswith('B9-8 M3x8 mount bolt'):
    bb=b.boundingBox;x=(bb.minPoint.x+bb.maxPoint.x)*5;y=(bb.minPoint.y+bb.maxPoint.y)*5
    tool=tm.createCylinderOrCone(p(x,y,49.7),.285,p(x,y,120),.285)
    for nn,bb in staged:
     a=ov(tool,bb)
     if a>.1:issues.append({'test':'mount driver access, matrix platform off','bolt':n,'obstacle':nn,'mm3':a})
    checks.append(n)
  electronics=[(n,b) for n,b in parts if n.startswith('B9-8') and any(k in n for k in ['envelope','IMU chip']) or 'sensor ir_' in n]
  for n,b in parts:
   if n.startswith('B9-8 M3x'):
    for nn,bb in electronics:
     a=ov(b,bb)
     if a>.1:issues.append({'test':'new bolt versus electronics','bolt':n,'obstacle':nn,'mm3':a})
  # 60-degree forward cone within the camera's nominal diagonal FOV, from approximate lens centre.
  cone=tm.createCylinderOrCone(p(92.1,0,106),.02,p(192.1,0,106),5.77)
  for n,b in parts:
   if 'camera envelope' in n:continue
   a=ov(cone,b)
   if a>.1:issues.append({'test':'forward view cone','obstacle':n,'mm3':a})
  report={'scope':'5.7mm straight driver envelope above 16 new mount bolts with matrix platform removed; new screw envelopes versus equipment; approximate 60deg forward camera cone. Connector plugs and flexible wiring not modelled.','issues':issues,'mounts_checked':len(checks),'thread_engagement_mm':{'M3x8_tab_mount':2.4,'M3x8_pillar':2.4,'M3x8_matrix_board':2.4,'M3x12_camera_ear':2.4},'nut_allowance_mm':{'across_flats':5.8,'thickness':2.6,'nominal_nut_AF':5.5,'nominal_nut_thickness':2.4}}
  (P/'access_report.json').write_text(json.dumps(report,indent=2))
 except:(P/'access_error.txt').write_text(traceback.format_exc())
