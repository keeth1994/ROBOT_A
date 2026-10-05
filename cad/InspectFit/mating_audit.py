import adsk.core as C, adsk.fusion as F, json, traceback, math
from pathlib import Path

def run(context):
 out=Path('C:/PROJECTS/ROBOT_A/cad/fit_audit_20261005')
 try:
  app=C.Application.get(); user=app.activeDocument; d=F.Design.cast(app.activeProduct);tm=F.TemporaryBRepManager.get();foot=tm.copy(d.rootComponent.bRepBodies.item(0))
  health=[dict(name=f.name,health=str(f.healthState),message=f.errorOrWarningMessage) for f in d.rootComponent.features]
  doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions('C:/PROJECTS/ROBOT_A/cad/B4_reinforced_harness/B4_Reinforced_Leg_Detail.f3d'))
  design=F.Design.cast(app.activeProduct);parts=[(o.component.name,tm.copy(b)) for o in design.rootComponent.occurrences for b in o.component.bRepBodies]
  def overlap(a,b):
   q=tm.copy(a);assert tm.booleanOperation(q,b,F.BooleanTypes.IntersectionBooleanType)
   return q.volume*1000 if q.isSolid else 0
  previous=next(b for n,b in parts if 'rocker foot' in n)
  added=tm.copy(foot);tm.booleanOperation(added,previous,F.BooleanTypes.DifferenceBooleanType)
  removed=tm.copy(previous);tm.booleanOperation(removed,foot,F.BooleanTypes.DifferenceBooleanType)
  checks=[]
  for angle in range(-45,46,5):
   m=C.Matrix3D.create();m.setToRotation(math.radians(angle),C.Vector3D.create(0,1,0),C.Point3D.create(3.8,0,2.7));q=tm.copy(foot);tm.transform(q,m)
   for n,b in parts:
    if any(k in n for k in ['yaw turret','pitch horn','pitch bushing','pitch pin']):checks.append(dict(angle=angle,target=n,intersection_mm3=overlap(q,b)))
  servo=next(b for n,b in parts if 'pitch servo' in n)
  checks.append(dict(angle=0,target='pitch servo casing',intersection_mm3=overlap(foot,servo)))
  baseline=[dict(target=n,intersection_mm3=overlap(previous,b)) for n,b in parts if any(k in n for k in ['yaw turret','pitch horn','pitch bushing','pitch pin','pitch servo'])]
  report=dict(baseline_neutral=baseline,health=health,parts=[n for n,b in parts],previous_volume_mm3=previous.volume*1000,new_volume_mm3=foot.volume*1000,added_mm3=added.volume*1000 if added.isSolid else 0,removed_mm3=removed.volume*1000 if removed.isSolid else 0,checks=checks)
  (out/'mating_audit.json').write_text(json.dumps(report,indent=2))
  doc.close(False)
  assembly=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions('C:/PROJECTS/ROBOT_A/cad/B4_reinforced_harness/Robot_B4_Reinforced_Harness.f3d'))
  dd=F.Design.cast(app.activeProduct);dims=[]
  for o in dd.rootComponent.occurrences:
   if any(k in o.component.name for k in ['integrated chassis','yaw bearing','yaw driver','retention washer']):
    for b in o.component.bRepBodies:
     cs=[]
     for face in b.faces:
      g=C.Cylinder.cast(face.geometry)
      if g:cs.append(dict(diameter_mm=g.radius*20,origin_mm=[a*10 for a in g.origin.asArray()],axis=g.axis.asArray(),min_mm=[a*10 for a in face.boundingBox.minPoint.asArray()],max_mm=[a*10 for a in face.boundingBox.maxPoint.asArray()]))
     dims.append(dict(component=o.component.name,cylinders=cs))
  (out/'assembly_hole_dimensions.json').write_text(json.dumps(dims,indent=2))
  assembly.close(False);user.activate()
 except:(out/'mating_error.txt').write_text(traceback.format_exc())
