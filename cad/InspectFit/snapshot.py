import adsk.core as C, adsk.fusion as F, json, traceback
from pathlib import Path

def run(context):
 out=Path('C:/PROJECTS/ROBOT_A/cad/fit_audit_20261005');out.mkdir(exist_ok=True)
 try:
  app=C.Application.get();d=F.Design.cast(app.activeProduct);root=d.rootComponent
  assert d, 'Open edited rocker first'
  assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(out/'User_updated_rocker.f3d')))
  assert d.exportManager.execute(d.exportManager.createSTEPExportOptions(str(out/'User_updated_rocker.step'),root))
  rows=[]
  for co in d.allComponents:
   for b in co.bRepBodies:
    holes=[]
    for face in b.faces:
     g=C.Cylinder.cast(face.geometry)
     if g: holes.append(dict(radius_mm=g.radius*10,origin_mm=[x*10 for x in g.origin.asArray()],axis=g.axis.asArray()))
    rows.append(dict(component=co.name,name=b.name,volume_mm3=b.volume*1000,solid=b.isSolid,lumps=b.lumps.count,bbox_mm=[[x*10 for x in b.boundingBox.minPoint.asArray()],[x*10 for x in b.boundingBox.maxPoint.asArray()]],cylinders=holes))
  data=dict(document=app.activeDocument.name,bodies=rows,features=[dict(name=f.name,type=f.objectType,health=str(f.healthState)) for co in d.allComponents for f in co.features])
  (out/'inspection.json').write_text(json.dumps(data,indent=2))
  app.activeViewport.saveAsImageFile(str(out/'User_updated_rocker.png'),1600,1200)
 except: (out/'error.txt').write_text(traceback.format_exc())
