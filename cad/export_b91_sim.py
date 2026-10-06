from pathlib import Path
import adsk.core as C,adsk.fusion as F,json,gzip,traceback,hashlib
BASE=Path(__file__).resolve().parents[1]
def run(context):
 try:
  app=C.Application.get();previous=app.activeDocument
  source=BASE/'cad/B9_1/Robot_B9_1.f3d'
  doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
  d=F.Design.cast(app.activeProduct);records=[]
  for o in d.rootComponent.allOccurrences:
   for b in o.bRepBodies:
    calc=b.meshManager.createMeshCalculator();calc.surfaceTolerance=.01;calc.maxNormalDeviation=.2;m=calc.calculate();vv=m.nodeCoordinatesAsDouble
    records.append(dict(component=o.component.name,body=b.name,vertices=[[vv[k]/100,vv[k+1]/100,vv[k+2]/100] for k in range(0,len(vv),3)],indices=list(m.nodeIndices),volume_m3=b.volume/1e6,com_m=[x/100 for x in b.physicalProperties.centerOfMass.asArray()]))
  with gzip.open(BASE/'reference/b91_cad_meshes.json.gz','wt') as f:json.dump(dict(revision='B9-1',source=str(source.relative_to(BASE)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),bodies=records),f)
  doc.close(False)
  if previous:previous.activate()
  (BASE/'cad/B9_1/sim_export_complete.txt').write_text(str(len(records))+' CAD bodies exported')
 except:(BASE/'cad/B9_1/sim_export_error.txt').write_text(traceback.format_exc())
