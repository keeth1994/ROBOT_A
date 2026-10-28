from pathlib import Path
import adsk.core as C,adsk.fusion as F,adsk,json,math,traceback,gc,hashlib
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'cad/B9_8'
def run(context):
 try:
  (P/'motion_error.txt').write_text('')
  app=C.Application.get();tm=F.TemporaryBRepManager.get();d=F.Design.cast(app.activeProduct)
  def p(x,y,z):return C.Point3D.create(x/10,y/10,z/10)
  def cp(b,m=None):
   q=tm.copy(b)
   if m:tm.transform(q,m)
   return q
  def rot(x,y,z,ax,ay,az,a):
   m=C.Matrix3D.create();m.setToRotation(math.radians(a),C.Vector3D.create(ax,ay,az),p(x,y,z));return m
  def overlap(a,b):
   if not a.boundingBox.intersects(b.boundingBox):return 0
   q=cp(a);assert tm.booleanOperation(q,cp(b),F.BooleanTypes.IntersectionBooleanType);return q.volume*1000 if q.isSolid else 0
  parts=[(o.component.name,cp(b)) for o in d.rootComponent.allOccurrences for b in o.component.bRepBodies]
  assert any('B9-8 lower equipment deck' in n for n,b in parts)
  digest=hashlib.sha256((P/'Robot_B9_8_candidate.step').read_bytes()).hexdigest()
  saved=json.loads((P/'motion_report.json').read_text()) if (P/'motion_report.json').exists() else {}
  if saved.get('candidate_sha256',saved.get('model_sha256'))==digest:issues=saved['issues'];count=saved['checks'];completed=saved['completed_legs']
  else:issues=[];count=0;completed=[]
  fixed=[(n,b) for n,b in parts if n.startswith('B9-8') or 'sensor ir_' in n]
  for leg,x,y,a in [('FL',52,52,45),('FR',52,-52,-45),('RL',-52,52,135),('RR',-52,-52,-135)]:
   if leg in completed:continue
   c=math.cos(math.radians(a));s=math.sin(math.radians(a));pc=(x+52*c,y+52*s,35)
   moving=[(n,b) for n,b in parts if n.startswith(leg+' ') and any(k in n for k in ['pitch servo','rocker foot','rubber tread','pitch cross brace','pitch brace M3'])]
   fork=next(b for n,b in parts if n.startswith(leg+' ') and 'yaw turret' in n)
   for yd in [-45,-30,-15,0,15,30,45]:
    gc.collect();adsk.doEvents();(P/'motion_progress.txt').write_text(leg+' yaw '+str(yd))
    yr=rot(x,y,0,0,0,1,yd);fq=cp(fork,yr)
    for nn,bb in fixed:
     count+=1;ov=overlap(fq,bb)
     if ov>.1:issues.append({'part':leg+' turret','other':nn,'yaw':yd,'overlap_mm3':ov})
    for pd in [-45,-30,-15,0,15,25]:
     pr=rot(*pc,-s,c,0,pd)
     for n,b in moving:
      q=cp(cp(b,pr),yr)
      for nn,bb in fixed:
       count+=1;ov=overlap(q,bb)
       if ov>.1:issues.append({'part':n,'other':nn,'yaw':yd,'pitch':pd,'overlap_mm3':ov})
   completed.append(leg)
   (P/'motion_report.json').write_text(json.dumps({'model_sha256':digest,'checks':count,'completed_legs':completed,'issues':issues,'scope':'New/changed fixed parts and equipment only. Yaw -45..45 by15deg; pitch -45,-30,-15,0,15,25deg. Discrete CAD check, not continuous, dynamic, cable or strength validation.'},indent=2))
  if len(completed)==4:(P/'motion_done.txt').write_text(str(len(issues)))
 except:(P/'motion_error.txt').write_text(traceback.format_exc())
