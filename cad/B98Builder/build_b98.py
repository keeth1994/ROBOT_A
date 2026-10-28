from pathlib import Path
import adsk.core as C,adsk.fusion as F,math,json,traceback,hashlib,adsk
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'cad/B9_8';BASE=ROOT/'cad/B9_8/source/User_assembly_base.step'
def run(context):
 try:
  (P/'build_error.txt').write_text('')
  app=C.Application.get();tm=F.TemporaryBRepManager.get();doc=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(BASE)));d=F.Design.cast(app.activeProduct);d.designType=F.DesignTypes.DirectDesignType;root=d.rootComponent;doc.name='B9-8 serviceable electronics decks'
  def p(x,y,z):return C.Point3D.create(x/10,y/10,z/10)
  def v(x,y,z):return C.Vector3D.create(x,y,z)
  def cp(b,m=None):
   q=tm.copy(b)
   if m:assert tm.transform(q,m)
   return q
  def move(x=0,y=0,z=0):
   m=C.Matrix3D.create();m.translation=v(x/10,y/10,z/10);return m
  def op(a,b,k):assert tm.booleanOperation(a,b,k);return a
  def un(a,b):return op(a,b,F.BooleanTypes.UnionBooleanType)
  def cut(a,b):return op(a,b,F.BooleanTypes.DifferenceBooleanType)
  def box(x,y,z,X,Y,Z):return tm.createBox(C.OrientedBoundingBox3D.create(p((x+X)/2,(y+Y)/2,(z+Z)/2),v(1,0,0),v(0,1,0),(X-x)/10,(Y-y)/10,(Z-z)/10))
  def rod(a,b,r,R=None):return tm.createCylinderOrCone(p(*a),r/10,p(*b),(r if R is None else R)/10)
  def cyl(x,y,z,Z,r,R=None):return rod((x,y,z),(x,y,Z),r,R)
  def hexz(x,y,z,Z,af=5.8):
   h=box(x-af/2,y-10,z,x+af/2,y+10,Z)
   for a in [60,120]:
    b=box(x-af/2,y-10,z,x+af/2,y+10,Z);m=C.Matrix3D.create();m.setToRotation(math.radians(a),v(0,0,1),p(x,y,0));op(h,cp(b,m),F.BooleanTypes.IntersectionBooleanType)
   return h
  new=[];hardware=[];holes=[]
  def add(q,name,real=True):
   assert q.isSolid and q.lumps.count==1,name
   o=root.occurrences.addNewComponent(C.Matrix3D.create());o.component.name='B9-8 '+name;o.component.bRepBodies.add(q)
   (new if real else hardware).append(o);return o
  def mount(q,x,y,top,bottom):
   cut(q,cyl(x,y,bottom-1,top+1,1.7));holes.append([x,y,top])
  def nutboss(q,x,y,base):
   # Side-entry M3 nut pocket with retaining floor and 2 mm deck roof
   un(q,cyl(x,y,base-2.6,base,4.8));cut(q,hexz(x,y,base-1.6,base+1));cut(q,box(x-2.9,y,base-1.6,x+2.9,y+6,base+1));cut(q,cyl(x,y,base-3,base+5,1.7))
  parts=[(o,o.component.name,cp(b)) for o in root.allOccurrences for b in o.component.bRepBodies]
  ch_o,ch_n,ch=next(t for t in parts if 'chassis' in t[1]);br_o,br_n,br=next(t for t in parts if 'centre bridge' in t[1])
  for o,n,b in parts:
   if any(k in n for k in ['equipment deck','IMU pillar','IMU shelf','sensor camera','camera bolt','camera removable','battery maximum','SERVO2','ESP32 provisional','IMU reservation','DFR0753']):o.deleteMe()
  # Thicken existing post stems by only 0.4 radial; preserve original bores and nut windows.
  for x,y in [(62,0),(-62,0),(0,62),(0,-62)]:
   ring=cyl(x,y,5,35.5,3.9);cut(ring,cyl(x,y,4,36,3.5));un(ch,ring)
  ch_o.component.bRepBodies.item(0).deleteMe();ch_o.component.bRepBodies.add(ch);ch_o.component.name='B9-8 chassis lightly reinforced posts';new.append(ch_o)
  # Bridge support: 11.8 mm OD / 3.4 bore = 4.2 mm wall, 2 mm thicker than the strengthened 7.8 mm posts.
  support=cyl(0,0,12,40.6,5.9);un(support,box(-15,-6,9,15,6,12));cut(support,cyl(0,0,9,41,1.7));cut(support,hexz(0,0,36,38.6));cut(support,box(-2.9,0,36,2.9,8,38.6))
  for x in [-10,10]:
   cut(support,cyl(x,0,8,13,1.7));cut(br,cyl(x,0,3,10,1.7));cut(br,hexz(x,0,4.5,7.1));cut(br,box(x-2.9,0,4.5,x+2.9,12,7.1))
  add(support,'bridge mounted centre deck support');br_o.component.bRepBodies.item(0).deleteMe();br_o.component.bRepBodies.add(br);br_o.component.name='B9-8 bridge captive support nuts';new.append(br_o)
  # Lower plate: flat underside with local nut sockets; no suspended platforms.
  deck=box(-68,-66,40.6,74,66,43.6)
  # Chamfer the corners by diagonal halfplanes.
  for angle in [45,135,225,315]:
   tool=box(70,-150,37,200,150,44);m=C.Matrix3D.create();m.setToRotation(math.radians(angle),v(0,0,1),p(0,0,0));cut(deck,cp(tool,m))
  for x,y in [(62,0),(-62,0),(0,62),(0,-62),(0,0)]:mount(deck,x,y,43.6,40.6)
  # Matrix board platform. Drawing top maps +X; drawing right maps -Y.
  upper=box(-77,-72,71.6,77,72,76.6)
  cut(upper,box(76.5,-11.5,71,80,11.5,77));postxy=[(-70.8,0),(70.8,0),(-10,-29),(-10,29)]
  for y0,y1 in [(-21.5,-11.5),(11.5,21.5)]:cut(upper,box(72.5,y0,71,80,y1,77))
  for i,(x,y) in enumerate(postxy):
   post=cyl(x,y,43.6,68.6,4.3);un(post,cyl(x,y,68.6,73.6,4.3,6));cut(post,cyl(x,y,43,74,1.7))
   for z in [45.6,69.0]:
    cut(post,hexz(x,y,z,z+2.6));cut(post,box(x-2.9,y-8,z,x+2.9,y,z+2.6))
   add(post,'matrix pillar '+str(i+1));mount(deck,x,y,43.6,40.6);un(upper,cyl(x,y,71.6,73.6,6));mount(upper,x,y,76.6,71.6);cut(upper,cyl(x,y,73.6,77,3.2))
  boardholes=[(x,y) for x in [-44.1,46] for y in [-18.5,48]]
  for x,y in boardholes:
   un(upper,cyl(x,y,76.6,79.1,4.8));cut(upper,cyl(x,y,73,82,1.7));cut(upper,hexz(x,y,75.3,77.9));cut(upper,box(x-2.9,y-7,75.3,x+2.9,y,77.9))
  add(cp(upper,move(0,0,2)),'matrix board platform 2p5mm solder clearance')
  board=box(-49.1,-23.5,79.1,51,53,81.1)
  for x,y in boardholes:cut(board,cyl(x,y,78,82,1.7))
  add(cp(board,move(0,0,2)),'matrix board 76p5x100p1 envelope',False)
  add(box(-2,-2,83.1,2,2,85.1),'IMU chip centred XY',False)
  # Battery cradle, closed at -Y and open at +Y for both leads. Two removable straps, no pouch clamp.
  tray=box(-61,-58,43.6,-21,58,46.6)
  for x in [-61,-23]:un(tray,box(x,-58,46.6,x+2,58,54.6))
  un(tray,box(-61,-58,46.6,-21,-56.1,54.6))
  for y in [-38,38]:
   for x in [-60,-22]:cut(tray,box(x-1,y-6,43,x+1,y+6,55))
   cut(tray,box(-61.1,y-6,43,-20.9,y+6,44.8))
  # Ears outside tray, bolted before fitting battery; top access.
  for x,y in [(-65,-12),(-17,-20),(-65,12),(-17,15)]:
   un(tray,cyl(x,y,43.6,46.6,5));mount(tray,x,y,46.6,43.6);nutboss(deck,x,y,40.6)
  cut(tray,cyl(-62,0,43,47,3.5))
  add(tray,'battery cradle 111p2x35x15 open lead end');add(box(-58.5,-55.6,46.6,-23.5,55.6,61.6),'battery envelope',False)
  # Controller seat with tie strap lanes; clamps act on case edges, not PCB or headers.
  seat=box(-1,-30,43.6,58,30,46.6)
  for x,y in [(3,-34),(40,-34),(3,34),(40,34)]:
   un(seat,cyl(x,y,43.6,46.6,5));mount(seat,x,y,46.6,43.6);nutboss(deck,x,y,40.6)
  for x in [-1,56]:
   for y in [-30,22]:un(seat,box(x,y,46.6,x+2,y+8,50.6))
  for x in [13,44]:
   for y in [-29,29]:cut(seat,box(x-5,y-.7,43,x+5,y+.7,46.7))
   cut(seat,box(x-5,-30.1,43,x+5,30.1,44.8))
  cut(seat,cyl(0,0,43,47,3.3))
  for y in [-12,12]:cut(seat,cyl(60,y,43,47,5.5))
  add(seat,'SERVO2 case cradle strap channels');add(box(1,-27.1,46.6,55.2,27.1,66.3),'SERVO2 envelope 54p2x54p2x19p7',False)
  # Regulators use a clear auxiliary mounting zone with strap slots, beside the controller.
  for x in [-20,-2]:
   for y in [-6,6]:cut(deck,box(x-4,y-1,40,x+4,y+1,44))
  spacer=box(-18,-10,43.6,-4,10,45.6);cut(spacer,box(-16,-8,43,-6,11,46));add(spacer,'regulator underside clearance saddle')
  add(box(-18,-10,45.6,-4,10,65.6),'DFR0753 carrier wiring reservation',False)
  # Keep the exact proven sensor seating and sensor hole geometry; add exposed lateral ears.
  for o,n,b in parts:
   if 'ear bracket' not in n:continue
   if 'ir_left' in n or 'ir_right' in n:
    side=1 if 'ir_left' in n else -1
    b=cp(b,move(4,0,0));sen=next(oo for oo,nn,bb in parts if ('sensor ir_left' if side==1 else 'sensor ir_right') in nn);sb=sen.component.bRepBodies.item(0);qq=cp(sb,move(4,0,0));sb.deleteMe();sen.component.bRepBodies.add(qq)
    cut(b,cyl(0,side*62,43,47,3.3))
    for x in [-14.5,22.5]:
     yy=side*44
     un(b,box(x-4,min(yy,side*56),43.6,x+4,max(yy,side*56),46.6));un(b,cyl(x,yy,43.6,46.6,5));mount(b,x,yy,46.6,43.6);nutboss(deck,x,yy,40.6)
   else:
    # Ground sensor shifted forward 16 mm to give camera feet and driver plugs room.
    b=cp(b,move(16,0,0));sen=next(oo for oo,nn,bb in parts if 'sensor ir_ground' in nn);sb=sen.component.bRepBodies.item(0);qq=cp(sb,move(16,0,0));sb.deleteMe();sen.component.bRepBodies.add(qq)
    for y in [-12,12]:
     yy=8 if y>0 else -8
     un(b,box(60,yy-2,43.6,86,yy+2,46.6));un(b,cyl(60,y,43.6,46.6,5));mount(b,60,y,46.6,43.6);nutboss(deck,60,y,40.6)
   o.component.bRepBodies.item(0).deleteMe();o.component.bRepBodies.add(b);o.component.name='B9-8 '+n.split('B8 ')[-1]+' accessible deck tabs';new.append(o)
  # Camera: upright, lens faces +X, 32 mm ear centres. 8 mm ears and 12 mm screws.
  for y in [-16,16]:
   camera=box(67,y-5,43.6,77,y+5,46.6)
   mount(camera,69,y,46.6,43.6);nutboss(deck,69,y,40.6)
   un(camera,box(73,y-4,46.6,77,y+4,92))
   un(camera,rod((73,y,88),(77,y,88),5));cut(camera,rod((72,y,88),(78,y,88),1.7))
   h=hexz(0,0,0,2.8);m=C.Matrix3D.create();m.setToRotation(math.pi/2,v(0,1,0),p(0,0,0));m.translation=v(7.3,y/10,8.8);cut(camera,cp(h,m))
   add(camera,'camera upright ear mount M3x12 '+str(y))
  add(box(77,-12,68,92,12,116.3),'camera envelope 48p3x24x15',False)
  for y in [-16,16]:
   ear=rod((77,y,88),(85,y,88),4.5);cut(ear,rod((76,y,88),(86,y,88),1.7));add(ear,'camera ear envelope '+str(y),False)
  # Small supported perimeter tabs for upper-deck pillar bolts.
  for x,y in postxy:
   un(deck,cyl(x,y,40.6,43.6,5));cut(deck,cyl(x,y,40,44,1.7))
  for x,y in [(62,0),(-62,0),(0,62),(0,-62),(0,0)]:cut(deck,cyl(x,y,40,44,1.7))
  # Camera-foot nuts load from below before deck installation; relieved bosses clear extreme yaw.
  for y in [-16,16]:cut(deck,cyl(69,y,37,39.5,5))
  add(deck,'lower equipment deck')
  # Actual screw-head and shaft envelopes for the selected 8 / 12 mm fasteners.
  def bolt(x,y,z,L,up=False):
   b=cyl(x,y,z,z+L,1.5) if up else cyl(x,y,z-L,z,1.5)
   un(b,cyl(x,y,z-3,z,2.85) if up else cyl(x,y,z,z+3,2.85));return b
  tabxy=[(-65,-12),(-17,-20),(-65,12),(-17,15),(3,-34),(40,-34),(3,34),(40,34),(-14.5,44),(22.5,44),(-14.5,-44),(22.5,-44),(60,-12),(60,12),(69,-16),(69,16)]
  for x,y in tabxy:add(bolt(x,y,46.6,8),'M3x8 mount bolt '+str((x,y)),False)
  for x,y in postxy:
   add(bolt(x,y,40.6,8,True),'M3x8 pillar lower bolt '+str((x,y)),False);add(bolt(x,y,75.6,8),'M3x8 pillar upper bolt '+str((x,y)),False)
  for x,y in boardholes:add(bolt(x,y,83.1,8),'M3x8 matrix bolt '+str((x,y)),False)
  for x in [-10,10]:add(bolt(x,0,12,8),'M3x8 support foot bolt '+str(x),False)
  add(bolt(0,0,43.6,8),'M3x8 centre deck bolt',False)
  for y in [-16,16]:
   b=rod((73,y,88),(85,y,88),1.5);un(b,rod((85,y,88),(88,y,88),2.85));add(b,'M3x12 camera ear bolt '+str(y),False)
  # Mounting/access references recorded separately from the actual bodies.
  report={'source':str(BASE.relative_to(ROOT)).replace('\\','/'),'issues':[],'parts':[],'matrix_chip_xy':[0,0],'matrix_corner_holes':boardholes,'board_bounds_mm':[[-49.1,-23.5,81.1],[51,53,83.1]],'camera_ear_spacing_mm':32,'camera_ear_depth_mm':8,'centre_support_wall_mm':4.2,'strengthened_post_wall_mm':2.2,'original_post_wall_mm':1.8,'solder_clearance_mm':2.5,'mount_tool_axes':[[x,y,75.6 if z==76.6 else z] for x,y,z in holes],'mount_tool_axes_note':'XYZ are screw-head bearing-plane access targets; upper pillar heads are recessed to z75.6.'}
  allb=[(o,o.component.name,cp(b)) for o in root.allOccurrences for b in o.component.bRepBodies]
  for o in new:
   b=o.component.bRepBodies.item(0);report['parts'].append({'name':o.component.name,'volume_mm3':b.volume*1000})
   for oo,n,bb in allb:
    if oo==o or not b.boundingBox.intersects(bb.boundingBox):continue
    q=cp(b);op(q,bb,F.BooleanTypes.IntersectionBooleanType);vol=q.volume*1000 if q.isSolid else 0
    if vol>.1:report['issues'].append({'part':o.component.name,'other':n,'overlap_mm3':vol,'bounds_mm':[[q.boundingBox.minPoint.x*10,q.boundingBox.minPoint.y*10,q.boundingBox.minPoint.z*10],[q.boundingBox.maxPoint.x*10,q.boundingBox.maxPoint.y*10,q.boundingBox.maxPoint.z*10]]})
  (P/'check.json').write_text(json.dumps(report,indent=2));d.exportManager.execute(d.exportManager.createSTEPExportOptions(str(P/'Robot_B9_8_candidate.step'),root));d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(P/'Robot_B9_8_candidate.f3d')));app.activeViewport.fit();app.activeViewport.saveAsImageFile(str(P/'candidate.png'),1500,1100);(P/'done.txt').write_text('candidate')
 except:(P/'build_error.txt').write_text(traceback.format_exc())
