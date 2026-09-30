issues=[]
def check(a,b,label,**kw):
 vol=overlap(a,b)
 if vol>.1:issues.append(dict(test=label,mm3=vol,**kw))
check(foot,pitchservo,'foot_vs_servo_mount')
check(turret,pitchservo,'fork_vs_servo_neutral')
check(turret,foot,'fork_vs_foot_neutral')
for ang in range(-45,46,5):
 m=rot(ang,(38,0,27),(0,1,0))
 for label,b in [('servo',pitchservo),('foot',foot)]:check(tx(b,m),turret,'pitch_sweep',angle=ang,part=label)
 adsk.doEvents()
fixed=[(o.component.name,b) for o,b,k in parts if not o.component.name.startswith(('FL','FR','RL','RR')) or 'yaw servo fixed' in o.component.name]
pm=ports['FL']
for ang in range(-135,136,15):
 m=rot(ang)
 for label,b in [('fork',turret),('servo',pitchservo),('foot',foot)]:
  moving=tx(tx(b,m),pm)
  for name,fb in fixed:check(moving,fb,'yaw_sweep_neutral_pitch',angle=ang,part=label,target=name)
 adsk.doEvents()
# Package envelopes must also clear stationary electronics and cradles.
for so,sb,sk in parts:
 if sk not in ['ir_sensor','camera_sensor','imu']:continue
 for oo,ob,ok in parts:
  if oo==so or oo.component.name.startswith(('FL','FR','RL','RR')):continue
  check(sb,ob,'sensor_static_packaging',sensor=so.component.name,target=oo.component.name)
# Check the new fixed sensor packages and mounts against all four moving legs.
sensor_fixed=[(o.component.name,b) for o,b,k in parts if 'sensor' in o.component.name.lower() or 'cradle' in o.component.name.lower() or 'MC6470' in o.component.name]
for leg,port in ports.items():
 for yaw in [-45,0,45]:
  for pitch in [-45,0,45]:
   for label,geo in [('fork',turret),('servo',pitchservo),('foot',foot)]:
    moving=geo if label=='fork' else tx(geo,rot(pitch,(38,0,27),(0,1,0)))
    moving=tx(tx(moving,rot(yaw)),port)
    for name,fb in sensor_fixed:check(moving,fb,'sensor_clearance',leg=leg,yaw=yaw,pitch=pitch,part=label,target=name)
   adsk.doEvents()
report=dict(issues=issues,scope='Own pitch assembly at 5 degree steps; FL yaw vs platform and fixed servos at 15 degree steps. Sensor packages/mounts checked against all legs at yaw and pitch -45/0/+45. Adjacent moving legs, wires, gear mesh and strength not validated.')
(OUT/'B3_clearance_audit.json').write_text(json.dumps(report,indent=2))
note('AUDIT '+str(len(issues)))
lo=[min(i['min_mm'][j] for i in inventory) for j in range(3)];hi=[max(i['max_mm'][j] for i in inventory) for j in range(3)]
mass=sum(i['volume_cm3']*1.24 for i in inventory if i['kind']=='PLA')
(OUT/'B3_size_mass.json').write_text(json.dumps(dict(neutral_size_mm=[b-a for a,b in zip(lo,hi)],solid_PLA_g=mass,servo_count=8,servo_model='Parallax 900-00005',mass_note='Solid PLA geometry only, not sliced or measured. No motor upgrade.'),indent=2))
# Detail copy shows fixed horn versus moving servo/foot clearly.
robotdoc=nd
copies=[(o.component.name,b.name,tx(b,inv(pm)),b.appearance) for o,b,k in parts if o.component.name.startswith('FL B3')]
detail=app.documents.add(C.DocumentTypes.FusionDesignDocumentType);detail.name='B3 Y fork detail - top yaw and leg aligned pitch';dd=F.Design.cast(app.activeProduct);dd.designType=F.DesignTypes.DirectDesignType;rr=dd.rootComponent
for cn,bn,geo,ap in copies:
 o=rr.occurrences.addNewComponent(C.Matrix3D.create());o.component.name=cn;b=o.component.bRepBodies.add(geo);b.name=bn;b.appearance=ap
save(dd,rr,'B3_Y_Fork_Detail');render('B3_Y_Fork_Detail',(170,-220,135),(15,0,0))
robotdoc.activate()
