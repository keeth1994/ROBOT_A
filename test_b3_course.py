"""Repeatable B3 course test: no base actuation, teleporting, or automatic resets."""
from pathlib import Path
import argparse,json,math
import numpy as np
import mujoco
from run_robot_b3 import Robot,ROOT
from course_b3 import local_points

def test(params,seconds=80,course=True):
 settings=json.loads((ROOT/'reference/b3_course.json').read_text())
 lo=settings['approach_end_x']+.01;hi=settings['downhill_end_x']-.01;half=settings['width_m']/2
 r=Robot(params,course);trace=[];terrain=['floor']+settings['obstacle_geoms'];seen=set();first={};bad=0;steps=0;sat=0;max_tilt=0;max_x=-1e9;edge=False;max_pen=0;start=None;overhang=0.;offside_steps=0
 tread_ids=[g for g in range(r.m.ngeom) if '_tread_' in (mujoco.mj_id2name(r.m,mujoco.mjtObj.mjOBJ_GEOM,g) or '')]
 tread_vertices={}
 for g in tread_ids:
  mid=r.m.geom_dataid[g];adr=r.m.mesh_vertadr[mid];n=r.m.mesh_vertnum[mid];tread_vertices[g]=r.m.mesh_vert[adr:adr+n].copy()
 for step in range(round(seconds/r.m.opt.timestep)):
  r.step()
  if r.d.time<2:continue
  if start is None:start=r.d.qpos[:3].copy()
  pos=r.d.qpos[:3];tilt=r.tilt();max_tilt=max(max_tilt,tilt);max_x=max(max_x,float(pos[0]));steps+=1;sat+=int(np.count_nonzero(np.abs(r.torque)>.95*r.limit))
  bad_step=False;offside_step=False
  for c in r.d.contact:
   names=[mujoco.mj_id2name(r.m,mujoco.mjtObj.mjOBJ_GEOM,g) or '' for g in [c.geom1,c.geom2]]
   active=[n for n in names if n in terrain]
   if c.dist<=.0005 and active:
    surface=active[0]
    if any('_tread_' in n for n in names):seen.add(surface);first.setdefault(surface,float(r.d.time))
    else:bad_step=True
    max_pen=max(max_pen,float(-c.dist))
    if course and surface=='floor' and lo<local_points(c.pos,settings)[0]<hi and abs(local_points(c.pos,settings)[1])>half:offside_step=True
  bad+=bad_step
  # Tread footprint, not merely the base centre, must stay within the ramp edges.
  offside_steps+=offside_step
  if course and step%10==0:
   for g in tread_ids:
    vv=tread_vertices[g]@r.d.geom_xmat[g].reshape(3,3).T+r.d.geom_xpos[g]
    vv=local_points(vv,settings)
    mask=(vv[:,0]>lo)&(vv[:,0]<hi)
    if mask.any():overhang=max(overhang,float(np.max(np.abs(vv[mask,1]))-half))
   edge=overhang>.001
  if step%50==0:trace.append(dict(time_s=float(r.d.time),x=float(pos[0]),y=float(pos[1]),z=float(pos[2]),tilt_deg=tilt))
  if not np.isfinite(r.d.qpos).all():raise RuntimeError('Nonfinite physics')
 final=r.d.qpos[:3].copy();warnings=int(sum(w.number for w in r.d.warning))
 passed=bool(course and set(settings['required_surfaces'])<=seen and local_points(final,settings)[0]>settings['success_base_x'] and max_tilt<45 and not edge and warnings==0 and bad==0)
 return dict(cad_revision=json.loads((ROOT/'reference/b3_kinematics.json').read_text())['cad_revision'],course_geometry=settings if course else None,controller="physics-driven programmed gait with ideal pose feedback",parameters=params,duration_s=seconds,course=course,mass_kg=float(sum(r.m.body_mass)),start_xyz=start.tolist(),final_xyz=final.tolist(),forward_m=float(final[0]-start[0]),max_x_m=max_x,max_tilt_deg=max_tilt,fallen=max_tilt>45,tread_contacts=sorted(seen),first_tread_contact_s=first,left_ramp_width=edge,max_tread_overhang_m=overhang,offside_floor_contact_fraction=offside_steps/max(steps,1),nonfoot_contact_fraction=bad/max(steps,1),stall_saturation_fraction=sat/max(8*steps,1),max_contact_penetration_m=max_pen,warnings=warnings,course_passed=passed,pass_scope="ramp only; corridor navigation not assessed",flat_passed=bool(not course and final[0]-start[0]>.5 and max_tilt<45 and bad==0 and warnings==0),trace=trace,torque_limits_Nm=r.limit.tolist(),limitations=['Approximate contact proxies and box inertias','Rigid 300 mm ramp; flexible real panel is not modelled','Assumed friction 0.8 and gear efficiency 0.85','Peak servo model, no thermal limits or voltage sag','Heading and optional lateral feedback use ideal simulated pose','No proof of full CAD clearance at combined joint poses'])

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--flat',action='store_true');ap.add_argument('--seconds',type=float,default=80);ap.add_argument('--params',default=str(ROOT/'reference/b3_gait.json'));ap.add_argument('--out');a=ap.parse_args()
 p=json.loads(Path(a.params).read_text());result=test(p,a.seconds,not a.flat)
 out=Path(a.out) if a.out else ROOT/'results'/('b3_flat_test.json' if a.flat else 'b3_course_test.json');out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))
