"""B9-1 transfer checks: provenance, links, axes, mass, contacts and motor limits."""
import json,hashlib
import numpy as np
import mujoco
from robot_b91.paths import ROOT
from robot_b91.geometry import rz
from robot_b91.run import Robot

def main():
 manifest=json.loads((ROOT/'reference/robot_b91_manifest.json').read_text())
 cfg=json.loads((ROOT/'reference/b91_kinematics.json').read_text())
 assert hashlib.sha256((ROOT/'cad/B9_1/Robot_B9_1.f3d').read_bytes()).hexdigest()==manifest['source_sha256']
 r=Robot();assert r.m.nu==8 and r.m.nq==19
 assert abs(r.m.body_mass.sum()-manifest['total_mass_kg'])<1e-8
 assert len(manifest['parts'])==97
 for leg,k in cfg['legs'].items():
  expected=np.array(k['yaw_center_mm'])/1000+rz(k['neutral_yaw_deg'])@np.array([.055,0,.035])
  np.testing.assert_allclose(r.d.xanchor[r.m.joint(leg+'_pitch').id]-r.d.qpos[:3],expected,atol=1e-8)
  assert next(x for x in manifest['parts'] if x['component'].startswith(leg+' ') and 'pitch servo' in x['component'])['link']==leg+'_pitch'
  assert next(x for x in manifest['parts'] if x['component'].startswith(leg+' ') and 'pitch horn' in x['component'])['link']==leg+'_yaw'
 for _ in range(1500):r.step(False)
 assert r.tilt()<1 and np.isfinite(r.d.qpos).all()
 assert not sum(w.number for w in r.d.warning)
 for c in r.d.contact:
  names=[r.m.geom(g).name for g in (c.geom1,c.geom2)]
  if 'floor' in names:assert any('_tread_' in n for n in names),names
 for _ in range(500):
  r.step_targets(np.tile([.15,-.2],4));assert np.all(np.abs(r.torque)<=r.limit+1e-12)
 assert np.max(np.abs(r.d.qpos[r.qa]))>.05
 assert np.isfinite(r.d.qpos).all() and not sum(w.number for w in r.d.warning)
 course=Robot(ramp=True);assert course.m.ngeom>r.m.ngeom
 print('PASS: B9-1 source hash, 97 bodies, axes, mass, standing contacts, actuation, torque limits and course load.')
if __name__=='__main__':main()
