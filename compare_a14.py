import json,numpy as np
from pathlib import Path
from run_robot_a13 import Robot as A13
from run_robot_a14 import Robot as A14
rows=[]
for label,cls,factor in [('A13',A13,None),('A14_single_hip_comparison',A14,1),('A14_dual_hip',A14,2)]:
 r=cls();r.set_mode('physics')
 if factor:r.multipliers[::4]=factor
 peak=np.zeros(16);sat=np.zeros(16);z=[]
 for i in range(2500):
  r.step();peak=np.maximum(peak,np.abs(r.last_torque));sat+=(np.abs(r.last_torque)>=r.last_available*.999)&(r.last_available>0);z.append(float(r.data.qpos[2]))
 rows.append(dict(case=label,mass_kg=float(r.model.body_mass.sum()),seconds=float(r.data.time),base_z_m=float(r.data.qpos[2]),joint_deg=np.degrees(r.data.qpos[r.qadr]).tolist(),peak_torque_Nm=peak.tolist(),saturation_fraction=(sat/2500).tolist(),warnings=r.data.warning.number.tolist()))
 print(label,rows[-1]['base_z_m'],flush=True)
Path('results/a14_comparison.json').write_text(json.dumps(rows,indent=2))
