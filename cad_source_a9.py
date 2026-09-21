"""A9 vertical corner mounts and octagonal platform, derived from preserved A8 geometry."""
from pathlib import Path
import json,hashlib
import numpy as np
from cad_source_a8 import load_cad_source as load_a8
ROOT=Path(__file__).resolve().parent
def load_cad_source():
    source=load_a8();path=ROOT/'reference/A9_mesh_patch.json';patch=json.loads(path.read_text(encoding='utf8'))
    assert source['source']==patch['base']
    axes=json.loads((ROOT/'reference/Robot_A8_simulering.json').read_text())['new_hinges']
    hips={a['name'][:2]:a for a in axes if a['name'].endswith('hip_pitch')}
    angles={'FL':45,'RL':135,'FR':-45,'RR':-135};records=[]
    changed={(r['component'],r['body']):r for r in patch['changed_leg_bodies']}
    for r in source['bodies']:
        leg=r['component'][:2]
        if leg not in angles:continue
        key=(r['component'],r['body'])
        if key in changed:records.append(changed.pop(key));continue
        r=dict(r);vertices=np.asarray(r['vertices']);com=np.asarray(r['com_m'])
        if r['component'].startswith(leg+' 01') and 'FAST' in r['component']:
            h=hips[leg];axis=np.asarray(h['axis_world']);axis=axis/np.linalg.norm(axis);x,y,z=axis
            skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);a=np.deg2rad(patch['hip_fixed_rotation_deg'])
            rot=np.eye(3)+np.sin(a)*skew+(1-np.cos(a))*(skew@skew);origin=np.asarray(h['origin_world_m'])
            vertices=(vertices-origin)@rot.T+origin;com=rot@(com-origin)+origin
        a=np.deg2rad(angles[leg]);delta=np.array([np.cos(a)*patch['radial_translation_mm']/1000,np.sin(a)*patch['radial_translation_mm']/1000,patch['vertical_translation_mm']/1000])
        r['vertices']=(vertices+delta).tolist();r['com_m']=(com+delta).tolist();records.append(r)
    assert not changed,changed.keys()
    records=patch['platform']+records
    inventory=json.loads((ROOT/'reference/A9_body_inventory.json').read_text())
    expected={(r['component'],r['body']):r for r in inventory}
    assert len(expected)==len(records)==len(inventory)
    max_com_error=0
    for r in records:
        check=expected[(r['component'],r['body'])];error=np.linalg.norm(np.array(r['com_m'])-check['com_m']);max_com_error=max(max_com_error,error)
        assert error<1e-6,(r['component'],r['body'],error)
        assert abs(r['volume_m3']-check['volume_m3'])<1e-10,(r['component'],r['body'])
    source['bodies']=records;source['source']=patch['source'];source['a9_com_error_m']=max_com_error
    source['previous_patch_sha256']=source['patch_sha256'];source['patch_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    return source
