"""A8 replacement solids over the preserved A7.2 CAD snapshot."""
from pathlib import Path
import json,hashlib
import numpy as np
from cad_source import load_cad_source as load_a7_2
ROOT=Path(__file__).resolve().parent

def load_cad_source():
    source=load_a7_2();path=ROOT/'reference/a8_mesh_patch.json';patch=json.loads(path.read_text(encoding='utf8'))
    assert source['source']==patch['base']
    records=source['bodies'];used=set();deleted=set();skipped=[]
    for change in patch['replacements']:
        old,new=change['old'],change['new']
        candidates=[i for i,r in enumerate(records) if i not in used and r['component'].replace(' - A7.2 180deg','')==old['component'].replace(' - A7.2 180deg','') and r['body']==old['body']]
        if not candidates:
            assert '04 BEVART 11 ' in old['component'],old
            skipped.append(old['component']);continue
        i=min(candidates,key=lambda j:np.linalg.norm(np.array(records[j]['com_m'])-old['com_m']))
        # Temporary Fusion BRep copies in the first export report zero COM.
        # These names are unique; require a real COM to disambiguate duplicates.
        if len(candidates)>1 or np.linalg.norm(old['com_m'])>1e-12:
            assert np.linalg.norm(np.array(records[i]['com_m'])-old['com_m'])<1e-6,(i,old)
        used.add(i)
        if new is None:deleted.add(i);continue
        if 'rigid_from_old' in new:
            new=dict(new);tr=new.pop('rigid_from_old');old_record=records[i]
            vertices=np.asarray(old_record['vertices'],dtype=float)
            com=np.asarray(old_record['com_m'],dtype=float)
            if 'rotation_deg' in tr:
                axis=np.asarray(tr['axis'],dtype=float);axis/=np.linalg.norm(axis)
                x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                a=np.deg2rad(tr['rotation_deg']);rot=np.eye(3)+np.sin(a)*cross+(1-np.cos(a))*(cross@cross)
                origin=np.asarray(tr['origin_m'])
                vertices=(vertices-origin)@rot.T+origin;com=rot@(com-origin)+origin
            delta=np.asarray(tr['translation_m']);vertices+=delta;com+=delta
            assert np.linalg.norm(com-new['com_m'])<1e-6,(new['component'],new['body'],com,new['com_m'])
            new['vertices']=vertices.tolist();new['indices']=old_record['indices']
        records[i]=new
    source['bodies']=[r for i,r in enumerate(records) if i not in deleted]
    refine_path=ROOT/'reference/A8_refine_patch.json'
    refine=json.loads(refine_path.read_text(encoding='utf8'))
    angles={'FL':45,'RL':135,'FR':-45,'RR':-135}
    for r in source['bodies']:
        leg=r['component'][:2]
        if leg in angles:
            a=np.deg2rad(angles[leg]);delta=refine['radial_offset_mm']/1000*np.array([np.cos(a),np.sin(a),0])
            r['vertices']=(np.asarray(r['vertices'])+delta).tolist()
            r['com_m']=(np.asarray(r['com_m'])+delta).tolist()
    refine_deleted=set()
    for change in refine['replacements']:
        old=change['old'];candidates=[i for i,r in enumerate(source['bodies']) if r['component']==old['component'] and r['body']==old['body']]
        assert len(candidates)==1,old
        i=candidates[0];assert np.linalg.norm(np.array(source['bodies'][i]['com_m'])-old['com_m'])<1e-6
        if change['new'] is None:refine_deleted.add(i)
        else:source['bodies'][i]=change['new']
    source['bodies']=[r for i,r in enumerate(source['bodies']) if i not in refine_deleted]+refine.get('additions',[])
    source['refine_patch_sha256']=hashlib.sha256(refine_path.read_bytes()).hexdigest()
    source['source']=patch['source'];source['previous_patch_sha256']=source['patch_sha256'];source['patch_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    source['a8_patch_audit']={'matched':len(used),'deleted':len(deleted),'hidden_spring_records_skipped':len(skipped)}
    return source
