"""Revision A7.2: eight edited yaw solids over the preserved A7.1 CAD export."""
from pathlib import Path
import gzip, json, hashlib
ROOT=Path(__file__).resolve().parent

def load_cad_source():
    base=ROOT/'reference/a7_1_cad_meshes.json.gz'
    patchfile=ROOT/'reference/a7_2_yaw_mesh_patch.json'
    with gzip.open(base,'rt',encoding='utf8') as f:source=json.load(f)
    patch=json.loads(patchfile.read_text(encoding='utf8'))
    assert source['source']==patch['base']=='Robot_A7_1_Hjullager.f3d'
    assert patch['source']=='Robot_A7_2_Yaw180.f3d'
    replacements={}
    for r in patch['bodies']:
        key=(r['component'][:2],r['body'].replace('A7.2 Y180 ','A7 '))
        assert key not in replacements
        replacements[key]=r
    assert len(replacements)==8
    hits=0;records=[]
    for r in source['bodies']:
        key=(r['component'][:2],r['body'])
        if '02 YAW F40 FAST' in r['component'] and key in replacements:
            records.append(replacements[key]);hits+=1
        else:records.append(r)
    assert hits==8
    source['bodies']=records;source['source']=patch['source']
    source['base_sha256']=hashlib.sha256(base.read_bytes()).hexdigest()
    source['patch_sha256']=hashlib.sha256(patchfile.read_bytes()).hexdigest()
    return source
