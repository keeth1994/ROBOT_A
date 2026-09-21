from cad_source_a10 import load_cad_source as previous
from pathlib import Path
import json,hashlib,numpy as np
ROOT=Path(__file__).resolve().parent
def load_cad_source():
 s=previous()
 for name,count in [('A11',3),('A12',48)]:
  patch=json.loads((ROOT/'reference'/f'{name}_mesh_patch.json').read_text());assert patch['base']==s['source']
  edits={(b['component'],b['body']):b for b in patch['bodies']};assert len(edits)==count
  s['bodies']=[edits.pop((b['component'],b['body']),b) for b in s['bodies']];assert not edits;s['source']=patch['source']
 p=ROOT/'reference/A13_mesh_patch.json';patch=json.loads(p.read_text());edits={(b['component'],b['body']):b for b in patch['bodies']};assert len(edits)==12
 inventory=json.loads((ROOT/'reference/A13_body_inventory.json').read_text());expected={(b['component'],b['body']):b for b in inventory};assert len(expected)==len(s['bodies'])
 records=[]
 for b in s['bodies']:
  key=(b['component'],b['body']);e=expected[key]
  if key in edits:b=edits.pop(key)
  else:
   assert abs(b['volume_m3']-e['volume_m3'])<1e-10,key
   delta=np.array(e['com_m'])-b['com_m'];assert min(np.linalg.norm(delta),np.linalg.norm(delta-[0,0,.0175]))<1e-6,(key,delta)
   b=dict(b,vertices=(np.array(b['vertices'])+delta).tolist(),com_m=e['com_m'])
  assert abs(b['volume_m3']-e['volume_m3'])<1e-10 and np.linalg.norm(np.array(b['com_m'])-e['com_m'])<1e-6,key
  records.append(b)
 assert not edits
 s['bodies']=records;s['source']=patch['source'];s['patch_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();return s
