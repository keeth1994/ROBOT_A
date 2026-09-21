from cad_source_a9 import load_cad_source as previous
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
def load_cad_source():
 s=previous();p=ROOT/'reference/A10_mesh_patch.json';patch=json.loads(p.read_text());assert s['source']==patch['base']
 edits={(b['component'],b['body']):b for b in patch['bodies']};assert len(edits)==12
 records=[];saved=0
 for b in s['bodies']:
  key=(b['component'],b['body'])
  if key in edits:
   n=edits.pop(key);assert 0<n['volume_m3']<b['volume_m3'];saved+=b['volume_m3']-n['volume_m3'];b=n
  records.append(b)
 assert not edits and abs(saved-264.863958873529e-6)<1e-10
 s['bodies']=records;s['source']=patch['source'];s['patch_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();return s
