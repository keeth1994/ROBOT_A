from cad_source_a14 import load_cad_source as previous
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
def load_cad_source():
 s=previous();p=ROOT/'reference/A15_mesh_patch.json';patch=json.loads(p.read_text());assert patch['base']==s['source']
 edits={(b['component'],b['body']):b for b in patch['bodies']};removed={(b['component'],b['body']) for b in patch['removed']}
 records=[]
 for b in s['bodies']:
  k=(b['component'],b['body'])
  if k in removed:removed.remove(k);continue
  records.append(edits.pop(k,b))
 assert not edits and not removed
 s['bodies']=records;s['source']=patch['source'];s['patch_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();return s
