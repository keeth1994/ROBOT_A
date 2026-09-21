from cad_source_a13 import load_cad_source as previous
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
def load_cad_source():
 s=previous();p=ROOT/'reference/A14_mesh_patch.json';patch=json.loads(p.read_text());assert patch['base']==s['source']
 edits={(b['component'],b['body']):b for b in patch['bodies']}
 records=[edits.pop((b['component'],b['body']),b) for b in s['bodies']]
 assert len(edits)==12
 records.extend(edits.values());s['bodies']=records;s['source']=patch['source'];s['patch_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();return s
