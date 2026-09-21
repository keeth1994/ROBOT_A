from pathlib import Path
import gzip,json,hashlib
ROOT=Path(__file__).resolve().parent
def load_cad_source():
 p=ROOT/'reference/a16_cad_meshes.json.gz'
 with gzip.open(p,'rt',encoding='utf8') as f:s=json.load(f)
 s.update(base_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),patch_sha256='',a9_com_error_m=0.)
 return s
