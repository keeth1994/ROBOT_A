import runpy
from pathlib import Path
def run(context):
 runpy.run_path(str(Path(__file__).resolve().parents[1]/'export_b91_sim.py'))['run'](context)
