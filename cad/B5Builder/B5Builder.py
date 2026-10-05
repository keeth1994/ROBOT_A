"""Fusion launcher for the current B5 CAD revision."""
from pathlib import Path
import runpy

def run(context):
    source = Path(__file__).resolve().parents[1] / "B5_plain_pivots" / "build_b5.py"
    runpy.run_path(str(source))["run"](context)
