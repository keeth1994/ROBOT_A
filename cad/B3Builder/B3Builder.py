"""Run from Fusion Scripts and Add-Ins; rebuild the current B3 archive."""
from pathlib import Path
import traceback

def run(context):
    import adsk.core
    script=Path(__file__).resolve().parent.parent/'build_b3_fusion.py'
    try:
        exec(compile(script.read_text(encoding='utf8'),str(script),'exec'),{'__name__':'__main__','__file__':str(script)})
    except Exception:
        adsk.core.Application.get().userInterface.messageBox(traceback.format_exc())
