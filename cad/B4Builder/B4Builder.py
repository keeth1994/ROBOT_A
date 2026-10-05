from pathlib import Path
import traceback

def run(context):
    import adsk.core
    base=Path(__file__).resolve().parents[2]
    script=base/'cad/build_b3_fusion.py'
    try:
        exec(compile(script.read_text(),str(script),'exec'),{
            '__name__':'__main__','__file__':str(script),'REINFORCED':True,
            'OUTPUT_DIR':base/'cad/B4_reinforced_harness',
            'ASSEMBLY_NAME':'Robot_B4_Reinforced_Harness','DETAIL_NAME':'B4_Reinforced_Leg_Detail'})
    except Exception:
        (base/'logs/b4_error.log').write_text(traceback.format_exc())
        adsk.core.Application.get().userInterface.messageBox(traceback.format_exc())
