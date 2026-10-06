"""Check current entry points and configuration paths."""
import json, subprocess, sys
from robot_b91.paths import ROOT

def main():
    import robot
    for command in robot.COMMANDS:
        run=subprocess.run([sys.executable,str(ROOT/'robot.py'),command,'--help'],cwd=ROOT.parent,capture_output=True,text=True,timeout=30)
        assert run.returncode == 0,(command,run.stderr)
    for task in json.loads((ROOT/'.vscode/tasks.json').read_text())['tasks']:
        assert task['args'][1] in robot.COMMANDS or task['args'][1]=='check'
    cfg=json.loads((ROOT/'cad/current_design.json').read_text())
    for key in ['assembly','step','fit_report','simulation_model','simulation_course','simulation_manifest']:
        assert (ROOT/cfg[key]).is_file(),key
    print('PASS: current command help, outside-repo launch, tasks and asset paths.')
if __name__=='__main__':main()
