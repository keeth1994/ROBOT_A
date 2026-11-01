"""Check current entry points and configuration paths."""

import json, subprocess, sys
from robot_b91.paths import ROOT


def main():
    import robot

    for command in robot.COMMANDS:
        run = subprocess.run(
            [sys.executable, str(ROOT / "robot.py"), command, "--help"],
            cwd=ROOT.parent,
            capture_output=True,
            text=True,
            timeout=30,
        )
        assert run.returncode == 0, (command, run.stderr)
    # Editor configuration is optional and ignored by git.
    tasks = ROOT / ".vscode/tasks.json"
    if tasks.exists():
        for task in json.loads(tasks.read_text())["tasks"]:
            assert task["args"][1] in robot.COMMANDS or task["args"][1] == "check"
    for path in (
        "cad/B9_8/Robot_B9_8.f3d",
        "cad/B9_8/Robot_B9_8.step",
        "cad/B9_8/source/User_assembly_base.step",
        "models/robot_b98.xml",
        "models/robot_b98_course.xml",
        "reference/robot_b98_manifest.json",
        "dashboard/walking.toml",
        "servo_calibration/measurements.json",
    ):
        assert (ROOT / path).is_file(), path
    print("PASS: current command help, outside-repo launch, tasks and asset paths.")


if __name__ == "__main__":
    main()
