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
        "cad/B10_1/Robot_B10_1.f3d",
        "cad/B10_1/Robot_B10_1.step",
        "models/robot_b10.xml",
        "models/robot_b10_course.xml",
        "reference/robot_b10_manifest.json",
        "reference/b91_gait.toml",
        "servo_calibration/measurements.json",
    ):
        assert (ROOT / path).is_file(), path
    print("PASS: current command help, outside-repo launch, tasks and asset paths.")


if __name__ == "__main__":
    main()
