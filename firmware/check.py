"""Compile native C++ tests and compare the firmware waveform to the simulated gait."""
import subprocess
import sys
import os
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_b91.gait import rocker_targets
from robot_b91.run import load_gait_parameters


def main():
    build = ROOT / "firmware/.pio/native"
    build.mkdir(parents=True, exist_ok=True)
    exe = build / "control_test.exe"
    env = os.environ | {"ZIG_GLOBAL_CACHE_DIR": str(build / "global-cache"),
                        "ZIG_LOCAL_CACHE_DIR": str(build / "local-cache")}
    subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O2",
                    "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "firmware/include"),
                    str(ROOT / "firmware/tests/control_test.cpp"), "-o", str(exe)], check=True, env=env)
    subprocess.run([str(exe)], check=True)
    p, _ = load_gait_parameters(ROOT / "reference/b91_gait.toml", "course")
    rng = np.random.default_rng(910)
    rows = np.column_stack((rng.uniform(0, 120, 1000), rng.uniform(-1, 1, 1000), rng.uniform(-.2, .2, 1000)))
    rows = np.vstack(([[0, 0, 0], [.6, 0, 0], [1, 0, 0]], rows))
    text = "\n".join(" ".join(map(str, row)) for row in rows) + "\n"
    result = subprocess.run([str(exe), "waveform"], input=text, capture_output=True, text=True, check=True)
    actual = np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
    expected = np.array([rocker_targets(p, t, t, h, y) for t, h, y in rows])
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
    print(f"PASS: firmware matches the simulated course gait for {len(rows)} time/feedback combinations")


if __name__ == "__main__":
    main()
