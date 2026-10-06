# B9-1 CAD

Open [Robot_B9_1.f3d](B9_1/Robot_B9_1.f3d) in Fusion, or use the [STEP assembly](B9_1/Robot_B9_1.step). The saved assembly is the starting point for edits.

`B9_1/User_yaw_backup.f3d` preserves the source of the current edited fork. Component names inside the assembly retain their original identifiers for traceability.

For simulation export, register `ExportB91` in Fusion Scripts and Add-Ins. It reads the saved archive and writes `reference/b91_cad_meshes.json.gz`. Then run `robot.py build` to regenerate MuJoCo assets. The exporter does not capture unsaved active-tab changes.

See [mechanical notes](B9_1/README.md) and [simulation notes](../docs/DESIGN_B91_SIM.md).
