# B9-1 assembly

`Robot_B9_1.f3d` and `Robot_B9_1.step` contain the full assembly. `assembly.png` shows the model. The current edited fork source is retained in `User_yaw_backup.f3d`.

The chassis underside is extended 4 mm to the bearing-housing base plane, retaining openings and bearing seats. Added CAD volume is approximately 48.9 cm3.

`check_report.json` records 784 neutral and sampled pitch clearance checks with no intersections above 0.1 mm3 in the checked pairs. Pitch was sampled from -45 to +45 degrees. Full yaw clearance, physical fit, access and strength remain unverified.

The assembly is transferred to MuJoCo. See [simulation notes](../../docs/DESIGN_B91_SIM.md).
