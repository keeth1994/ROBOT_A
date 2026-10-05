# CAD versions and build workflow

B5 is the current mechanical revision. The simulation remains B3.

| Path | Role |
|---|---|
| `B5_plain_pivots/` | Current Fusion/STEP assembly, preview, fit report and build source |
| `B5Builder/` | Dedicated Fusion script launcher for B5 |
| `fit_audit_20261005/User_updated_rocker.f3d` | Preserved user-edited rocker; required B5 input |
| `B4_reinforced_harness/Robot_B4_Reinforced_Harness.f3d` | Required B5 source assembly |
| `B4_reinforced_harness/` | Prior revision and its audit evidence |
| `InspectFit/` | B4 inspection scripts; not the B5 build launcher |
| `Rocker update.3mf` | Original user-provided mesh; preserved unchanged |
| `current_design.json` | Machine-readable CAD/simulation revision and source hashes |

## Rebuild B5

In Fusion, open Scripts and Add-Ins, add the `cad/B5Builder` folder from this checkout and run B5Builder. It creates a separate document, preserves its source archives, checks fit and exports to `cad/B5_plain_pivots/`. Paths are resolved from the checkout, so the launcher works if the repository is moved.

A successful run writes `complete.txt`; a failure writes `error.txt`. Old status markers are cleared when a build starts. Existing exports may remain after a failed run: require the new completion marker and inspect the fit report before using them.

See [B5 assembly and limitations](B5_plain_pivots/README.md). B5 removes separate bearings/bushings but still needs screws, nuts and passive pitch axles. Servo internals are unchanged. Horn attachment dimensions remain unverified.

## Simulation boundary

`robot.py build` reads the B3 reference meshes and regenerates B3 MuJoCo assets. It does not run Fusion or ingest B5. Keep existing B3 experiments and policies identifiable until B5 has its own geometry/mass export and friction assumptions, followed by revalidation. Do not rename old results to B5.
