# B4 reinforced servo harness

Open `Robot_B4_Reinforced_Harness.f3d` for the assembly, or `B4_Reinforced_Rocker.f3d` to inspect the four mounting ties without the servo hiding them. `B4_Reinforced_Leg_Detail.f3d` includes the servo and yaw fork.

Changes apply to all four legs:
- Servo mounting plate: 2 to 3 mm nominal thickness, with existing servo clearance retained.
- Four corner bridges at local x=25/51 mm and z=-9/44 mm; nominal 6 x 6 mm section.
- 6 mm diameter diagonal ribs at the upper corners; lower ties join the existing leg struts.
- Original servo opening, mounting-hole centers, rocker soles and joint centers retained.
- Fork clearance relief regenerated for the thicker mount.

Each rocker remains one connected solid. All four bridges retain a checked 4 x 4 mm solid core after clearance cuts; see `harness_ties.json`.

No intersections above the audit threshold were found for the own pitch sweep at 5-degree intervals from -45 to +45 degrees, or for the neutral-pitch FL yaw samples within -45 to +45 degrees. Sensor checks at the existing sample poses also passed. Larger yaw angles still collide with fixed components; see the complete audit. This is a sampled geometry check, not continuous-motion or load validation.

Solid CAD PLA volume implies about 2.78 g more per rocker. Including slight additional fork relief, the complete assembly gains about 11.09 g at the existing 1.24 g/cm3 assumption. Overall neutral bounds remain 241.18 x 241.18 x 106 mm.

This is a separate CAD revision. The active B3 simulation, source configuration and learned policies have not been replaced. B4 must be transferred and revalidated before presenting old policy results as B4 results.

Run `cad/B4Builder` from Fusion Scripts and Add-Ins to rebuild. It uses the shared B3 generator with the reinforcement flag and a separate output directory.
