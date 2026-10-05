# B5 â€” integral plain pivots

B5 removes the four separate yaw bearings and four separate pitch bushings. It uses direct sliding contact at the joints. The servo internals are unchanged.

Open `Robot_B5_Plain_Pivots.f3d` in Fusion or use the STEP assembly. The user's updated rocker is retained, with local clearance cuts around the inherited pitch horn and passive axle overlaps. The B4 originals and the user's archived rocker remain intact.

## Mechanical changes

- Four old 21.1 mm through-bores are filled by integral chassis material, leaving 12.4 mm bores and a 12.7 mm guide length.
- Each yaw fork has a 12 mm journal. Nominal diametral clearance is 0.4 mm; this is a starting allowance, not verified physical fit.
- The driven gear underside forms the upper thrust shoulder. An 18 mm diameter, 2 mm thick removable retainer captures the journal below the chassis.
- At the nominal position there is 0.3 mm clearance above and below the chassis guide (0.6 mm total axial travel before contact). Adjust only after checking actual free movement and backlash.
- Retention uses an M3 x 25 mm screw from below and an M3 hex nut inserted through the top of the fork hub. The hex pocket is 5.7 mm across flats and 2.7 mm deep; the top access well is 6.8 mm diameter. Select a nut no thicker than 2.7 mm. Screw length is measured from beneath the head. These fasteners are specified but not represented by detailed solids.
- The retainer tightens against the end of the rotating journal, not against the chassis. Check that it stays secure without clamping rotation.
- The passive pitch side now has an integral 4.4 mm bore for the existing 4 mm axle. It has no separate bushing. Axle head/nut retention remains to be detailed against the actual hardware.

## Assembly order

1. Insert the yaw fork journal into the chassis from above.
2. Drop the M3 nut into the top access well and seat it in its hex pocket.
3. Fit the retainer below and install the M3 x 25 screw. Confirm free rotation and controlled end play before fitting a servo.
4. Install the yaw servo/driver and pitch servo/rocker assemblies. The stock horn hole-spacing issue from the fit inspection is still pending a physical measurement; B5 does not silently guess a replacement spacing.
5. Fit the passive pitch axle and check full travel without binding.

## Validation and limitations

`fit_report.json` records 99 sampled CAD checks: rocker versus fork at pitch -45 to +45 degrees in 5-degree steps; each yaw fork versus chassis at yaw -45 to +45 degrees; and each lower retainer versus chassis. All passed with no intersections above 0.1 mmÂ³. Each exported component is a single solid.

This does not prove acceptable friction, wear, strength, servo torque margin, fastener security or the complete combined motion envelope. Direct sliding pivots need physical fit and load testing. The current MuJoCo model and trained policies remain B3 and do not represent these new friction surfaces or B5 mass.

Build launcher: register `cad/B5Builder` in Fusion Scripts and Add-Ins and run B5Builder. Build source: `build_b5.py`. It imports preserved B4 and user-rocker archives, builds a separate document and exports B5. It does not rebuild the user's rocker from the old parameter set.
