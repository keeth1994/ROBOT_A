# B4 mechanical fit inspection — 5 October 2026

Status: inspection complete for the scope below; hardware interfaces are not assembly-ready. No design geometry was changed. User edits were preserved from the active Fusion document, including its timeline, in `cad/fit_audit_20261005/User_updated_rocker.f3d` and STEP.

## Updated rocker

- One connected solid; bounding envelope 38 × 57 × 79 mm.
- Volume 16,393.30 mm³ versus 16,061.89 mm³ in the previous B4 rocker: +2.06%, approximately +0.41 g per rocker using the existing 1.24 g/cm³ material assumption. This is a geometry estimate, not a measured part mass.
- Four servo-ear holes remain Ø4.5 mm on a 10 × 50.5 mm rectangular pattern. They match the existing modeled servo casing. Physical servo dimensions still need confirmation.
- User's added material and rounded transitions are preserved. Connected geometry and better-looking transitions alone do not establish load capacity.
- Checked pitch from −45° to +45° at 5° intervals against the existing Y-fork, pitch horn, passive bushing and axle. No intersection with the Y-fork or bushing was reported. The moving rocker has no intersection with the modeled servo casing at its fixed relative mounting position.
- Small intersections remain with the horn (about 5.68–6.38 mm³ over the sampled sweep) and passive axle (about 6.43 mm³). Neutral-position intersections are essentially identical in the previous B4 rocker: these are inherited interface defects, not introduced by the user's edits. The horn and axle are simplified placeholders; inspect the actual hardware before deciding where to relieve material.
- Extrude8 reports a nonzero Fusion health-state value (4), without an error/warning message. Other recorded features report 0. No diagnosis of feature failure is implied by that number alone.

## Cogwheel and servo cross

All four yaw drivers share the same generated interface:

| Feature | Current CAD dimension |
|---|---:|
| Opposite mounting-hole spacing | 12 mm centre-to-centre |
| Mounting-hole diameter | 2.5 mm |
| Central screw passage | 3.4 mm |
| Underside circular recess | 12.6 mm diameter |
| Recess top | z = 19.3 mm in the local assembly |
| Driver hub top | z = 21 mm |
| Nominal gear-axis spacing | 25 mm |

The real cross horn was never captured accurately in this geometry. A 1–2 mm observed mismatch cannot determine whether the required change is 1–2 mm in total or at each end. Measure opposite hole centres, the selected screw diameter, horn thickness, hub diameter and cross-arm envelope. Do not scale the complete gear: that would alter its tooth geometry and mesh. Change the mounting pattern and horn pocket only, keeping the centre fixed.

The gear-axis spacing agrees with the nominal module-1, 30T/20T pitch radii (15 + 10 mm). This does not validate physical backlash or horn fit. The pitch horn also uses a provisional 12 mm opposite-hole pattern and needs the same hardware check.

## Four large chassis holes

Measurements from the exported full B4 Fusion assembly confirm all four:

| Axis position, mm | Bore | Axial extent |
|---|---:|---:|
| (+52, +52) | Ø21.1 mm | z = 0–9 mm |
| (−52, +52) | Ø21.1 mm | z = 0–9 mm |
| (+52, −52) | Ø21.1 mm | z = 0–9 mm |
| (−52, −52) | Ø21.1 mm | z = 0–9 mm |

These are bearing seats. The model places a 12 × 21 × 5 mm bearing at each axis, z = 4–9 mm. The seat has 0.1 mm diametral clearance to that envelope, but it is a straight through-bore with no axial shoulder. The nominal Ø12 mm turret journal matches the nominal bearing bore without a specified practical fit allowance.

The modeled retention washer is Ø16 mm at z = 1–3 mm, while the shaft begins at z = 4 mm. It does not retain the bearing outer ring in the Ø21.1 mm bore; its shaft attachment/spacer stack is also unfinished. A washer on the shaft cannot by itself replace outer-ring retention in the chassis.

Required design resolution:

1. A stepped bearing seat supporting the outer ring, plus a removable retainer that prevents extraction in the opposite direction.
2. A separate shoulder/spacer and screw arrangement locating the rotating shaft against the inner ring, without rubbing the stationary chassis or bearing seals.
3. Enough axial clearance to turn freely, with controlled end play. Final ring-contact diameters depend on the actual bearing construction.
4. Retainer screws and tool access checked against the nearby servo body, gear and chassis ribs.

Do not simply shrink these holes to Ø12 mm: that would remove the bearing accommodation. The bearing part number and actual dimensions should be confirmed before finalizing the seat.

## Evidence and scope

- `cad/fit_audit_20261005/inspection.json`: active rocker cylinders, solid count, bounding box and feature inventory.
- `cad/fit_audit_20261005/mating_audit.json`: pitch sampling and previous-B4 comparison.
- `cad/fit_audit_20261005/assembly_hole_dimensions.json`: actual cylindrical surfaces in all four chassis seats, yaw drivers, bearing envelopes and washers.
- `cad/InspectFit/snapshot.py`: active-document snapshot routine.
- `cad/InspectFit/InspectFit.py`: mating audit routine. Requires the updated rocker active; imports existing B4 archives for comparison.

This inspection does not establish strength, fatigue life, physical tolerances, wire clearance, fastener-head clearance or collision-free combined yaw/pitch movement. The current simulation assets and original B4 assembly were left unchanged. The earlier assembly guide remains conceptual at the horn and bearing-retention interfaces; use this inspection as the current list of unresolved fit issues.
