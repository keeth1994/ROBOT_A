# B3 — top-facing yaw and leg-mounted pitch

Saved as Robot_B3_Compact_Rockers.f3d and .step. B2 files are preserved. B3_Y_Fork_Detail.f3d/.step show a standalone leg. The authoritative Fusion builder is work/build_b3.py, invoked by work/leg_dispatch.py through JointDesigner.

- Retains eight Parallax 900-00005 servo envelopes and the original motor assumptions. No stronger motors or battery change incorporated.
- Yaw servo outputs and 30T/20T gear pair now face upward. The gearing retains the requested 1.5 speed/travel ratio; this does not mean the assembled robot clears 270 degrees.
- Two-sided Y fork connects the yaw output to the pitch horn and opposite passive pivot.
- Pitch servo casing and four-hole flange mounting move with the rocker. The casing's long direction follows the leg. Pitch horn is attached to the fork; a passive M4 axle and bushing support the other side.
- Relieved platform corners and fork/mount clearances. The rockers retain an 82 mm outside tread diameter.
- Neutral assembly bounds: 263.8 x 263.8 x 117.0 mm (B2 height 166 mm). These bounds are not a swept-motion envelope.
- Solid PLA volume estimate at 1.24 g/cm3: 277.8 g, versus B2 331.5 g: 53.7 g less. This is printed geometry only, not sliced weight or total robot mass.

## Checks and limits

B3_clearance_audit.json records the sampled CAD checks. No overlap above 0.1 mm3 was found between the moving pitch servo/foot and its Y fork at 5-degree increments from -45 to +45 degrees. The static servo-to-foot mounting check also passed. All structural template solids are connected.

At neutral pitch, the FL leg clears the tested fixed platform/electronics/yaw-servo geometry from -45 to +45 degrees yaw at 15-degree increments. These separate checks do NOT establish clearance at combined pitch/yaw poses, nor for every other leg against asymmetric electronics reservations.

54 interference records remain at larger yaw angles in the requested +/-135-degree sweep. Full 270-degree assembled yaw is NOT achieved. Do not set that as a safe operating limit. Platform packaging and/or joint placement needs another revision for full travel.

Horn envelopes and mounting patterns are provisional pending measurement of the provided horns. Gear fit, bearing retention, cable routing, fastener access, simultaneous neighbouring-leg motion, strength, print orientation and actual servo travel need validation. This is a CAD concept, not a print-ready release. B3 has not been dynamically tested in MuJoCo; the existing B1 simulation was not changed.
