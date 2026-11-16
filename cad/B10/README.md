# B10 direct-drive robot with M3 camera horn

The current assembly is [Robot_B10.f3d](Robot_B10.f3d), with a matching [STEP assembly](Robot_B10.step). It includes the user's edited top plate and the replacement camera input crank for the new metal horn. Open the Fusion archive to retain the assembly's 12 revolute joints. Its saved camera position is 40 degrees down.

## Current mechanical model

- Four direct-drive yaw joints use the threaded metal servo horns. The separate yaw gears, yaw bearings and M6 yaw axles are removed. Front yaw servos are Hitec HS-475HB, rear yaw servos are Tower Pro MG995. The passive pitch bearings remain.
- The narrowed chassis and rockers, shorter yaw forks, reinforced pillars and raised equipment deck are included. The chassis is 160 x 108 mm. The nominal neutral outer width is approximately 195 mm, with the previously checked gait corridor approximately 259 mm.
- The top plate includes the user's cable-management edit. The electronics allowance extends 26.67 mm above the matrix board. The two side IR sensors remain. The front IR sensor is omitted.
- A low-mounted Parallax 900-00005 drives the elevated camera through a 22 mm-radius, 1:1 linkage. Modeled camera travel is 10 degrees up to 65 degrees down. At 40 degrees down, the nominal optical model includes both front top-plate calibration markers.
- The [optional rigid 33-degree camera bridge](parts/Rigid_camera_bridge_33deg.step) is supplied as an alternative component, not simultaneously fitted to the powered assembly.

## Replacement camera crank

The camera motor remains the **Parallax 900-00005**. Its new metal horn has a 20 mm diameter, a 2.5 mm plate thickness and four M3-threaded holes on a nominal 14 mm bolt circle. The user measured **7.34 mm from the flat case top to the seated horn's outer mounting face**.

The replacement uses **two opposite M3 x 8 mm bolts**, with 3.4 mm clearance holes at 14 mm spacing. Its 5.8 mm bosses leave 2.2 mm nominal thread engagement. The other two horn holes are unused because additional raised screw seats interfere with the pushrod near maximum downward tilt. The 6.9 mm central opening gives access to the original spline-retaining screw.

The 22 mm lever radius and rod-facing pivot surface are retained. The web is now 4.16 mm thick, accommodating the 0.66 mm horn-height change while keeping one flat back face. The lower pivot nut moves back 0.66 mm. The modeled M3 x 12 linkage screw still passes through a 2.4 mm standard nut; a taller locknut needs a separate engagement check.

Fit the crank to the horn before installing the servo in its tower. The tower obstructs straight tool access to one horn screw after assembly. Reuse the pushrod sleeve and thrust washer so tightening the linkage screw does not clamp the moving pushrod.

- [Standalone Fusion crank](parts/Camera_input_crank_M3.f3d)
- [Standalone STEP crank](parts/Camera_input_crank_M3.step)
- [Replacement STL](parts/Camera_input_crank_M3.stl)
- [Measured crank parameters](camera_crank.json)
- [CadQuery part generator](generate_camera_crank.py)

The STL is the same checked replacement delivered to the user. Complete build plates and slicer projects remain outside this repository.

## Verification and scope

[Fusion update checks](checks/Fusion_update.json) confirm that all 221 bodies outside the camera input mechanism are unchanged from the released B10 assembly. The camera input changes from 11 bodies to 9, giving 230 visible assembly bodies. All eight leg joints remain, and the four camera joints were rebuilt against the new solids. The exported Fusion archive was reopened and checked.

[Camera-crank clearance checks](checks/Camera_crank_fit.json) cover 17,434 pairs over a one-degree sweep from -10 to +65 degrees. No tested interference exceeded 0.01 mm3. Minimum modeled crank-to-pushrod clearance is 0.60 mm. The nearest new screw head remains approximately 1.21 mm clear at 65 degrees. The fitted metal horn's spline engagement is based on the user's hardware, not the simplified cylindrical servo-shaft envelope.

[Export checks](checks/Export_integrity.json) verify the STEP solids, standalone crank, and STL. The [earlier B10 checks](checks/baseline) retain their original scope for the unchanged geometry: sampled leg motion, camera calibration sightlines, and the user's top-plate edit. They are not newly run dynamics or load tests. FDM tolerances, wiring, backlash, physical fit and structural strength still require hardware testing.

The MuJoCo model remains **B9-1**. B10 has not been transferred to simulation, and B9-1 mass, gait direction, gear ratio and course results must not be applied directly to the direct-drive robot.

## Editing

Use this Fusion archive as the authoritative current assembly. `generate_camera_crank.py` reproduces only the standalone crank and requires a separate CadQuery environment. After geometry edits, update the assembly, rerun the affected clearance checks, and refresh the hashes in [current_design.json](../current_design.json).
