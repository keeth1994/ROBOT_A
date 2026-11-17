# B10 direct-drive robot with centred camera

The current assembly is [Robot_B10.f3d](Robot_B10.f3d), with a matching [STEP assembly](Robot_B10.step). It includes the user's edited top plate, the M3 camera input crank and the replacement beam that centres the camera on the robot. Open the Fusion archive to retain the assembly's 12 revolute joints. Its saved camera position is 40 degrees down. The revision remains B10.

## Current mechanical model

- Four direct-drive yaw joints use the threaded metal servo horns. The separate yaw gears, yaw bearings and M6 yaw axles are removed. Front yaw servos are Hitec HS-475HB, rear yaw servos are Tower Pro MG995. The passive pitch bearings remain.
- The narrowed chassis and rockers, shorter yaw forks, reinforced pillars and raised equipment deck are included. The chassis is 160 x 108 mm. The nominal neutral outer width is approximately 195 mm, with the previously checked gait corridor approximately 259 mm.
- The top plate includes the user's cable-management edit. The electronics allowance extends 26.67 mm above the matrix board. The two side IR sensors remain. The front IR sensor is omitted.
- A low-mounted Parallax 900-00005 drives the elevated camera through a 22 mm-radius, 1:1 linkage. Modeled camera travel is 10 degrees up to 65 degrees down. At 40 degrees down, both front top-plate corners lie in the nominal optical view. See the wiring qualification below for the calibration patches.
- The [optional rigid 33-degree camera bridge](parts/Rigid_camera_bridge_33deg.step) is supplied as an alternative component, not simultaneously fitted to the powered assembly.

## Centred camera beam

The lens previously sat at Y = +20 mm. The new beam moves the camera and its two ear fasteners by 20 mm to put the lens on the robot's Y = 0 centreline. The camera holes are at Y = -16 and +16 mm, keeping their 32 mm spacing, 3.4 mm clearance diameter and existing nut pockets. Reuse the two M3 x 12 mm camera screws and M3 nuts.

The beam's outline, bearing pivots, drive lever and linkage positions are unchanged. It remains asymmetric because the existing towers are asymmetric; the camera is centred on the robot rather than between the tower ends. The top plate, towers, lower M3 crank, yaw and pitch parts are unchanged.

- [Replacement beam STL](parts/Camera_beam_centred.stl)
- [Standalone Fusion beam](parts/Camera_beam_centred.f3d)
- [Standalone STEP beam](parts/Camera_beam_centred.step)
- [Printing and installation notes](parts/Camera_beam_print_notes.txt)
- [Beam preview](camera_beam.png)
- [Centred camera in the assembled robot](camera_centred_front.png)

Print one replacement beam, using the provided orientation and supports beneath the elevated pivot/arm features. The STL is approximately 57.1 x 138.5 x 18.6 mm and contains one watertight connected solid. Earlier B10 slicer projects have not been rewritten: substitute this part for their original `Camera_pitch_cradle`.

The [updated optical check](checks/Camera_centred_calibration.json) gives approximately 96 pixels of minimum frame margin for the front corners and marker patches at 640 x 480, using the nominal 66.5-degree diagonal full-frame lens model. Both front-corner sightlines clear. Part of the positive-Y 6 mm marker patch intersects the conservative allowance for tall electronics/wiring, although no actual modeled component blocks it. Route plugs and wires out of that sightline and confirm the physical image before calibration.

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

[Fusion centring checks](checks/Fusion_centred_camera.json) confirm that 223 bodies are unchanged from the preceding M3-crank assembly. Seven bodies change: the beam, camera housing, lens and four ear fasteners. There are still 230 assembly bodies and 12 joints. All eight leg joints remain, and the four camera joints were rebuilt against the new solids. The exported Fusion archive was reopened and checked.

[Centred-beam clearance checks](checks/Camera_beam_fit.json) cover 54,832 pairs over a one-degree sweep from -10 to +65 degrees, including the camera plug allowance, electronics envelope and user's edited top plate. No tested interference exceeded 0.01 mm3. Straight tool access to both camera screws is clear at the checked 0, 40 and 65-degree poses.

[Camera-crank clearance checks](checks/Camera_crank_fit.json) cover 17,434 pairs over a one-degree sweep from -10 to +65 degrees. No tested interference exceeded 0.01 mm3. Minimum modeled crank-to-pushrod clearance is 0.60 mm. The nearest new screw head remains approximately 1.21 mm clear at 65 degrees. The fitted metal horn's spline engagement is based on the user's hardware, not the simplified cylindrical servo-shaft envelope.

[Export checks](checks/Export_integrity.json) verify all 230 STEP solids, the seven replacement bodies, 223 unchanged bodies, standalone beam and STL. The earlier [M3-crank Fusion update](checks/Fusion_update.json) and [B10 baseline checks](checks/baseline) remain historical evidence. Their leg-motion and top-plate checks retain their scope; the centred-camera optical report supersedes the old offset-camera calibration check. These are not dynamics or load tests. FDM tolerances, wiring, backlash, physical fit and structural strength still require hardware testing.


## Editing

Use this Fusion archive as the authoritative current assembly. `generate_camera_crank.py` reproduces only the standalone crank and requires a separate CadQuery environment. After geometry edits, update the assembly, rerun the affected clearance checks, and refresh the hashes in [current_design.json](../current_design.json).
