# M24.2 Character Authoring SDK

## PTC character asset contract v1

Author in Blender with +Z up, −Y forward, and one Blender unit per meter. Apply the mesh and rig object transforms and establish the origin at ground level between the feet. Export glTF 2.0 GLB with +Y up and +Z forward. Store the bind pose in Blender Rest Position. Fit the skeleton to the mesh in its natural A-pose when available.

Export one skinned mesh node with one skin, triangle primitives, and the deformation hierarchy. Unique semantic mappings associate anatomical roles with joint names. The standard roles are Hips, Spine, Chest, Neck, Head, and the paired Clavicle, UpperArm, Forearm, Hand, Thigh, Shin and Foot roles with .L and .R suffixes. Joint order, count, axes, lengths and bind rotations may vary. Additional deform joints and intermediate parents are allowed. Write `ptc_character_version` (integer 1) and `ptc_semantics` (JSON object encoded as a string) into the skinned mesh node extras. Runtime seating consumes the Hips mapping. Offline tools consume the remaining roles.

Every vertex needs one to four positive weights on exported joints, normalized to one. Verify zero-weight vertices, stray groups and joint indices. Use opaque glTF metallic-roughness materials, one UV set, and embedded PNG or JPEG resources. Keep authoring textures packed and preserve the source UVs. The character renderer samples base color. The GLB carries the normal and metallic-roughness maps for PBR-capable consumers.

Export independent, positive-duration Idle, Walk, Run and Turn loops starting in their animated poses. Match endpoint poses to prevent pops, and keep horizontal root motion in place for controller-owned displacement and yaw. Bake at the scene frame rate, normally 30 FPS. Seated is a held Action with duplicate endpoint keys and a positive duration. The renderer samples time zero and aligns the mapped Hips with the Jeep DriverSeat anchor plus the optional `ptc_seat_offset` vector. The vector uses Jeep-local glTF axes and meters and defaults to zero. Fit the feet to the footwell and the hands to the driving position offline. The legacy pilot uses its existing seating fallback.

Gameplay currently uses a shared 0.30 m collision radius, 1.80 m body height, and 1.63 m eye height. Author compatible dimensions and check vehicle clearance in game. Use the SDK validator before export and the runtime loader to validate the resulting skin, accessors, clips, and materials.

## Meshy source inspection

`assets/source/Female_Fighter_Pilot_Navy_10K_adaptive.blend` contains one mesh object, `Mesh0`, and zero armatures. The mesh has 5,479 vertices, 10,950 edges, 5,413 faces (5,301 quads, 112 triangles), and 10,714 render triangles. The object rotates local Y onto Blender Z and local Z onto Blender −Y. Its world XYZ dimensions are 0.927 × 0.408 × 1.892 m before normalization.

There are 11 connected components. The largest has 5,366 vertices, with the other components ranging from four to 20 vertices. 360 edges are boundaries and the other 10,590 carry two faces. The mesh has one `UVMap` and one Principled material, `Material`. The four 4096² PNG images for base color, metallic, roughness and normal are all packed. The serialized relative paths are under `//textures/`. Base color uses sRGB and the remaining maps use Non-Color.

## Start from a blank Blender file

1. Install `tools/ptc_character_sdk.py` using **Edit → Preferences → Add-ons → Install from Disk** in Blender 5.0.1. Enable **PTC Character**. Open a blank scene, import the Meshy asset or append its mesh from a .blend, then open the 3D View sidebar with **N** and select **PTC Character**.
2. Select the humanoid mesh and click **Use Selected Object**. Set **Height** in meters and the yaw correction needed for the character to face Blender −Y. Click **Apply Scale, Yaw and Ground Origin**. The tool applies the object transform, sizes the geometry, and places the applied origin at the ground. Save your own working .blend under `assets/blender/characters/`.
3. Click **Create A-pose Skeleton**. In armature Edit Mode, inspect front, side, and top views. Move each bone’s head and tail to the character’s joint centers. Set bone rolls where the anatomical bend planes differ. Fit shoulder and hip roots, elbow and knee bends, wrist and ankle placement, and feet. The preset is a starting point for fitting and scales with the configured height. Use the **Deform Armature** field to assign an existing armature. For renamed joints, select a **Semantic role** and its **Deform bone**, then click **Assign Semantic Bone** for each role.
4. Click **Bind** for a proximity-weighted starting skin, or assign the fitted armature and existing vertex groups from your own binding workflow. In **Rest Position**, verify neutral deformation against the imported A-pose. Click **Check Weights** to select problem vertices. Enter Edit Mode to inspect the selection, or Weight Paint to correct individual influences. Keep a maximum of four deform-joint weights per vertex and normalize them. Check armor and accessories around knees, elbows, and hips during motion. The source’s small disconnected components require the same verification as the main mesh.
5. Click **Generate Fitted Rigify Controls** after skeletal fitting. The SDK adapts the Basic Human metarig and generates an IK/FK rig with a bridge to the deform skeleton. Click **Use IK/FK Controls**, enter Pose Mode, and move the hand and foot IK controls. The limb parent controls expose the Rigify `IK_FK` slider. Set it to 0 for IK or 1 for FK. Blender can run the generated Rigify UI script from the trusted production file for its custom panels. The SDK and the deform Action previews work with Blender’s standard pose properties. Fit all joints before generating the bridge.
6. Set **Reference .blend** to `assets/blender/pilot.blend`, **Reference armature** to `Pilot Deform` and **Reference FPS** to 24. Set the target scene to 30 FPS and click **Retarget Locomotion**. Preview each clip and scrub the Action in the Dope Sheet. Refine planted contacts, forward swing, and weight transitions for the character. Use **Store Active Action** to register an edited deform Action under the selected standard name. To author with Rigify, key the controls, set the scene playback range, select a standard Action, and click **Bake Controls Over Scene Range**. The baked Action is editable on the deform rig.
7. Click **Create Jeep Seat Draft** or make an Action in the Action Editor. Keep both knees forward with feet above the footwell and wrists reaching within the driver space. Import the Jeep GLB as a placement reference in a separate working scene or use a trace of its seat and footwell. Fit the pelvis to `DriverSeat` with the optional `ptc_seat_offset` float array on the mesh. Key the held pose at the start and end, then store it under **Seated**. The female pilot uses offset `[0, 0.18, -0.02]` and a half-second hold.
8. Click **Pack Resources**, **Validate Character**, and **Scratch**. Use **Mesh and Material Summary** to review counts and resource names. Supply the destination in **Runtime GLB**, click **Export GLB**, then use **Validate Exported GLB** to inspect it again. The export stages the file first, validates the skin and clips, and publishes the result to the chosen path. The SDK exports the mesh and deformation hierarchy and temporarily mutes preview constraints.
9. Add the exported GLB under **assets** in `assets/worlds/tango_charlie/world.json` with type `character`. Set the **asset** field of entity `pilot_01` to the registry ID. `female_pilot` is the reference default and `pilot` selects the original. The same `pilot_01` entity retains its placement and vehicle access. Test another manifest with `main.py --world-manifest /path/to/world.json --enemies 0`.
10. Start in the TC167. Press **Enter** to exit and board, use **WASD** on foot, hold **Shift** to run, and press **P** to release the Jeep parking brake. Inspect the character in both camera views. Check the Jeep seat from above and in profile, drive and stop the Jeep, then exit and verify Idle on the first standing frame. Return to the aircraft and exercise its configuration controls.

### How the reference rig and retarget operate

The 19 deform bones use the semantic names in the contract. Hips is the root. Spine, Chest, Neck and Head form the midline chain. Each Clavicle branches from Chest and leads to UpperArm, Forearm and Hand. Each Thigh branches from Hips and leads to Shin and Foot. The female pilot’s joints and rolls were fitted from its own front and side geometry at 1.80 m. The Meshy polygon and UV topology and all four packed textures are preserved.

The retargeter establishes armature-space standing reference directions from the source skeleton and rotates each target bind direction into that reference. For each frame, it applies the evaluated source motion relative to the source bind orientation. It then reconstructs target joint transforms hierarchically with target bone lengths and converts them to parent-local pose transforms in each fitted joint’s basis. Grounding samples the evaluated skin and adjusts the vertical pelvis translation to bring the lowest sole to the ground in each baked frame. The runtime samples the exported channels and inverse bind matrices. Game movement supplies horizontal displacement and turn direction.

The Rigify bridge uses helper bones parented to generated Rigify bones. Each helper’s rest transform matches the corresponding deform joint. This lets edits from IK, FK, and torso controls reach the skin without copying mismatched local rotations. **Rest Position** displays the A-pose independently of the current Action. The runtime rig is the only exported armature.

| Action | Exported duration |
| --- | ---: |
| Idle | 2.400 s |
| Walk | 0.800 s |
| Run | 0.567 s |
| Turn | 0.500 s |
| Seated | 0.500 s |

### Command-line authoring and validation

From the repository root, with Blender 5.0.1 and the project Python requirements installed:

```sh
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/author_character.py -- export --output /tmp/female_pilot.glb
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/author_character.py -- validate --output /tmp/female_pilot.glb
.venv/bin/python -c 'from engine.pilot_asset import PilotAsset; PilotAsset("/tmp/female_pilot.glb")'
cmp /tmp/female_pilot.glb assets/models/characters/female_pilot.glb
```

Use `export` without an output override to publish edits from the saved production .blend. Pass `--blend /path/to/character.blend --mesh "Character Mesh" --output /path/to/character.glb` for another character. The `build` subcommand regenerates the reference authoring asset from the preserved source and retargets the original pilot’s animation set. Use `export` for artist edits you want to retain.

```sh
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/test_character_sdk_blender.py
.venv/bin/python -m unittest discover -s tests
.venv/bin/python -m tools.check_character_gl --output /tmp/ptc-female
.venv/bin/python -m tools.check_character_gl --legacy --output /tmp/ptc-original
```

## Rendering and known limitations

The authoring mesh has 5,479 vertices and 10,714 triangles. The GLB has 11,073 vertex records after the exporter splits per-corner attributes at UV and normal seams. Blender and OpenGL show the same navy suit, white armor, red stripes, and dark visor. The GLB embeds the base color, normal, and combined metallic-roughness images. OpenGL uses the base color with the game’s flat shading. Blender’s Principled material can show normal detail, specular response, and roughness under lighting.

The existing Jeep interior has a shallow footwell, a largely recessed seat cushion, and a simplified dashboard. The female pilot fit uses elevated knees and a small hip offset. Exported skin vertices clear the floor and dashboard bar. The forward toe geometry overlaps the hood edge by up to 3.5 mm. Her hands are posed toward the driving position. The Jeep supplies neither a steering-wheel hand anchor nor a pedal target, and the seated Action is a hold while driving. The controller still uses its shared collision dimensions and selects Turn during a blocked pivot. The OpenGL route exercises both directions from a controlled wall-side setup and uses camera placement for readable screenshots.

The starter skin and offline retargeting are intended for artist refinement. Grounding adjusts pelvis height across samples. Planting a specific foot across several frames, preserving hand contact during driving, and final armor weighting are artist passes. The runtime accepts one skin and one mesh node with multiple primitives. It uses triangle geometry, a single UV set, and opaque materials.

## Verification and delivery

- Blender SDK coverage exercised setup from a blank file, invalid weight selection, the production Rigify hand and foot IK, arm FK, baking, and restoration of the neutral pose. The production .blend re-exported a byte-identical GLB. A comparison against the source confirmed the same polygon indices, per-corner UVs, material assignments, material node links, and packed image bytes.
- The original pilot’s Blender integration test passed. The active `TC167_Authoring.blend` has a newer embedded authoring script, and the legacy suite expects its former `choose` API. All 12 TC167 authoring tests pass against the existing `TC167_Authoring.blend1` when copied to a temporary .blend and selected via `TC167_TEST_BLEND`. Use separate copies for run logs and generated reports.
- SHA-256 comparisons cover the Meshy source and the existing pilot, Jeep, TC167, hangar, and repair station runtime GLBs.
- All 488 Python tests passed, including five new tests for independently fitted skeletons, an extra joint, remapped semantics, invalid weights, required Actions, authored seating, and manifest selection.
- All four existing ground asset exports reproduced byte for byte and passed Blender reimport.
- OpenGL 4.1 on Apple M3 Pro ran the full route using the female pilot and repeated it with the original pilot. The harness verified walking and running, the two female pilot pivots, Jeep occupancy and motion, first-frame Idle on exit, aircraft reboarding, flap input, and all three configuration changes.

| Verification view | Captures |
| --- | --- |
| Start and hangar | [Cockpit start](images/m24_2/01_cockpit_start.png), [OpenGL Idle](images/m24_2/02_hangar_idle.png), [Blender Idle](images/m24_2/blender_idle.png), [Walk](images/m24_2/03_hangar_walk.png) |
| Apron and workshop | [Run](images/m24_2/04_apron_run.png), [Left pivot](images/m24_2/turn_left.png), [Right pivot](images/m24_2/turn_right.png), [Repair station](images/m24_2/05_repair_station.png) |
| Jeep and return | [Seated](images/m24_2/06_jeep_seated.png), [Driving](images/m24_2/07_jeep_driving.png), [Exit](images/m24_2/08_jeep_exit.png), [Aircraft reboarded](images/m24_2/09_reboarded.png) |
| Aircraft configurations | [VTOL](images/m24_2/10_0_vtol.png), [Battledroid](images/m24_2/10_1_battledroid.png), [Fighter](images/m24_2/10_2_fighter.png) |
| Original pilot | [Idle](images/m24_2/original_hangar_idle.png), [Legacy seating](images/m24_2/original_jeep_seated.png) |

### Working tree

The M24.2 changes are in the working tree. They include the new Blender and GLB assets, the Blender add-on, CLI and verification tools, character runtime and texture shader changes, the world manifest selection, and this guide with captures. The repository also contains pre-existing modified and untracked work. The verified source and legacy GLBs match their pre-task SHA-256 checksums. No commit, tag, reset, or push was performed.
