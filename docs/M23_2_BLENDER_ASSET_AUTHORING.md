# M23.2 Blender asset authoring

Blender 5.0.1 is the source of truth for the pilot and Jeep. Edit the
`.blend` files and export the runtime GLBs. Both projects are independent
of the TC167 sidebar and animation pipeline.

| Asset | Editable source | Runtime artifact |
| --- | --- | --- |
| Pilot | `assets/blender/pilot.blend` | `assets/models/characters/pilot.glb` |
| Jeep | `assets/blender/jeep.blend` | `assets/models/vehicles/jeep.glb` |

## Pilot

The **Pilot Asset** collection contains **Pilot Deform** and its skinned
geometry. The separate **Pilot Rigify Authoring** collection contains the
fitted metarig and generated control rig. See the
[pilot rig guide](M23_3_PILOT_RIG_STANDARDIZATION.md) for the 19-bone
hierarchy, axis and weight conventions, IK/FK, and baking workflow.

Select **Pilot Deform** to edit **Idle**, **Walk**, **Run**, or **Turn** in
the Action Editor. Keep its armature modifier and applied mesh transform.
Preserve the four joint-influence limit and keep the clips in place.

## Jeep

The **Jeep Asset** collection contains **Body**, **Wheels**, **Anchors**, and
**Details**. Add decorative mesh children under `Jeep` in **Details**. Keep
runtime geometry inside **Jeep Asset**, and place cameras and visual
references outside it. The scene uses meters, with +Y up and +Z forward
in glTF space.

- `Jeep` is the single, identity-transformed export root and chassis mesh.
- Keep `Jeep > Hub.FL > Wheel.FL`, `Jeep > Hub.FR > Wheel.FR`,
  `Jeep > Hub.RL > Wheel.RL`, and `Jeep > Hub.RR > Wheel.RR`.
  Front hubs steer around local Y. All centered wheel children spin around
  local X. Keep the hubs at the original suspension pivots with identity
  rotation and scale, and edit tire geometry around each wheel origin.
- Keep empty children `DriverSeat`, `Entry.L`, `Entry.R`, `Exit.L`,
  and `Exit.R` directly under `Jeep`, with identity rotation and scale.
  Left is positive X in the asset. Check boarding, seating, and exit
  clearance after moving an anchor.

The controller supplies suspension motion and collision bounds. Remodeling
within the current footprint leaves those assumptions valid. Changing
wheelbase, track, or collision size requires a coordinated physics change.

## Blender UI export

1. Open the source and select the objects in **Pilot Asset** or **Jeep Asset**,
   including the objects inside its subcollections. Export from Object Mode.
2. Choose **File → Export → glTF 2.0**, set **Format** to **glTF Binary (.glb)**,
   enable **Selected Objects** and **+Y Up**, and leave cameras, lights, shape
   keys, and compression disabled.
3. Use the CLI for the pilot's canonical export. For a manual scratch
   export, finish baking, disable the preview, remove authoring control
   armatures and preview constraints in a disposable copy, and enable skinning and **Use Rest Position Armature**. Enable
   **Remove Armature Object**, keep all-influences export off, and select
   **Actions** as the animation export mode with **Always Sample Animations**
   disabled. For the Jeep, disable animation export.
4. Export to a scratch GLB and validate it before updating the runtime asset.
   Use the CLI below to regenerate the canonical artifact reproducibly.

## Command-line export and validation

Run from the repository root. Set `BLENDER` to the Blender 5.0.1 binary.

```sh
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender

"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/export_ground_assets.py -- pilot
"$BLENDER" -b --factory-startup --python-exit-code 1 \
  --python tools/export_ground_assets.py -- jeep

.venv/bin/python -m tools.validate_ground_assets pilot
.venv/bin/python -m tools.validate_ground_assets jeep
```

The exporter validates a temporary candidate through the game loaders before
replacing the runtime artifact. Use `--blend-input /tmp/working.blend
--output /tmp/preview.glb` after the asset name to export a working copy.
The validator accepts a GLB path after the asset name. Add **--check** to
either Blender export command to test a fresh export, import that GLB in a
clean Blender scene, and compare the bytes with the current runtime file.
Both canonical assets reproduced byte for byte under Blender 5.0.1.
A byte difference requires review even if runtime validation passes.

If a required node is missing, check the export selection, capitalization,
and parent relationships. A transformed PilotMesh or extra armature root
usually means the pilot was moved at object level or **Remove Armature
Object** was disabled. Unexpected clips can come from duplicate Actions or
extra stashed tracks. If the gait fails validation, inspect the shin and
forearm directions and sample the opposing arm and thigh at equivalent
points in the loop.

`scripts/generate_pilot.py` and `scripts/generate_jeep.py` are legacy
bootstrap tools. They require a new scratch `--output` path outside
`assets/`. Use Blender for geometry, animation, and runtime exports.

## M23.2 verification

The baseline passed 470 Python tests. The final suite passed 472 Python
tests and all 12 TC167 Blender integration tests. The TC167 runtime
comparison passed, and its fresh export matched its existing GLB byte for
byte. A disposable pilot mesh edit exported with the same sampled bone
transforms and excluded a reference outside the asset collection.

The rendered OpenGL check walked and ran with the rebuilt pilot, drove the
Jeep onto natural terrain, and exited on an 11.2° slope. The pilot walked
back to the TC167 and resumed throttle and transformation control. The
pilot keeps its simple skinned mesh and procedural seating pose. The
runtime asset loaders still use flat materials and the existing rig and
vehicle dimensions.
