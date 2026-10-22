# TC167 configuration-driven authoring — M21.3

Open [TC167_Authoring.blend](../assets/blender/TC167_Authoring.blend) in **Blender
5.0.1**. This separate working file preserves M21.2 and every original production
asset. Geometry, node names, hierarchy, mesh origins, mechanical pivots, parent
inverses and production rest data remain unchanged. No skeletal rig is introduced.

## Redesigning the VTOL Configuration

1. Open `assets/blender/TC167_Authoring.blend`. In a Text Editor select the saved
   **TC167_Action_Controls.py**, then **Run Script** (Alt-P). The script is embedded
   and works without importing the game or finding the repository. It only registers
   controls; running it does not save configurations or synchronize keys.
2. Return to the 3D Viewport, press **N**, and choose the **TC167** sidebar tab.
   Click **VTOL** under Configurations. This selects `TC_Pose_VTOL`, assigns every
   relevant object's slot, disables NLA blending and moves to its first frame.
3. Select the mechanical pivot **LeftWing**, not its decorative mesh child. Press
   G then X to move it slightly, or use the Item panel. Turn Auto Keying off for
   this workflow. Do not apply transforms, edit mesh vertices, move origins,
   change parents, clear parent inverses or add constraints/drivers. The editor
   recognizes structural pivot objects and existing intentionally articulated
   shells; unchanged decorative mesh companions are excluded from pose capture.
4. Click **Save Configuration Pose** while VTOL is active. It captures evaluated
   parent-local transforms (compensated for the original parent inverse) into
   real Location, Quaternion Rotation and Scale keys in `TC_Pose_VTOL`. Both static
   endpoint keys, and any existing keys of the changed configuration channel,
   receive the same value. Only changed node/channel groups are written. Other
   Actions remain untouched. The status reports the count; **Text Editor →
   TC167_Last_Report.json** lists exact nodes, channels, old and new values.
   A viewport transform is temporary until saved into the Action. Selecting another
   Action never saves it automatically. Ctrl-S persists the saved pose to disk.
5. Click **Preview Endpoint Changes**. Read `TC167_Last_Report.json` for the VTOL
   configuration, affected Actions/nodes, channel groups, component additions,
   first/last frames, old/new values and numerical differences. VTOL connects to
   all four transformation Actions: the destination of FighterToVTOL and
   BattledroidToVTOL, and source of VTOLToFighter and VTOLToBattledroid.
6. Click **Sync Transformation Endpoints**. A confirmation dialog shows the
   configuration and proposed changes (first 14 rows, with the full list in the
   report Text). Check **Apply these endpoint changes**, then click OK. **Cancel**
   or an unchecked box changes no keys. A stale preview is rejected; reopen the
   dialog to review a fresh plan. Nothing synchronizes on playback, file loading,
   configuration selection, or running the helper.
7. Click **Validate Configuration Consistency**. Inspect the errors and warnings
   in `TC167_Last_Report.json`. Endpoint tolerance is **2e-5** in Blender basis TRS
   components (quaternion sign equivalents are accepted). The validator reads the
   Actions directly and does not assign Actions, move the playhead or change keys.
   Missing channels, bad slots, unexpected decorative motion, nonfinite values,
   quaternion norms, hierarchy/pivot changes and mesh-origin changes are errors.
8. Play each related transformation and scrub its timeline. Sync changes only
   endpoint values: existing interior keys, their handles/interpolation, key times,
   clip ranges, locomotion and unrelated Actions are preserved. Missing components
   get endpoint curves only, with the opposite endpoint read from its configuration.
   No new decorative-node animation is generated. Use the Graph Editor to refine
   choreography yourself. Large adjacent rotation/translation/scale steps,
   quaternion sign changes, scale zero crossings, and coarse major-shell overlaps
   produce warnings; the helper does not smooth or hide them.
9. Save with Ctrl-S, then explicitly export and validate from the repository root:

   ```sh
   /Applications/Blender.app/Contents/MacOS/Blender \
     --background --factory-startup --python-exit-code 1 \
     --python tools/author_tc167_authoring.py -- --export-only

   /Applications/Blender.app/Contents/MacOS/Blender \
     --background --factory-startup --python-exit-code 1 \
     --python tools/validate_tc167_authoring_blender.py

   .venv/bin/python -m tools.inspect_glb_animations \
     assets/models/vehicles/tc167_authoring.glb --require-bd-clips
   ```

   Export refuses unsynchronized/invalid configurations. It exports saved edits,
   not regenerated starter motion. Use `--blend-input PATH --glb-output PATH` for
   alternate copies. Old milestone output paths are protected. Ten clips are
   exported: three configurations, four transformations and three locomotion clips.
   Production geometry, materials, node hierarchy and rest TRS remain the export
   baseline even if the authored FIGHTER configuration changes. This distinction
   preserves the engine's immutable rest contract.
10. To prepare runtime pose data explicitly, run:

    ```sh
    .venv/bin/python -m tools.tc167_runtime_pose_bridge \
      assets/models/vehicles/tc167_authoring.glb \
      --output /tmp/tc167_pose_candidate.json \
      --python-output /tmp/tc167_pose_candidate.py
    ```

    This creates full parent-local glTF TRS data and an optional generated Python
    `VehicleMode → node → PoseTransform` mapping. No angles need to be copied by
    hand. The tool requires all configuration/transition endpoints to match within
    **2e-5 per local matrix component**, validates production rest/hierarchy, and
    round-trips JSON through real runtime PoseTransform/model APIs. Its report lists
    node-by-node differences from today's Python configurations. Candidate outputs
    are prohibited inside `game/`, `engine/`, and `assets/`. Review the candidate and
    tests before any explicit developer integration. Generation changes no gameplay.

## Undo, recovery and Action slots

**Save Configuration Pose** and confirmed endpoint synchronization are Blender
Undo operators. Ctrl-Z restores the previous operation; Edit → Undo History lets
you choose an earlier point. If configuration and endpoint sync were two operations,
undo sync first, then the configuration save if desired. Canceling the dialog does
not add endpoint changes. Save a new `.blend` copy before a major redesign; reopen
that copy to recover across sessions. Undo history is not preserved when reopening
Blender. Running preview/validation only updates their report Text.

The Actions are multi-object, slotted Blender data. A slot identifies one object's
curves inside a shared Action; the helper finds slots by exact `OB` + node name,
not by their position or a presumed uniform curve list. Curves can have different
key counts or absent components. Retain the complete Action while selecting an
individual node slot in Dope Sheet → Action Editor or Graph Editor. Muted NLA
archive strips are exporter bindings; they do not layer other clips on the selected
preview. Fake users preserve Actions. The sidebar is a convenience interface;
Blender keyframes are the saved animation authority.

## Configuration authority and runtime migration

Within this file, `TC_Pose_Fighter`, `TC_Pose_VTOL` and `TC_Pose_Battledroid` are the
configuration authority. Sync propagates only the currently selected configuration
into entering/leaving clips. FIGHTER and BATTLEDROID each affect two endpoints;
VTOL affects four. Locomotion is intentionally independent: redesigning BATTLEDROID
may require separately refining its gait poses. Configuration Actions are static;
use transformation Actions for choreography.

**Gameplay still uses Python.** `TransformationController.reset()` and
`apply_curve()` use `game/tc167_poses.py`, imported production rest fallbacks and
procedural engine-to-foot attachments. FIGHTER reset restores imported rest directly.
The game never selects the exported `TC_Pose_*` or transformation clips. At rest in
BATTLEDROID, the existing animation/locomotion layer can additionally override node
visuals with BD_Idle/Walk/Run; those layers are cleared/suppressed during procedural
transformations. Physics, camera and combat do not acquire pose authority from the
new authoring GLB. Selecting this asset in the game therefore does not install a
redesigned Blender configuration.

The bridge's generated data is a **candidate, not a drop-in replacement** for
`game/tc167_poses.py`: it includes engines and the full FIGHTER pose. The existing
controller would otherwise recompute engine poses and bypass edited FIGHTER data.
For explicit migration, use `apply_candidate(model, candidate, configuration)` from
`tools.tc167_runtime_pose_bridge` on an isolated CPU model to inspect the complete
state. The generated Python mapping represents those same validated runtime values.
A later, separately reviewed developer change must install that data and choose a
single owner for FIGHTER reset, engine attachment and transformation interpolation.
Before enabling it, rerun the bridge comparisons, validate configured endpoints,
review intermediate engine paths/gaits, and run the gameplay suite. Do not combine
absolute candidate engine poses with another procedural attachment write. M21.3
provides data generation and round-trip verification; it does not activate migration.

## Validation and known limits

[TC167_AUTHORING_WORKFLOW_VALIDATION.json](TC167_AUTHORING_WORKFLOW_VALIDATION.json)
records the real Action inventory, structural checks, warnings, bridge round trips,
protected asset hashes and Blender integration test results. Reproducible tests:

```sh
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/test_tc167_authoring_blender.py
.venv/bin/python -m unittest discover -s tests
```

To generate another initial authoring copy, run `tools/author_tc167_authoring.py`
without `--export-only`, specifying new `--blend-output` and `--glb-output` paths.
Creation starts from the preserved M21.2 file and refuses an existing authoring
output. The embedded helper is self-contained; the export and validation tools
run from this repository. Keep the helper source and embedded Text in sync when
changing the tools.

Background Blender exercises actual Actions, dependency-graph transforms and Undo.
Interactive dialog layout and visual artistic quality still need manual inspection.
Overlap detection is a conservative AABB test on five major shells at each clip's
start, midpoint and end; it can flag intentional overlaps and cannot prove collision
freedom. Motion warnings are heuristic adjacent-key thresholds (45° rotation,
0.5 m translation-component change, 0.25 scale-component change), not a physics
solver. Existing mirrored rest scales and zero crossings are preserved and warned
about. Quaternion sign alignment uses the nearest surviving interior key, without
rewriting that key. Unsupported constraints, drivers, destructive mesh changes and
shear are outside this rigid-node workflow. Export keeps Blender sampling and
normalizes equivalent signed-scale/quaternion branches as in M21.2.

---

# M21.2 legacy complete-vehicle reference

Use [TC167_Complete.blend](../assets/blender/TC167_Complete.blend) with Blender
**5.0.1**. It contains the original rigid objects, geometry, materials, parents,
origins and parent inverses, three reference configurations, four editable
transformations, and the existing three BATTLEDROID locomotion Actions. There is
no armature, skinning, driver-based playback, or runtime-controller dependency in
the saved animations. The four original production/M21.1 files remain unchanged.

## First preview: FIGHTER → VTOL → BATTLEDROID

1. Open `assets/blender/TC167_Complete.blend`. It opens in FIGHTER.
2. Change an editor to **Text Editor** using the editor-type menu at its top left.
   Select the saved text **TC167_Action_Controls.py**, then click **Run Script**
   (Alt-P). Run it once each time you open the file; automatic script execution
   is unnecessary. Running it again safely replaces the panel registration.
3. Return to the 3D Viewport, press **N**, and open the **TC167** sidebar tab.
4. Under **Configurations**, click **FIGHTER**, **VTOL**, or **BATTLEDROID** to
   inspect a complete reference pose. These buttons select static keyed Actions;
   **FIGHTER** restores the exact original model basis without applying transforms.
5. Click **FIGHTER → VTOL** under **Transformations**. Every object's Action slot
   is assigned, archived NLA evaluation is disabled, the range becomes 1–181,
   and the playhead returns to frame 1. Press Space in the viewport or use
   **Play / Pause**. Pause at frame 181 to inspect VTOL.
6. Click **VTOL → BATTLEDROID**. The timeline becomes 1–229; press Space again.
   Its first pose matches the previous clip's final pose. The final frame is the
   canonical BATTLEDROID stance. Reverse buttons follow the same curves backward.
7. **Return to first frame** restarts the selected range. **Reset to reference
   pose** restores the last configuration button you selected (FIGHTER initially).
   Blender normally loops its playback range; transformation Actions themselves
   have no Cycles modifiers. Pause at the last frame to inspect the destination.

## Actions, slots, NLA, and editing

An **Action** stores editable keyframes. Each Action **slot** binds a subset of
those curves to one rigid object. The new Actions have 68 slots so switching away
from any other configuration resets every runtime component. An **NLA strip** can
schedule or blend an Action; muted archive strips here retain exporter bindings.
The **TC167 panel** only assigns those slots and configures playback. It does not
calculate motion. All Actions have fake users and survive saving/reopening.

Select a mechanical pivot such as `LeftUpperLeg`, `RightFoot`, `LeftWing`,
`LeftShoulder`, `Fuselage`, or `Head`. Change the Timeline to **Dope Sheet**, then
choose **Action Editor** in its mode menu. Disable **Only Show Selected** if you
want to inspect more objects. Keep the existing complete Action selected and
choose the selected object's slot in the slot selector. Move or insert keys in
that slot; do not create a separate Action for each part or unlink the other
slots. Sidebar buttons reassign the complete vehicle if a slot is changed manually.

Change another editor to **Graph Editor**. Expand Location, Quaternion Rotation,
and Scale to edit individual channels. The starting curves use LINEAR keys with
continuous quaternion signs. Quaternion components work together; normalize
rotations and preserve endpoint poses when editing. Blender interpolates these
components and normalizes the resulting orientation; the engine uses shortest-arc
quaternion slerp. The exporter samples saved curves to bridge that difference.
Bezier edits are supported through sampling. Keep end keys and loop endpoints
consistent. Engines are root siblings with baked foot-following motion: changing
leg motion may require editing engine channels as well.

Save with Ctrl-S. Do not apply object transforms, alter origins, change parents,
clear parent inverses, rename nodes, or modify mesh/material data. The exporter
rejects geometry/hierarchy/pivot changes against the saved baseline. Edit new
copies for experiments; creation refuses to overwrite existing authoring files.

The complete scene uses **120 FPS** so 1.9 seconds lands exactly on a frame.
Original gait curves are retimed from 24 FPS without changing their motion or
seconds. Sparse transformation keys are reduced separately per object; constant
channels retain just endpoints. The nonlinear engine attachment paths need more
keys than simple joints. Locomotion retains its original keys and Cycles modifiers.

| Action | Frames | Seconds | Targeted nodes |
| --- | --- | ---: | ---: |
| TC_FighterToVTOL | 1–181 | 1.5 | 68 |
| TC_VTOLToFighter | 1–181 | 1.5 | 68 |
| TC_VTOLToBattledroid | 1–229 | 1.9 | 68 |
| TC_BattledroidToVTOL | 1–229 | 1.9 | 68 |
| BD_Idle | 1–241 | 2 | 36 |
| BD_Walk | 1–121 | 1 | 36 |
| BD_Run | 1–91 | 0.75 | 36 |

`TC_Pose_Fighter`, `TC_Pose_VTOL`, and `TC_Pose_Battledroid` have identical keys
at frames 1 and 2 (68 nodes each). They export as short static clips. The legacy
`FIGHTER_Rest` Action remains retained; use the new FIGHTER button for restoration.

## Export, validation, and game testing

After saving artist edits, run from the project root:

```sh
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/author_tc167_complete.py -- --export-only

.venv/bin/python -m tools.inspect_glb_animations \
  assets/models/vehicles/tc167_complete.glb --require-bd-clips

/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/validate_tc167_complete_blender.py

.venv/bin/python -m unittest discover -s tests

.venv/bin/python main.py --vehicle-model assets/models/vehicles/tc167_complete.glb \
  --battledroid-test ground --enemies 0
```

Export-only exports saved curves, including gait edits, without regenerating
Actions or saving over the Blender file. Use `--blend-input PATH --glb-output PATH`
for another working copy. Production and M21.1 output paths are protected. All ten
clips are included. Only animation buffers are appended to the original
production geometry/material/rest document; node targets map by unchanged names.
Equivalent signed-scale/quaternion branches are made continuous in exported
transformation samplers. On initial creation only, the original runtime gait
samplers are copied exactly to avoid extra resampling differences. Exporting artist
edits uses the edited Blender curves for every clip.

To reproduce the initial complete file from the preserved M21.1 authoring file,
run the author tool without `--export-only`, choosing a new `--blend-output` path
and `--glb-output` path. No existing authoring output is overwritten.

The Blender validator reopens the complete file, runs the embedded helper,
selects every reference and Action, evaluates all gait frames and 241 timeline
positions per transformation, and writes
[TC167_COMPLETE_VALIDATION.json](TC167_COMPLETE_VALIDATION.json). It compares
local matrices to fresh `TransformationController` results after reversing the
coordinate conversion. The asset tests additionally use the actual runtime loader
and player at 121 times per transformation, check forward/reverse agreement and
clip joins, preserve the binary geometry prefix, and compare gameplay updates.
The validator intentionally checks the supplied reference motion: artistic edits
that depart from it will need reviewed tolerances/expectations.

The game continues to use its procedural transformation controller. Supplying the
complete GLB does **not** activate transformation clips in gameplay; only the
existing BD gait names are automatically selected. Active transformations disable
native gait ownership. No game code or controls changed. Preview transformations
in Blender or use the CPU `AnimationPlayer` in the asset tests; no new gameplay
preview controls were added.

## Coordinate and validation limits

The M21.1 coordinate/pivot explanation below still applies: Blender `(X,Y,Z)` =
glTF `(X,-Z,Y)`; compensate each existing parent inverse when keying the basis.
The game renderer's separate 180° Y conversion is not baked into Actions.
Reference poses use original rest TRS fallbacks plus the authoritative engine
attachments. Transformation staging uses cubic smoothstep timing windows and
shortest-arc rotations from `game/transformation.py`, then stored Blender keys.

Endpoint tolerance is **2e-5 per local matrix component**; intermediate tolerance
is **0.004**, covering sparse keys and sampled quaternion interpolation. These
are matrix-component bounds, not world-space vertex-distance guarantees. No
intentional artistic pose changes were made. The original controller deliberately
lifts `TC167Root` by 3.2873 m during BATTLEDROID deployment; this is a visual datum
change, not additional world translation. Root rotation and other root translation
components remain unchanged. Mirrored rest scales cross zero on some parts in
the authoritative transition; this existing collapse/reflection is preserved,
not replaced with positive scales. Finite TRS, expected scale matrices, hierarchy,
endpoint continuity, and repeated nonaccumulating playback are checked numerically.

Validation uses background Blender with the real embedded helper and dependency
graph. Interactive panel clicks, real-time viewport playback, and visual artistic
approval still require an artist's inspection. This is a rigid-node transformation
authoring environment; it does not automatically solve collision avoidance or
regenerate engine attachments after artist edits.

---

# M21.1 legacy authoring reference

Open [TC167_Animated.blend](../assets/blender/TC167_Animated.blend) in Blender 5.0.1.
It opens at frame 1 in the canonical BATTLEDROID stance, with `BD_Idle` active and
the viewport framed for editing. The original `TC167.blend` and `tc167.glb` were
preserved. The separate runtime asset is
[tc167_animated.glb](../assets/models/vehicles/tc167_animated.glb).

These are working starter cycles for further artistic refinement. Blender plays
stored object keyframes directly. The saved helper script only switches Action
slots; playback has no Python drivers, armature, or skeletal skinning.

## Actions and timing

Each Action has 36 rigid-object slots, 360 editable FCurves, and closed endpoints.
The scene uses 24 FPS. The duplicate endpoint closes the cycle; elapsed duration
is `(last frame - first frame) / 24`, rather than counting the endpoint twice.

| Action | Frames | Duration | Real keyframe points | Motion |
| --- | --- | --- | ---: | --- |
| BD_Idle | 1–49 | 2 seconds | 17,640 | 12 mm torso movement, subtle shoulders/head, fixed feet |
| BD_Walk | 1–25 | 1 second | 9,000 | Alternating planted/lifted feet, knees, opposing arms, torso counterrotation |
| BD_Run | 1–19 | 0.75 seconds | 6,840 | Longer stride, faster articulation, stronger arm swing, 6° forward lean |

Root translation and orientation are constant throughout all three loops. Walk
uses a 1.20 m stance sweep over 60% of its cycle, matching the M21 nominal 2 m/s.
Run uses a 1.89 m stance sweep over 42% of its cycle, matching nominal 6 m/s.
The legs use their existing 2.6 m links, with modest suspension compression and
flat ankle orientation. Walk swing-foot lift is 0.23 m; run lift is 0.40 m.
The head counteracts body lean and receives restrained motion rather than tracking
every arm/leg movement.

Curve keys use LINEAR interpolation and Cycles modifiers for editing/playback.
The exported glTF uses supported LINEAR and STEP samplers. You can refine the
Blender curves in the Graph Editor; the supplied exporter samples them every frame.

## Preview and select clips

1. Open the animated file and press Space to play the active idle cycle.
2. In a Text Editor, select the saved `TC167_Action_Controls.py` text and choose
   **Run Script** once. Open the 3D Viewport sidebar (`N`) and select **TC167**.
3. Click `BD_Idle`, `BD_Walk`, or `BD_Run`. Each button assigns all relevant object
   slots to the shared Action and sets its matching playback range.
4. Select a joint such as `LeftUpperLeg`, `RightLowerLeg`, `LeftFoot`, `TorsoCore`,
   `LeftUpperArm`, or `Head`. Use Dope Sheet → Action Editor to edit its slot, and
   the Graph Editor to edit Location, Quaternion Rotation, and Scale curves.
5. Preserve matching first/last keys for seamless looping. Save your `.blend` edits
   before running the export command.

The three Actions are also retained in muted single-strip NLA tracks. These archive
the Actions for export; they do not stack multiple gaits on the model. The helper
turns NLA evaluation off while assigning the selected Action. Changing the Action
on one selected object in the Action Editor changes only that object's slot;
the sidebar buttons switch the whole machine together.

`TC167_README`, `TC167_FIGHTER_BASIS.json`, and `TC167_GEOMETRY_BASELINE.json` are
saved Text datablocks containing instructions and export/rest-preservation metadata.
The BD animation data lives in the Actions' FCurves.

## Restore FIGHTER

Click **FIGHTER_Rest** in the TC167 sidebar. This real restoration Action assigns
all 68 runtime objects their production transforms at frame 0. Reopening the file
and evaluating this Action was verified to reproduce the original Blender local
matrices exactly, including the original mirrored parts.

Click **BD_Idle** to return to the standing BATTLEDROID reference. Do not apply
transforms, change origins, clear parent inverses, or edit meshes to switch poses.

## Hierarchy, units, and coordinate conversion

The source Blender file has 71 objects: the 68 runtime nodes and three existing
helper empties (`Empty`, `Empty.001`, and `Empty.002`). All remain in the authoring
copy. Export selects the production 68-node hierarchy rooted at `TC167Root`.

| Nodes | Existing parent | Role |
| --- | --- | --- |
| Fuselage | TC167Root | Main body configuration/lean |
| TorsoCore, Head | Fuselage | Torso and head articulation |
| Left/RightIntake | TC167Root | Hip carrier and intake geometry |
| Left/RightUpperLeg | Corresponding Intake | Hip articulation |
| Left/RightLowerLeg | Corresponding UpperLeg | Knee articulation |
| Left/RightFoot | Corresponding LowerLeg | Ankle and foot placement |
| Left/RightShoulder | TC167Root | Shoulder pivot |
| Left/RightUpperArm | Corresponding Shoulder | Arm swing |
| Left/RightForearm, Hand | Corresponding UpperArm, Forearm | Elbow and hand chain |
| Left/RightEngine | TC167Root | Root siblings whose foot attachments are baked into object keys |

Mesh companions keep their original parents, geometry, and material assignments.
The existing `BATTLEDROID_POSE` supplies the absolute parent-local reference values;
`TransformationController.reset(BATTLEDROID)` also evaluates the authoritative
engine attachment transforms. The idle cycle's first pose matches that result
within **0.000000573** maximum matrix-component error after Blender/float32 export.

Source glTF coordinates use +Y up and +Z toward the nose. Blender uses +Z up:

```text
Blender (X, Y, Z) = glTF (X, -Z, Y)
Blender quaternion = (w, x, y, z)
glTF quaternion    = (x, y, z, w)
```

For conversion matrix `C`, source node-local matrices become `C M C⁻¹`. Six source
mesh objects have nonidentity parent inverses: the three right intake surfaces and
the right upper-leg, lower-leg, and foot mesh companions. The keyed Blender basis
is `parent_inverse⁻¹ C M C⁻¹`; their existing parent inverses remain unchanged.
The game's separate 180° Y root conversion is applied by its renderer, not baked
again into the Blender object animation.

One unit is one meter. The canonical root datum is 3.2873 m upward, and the soles
stand near Blender Z = −3.2 m because the model is expressed relative to the
physical center of mass. The root datum stays constant within each loop. The game
adds its world-space vehicle position independently.

## Export saved artist edits

Run from the project root after saving the Blender file:

```sh
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/author_tc167_blender.py -- \
  --export-only \
  --blend-input assets/blender/TC167_Animated.blend \
  --glb-output assets/models/vehicles/tc167_animated.glb

python -m tools.inspect_glb_animations \
  assets/models/vehicles/tc167_animated.glb --require-bd-clips

python main.py --vehicle-model assets/models/vehicles/tc167_animated.glb \
  --battledroid-test ground --enemies 0
```

`--export-only` reads your saved FCurves and exports their current values. It does
not regenerate the starter cycles or save over the `.blend` file. A verification
edit to an actual torso keyframe was exported and detected in the resulting glTF
channel values.

The exporter restores the original FIGHTER basis for the temporary Blender export,
uses Actions mode with merge-by-Action, samples at one-frame intervals, and slides
clip time to zero. It selects the production hierarchy and exports no skin, morph,
camera, light, or flattened hierarchy.

For the game GLB it retains the original production document's geometry, materials,
node-local rest TRS, and binary geometry prefix exactly, then appends animation
accessors decoded from Blender's export. Targets are mapped by unchanged node
names to their original indices. This keeps the saved BATTLEDROID authoring pose
from becoming a new incompatible bind pose. Use this command when exporting to
the M21 runtime so the production rest/material contract remains intact.

Mesh, material-assignment, hierarchy, or parent-inverse edits are rejected by the
saved geometry baseline. Animation edits remain editable and exportable. The
original `TC167.blend` and `tc167.glb` are protected output paths.

To generate another starter copy, choose a new output filename:

```sh
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/author_tc167_blender.py -- \
  --blend-output assets/blender/TC167_Animated_New.blend \
  --glb-output assets/models/vehicles/tc167_animated_new.glb
```

Creation refuses to overwrite an existing authoring file. Use export-only for
normal work on the prepared file.

## Rendered samples

These Blender Workbench renders use a temporary preview floor and camera that were
not saved into the model file.

| Idle | Walk | Run |
| --- | --- | --- |
| ![Idle](images/TC167_BD_Idle.png) | ![Walk](images/TC167_BD_Walk.png) | ![Run](images/TC167_BD_Run.png) |

## Validation

- Original Blender and production GLB SHA-256 hashes remain unchanged.
- Reopened authoring copy: object names, hierarchy, parent inverses, mesh
  coordinates/topology/UVs, material assignments, and material shader data match
  the source. Unused source materials/meshes are retained using fake users.
- All Actions contain genuine keyframes, with matching endpoints and no root
  displacement. The file contains no armature.
- Native GLB geometry/material arrays and FIGHTER rest matrices are exact copies
  of production data; all three clips pass the M21 loader.
- Idle feet are fixed. Dense native sampling found walk/run contact interpolation
  error below 2.1 mm around the canonical floor; swing feet lift as intended.
- Seven new asset tests pass, including full game updates with identical physics
  for native/procedural backends and preserved transformation endpoints.
- Complete suite: **427 tests, 425 passed, two unchanged pre-existing failures**
  concerning wing bounds and mirrored pose scales. Existing 35 lint findings remain.

Detailed machine-readable results are in
[TC167_AUTHORING_VALIDATION.json](TC167_AUTHORING_VALIDATION.json). Physics,
locomotion decisions, transformation logic, and the M21 animation engine were not
changed for this milestone.
