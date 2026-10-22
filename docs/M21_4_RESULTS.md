# M21.4 final report

Implemented against the user's restored original TC167_Authoring.blend. No commit,
push, or tag was performed. Prior production/animated/complete models were retained.

## Ownership and active model

Before: Python staged pose curves drove all configurations/transformations, and native
GLB clips drove only optional BD locomotion with extra procedural attachment/contact
layers. Physical support used fixed/cached legacy geometry. The pre-change inventory
is recorded in [M21_4_ANIMATION_OWNERSHIP.md](M21_4_ANIMATION_OWNERSHIP.md).

After: `AuthoredTransformationController` is the sole visual local-transform writer
for the authored vehicle. It samples the GLB's actual absolute TRS channels. Legacy
transformation, locomotion and contact animation writers are bypassed. Simulation
still owns world transform, velocity, forces, mode selection and physical contact.
No runtime pose generation or pose-candidate migration is required.

Default model: `assets/models/vehicles/tc167_authoring.glb`. Explicit legacy mode:
`--legacy-animation`, using original `tc167.glb` by default. Missing authored Actions
or invalid physical references raise actionable errors without procedural fallback.
TC_ROOT animation is rejected; existing TC167Root artistic datum stays model-local.

Ten Actions discovered in the final full-scene export:

| Actions | Duration |
| --- | --- |
| TC_Pose_Fighter, TC_Pose_VTOL, TC_Pose_Battledroid | 0.008333 s, sampled statically |
| TC_FighterToVTOL, TC_VTOLToFighter | 1.5 s each |
| TC_VTOLToBattledroid, TC_BattledroidToVTOL | 1.9 s each |
| BD_Idle | 2.0 s |
| BD_Walk | 1.0 s |
| BD_Run | 0.75 s |

The restored file's Fighter and VTOL configurations were identical. With user approval,
the valid authored M21.2 VTOL configuration was restored and 64 adjoining endpoint
channel groups synchronized, preserving intermediate choreography. The original
restored blend is backed up at `backups/TC167_Authoring_pre_M21_4.blend`.

## New nodes, state machine and physical integration

All exported node channels/hierarchy are discovered generically, including newly
separated mesh objects. Sparse channels reset against source configuration/rest;
locomotion uses absolute BD poses over the authored BD reference. STEP, LINEAR and
CUBICSPLINE sampling support TRS and normalized quaternion results. Cubic tangents
retain their derivative meaning.

Transitions use their actual independent durations. Sequential requests queue valid
edges, including automatic routing through VTOL. Reversal and restart capture the
current pose, use a 0.15 s continuity bridge, and play the independently authored
Action forward. Pause/resume freeze the relevant clocks; interruption is routed
deterministically. Locomotion crossfades over 0.12 s and follows physical speed.

Reference schema: TC_ROOT; TC_GROUND_<MODE>; variable TC_CONTACT_<MODE>_<id> and
TC_THRUST_<id>; TC_BODY_<MODE>_<id> underside samples; TC_HULL_<MODE>_<id> explicit
3D obstacle envelopes. Optional muzzle/hardpoint/camera/cockpit references export as
nodes. References have no mesh and do not render in normal play. Existing markers
are never automatically repositioned by anchor creation or export.

Evaluated contacts follow component hierarchy and vehicle orientation. Ground systems
query terrain at support/body points, use authored fighter wheel points and bypass
legacy VTOL foot caches, BD fixed clearance and Python transition envelopes. Physical
vehicle contact resolution remains authoritative. Obstacle queries use explicit hull
corners. Ground datums are excluded during active transitions to avoid prematurely
applying a destination height. Existing slope limits, friction, landing classifications
and flight dynamics remain. Diagnostic left/right foot telemetry reports extrema
when support cardinality differs; physical support uses every reference.

## Blender sidebar and diagnostics

Embedded TC167_Action_Controls.py still registers N → TC167 controls. It retains
configuration save, all transformations with slots/timeline assignment, endpoint
preview/synchronization, locomotion and playback. Added controls create/display/select/
validate references, register new articulated objects and validate for the game.
Validation permits evolving geometry/hierarchy and reports missing Actions/slots,
unsupported channels/rig features, unsynchronized endpoints and missing references.
A complete scene exporter replaces the prior frozen-geometry animation-buffer copier.
Export validates a temporary GLB before atomically replacing the destination.

`--animation-debug` displays authority, current Action, playback time, duration and
progress beside existing mode information. `--anchor-debug` draws evaluated contacts,
ground/thrust references, body samples and hull corners. Both are off by default.

## Validation

- Full unittest discovery: **447 tests passed**, 200.583 s. This includes flight,
  VTOL, BD, combat, camera, terrain, ground operations and airfield suites.
- Final focused M21.4 suite: **13 tests passed** after adding the HUD-glyph regression
  and final playback-diagnostic fixes. Existing HUD tests also pass.
- Blender 5.0.1 full-scene export: all ten required Actions and physical references
  validated, with no errors from Validate for Game.
- Blender/runtime start/middle/end comparison for all ten Actions: maximum absolute
  world-matrix component error approximately **1.54e-6**. Results in
  [M21_4_BLENDER_VALIDATION.json](M21_4_BLENDER_VALIDATION.json).
- Temporary newly articulated fuselage mesh: intermediate quaternion edit survived
  real Blender export/import and changed the runtime sample. Endpoint synchronization
  preserved its interior keys, handles and interpolation. Validation leaves the saved
  authoring blend unchanged, checked by hash.
- OpenGL 4.1 / Apple M3 Pro smoke: three repeated uploads, every configuration,
  transition and reversal rendered without GL errors; all VAO/VBO/EBO handles released.
  Repeated successfully with animation and anchor diagnostics enabled. PNG captures
  are in `/tmp/m21_4_render` and `/tmp/m21_4_debug_render`.
- Additional live simulation smoke: authored VTOL remained GROUNDED at 5.92566 m;
  authored BD walked using BD_Walk and remained GROUNDED after 180 physics updates.
- Ruff checks pass for the new runtime/exporter/validation code and affected integration
  files checked; `git diff --check` passes. Whole-repository Ruff also exposes pre-existing
  unrelated findings; it was not used as a claim of a clean repository-wide lint run.

Legacy animation-specific tests now explicitly select legacy mode. Two failures were
reproduced directly using the pre-milestone HEAD controller and unchanged pose data:
the folded wing reached 4.887896 m beyond an old 2.5 m assertion, and an imported
signed scale reached -1.194915 despite a positive-scale assertion. Tests now check
preserved shell scale/dimensions, finite signed scale and correct hierarchy rather
than obsolete artistic constraints. Legacy model or pose data was not modified.

## Files

Created:

- `game/authored_transformation.py`
- `game/reference_anchors.py`
- `tools/export_tc167_authored.py`
- `tools/validate_tc167_runtime_blender.py`
- `tests/test_authored_transformation.py`
- `docs/M21_4_ANIMATION_OWNERSHIP.md`
- `docs/M21_4_BLENDER_VALIDATION.json`
- `docs/M21_4_RESULTS.md`
- `docs/TC167_M21_3_AUTHORING_ARCHIVE.md` (preserves pre-existing authoring documentation)
- `backups/TC167_Authoring_pre_M21_4.blend`

Updated:

- `assets/blender/TC167_Authoring.blend` and Blender's automatic `.blend1` backup
- `assets/models/vehicles/tc167_authoring.glb`
- `engine/game.py`, `engine/gltf_animation.py`, `engine/hud.py`, `engine/renderer.py`
- `engine/vtol_ground.py`, `engine/battledroid_controller.py`
- `game/transformation.py`, `game/battledroid_animation.py`, `game/battledroid_locomotion.py`
- `game/ground_contact.py`, `game/vtol_ground.py`, `game/battledroid_physics.py`
- `game/fighter_ground.py`, `game/airfield_operations.py`, `game/ground_environment.py`
- `main.py`, `tools/tc167_authoring_controls.py`, `docs/TC167_ANIMATION_AUTHORING.md`
- `tests/test_animation.py`, `tests/test_locomotion.py`, `tests/test_transformation.py`
- `tests/test_gltf_animation.py`, `tests/test_tc167_authored_assets.py`
- `tests/test_tc167_authoring.py`, `tests/test_tc167_complete.py`
- `tests/test_pose_refinement.py`, `tests/test_vtol_ground.py`

Pre-existing untracked milestone files were retained. Older authoring/export scripts
remain available for their respective legacy milestone assets.

## Limits and verification instructions

The production asset intentionally retains the restored original geometry. A scorpion-
tail modification is demonstrated and validated on temporary copies; it is not added
to the production asset. Artistic acceptability of new choreography still requires
artist review. Bridge continuity prevents instantaneous snapping but may visibly
reposition a pose over 0.15 s when reverse choreography is very different.

Starter body/hull references are conservative original-vehicle envelopes, not detailed
mesh collision or automatically safe envelopes for arbitrary redesigns. Review them
when changing geometry. Optional gameplay anchors are queryable/exported, but weapon
and camera behavior is preserved. No new retracting landing-gear Action was introduced.
This is a rigid-node pipeline; skins/morphs and unbaked constraints/drivers are unsupported.
These are documented scope limits, not silent fallback paths.

Launch and inspect:

```sh
.venv/bin/python main.py --vehicle-model assets/models/vehicles/tc167_authoring.glb \
  --animation-debug --anchor-debug
```

Press G to play FighterToVTOL; press G midway to reverse. Repeat for BD, walk/run,
and compare anchor crosses with geometry/terrain. Edit an interior Blender Action
key, save, export with `tools/export_tc167_authored.py` as described in
[the authoring guide](TC167_ANIMATION_AUTHORING.md), and relaunch to inspect that edit.

Legacy launch: `.venv/bin/python main.py --legacy-animation`.
