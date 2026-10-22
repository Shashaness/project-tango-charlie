# M21.3 — configuration-driven TC167 authoring

Implemented in **Blender 5.0.1**. Gameplay and engine modules are unchanged. No
commits, pushes or tags were made.

## Files

Created:

- `assets/blender/TC167_Authoring.blend`
- `assets/models/vehicles/tc167_authoring.glb`
- `tools/tc167_authoring_controls.py` (self-contained embedded helper source)
- `tools/author_tc167_authoring.py` (creation and saved-edit export)
- `tools/validate_tc167_authoring_blender.py`
- `tools/test_tc167_authoring_blender.py`
- `tools/tc167_runtime_pose_bridge.py`
- `tests/test_tc167_authoring.py`
- `docs/TC167_AUTHORING_WORKFLOW_VALIDATION.json`
- `docs/TC167_RUNTIME_POSE_CANDIDATE.json`
- This report.

Updated `docs/TC167_ANIMATION_AUTHORING.md` with the practical **Redesigning the
VTOL Configuration** tutorial, Undo/recovery, runtime authority, and explicit
candidate migration workflow. Existing M21.2 uncommitted work is retained.

The original, M21.1 and M21.2 Blender/GLB assets remain unchanged. The authoring
copy retains exact object names, hierarchy, mesh geometry, material assignments,
parent inverses and production rest metadata. It contains no skeletal rig.

## Sidebar and data model

Configurations remain the standard shared Actions `TC_Pose_Fighter`,
`TC_Pose_VTOL`, and `TC_Pose_Battledroid`. Within Blender these are authoritative.
The sidebar adds Save Configuration Pose, Preview Endpoint Changes, confirmed
Sync Transformation Endpoints, and Validate Configuration Consistency. Existing
configuration/transition selection, BD_Idle/Walk/Run and playback remain available.

The helper captures evaluated local transforms into the original object basis by
compensating for its unchanged parent inverse. A manifest identifies 43 editable
pivot/articulation nodes, including existing intentionally transformed shell meshes.
Unchanged decorative meshes are excluded. Changes smaller than 2e-5 are ignored to
avoid float-noise rewrites. Saving changes only the selected configuration's
changed TRS channel groups, preserving unrelated Actions. Static pose keys are
updated together, and node/channel old/new values are reported.

Explicit edge relationships associate each transformation with its source and
destination configurations. A sync plan compares the chosen configuration with
all entering/leaving endpoints; VTOL affects four, FIGHTER/BATTLEDROID two each.
Translation and scale comparisons are component-wise; quaternion signs are aligned
to the nearest surviving interior key. The updater changes endpoint values only.
It does not retime clips, move keys, rewrite interior values/handles or smooth
choreography. Missing components receive endpoint keys; missing slots receive
correct muted NLA export bindings. Existing channels are resolved per slot and
path/index rather than assuming uniform Actions.

The preview lists configuration, Action, node, channel, first/last frame, numerical
old/new values, differences and added components in `TC167_Last_Report.json`.
The dialog displays the first 14 rows and links conceptually to that complete Text
report; an explicit checkbox and OK are required. Cancel/unchecked confirmation
changes no keys. Stale plans are rejected. Both editing operations are Blender
Undo operators. No synchronization runs on selection, playback, loading or helper
registration. Preview/validation only update their report Text.

## Validation and export

The read-only validator checks endpoints, missing channels, unexpected slot targets
and decorative animation, finite keys/handles, quaternion norms, hierarchy, pivots,
geometry/origin changes, unsupported rigs/constraints/drivers, and immutable rest
metadata. It reports Action/node/channel/frame details. Motion diagnostics warn
about adjacent-key translation, rotation, scale and quaternion sign discontinuities.
A conservative major-shell AABB test checks endpoints/midpoints and reports possible
overlaps. No intermediate keys are changed to suppress warnings.

The initial authoring file has **zero consistency errors** and **46 warnings**,
including inherited mirrored-scale zero crossings and conservative overlap flags.
All 11 retained Blender Actions (including legacy FIGHTER_Rest) survive reopening.
All ten intended clips export:

| Clips | Duration | Slots / targeted nodes |
| --- | --- | ---: |
| TC_Pose_Fighter / VTOL / Battledroid | 1/120 s constant | 68 each |
| TC_FighterToVTOL / TC_VTOLToFighter | 1.5 s | 68 each |
| TC_VTOLToBattledroid / TC_BattledroidToVTOL | 1.9 s | 68 each |
| BD_Idle | 2 s | 36 |
| BD_Walk | 1 s | 36 |
| BD_Run | 0.75 s | 36 |

Export validates consistency before writing, appends animation data to the
production geometry/material/rest document, and maps unchanged target node names
to original indices. Initial native gait samplers are preserved exactly; export-only
uses artist-edited curves. Configuration-to-transition endpoint error in the initial
GLB is below **3e-8 per local matrix component**. Candidate JSON → runtime
PoseTransform → model matrix round trips have **zero error**. Current Python
configuration comparisons have no mismatched nodes. The edited-VTOL integration
test verifies these comparisons also survive a real edited export, including a
repaired missing multi-object Action slot.

## Runtime ownership and explicit migration

Python remains authoritative in gameplay: `TransformationController` reads
`game/tc167_poses.py`, resets FIGHTER to production rest and computes engine-to-foot
attachments. BD gait/contact layers additionally own visual overrides when enabled;
they are suppressed during active transformations. Exported configuration and
transformation clips are not automatically selected by the game.

`python -m tools.tc167_runtime_pose_bridge INPUT.glb --output CANDIDATE.json
--python-output CANDIDATE.py` explicitly generates full local glTF TRS and an
optional runtime `VehicleMode → node → PoseTransform` mapping. It validates source
hierarchy/rest and every clip endpoint, round-trips through the runtime model API,
and lists differences from Python configurations. Outputs inside game/, engine/
and assets/ are prohibited. `apply_candidate()` applies the complete candidate to
an isolated CPU model for developer inspection. No joint-angle copying is needed.

These are reviewable runtime data candidates, not an installed controller change.
The existing controller would override engines and bypass edited FIGHTER data;
a separately reviewed integration must select one owner for those writes and
validate transformations/gaits before enabling the candidate. M21.3 intentionally
preserves current gameplay throughout.

## Tests

Complete Python suite: **435 tests in 127.489 seconds; 433 passed, 2 failed**.
All three new M21.3 runtime/asset tests passed. The unchanged, previously documented
failures are:

- `test_pose_refinement.PoseRefinementTests.test_wings_fold_as_back_planes_without_scaling_or_hiding`
- `test_transformation.TransformationTests.test_norm_quaternions_positive_scales_and_hierarchy_every_sample`

They concern original wing bounds and positive-scale expectations for mirrored
source poses. New Python files pass Ruff; `git diff --check` passes.

All **nine Blender integration tests passed**. They exercise saving revised VTOL/FIGHTER/BATTLEDROID poses,
all related endpoints, exact interior key/handle and timing preservation,
unrelated Actions, slots and missing components, confirmation/cancellation/stale
plans, Undo, repeated sync, structural validation, warning-only behavior, selection
without saving, missing endpoint insertion, and edited GLB/runtime round trips.
Machine-readable counts and details are in the workflow validation JSON.

## Exact VTOL redesign steps

1. Open `TC167_Authoring.blend` and run embedded `TC167_Action_Controls.py`.
2. In Viewport → N → TC167, click VTOL.
3. Select `LeftWing` and move/rotate its pivot using normal object transforms.
4. Click Save Configuration Pose; inspect `TC167_Last_Report.json` and save the file.
5. Click Preview Endpoint Changes and inspect all four VTOL-related endpoints.
6. Click Sync Transformation Endpoints, check the confirmation box and click OK.
7. Validate Configuration Consistency; inspect warnings, play/scrub each clip, and
   refine interiors in the Graph Editor if necessary. Ctrl-Z undoes synchronization.
8. Save, then run `tools/author_tc167_authoring.py -- --export-only` in Blender.
9. Run the Blender validator and pose bridge; review the generated runtime candidate
   before any later explicit developer integration. Gameplay remains unchanged.

## Limits

Interactive dialog layout, manual UI clicking and artistic appearance still need
visual review. Real background Blender evaluates the Actions, embedded operations
and Undo stack. Collision warnings use only five major-shell AABBs at three times
per clip, so they can produce false positives and miss collisions. Adjacent-key
thresholds are heuristics, not continuous velocity/acceleration guarantees. Existing
mirrored scale collapse/reflection remains intentional source behavior. BATTLEDROID
redesigns do not automatically regenerate gait clips. Leg edits do not regenerate
engine interior choreography. Constraints, drivers and sheared transforms are
outside the supported rigid-node authoring workflow.
