# M21.2 — complete TC167 rigid-node authoring

Implemented with **Blender 5.0.1**. No game/engine modules changed. No commits,
pushes, or tags were made.

## Deliverables

- `assets/blender/TC167_Complete.blend`: saved editable multi-object Actions,
  configurations, embedded `TC167_Action_Controls.py` panel, original geometry,
  materials, hierarchy, pivots, and parent inverses.
- `assets/models/vehicles/tc167_complete.glb`: ten runtime-readable clips over
  unchanged production geometry/materials/rest hierarchy.
- `tools/author_tc167_complete.py`: reproducible creation and saved-edit export.
- `tools/validate_tc167_complete_blender.py`: reopened-file and runtime validation.
- `tests/test_tc167_complete.py`: five asset and gameplay regression tests.
- `docs/TC167_ANIMATION_AUTHORING.md`: beginner workflow, slot editing, export,
  coordinate conventions, validation tolerances, and runtime boundaries.
- `docs/TC167_COMPLETE_VALIDATION.json`: numerical results and protected hashes.
- This report.

The four original files (`TC167.blend`, `TC167_Animated.blend`, `tc167.glb`, and
`tc167_animated.glb`) retain their starting SHA-256 hashes. No armature, skinning,
mesh changes, origin changes, destructive transform application, or gameplay
control changes were introduced.

## Configurations and Action inventory

Reference presets are FIGHTER, VTOL, and BATTLEDROID, stored as
`TC_Pose_Fighter`, `TC_Pose_VTOL`, and `TC_Pose_Battledroid` (68 slots each,
frames 1–2, constant pose). The original rest-basis metadata remains embedded.

| Action | Duration | Frames at 120 FPS | Targeted nodes | Saved keyframe points |
| --- | ---: | --- | ---: | ---: |
| TC_FighterToVTOL | 1.5 s | 1–181 | 68 | 2,980 |
| TC_VTOLToFighter | 1.5 s | 1–181 | 68 | 2,980 |
| TC_VTOLToBattledroid | 1.9 s | 1–229 | 68 | 8,685 |
| TC_BattledroidToVTOL | 1.9 s | 1–229 | 68 | 8,685 |
| BD_Idle | 2 s | 1–241 | 36 | 17,640 |
| BD_Walk | 1 s | 1–121 | 36 | 9,000 |
| BD_Run | 0.75 s | 1–91 | 36 | 6,840 |

Transformation Actions export 204 channels each; locomotion exports 108 each.
All slots are assigned together by the panel, with NLA evaluation disabled.
Muted archive strips preserve export bindings. Fake users retain Actions across
reopening. Constant transformation components have only two keys. Engine
attachment curves require up to 154 keys to stay within the reduction tolerance;
most mechanical joint channels use far fewer. No duplicate per-object Actions
were created. Existing locomotion shapes and durations are preserved; the new
GLB's initial gait samplers are exact copies of the existing animated GLB.

## Validation

Background Blender reopened the deliverable, registered its actual embedded
panel, selected all presets and Actions, evaluated 241 timeline positions per
transformation and every gait frame, and verified the geometry/hierarchy/pivot
fingerprint and absence of armatures. Both forward and reverse endpoints match
the game configurations. Blender maximum endpoint error is 5.25e-7 per local
matrix component; maximum intermediate error is below 0.00132. Runtime results
are recorded separately in the JSON report and pass the 0.004 intermediate and
2e-5 endpoint bounds.

Asset tests check original node indices/names/parents, geometry bytes and material
metadata, duration/channel inventory, finite world matrices, procedural reference
poses throughout transitions, exact native gait samples, reversible paths, joins
between clips, and repeated application without drift. Full game comparisons using
the old and new assets cover flight, taxi, VTOL, BATTLEDROID, camera position, gun
cooldown, and transformation ownership. Procedural transformations remain
responsible for gameplay; transformation clips never automatically play in-game.

Export-only was tested on a temporary saved Blender copy: adding 0.025 m to a
Fuselage key channel produced 0.025 m in the exported runtime sampler. The
original deliverables were unaffected. New Python files pass Ruff and compilation;
`git diff --check` passes.

Full suite: `python -m unittest discover -s tests` ran **432 tests in 122.900 s**:
**430 passed, 2 failed**. All five new M21.2 tests passed. The two unchanged,
previously documented failures are:

- `test_pose_refinement.PoseRefinementTests.test_wings_fold_as_back_planes_without_scaling_or_hiding`:
  original wing bounds reach 4.887896 m against a 2.5 m expectation.
- `test_transformation.TransformationTests.test_norm_quaternions_positive_scales_and_hierarchy_every_sample`:
  original mirrored source poses violate the test's positive-scale expectation.


## Preview

1. Open `assets/blender/TC167_Complete.blend` in Blender 5.0.1.
2. In a Text Editor, select `TC167_Action_Controls.py` and Run Script (Alt-P).
3. In the 3D Viewport press N, then open TC167.
4. Click FIGHTER, then FIGHTER → VTOL. Press Space; pause at frame 181.
5. Click VTOL → BATTLEDROID. Press Space; pause at frame 229.
6. Use the reverse buttons to fold back. Select individual pivots to edit their
   shared Action slots in the Action Editor and Graph Editor; save before export.

## Limits

Interactive UI clicking, real-time playback appearance, and artistic approval were
not manually verified; background dependency-graph evaluation and helper selection
were verified. Interpolation bounds are local matrix-component errors, not bounds
on every rendered vertex. No intentional artistic changes were made. The existing
controller's 3.2873 m visual root lift and mirrored-scale zero crossings remain;
no additional root-world motion is authored. Changing leg keys does not
procedurally regenerate engine attachment keys. Export-only samples artist-edited
gaits, so it may introduce small interpolation differences compared with the
initial exact-copy native gait samplers. Standard viewport playback loops the
range; pause at the destination for a one-shot inspection.
