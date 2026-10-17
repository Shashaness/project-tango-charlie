# M21 animation foundation — implementation and validation

Implemented on 2026-10-12. Existing working-tree changes and assets were preserved.
No commit, push, or tag was performed. No runtime dependency was added.

## Delivered files

| File | Responsibility |
| --- | --- |
| `engine/gltf_loader.py` | Decode/validate animation channels and float32 samplers; accept VEC4 rotation output. |
| `engine/gltf_animation.py` | Immutable native clips/channels/samplers; timed playback and TRS crossfades. |
| `engine/model.py` | Native clip inventory, canonical TRS references, and independent absolute visual poses. |
| `game/battledroid_animation.py` | Optional BD_Idle/BD_Walk/BD_Run selection, velocity-scaled playback, and fallback. |
| `game/battledroid_locomotion.py` | Preserve state/velocity observations while suppressing overlapping procedural writes. |
| `engine/game.py` | Optional player GLB input and native-backend configuration retained across resets. |
| `main.py` | Add `--vehicle-model` and `--no-gltf-animation`; keyboard controls are unchanged. |
| `tools/inspect_glb_animations.py` | Headless clip/channel inventory and optional complete BD clip-set validation. |
| `tests/test_gltf_animation.py` | 31 native import, playback, integration, and physical-invariance regressions. |
| `docs/BLENDER_GLTF_ANIMATION.md` | Blender 5.0.1 workflow, export settings, APIs, ownership, and future layering contract. |
| `README.md` | Link the workflow and expose the inspection/startup commands. |

## Automated results

| Check | Result |
| --- | --- |
| Baseline complete suite | 389 tests: 387 passed, 2 existing failures |
| New native-animation tests | 31 tests passed |
| Complete suite with M21 | 420 tests: 418 passed, the same 2 existing failures |
| Existing model-import subset | 22 tests passed |
| Existing animation subset | 12 tests passed |
| Existing locomotion subset | 16 tests passed |
| Ruff formatting and E501/import checks | Passed |
| Ruff checks on new animation code and loader/model/game integration | Passed |
| Complete Ruff check | Same 35 existing findings; no new diagnostic identities |
| Python compilation and `git diff --check` | Passed |

The final complete suite took **106.084 seconds** in the existing Python 3.14
virtual environment. All 31 added tests passed again after preserving locomotion
state/velocity telemetry while native joint writes are active.

The complete runs cover flight, atmosphere, ground operations, combat, defenses,
camera, model loading, and configuration/transformation regressions. A direct test
runs identical full game updates with native playback enabled/disabled and compares
position, velocity, acceleration, orientation, throttle, and flight state exactly.

The new suite also checks malformed target/sampler references, duplicate/conflicting
channels/names, unsupported interpolation/skinning, accessor types/counts/alignment/
bounds, finite/increasing keyframes, normalization, shortest-arc and STEP rotation,
translation/scale blending, looping/one-shot endpoints, pause/resume/stop/speed,
interrupted fades, frame partitions, missing/disabled clips, drift, unchanged rest
and configuration poses, engine attachment, and zero visual root/foot scale.

## Existing failures retained

1. `test_pose_refinement.PoseRefinementTests.test_wings_fold_as_back_planes_without_scaling_or_hiding`:
   wing extent remains **4.887896060943603 m**, above the asserted **2.5 m** bound.
2. `test_transformation.TransformationTests.test_norm_quaternions_positive_scales_and_hierarchy_every_sample`:
   the existing positive-scale assertion still fails on canonical pose data.

Both failures were reproduced before M21 and remain identical afterward. The code
and model/pose data responsible for them were not changed. Complete lint findings
remain 23 F401, five E741, four F405, one F403, one E731, and one F841; these are the
existing findings recorded by the readability pass.

## Installed Blender round-trip

Blender **5.0.1**, build **a3db93c5b259**, imported the repository TC167 GLB in a
temporary background scene. Three shared slotted Actions were assigned to the two
upper-leg objects, retained in NLA tracks, and exported with Actions mode, merge by
Action, animation sampling enabled at one-frame intervals, and slide-to-zero enabled.

The temporary export contained `BD_Idle`, `BD_Walk`, and `BD_Run`, each **1 second**
long with **six channels**: translation, quaternion rotation, and scale for each
upper leg. Rotation exported as LINEAR with 25 keys; constant translation/scale
exported as STEP with two keys. The runtime loader and inspection utility accepted
the file, including `--require-bd-clips`.

Every selected node name and parent relationship matched the original loaded model.
A headless Game instance loaded this actual Blender export, completed 120 updates at
1/120 second, and selected `BD_Walk` from physical movement. The exported file and
logs stayed under `/tmp/m21-blender`; no repository GLB or `.blend` was rewritten.
The animation used for this smoke check is a simple authoring/import proof, not a
production gait or a migrated transformation asset.

## Scope and limitations

The repository TC167 GLB still has zero native clips and therefore retains its
procedural behavior. Native clips are available through compatible exported models.
Idle/walk/run are the only migrated visual states; airborne, landing, recovery,
turning, and stopping retain the procedural/JSON implementation.

Crossfades are between native clips; backend changes switch visual ownership before
the procedural path resumes its existing blend. Reverse/strafe-specific authored
clips, IK, foot locking, skeletal skinning, CUBICSPLINE, and transformation migration
are deferred. The canonical transformation controller remains the pose/physics-switch
owner, and native gait is disabled during its transitions and completion frame.

Interactive viewport rendering was not exercised. Verification includes headless
game playback, matrix/hierarchy checks, mocked graphics regressions, and a real
Blender export/import round-trip; it does not substitute for artist visual approval.
