# M21 — Blender/glTF rigid-node animation foundation

TC167 can now load and play native glTF 2.0 object animation. Its rigid-node
hierarchy, physical integration, locomotion decisions, and canonical configuration
transitions remain in place. The repository vehicle currently has no native clips,
so its normal gameplay continues using the existing procedural/JSON animations.

M21.1 supplies a separate `assets/blender/TC167_Animated.blend` with editable Actions
and `assets/models/vehicles/tc167_animated.glb` with native idle/walk/run clips.
See the [prepared artist setup and export instructions](TC167_ANIMATION_AUTHORING.md).

## Supported asset contract

The loader accepts translation and scale VEC3 channels and rotation VEC4 channels
in `[x, y, z, w]` order, with float32 SCALAR key times in seconds. It validates
node/sampler indices, accessor widths/counts/alignment/bounds, finite values,
nonnegative strictly increasing times, nonzero quaternions, and duplicate node/path
channels. Valid rotations are normalized on import. LINEAR vectors use linear
interpolation; LINEAR rotations use shortest-arc SLERP. STEP holds the previous key
until the next timestamp. These semantics follow the
[Khronos glTF 2.0 animation specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html).

CUBICSPLINE, morph weights, skeletal skinning, animation pointers, sparse accessors,
and animation of matrix-authored nodes are outside M21 and fail validation. The
existing geometry/material subset is unchanged: triangles, supported attributes,
and opaque untextured materials. Clips may share samplers and target different TRS
properties of the same node. Duplicate channels for the same property are rejected.

Named clips must have unique nonempty names. An unnamed animation receives the
stable lookup key `Animation_<document index>`; model node names are never changed.
Channel indices retain original document order even when scene traversal differs.
Channels targeting nodes outside the selected scene remain valid metadata but do
not make those nodes visible in the selected scene.

## Blender 5.0.1 workflow

The installed application is **Blender 5.0.1**, build `a3db93c5b259`. The settings
below were checked against its bundled `io_scene_gltf2` exporter and exercised by
a temporary import/export round-trip. See the
[Blender 5.0 glTF manual](https://docs.blender.org/manual/en/5.0/addons/import_export/scene_gltf2.html)
for its editor/exporter interface.

1. Import `assets/models/vehicles/tc167.glb` using **File → Import → glTF 2.0**.
   Keep a separate working `.blend` file and export to a new GLB while iterating.
2. Preserve all object/node names, parent relationships, joint pivots, and geometry.
   Keep the unscaled joint objects distinct from their mesh companions. Do not
   flatten the hierarchy, apply new rest transforms, or introduce an armature.
3. Switch the **Dope Sheet** to **Action Editor** and create `BD_Idle`, `BD_Walk`,
   and `BD_Run`. Blender 5 uses slotted Actions: assign one shared Action to the
   participating rigid objects, with a separate object slot for each joint. Stash
   or push down each Action so all three remain associated with their objects.
   Give the NLA tracks matching clip names for clarity; do not use Broadcast mode
   to assign one joint's curves to unrelated joints.
4. Keyframe object location, quaternion rotation, and scale on the existing nodes.
   Use the **Graph Editor** to tune timing/easing. Use **Linear** or **Constant**
   interpolation for direct supported curves, or bake other easing through the
   sampling settings below. Author cycles with matching start/end poses.
5. Author the clips as **absolute local BATTLEDROID poses**, relative to the same
   parents as the configuration controller. They are not offsets to multiply by
   the fighter rest pose. Use the game’s canonical BATTLEDROID stance as the pose
   reference; `game/tc167_poses.py` and the controller define it. Imported fighter
   key values do not automatically become a battledroid stance.
6. Export **File → Export → glTF 2.0**, with **glTF Binary (.glb)** and Animation
   enabled. Use the settings below, then inspect the resulting file before loading
   it as the player model.
7. Run the game with the exported GLB and inspect idle, walking, running, fallback
   states, and configuration transitions. Replace production assets only after
   reviewing the results and preserving the project's geometry/material contract.

Important authoring detail: sampling can export constant translation/scale channels
in addition to keyed rotation. Every exported channel owns that property. If those
constants describe the imported FIGHTER stance, they will override the BATTLEDROID
stance. Key them to the intended BATTLEDROID pose, or omit unnecessary channels.
Properties omitted from a clip retain the current canonical configuration values.
The validation utility lists exactly which properties were exported.

| Blender 5.0.1 export option | Setting for M21 |
| --- | --- |
| Format | glTF Binary (.glb) |
| Animation → Animation Mode | Actions |
| Animation → Bake & Merge → Merge Animation | Actions |
| Animation → Sampling → Always Sample Animations | On for Blender easing/constraints |
| Sampling rate / frame step | 1 frame |
| Sampling Interpolation Fallback | Linear |
| Animation → Rest & Ranges → Slide to Zero | On |
| Use Current Frame as Object Rest Transformations | Off; preserve the imported bind/rest state |
| Limit to Playback Range | Off unless it intentionally contains every Action |
| Filter Actions | Select the three BD Actions if unrelated Actions exist |
| Flatten Object Hierarchy | Off |
| Skinning / Shape Keys | Off |
| Cameras / Punctual Lights / Animation Pointer | Off |
| Compression / mesh optimization extensions | Off |
| Materials | Export supported opaque materials; no textures or extra extensions |
| +Y Up | On, retaining the existing glTF axis conversion |

For automated Blender exports these correspond to `export_animation_mode='ACTIONS'`,
`export_merge_animation='ACTION'`, `export_force_sampling=True`, `export_frame_step=1`,
`export_anim_slide_to_zero=True`, `export_current_frame=False`, and
`export_hierarchy_flatten_objs=False`. The defaults/settings were verified locally
on 5.0.1 rather than inferred from an older NLA-only exporter workflow.

## Validate and run

From the project root, with the virtual environment active:

```sh
python -m tools.inspect_glb_animations path/to/tc167-animated.glb
python -m tools.inspect_glb_animations path/to/tc167-animated.glb --require-bd-clips
python main.py --vehicle-model path/to/tc167-animated.glb --battledroid-test ground --enemies 0
python main.py --vehicle-model path/to/tc167-animated.glb --no-gltf-animation --enemies 0
```

The utility uses the same loader as the game, prints names, durations, original node
indices/names, target paths, interpolation, key counts, and time ranges, and exits
with status 1 for invalid files. `--require-bd-clips` additionally requires all three
BD names; the game itself allows partial sets and falls back for missing states.
Running this inspection needs no display or GPU context.

`--vehicle-model` must supply the same TC167 named hierarchy and geometry. It does
not replace the physics collider with a new shape. `--no-gltf-animation` keeps all
procedural/JSON animation paths. The existing `--no-locomotion-animation` disables
both gait backends without changing physics. These options add no keyboard mapping.

## Native player API

`Model.animations` maps names to `engine.gltf_animation.AnimationClip` objects.
Each clip contains immutable channels/samplers bound to CPU node objects; it has
a duration in seconds and samples absolute local component values. The timeline
starts at zero and ends at the final key time. Channels clamp at their endpoints.

```python
from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import load_glb

model = load_glb("path/to/tc167-animated.glb")
player = AnimationPlayer(model)
player.play("BD_Idle", loop=True)
player.update(1 / 60)  # seconds; use the actual frame delta in a frame loop
player.apply()  # imported rest supplies omitted TRS properties by default
player.play("BD_Walk", speed=0.8, transition=0.12)
player.pause()
player.resume()
player.stop()  # reveals configuration/rest without changing it
```

`update(dt)` advances time; `apply(reference)` writes fresh absolute visual matrices.
To layer over a configuration, pass `{node: node.configuration_trs}` for TRS-authored
nodes. The battledroid adapter does this automatically. Speed is a finite nonnegative
multiplier; zero holds the timeline. One-shot clips clamp to their final pose and
set `finished`; looping clips wrap to zero at the duration boundary. Pause holds
both clocks. Stop removes this player's owned visual overrides.

Crossfades freeze the outgoing evaluated pose and advance the incoming clip. An
interrupted fade starts from its current blended pose. Translation and scale blend
linearly, rotation uses SLERP, and absent components blend to the reference pose.
Fade duration uses real seconds, independent of playback speed. No transform is
multiplied onto the previous animation frame.

## BATTLEDROID selection and fallback

The existing state machine still observes grounded state, commanded locomotion,
actual tangent velocity, touchdown events, stopping, and running hysteresis.

| Existing state | Optional clip | Visual playback speed |
| --- | --- | --- |
| IDLE | BD_Idle | 1× |
| WALKING | BD_Walk | Actual tangent speed / 2 m/s |
| RUNNING | BD_Run | Actual tangent speed / 6 m/s |
| Other states or missing/disabled clip | Existing procedural/JSON path | Existing behavior |

The nominal speeds are animation authoring references, not new physics parameters.
Author one walk/run cycle at those speeds for useful synchronization. Reverse and
lateral motion currently reuse the same clip direction; directional clips and
foot-lock/IK are future work.

Native-to-native changes fade over 0.12 seconds. Entering/leaving the native backend
switches ownership immediately; procedural blending then follows its existing
rules. During native gait the entire procedural gait and contact/idle visual layer
is suppressed, preventing two writers on the same joints. Unauthored joints stay
canonical. Engines follow their feet through the existing sibling attachment
relationship, unless an explicit engine channel supplies their pose.

## Configuration compatibility and future transformations

The visual order is:

1. Clear the previous frame's owned visual poses and deltas.
2. Run the existing transformation controller and physical integration.
3. Use the existing locomotion state machine to choose one animation backend.
4. Evaluate native absolute local TRS over the canonical configuration reference,
   or run the procedural/JSON path.
5. Resolve visual engine attachments and traverse each parent chain once for rendering.

`ModelNode.base_matrix`/`base_trs` remain imported rest. `configuration_matrix` and
`configuration_trs` remain owned by the transformation controller. A native player's
separate absolute visual pose can temporarily replace a targeted local matrix,
followed by any explicitly owned visual delta. Missing channels retain the canonical
TRS components, so rotation-only clips do not reset joint translations or scale.

Never calculate `configuration_matrix @ absolute_clip_matrix`: that double-applies
translations, rotations, and root motion. Absolute clips replace/blend local TRS;
deliberate additive clips require an explicitly defined reference-relative delta.
Visual root movement is not copied into world-space vehicle position or velocity.

Future transformation clips should get one configuration-pose owner and an explicit
transition/reference contract, with locomotion disabled during the transition.
The existing controller must continue owning configuration requests, reversal,
physics-switch hysteresis, and exact endpoints. Native gait is stopped during an
active transformation and skipped on its completion frame. M21 does not migrate
any FIGHTER/VTOL/BATTLEDROID transformation curve or pose data.

## Verification

The new synthetic tests cover import validation, translation/scale/quaternion
interpolation, STEP, loop/one-shot boundaries, controls, interrupted crossfades,
frame partition independence, missing-clip/disabled fallback, drift, foot attachments,
zero visual scale, canonical transitions, and equal physical state with either backend.
Run `python -m unittest discover -s tests` for the complete regression suite.

See [M21 validation results](M21_ANIMATION_RESULTS.md) for test counts, existing
failures, and the Blender round-trip evidence.
