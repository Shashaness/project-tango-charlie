# M19A — BATTLEDROID animation foundation

TC167 now has a read-only animation state controller and original MIT-licensed
rigid-node offset clips. Physics remains authoritative. No GLB, collision tuning,
control bindings, camera math, weapons, or flight mechanics were changed.
The pre-existing jump control remains available; M19A adds no jump mechanics.

## Repository inspection and integration

The actual rebranded repository uses these modules:

| Concern | Modules |
| --- | --- |
| Physical vehicle and ground locomotion | `game/player_vehicle.py`, `game/battledroid_physics.py`, `engine/battledroid_controller.py` |
| Flat-ground collision and pre-impulse landing speed | `game/ground_contact.py`; shared with `game/vtol_ground.py` |
| Procedural gait | `game/battledroid_locomotion.py` |
| Canonical transformation and mechanical poses | `game/transformation.py`, `game/tc167_poses.py` |
| Rigid hierarchy, GLB loading and drawing | `engine/model.py`, `engine/gltf_loader.py`, `engine/renderer.py` |
| Input and camera modes | `engine/input.py`, `engine/camera.py`, `engine/camera_input.py` |
| Simulation orchestration | `engine/game.py` |
| Existing validation | `tests/test_locomotion.py`, `test_battledroid.py`, `test_transformation.py`, `test_vtol_ground.py`, model/pose/camera tests; `tools/check_transformation_gl.py` |
| Model documentation/assets | `docs/MODELS.md`, `PROTOTYPE.md`, `TRANSFORMATION.md`, `LOCOMOTION.md`; `assets/models/vehicles/tc167_prototype.glb` |

Before M19A, each simulation substep cleared gait deltas, evaluated the canonical
transformation, advanced physics, and evaluated the procedural gait. Engines are
root siblings: the gait manually follows each ankle's change and refreshes exhaust
children. Model nodes already support configuration overrides and local deltas.
The model has all required pivots; no architectural stop condition was encountered.

The new order in `Game.update` is:

1. Clear previous state/clip and gait deltas.
2. Evaluate the existing transformation pose and advance existing physics.
3. Select animation state from current physical telemetry.
4. Evaluate existing procedural gait, gated by animation state.
5. Blend state clips, then append their local deltas after the gait.
6. Follow the final ankle movement with sibling engines and refresh world matrices.

`node_local = configuration_matrix @ gait_delta @ state_clip_delta`.
The visual root translation is local animation only; it never moves the vehicle.
All deltas are rebuilt from the current configuration every substep. The state
controller must be prepared once and applied once per substep after clearing.
`--no-locomotion-animation` disables both visual layers.

## States and transitions

| State | Rule and visual behavior |
| --- | --- |
| AIRBORNE | Atmospheric BATTLEDROID without actual ground contact; slight knee preparation, no gait phase advance |
| LANDING | New genuine FLYING → contact event; knee compression, root lowering, torso stabilization and arm counterbalance |
| RECOVERY | After absorption, ease back to standing; duration follows impact severity |
| IDLE | Tangential speed below 0.3 m/s with no meaningful yaw; subtle looping torso/arm motion |
| WALKING | Actual tangential speed at least 0.3 m/s, movement commanded, below run entry speed |
| RUNNING | Enter above existing 4.5 m/s threshold, remain running until speed is at most 4.0 m/s |
| TURNING | Below 0.3 m/s translation and yaw faster than existing 3 degrees/s; existing alternating turn steps |
| STOPPING | Actual deceleration or uncommanded residual motion; restrained brace clip and gait settling |

Landing and recovery take priority over grounded movement. SPACE, crashed,
FIGHTER and VTOL states cannot activate this controller. The legacy LOC debug
label still describes the procedural layer; STATE describes the new controller.
Running selection uses hysteresis, while gait amplitude/cadence retain the
existing continuous 3.5–6 m/s run blend and 0.35–1.05 cycles/s range. Existing
signed forward/backward, strafe, diagonal and yaw articulation is preserved.
Commanded deceleration retains stepping underneath the stopping clip; released
commands freeze/fade the gait as before. A stopping clip finishes its settling
window even if physical speed reaches zero first.

The ground resolver increments `GroundContact.touchdown_id` only when an
unsupported vehicle in FLYING status resolves a contact, and stores
`touchdown_impact_speed` before zeroing vertical velocity. Standing contact does
not repeat the event. Existing `impact_speed`, hard-landing timer and crash rules
retain their original behavior. Animation never writes any contact field.
A crash event cannot override the physical crash.

Severity is clamped `impact_speed / 14` into 0.15–1.0. Absorption takes
`0.10 + 0.12 * severity` seconds; recovery takes `0.18 + 0.65 * severity` seconds.
The full-intensity template reaches 16 degrees extra knee bend and 0.12 m visual
root lowering. Gentle contacts scale down both offsets and timing. Clip times
are remapped to these durations; timed transitions carry elapsed-time overshoot.
A 0.12-second smoothstep crossfade blends state offsets, and a 5 units/second
envelope initializes/removes the layer during transformation. The completion
frame into BATTLEDROID exposes the exact canonical endpoint before ramping in.

## Human-editable clip schema

`game/animation_clips.py` implements `AnimationClip`, `ClipPlayer`, `JointOffset`,
`blend_pose` and `additive_pose`. Assets are project-relative, independent of cwd.

```json
{
  "name": "example",
  "license": "MIT",
  "duration": 1.0,
  "loop": false,
  "targets": {
    "LeftLowerLeg": {
      "translation": [
        {"time": 0.0, "value": [0, 0, 0]},
        {"time": 1.0, "value": [0, 0, 0]}
      ],
      "rotation": [
        {"time": 0.0, "value": [0, 0, 0, 1]},
        {"time": 1.0, "value": [0.087156, 0, 0, 0.996195]}
      ]
    }
  }
}
```

Duration must be finite and positive, loop must be boolean, and each channel
requires nonempty keys in strictly increasing time order within the duration.
Translation is a finite three-component local displacement in meters. Rotation
is a finite nonzero `[x,y,z,w]` quaternion, normalized on loading and playback.
Translation interpolation is linear; rotation uses normalized shortest-path
SLERP. Outside a channel's keys its nearest endpoint is held. Non-looping clips
clamp at duration; looping clips wrap. Loop authors should match both endpoints.
Omitted translation/rotation and missing blend targets mean identity offsets.
Named targets must resolve uniquely in the model; missing nodes fail at startup.
No scale animation or root motion is provided.

Pose blending interpolates translation and SLERPs rotation. Additive layers
multiply local transforms in order, rotating subsequent translations by preceding
rotations. They never mutate their source poses. Gameplay appends clip matrices
after gait matrices with `ModelNode.compose_animation_delta`.

Original examples in `assets/animations/`:

- `landing.json`: controlled absorption ending in a compressed stance.
- `recovery.json`: matching compressed leg/root/arm start, ending at identity.
- `idle.json`: subtle mechanical loop.
- `stopping.json`: short brace-and-settle action ending at identity.

These templates were authored for this project under its MIT license. They use
no Mixamo, downloads or proprietary animation sources. Edit times, local meters
or quaternion values, keeping modest motion and matching landing/recovery
endpoints. Retest hierarchy composition, foot separation and ground clearance.
Cadence and directional gait tuning remain in `battledroid_locomotion.py`.

## Required model pivots

`TC167Root` provides the visual datum. `TorsoCore` stabilizes the torso. Each
`Left`/`Right` side requires `UpperLeg`, `LowerLeg`, `Foot`, `UpperArm`, and `Engine`.
Hip joints remain below intake pivots; knees and ankles use the existing rigid
leg chain. Upper arms rotate at approved shoulder pivots; forearms/hands inherit
motion and retain canonical bends. Engine collars remain root siblings and
follow ankle deltas once, including root compression, with exhaust descendants
updated afterward. No imported rest matrix, scale, hierarchy or pivot is edited.
Final clip rotation uses the remaining angular budget after the gait: hips 28,
knees 38, ankles 6 and upper arms 18 degrees. This bounds additive articulation
while gait fades. There is no IK or generalized collision/constraint solver for
arbitrarily edited templates; authors must preserve pivots and clearances.

## Validation and manual testing

From the repository root:

```sh
.venv/bin/python -m unittest discover -s tests
.venv/bin/python -m unittest discover -s tests -p test_animation.py
.venv/bin/python -m tools.check_transformation_gl --locomotion --animation --vtol-ground --output /tmp/tc167_m19a
.venv/bin/python main.py --enemies 0 --battledroid-test landing
.venv/bin/python main.py --enemies 0 --battledroid-test ground
.venv/bin/python main.py --enemies 0 --battledroid-test skid
```

The hidden-window checker needs macOS window-server access. Its animation option
uses real contact physics for gentle and harder survivable impacts and requires
AIRBORNE, LANDING, RECOVERY and IDLE to be observed. It draws sampled poses and
checks GL errors and resource deletion through three upload/cleanup cycles.

Inspect landing from a side orbit: check knee compression, slight hip/root drop,
arm counterbalance and return to standing. On ground use W/S, A/D, Q/E and release
movement to inspect starts, backward/strafe steps, run transitions, turns and
stopping. Use G during movement and recovery to inspect transformation fades;
verify FIGHTER/VTOL flight and combat. Check C, 0, orbit dragging and zoom using
the existing controls. F1 SPACE should suppress ground gait; F2 returns to the
atmosphere. Compare with `--no-locomotion-animation` for the canonical stance.
The existing H debug view reports the new STATE and impact severity.

Automated coverage includes transitions, run hysteresis, genuine single-event
contact telemetry, severity/timing, clip interpolation/playback, normalized
quaternions, additive composition, no accumulation, timestep partitioning,
air/SPACE/crash gating, transformation restoration and physical invariance.
The existing enabled/disabled full-game trajectory comparison also exercises
the new layer, alongside all flight/combat/camera/model regression tests.

## Limitations and future authoring

This is rigid-node procedural animation without IK, foot locking or terrain
adaptation. Existing stride/cadence uses actual speed, but support feet may still
slide or intersect the ground slightly, especially on strafes and transitions.
Templates improve settling without solving exact sole contact. State selection
is sampled at physics substeps; different physical trajectories or threshold
crossing times can produce small pose differences across timestep sizes. Clip
sampling, phase integration and timed-state overshoot use elapsed seconds.
The ground is static and flat; moving-platform velocity is not implemented.

Future Blender work can author rigid named-node poses against the existing pivot
hierarchy and translate reference-relative samples into this JSON schema. Keep
local axes, names and rest transforms intact and inspect exported quaternions.
Blender clip export and glTF animation import are **not implemented or tested**;
the strict GLB loader still rejects embedded animation/skinning. A future export
adapter would require explicit validation. No M19B work, new jump/leap/roll,
weapons, aiming controls, skeletal skinning or external dependencies are included.

## M19A validation record

The final automated suite passed **285 tests, 0 failures**, including 12 new
animation tests. `git diff --check` passed. The combined hidden-window check with
`--locomotion --animation --vtol-ground` passed three repeated GPU lifecycles on
Apple M3 Pro / OpenGL 4.1 Metal, with no GL errors and no surviving VAO/VBO/EBO
handles. Landing/recovery, gait, transformation, reversal and VTOL paths were
rendered. Side-view hard-landing and recovered-standing PNGs were inspected.
Captures are temporary local artifacts under `/tmp/tc167_m19a_final` (and the
final landing-only rerun under `/tmp/tc167_m19a_final_landing`). This is scripted
render validation; interactive keyboard/trackpad feel still needs manual review.
