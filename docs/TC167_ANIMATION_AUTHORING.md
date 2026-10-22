# TC167 Blender Actions and game integration — M21.4

Blender owns visual motion. Python owns controls, world position/orientation, flight,
forces, contact resolution, and playback state. The default vehicle is
`assets/models/vehicles/tc167_authoring.glb`, exported from
`assets/blender/TC167_Authoring.blend`. The pre-milestone restored file is preserved
as `backups/TC167_Authoring_pre_M21_4.blend`. Prior authoring instructions are retained
in [the M21.3 archive](TC167_M21_3_AUTHORING_ARCHIVE.md); their frozen geometry and
Python pose-candidate workflow apply only to legacy assets.

## Actions and ownership

The game plays the real exported channels, including intermediate keyframes,
per-channel interpolation, and independent Action durations:

| Purpose | Action | Current duration (seconds) |
| --- | --- | --- |
| FIGHTER configuration | TC_Pose_Fighter | static, 0.008333 |
| VTOL configuration | TC_Pose_VTOL | static, 0.008333 |
| BATTLEDROID configuration | TC_Pose_Battledroid | static, 0.008333 |
| FIGHTER → VTOL | TC_FighterToVTOL | 1.5 |
| VTOL → FIGHTER | TC_VTOLToFighter | 1.5 |
| VTOL → BATTLEDROID | TC_VTOLToBattledroid | 1.9 |
| BATTLEDROID → VTOL | TC_BattledroidToVTOL | 1.9 |
| Idle | BD_Idle | 2.0 |
| Walk | BD_Walk | 1.0 |
| Run | BD_Run | 0.75 |

Configuration Actions are sampled at their first frame. Transformations sample their
entire authored timeline. All TRS channels are absolute node-local values. Missing
transformation components use the source configuration, then imported rest; missing
locomotion components use TC_Pose_Battledroid. The full pose is rebuilt each time,
so an old Action cannot leave stale components. Locomotion uses absolute poses,
not offsets or an extra multiplication of the BD configuration. Idle/walk/run
crossfade over 0.12 seconds. Walk and run cadence follows physical horizontal speed;
airborne/space/crashed BD uses the idle Action. There are no Python joint or engine
attachment corrections in this runtime.

STEP, LINEAR (quaternion shortest-arc slerp), and glTF CUBICSPLINE are supported.
Cubic tangents retain their derivative values and are scaled by key intervals;
quaternion results are normalized. Signed and zero scale components are legal.
The loader preserves node indices, parent hierarchy, and newly exported objects.

`TC_ROOT` is a static, non-rendered model reference and must never have animation
channels; such a GLB is rejected. The simulation supplies the outer vehicle world
matrix. The existing `TC167Root` remains an artistic visual datum (including BD's
local body lift); its channels affect the model and its articulated anchors only,
never vehicle position, orientation, momentum, or world travel. In-place locomotion
never extracts or applies root motion to physics. Do not animate a locomotion travel
path: animate steps in place.

## Playback behavior

G cycles configurations; G during transformation reverses. FIGHTER ↔ BATTLEDROID
routes through VTOL. Physics mode switches when the current authored Action completes.
Time remaining in a long update carries into queued transitions.

A reversal captures the currently rendered full local pose, bridges for 0.15 seconds
to the separately authored reverse Action's first frame, then plays that entire
Action forward. It does not sample the original Action backward. This guarantees
continuity at the request instant while retaining all reverse choreography; a very
different reverse starting pose can require a visible short repositioning bridge.
Starting a transformation from locomotion uses the same bridge. Bridge duration is
separate from the Action's duration.

The controller API supports `request(mode)` (append a valid path), `reverse()`
(clear queue), `interrupt(mode)` (reverse continuously and route onward), `pause()`,
`resume()`, and `restart()` (bridge to first frame and replay). Pause freezes both
clip and bridge clocks. `reset(mode)` is an intentional immediate development reset
and leaves world physics untouched. No missing Action silently invokes legacy poses.

## Blender sidebar and configuration editing

1. Open the authoring blend in Blender 5.0.1. In a Text Editor run the embedded
   **TC167_Action_Controls.py** (Alt-P). Return to the 3D Viewport, press **N**, and
   select **TC167**. The embedded script is self-contained; it needs no game import.
2. Choose FIGHTER, VTOL, or BATTLEDROID. This assigns all required Action slots,
   disables NLA mixing, and sets the timeline. Turn Auto Keying off while editing
   configuration poses. Move/rotate objects with ordinary Blender tools.
3. Click **Save Configuration Pose**, then **Preview Endpoint Changes**. The full
   proposed changes are in Text Editor → `TC167_Last_Report.json`.
4. Choose **Sync Transformation Endpoints**, review its dialog and apply the changes.
   Only endpoint keys change. Interior keys, handles, interpolation and timing survive.
   The confirmation belongs to Blender's existing editing workflow, not game startup.
5. **Validate Configuration Consistency** checks endpoint agreement. **Validate for
   Game** checks required Actions, slots/targets, supported rigid-object channels,
   endpoint consistency, references and export requirements. Read the report for
   actionable errors. Save with Ctrl-S.
6. Choose any transformation button for immediate timeline playback/editing. Play /
   Pause, First Frame, and Reset to Configuration remain available. BD preview uses
   the BD configuration for components not keyed by the locomotion Action.

The restored original supplied for M21.4 contained identical Fighter/VTOL endpoint
poses. With user approval, TC_Pose_VTOL was restored from the preserved M21.2 blend
and 64 adjoining endpoint channel groups synchronized. Existing intermediate motion
was preserved. No new transformation motion was generated from Python pose tables.

## New articulated components and scorpion-tail walkthrough

Separate selected fuselage faces into a new object, or add a mechanical pivot and
parent the segment beneath it. Keep meaningful stable names; the tooling does not
rename objects. Use **Register New Articulated Objects** to adopt their local rest
transforms, create missing configuration/transition slots and endpoint channels,
and remove deleted decorative targets from the manifest/slots. Existing artistic
channels are preserved. Then choose a configuration again before editing and saving.
Ordinary unanimated decorative geometry can also be exported without registration.

For a scorpion-tail fuselage, create several separated segments with a nested pivot
chain. Set the straight Fighter and curved VTOL endpoint shapes, save each
configuration, preview and synchronize endpoints. Select TC_FighterToVTOL, move to
an interior frame (for example frame 91), bend the segments upward in sequence and
insert Quaternion Rotation/Location keys into that Action's corresponding object
slots. Use the Graph Editor to shape timing and interpolation. Author the reverse
Action independently. Validate, save and export. The game samples those exact
interior exported channels; no Python node-name list or pose-table edit is required.
The restored production asset is kept free of experimental scorpion-tail geometry;
the automated round-trip test creates a temporary articulated fuselage mesh and
verifies that its intermediate bend survives export and runtime evaluation.

After changing geometry, review physical support/body references and obstacle
clearance. The program cannot infer safe collision geometry from appearance.
Bake constraints/drivers to rigid object TRS before export; skeletal/skinned and
morph animation are outside this rigid-node runtime.

## Physical references

All references are non-rendered Empties. Names are case-sensitive; `<id>` is any
stable identifier and cardinality is variable:

| Name | Purpose |
| --- | --- |
| TC_ROOT | Static top-level vehicle reference |
| TC_GROUND_FIGHTER / VTOL / BATTLEDROID | Configuration ground datum |
| TC_CONTACT_FIGHTER_\<id\> | Gear contacts (starter LEFT, RIGHT, NOSE) |
| TC_CONTACT_VTOL_\<id\> | VTOL supports |
| TC_CONTACT_BATTLEDROID_\<id\> | Feet/supports |
| TC_THRUST_\<id\> | Propulsion locations |
| TC_BODY_\<MODE\>_\<id\> | Terrain/body underside clearance samples |
| TC_HULL_\<MODE\>_\<id\> | Explicit 3D obstacle-envelope corners |
| TC_MUZZLE_ / TC_HARDPOINT_\<id\> | Optional weapon references |
| TC_CAMERA / TC_COCKPIT | Optional viewing references |

**Create Missing References** seeds the original vehicle only and never moves existing
anchors. It is not a collision-design generator for redesigns. **Display / Hide**,
**Select Ground Reference**, **Select Landing Contacts**, **Select Thrust Anchors**,
and **Validate Anchors** support ordinary G/R tools. Parent contacts/thrust markers
to their articulated component. Ground datums/body/hull references may be root-relative.
If a redesign has more supports or engines, add further named references; there is
no runtime assumption of exactly two feet/engines or three wheels. Ground markers
can deliberately specify a higher support datum than the individual contacts.

Reference positions follow the evaluated GLB hierarchy and vehicle orientation.
Terrain height is sampled at each support/body point, so slope and elevation affect
contact altitude. During transformations, both modes' animated contacts and body
references form a conservative envelope; static destination ground datums do not
prematurely lift the vehicle. Contact correction moves the physical vehicle, with
no independent cosmetic model repositioning. Fighter wheel/runway queries consume
authored contacts. VTOL and BD bypass cached legacy foot vertices, Python pose
clearance interpolation and fixed BD height. Explicit hull corners replace original
fixed obstacle bounds for the authored vehicle. Underside samples, contacts, and
3D hull bounds have separate roles: ground markers alone are not collision geometry.
The existing terrain slope limits, landing impulses, friction and flight forces remain.
Legacy gear extension remains an operational physics state; this milestone does not
add a separately authored retracting-gear Action.

Starter hulls are conservative envelopes for the original model, not detailed mesh
collision or certified redesign envelopes. Review them in Blender after any geometry
change. Optional weapon/camera references are exported and queryable, but existing
weapon aiming/camera behavior is unchanged; thrust references do not retune forces.

## Export, launch, diagnostics and validation

From the repository root:

```sh
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/export_tc167_authored.py

.venv/bin/python main.py --vehicle-model assets/models/vehicles/tc167_authoring.glb \
  --animation-debug --anchor-debug
```

The requested `python main.py --vehicle-model assets/models/vehicles/tc167_authoring.glb`
also works when `python` resolves to the environment containing requirements.txt.
Plain launch uses the same authoring GLB by default. Press G and inspect the Action,
playback time/duration and progress overlay. Debug flags are disabled by default.
Contact crosses are amber, thrust blue, body/hull red, ground green. H retains
existing force diagnostics. Both local artistic motion and physical references can
be compared against the vehicle and ground.

Explicit compatibility mode:

```sh
.venv/bin/python main.py --legacy-animation
# Original procedural locomotion too, if desired:
.venv/bin/python main.py --legacy-animation --no-gltf-animation
```

The legacy default asset is `assets/models/vehicles/tc167.glb`. Older models can be
selected explicitly with `--legacy-animation --vehicle-model PATH`. No unrelated
production GLBs or older milestone blends are rewritten.

The full exporter exports actual scene geometry, hierarchy, anchors and Actions;
it does not copy animation buffers onto a frozen original GLB. `--blend-input PATH`
and `--glb-output PATH` support copies. `--install` embeds the current sidebar source
and creates missing original-vehicle anchors; ordinary export never saves the blend
or resets artist-positioned references. Invalid/missing Actions, slot targets,
unsynchronized endpoints, TC_ROOT animation, missing physical references, and invalid
hulls produce errors. Fix the reported Blender contract rather than using a pose
candidate. `tools/tc167_runtime_pose_bridge.py` is now an offline legacy tool only.

```sh
.venv/bin/python -m unittest discover -s tests
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/validate_tc167_runtime_blender.py
.venv/bin/python -m tools.check_transformation_gl --output /tmp/m21_4_render
```

The Blender test compares evaluated world matrices at start/middle/end of all ten
Actions with game evaluation, verifies an added articulated mesh and artistic
intermediate edit on temporary copies, and checks endpoint sync preserves interior
keys. Results are in `M21_4_BLENDER_VALIDATION.json`. See
[M21_4_RESULTS.md](M21_4_RESULTS.md) for final verification and limitations.
