# TC167 Blender Actions and game integration — M21.5.1

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

Open `assets/blender/TC167_Authoring.blend` in Blender 5.0.1. In the Text Editor,
select the embedded **TC167_Action_Controls.py** and run it with **Alt-P**. In the
3D Viewport press **N** and open **TC167**. The embedded script is self-contained.

A configuration is one saved local pose. Its Action owns location, quaternion
rotation, and scale for each articulated object at the Action's first frame.
Parenting and the parent-inverse matrix are shared across configurations and
recorded in the manifest. The three configuration buttons open a detached
preview. **Edit Pose** assigns the selected configuration Action and each object's
own slot. Auto Key starts off. Use Object Mode transforms and **Save Configuration
Pose** to commit the result to that Action after confirmation. The save checks
all three poses before and after writing. Missing channels display the manifest's
local reference components, and validation flags incomplete canonical poses.

A transformation is a separate timed Action. Select a transformation to preview
it, then choose **Edit Animation** to bind its slots and key choreography with
Blender's animation tools. The sidebar displays the session, configuration,
and active object's assigned Action. It flags pending Object Mode transforms when
it can detect them and confirms before a sidebar switch discards them. Insert or
save pending transforms before scrubbing. Blender evaluates the Action at the new
frame when you scrub, and manual Graph Editor or keyframe edits apply immediately.
Keep Auto Key off when editing a configuration. Direct changes through Blender's
animation editors can bypass the sidebar's editing workflow.

**Preview Endpoint Changes**, **Validate Configuration**, and **Validate All
Endpoints** write reports to Text Editor → `TC167_Last_Report.json`. Configuration
validation compares each object's local TRS, parent and world matrix across all
three saved poses. An empty list of differences for a part means it has the same
pose for that comparison. **Sync Transformation Endpoints** uses the saved
canonical pose, previews the changes, and requires confirmation before writing.

| Configuration | First frame | Last frame |
| --- | --- | --- |
| FIGHTER | TC_FighterToVTOL | TC_VTOLToFighter |
| VTOL | TC_VTOLToFighter, TC_VTOLToBattledroid | TC_FighterToVTOL, TC_BattledroidToVTOL |
| BATTLEDROID | TC_BattledroidToVTOL | TC_VTOLToBattledroid |

Sync preserves existing opposite endpoint keys and interior key coordinates,
handles, interpolation types, and timing. The segment adjoining an adjusted
endpoint evaluates differently. A missing key is inserted with linear interpolation.
For an entirely missing curve, the preview identifies the added component and the
repair anchors the opposite frame to its prior reference value. If the original
opposite key was lost, inspect that endpoint and explicitly sync its own
configuration when appropriate. Undo/redo works for saves, syncs, and initialization.
Save the Blender file with **Ctrl-S** to keep these edits across sessions.

## Creating a scorpion-tail VTOL configuration

1. Select **FIGHTER** in the sidebar. If the fuselage mesh shares a datablock
   and the split should affect only one instance, make that mesh single-user
   before editing. Select faces in Edit Mode and use **P → Selection** to split
   each section. Return to Object Mode.
2. Set the intended straight Fighter layout. Set any required pivots and
   parent chain before initialization. **Detect New Objects** reports candidates
   and **Show Unregistered Objects** selects them. Use **Register Selected
   Objects** for a subset or **Initialize New Parts Across Configurations** for
   all candidates, then confirm the initial layout. Include any new parent
   parts in the same operation.
3. Verify the initial FIGHTER pose by selecting it again. Initialization records
   each part's local transform and parent, adds missing slots and TRS keys to
   the three configuration Actions, and seeds both ends of the four
   transformation Actions. Existing channels and interior keys are preserved.
   Initialization from VTOL or BATTLEDROID explicitly asks whether to copy that
   current layout to every configuration. Use FIGHTER for a straight starting pose.
4. Select **VTOL**, click **Edit Pose**, then rotate and position the tail sections
   in Object Mode. Click **Save Configuration Pose** and confirm the overwrite.
5. Click **Preview Endpoint Changes** and inspect the report. Click **Sync
   Transformation Endpoints**, review the four VTOL endpoint changes and confirm.
6. Click **Validate Configuration** and inspect the FIGHTER/VTOL and
   BATTLEDROID/VTOL differences. Use the configuration buttons to verify the
   straight FIGHTER and BATTLEDROID tail and raised VTOL tail. Click **Validate
   All Endpoints** to check the four transformation Actions.
7. Select a transformation and click **Edit Animation** to refine its
   intermediate keys. For example, add a mid-clip bend in `TC_FighterToVTOL`,
   then use the Graph Editor to shape the timing. Repeat for the reverse Action.
8. Review anchors and contacts after changing geometry. Run **Validate for Game**,
   save the blend, and choose **Export GLB**. Export to the runtime path below
   or point the game at your test GLB. Launch the game and press **G** to inspect
   each configuration and its transformations.

Use Object Mode for articulated movement. Edit Mode changes the mesh geometry
across all configurations. Separate creates a new object that needs its own
slots. **Validate Configuration** reports geometry changes, linked mesh data,
shape keys, and modifiers. Any configuration-specific mesh shape requires explicit
morph support or a different supported asset design. The current rigid GLB export
omits morphs and the runtime plays rigid object TRS. Geometry modifiers can affect
the export and should be reviewed.

### Migration and failure reproduction

The old sidebar came from `TC167_Action_Controls.py`. Its switch only reset
reference transforms when an object lacked an Action slot. An object with a slot
and missing components could keep values from the preceding configuration.
Unregistered separated objects were absent from the manifest entirely, and validation
missed them when they had no active Action. In Blender 5.0.1 a new mesh child under
`FuselageSurface1` with a 1-radian Y rotation kept that rotation through Fighter,
VTOL, and Battledroid. Registering it in VTOL then keyed the same rotation into
all three. The stock file had separate canonical Actions and node-specific slots.
Shared slot assignment is checked when saving. Shared geometry is an additional
source of all-configuration visual changes and is reported independently.

The embedded manifest is now version 2 and identifies registered objects through
persistent IDs. Its historical `TC167Root` parent record was updated to match the
already-installed M21.4 `TC_ROOT` hierarchy. **Validate Parent Hierarchy** reports
renamed, replaced, missing and reparented registered objects. It preserves their
slots and Actions for review. Restore the recorded name/hierarchy to resolve an
accidental change. Intentional hierarchy migrations require manual review of the
affected rest transforms, slots, and choreography. Legacy inconsistent data is
reported before pose/endpoint writes. The migration changes metadata and sidebar
code, without rewriting existing artist keys, meshes, pivots, or anchors.

### Troubleshooting and release checks

- **The sidebar is missing or running the Text only writes a JSON report.** Verify
  that `TC167_Action_Controls.py` contains the code from
  `tools/tc167_authoring_controls.py`. Restore that source to the named Text
  datablock on a working copy, run it, then reopen the saved copy and run it
  again. Use a separate Text datablock or an external script to write reports.
- **The tail looks right during transformations but shifts when BD locomotion
  starts.** Inspect each separated part’s stashed NLA tracks. Blender can copy
  the source object's `BD_Idle`, `BD_Walk`, and `BD_Run` bindings and create
  suffixed copies of `TC_Pose_Fighter`. Identify copied strips by their slot
  target and archive or unlink the inherited bindings on the new objects.
  Preserve the source object's bindings and any intentional new animation.
  The game uses the saved BATTLEDROID pose for a part with no locomotion channels.
  Re-export and compare the three gaits as well as all configuration and
  transformation clips with Blender.
- **The report flags shared geometry or a changed fuselage mesh.** Shared
  mesh warnings are expected for the current asset. Inspect datablock users
  and make the mesh single-user before editing an individual instance.
  The changed `FuselageSurface1` geometry is expected after separation.
- **The report requests a physical-reference review.** Examine the
  `TC_HULL_*` obstacle bounds and `TC_BODY_*` underside points after
  geometry changes. Check the complete transition as well as the endpoints.
  Move contact and ground references only when the intended support points
  change, then repeat the flat-ground, slope, landing, and taxi checks.

Use **Validate for Game** and the Blender/runtime round-trip checks together.
Endpoint validation checks the three poses against transformation endpoints.
The runtime comparison also exercises the exported locomotion and hierarchy.
Use a fresh export from the saved blend and require exactly the ten clips
listed above. A deliberate fuselage split increases the primitive count.
Release geometry checks should verify that the registered segments are present
and parented correctly, that their combined FIGHTER geometry preserves the
intended outline, and that untouched meshes retain their original geometry.

### M21.5.1 asset audit

The release blend embeds the repository sidebar source again. The six tail
sections retain their geometry, pivots, hierarchy, and saved Actions. Copied
source-slot bindings were unlinked on those sections, including the three
Battledroid gait bindings and the six suffixed Fighter Actions. The source
object's bindings and all ten intended clips were preserved. Initialization
also removes source-slot bindings inherited by newly separated parts.

Only the four rear `TC_HULL_*` corners in each of FIGHTER and VTOL moved,
from Blender Y = 10.50 to 11.25. The fuselage reaches 11.207 in FIGHTER,
11.105 in VTOL, and 11.229 during the sampled transformations. This restores
rear obstacle coverage with about 0.021 units of clearance at the tightest
sample. The front, lateral, and vertical hull bounds were preserved, along
with the BATTLEDROID hull, all underside points, the three ground datums,
and the seven contacts. The FIGHTER and VTOL fuselage undersides remain
about 0.082 units above the body reference plane.

Ground and body reference transforms matched the pre-repair export, and
flat and sloped terrain support matched with the vehicle level and tilted.
A rearward sweep intercepted an obstacle at the corrected hull without
triggering a crash at taxi speed. All 484 sampled transition poses were
contained, with terrain support points unchanged. Existing game landing,
taxi, obstacle, and transformation regressions exercise the released asset.

The release assets passed all 448 Python regression tests and all 12 Blender
5.0.1 integration tests. A new Blender
process reopened the blend, registered the embedded sidebar, selected all
three configurations, verified edit slots, and reported zero endpoint errors.
A fresh export matched the release GLB byte for byte. The ten-clip runtime
comparison agreed with Blender within about 1.6 × 10⁻⁶ in world-matrix elements.

The remaining Blender messages are 32 shared mesh notices, one expected
`FuselageSurface1` geometry change, and the reminder to review physical
references after future geometry edits. Ruff reports the same 37 existing
findings as the pre-milestone source.

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
anchors. It is not a collision-design generator for redesigns. **Show / Hide Anchors**,
**Select Ground References**, **Select Contact Points**, **Select Thrust Anchors**,
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
and initializes identity metadata and missing original-vehicle anchors. Ordinary
export never saves the blend. The sidebar export uses the same Blender export settings
and restores the editing session. The command-line exporter also opens the GLB with
the game loader before publishing it. Invalid/missing Actions, slot targets,
unsynchronized endpoints, TC_ROOT animation, missing physical references, and invalid
hulls produce errors. Fix the reported Blender contract rather than using a pose
candidate. `tools/tc167_runtime_pose_bridge.py` is now an offline legacy tool only.

```sh
.venv/bin/python -m unittest discover -s tests
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/test_tc167_authoring_blender.py
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/validate_tc167_runtime_blender.py
.venv/bin/python -m tools.check_transformation_gl --output /tmp/m21_4_render
```

The authoring suite uses Blender's real mesh separation operator, then saves a
curved three-segment VTOL pose and verifies configuration isolation, all four
VTOL endpoints, opposite endpoints, interior keys, repeat sync, undo/redo, and
reopen. The temporary export is loaded by the game and reimported into Blender.
The M21.4 runtime validation compares evaluated world matrices at start/middle/end
of all ten Actions and checks an independently authored intermediate edit. See
[M21_4_RESULTS.md](M21_4_RESULTS.md) for the original runtime verification.
