# M25.1 Vehicle Authoring SDK

## Install and start

In Blender 5, open Preferences → Add-ons, choose **Install from Disk**, select
`tools/ptc_vehicle_sdk.py`, and enable **PTC Vehicle**. Open the 3D View sidebar
with **N** and select **PTC Vehicle**. Blender can save the add-on preference
for future sessions.

For an existing Jeep, open `assets/blender/jeep.blend` and use **Save As** to
create a derived working file. Select the original Jeep chassis in Object
Mode and choose **Migrate Legacy Jeep**. It creates an identity vehicle root,
reparents the original hubs and access anchors in place, and inserts one
spin pivot between each hub and its tire mesh. Use the existing
[legacy workflow](M23_2_BLENDER_ASSET_AUTHORING.md#jeep) for the unmigrated file.

In a blank project, enter a semantic vehicle ID and choose **Initialize
Vehicle** or assign an unparented Empty at the origin. Set reference
dimensions and mass in the panel. Add hubs, joints, and anchors with the
3D cursor or assign existing local Empties to the appropriate role. Adjust
anchor transforms with Blender's standard transform tools. To attach a
moving panel, boom, or equivalent visual, create a Joint under its
mechanical parent and use **Assign Joint Mesh**.

## Replace the body or tires

1. Enter a source path, optionally specify a Blender source object such as
   `Mesh0`, and choose **Import Source Geometry**. The add-on appends mesh data
   from a `.blend` or imports a `.glb` or `.gltf` into the active working file.
2. Choose the mesh in **Mesh**, or clear that field to use the active selected
   mesh. Align it using Blender's transforms or **Apply Geometry Corrections**.
   Translation, Euler rotation, and uniform scale operate in the mesh's parent
   frame. The optional local origin offset rebases a copy of the mesh data
   and preserves its visible placement before the other corrections.
3. Choose **Replace Body Mesh**. Choose **Assign Interior Mesh** for a separate
   interior. For a tire, select its hub and choose **Replace Wheel Mesh**.
   The mesh retains its placement, UVs, materials, and image references.
4. To restore the previous visual, select the archived object by name with the
   **Mesh** picker and run the same operation. The old visual is hidden in
   the viewport and render and excluded from the GLB. Blender Undo also
   reverses the operation.
5. Choose **Validate Vehicle**, then export a candidate GLB outside
   `assets/source`. The export uses the selected root's tagged active
   components and writes through a temporary file. Save the derived Blender
   project separately. Review a candidate before putting it at a live
   manifest path.

Wheel hubs allow direct editing of position and wheel parameters. **Show Wheel
Guides** displays the spin axis and suspension travel next to the hub center.
Adjust the spin, steering, and suspension controls and choose **Preview Wheel
Motion** to refresh the wire overlays. Steering is clamped to each wheel's
limits. **Clear Wheel Preview** removes guides and overlays. Saving clears
them automatically. The authored transforms and constraints stay at neutral.

The Jeep and crane source models are combined visual meshes. Wheel and
body geometry in a combined source must be separated in a derived working
mesh before independent motion can look correct. A body-only swap of the
unseparated Jeep source may show its modeled tires beside the rotating
runtime tires.

## Vehicle Asset Contract v1

All distances use meters. Authoring uses Blender **+Z up, −Y forward**.
Blender's Y-up glTF export maps `(x, y, z)` to `(x, z, -y)`; runtime
coordinates are **+Y up, +Z forward**, with positive X on the driver's left.
The root is a mesh-free, identity-transformed Empty. The world manifest
owns instance identity and placement. The root's `ptc_id` identifies the
reusable asset, independently of the manifest instance ID.

Every exported component has a unique, stable string `extras.ptc_id` and an
`extras.ptc_role`. Blender object names are artist labels. The root also
carries `ptc_vehicle_version: 1`, `ptc_controller` (`none` for a generic
mechanical asset or `jeep` for the legacy driving adapter), reference
`ptc_dimensions` in width/length/height order, and `ptc_mass_kg` (zero means
unspecified).

| Role | Representation |
| --- | --- |
| `root` | Stable coordinate frame and asset metadata. |
| `body`, `interior` | Replaceable leaf meshes under the root. |
| `wheel_hub` | Mechanical Empty at the authored hub center. Its ID identifies the wheel. |
| `wheel_spin` | Neutral pivot Empty beneath the hub, with ID `<wheel-id>.spin`. |
| `wheel_visual` | Replaceable leaf mesh beneath the spin pivot. |
| `joint`, `mechanical_visual` | A mechanical Empty and its replaceable visual. Joints may be nested. |
| `driver_seat`, `passenger_seat`, `entry`, `exit`, `camera`, `operator`, `attachment` | Local position and orientation under a mechanical parent. |
| `collision` | Explicit box marker. The transformed local cube spans −1 to +1 in each axis; its scale specifies half extents. |

Wheel extras are `ptc_rotation_axis`, `ptc_steering_axis`,
`ptc_suspension_axis`, `ptc_radius`, `ptc_width`, `ptc_steers`,
`ptc_steering_min`, `ptc_steering_max`, `ptc_driven`, `ptc_brakes`, and
`ptc_suspension_travel`. Axis arrays are authored local Blender vectors and
retain that basis in extras; consumers convert them using the mapping above.
Steering limits are in degrees around the positive steering axis. Suspension
travel is positive displacement along its axis. Other dimensions are in
meters. Wheel count is unrestricted by the asset contract.

Mechanical constraints remain editable in Blender. Optional rigid-node
GLB animation clips can be exported with **Include Animation Clips**.
Runtime controllers own mechanical state such as wheel spin, suspension,
steering, occupancy, and parking brake, plus the choice of whether and when
to play clips. Those values are not authored as part of a visual mesh.
The Jeep controller continues to use its existing chassis collision box and
physics constants. The migrated collider and metadata document those design
values. The Jeep adapter requires the original four wheel IDs and axle
layout and resolves its anchors and pivots by semantic ID. Generic assets
may use a different hierarchy or wheel count and require a controller to
participate in gameplay. The existing untagged Jeep GLB still binds by its
legacy names.

The Jeep shader displays embedded PNG/JPEG base color with `TEXCOORD_0`.
Normal and metallic/roughness maps may be carried in the GLB, but are not
shaded by the current renderer. Blender preserves the authored materials in
the derived project.

## Source inventory and preservation

The inspection was read-only. All original Blender sources remained closed for
writing, and all smoke-test builds were generated outside the source directory.

| Source | Blender inspection |
| --- | --- |
| `Jeep_CJ3-X3_32.blend` | One combined mesh, `Mesh_0`, 2,303,512 vertices. |
| `Jeep_CJ3-X3_32_medium_res.blend` | One combined mesh, `Mesh0`, 40,908 vertices. Used for the replacement test. |
| `Jeep_military_full.blend` | One combined mesh, `Mesh_0`, 2,499,225 vertices. |
| `Heavy_Lift_Crane_10k_adpative.blend` | One combined mesh, `geometry_0`, 10,738 vertices. |
| Aircraft recovery trolley | No source or production model found. Arrival has an interactive trolley definition. |

The three Jeep source files and the crane file have no independently
articulated object hierarchy or object constraints. The production Jeep
authoring file and its working copy each contain the chassis, four separate
wheel meshes, four hubs, and five access/occupancy anchors.

The before and after SHA-256 source inventories matched for all eight files.
The production Jeep GLB, Blender file, and artist working copy also matched
their pre-implementation hashes. These are the shared source hashes.

```text
1b496954873bf2a26f95fff2b4b13ee5938ee20469b9152a42a1e53df9fddf21  assets/source/Heavy_Lift_Crane_10k_adpative.blend
4bb04c7be2f98b865c74e07d691c134bf88a61040b64ce9b7e014eb1f74bdfb1  assets/source/Female_Fighter_Pilot_Navy.blend
4c954805201dce554df72f69a4fa67fa19a9bfe6c26d64ffb99d20fb99f335d3  assets/source/Jeep_military_full.blend
a2935e5e01730eb596a698fbe1c75883452248c68df147943d408fbdaf6fd189  assets/source/Jeep_CJ3-X3_32.blend
bfcc5170a94f0c6a426d48052ae48a54ab929047f21a83611be2b8295f25d776  assets/source/Female_Fighter_Pilot_Navy.blend1
d5cbac3537e88f0b5084bc720747401d4ecc736818dc19bea060a9d80db206e7  assets/source/Female_Fighter_Pilot_Navy_10K_adaptive.blend
d65165279105ca6773180500688df4bdc69a2c7b771752f0a46ef120b7fd8ec3  assets/source/.DS_Store
deb8954346f05a82e3fec324644b729827a86ae4eb822628a65c7fc96de2ac2a  assets/source/Jeep_CJ3-X3_32_medium_res.blend
```

## M25.1.1 authoring and export

To update an installed copy, disable **PTC Vehicle**, install the current
`tools/ptc_vehicle_sdk.py` from disk, then enable it and save your preferences.
Restart Blender if it is still using the old module. The add-on version is
1.0.1 and the asset contract remains v1. Compare the installed and repository
scripts with SHA-256 to confirm the update.

**Configured vehicle root** controls which vehicle the SDK edits. The
**Selected object** line shows Blender's active object. The root picker
lists valid authored roots. To switch vehicles from a wheel, a mesh, or
an anchor, select the descendant and choose **Use Selected Object's
Vehicle**. An untagged mesh has no owning vehicle. To start a new
vehicle, use **Initialize Vehicle** or **Assign Active Empty as Root**
with a local, unparented Empty at identity.

Select a wheel hub, wheel visual, or spin pivot to display that hub's
settings. Move the hub to adjust wheelbase or track, and adjust the
visual relative to its spin pivot when fitting a new mesh. Match the
target controller's supported layout before making a live export. Use
the mesh picker to restore an archived body or tire to its original slot.

Click a component in **Mechanical hierarchy** to reveal and select it,
including seats, access anchors, hubs, spin pivots, collision boxes,
and attachments. This explicit action may reveal its collection path.
Use Blender's ordinary Move and Rotate tools to make intentional
adjustments. Mechanical Empties carry transforms and semantic IDs
without geometry. Their position is independent of the visual mesh.

Set the GLB destination, then choose **Validate Vehicle** and **Export
Vehicle GLB**. Both check the same structure and viewport preparation
in Object Mode. The exporter temporarily opens the required collection
paths and reveals active components, then restores authoring visibility,
selection, and the active object. An object or nested collection may
remain hidden or excluded while you work. The semantic hierarchy
determines GLB inclusion. Archives and unrelated vehicles remain
outside the export. The destination is replaced only after the candidate
passes semantic ID, role, and parenting checks and Blender's state
has been restored.

If preflight identifies a component, use its object name, semantic ID,
role, and collection to find it. Link components into the current
scene, correct invalid parent or ID metadata, or use an editable local
component if a linked restriction cannot be lifted. Verify the GLB
destination is a writable file outside the source tree.
