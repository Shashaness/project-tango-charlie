# Q1.2A interactive asset requirements

Arrival loads [objects.json](../assets/worlds/arrival/objects.json). Procedural boxes currently depict the objects. A project-relative `visual` GLB can replace an individual box while its authored state, bounds, translation, access rules and anchors remain in JSON. Export a rigid opaque GLB per moving segment. Use Blender +Z up, −Y forward, and one unit per meter. The exported GLB uses +Y up and +Z forward. Apply object scale. Put the GLB origin at the center of the corresponding authored box. For a composite, align the independent moving GLB origin to its part origin in the closed configuration.

The current runtime applies a configurable translation and derives collision from that same progress. Keep meshes in their initial position in each GLB. For future authored clips, use `Open` and `Close` and match the closed/open endpoints below. The production asset's clip playback can be added as a visual adapter while the simulation retains ownership of progress. The inspected point defaults to the nearest box surface. A fixed authored `anchor` can override it. Keep anchors on the accessible side of solid geometry.

| Placeholder | Root and moving parts | Origin, anchors and bounds |
| --- | --- | --- |
| Desk | `office_desk` tabletop with separate drawer | Center tabletop root. Retain the walk-up face and clearance around the drawer. Author static leg bounds separately. |
| Drawer | `office_drawer` single moving segment | Closed center. Q1.2A slides 1.03 m west. Keep its pull accessible and the key visible on the open tray. |
| Computer | `computer` rigid enclosure | Center root, screen toward the walk-up side. Inspect anchor at the screen. |
| Printer | `printer` rigid body | Center root. Inspect anchor facing the office aisle. |
| Key | `office_key` collectible | Center root in the tray. It inherits drawer translation and disappears after transfer to inventory. |
| Personnel door | `personnel_door` moving panel, separate fixed frame | Center of the closed 2 × 3 m panel. Lift 3.15 m. Keep the accessibility anchor fixed near the threshold. |
| Main gate | `main_gate` moving panel, separate fixed façade | Center of closed 24 × 9.1 m panel. Lift 9.2 m with collision following the panel. Maintain the full opening for a future loaded trolley. |
| Gate control | `gate_control` rigid button or panel | Center root, operating surface inside the hangar. Configure an interaction anchor facing the interior aisle. |
| Crane | `crane` fixed gantry plus hook and `crane_station` | Center girder root and separate hook origin at the attachment. Keep support bounds clear of the loading lane. A future lift clip should use a hook pivot and expose an attachment anchor. |
| Recovery trolley | `trolley` rack with separate wheel placeholders | Center chassis root. Keep the tow anchor at the forward coupling and the future aircraft attachment over the support bed. Wheel pivots should align to their axles. |

## Mechanical layout

The Jeep parks inside the gate with room to approach the tow coupling. The crane remains stationary and would lift the airframe or heavy equipment above the future loading position. The trolley would carry the load and be towed separately. JSON supplies Jeep positioning, tow connection, crane attachment, trolley position, future loading and equipment control anchors. They are provisional points with world coordinates. No tow physics or lifting simulation is included. The gate's operational power and any future crane power are provisional gameplay assumptions.
