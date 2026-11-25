# M22 pilot and on-foot gameplay

## Integration plan

The player-control state has two values, VEHICLE and ON_FOOT. Exit checks the
parked vehicle and finds a supported capsule placement before changing control.
Re-entry checks the same vehicle and returns control without replacing it.
The TC167 configuration controller keeps its existing three states.

The game loop routes movement and camera input to the active controller.
A parked TC167 keeps its last pose and remains in the scene and combat
simulation. A pedestrian controller uses the existing terrain query, static
obstacle index, and evaluated vehicle hull. The pilot and hangar share the
exterior scene's meter coordinates with +Y up.

The pilot uses a separate skinned GLB loading path that shares the existing
accessor and TRS animation decoder. The rigid vehicle loader and TC167 Blender
assets stay independent. The existing open-front hangar remains in the world
environment and uses the same floor and collision surfaces as the apron.

Regression risks are input conflicts, parked-vehicle state changes, collision
at low frame rates, accidental changes to chase-camera inputs, and changes to
vehicle GLB behavior. Coverage uses the existing full suite and targeted
exit, movement, camera, and re-entry tests.

## Play

The default start parks the TC167 in the open-front mechanics hangar. Press
**Enter** to exit. Use **WASD** to move relative to the pilot's heading, either **Shift**
key to run, and **Space** to jump. Left mouse drag orbits. Scroll inward to
approach eye level and outward to return to third person. **Home** or
**Backspace** resets the on-foot view. Press **Enter** while grounded near an
unblocked side of the vehicle to board. **E** remains vehicle yaw.

The hangar opens west onto the apron. Walk past the parking markings and out
through the doorway to reach the apron, taxiway, runway, and terrain. These
are continuous world coordinates. The hangar's static walls, roof, and
cabinets use the shared obstacle index. The light strips are unlit scenery
geometry, consistent with the renderer's vertex-color shading.

## Vehicle eligibility and control routing

Exiting and re-entering require atmospheric ground contact, no active
transformation, and speed at or below the controller's configurable 0.35 m/s
default. The evaluated ground-support height must agree with the vehicle
position within 0.5 m. Crashed or destroyed vehicles are ineligible.
The player controller tests both sides and other directions around the
evaluated hull. It checks the capsule, obstacle access, terrain height, and
slope at each exit candidate. A blocked preferred side can fall back to
another location. Re-entry checks a 2.4 m reach from the pilot's eye to the
vehicle bounds and verifies the path is unobstructed.

While unoccupied, the game retains the same TC167, flight state, pilot
settings, and animation pose. It suspends flight integration and animation
updates for the parked vehicle and keeps that vehicle visible and solid.
Enemies, projectiles, and other simulation systems continue to advance.
The switch back to flight consumes the interaction frame so held pedestrian
keys cannot issue vehicle commands on that frame. Subsequent frames use
the existing flight mappings and camera settings.

## Locomotion and coordinates

All positions use world meters and +Y up. The pilot position denotes the
capsule's feet. The capsule is 1.8 m tall with a 0.30 m radius, and the
camera eye is 1.63 m above the feet. Movement runs at fixed 120 Hz simulation
steps and resolves terrain, obstacle boxes, and the parked TC167's conservative
axis-aligned hull. The controller handles a 0.32 m step, slopes up to 46°,
gravity, and a grounded jump. The camera tests a small sphere along its boom
against the same collision space. Terrain rendering follows the active viewer
over the existing terrain LOD without a separate pedestrian map or boundary.

## Pilot rig and replacement

Edit the pilot in `assets/blender/pilot.blend` and export the runtime
`assets/models/characters/pilot.glb`. The project is independent of TC167.
See the [Blender asset authoring guide](M23_2_BLENDER_ASSET_AUTHORING.md).
The bind pose uses +Y up, +Z facing forward, meters, and an origin at the
feet. See the [current humanoid rig guide](M23_3_PILOT_RIG_STANDARDIZATION.md)
for the 19-bone hierarchy, including clavicles, and the
character-space hinge conventions.

In M23.1, Seated uses a separately baked joint palette over the same mesh.
It has no write access to locomotion poses. Each disembark starts a new
locomotion session at Idle and clears all joint overrides and animation
timing. Other clip transitions reuse the existing blend against the bind
pose. First-person visibility has no effect on pose updates.

Use unique bone names and map the source hips, spine, head, and paired
limbs to those joints when retargeting. Match rest axes and limb lengths,
then bake local TRS channels and export the loops as `Idle`, `Walk`,
`Run`, and `Turn`. Keep the clips in place. The game owns root motion,
collision, and camera height. It blends visual clip changes and rotates
the pilot toward movement separately from animation.

Export one skinned mesh node with its transform applied and one skin, with
up to four influences per vertex, `JOINTS_0`, float `WEIGHTS_0`, and
the inverse bind matrices. Use triangle primitives and flat material colors.
Replace the GLB at the same path and rerun the pilot import and movement
tests. The standalone reader uses the existing GLB accessor, hierarchy,
and TRS clip infrastructure. The vehicle loader continues to handle rigid
TC167 nodes.

## Verification and limitations

Run from the repository root:

```sh
.venv/bin/python -m unittest discover -s tests -p test_player_control.py
.venv/bin/python -m unittest discover -s tests
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/test_tc167_authoring_blender.py
```

For an interactive check, exit at the hangar, walk outside, zoom to eye
level and back, then return and board. Try exiting on the runway and use
Enter after takeoff to verify rejection. An exit denial appears in the HUD.
For a failed pilot import, inspect the mesh node transform, bone names,
weight accessor, and exact clip names.

Collision uses the gameplay terrain and conservative obstacle boxes, so
some open-looking spaces beneath the aircraft are blocked. The placeholder
uses segmented skin weighting and CPU skinning with flat colors. More
detailed mesh collision, deforming character materials, foot IK, and
dedicated jump and boarding animation remain future work.
