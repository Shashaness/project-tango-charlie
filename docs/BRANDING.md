# Branding refactor verification

Project title: Project Tango Charlie. Current unit: TC167. Organization: Tango
Charlie. Canonical modes: FIGHTER, VTOL, BATTLEDROID. Enum ordering, numeric
parameters, controls and transformation timings remain unchanged. This is a
clean identifier break; no old development-mode aliases or saved-state migration
were added. No persisted save format was found.

The selected custom asset is assets/models/vehicles/tc167.glb; the reproducible
generated reference is tc167_prototype.glb. Both use TC167Root. Generic mechanical
nodes retain their names. The hierarchy fixture's generator metadata was updated.
All non-JSON GLB chunks are byte-identical, and all non-string JSON fields match
before/after, including transforms, indices, accessors, hierarchy and materials.
No geometry was regenerated.

Renamed module families: vtol_physics, vtol_ground, battledroid_physics,
battledroid_controller, battledroid_locomotion, tc167_poses and their tests/tools.
Documentation and endpoint image paths use the same canonical names.

273 automated tests pass. Existing combat/AI/avionics, camera, transformation,
M17 locomotion and M18 landing tests remain unchanged in behavior. Hidden-window
OpenGL validation exercises transformations, VTOL contact, locomotion, orbit and
three resource-cleanup cycles. Native interactive feel is not claimed as tested.

The terminology audit covers prior project/configuration names, franchise names,
VF model identifiers and explicit aircraft-inspiration references. Additional
checks include character, organization and ship terminology. No obsolete branding
remains in maintained source, text or GLB JSON metadata. The float32 dtype '<f4'
is an intentional numeric format, not an aircraft reference. Git history, the
user's checkout directory and third-party environment files are outside this
current-tree content refactor and remain untouched. Checkout paths in examples
are generic; code resolves assets relative to the project.

No license was added and no legal/trademark clearance is asserted. Before
publication, review the licenses/distribution notices for NumPy, PyOpenGL and
GLFW (including its native library), and the provenance of the selected custom
model. This pass establishes names, not an asset-rights certification.

## Modified content (current paths)

- `README.md`
- `assets/models/test/hierarchy_test.glb`
- `assets/models/vehicles/tc167.glb`
- `assets/models/vehicles/tc167_prototype.glb`
- `docs/LOCOMOTION.md`
- `docs/MODELS.md`
- `docs/PROTOTYPE.md`
- `docs/TRANSFORMATION.md`
- `docs/VTOL_GROUND.md`
- `docs/premise.md`
- `engine/battledroid_controller.py`
- `engine/camera.py`
- `engine/flight_controller.py`
- `engine/game.py`
- `engine/hud.py`
- `engine/renderer.py`
- `engine/vtol_ground.py`
- `game/atmospheric_physics.py`
- `game/battledroid_locomotion.py`
- `game/battledroid_physics.py`
- `game/flight_control_computer.py`
- `game/flight_state.py`
- `game/ground_contact.py`
- `game/identity.py`
- `game/player_vehicle.py`
- `game/prototype_model.py`
- `game/tc167_poses.py`
- `game/transformation.py`
- `game/transformation_test.py`
- `game/vtol_ground.py`
- `game/vtol_physics.py`
- `main.py`
- `scripts/generate_hierarchy_test.py`
- `tests/test_arm_pose.py`
- `tests/test_battledroid.py`
- `tests/test_camera_orbit.py`
- `tests/test_combat.py`
- `tests/test_flight_computer.py`
- `tests/test_hud.py`
- `tests/test_leg_extension.py`
- `tests/test_locomotion.py`
- `tests/test_pose_refinement.py`
- `tests/test_prototype.py`
- `tests/test_transformation.py`
- `tests/test_vtol.py`
- `tests/test_vtol_ground.py`
- `tools/check_transformation_gl.py`
- `tools/generate_tc167_prototype.py`
- `tools/preview_tc167_prototype.py`

## Renamed files (current names)

- `engine/battledroid_controller.py`
- `engine/vtol_ground.py`
- `game/battledroid_locomotion.py`
- `game/battledroid_physics.py`
- `game/vtol_physics.py`
- `game/vtol_ground.py`
- `game/tc167_poses.py`
- `tests/test_battledroid.py`
- `tests/test_vtol.py`
- `tests/test_vtol_ground.py`
- `tools/generate_tc167_prototype.py`
- `tools/preview_tc167_prototype.py`
- `docs/VTOL_GROUND.md`
- `docs/transformation_vtol.png`
- `docs/transformation_battledroid.png`
- `assets/models/vehicles/tc167_prototype.glb`
- `assets/models/vehicles/tc167.glb`
