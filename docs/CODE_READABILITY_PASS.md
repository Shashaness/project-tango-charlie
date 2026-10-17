# Python readability pass — 2026-10-12

This pass changes documentation, comments, whitespace, literal layout, and import
organization. It preserves public signatures, keyboard mappings, tuning constants,
algorithms, gameplay state transitions, rendering calls, and asset paths. Existing
working-tree edits and untracked ground/airfield modules were included in the
baseline; they were not reverted. No model, shader, terrain asset, or runtime
requirements file was edited by this pass.

## Scope and review groups

The baseline contains 100 Python files. This pass modifies 97: 17 in `engine`,
47 in `game`, 28 tests, three tools, one script, and `main.py`. The three unchanged
files already satisfy the applicable formatting/module-documentation requirements.

Changes were applied and verified in three groups:

1. Engine and entry point: control integration, observation cameras, model hierarchy,
   GLB validation, HUD, and GPU resource ownership.
2. Gameplay: force/contact units, configuration transitions, procedural animation,
   AI decisions, weapons, seeker state, and collision handling.
3. Terrain, geographic sources, scenery, developer tools, and test formatting.

Modules retain responsibility docstrings. Classes and nontrivial production
functions now describe their contracts with Google-style sections where applicable.
Added comments explain midpoint integration, quaternion interpolation, swept hull
slab tests, spherical terrain noise, void-weight interpolation, LOD skirts, and
animation layering. Existing names are retained even where a naming lint remains.

`pyproject.toml` sets a 100-character limit, four-space formatting, and E/F/I checks.
Only import checks are automatically fixable. `requirements-dev.txt` pins Ruff
0.17.0 without adding a game dependency. `CONTRIBUTING.md` now documents setup,
units, checks, baseline comparison, review workflow, and asset conventions.

## Verification

Before editing, `python -m unittest discover -s tests` ran **389 tests**:
**387 passed and two failed**. Full runs after each review group reproduced the
same two failures and no errors or additional failures.

| Run | Tests | Passed | Existing failures |
| --- | ---: | ---: | ---: |
| Baseline | 389 | 387 | 2 |
| Engine group | 389 | 387 | 2 |
| Gameplay group | 389 | 387 | 2 |
| Terrain/tools/tests group | 389 | 387 | 2 |
| Final | 389 | 387 | 2 |

The final suite completed in 102.892 seconds using the existing project virtual
environment (Python 3.14).

Final checks:

- `python -m ruff format --check .`: passes.
- `python -m ruff check --select E501,I .`: passes.
- `python -m compileall -q engine game main.py scripts tools tests`: passes.
- `git diff --check`: passes.
- Executable syntax-tree comparison against the pre-pass workspace: passes after
  removing docstrings and normalizing import groups. Function signatures, names,
  constants, expressions, branches, calls, and runtime strings are unchanged.
- Import organization changes were exercised by the full suite. Syntax equivalence
  does not independently prove arbitrary import-side-effect equivalence.
- The full `python -m ruff check .` still reports **35 existing findings**, exactly
  matching the pre-pass nonformatting diagnostic identities and counts.

Interactive gameplay and the display-dependent OpenGL smoke tool were not run.
The checks establish headless regression equivalence, not visual acceptance on a
particular graphics driver. No generated assets were rewritten for verification.

## Existing test defects — unchanged

- `test_pose_refinement.PoseRefinementTests.test_wings_fold_as_back_planes_without_scaling_or_hiding`:
  maximum absolute wing X extent is **4.887896060943603 m**, exceeding the asserted
  **2.5 m** bound.
- `test_transformation.TransformationTests.test_norm_quaternions_positive_scales_and_hierarchy_every_sample`:
  the assertion that every sampled pose scale component is positive fails.

These failures predate the pass. Their assertions, pose data, and geometry were
preserved; fixing them requires a separate behavior/asset change.

## Existing static findings — unchanged

| Rule | Count | Reason left unresolved |
| --- | ---: | --- |
| F401 | 23 | Removing unused imports can alter exposed names or import side effects. |
| E741 | 5 | Ambiguous local names require identifier edits rather than formatting. |
| F405 | 4 | Wildcard-import name resolution requires changing the test import contract. |
| F403 | 1 | Replacing the wildcard import is outside this pass. |
| E731 | 1 | Replacing the terrain priority lambda changes executable syntax. |
| F841 | 1 | Removing an unused assignment may change its evaluation or side effects. |

- `engine/terrain_renderer.py`: E731: Do not assign a `lambda` expression, use a `def`.
- `game/battledroid_locomotion.py`: E741: Ambiguous variable name: `l`; E741: Ambiguous variable name: `l`.
- `game/countermeasure.py`: F401: `numpy` imported but unused.
- `game/ground_environment.py`: F401: `game.battledroid_physics.BATTLEDROID` imported but unused.
- `game/planetary_elevation.py`: F401: `game.geography.GeographicTile` imported but unused.
- `game/prototype_model.py`: F401: `numpy` imported but unused.
- `game/world_environment.py`: F401: `game.urban_layout.CITY_CENTER` imported but unused; F401: `game.urban_layout.CITY_SIZE` imported but unused.
- `tests/test_ai.py`: F401: `game.atmospheric_physics.reset_test_flight` imported but unused.
- `tests/test_airfield_operations.py`: F401: `game.world_environment.RUNWAY` imported but unused.
- `tests/test_defenses.py`: F401: `dataclasses.replace` imported but unused; F401: `game.ai_config.AI` imported but unused.
- `tests/test_flight_computer.py`: F401: `engine.game.Game` imported but unused; F401: `game.flight_state.VehicleMode` imported but unused.
- `tests/test_geotiff.py`: F401: `dataclasses.replace` imported but unused; F401: `game.elevation.ElevationTile` imported but unused; E741: Ambiguous variable name: `l`.
- `tests/test_ground_operations.py`: F401: `engine.flight_controller.FlightController` imported but unused.
- `tests/test_locomotion.py`: F403: `from game.battledroid_locomotion import *` used; unable to detect undefined names; F405: `RUN_MAX_CADENCE` may be undefined, or defined from star imports; F405: `RUN_KNEE_FLEX` may be undefined, or defined from star imports; F405: `ANKLE_AMPLITUDE` may be undefined, or defined from star imports; F405: `RUN_HIP_SWING` may be undefined, or defined from star imports.
- `tests/test_models.py`: F401: `json` imported but unused.
- `tests/test_planetary_world.py`: F401: `game.geography.GeographicPosition` imported but unused; F841: Local variable `roots` is assigned to but never used.
- `tests/test_terrain.py`: F401: `game.world_environment.TERRAIN_BOUNDS` imported but unused; E741: Ambiguous variable name: `l`.
- `tests/test_terrain_compatibility.py`: E741: Ambiguous variable name: `l`.
- `tests/test_transformation.py`: F401: `test_vehicle.commands` imported but unused; F401: `engine.flight_controller.FlightController` imported but unused; F401: `game.flight_state.Environment` imported but unused; F401: `game.tc167_poses.PHYSICS_SWITCH` imported but unused.
- `tests/test_vtol.py`: F401: `game.flight_state.FlightStatus` imported but unused.

## Modified Python files

### engine

- `engine/asset_paths.py`
- `engine/battledroid_controller.py`
- `engine/camera.py`
- `engine/camera_input.py`
- `engine/flight_controller.py`
- `engine/game.py`
- `engine/gltf_loader.py`
- `engine/hud.py`
- `engine/input.py`
- `engine/mesh.py`
- `engine/model.py`
- `engine/quaternion.py`
- `engine/renderer.py`
- `engine/shader.py`
- `engine/terrain_renderer.py`
- `engine/transform.py`
- `engine/vtol_ground.py`

### game

- `game/ai_config.py`
- `game/ai_pilot.py`
- `game/airfield_operations.py`
- `game/animation_clips.py`
- `game/atmospheric_physics.py`
- `game/battledroid_animation.py`
- `game/battledroid_locomotion.py`
- `game/battledroid_physics.py`
- `game/countermeasure.py`
- `game/defensive_avionics.py`
- `game/elevation.py`
- `game/enemy_aircraft.py`
- `game/fighter_ground.py`
- `game/fire_control_radar.py`
- `game/fire_control_solution.py`
- `game/flight_control_computer.py`
- `game/flight_instruments.py`
- `game/flight_state.py`
- `game/geographic_registry.py`
- `game/geography.py`
- `game/ground_contact.py`
- `game/ground_environment.py`
- `game/gun.py`
- `game/hgt.py`
- `game/identity.py`
- `game/missile.py`
- `game/missile_fire_control.py`
- `game/missile_guidance.py`
- `game/missile_seeker.py`
- `game/model_test.py`
- `game/planetary_elevation.py`
- `game/player_vehicle.py`
- `game/projectile.py`
- `game/prototype_model.py`
- `game/target.py`
- `game/target_manager.py`
- `game/tc167_poses.py`
- `game/terrain.py`
- `game/terrain_compatibility.py`
- `game/transformation.py`
- `game/transformation_test.py`
- `game/urban_layout.py`
- `game/vtol_ground.py`
- `game/vtol_physics.py`
- `game/weapons.py`
- `game/world.py`
- `game/world_environment.py`

### tests

- `tests/test_ai.py`
- `tests/test_airfield_operations.py`
- `tests/test_animation.py`
- `tests/test_arm_pose.py`
- `tests/test_atmosphere.py`
- `tests/test_battledroid.py`
- `tests/test_camera_orbit.py`
- `tests/test_combat.py`
- `tests/test_combat_energy.py`
- `tests/test_defenses.py`
- `tests/test_flight_computer.py`
- `tests/test_geotiff.py`
- `tests/test_ground_operations.py`
- `tests/test_hud.py`
- `tests/test_leg_extension.py`
- `tests/test_locomotion.py`
- `tests/test_missiles.py`
- `tests/test_models.py`
- `tests/test_planetary_world.py`
- `tests/test_pose_refinement.py`
- `tests/test_prototype.py`
- `tests/test_terrain.py`
- `tests/test_terrain_compatibility.py`
- `tests/test_transformation.py`
- `tests/test_vehicle.py`
- `tests/test_vtol.py`
- `tests/test_vtol_ground.py`
- `tests/test_world_environment.py`

### scripts

- `scripts/generate_hierarchy_test.py`

### tools

- `tools/check_transformation_gl.py`
- `tools/generate_tc167_prototype.py`
- `tools/preview_tc167_prototype.py`

### Entry point

- `main.py`

## Supporting files

- `pyproject.toml` (new Ruff configuration).
- `requirements-dev.txt` (new development requirements).
- `CONTRIBUTING.md` (updated existing contributor guide).
- `docs/CODE_READABILITY_PASS.md` (this report).
