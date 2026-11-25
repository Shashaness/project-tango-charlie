# M24 World architecture

The default world lives in [world.json](../assets/worlds/tango_charlie/world.json).
Launch it with `python main.py`. Use `python main.py --world-manifest path/to/world.json`
to load another definition. The `--terrain-*`, `--planet-*`, and `--vehicle-model`
development options can override the corresponding startup data.

## Manifest version 1

| Field | Purpose |
| --- | --- |
| `schema_version`, `world_id`, `coordinates` | Version, stable world name, and required axis, unit, and rotation conventions. |
| `assets` | Reusable GLB references and their types. Paths are relative to the project root, even if the manifest is elsewhere. |
| `entities` | Unique IDs, asset keys, optional local transforms, optional parent and named anchor, and type-specific initial state. |
| `environment` | Terrain generation and streaming settings, `legacy_blocks` town seed, runway and paving layout, procedural side hangars, and the primary authored hangar ID. |
| `player` | Startup control mode and ID of an entity of the matching type. |

The initial world uses one playable aircraft and one pilot. A manifest can
instantiate any number of compatible ground vehicles and static assets.
Available asset types are `aircraft`, `character`, `ground_vehicle`, `structure`,
and `prop`. Aircraft start grounded in atmosphere and accept `FIGHTER`, `VTOL`,
or `BATTLEDROID`. Ground vehicles take a Boolean `parking_brake`. The player can
start in the aircraft, on foot, or in a registered ground vehicle. The default
starts in the parked Fighter. Terrain support determines the final Y of
grounded actors using each actor's existing contact or ride-height calculation.

Two actual entries from the default world illustrate where placement comes from.

```json
[
  {"id": "hangar_01", "asset": "hangar", "transform": {"position": [300, 0, 3000]}},
  {"id": "hangar-jeep", "asset": "jeep", "parent": "hangar_01",
   "anchor": "vehicle_parking", "initial_state": {"parking_brake": true}}
]
```

The Jeep anchor is at hangar-local `[-2, 0, -14]` and faces west, so the
Jeep starts at world X/Z `298, 2986`. The TC167 parking anchor is at
`[10, 0, 0]`, also facing west. The workshop anchor yields world
`[340, 0, 3038]`. Edit an anchor in the hangar source to move everything
attached to it. Add a local transform to move just one child.

## Transforms and startup

- Environment glTFs and manifest coordinates are right-handed, in meters,
  with +Y up. Actor placements and anchor direction use local +Z forward.
  The aircraft keeps its own existing flight, model, and animation axes.
- Blender uses Z up. Blender `(X, Y, Z)` exports to glTF `(X, Z, -Y)`.
  Leave the exporter’s Y-up option enabled. A Blender origin at floor level
  makes an environment asset easy to place.
- Local transforms default to identity. Rotation is fixed-axis Euler
  `[X, Y, Z]` in degrees, applied X then Y then Z to the local vector.
  Column-vector composition is `T × Rz × Ry × Rx × S`. Scale must be positive.
- A child uses `parent_world × anchor_asset_local × child_local`. Anchors are
  GLB empties named `Anchor.<name>`. A child with a parent and no anchor
  uses the parent's origin. Gameplay actor placements resolve upright and
  at unit scale.
- `game/world_manifest.py` parses and validates JSON, checks file paths,
  checks role/type consistency, and orders parents before children.
  `game/world_loader.py` resolves positions, rotations and anchors and provides
  a read-only placement mapping keyed by ID. Missing assets, duplicate IDs,
  invalid transforms, unknown parents or anchors, and cycles report errors
  before gameplay starts.
- World construction uses those placements to build scenery and the terrain
  protection mask, then builds the shared ground and obstacle index. `World`
  and `Game` instantiate gameplay actors and select the player controller.
  `World.live_entities` maps stable IDs to the live aircraft, pilot and vehicles.
  It is separate from the authored placements. Driving or flying mutates
  the live actors. Constructing another game restores the defined starts.
- A future save can associate world ID and schema version with mutable state
  keyed by entity ID and apply that state after initialization. M24 defines no
  save format.

## Environmental Blender workflow

The authoritative sources are
[hangar.blend](../assets/blender/environment/hangar.blend) and
[repair_station.blend](../assets/blender/environment/repair_station.blend).
The hangar exports `Structure`, `Floor`, `Collision`, `Anchors`, and
`Details`. The workbench exports `Equipment`, `Collision`, and `Anchors`.
The exporter selects supported asset objects in these collections, including
children. Keep cameras, lights, reference images, and staging objects outside
the exported collections. Each exportable object's parent must also export.
The two side hangars stay procedural. The primary authored hangar retains its
90 × 110 m footprint, 24 m height, and west-facing opening.

- Ordinary meshes contribute static triangles with their material base colors.
  Use a Principled BSDF base color for export. The runtime uses solid colors.
- `Collision.<name>` empties describe solid unit cubes spanning local −1 to +1.
  Scale the empty to half the intended dimensions. The runtime transforms
  each box into its enclosing world AABB. Use separate boxes for walls and
  overhead structures to keep the entrance open.
- `Surface.Floor` is a horizontal rectangular mesh at world Y=0 with axis-aligned
  edges. The runtime inserts it into the shared pavement partition and provides
  flat support through the normal ground queries. `Surface.Marking.<name>`
  rectangles overlay that partition. The shared pavement palette sets their
  visual colors. Omit any collision box for the floor and keep one authored
  floor per structure.
- Use `Anchor.<name>` empties for children and interaction points. The repair
  station has `Anchor.interaction` and `Anchor.pilot_standing`. Leave capsule
  clearance around the standing anchor. The space can be approached and
  walked around. Repair interactions are reserved for M24.1.

From the repository root with Blender 5.0.1 on PATH:

```sh
blender -b --python-exit-code 1 --python tools/export_ground_assets.py -- hangar
blender -b --python-exit-code 1 --python tools/export_ground_assets.py -- repair_station
blender -b --python-exit-code 1 --python tools/export_ground_assets.py -- hangar --check
blender -b --python-exit-code 1 --python tools/export_ground_assets.py -- repair_station --check
blender -b --python-exit-code 1 --python tools/export_ground_assets.py -- hangar --output /tmp/hangar.glb
python -m tools.validate_environment_assets hangar
python -m tools.validate_environment_assets repair_station
```

Environmental exports are validated with the same loader used at runtime,
then reimported into a clean Blender scene. The round trip checks required
node types and preserves anchor and collision transforms. `--check` compares
fresh bytes with the selected output file without replacing it. Existing
pilot, Jeep, and TC167 export commands still apply to those assets.

## Adding instances

Register another building or prop GLB with type `structure` or `prop`, then
add a uniquely named entity with its own transform. Follow the same GLB
marker contract. If you add a new independently authored source to the
shared Blender exporter, register its source collections, runtime output,
and required markers in `tools/validate_environment_assets.py`. Version 1
supports flat axis-aligned walkable floors at the shared ground elevation
and conservative AABB collision. Put static placements within the legacy
world bounds. A future elevated interior would require ground-system work.

For another Jeep, reuse `"asset": "jeep"` with its own entity ID, transform
or spawn anchor, and parking-brake state. Each instance has separate
suspension, position and occupancy. Rendering shares the corresponding GLB
resources. Update `player.controlled_entity` and `player.starting_mode` if
that Jeep should be occupied at launch.

## Regression checks

Run `python -m unittest discover -s tests` for schema, deterministic scene,
collision, terrain, occupancy and flight regressions. Run
`tools/test_tc167_authoring_blender.py` and `tools/test_pilot_rig_blender.py`
inside Blender for authoring regression coverage, plus `--check` on all
four assets supported by the shared exporter. The existing airplane
authoring suite accepts the `TC167_TEST_BLEND` environment variable for
verification against a separate authoring file.
