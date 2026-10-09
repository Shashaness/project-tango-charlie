# M20.5 planetary terrain streaming foundation

M20.5 replaces the fixed 32 km rendering boundary with a camera-following forest
of geographically anchored terrain patches. TC167, flight and contact equations,
transformation, combat, existing city and airport remain in their original meter
frame. The local files stay in `hgt/`; no dataset is moved or downloaded.

## Planet and addresses

The current planet is Earth. `TerrainConfig` configures geographic origin,
source elevation offset, registry directory, legacy raster directory, planetary
seed (default 205), real/procedural blend width (750 m), and existing compatibility
mask parameters. Other planets, curvature and spherical physics are future work.

`GeographicTile.at(latitude, longitude)` derives a canonical one-degree southwest
corner with **floor**, including negative coordinates. For example, latitude
34.8 / longitude -110.1 gives `N34W111`; -0.1 / 36.2 gives `S01E036`.
Longitudes wrap into [-180,180), so +180 and -180 identify the same western edge.
Valid latitudes are [-90,90]; +90 is assigned to the northernmost N89 band and
-90 to S90. Pole addresses retain a longitudinal wedge, while procedural heights
at a pole are longitude-independent. Canonical names reject S00/W000 aliases.

These are geographic cells, not fixed-size meter tiles. East-west physical width
changes with latitude. HGT/GeoTIFF metadata supplies actual elevation coverage.
Longitude-aligned metadata sampling and neighbor-edge checks join dateline tiles.
The existing local projection still requires an origin within ±80° latitude;
polar addresses/noise work, but polar flight needs a later projection/frame design.

## Asset registry

Default layout:

```text
assets/planets/earth/tiles/
  N34W112/
    manifest.json          # optional
    elevation.tif          # optional local raster
    city.json              # optional future content
    airport.json
    objects.json
hgt/                       # existing local legacy rasters, left in place
```

A version-1 manifest uses lists of relative file paths:

```json
{
  "version": 1,
  "elevation": ["elevation.tif"],
  "cities": ["city.json"],
  "airports": ["airport.json"],
  "objects": ["objects.json"]
}
```

Without an elevation list, directly contained HGT/TIF/TIFF files are discovered.
All extension matching is case-insensitive. Paths must remain inside the cell;
missing files, malformed manifests, noncanonical directory IDs and unsupported
raster formats produce diagnostics. Manifests are limited to 1 MiB. Registry
ordering is deterministic. Registry elevation paths precede legacy `hgt/` paths,
with duplicate physical paths removed. Raster bounds remain authoritative even
when a filename differs from its geographic cell. Optional city/airport/object
paths are recorded only: M20.5 does not load models, instantiate objects, or rebuild
cities while streaming. Existing fictional assets have stable geographic anchors
in `World.geographic_anchors` and retain their original single environment batch.

```sh
.venv/bin/python main.py --atmosphere \
  --planet-tiles assets/planets/earth/tiles --terrain-dir hgt \
  --planet-seed 205 --terrain-patch-span 32000 --terrain-view-distance 16000
```

No network data retrieval exists; the registry README contains no dataset.

## Unified elevation and continuity

Source priority is valid real elevation, procedural relief where absent/void,
then approved local city/airport modifications. `ElevationDataset` retains strict
HGT and supported WGS84 GeoTIFF validation. `PlanetaryElevation` blends that data
with `ProceduralElevation`, and `Terrain.height_at` applies the unchanged M20.4
compatibility weight and permanent vertical offset. `normal_at` derives normals
from the same field. Queries do not depend on visible meshes or residency in a
terrain patch cache. Existing contact remains globally at Y=0; terrain collision
is still deferred and no visible LOD is treated as collision authority.

Procedural relief evaluates seeded three-dimensional value noise on the unit
sphere direction derived from latitude/longitude. Eight hashed lattice corners
are interpolated with quintic weights. Four frequencies (64,128,256,512) use
amplitudes 900,450,200,80 m over a 1200 m base, plus 350 m ridged relief. There are
no independently seeded tile boundaries. The same position always gives the same
value, including antimeridian and pole limits. This is invented mountainous/desert
relief, not a reconstruction of actual missing topography or an Earth biome map.

The 750 m exterior-data blend fades real influence to the procedural field,
rather than to flat zero. Valid connected geographic edges retain real influence.
For voids, each loaded raster lazily computes a bounded confidence grid (at most
257² nodes). A coarse block containing any invalid sample sets all its confidence
corners to zero. Bilinear confidence with quintic smoothing feathers surrounding
real data toward procedural relief and covers the entire void footprint. Nearby
rasters' confidence halos are combined across edges, including adjacent cells;
this conservative treatment may replace some valid samples near voids. Confidence
storage is at most about 0.26 MiB per mapped source, additional to raster bytes.

Expected file/decoder failures are recorded and disable that source; missing or
unsupported sources use procedural relief. Programming exceptions are not caught
by the provider or terrain worker result handling. A RuntimeError in a generator
propagates, rather than being disguised as missing geographic data.

## Streaming forest and caches

`TerrainConfig.side` now defines one **coarse root span**, not a world boundary.
It defaults to 32 km. Integer root indices extend across the regional meter plane;
children retain stable `(level,x_index,z_index)` addresses. Patch geographic
identity includes Earth, the permanent origin and span, containing geographic cell,
and hierarchical key. Camera movement never reanchors or renumbers cached roots.
The root span must cover the view diameter (default 16 km viewing radius), keeping
visible coarse roots within the minimum four-patch budget. Near/medium/far detail
uses the existing distance/altitude hierarchy and split hysteresis. Previously
split nodes are preferred while they remain eligible, avoiding budget reshuffles
from small movements. Frustum tests use conservative global elevation bounds.

The camera drives visible selection. TC167 position and velocity drive speculative
prefetch, default four seconds ahead. Lead is limited to one root span, with roots
near the predicted point prepared before finer patches. This does not generate
new city, airport, enemy or combat assets.

One CPU worker prepares real/procedural meshes and source diagnostics. At most
eight mesh jobs plus one diagnostic job are queued/running. At most two completed
normal meshes upload per update. A CPU LRU retains 32 meshes with a 32 MiB byte cap;
GPU LRU retains at most 96 meshes. Raster residency remains four mappings and
128 MiB of raster bytes, with 64 MiB per-source acceptance. Active terrain retains
the 64-patch budget. Completed future arrays add at most eight bounded mesh buffers
to the CPU cache limit; GL driver overhead and OS file cache are not included.

At initialization, the origin root is prepared and retained for F4. Newly reached
roots receive an immediate tiny procedural fallback (nine top vertices, skirts,
flat inexpensive lighting). This bounded safety path performs no raster mapping,
decoding, confidence scan or detailed normal generation on the render thread.
It is used only on missing roots; ordinary movement usually reaches prefetched
roots. Detailed meshes, raster decoding and diagnostics run on the CPU worker.
Fallbacks are approximations and may visibly change when real/detail meshes arrive.

Every visible root has coverage immediately. A parent is replaced only when the
required descendants are resident; parents and descendants are never drawn
together. Skirts cover LOD/fallback edge differences. Protected city/airport ground
is excluded exactly as in M20.4. Patch and source eviction cannot remove current
root coverage. The pinned origin mesh counts against the GPU cap. Cache pressure
can retain coarser coverage than requested. Cleanup cancels pending jobs, joins the
worker, closes all GL resources on the context thread, clears CPU caches, and
closes source mappings.

## Coordinates, precision, and phased rebase migration

`GeographicPosition` stores latitude, wrapped longitude and source-reference
height. TC167 exposes `geographic_position` through its permanent `GeographicFrame`.
The frame converts its existing float64 local position without changing position,
velocity, attitude, gravity, radar or missile state. F4 consequently restores both
the original local runway position and its geographic address. The fixed city and
airport anchors never move with streaming windows.

The regional WGS84 linearization remains unchanged. This **does not make local
flat physics geodesically accurate over planetary distances**. The terrain forest
has no fixed 32 km boundary and tests travel across several one-degree cells, but
regional projection distortion grows with travel. Treat roughly 100 km from the
initial origin as the point to plan a coordinate migration, not a certified
accuracy radius. Pole crossings/global circumnavigation are not supported.

Terrain vertices are float32 offsets from each patch center. The render thread
subtracts the camera in float64 before setting the terrain model translation and
uses a rotation-only terrain view. It restores the ordinary view afterwards.
This improves terrain precision without rebasing physics or other renderables.
Other models/combat rendering still use their existing coordinate handling.

`GeographicFrame.rebase_plan` returns a proposed frame and shift without mutating
any entity. A full rebase requires architectural review and these phases:

1. Define canonical geographic/velocity state for player, enemies, missiles,
   countermeasures, fixed assets and replay/test fixtures.
2. Separate simulation frames and render origins, including absolute source-query
   coordinates and permanent city/airport anchors.
3. Atomically migrate cameras, previous positions, radar/seeker calculations,
   target histories and combat integration to the new frame; test identical
   trajectories and collision results around the rebase instant.
4. Introduce geodesic/tangent-frame travel and later curvature/gravity changes
   under a separate physics milestone. Choose a polar-safe projection explicitly.

M20.5 performs none of those coordinated rebases or spherical physics changes.

## Diagnostics, validation and known limits

Existing H debug shows geographic coordinates/cell, loaded raster count, active
patch count/triangles, LOD distribution, CPU/GPU mesh usage, raster bytes in tool
stats, source SRTM/BLEND/PROCEDURAL, data-warning count and queue depth. These terrain
coordinates follow the camera (the streaming observer); TC167's permanent position
is separately available as `player_vehicle.geographic_position`. The normal HUD
shows a true-heading compass tape at top center, with TC167 latitude and longitude
below it, green inside available HGT or
GeoTIFF map coverage and orange outside it over procedural regions. Color uses
metadata coverage, including real/procedural blend borders; it does not decode
rasters or classify individual void pixels on the render thread.
Camera AGL/source are sampled asynchronously; initial/stale AGL uses a
procedural estimate marked EST and source PENDING until the worker catches up.

```sh
.venv/bin/python -m unittest discover -s tests -q
.venv/bin/python -m tools.check_transformation_gl --planetary-streaming \
  --terrain-dir hgt --world-environment --vtol-ground --locomotion --animation \
  --output /tmp/m205-local
```

Synthetic tests need no downloaded elevation data. They cover addressing,
registry manifests/legacy sources, conversion, global deterministic relief,
dateline/poles, data/void blending, delayed root coverage, queue/cache bounds,
velocity prefetch, geographic travel, precision, city/runway preservation and F4.
The GL checker captures immediate and warmed remote regions, returns to F4, and
repeats existing flight/ground/model/resource checks three times.

Known limits: regional projection and flat collision; no curvature or global
physics rebasing; conservative void confidence may hide nearby valid detail;
coarse fallback/detail popping; skirts visible from below; conservative culling;
metadata registry indexing at startup; direct height queries may synchronously
load sources and must be scheduled appropriately by future physics integration;
source files are not hot-reloaded. No vegetation, traffic, NPC spawning, automatic
asset instantiation, network downloading, or new combat/flight features.


## M20.5 validation record

The Apple M3 Pro OpenGL 4.1 Core check passed three complete resource cycles,
remote travel at 45 km, 166 km, 260 km and 165 km from the origin, immediate and
warmed coverage, real and procedural sources, F4 return, VTOL/locomotion,
transformation/animation, and all VAO/VBO/EBO deletion checks without GL errors.
Captures are in `/tmp/m205-local`. Immediate remote coverage, warmed procedural
relief and the compact city/airport plan were visually inspected.

Remote warmed views used 63–64 active patches / 145,152–148,144 triangles,
32 CPU meshes / approximately 1.83–1.87 MiB, and 96 GPU meshes /
approximately 6.05–6.92 MiB of array payload. Infrastructure views reached
178,198 triangles / approximately 9.61 MiB GPU payload due to retained clipping
and transition lattice geometry. Four loaded local rasters occupied 98.93 MiB,
below the 128 MiB cap. These are array payloads, not process or GL-driver totals.

Ten no-data mesh preparations measured median immediate root fallback at
0.679 ms (nine top vertices, 25 total including skirts, 24 triangles, 888 bytes),
and an ordinary procedural patch at 6.953 ms (1,345 vertices, 2,304 triangles,
59,928 bytes). Full meshes run on the worker. These local CPU observations are
not frame-rate guarantees. Initial origin preparation is synchronous, and newly
reached roots perform only the bounded coarse safety generation on the main thread.

Created files: `game/geography.py`, `game/geographic_registry.py`,
`game/planetary_elevation.py`, `tests/test_planetary_world.py`,
`assets/planets/earth/tiles/README.md`, and this document. Modified terrain/source
code: `game/elevation.py`, `game/terrain.py`, `engine/terrain_renderer.py`.
Integration: `engine/game.py`, `engine/renderer.py`, `engine/hud.py`,
`game/player_vehicle.py`, `game/world.py`, `main.py`. Validation/docs:
`tools/check_transformation_gl.py`, `tests/test_terrain.py`,
`tests/test_geotiff.py`, `tests/test_terrain_compatibility.py`,
`docs/TERRAIN_LOD.md`, and `README.md`.

Recommended next milestone: review and implement an atomic floating-origin
migration across all coordinate consumers using the staged plan above. Keep
terrain-aware contact as a separate reviewed physics change. No new gameplay or
rebasing implementation is included in M20.5.


Final full suite: **349 tests in 62.752 seconds**, **347 passed** and the same two
pre-existing imported-model failures remained: folded-wing lateral extent
(`test_pose_refinement`, 4.887896 m versus a 2.5 m limit) and negative imported
node scale (`test_transformation`). All 14 new planetary tests passed. Existing
flight, ground, combat, city/runway and terrain regressions introduced no new
failures. The model hierarchy and its tests were not altered to conceal these.


## HUD compass

The normal HUD compass remains visible in FIGHTER, VTOL and BATTLEDROID, using
TC167's forward orientation projected onto the geographic frame's true east/north
basis. Velocity and camera rotation do not determine heading. The current regional
frame uses east +X and north -Z; translation-only origin changes preserve that
basis. A later rotated-frame rebase must supply its transformed horizontal basis.

A fixed center chevron marks heading; the tape spans ±50°, has 10° minor ticks,
30° three-digit labels, and N/E/S/W at cardinal headings. A degree-marked heading
above the tape rounds to the nearest degree modulo 360. Unwrapped tick offsets
make travel through 359°/000° continuous. Direct orientation sampling adds no
smoothing latency. A strictly vertical nose has no horizontal heading and displays
`---°` with the pointer. Existing flight/combat indicators remain, and the colored
latitude/longitude readout sits below the compass. All drawing uses the existing
static glyph and triangle HUD pipeline with its framebuffer scaling.


Compass validation: all 12 HUD tests passed, including cardinal headings in every
mode, wraparound tick motion, camera independence, translation rebasing, colored
coordinates and undefined vertical heading. The complete suite ran 354 tests in
68.597 seconds: 352 passed, with the same two pre-existing imported-model failures
listed above. The synthetic-terrain OpenGL checker passed three complete cycles
without GL errors or leaked handles; `/tmp/compass-gl/VTOL.png` was visually
inspected for compass, coordinates and existing instrument separation.
