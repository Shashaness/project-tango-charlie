# M20.3 — SRTM terrain LOD foundation

The terrain system adds geographically anchored desert terrain around the existing
city and airport. It preserves their geometry, runway heading, F4 reset, flight
physics, VTOL ground support, and Battledroid contact. No external dataset is
redistributed. Missing or malformed elevation files leave the existing environment usable.

## Data installation and format

The default directory is project-root **`hgt/`**. Discover `.hgt`, `.tif`, and
`.tiff` files there, case-insensitively, or select another local directory with
`--terrain-dir`. The older `assets/terrain/hgt/` location remains usable by passing
that path explicitly. Nothing downloads terrain data. The local `hgt/` directory
is ignored by Git; check the provider's attribution and redistribution terms
before distributing any dataset.

`game/elevation.py` supplies a common `ElevationDataset` for both formats.
Metadata indexing records geographic coverage and source characteristics without
reading whole image arrays. Direct raster inspection raises descriptive errors
for unsupported files; directory loading records them in `dataset.warnings`,
skips those files, and the renderer prints those warnings at initialization.

`game/hgt.py` accepts exactly 1201 × 1201 (2,884,802 bytes) and 3601 × 3601
(25,934,402 bytes) samples, stored as big-endian **signed** int16. A filename such
as `N34W112.hgt` identifies the southwest corner of a one-degree tile. The first
row is its north edge; columns run west to east. Negative heights are valid.
Invalid names, coordinates, and byte counts are rejected by `HgtTile`; directory
loading records these failures in `dataset.warnings` and skips those files.

Bilinear sampling ignores -32768 void corners and renormalizes the remaining
weights. An entirely void footprint returns NaN. Exact sample coordinates are
snapped within numerical roundoff so an exact void does not acquire spurious
neighbor weights. Dataset borders use the first available tile in sorted filename
order, including its shared edge. Missing/void game heights fall back to zero.
The elevation cache holds at most four rasters, subject to a separate 128 MiB
uncompressed-byte budget, under a sampling lock. Each raster is limited to 64 MiB;
oversized rasters are rejected before mapping or decoding. Limits can be configured
through `ElevationDataset(capacity=..., cache_bytes=..., max_tile_bytes=...)`.
Uncompressed TIFFs and HGT files are memory-mapped. Supported DEFLATE TIFFs decode
with one worker into temporary disk-backed mappings; eviction/close releases
mappings. Range scans use 128-row chunks and only intersecting tiles. The byte
budget bounds mapped raster size, not total process RSS: decoder working buffers,
OS file cache, GPU resources, and mesh work also consume memory.

SRTM geographic coordinates use WGS84, while elevations are meters above the
EGM96 geoid. Source elevation is **not** an ellipsoidal height. See the
[USGS/NASA SRTM user guide](https://lpdaac.usgs.gov/documents/179/SRTM_User_Guide_V3.pdf).
No geoid-to-ellipsoid conversion is performed here.

## GeoTIFF metadata and supported CRS

The lightweight dependency **`tifffile>=2024.8.30`** is listed in
`requirements.txt`. It reads TIFF metadata, preserves numeric sample values and
byte order, and supports direct memory mapping without GDAL, rasterio, or pyproj.
Install/update the existing environment with:

```sh
.venv/bin/python -m pip install -r requirements.txt
```

See the [tifffile project](https://github.com/cgohlke/tifffile) and the
[OGC GeoTIFF specification](https://docs.ogc.org/is/19-008r4/19-008r4.html).

Supported GeoTIFFs contain one numeric elevation band and one image, with
north-up **EPSG:4326 / WGS84 geographic coordinates in degrees**. Pixel scale and
tiepoint tags define bounds and spacing; nonzero raster tiepoint coordinates are
handled. An axis-aligned ModelTransformation matrix is also accepted. Filenames
are arbitrary and never determine coverage. Projected CRSs, other geographic
CRSs, rotated/sheared transforms, inconsistent georeferencing, multiband images,
and unsupported vertical references are rejected explicitly. No reprojection is
performed. Missing vertical metadata retains the user-supplied SRTM assumption of
EGM96 elevations in meters; explicitly declared alternatives are rejected.

Both RasterPixelIsPoint and RasterPixelIsArea are supported. Point rasters place
the first sample at the tiepoint-derived origin and span `(width-1) × pixel_step`.
Area rasters describe outer pixel edges and sample at half-pixel centers, spanning
`width × pixel_step`. The reader handles north-to-south rows, distinct X/Y pixel
sizes, and integer or floating-point elevations. GDAL_NODATA tag values, NaNs, and
infinities are excluded from interpolation. Unlike HGT, a GeoTIFF's declared
NoData marker is authoritative; -32768 remains valid when a different marker is
specified. Valid corners are renormalized, and entirely invalid queries return
NaN. Uncompressed and DEFLATE compression are supported; other codecs and
floating-point prediction are rejected rather than adding imagecodecs.

Point tiles share boundary samples. Area-tile boundary interpolation obtains
neighboring center samples, including four-tile corners, before retrieving the
current mapping; this remains safe even with a one-tile cache. Exterior missing
neighbors use available corner weights and the existing fade to flat ground.
Shared and partially shared geographic edges are detected from metadata rather
than an integer filename grid. Internal connected edges do not fade to zero.
Overlapping rasters use the first valid result in sorted filename order.

Raster tiles supply one continuous queried height field to the existing quadtree;
there is no mesh per GeoTIFF. Each patch's world-space footprint is generated once,
so adjacent source tiles cannot create duplicated boundary surfaces. Existing
patch skirts still handle changes in mesh LOD.

## Geographic configuration

The nine installed local tiles are automatically discovered. Their metadata shows
3601² signed int16 point rasters, EPSG:4326, 1/3600° spacing, -32767 NoData, and
uncompressed storage. Together they cover longitude [-112,-109] and latitude
[33,36]. The geographic origin is unchanged at 34.5, -111.5; loading additional
tiles does not enlarge the existing 32 km rendered domain or move the city.

Start using the local data with the automatically sampled vertical offset:

```sh
.venv/bin/python main.py --atmosphere
```

Example using an explicit directory and offset:

```sh
.venv/bin/python main.py --atmosphere --terrain-dir /path/to/hgt \
  --terrain-origin 34.5 -111.5 --terrain-offset 1000 \
  --terrain-budget 64 --terrain-view-distance 16000
```

`TerrainConfig` also exposes domain side length, maximum subdivision level, cells
per patch, cache size, blend width, and hysteresis. Defaults are a **32 × 32 km**
domain centered at world X=Z=0, 16 km viewing distance, 64 active patches,
96 cached GPU patches, five subdivision levels, and 32 cells per patch side.
The side length is restricted to 22–64 km, viewing distance to 1–20 km, and origins
to ±80° latitude. Camera controls and its 20 km far clip remain unchanged.

`LocalProjection` linearizes the WGS84 ellipsoid at the origin. With meridional
radius M and prime-vertical radius N, X = N cos(latitude) Δlongitude and
Z = -M Δlatitude, using angular differences in radians. Longitude differences
wrap at the dateline. +X is east, +Y is up, -Z is north; one unit is one meter.
Longitude scale varies with origin latitude. This is a documented local
approximation, not a large-area geodesic projection. World vertices contain local
meter coordinates, never Earth-centered float32 values.

The vertical offset subtracts a source EGM96 elevation from every sampled height.
When omitted, the origin's bilinear source elevation becomes world zero; an
unavailable origin uses offset zero. The city and airport always remain at their
existing world elevation zero, independent of this choice.

## Compatibility and queries

The whole existing ground rectangle, X=[-9000,9000], Z=[-8000,10000], stays flat.
Keeping this 18 × 18 km area avoids rebuilding existing city/runway meshes.
Terrain patches insert the rectangle's exact edges into their sampling grids and
omit every top triangle inside it. There is no second coplanar ground underneath
the city or airport. Outside it, a 1500 m smoothstep blends source heights from
zero. Unavailable exterior tile borders additionally fade to flat over 750 m.

```python
terrain = game.world.terrain
height = terrain.height_at(x, z)
normal = terrain.normal_at(x, z)  # normalized, upward world-space vector
source = terrain.dataset.sample(latitude, longitude)  # EGM96 meters / NaN
```

Queries sample the source field and compatibility blend directly, independently
of visibility, patch cache, and rendered LOD. Normals use centered 10 m finite
differences on the same field. Missing data returns flat world ground.

**Terrain-aware collision is deferred.** Existing contact remains at Y=0 globally,
including outside the compatibility rectangle. Aircraft can intersect rendered
hills and Battledroids can walk below them. A later milestone must wire the query
API into contact, slope handling, and continuous collision without depending on
render meshes. This milestone changes no contact or flight equations.

## Patches, selection, and rendering

Each patch is addressed by `(level, x_index, z_index)` in a quadtree. Its bounds
and children are computed without building meshes. The root spans the domain;
default finest patches span 1 km with approximately 31.25 m sample spacing.
Exact compatibility edges can add rows/columns. Coarse source resampling can
alias small features; LOD does not change source-query fidelity.

Selection refines nearby patches first using distance relative to patch size and
altitude, subject to the active patch budget. Existing splits have a 15% wider
exit threshold than new splits' entry threshold. XZ distance bounds viewing
range; conservative source-height AABBs are tested against view-frustum planes.
Terrain entirely inside the flat compatibility zone is excluded.

Indexed static meshes contain position and RGB only, matching the existing
OpenGL 3.3 Core shader. Elevation and slope blend sandy brown, weathered tan, and
muted rock gray, blending back to the existing desert color at the flat boundary. Directional illumination is baked into colors using source-field
normals; there is no new lighting shader, vegetation, water, or material system.
Skirts descend below patch outer edges to cover neighboring LOD differences.
Their conservative depth is the domain's source elevation range plus 64 m.
The range is scanned in bounded row chunks at startup, only for intersecting
tiles. Skirts do not descend inside the compatibility zone.

One CPU worker prepares meshes. At most eight tasks are pending and at most two
meshes upload per rendered frame; the render thread creates/deletes all GL objects.
The root mesh is prepared at initialization and pinned as a fallback. Descendants
replace a parent only when the required coverage is resident, so coarse parents
and their descendants are never drawn together. GPU LRU eviction and the desired
working set are both bounded by the cache size. Ancestors count toward this limit;
small caches may retain coarser fallback coverage. Close cancels queued jobs, joins
the worker, deletes every cached VAO/VBO/EBO, and closes mapped source files.

A full ordinary patch has about 2304 triangles including skirts. The 64-patch
budget is roughly 147,456 triangles and at most 64 terrain draw calls, separate
from the existing single city/environment draw. Compatibility clipping often
reduces this. Cache limits bound GPU memory; the default is roughly 5–6 MiB for
96 ordinary patches. These are estimates, not frame-rate guarantees. Startup
range scans and root generation are synchronous; subdivision mesh generation is
not performed in the frame loop. Source-query cache misses can map a file on the
calling thread. Shutdown waits for the currently running bounded mesh task.

## Diagnostics and validation

Press existing **H** in atmosphere mode to show LOD range, active patch count,
approximate rendered triangles, camera altitude above the queried terrain, and
currently mapped elevation tile count (`TILES`, either format). No control binding was added.

```sh
.venv/bin/python -m unittest discover -s tests -q
.venv/bin/python -m tools.check_transformation_gl --terrain-test \
  --world-environment --vtol-ground --locomotion --animation --output /tmp/m203-gl
```

Additional GeoTIFF checks:

```sh
.venv/bin/python -m tools.check_transformation_gl --terrain-test \
  --terrain-test-format geotiff --world-environment --vtol-ground \
  --locomotion --animation --output /tmp/m203-geotiff-synthetic
.venv/bin/python -m tools.check_transformation_gl --terrain-dir hgt \
  --world-environment --vtol-ground --locomotion --animation \
  --output /tmp/m203-geotiff-local
```

Tests synthesize standard-sized HGT files and small GeoTIFF fixtures locally.
They do not require the nine user-supplied files. GeoTIFF cases cover metadata,
byte order, negative/void/NaN elevations, area/point sampling, both tile-edge
orientations, four-tile corners, mixed formats, unsupported CRS/transforms,
DEFLATE decoding, count/byte limits, patch continuity and flat city/runway ground.
The GL tool creates a temporary
1201² hill field and repeats upload, drawing, and resource cleanup three times;
its near, compatibility-boundary, and far captures require no dataset download.
Inspect those captures for continuous seams and flat runway/city ground. For
manual play, install a tile, start in atmosphere, use F4 to verify runway reset,
and fly east past X=9000 or south past Z=10000 to reach the terrain transition.
Use H and compare low/near flight with high-altitude views. Collision outside the
flat zone remains subject to the limitation above.

Known limits: local projection approximation; fixed finite domain; no automatic
streaming beyond that domain; linear height interpolation; no geomorphing (some
LOD popping remains); skirts can show as vertical walls from below; conservative
culling; data void fallback may create local depressions; startup range scanning;
no terrain collision. The amendment adds only tifffile as a dependency; no gameplay
or model changes. GeoTIFF limitations additionally include the supported CRS,
transform, band, compression, vertical-reference and per-raster size restrictions
above. GeoTIFF GDAL band scale/offset metadata is not applied; install elevation
rasters containing actual meter values, as these USGS SRTM files do.

## GeoTIFF amendment validation

All 12 neighboring borders in the nine local 3601² USGS tiles were inspected;
all 3601 valid shared samples on every border matched exactly (maximum height
difference zero). Metadata indexing accepted all nine with no warnings. Cycling
queries through all nine retained four mappings totaling 103,737,608 bytes
(98.9 MiB), below the 128 MiB byte limit. The default rendered domain needs one
25,934,402-byte mapping (24.7 MiB). The nine rasters remain local and unmodified.

Apple M3 Pro, OpenGL 4.1 Metal with a requested 3.3 Core context: synthetic
GeoTIFF and local-data validation each passed three complete upload/draw/cleanup
cycles, including world, VTOL, locomotion, animation, transformation, and resource
checks. Local terrain near view: 34 patches / 72,688 triangles; boundary view:
13 / 26,048; far view: 8 / 15,696. Maximum observed GPU patch residency was 87,
below the unchanged 96-patch limit. Captures are in `/tmp/m203-geotiff-local`
and `/tmp/m203-geotiff-synthetic`.

A warmed local benchmark measured about 31 ms terrain startup, 10 ms preparation
of one fine patch, and 0.4 ms per scalar height query. These are machine- and
cache-dependent CPU observations, not frame-rate guarantees. Mesh preparation
continues on the existing worker; no per-frame raster-wide scan was added.

Automated tests: 12 synthetic GeoTIFF tests and all 15 existing terrain tests
passed. Full suite: 327 tests in 60.879 s, 325 passed, with the same two existing
GLB assertions failing (folded-wing bounds and positive imported scales).
No physics or model assertions were relaxed. Compile checks and
`git diff --check` passed.

Amendment files created: `game/elevation.py`, `tests/test_geotiff.py`.
Files modified: `game/terrain.py`, `engine/terrain_renderer.py`, `engine/hud.py`,
`main.py`, `requirements.txt`, `tools/check_transformation_gl.py`, `README.md`,
`assets/terrain/hgt/README.md`, and `docs/TERRAIN_LOD.md`.
The existing HGT loader, flight/contact code, city/runway geometry and TC167
model were preserved. No commit, push, tag, dataset download, terrain collision
integration, or new gameplay feature was performed.
