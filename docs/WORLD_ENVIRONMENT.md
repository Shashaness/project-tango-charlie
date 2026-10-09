# M20.2 organic urban layout and airfield test environment

The default game starts TC167 at rest in FIGHTER configuration on runway 36,
in ATMOSPHERE. This is original procedural test scenery, not a city simulation.
M20.2 varies the street/block plan, reserves parks, and adds district-specific
parcels and buildings. M20.1 ground-flicker fixes are retained. Runway, reset,
physics, animation, HUD and gameplay cameras retain their M20 implementations.

## Layout and coordinates

+X is right, +Y up, and -Z forward; one unit is one meter. The established
flat-ground reference is Y=0. The city is a 1500 × 1500 m square centered at
(X,Z)=(0,-300), spanning X=-750..750 and Z=-1050..450. A fixed NumPy seed of
20 generates the authoritative `UrbanLayout` in `game/urban_layout.py`; roads,
block boundaries, parks and building parcels all derive from that plan. The
seed-20 environment has **122 blocks and 286 buildings**, including three landmarks.
There are no external architectural assets, vegetation, inhabitants, traffic,
signs or active city lights.

### Street and block algorithm

Twelve X intervals and nine Z intervals are fitted independently to the city
boundary, interleaved with road bands. Seeded widths of 60–140 m and lengths of
80–180 m are adjusted within those limits to fill the available extent exactly.
Street spacing differs along the axes. Two bands near the center of each axis
are widened into avenues; the remaining main grid uses secondary streets.

| Road class | Width | Use |
| --- | --- | --- |
| Major | 28–40 m | Long downtown avenues; preserved across block mergers |
| Secondary | 12–20 m | Varied main-grid spacing and district connections |
| Minor | 7–12 m | Local access and offset routes through subdivided blocks |

Occasional neighboring blocks merge outside downtown. The intervening road
segment is removed from the same plan, producing wider commercial/industrial
sites. Selected longer blocks are subdivided by three-segment access-road doglegs
with 12–20 m offsets. Their footprints are subtracted from block bounds before
parcels are created. Road intersections can overlap in the *plan* but are composed
into the existing disjoint ground partition, so they never stack horizontal faces.

Central-park exclusions clip block and street footprints. Short roads stop at the
park edge, keeping their nominal width; narrow leftover strips remain sandy.
Four additional open spaces reserve full district blocks. The default layout has
73 ordinary blocks, six merged blocks, 36 subdivision pieces, three park-edge
pieces and four open-space blocks. Merged, subdivided and park-edge blocks are
intentional exceptions to ordinary width/length ranges. Layout coordinates are
normalized to centimeters to keep shared cut boundaries exact. There are no
separate independently positioned road and building grids.

### Parks and plazas

All open spaces are flat and deserted, with muted dry-earth/grass color, concrete
plazas and simple 4–5 m walking paths. Buildings are excluded from their bounds.
The central park replaces the streets passing through it; paths are pedestrian
surfaces rather than vehicle roads. No plants, inhabitants or monuments are added.

Seed-20 locations (X,Z, in meters; one central park plus four smaller spaces):

| Open space | Center | Width × length |
| --- | --- | --- |
| Central dry park | (-245, -300) | 250 × 450 m |
| District park 1 | (-537.90, -800.83) | 101.76 × 129.20 m |
| District plaza 2 | (475.03, -628.72) | 77.42 × 167.03 m |
| District park 3 | (-408.89, 182.27) | 112.26 × 148.63 m |
| District plaza 4 | (475.03, 182.27) | 77.42 × 148.63 m |

Two open plazas are cut into the central park, along with perimeter and crossing
paths. Each smaller open space also has paths and a plaza. Three landmark sites
have separate small paved forecourts. Park/plaza names are metadata only; there
is no signage or environmental storytelling asset.

### Districts, parcels and vacancy

The existing asymmetric downtown-distance envelope assigns blocks to downtown
(<0.48), midtown (<0.90) and outer districts. Downtown uses more, smaller parcels
and few vacant sites; midtown mixes commercial footprints and larger buildings;
outer districts favor wide low warehouses, fewer buildings and vacant sandy lots.
Counts vary per block rather than placing four buildings everywhere.

Sidewalks inset the block by 4 m. Buildable parcels begin 7 m inside its boundary,
then buildings fit within a further 2.5 m parcel margin with district-dependent
coverage and seeded position/footprint variation. Every style stays inside its
footprint, so parcel separation protects roads, parks and neighbors. Seed 20 has
363 regular parcels, including 80 vacant ones, plus three reserved landmark sites.
Downtown averages approximately 5.01 buildings/ha and 205 m height, midtown
2.37/ha and 79 m, and outer districts 0.79/ha and 17 m. Density is measured over
buildable blocks rather than park area. Landmarks contribute to these averages.

### Desolate palette and skyline

The desert uses muted sandy brown `(0.57, 0.48, 0.36)`; lots are dry earth rather
than grass. Buildings choose from the defined `BUILDING_PALETTE`: weathered light
concrete, medium gray, dark gray, charcoal, near-black, desaturated steel and faded
industrial brown. There is no independent random RGB variation. Dark gray glass
and rooftop panels use `(0.12, 0.14, 0.15)`. One broad facade panel on taller
buildings replaces part of the wall; no coplanar wall or per-window detail sits
beneath it. Side-specific baked sun/ambient shading preserves readable contrast.

Heights follow an elliptical envelope centered at downtown `(0,-300)`, with axes
650/720 m and angular modulation to avoid a symmetrical mound. Smoothstep blends
between district bounds, then seeded variation selects each building's height.
The approximate height targets remain fringe 8–30 m, residential/industrial
20–65 m, inner city 60–160 m and core 140–320 m; transitions blend neighboring
ranges. Outer warehouses use 8–24 m heights. Landmark heights **including crowns**
remain 420, 385 and 350 m. Their positions are chosen from suitable downtown blocks
rather than fixed points that might fall on new roads or parks.

Rectangular footprints range from narrow towers to unusually wide warehouses.
Slab buildings, three-tier setback towers, two-section buildings and warehouses
share the existing palette and simple dark facade panels. Roof details vary
between flat roofs, service caps and stepped crowns. Roof caps follow the highest
section of each building. Seed 20 has 96 slabs, 74 setback buildings, 69 multi-section
buildings, 44 warehouses and three landmarks. There are no textures, reflections,
window shaders or separate per-building draw calls.

The fixed seed drives both layout and parcel/architecture variation. Repeated
construction with the same seed produces identical layout metadata and vertex
arrays; changing the seed changes the city while retaining park and road rules.
The central park stays at its declared coordinates; district spaces and landmark
sites follow the generated blocks. Layout RNG and building RNG are separate
seeded streams so architectural detail does not move the street plan.

The airfield center is (0,0,2500), 2800 m from the city center. Runway 36/18 is
2400 × 45 m, X=-22.5..22.5, Z=1300..3700. Departure is -Z (heading 000 degrees),
matching the aircraft's converted vehicle-local forward axis. The north runway
end is 850 m beyond the city's southern edge. Taxiway, three connectors, apron,
three hangars, control tower and perimeter roads sit beside the runway. The
existing origin-area test cubes retain their development-test positions inside
the city extent; they may intersect non-collidable scenery. The airfield is separate.

`game.world_environment.RUNWAY` is the shared authoritative definition used by
geometry, ground support, spawn/reset and tests. Spawn X,Z=(0,3580), 120 m inside
the departure threshold. Y is computed from the lowest transformed vertex of
TC167's canonical FIGHTER GLB, including its existing coordinate conversion:
currently approximately 0.950000025 m. No model hierarchy or animation asset is
modified. Threshold bars, centerline dashes and original block-digit 36/18 markings
are static planar geometry at Y=0, with no separate pavement collider. Airport
paving, buildings, dimensions, designation and departure heading stay in their
M20 positions; only their surface representation and colors change.

## Runway support and F4

F4 is an edge-triggered development command. It selects ATMOSPHERE/FIGHTER,
sets the canonical runway position and -Z orientation, zeros velocities,
acceleration and angular rates, sets idle throttle and forward thrust vector,
clears current controls, jump/hover, ground/landing telemetry, procedural gait and
M19 animation state, and clears transformation queue/interpolation. It restores
external CHASE and snaps the observer. It also resets defenses and missile fire
control and refreshes hostile spawns, following existing development reset behavior.
No touchdown event is synthesized. F1/F2/F3 and existing controls are retained.
F2 and R still use the existing airborne atmospheric test reset.

FIGHTER support is confined to the runway rectangle. A canonical, orientation-aware
body envelope determines clearance independently of visual animation. Unilateral
normal reaction cancels only downward net load; small load-dependent rolling/skid
resistance opposes horizontal motion without reversing it. Existing aerodynamic
lift, drag, thrust, gravity, pressure-dependent control authority and stall remain
unchanged. The pilot must gain speed and pitch up; level rolling does not produce
lift. Positive net vertical force naturally releases support. Contact uses the
existing normal-impulse resolver; vertical impact and excessive tilt can crash.
There is no landing gear, suspension, steering model or automatic takeoff.
Off-runway FIGHTER contact retains its original freeze/crash behavior. VTOL and
BATTLEDROID keep their existing flat-ground contact and locomotion implementations.

## Rendering and limitations

### Ground flicker root cause and fix

M20 stacked thin boxes across the same X/Z area: terrain, city asphalt, sidewalks,
lots, airport paving and runway markings. Some box undersides were exactly
coplanar with the terrain; other surfaces were only centimeters apart. Their
competing depths were especially visible at flight distance with the established
0.1 m near / 20 km far perspective projection. Small height offsets did not solve
this. Box face winding was outward and depth testing/writes were enabled; there
was no per-frame scenery regeneration or duplicate draw of the atmospheric grid.

M20.1 replaces all terrain and paving boxes with a **disjoint planar partition at
Y=0**, using the same elevation as flat-ground physics. Rectangle subtraction cuts
roads, sidewalks, dry lots, taxiways, apron and markings into neighboring tiles.
Each ground point has one horizontal surface instead of an overlapping base plus
an overlay. Hidden building/roof undersides are also omitted. This fixes the
competing geometry without changing camera clipping, disabling depth testing,
adding polygon offsets or introducing new shaders.

Neighboring tile edges share every boundary vertex. Splitting those edges removes
T-junctions, which otherwise leave isolated sky-colored pixels at pavement seams
after projection. Center-fan triangulation preserves upward winding and avoids
zero-area triangles. Tests check total terrain coverage, no positive-area tile
intersections, and exactly two triangles sharing every interior ground edge.
Sidewalks remain visibly distinct but are flat rather than raised curbs.

### Performance and scope

CPU scenery is generated once at world construction and uploaded once into the
existing static position/color mesh, reused in **one environment draw call** per
frame. The seed-20 scene has 1,272 ground tiles, 53,316 vertices / 17,772 triangles,
and 1,279,584 bytes (~1.22 MiB) of vertex data, approximately 27% fewer vertices
than M20.1's 72,777. It still uses one static environment mesh and draw call.
Local CPU construction measured about 0.41 s, compared with approximately 0.16 s
for M20.1; layout planning and additional surface cuts increase startup work.
No geometry generation or GPU allocation is added to the frame loop. These are
local startup measurements, not FPS benchmarks.

Existing shaders and OpenGL 3.3 Core layout are retained. ATMOSPHERE keeps readable
daylight and sky-blue background; SPACE retains its grid and dark background.
Camera projection, controls and follow behavior stay unchanged. No shadows,
clouds, weather, textures, LOD, culling, terrain deformation, per-frame scenery
allocation or new dependencies are added. Smooth gameplay has not been benchmarked.

All buildings, rooftops, hangars and towers remain **non-collidable**. Building
footprints and parcel metadata now describe the new layout, but no static-world
collision implementation exists to regenerate.
Existing physics handles a flat surface and has no swept static-world collision
interface. Building collision is explicitly deferred: a future implementation must
integrate conservative swept volumes into all vehicle substeps, define safe sliding
and aircraft impacts, and keep roof contacts separate from flat-ground landing
telemetry. No partial per-frame point clamp is introduced. Do not use rooftops as
landing surfaces. Weapons/AI retain their existing collision/terrain behavior and
can pass through scenery. Ground terrain is a finite 18 km visual square while
physics retains its infinite plane. Static scene bounds stay within a few km of
the origin, without floating-origin machinery.

## Manual testing

1. Run `.venv/bin/python main.py --enemies 0` (use the equivalent project Python
   environment elsewhere). Confirm idle runway rest, centerline alignment and skyline ahead.
2. Hold Up for throttle. Let speed reach roughly 85–100 m/s, then briefly hold S
   for a small positive pitch; release near 8–10 degrees. Confirm a force-based
   climb with runway remaining. Automated gentle-input liftoff occurs around 7 s.
3. Fly toward the skyline; inspect concrete/charcoal buildings, broad dark facade
   panels, dry lots, flat sidewalks and downtown height concentration. Inspect the
   central park west of downtown, the four district spaces, wider central avenues,
   access-road doglegs, wide warehouses and vacant outer lots. Move and
   orbit near pavement, at 450–900 m altitude, and across the runway/taxiway/apron
   junctions. Confirm there is no green/black striping, depth flicker or sky-colored
   seam pinholes. Expect ordinary distant edge aliasing without anti-aliasing.
4. Test G transformations, guns/targeting/HUD, C cockpit, 0 CHASE/DOLLY,
   mouse orbit and scroll zoom; no camera bindings change.
5. Press F4 in each configuration, during a transformation, in cockpit/DOLLY,
   after landing/crashing and in SPACE. Verify idle runway FIGHTER and CHASE,
   no old velocities, hover/jump, gait or landing/recovery state.
6. Use `--vtol-test ground`, `--vtol-test landing`, and
   `--battledroid-test ground` to check landing, throttle liftoff and walking.
7. Test F1 SPACE, F2 atmospheric airborne test flight and F3 SAS. F4 returns
   to runway operations. On keyboards mapping F keys to media controls use Fn.

## Automated and graphical validation

Run `.venv/bin/python -m unittest discover -s tests -q` for the complete suite.
`tests/test_world_environment.py` covers deterministic geometry and palette,
variable blocks, classified road widths, road/park/parcel exclusion, district
density/vacancy and height concentration, disjoint ground coverage, watertight shared
edges, winding, surface materials, blocks/setbacks,
airfield separation/dimensions, canonical spawn/heading/clearance, idle support,
F4 from all modes/environments, edge detection, physical/animation reset,
runway roll/liftoff, airborne force equivalence, SPACE, VTOL/BATTLEDROID and camera
projection. Existing tests cover the detailed camera, combat and animation behavior.

Run `.venv/bin/python -m tools.check_transformation_gl --world-environment
--animation --vtol-ground --output /tmp/m202-validation` on a machine with a window
server. This checks repeated GL uploads/draws/cleanup, transformations, M19,
VTOL operations and captures runway, skyline, street and moving views at street
level, 900 m city altitude, 650 m airfield altitude and a city-plan overhead view.
Add `--locomotion` to check
the existing gait cases. Validation asserts enabled depth testing/writes and LESS
comparison, and reports the actual framebuffer depth size. Use a normal terminal
with window-server access for graphical checks.



M20.2 validation: the full automated suite ran **300 tests in 56.012 s: 298 passed,
2 failed**. All fifteen environment/layout, runway reset, takeoff, ground contact
and physics regression tests passed. Existing failures are
`test_wings_fold_as_back_planes_without_scaling_or_hiding` and
`test_norm_quaternions_positive_scales_and_hierarchy_every_sample`, against the
TC167 GLB that was already modified in the workspace before M20.2. Both failures
reproduce independently of city generation and both tests pass when the model
path is temporarily redirected to a read-only extracted copy of the repository's
HEAD model. The workspace GLB, pose tables and failing test expectations were
not changed by M20.2; no tests were skipped or relaxed to conceal these failures.

The combined OpenGL checker passed with `--world-environment --animation
--vtol-ground --locomotion` on Apple M3 Pro / OpenGL 4.1 Metal (3.3 Core requested),
with a reported 32-bit depth attachment. Three upload/draw/cleanup cycles passed
without GL errors or surviving checked handles. Skyline, overhead city plan and
moving views were visually inspected: the central/district parks, block spacing,
avenues, mixed heights and sparse outer development are visible, with no observed
return of ground striping or pavement seam pinholes in those captures. Graphical
checks do not override the two model-specific automated failures described above.
