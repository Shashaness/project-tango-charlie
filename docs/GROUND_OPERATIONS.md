# Ground operations and hangar lifecycle

This refinement supersedes M20.6's runway-only support and automatic off-runway
crashes. It retains gear/flaps/brakes, the existing force integrator, transformations,
VTOL and robot controls, cameras, compass, source-colored geographic HUD, and
terrain streaming. The request's acceptance-list Q/W reference was interpreted as
an older typo: the requested ground steering controls are **Q/E**.

## Controls and startup

Normal startup, including `--atmosphere`, is atmospheric FIGHTER inside the middle
hangar, stationary, gear DOWN, flaps UP and brakes released. Spawn is (310,3,3000)
in the current flat airfield frame, heading 270° west toward the open entrance.
The spawn height is sampled ground elevation plus operational wheel clearance.
F4 remains the existing north-facing runway reset. F2/R fighter recovery still
starts the airborne test flight. No mandatory key was added.

| Controls | Grounded FIGHTER with deployed gear | Airborne / other modes |
| --- | --- | --- |
| Q / E | Left / right nose-wheel steering | Existing yaw controls |
| W / S | Existing nose-down / nose-up pitch | Unchanged |
| Up / Down | Existing throttle increase / decrease | Unchanged |
| 6 held | Loaded wheel brakes on traversable surfaces | No airborne wheel braking |
| 7 | Flap toggle | Atmospheric FIGHTER only, unchanged |
| 8 | Timed gear toggle/reversal | Atmospheric FIGHTER only, unchanged |
| C / 0 / mouse | Existing cockpit, chase/dolly, orbit and zoom | Unchanged |

There is no nose-wheel mesh or pivot in the existing imported hierarchy. Steering
is operational, with no invented node lookup and no repurposing of robot limbs.
A later compatible wheel/strut asset can consume the operational steering angle.
The user-owned Blender files were not edited by this implementation.

## Tire forces and contact

`FighterGroundContact` no longer uses runway membership to decide support, braking
or contact validity. `on_runway()` remains available as a geometric diagnostic.
Deployed wheel points determine ground clearance using three sampled positions;
gear-up, excessive impact speed/attitude and body strikes retain their safety checks.
There is no automatic crash from crossing a runway edge or landing outside it.

Q/E feed a steering target rather than directly applying airborne yaw authority
while supported. A bicycle-style target rate is speed × tan(wheel angle) / wheelbase.
A bounded tire torque, yaw inertia and response integrate angular velocity; the
existing rotational integration applies that rate. There is no stationary pivot.
Lateral tire force turns ground velocity and opposes slip, bounded by tire friction
and actual normal wheel load. Steering angle fades with ground speed. Pitch and
lift remain in the existing integrator, allowing normal pilot-controlled rotation.

Defaults in `GroundHandlingParameters`:

| Parameter | Default |
| --- | ---: |
| Maximum steering angle | 28° |
| Steering fade speed | 45 m/s |
| Wheelbase | 10 m |
| Yaw inertia | 90,000 kg m² |
| Yaw response | 3/s |
| Lateral grip coefficient | 0.65 |
| Fighter traversable slope | 12° |
| VTOL landing slope | 25° |
| Battledroid ground slope | 35° |
| Severe obstacle normal impact speed | Greater than 8 m/s |
| Severe obstacle impact deceleration | Greater than 120 m/s² |
| Impact-duration estimate | 0.1 s, independent of physics timestep |

Surface rolling-resistance coefficients: runway/markings 0.015, taxiways 0.018,
aprons/streets 0.020, plazas 0.025, sidewalks 0.035, perimeter 0.040, paths 0.050,
desert 0.070, parks 0.080 and sandy/vacant lots 0.090. Original 5 m/s² capped,
load-dependent wheel braking now applies away from runways as well. Wheel brakes
and resistance still avoid velocity reversal at low speed.

## Authoritative ground and obstacles

`GroundEnvironment` is the shared world service, separate from runway geometry.
Inside approved flat city/airfield grading it returns the existing Y=0 ground.
Outside that area it samples the existing real/procedural `Terrain.height_at` into
a fixed 32 m CPU height lattice, with bilinear height and finite-difference normals.
The lattice does not depend on visible LOD or GPU residency. An LRU holds at most
512 height nodes; missing real data uses the existing procedural provider. Source
cache misses may still decode synchronously on first physics access; the flat
region takes a fast path and needs no raster decoding. This is localized height
support, not a new terrain renderer or spherical physics system.

Fighter support follows the terrain normal and slope suitability. Existing VTOL
and Battledroid height-query hooks now use this same service, including normal
updates and suitability checks; their landing/hover/movement force models and
mode-specific impact limits remain. No runway or hangar membership is imposed.

World solid geometry is indexed into 128 m XZ cells. Airport structural boxes,
hangar wall/roof boxes and conservative city building bounds are colliders.
Swept vehicle AABBs find the earliest obstacle contact, including movement across
an entire wall in one update. Gentle contact blocks normal motion; sufficiently
severe speed or estimated collision force crashes using the existing CRASHED
state. The inward velocity is removed and the vehicle stops before penetration.
Ground/obstacle collision is currently player-side, not a new NPC collision model.

## Hangar and connecting pavement

The middle airport block at (300,0,3000) is now a shell rather than a solid box:
90 m wide in X, 110 m in Z and 24 m high, with an open west side at X=255.
Its east wall is X=343–345; side walls lie at Z=2945–2947 and 3053–3055;
roof is Y=23–24. The wide entrance admits the canonical fighter (approximately
12 m span and 19 m length) with generous height clearance. Roof and individual
walls are physical colliders; there is no invisible entrance/front box.

Existing apron covers X=105–355, Z=2750–3250. A new 110 × 24 m connector centered
at (55,3000) joins it to the runway and existing X=100 taxiway. Pavement is cut
into the existing disjoint ground mesh, so no coplanar overlay is added. Airport
grading still derives from infrastructure bounds, keeping the shell/apron flat.
All new structural geometry remains in the existing static environment batch.

## Manual lifecycle

Run `.venv/bin/python main.py --atmosphere --enemies 0`.

1. Confirm heading 270°, GEAR DOWN and GROUNDED inside the hangar. Use modest Up
   input and release it; throttle persists. Taxi at roughly 3–5 m/s. Use 6 and
   Down to reduce speed; Q/E steer only once wheels are rolling.
2. Taxi west through the open wall at X=255, across the apron toward X=120.
   Use E to turn right toward heading 000°: with a broad 20 m radius you reach
   the X=100 taxiway near Z=2980. Keep clearance from the two solid neighboring
   hangars and tower. C/0 provide cockpit or orbit inspection as before.
3. Follow the taxiway north toward Z=2500. Use Q to turn left toward 270°, taxi
   west toward X=20, then E to align north near the runway centerline. F4 is an
   alternative direct runway reset, not a mandatory step in this lifecycle.
4. Use the existing takeoff procedure: flaps as desired, throttle up, gentle S
   rotation at roughly 75–85 m/s, positive climb, then gear up after liftoff.
   Base aerodynamic parameters and engine thrust have not changed.
5. Fly the circuit and return heading north to runway 36. Deploy gear well before
   touchdown, configure flaps at suitable speed, wings level, shallow descent
   below 4 m/s. Touch down with enough remaining rollout room and hold 6 while
   reducing throttle. Off-runway paved or suitable natural contact is allowed.
6. Once slow, use Q/E to taxi back via the connector at Z=3000. Approach the
   hangar heading 090° east, centered approximately at Z=3000. Taxi through
   X=255 and stop near X=300–310 with brakes and zero throttle. Do not drive into
   the east wall at X=343. There is no auto-docking or hangar menu.

The focused tests execute ground routes with test-generated pilot commands,
including taxi-out/takeoff and landing/rollout/taxi-return; they do not install an
autopilot. A complete human-flown circuit remains a manual validation procedure.

## Limits and validation

The wheel/strut meshes are still missing. Collision hulls are conservative AABBs,
not triangle-exact meshes or suspension; city towers use full building bounds.
Natural ground support is a 32 m interpolation of the CPU height field and can
smooth fine terrain features. No spherical rebasing, reverse taxi control,
differential braking, NPC collision system, wheel animation, or damage simulation
was added. Far-world float/projection limits remain as documented for M20.5.

New implementation: `game/ground_environment.py`. Updated world geometry,
fighter contact, flight-controller hooks, normal startup, and visual check tool.
Tests add `tests/test_ground_operations.py`; prior runway-only crash expectations
and runway-specific combat test setup were updated for the new lifecycle. The
existing imported-model baseline failures were not changed or hidden.

Final validation: all nine new ground-operation tests passed, including the
complete taxi-out/takeoff path, an airborne-final setup followed by landing,
braking and taxi-return into the hangar, Q/E steering dynamics, natural-ground
uphill support, unsafe slope, wall impact and off-airfield VTOL/robot contact.
All 18 gear/flap/brake tests and 15 world-environment tests pass. The full suite
ran 386 tests in 88.389 seconds: 384 passed with the same two pre-existing imported
model failures (folded-wing extent and negative scales). Those model tests were
not changed. OpenGL passed three complete world/VTOL/locomotion/transformation
and resource-cleanup cycles without errors or leaked handles. Final captures are
in `/tmp/taxi-gl-complete`; hangar-start and entrance views were visually inspected.

Files created: `game/ground_environment.py`, `tests/test_ground_operations.py`,
and this document. Updated for this refinement: `game/fighter_ground.py`,
`game/world_environment.py`, `game/world.py`, `engine/flight_controller.py`,
`engine/game.py`, `main.py`, `tools/check_transformation_gl.py`,
`tests/test_airfield_operations.py`, `tests/test_world_environment.py`,
`tests/test_combat.py`, `tests/test_missiles.py`, `README.md`,
`docs/AIRFIELD_OPERATIONS.md`, `docs/PLANETARY_WORLD.md` and `docs/TERRAIN_LOD.md`.

## Taxi steering collision regression

Sustained 5 m/s turns on clear apron pavement reproduce neither a heading limit
nor loss of support: yaw continues beyond two complete revolutions in either
direction. Ground yaw is preserved between integration steps, velocity follows
the aircraft with negligible side slip, and speed attenuation is mild at taxi
speed. No steering, airborne handling or runway parameters were changed.

The reproduced failure is in obstacle overlap recovery. A rotating conservative
fighter hull can overlap a nearby hangar wall. The collision routine calculated a
separating position, then overwrote it with the swept-motion response at fraction
zero. Even moving away from the wall advanced only the 1 mm contact bias each
update, giving the appearance of an invisible steering stop. Candidate evaluation
now retains the chosen separating position and applies it once. Ordinary swept
contacts and severe-impact crashes remain active. Conservative hulls still require
clearance around real walls; this does not make hangar geometry passable.

Regression coverage includes two-plus circles each with Q and E at 30/60/120 FPS,
curved trajectories, bounded side slip, continuous support, release and straight
taxi, signed reverse steering, increasing turning radius, overlap escape, and
existing gentle/severe swept hangar-wall impacts. For a manual check, taxi clear
of the hangar to the open apron around X=185, Z=3000, establish approximately
5 m/s, and hold each steering key for about 52 seconds. Release it and confirm
straight taxi. Leave sufficient space for the approximately 19 m turning radius
plus aircraft dimensions; nearby hangars remain physical obstacles.

Validation for this fix: all 12 ground-operation tests pass. The full unittest
suite runs 389 tests: 387 pass, with the same two existing imported-model failures
(folded-wing width in `test_pose_refinement` and negative node scales in
`test_transformation`). Python compilation and `git diff --check` also pass.
Manual rendering/flight inspection was not performed for this fix.
