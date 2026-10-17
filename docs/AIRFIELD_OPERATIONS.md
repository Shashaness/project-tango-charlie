> Ground operations now supersede runway-only support and boundary crashes. See [GROUND_OPERATIONS.md](GROUND_OPERATIONS.md) for current taxi, terrain and obstacle behavior. The M20.6 record below describes that milestone.

# M20.6 airfield operations

M20.6 adds operational gear, flaps and wheel brakes to atmospheric FIGHTER mode.
VTOL contact/hover, BATTLEDROID locomotion, transformations, camera profiles,
combat, terrain streaming and geographic frames retain their existing systems.
No base mass, engine, wing-area, lift-curve or attitude-control parameters changed.

## Inspection and controls

Keys 6, 7 and 8 were unused in `engine/input.py`; GLFW polling now assigns:

| Key | Action | Activation |
| --- | --- | --- |
| 6 | Wheel brakes | Held; force only with supported runway wheels |
| 7 | Flaps up/down | Press edge, atmospheric FIGHTER only |
| 8 | Gear toggle/reverse | Press edge, atmospheric FIGHTER only |

Existing X space brake / VTOL vector, Z vector, 1/2 missile selection, B chaff,
G transformation, R recovery, F4 runway reset, flight axes, C and 0 are retained.
Wheel brakes are separate from the existing `brake` command; FCC mappings did not
change. Flap/gear edges are processed once per game update, not once per substep.
Gear timing advances inside the existing fighter physics substeps. Controls are
ignored in SPACE, VTOL and BATTLEDROID; their aero/force paths do not gain flap or
gear effects. Operational state persists through transformations; F4 resets it.

## Model integrity

The imported hierarchy was inspected: Fuselage, Nose, wings, engines, legs/feet,
weapons and their geometry children are present. There are **no dedicated landing
gear, strut, wheel or flap nodes**. Runtime does not look up invented node names,
move robot feet as wheel assemblies, or modify the GLB/hierarchy/animation curves.

A later Blender asset pass needs a nose wheel/strut and two main wheel/strut
assemblies, articulated pivots with stored/deployed poses and agreed stable names.
Optional independently hinged flap meshes may follow. Bind their visual poses to
operational extension only after verifying compatible exports. Until then, gear
and flaps are operational/HUD-only; the body visibly floats above wheel contact
height because wheel meshes are absent. No re-export is needed to test physics.

Canonical fighter mesh bounds are approximately X ±6 m, Y -0.968 to 3.25 m,
Z -8.51 to 10.35 m. Configured wheel contacts in aircraft/model meters:

| Wheel | X | Y | Z |
| --- | ---: | ---: | ---: |
| Left main | -2 | -3 | 4 |
| Right main | 2 | -3 | 4 |
| Nose | 0 | -3 | -6 |

`wheel_world(vehicle)` applies the full vehicle orientation and translation.
Clearance follows the lowest wheel; partial-extension envelopes include the body
and interpolate wheel Y. Fully retracted contact uses the canonical body envelope.
The fully deployed attitude is also checked for body-versus-wheel penetration.

## Parameters and aerodynamic effects

All operational/contact thresholds live in `AirfieldParameters` in
`game/airfield_operations.py`. Supply an `AirfieldOperations(parameters)` override
on a vehicle for tests/configuration; defaults are:

| Parameter | Default |
| --- | ---: |
| Full gear deployment/retraction | 2.5 s |
| Fully extended extra drag coefficient | 0.025 |
| Flap extra drag coefficient | 0.08 |
| Maximum flap lift-coefficient increment | 0.35 |
| Angle of attack for full flap lift benefit | 10° |
| Rolling resistance / normal load | 0.015 |
| Maximum wheel-brake deceleration | 5 m/s² |
| Wheel-brake friction / normal load | 0.65 |
| Contact tolerance | 0.02 m |
| Body clearance comparison tolerance | 0.01 m |

Gear states are RETRACTED, DEPLOYING, DEPLOYED and RETRACTING. Pressing 8 during
transit reverses continuously from the current extension. Drag follows extension.
Flaps have RETRACTED/DEPLOYED states. Positive angle of attack ramps flap benefit
from zero to 0.35 at 10°, multiplied by the existing stall effectiveness. At zero
angle of attack they add no ground-roll lift: the pilot must rotate. High speeds
increase flap drag with dynamic pressure; no flap lift is added in SPACE.

The existing midpoint atmospheric integration, thrust, gravity, stall model and
control authority remain. Normal reaction cancels downward force only while
supported. Positive net lift releases the aircraft with no artificial launch
velocity, takeoff speed gate or autopilot. Wheel brakes oppose horizontal velocity,
scale down with normal load as wings unload the wheels, and are capped by both
5 m/s² and friction/load. Rolling resistance adds separately. Friction cannot
reverse velocity within a substep, and zero-speed normalization is avoided.
Brakes never apply to airborne aircraft. These are rigid operational contacts,
not a suspension or steering simulation.

## Runway and touchdown safety

The actual `RUNWAY` supplies center (0,0,2500), elevation 0, forward (0,0,-1),
length 2400 m and width 45 m. Bounds project wheel positions onto runway forward
and right vectors, rather than assuming an axis-aligned rectangle. All deployed
wheel points must lie in the usable strip for runway support. Current usable
strip is X ±22.5 m, Z 1300–3700 m. Spawn is 120 m inward from the south end and
faces runway 36. F4 sets gear DOWN, flaps UP, brakes released, zero throttle and
velocity, and operational wheel clearance (root Y=3 m). It creates no touchdown.

| Unsafe condition | Crash threshold |
| --- | --- |
| Gear not fully deployed at contact | Any operational touchdown |
| Vertical descent | Greater than 4 m/s |
| Absolute pitch | Greater than 18° |
| Absolute roll | Greater than 8° |
| Initial horizontal touchdown speed | Greater than 120 m/s |
| Outside usable runway / grounded departure | Greater than 5 m/s |
| Body extends below supporting wheels | More than 0.01 m |

A valid touchdown removes only vertical velocity and keeps horizontal velocity for
rollout. Existing CRASHED state freezes the aircraft and retains R/F2 airborne
recovery and F4 runway recovery. `airfield.crash_reason` records the cause. A
supported rolling aircraft crossing the runway boundary too quickly crashes as
RUNWAY OVERRUN; an airborne off-runway touchdown is OFF RUNWAY. Positive upward
liftoff is distinguished from ground departure. Parked or very slow aircraft near
an end are not crashed merely because a wheel lies outside the strip. Safe
stationary off-runway fighter handling retains the earlier frozen-ground behavior.

## HUD

A small lower-right panel, above countermeasure status and below the radar/weapon
readout, shows GEAR UP/DOWN/TRANSIT, FLAPS UP/DOWN, GROUNDED/AIRBORNE, and BRAKES
ACTIVE when loaded runway wheels have brakes applied. The normal parked fighter
no longer emits a generic reset warning. Descending below 150 m with gear not fully
down shows GEAR NOT DOWN; flaps above 95 m/s show FLAPS SPEED. Compass, geographic
coordinates/source colors, combat and flight indicators retain their locations.

## Manual takeoff, circuit and landing

Launch `.venv/bin/python main.py --atmosphere --enemies 0`. These are pilot instructions, not
an automated circuit or a claim that a human flight was completed.

1. Press F4. Confirm GEAR DOWN, FLAPS UP, GROUNDED, zero speed, heading 000°.
   Use C for cockpit/external and 0 for DOLLY inspection if desired. Press 7 for
   takeoff flaps. Hold 6 briefly to verify BRAKES ACTIVE, then release it.
2. Hold Up to increase throttle (two seconds to full). Stay aligned with Q/E,
   watching heading and runway centerline. At roughly 75–85 m/s begin gentle S
   nose-up input toward 8–10° pitch. Release S progressively; verify AIRBORNE,
   positive vertical speed and no ground reaction. Do not retract gear while rolling.
3. Once climbing clear of the runway, press 8; observe TRANSIT then UP after
   2.5 s. Press 7 to retract flaps before accelerating through 95 m/s. Climb to
   approximately 250–400 m. Adjust throttle with Up/Down to maintain airspeed.
4. Fly a broad rectangular circuit: depart north, turn west, then south for
   downwind, then east for base and north for final. Use Q/E for heading with
   modest A/D bank and S/W pitch corrections; level the bank for final. Avoid
   terrain and the city north of the airport. The prototype has no coordinated
   turn autopilot or full navigation/approach aids.
5. Set up south of the strip (Z greater than 3700 m), tracking heading 000° toward
   its centerline. Reduce speed toward 70–80 m/s. Press 8 well before touchdown
   and wait for DOWN; press 7 for flaps below 95 m/s. Use small throttle/pitch
   adjustments to stabilize a shallow descent, preferably 1–2 m/s. Landing
   requires all wheels inside the strip; avoid touching down short of it.
6. Cross the south threshold with wings level, pitch roughly 3–8°, sink rate below
   4 m/s. Aim to touch down around Z 3400–3500 m, leaving room for rollout.
   Root Y is not wheel altitude: level deployed-wheel contact occurs at Y=3 m,
   and nose-up pitch changes the required root clearance.
7. Confirm GROUNDED, reduce throttle to zero, and hold 6 until stopped. A 70 m/s
   rollout needs roughly 500 m at nominal braking, plus margins; do not wait near
   the far end. Release 6 and confirm speed stays near zero. If an approach is
   unstable, increase throttle and go around while still airborne; F4 resets a crash.

## Known limitations

World ground/contact remains globally Y=0. Visual SRTM/procedural hills are not
collision surfaces, and buildings have no active rigid-body collision; M20.6
adds neither a terrain collision engine nor obstacle raycasts. This limitation
was already present. Use the flat, graded runway for operations. Contact is
zero-restitution without wheel suspension, tire steering, differential brakes,
wind, hydroplaning, runway friction variations, damage or gear/flap meshes.
Body-strike checks use the canonical fighter envelope, not animated landing gear.
Transformation clearance/VTOL feet/robot locomotion are unchanged; transforming
a fighter during rollout does not supply an automatic gear interlock.

## Files and validation

Created `game/airfield_operations.py`, `tests/test_airfield_operations.py`, and
this document. Modified `engine/input.py`, `engine/game.py`,
`engine/flight_controller.py`, `engine/hud.py`, `game/player_vehicle.py`,
`game/atmospheric_physics.py`, `game/fighter_ground.py`,
`game/world_environment.py`, `tests/test_atmosphere.py`,
`tests/test_world_environment.py`, `tests/test_vtol_ground.py` and `README.md`.
Legacy skid-contact expectations were updated for gear-equipped runway operation
and the required gear-up crash behavior; the imported-model baseline failures
were not modified.


All 18 new operational tests passed. Existing VTOL ground tests (16) and the
world/atmosphere regressions pass with updated fighter gear-contact expectations.
The complete suite ran 377 tests in 66.711 seconds: 375 passed, with the same two
pre-existing imported-model failures (folded-wing lateral extent and negative node
scales). No failures were removed or weakened in the imported-model tests.


The final real OpenGL check passed three complete upload, transformation,
VTOL/locomotion and resource-cleanup cycles without GL errors or leaked handles.
A preceding synthetic-terrain check also passed three cycles. The final FIGHTER
capture in `/tmp/m206-gl-final/FIGHTER.png` was visually inspected: radar/weapon
text, compass/geographic readouts and the lower-right operations panel are
separate. The manual full circuit remains a user-operated flight procedure;
automated tests validate takeoff and landing/rollout separately.
