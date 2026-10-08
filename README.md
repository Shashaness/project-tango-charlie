# Project Tango Charlie

Project Tango Charlie is an experimental cross-platform Python/OpenGL 6DOF
combat-flight and transformable-vehicle simulator, being prepared for open-source
publication. It uses Python, GLFW, PyOpenGL, NumPy and OpenGL 3.3 Core.

The pilot is stranded on an unfamiliar planet with their trusted transformable
battledroid, **TC167**, associated with **Tango Charlie**. The pilot's gender is
unspecified. TC167 has three configurations: **FIGHTER** (high-speed aircraft),
**VTOL** (vertical-takeoff/hover), and **BATTLEDROID** (humanoid combat).
See [the premise](docs/premise.md). No additional dependencies are required.

## License

Project Tango Charlie is released under the **MIT License**.

Copyright (c) 2026 Shawn Bakhtiar.

Unless otherwise noted, this license applies to the entire repository, including:

- Source code and scripts
- 3D models and GLB assets
- Textures, materials, and shaders
- Original sound effects and music
- Documentation and other original project assets

You are free to use, modify, distribute, and commercially exploit these materials under the terms of the MIT License.

See [LICENSE](LICENSE) for the complete license text.

Third-party dependencies and assets retain their respective licenses. Only materials owned by the project or appropriately licensed for redistribution are included under the project's MIT License.


## Run

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

On Windows, activate with `.venv\Scripts\activate` instead. An existing environment
can be activated directly. A display and OpenGL 3.3 Core graphics driver are required.

## Coordinates, units, and controls

Right-handed coordinates: **+X right, +Y up, -Z forward**. One render unit is
one meter. Simulation uses SI: position in meters, velocity in m/s,
acceleration in m/s², mass in kg, force in N, pressure in Pa, time in seconds.
Internal angles/rates use radians and rad/s; HUD angles use degrees.

| Key | Action |
| --- | --- |
| W / S | Pitch down / up |
| A / D | Roll left / right |
| Q / E | Yaw left / right |
| Up / Down | Increase / decrease persistent throttle |
| Left / Right | Local-right maneuvering thrust in VTOL/BATTLEDROID or SPACE; disabled in atmospheric FIGHTER |
| X | Vector thrust upward in VTOL; inertial brake in SPACE FIGHTER/BATTLEDROID; ignored in atmospheric FIGHTER |
| Z | Vector thrust forward in VTOL |
| G | Cycle FIGHTER → VTOL → BATTLEDROID → FIGHTER once per press; preserves momentum |
| K | Battledroid grounded thruster jump; no effect in Fighter/VTOL |
| F3 | Toggle SAS stability assist (default ON) |
| V | Toggle airborne atmospheric VTOL/BATTLEDROID hover assist (default OFF) |
| TAB | Select next available living target (once per key press) |
| M | Launch one selected missile per press, requiring established lock and inventory |
| 1 / 2 | Select RADAR / IR missile; switching clears acquisition |
| L | Dispense one flare per press (F retains downward thrust) |
| B | Dispense one chaff package per press |
| J | Toggle ECM (default OFF) |
| Spacebar | Hold for automatic fixed-forward gun fire |
| C | Toggle chase / cockpit |
| F1 | Select SPACE, retaining position, orientation, velocity, and throttle |
| F2 | Reset and select atmospheric test flight |
| R | Local-up maneuvering thrust in VTOL/BATTLEDROID; atmospheric FIGHTER reset; SPACE FIGHTER local-up translation |
| F | Local-down maneuvering thrust in VTOL/BATTLEDROID or SPACE; disabled in atmospheric FIGHTER |
| H | Toggle force vectors in VTOL/ATMOSPHERE, orientation axes in SPACE FIGHTER (chase view) |
| ESC | Quit |

Opposing keys cancel. Throttle changes at 50 percentage points per second,
clamped to 0–100%, and persists on release. Throttle controls thrust, never
velocity. Orientation controls force directions but never directly rotates
existing world-space momentum. No orientation correction to world-up occurs.
F1/F2/R are development controls; F2 resets on each new press, not each held frame.

## Architecture and SPACE behavior

Game owns World, PlayerVehicle, FlightController, Camera, and Renderer.
PlayerVehicle owns position, basis/orientation, velocity, acceleration,
angular rates, throttle, FlightState, and the latest aerodynamic telemetry.
`engine/transform.py` shares Rodrigues' axis-angle rotation and orthonormal
basis maintenance. Orientation is a local-to-world matrix with columns
right, up, backward. It has no Euler-angle singularity or world-up correction.

FlightState uses Environment (SPACE/ATMOSPHERE), VehicleMode
(FIGHTER/VTOL/BATTLEDROID), and FlightStatus (FLYING/GROUNDED/CRASHED) enums.
All three configurations are operational. Atmospheric force calculations live in
`game/atmospheric_physics.py`; FlightController chooses the physics path.
The HUD/cameras only observe state and do not modify physics. FlightController
owns a FlightControlComputer which reads pilot input/vehicle feedback and returns
bounded control commands before integration. The FCC never writes vehicle state.

SPACE preserves the existing analytic integration of
`dv/dt = local_thrust_acceleration - damping * velocity`, with midpoint
orientation/throttle sampling and steps no longer than 1/120 second.
There is no gravity, lift, aerodynamic drag, or ground contact in SPACE.
VTOL uses the same SPACE damping/integration but its configuration-specific
engine and maneuvering forces, without the X inertial brake.
Existing SPACE constants retain their values:

| Constant | Value |
| --- | --- |
| MAX_THRUST (forward acceleration) | 12 m/s² |
| TRANSLATION_ACCELERATION | 8 m/s² |
| PITCH_RATE / YAW_RATE / ROLL_RATE | 60 / 60 / 90 deg/s |
| DAMPING | 0.35 /s |
| BRAKE_DAMPING (additional while X held) | 2 /s |
| THROTTLE_RATE | 0.5 /s |
| MAX_STEP | 1/120 s |

## Atmospheric forces and signs

The air velocity is `vehicle.velocity - wind_velocity`, with wind fixed at
zero. No density variation with altitude is implemented. For speed V:

```text
q = 0.5 * rho * V²
vf = dot(air_velocity, forward)
vu = dot(air_velocity, up)
vr = dot(air_velocity, right)
alpha = atan2(-vu, vf)
beta = atan2(vr, hypot(vf, vu))
```

Positive AoA means the nose is above the flight path in the local pitch plane.
Positive beta means velocity toward local right (incoming air from the right).
AoA/beta come from airflow and orientation, not keys. At zero airspeed both
are reported as zero and aerodynamic forces vanish without normalization.

The exact prototype coefficient and force equations are:

```text
excess = max(0, abs(alpha) - radians(stall_angle_deg))
effectiveness = exp(-(excess / radians(stall_decay_width_deg))²)
CL = clamp(lift_curve_slope * alpha, -max_CL, max_CL) * effectiveness
CD = CD0 + induced_drag_factor * CL² + stall_drag_coefficient * (1 - effectiveness)
L = q * wing_area * CL
lift_direction = normalize(cross(vehicle.right, air_velocity / V))
lift_force = lift_direction * L
D = q * wing_area * CD
drag_force = -(air_velocity / V) * D
thrust_force = vehicle.forward * max_engine_thrust * throttle
gravity_force = (0, -mass * gravity, 0)
acceleration = (lift_force + drag_force + thrust_force + gravity_force) / mass
```

Pure spanwise airflow has no defined pitch-plane lift direction, so lift is
zero there. Drag still acts. Lift is perpendicular to airflow and follows
bank; banked turns come from banked force, with no velocity/yaw turn hack.
Negative AoA produces negative lift. Inverted positive-AoA flight produces
downward lift; inverted negative-AoA can produce upward lift, as dictated by
signed CL rather than automatic altitude holding.

Above the stall angle, the Gaussian effectiveness smoothly reduces lift
while an additional drag term increases toward 0.45. There is no binary lift
cutoff. The same response applies to negative AoA. Control authority also
softens during stalls. This is an understandable prototype force model,
not CFD, a supersonic model, or a rigid-body aerodynamic moment simulation.

For aerodynamic rotation control:

```text
q_reference = 0.5 * rho * reference_airspeed²
pressure_factor = 1.25 * q / (q + 0.25 * q_reference)
stall_factor = 0.35 + 0.65 * effectiveness
authority = pressure_factor * stall_factor
angular_rate = pilot_command * configured_angular_rate * authority
```

At zero speed control authority is zero; at reference speed it is one and
approaches 1.25 at high speed before stall attenuation. No automatic trim,
weathercock stability, or physical angular inertia is simulated. Pitch/roll/yaw remain
direct rate controls scaled by aerodynamic authority.

Forces use explicit midpoint integration in substeps no longer than 1/120 s:
evaluate initial acceleration, predict midpoint velocity, evaluate midpoint
forces, then advance position with midpoint velocity and velocity with
midpoint acceleration. Atmospheric physics never applies SPACE damping or X braking. Atmospheric
FIGHTER has no strafe/vertical maneuvering thrust; VTOL has these forces. Nonfinite values raise clear
errors instead of silently entering state. Zero vectors are guarded.

## Prototype aircraft parameters

All atmospheric values are grouped in the frozen AircraftParameters dataclass
and PARAMETERS instance in `game/atmospheric_physics.py`:

| Parameter | Current value |
| --- | --- |
| mass | 6000 kg |
| wing_area | 30 m² |
| air_density | 1.225 kg/m³ |
| gravity | 9.81 m/s² |
| max_engine_thrust | 120000 N |
| zero_lift_drag_coefficient | 0.05 |
| induced_drag_factor | 0.06 |
| lift_curve_slope | 4.5 per radian |
| max_lift_coefficient | 1.4 |
| stall_angle_deg | 24° |
| stall_decay_width_deg | 30° |
| stall_drag_coefficient | 0.45 |
| high_aoa_warning_fraction | 0.8 (warning starts at 19.2°) |
| pitch_authority_deg | 60 deg/s |
| roll_authority_deg | 90 deg/s |
| yaw_authority_deg | 40 deg/s |
| reference_airspeed | 100 m/s |
| minimum_stall_control_factor | 0.35 |
| test_altitude | 1000 m |
| test_airspeed | 120 m/s |
| ground_altitude | 0 m |
| crash_speed | 5 m/s |

F2/R set position `(0, 1000, 0)` and velocity `(0, 0, -120)` with zero bank/yaw.
The reset numerically solves a small nose-up trim angle and throttle to balance
lift, thrust, gravity, and drag at this speed. With the tuned 120 kN engine,
trim throttle is approximately 11.68% at 2.7994° AoA; UP still ramps toward
full combat thrust. This lower starting throttle preserves the balanced test
condition rather than silently adding excess thrust at reset. Slight positive incidence is
necessary because the model's symmetric wing has CL=0 at zero AoA. This gives
a level flight path, not a nose exactly horizontal. Reset clears contact state,
angular rates, and telemetry, and snaps the camera without changing its mode.

Y=0 is the flat ground reference. Atmospheric contact clamps Y to zero and
freezes simulation: at total speed >=5 m/s status is CRASHED, otherwise GROUNDED.
FIGHTER can use R to reset. In VTOL, R is thrust: use F2 to reset a crash
to the atmospheric FIGHTER test flight. There is no landing gear, bouncing,
terrain, damage, or ground handling. SPACE ignores this contact rule.

## Cameras, HUD, and debug drawing

Chase remains behind/above the craft with exponential position smoothing.
Cockpit copies vehicle position and orientation, hiding the wedge mesh.
The camera far plane is 20 km so the 1000 m start can see the sparse flat
ground grid. The local grid remains near origin; cubes retain their positions.

HUD shows SPD (airspeed in ATMOSPHERE), ALT above Y=0, THR, AOA/BETA in degrees,
MODE, ENV, and CRASHED/GROUNDED when relevant. Atmospheric debug data also
shows VS (signed world vertical speed in m/s), CL, CD, and lift/drag/thrust
magnitudes in kN. HIGH AOA starts at 19.2° absolute AoA; STALL starts at 24°.
Warnings only report the envelope; they do not alter controls or stabilize flight. AOA/BETA show '-' in SPACE.
A tiny internal debug font draws triangles in framebuffer-pixel orthographic
coordinates after the 3D scene. Depth state is restored and all GPU buffers
are explicitly released before context destruction. Resize updates both HUD
coordinates and camera aspect ratio. No GUI or font dependency is added.

Green center cross is the cockpit nose bearing. In chase, the additional green
chevron shows actual nose direction because the camera looks down at the craft.
The amber flight-path ring projects actual velocity using
`projection @ view @ [normalized_velocity, 0]`, independently of the nose.
Off-screen/rearward travel uses an edge cue; speed below 0.05 m/s hides it.

H in atmospheric chase draws lift green, drag red, thrust blue, velocity amber.
Force vectors use 1 render meter per 10000 N, velocity 1 meter per 20 m/s,
with displayed length limited to 12 m. These visual scales never affect physics.
SPACE FIGHTER H retains its orientation axes; SPACE VTOL shows forces. Debug vectors are hidden in cockpit.

## Manual checks

- **SPACE regression:** F1; test throttle persistence, pitch/roll/yaw, strafe,
  X braking, and drifting independently of orientation.
- **Level flight:** F2; start at 1000 m and 120 m/s. Increase/decrease throttle
  and observe acceleration/deceleration from forces.
- **Banked turn:** roll 45–60°, add some pitch-up, use H to see banked lift,
  and observe velocity turn without a yaw hack.
- **Stall:** reduce speed and pull up beyond 24° AoA. Look for progressive
  lift/control loss and increased drag; lower the nose to regain speed.
- **High AoA:** in cockpit, pull up at speed and see nose/velocity markers
  diverge while AOA increases.
- **Inverted:** roll 180° and confirm body-relative lift and no automatic leveling.
- **Low speed:** controls weaken; authority returns with speed.
- **Crash/reset:** dive into Y=0, verify CRASHED and frozen motion, then R.
- **Camera/HUD:** C switches views without changing physics; resize/minimize/
  restore, check ENV/angles, toggle H, and quit with ESC.

## Automated checks

```sh
python -m unittest discover -s tests -v
```

Tests cover atmosphere/SPACE separation, signed AoA/beta, banked/inverted lift,
drag direction, zero/reverse/spanwise airflow, continuous stall, trim, control
authority, disabled atmospheric strafe/braking, crash/reset, world momentum,
frame-rate/camera independence, HUD projection, input edges, and cleanup.

## Milestone 6 combat tuning regression

Atmospheric engine thrust increased from 25 kN to 120 kN with mass still
6000 kg: thrust-to-weight changed from 0.425 to 2.039 and maximum engine
acceleration is 20 m/s². Stall onset increased from 18° to 24°, Gaussian decay
width from 20° to 30°, and extra post-stall drag coefficient decreased from
1.2 to 0.45. CL remains capped at 1.4 before stall attenuation. No SPACE
physics, control mappings, or velocity steering changed.

With the retained Gaussian equation, CL at 25°/35°/60°/90° is approximately
1.398/1.224/0.332/0.011; CD is 0.168/0.196/0.400/0.496. Useful lift persists
through 25–35°, but extreme AoA still loses lift and bleeds energy.

`tests/test_combat_energy.py` starts at the standard trimmed condition and
sets full throttle. It commands right roll for 0.6 s, full pitch-up for 10 s,
then scripted pilot recovery (left roll 1.3 s, pitch-up at 30% for 2 s), followed
by 10 s with no rotation command. It uses the actual velocity heading to
check reversal, not nose direction. Recovery is pilot input; no autopilot
or automatic nose alignment is introduced.

At 120 FPS the velocity heading reached -177.47°, minimum speed was
63.57 m/s, and maximum altitude loss across maneuver/recovery was 254.48 m.
Speed at reversal end was 76.90 m/s; after recovery controls plus 5 seconds
of uncommanded rotation it was 201.60 m/s, then 258.39 m/s after another
5 seconds. Final altitude was 804.23 m and vertical speed +8.77 m/s.
All states remained finite and airborne. Equivalent runs at 30/144 FPS
check energy metrics against the 120 FPS result. These are reproducible
scripted regressions, not a guarantee for every possible pilot maneuver.

The straight-line test holds the initial orientation and full throttle for
5 seconds: speed increases from 120 to 198.41 m/s, altitude reaches 1015.96 m,
and initial forward acceleration is 17.64 m/s² after drag. It demonstrates
substantial excess thrust without requiring perfectly constant altitude.

## Milestone 7: VTOL and transformation

G invokes `PlayerVehicle.toggle_configuration()`, changing only the existing
VehicleMode enum. Position, velocity, orientation/basis, angular rates, altitude,
throttle and environment are untouched. Leaving grounded Battledroid releases its support state. The same vehicle and
camera persist. No reset, velocity rotation, or speed clamp occurs during transformations. This explicit method is a future transition-animation boundary;
there is no animation or transition physics now.

`game/vtol_physics.py` groups the new configuration parameters and force
helpers. The existing atmospheric force evaluator and both integrators reuse
them; there is no separate unrelated VTOL simulation.

| VTOL parameter | Value |
| --- | --- |
| Maximum engine thrust | 110000 N |
| Mass (shared with FIGHTER) | 6000 kg |
| Thrust-to-weight ratio | 1.86884 |
| Wing lift factor | 0.30 |
| Drag multiplier | 3.5 |
| Lateral / vertical maneuvering thrust | 48000 N per axis (8 m/s²) |
| Thrust vector change rate | 0.5 /s |
| Pitch / roll / yaw rate | 60 / 90 / 60 deg/s |

Vector starts at zero (forward), persists across G toggles, and F2 resets it.
X increases it toward one; Z decreases it toward zero. Opposing keys cancel.
Holding either for two seconds traverses the full range. Engine force is:

```text
direction = normalize(forward * (1 - vector) + up * vector)
thrust_force = direction * 110000 * throttle
maneuver_force = (right * strafe_command + up * lift_command) * 48000
```

The up/right axes are vehicle-local, including when inverted. Intermediate
vector percentages are normalized linear blends, not linear angle degrees.
With HOV OFF and SAS OFF there is no hover assistance or rate augmentation.
Optional Milestone 8 assists are documented below; neither levels attitude. At zero speed with a level
vehicle and vector=100%, throttle `mass * gravity / max_thrust = 53.5091%`
balances weight. Higher throttle climbs and lower throttle descends; existing
vertical momentum persists. Pitching/rolling redirects thrust and reduces
its available world-up component, so hover requires pilot management.

In ATMOSPHERE, the existing signed CL is multiplied by 0.30 for wing lift,
and the existing CD (including induced and progressive post-stall drag) is
multiplied by 3.5. This preserves airflow-derived forces while penalizing
high-speed deployment. There is no conventional VTOL stall warning:
AoA/CL/CD still appear, but low-speed thrust and attitude controls remain
available. Rates are independent of dynamic pressure and stall effectiveness.

In SPACE, VTOL has no lift/drag/gravity. Engine and maneuvering forces are
divided by the shared mass and integrated using existing temporary SPACE
damping. X vectors thrust instead of adding inertial braking. Returning to
FIGHTER restores the previous SPACE controls/forces without changing momentum.

The HUD shows VEC percentage in VTOL and retains speed, altitude,
throttle, mode, environment, and the actual flight-path marker. H thrust debug
rotates with Z/X; colors/scales remain as documented above. The temporary
VTOL shape adds two downward engine/leg blocks with orange feet to the
existing wedge, attached to the same model transform. Cockpit hides all body
geometry. All additional mesh buffers participate in explicit cleanup.

### Exact manual procedure

1. Run `python main.py`, press F2, then UP to accelerate toward 150–200 m/s.
   Use small attitude inputs to keep the path approximately level.
2. Press G once. Confirm MODE VTOL, deployed legs, and continuous position/
   velocity (subsequent drag changes speed, not transformation itself).
3. Hold X about two seconds to reach VECTOR 100%; H shows thrust rotating up.
   The 3.5x aerodynamic drag slows travel. Full throttle produces strong climb;
   reduce throttle as speed falls.
4. To cancel remaining forward drift, pitch the nose up with S: vehicle-up
   thrust then tilts backward. Use the amber velocity marker and VS readout.
   Restore a level attitude as horizontal speed approaches zero to avoid
   accelerating backward. Drag alone becomes weak near zero speed.
5. With wings level, vector fully upward, and little drift, tune throttle near
   53.5%. Watch vertical speed as well as altitude: balancing weight alone does
   not erase an existing climb/descent. Use throttle or R/F to arrest it.
6. Test W/S pitch, Q/E yaw, A/D roll, Left/Right strafe, and R/F vertical thrust
   near zero speed. Tilt forward/back to accelerate/brake through thrust.
7. To return to FIGHTER, hold Z to move thrust forward, increase throttle,
   manage altitude with remaining vectored thrust/pitch, and build useful
   forward speed. Press G and verify momentum remains continuous while full
   wing lift returns. Stored vector has no effect in FIGHTER.
8. Press F1; test VTOL vectoring/translation without gravity, then G and
   confirm original SPACE FIGHTER throttle/strafe/X braking. Test C, H, resize,
   and ESC. F2 is a development reset, not a transformation.

`tests/test_vtol.py` covers exact transform/throttle/angular-rate preservation
both ways, upward thrust/acceleration, ten-second force-equilibrium hover,
persistent vertical momentum, increased drag, reduced lift, zero-speed controls,
local maneuvering forces, SPACE isolation, G key edges, vector limits, warning
suppression, and frame-rate consistency. Battledroid is implemented by Milestone 13 below. Optional Milestone 8 vertical-speed assistance
is documented below.

## Milestone 8: flight-control computer and instruments

`game/flight_control_computer.py` defines FlightControlComputer and a small
ControlCommands dataclass. Each simulation substep reads pilot commands and
feedback, returns augmented pitch/yaw/roll commands plus an optional absolute
engine-throttle command, then lets the existing forces/integrators advance the
vehicle. FCC only mutates its own enable/override state. Transformation still
preserves position, world velocity, orientation, angular rates and pilot throttle.
The existing combat energy test explicitly runs with SAS OFF to keep the
Milestone 6/7 manual regression comparable.

### SAS

SAS defaults ON and F3 toggles it once per press. It applies to VTOL in
both environments and FIGHTER in ATMOSPHERE. SPACE FIGHTER behavior is unchanged.
With an axis released, its current angular rate decays by
`exp(-8 * dt)` and is converted back to a normalized axis command bounded to
[-1,1] and the current mode's control authority. An actively commanded axis
passes through immediately; the FCC does not delay pilot input. No world-up,
roll-zero, pitch-zero, AoA-zero or altitude target enters this calculation.

SAS OFF preserves previous manual/direct-rate controls: rates follow held keys
and return to zero immediately on release. This project has no physical
angular momentum model, so manual release already stopped rotation. SAS ON
adds a short, smooth rate tail; it is not being presented as a new rigid-body
angular simulation. An upside-down vehicle with zero rates stays upside-down.

### Optional hover assist

V toggles HOV only for airborne ATMOSPHERE + VTOL. It defaults OFF. Enabling
captures `hover_target_altitude` for future use, but altitude error is deliberately
not used: this is vertical-velocity hold, not altitude locking. Leaving this
mode/environment, reset, or ground contact clears it.

```text
passive_force = gravity + aerodynamic forces + current maneuvering force
required_total_acceleration_y = -1.5 * world_vertical_velocity
required_engine_force_y = mass * required_total_acceleration_y - passive_force.y
available_engine_force_y = VTOL_MAX_THRUST * thrust_direction.y
engine_throttle = clamp(required_engine_force_y / available_engine_force_y, 0, 1)
```

The resulting throttle passes through the normal vectored-engine force and
midpoint integration. No velocity/position assignment is made by FCC. Pilot
persistent throttle remains untouched; `engine_throttle` separately records
what physics actually used. THR displays this applied engine command; H debug
shows PILOT THR as well. With HOV disabled, the manual engine command is restored.
There is no horizontal hold: strafe, yaw, pitch, roll and horizontal flight remain
available. Tilt reduces available upward thrust; commands saturate at 100% rather
than inventing force. When thrust's world-up component is below 0.15 (nearly
horizontal/inverted), correction is suspended.

Any throttle-rate, thrust-vector, or vertical maneuver input overrides HOV for
as long as held and 0.75 seconds afterward. The pilot can deliberately climb or
descend. Corrections resume after that grace period. HOV remains enabled during
this override but the warning `HOV PILOT / LIMITED` makes suspension visible.
No automatic vector adjustment, attitude leveling, navigation, or landing is added.

### Read-only instrumentation and HUD layout

`game/flight_instruments.py` provides:

- VSI: exactly `vehicle.velocity.y` in m/s, positive climbing.
- Normal G: `dot(vehicle.acceleration - world_gravity, vehicle.up) / 9.80665`.
  Atmospheric world_gravity is `(0,-9.81,0)`; SPACE gravity is zero. This is a
  signed local-up specific-force/load approximation, not speed-derived G.
  Level atmospheric equilibrium is about +1 G; free fall is zero. SPACE uses
  the existing stored thrust/maneuver acceleration, excluding its artificial
  damping bookkeeping.
- Heading: `degrees(atan2(forward.x, -forward.z)) % 360`, giving -Z=000,
  +X=090, +Z=180, -X=270. When horizontal forward magnitude is below 1e-6,
  heading displays ---.
- Pitch: `asin(clamp(forward.y,-1,1))`.
- Bank: `atan2(-right.y, up.y)`. Near vertical, bank/horizon reference is
  undefined and bank displays --- instead of unstable normalization.

Left HUD: SPD, ALT, signed VSI, HDG, signed G, signed AOA/BETA.
Right HUD: THR, VEC, MODE, ENV, SAS, HOV. Center retains the fixed reticle and
actual-velocity flight-path marker; the separate lower-center circular attitude
reference has a pitch ladder, rotated horizon, directional ground-side ticks,
bank pointer/reference marks and numeric pitch/bank. This instrument uses the
vehicle basis relative to world +Y, independent of chase/cockpit camera, and
never feeds physics. It displays inverted bank without leveling the aircraft.
The VTOL lower-right vector graphic points between local FWD and UP.
Its value represents THRUST FORCE; nozzles/exhaust point in the opposite direction.

Warnings occupy the upper-center area. HIGH AOA/STALL only apply to atmospheric
FIGHTER; crash/reset guidance accounts for the VTOL R-thrust binding. H toggles
both detailed lower-left CL/CD/force telemetry and existing 3D debug vectors.
Normal HUD omits those physics internals. Text uses the existing internal font,
now with signed values; there are no new dependencies or textures.

The amber flight-path marker still projects `[normalized_world_velocity,0]`
through the observing camera matrices. Near-zero speed hides it, rear/offscreen
vectors give a bounded edge cue, and nonfinite directions/projections are rejected.
In cockpit the fixed center is the nose; chase retains a separate true nose cue
because its viewing axis points down toward the aircraft.

### Exact manual checks for Milestone 8

1. Launch, check SAS ON/HOV OFF, then F2. Check left/right groups, heading,
   signed VSI/G/AOA/BETA, and the attitude reference. H reveals detailed data.
2. Press G, vector fully upward with X, level the vehicle and manually tune
   throttle near 53.5% with HOV OFF. Confirm manual hover still requires finesse.
3. Introduce a small climb/descent, press V, release throttle/vector keys, and
   watch VSI settle toward zero through changing engine thrust. Altitude may
   drift while vertical speed settles; horizontal strafe/yaw remain available.
4. Hold UP/DOWN or R/F to deliberately climb/descend; X/Z changes vector without
   fighting the FCC. Check HOV PILOT / LIMITED during override and resumption
   after release. Toggle V off to restore the pilot throttle setting.
5. F1, VTOL: F3 OFF, hold D then release (raw rate stops immediately). F3 ON,
   repeat and observe gradual rate decay. Repeat inverted; settled attitude
   must remain inverted. SPACE FIGHTER must still feel unchanged.
6. F2: roll 45°, pitch up/down, and roll inverted. Check bank pointer, horizon
   tilt/pitch shift, ground-side marks and numeric attitude. C must not change
   these vehicle-relative instrument values.
7. In atmospheric FIGHTER, pull/push at speed: nose/flight-path separation and
   AoA/beta are real airflow values; signed G responds to specific force.
   Confirm stall warnings remain FIGHTER-only.
8. Hold F3/V/G/C to confirm one toggle per press. Check F1/F2 reset edges,
   transformation momentum, crash/F2 reset, resize/minimize/restore and ESC.

New regression tests cover heading cardinals/vertical guard, exact VSI,
FCC non-mutation and output bounds, hover velocity decay and altitude freedom,
pilot override, eligibility/thrust limits, SAS decay/manual behavior/inverted
attitude, signed G, world-up attitude instruments, safe velocity projection,
HUD debug grouping and F3/V key edges. Existing transformation, SPACE,
FIGHTER, and manual VTOL tests continue to run.

## Milestone 9: guns, targeting and fire-control radar

Targets live in World independently of the camera and vehicle. Each has its
own ID, position, constant world-space velocity, collision radius, and alive
flag. Default targets have a 25 m sphere radius and orange octahedron geometry:

| ID | Initial position (m) | Velocity (m/s) |
| --- | --- | --- |
| 01 stationary | (0, 0, -1000) | (0, 0, 0) |
| 02 crossing | (-500, 100, -1500) | (50, 0, 0) |
| 03 vertical | (300, -200, -2000) | (0, 30, 0) |
| 04 receding | (0, 200, -2500) | (0, 0, -75) |

Targets are constant-velocity test objects, not enemy AI. A hit immediately
marks one target dead. It stops rendering and is skipped by selection; there
is no health, damage model, explosion, or hostile fire. Initially target 01 is
selected automatically. TAB cycles available living targets once per press. Destruction
selects the next survivor; with no survivors current_target is None. Targets
continue moving through F1/F2 flight resets; restart the application for a fresh
combat test set. These resets do not respawn targets or redirect existing rounds.

### Responsibilities and timing

- `game/target.py`: target state and constant-velocity motion.
- `game/fire_control_radar.py`: selected target, range/closure/relative motion,
  line of sight, radar-range gating, lead solution and SHOOT cue.
- `game/fire_control_solution.py`: pure mathematical constant-velocity intercept.
- `game/gun.py`: local muzzle transform, launch velocity and shot cooldown.
- `game/projectile.py`: finite-lifetime constant-velocity motion and swept collision.
- `game/weapons.py`: CombatSystem owns rounds and coordinates gun timing, target
  updates, collision/removal, brief HIT display, and radar refresh.

Game advances flight and combat together in steps no longer than 1/120 s.
Within each step, gun cooldown generates birth-time offsets and the shooter
position/velocity are sampled between the step endpoints. Orientation is sampled
from that small step's resulting vehicle basis. New rounds advance only for the
time since their birth, not for an entire rendered frame. Idle time does not
bank a burst of shots. Targets move using `position += velocity * dt`.

The radar reads perfect target information and never creates rounds. The HUD
only reads radar/combat state and projects symbology; it does not solve
intercepts or perform weapon physics. The gun never changes flight state:
there is no recoil, auto-aim, homing, speed clamp, or momentum reset.

### Prototype gun and radar values

| Parameter | Value |
| --- | --- |
| Muzzle velocity relative to shooter | 1000 m/s |
| Fire rate | 12 rounds/s |
| Projectile lifetime | 5 s |
| Projectile collision radius | 0.5 m |
| Local muzzle offset (+X right, +Y up, +Z backward) | (0, 0, -3) m |
| Gun effective range | 2000 m |
| SHOOT angular tolerance | 1.5° |
| Radar maximum range | 10000 m |
| HIT display duration | 0.6 s |

Gun parameters are in GunParameters/GUN; radar range is RADAR_MAX_RANGE.
Muzzle position is `vehicle.position + vehicle.orientation @ local_offset`.
Launch velocity is exactly `vehicle.velocity + vehicle.forward * muzzle_velocity`.
FIGHTER and VTOL use the same fixed-forward gun, independent of the camera,
engine vectoring, and velocity direction. There is no turret articulation.

Projectiles use constant world velocity in both SPACE and ATMOSPHERE. Atmospheric
rounds deliberately omit gravity, bullet drop and aerodynamic drag for this
milestone. Expiration clips their final motion to their remaining lifetime.
Tracers are short bright lines batched into one reused dynamic VBO, not one
GPU allocation per round; targets share one mesh. Cleanup releases both new
meshes before GLFW context destruction.

### Exact intercept and lead

The fire-control reference is the actual world-space muzzle, with shooter
translation inherited by the projectile. For muzzle position ps, shooter
velocity vs, target position pt, velocity vt, and relative muzzle speed s:

```text
r = pt - ps
v = vt - vs
|r + v*t| = s*t
(v·v - s²)*t² + 2*(r·v)*t + r·r = 0
```

The solver chooses the smallest strictly positive finite root. A scale-relative
near-zero quadratic coefficient uses the linear equation instead. Negative
discriminants reject the solution (only tiny floating-point roundoff is clamped).
Stable quadratic roots use `q = -0.5*(b + copysign(sqrt(discriminant), b))`,
then `q/a` and `c/q`, protecting the near-zero denominator. Invalid vectors,
speeds, coincident positions and nonpositive times return no solution.

```text
intercept_position = pt + vt*t
required_direction = normalize(r + (vt - vs)*t)
projectile_world_velocity = vs + required_direction*s
```

Required direction is the muzzle-aim direction, not simply the direction to
future target position. This distinction preserves correct lead during lateral
shooter drift. The radar computes range, relative vectors, and LOS from the
muzzle. `closure = -dot(relative_velocity, LOS)` is positive for decreasing range.
Beyond 10 km there is no fire-control solution. SHOOT additionally requires:
range <=2000 m, intercept time <=5 s lifetime, and alignment of vehicle.forward
with required_direction within 1.5°. SHOOT never triggers automatic firing.

### Collision and HUD

Collision tests a projectile's swept relative path against each live target's
sphere, expanded by projectile radius. For overlapping time intervals:

```text
relative_start = projectile_start - target_start
relative_end = projectile_end - target_end
```

Testing this segment against a sphere at zero accounts for both target and
projectile motion, preventing tunneling even for crossing targets. New-round
birth times and expiration limit the target's matching motion interval. The
first sphere contact along each round's segment wins, so one round cannot destroy
multiple targets. Expired/hit rounds and dead targets are not rendered.

Normal flight instruments remain intact. New combat symbology is:

- Cyan box: selected target's finite world position projected with homogeneous
  w=1, not a fixed center box. Behind/off-screen targets receive a cyan edge cue.
- Magenta diamond: required gun-aim direction projected with w=0 through camera
  matrices; off-screen aim receives a magenta edge cue. This is distinct from
  the amber flight-path ring (actual vehicle velocity).
- Left target-data area: TGT, RNG meters and signed CLS m/s; radar-range/no-solution
  status appears when relevant. H adds TOF seconds.
- Magenta SHOOT/HIT near the boresight. H also shows target velocity (cyan),
  relative velocity (purple), predicted intercept point/path (magenta), and
  existing flight debug vectors. These are display-only.

For precision gunnery, C selects cockpit: fixed center boresight is exactly
vehicle.forward. Chase preserves its actual green nose-direction cue because
its camera looks down at the craft; align that cue with magenta lead rather
than confusing the chase optical center with gun direction. The gun always
uses the same physical vehicle-forward direction in either camera mode.

### Exact manual combat checks

1. Launch in SPACE, press C for cockpit. Target 01 is initially selected.
   Check cyan box and magenta lead near the center, RNG about 1000 m, SHOOT.
   Hold Spacebar: visible rounds travel forward and destroy it after roughly
   one second. HIT flashes and selection moves to a living target.
2. Press TAB to select target 02 (or cycle until it is selected). Observe lead
   ahead of the crossing target. Maneuver to align center boresight with the
   magenta diamond, not merely the cyan box; hold Spacebar when SHOOT appears.
3. Select vertical/receding targets. Observe velocity-dependent lead and signed
   closure. Outside 2000 m there is no SHOOT cue even with valid radar lead.
4. G to VTOL in SPACE. Hold Left/Right to build significant lateral drift,
   then rotate toward the magenta lead and fire. Rounds must retain lateral
   shooter velocity and still intercept when aim is correct. Transform back
   and confirm flight momentum is unchanged.
5. Put a selected target behind/off-screen; check bounded edge cues rather than
   invalid boxes. H exposes TOF and predicted intercept debug geometry.
6. Hold TAB: only one target change per press. Destroy all targets: TGT --/00,
   no solution/cue; the gun can still fire. Restart to respawn test targets.
7. F2 verifies the same gun can fire in atmosphere; rounds remain simplified
   constant-velocity. Check flight HUD, SAS/HOV, G, C, resize, cleanup and ESC.

Automated combat tests cover stationary/crossing/receding targets, inherited
lateral/approaching shooter velocity, tangent/near-linear/no-solution quadratics,
fire cadence across frame rates, expiry, moving-target swept collision,
nearest-target contact, target selection/key edges, radar range/closure/cue,
projection guards, and end-to-end interception with a laterally moving shooter.
Existing flight/FCC/transformation/SPACE tests remain in the same suite.

## Milestone 10: missile fire control and target management

`game/target_manager.py` owns the available list and current selection. Available
means alive with finite position/velocity and positive radius in this perfect-
information test world; radar/weapon range gating remains separate. Radar
exposes the manager's selection for gun compatibility. TAB cycles once per press
and wraps, excluding destroyed/invalid targets. Destruction refreshes selection
and counts in the same combat update. Index is one-based within the current
available list, not target_id: HUD shows `TGT 03/08` and `ID 17` separately.
An empty list shows `TGT --/00`. T is now unassigned.

New modules separate missile responsibilities:

- `game/missile_fire_control.py`: lock timer, inventory, launch envelope/advice,
  and launch authorization.
- `game/missile.py`: missile state, rocket motor/separation, bounded orientation
  response, force integration, lifetime and path-length removal.
- `game/missile_seeker.py`: assigned-target cone/range checks and track-loss timer.
- `game/missile_guidance.py`: vector proportional-navigation acceleration limits.

CombatSystem owns a collection of active missiles, advances them independently,
checks swept fuses, removes hits/misses and refreshes the manager/radar. A missile
stores its assigned Target reference at launch. TAB never replaces that reference;
several missiles may track different targets simultaneously. They do not switch
victims when the selected target changes or their assigned target is destroyed.

### Current missile parameters

All values live in MissileParameters/MISSILE in `game/missile.py`:

| Parameter | Value |
| --- | --- |
| Inventory | 12 |
| Missile mass | 100 kg |
| Motor thrust | 6000 N (60 m/s²) |
| Motor burn time | 6 s after separation |
| Separation period | 0.2 s |
| Relative launch/ejection speed | 80 m/s along vehicle forward |
| Local hardpoint | (1.2, -0.5, -1.5) m |
| Maximum lifetime | 20 s |
| Maximum integrated travel distance | 10000 m |
| Navigation constant N | 4 |
| Maximum commanded lateral G | 35 G (343.23275 m/s²) |
| Maximum flight/visual turn rate | 90 deg/s |
| Seeker range / half cone | 10000 m / 45° |
| Track-loss grace | 0.8 s |
| Proximity fuse radius | 20 m from the target sphere surface |
| Minimum / effective / maximum acquisition range | 200 / 4000 / 8000 m |
| Nominal engagement speed estimate | 450 m/s |
| Lock half cone / time | 30° / 1.5 s |

M is edge-triggered: a press launches at most one missile, never a salvo. A normal
launch requires a live selected target, established lock and missiles remaining.
Failed attempts consume nothing. Inventory decrements only when a missile is
created. F2 or the existing atmospheric FIGHTER R reset restores inventory and
clears lock for development; it does not recall already launched missiles,
respawn targets, or reload automatically in flight. No target/no inventory
prevents launch. There is no debug override.

Launch position is `vehicle.position + vehicle.orientation @ hardpoint`.
Launch velocity is `vehicle.velocity + vehicle.forward * 80`, preserving all
launcher world-space momentum. Configuration, camera and environment do not
change this inheritance. For 0.2 s, the missile moves without motor thrust,
guidance or sudden attitude changes; the fuse is unarmed. Afterward the motor
burns along missile.forward for six seconds, then produces zero thrust. Velocity
is never assigned to a fixed missile speed. Both environments currently omit
missile gravity and drag; coast is inertial except for available guidance force.
This is a finite-authority prototype, not detailed missile aerodynamics.

### Lock and launch advice

Acquisition measures hardpoint-to-target range and angle against vehicle.forward,
using selected radar information. Range must be within 8000 m and radar range,
and target must stay within the forward 30° half cone for 1.5 uninterrupted
seconds. Leaving range/cone, changing target, or losing the target resets progress.
HUD states are NO TARGET, MSL ACQ, MSL LOCK, and MSL NO RNG. A locked target gains
amber diamond/bracket symbology. None of this points the vehicle or fires for you.

A lock is separate from a good shot. The advised range is:

```text
recession_factor = clamp(1 + closure/450, 0.25, 1.25)
crossing_factor = 1/sqrt(1 + (relative_lateral_speed/450)²)
advised_range = min(8000, 4000 * recession_factor * crossing_factor)
estimated_TOF = hardpoint_range / max(50, 450 + closure)
IN RNG = range >= 200
         and range <= advised_range
         and estimated_TOF <= 19.8
```

Closure sign remains positive when range decreases. Relative lateral speed is
relative velocity with its LOS component removed. This approximate envelope
penalizes recession/crossing geometry; 450 m/s is an estimate only, never a
speed command. Locked NO RNG launches are intentionally allowed for testing,
so poor aspect/long range can miss. Even IN RNG is advice, not a guaranteed hit.

### Exact guidance, seeker and finite maneuverability

For assigned target relative position r, relative velocity v, missile velocity vm:

```text
LOS = r / |r|
LOS_angular_velocity = cross(r, v) / |r|²
closing_speed = max(0, -dot(v, LOS))
requested_acceleration = N * closing_speed * cross(LOS_angular_velocity, vm/|vm|)
limit = min(35 * 9.80665, |vm| * radians(90))
```

Command magnitude is clamped to this limit and is normal to missile velocity.
Zero range/speed safely yields zero command. This PN law suppresses LOS rotation
instead of setting forward to the target's current bearing. It can lead crossing
targets but cannot steer infinitely hard. Guidance plus motor acceleration is
integrated into velocity/position in <=1/120 s steps, split at separation/burnout
boundaries. Missile orientation rotates progressively toward actual velocity,
limited to 90 deg/s and protected at near-zero speed. There is no homing snap,
velocity rotation assignment, fixed-speed enforcement, teleport, or hit guarantee.
The prototype models guidance as a bounded lateral force even during coast.

Seeker feedback is available only if the assigned target is alive, within 10 km,
and within 45° of missile.forward. Invalid geometry pauses PN, allowing inertial
flight/motor thrust without fresh steering. Visibility can recover during the
0.8 s grace period; sustained loss removes the missile and reports MISS. Other
miss conditions are destroyed assigned target, 20 s lifetime, or 10 km integrated
travel. Missiles do not retarget automatically.

### Proximity fuse, visuals and HUD

After separation, CombatSystem sweeps the missile's motion relative to each living
target over the same interval, choosing its earliest proximity contact. Guidance
continues to use only the assigned target; a fuse may trigger on another target
encountered along the physical path. The sphere radius is `20 + target.radius` meters:
20 m clearance to the target surface (45 m center distance for default targets).
This catches high-speed crossings between frames. The armed interval excludes
separation; final lifetime-truncated motion is respected. A fuse hit marks the
target dead, removes the missile, flashes HIT, and refreshes target count/selection.
There is no explosion, area damage, health, armor, or blast simulation.

HUD retains all flight/gun instruments. It adds TGT index/count, separate ID,
MSL inventory, lock state, IN RNG/NO RNG and most-recent-active missile age/TOF.
HIT/MISS appears briefly; it does not show telemetry for every missile. Missile
geometry is a reused small elongated mesh, with an orange exhaust line during
burn. H debug adds missile velocity, target LOS and commanded acceleration to
existing target/flight debug vectors. New buffers use explicit cleanup.

### Exact manual missile procedure

1. Launch in SPACE, use C for cockpit if desired. Use `--enemies 0` to isolate passive-target tests. Confirm ten targets,
   `TGT 01/10`, `ID 01`, `MSL 12`. TAB cycles and wraps once per press; T does nothing.
2. Point at stationary target 01; wait about 1.5 s for MSL LOCK and IN RNG.
   Press M once. Inventory becomes 11; hold M and verify no second launch.
   Watch separation, powered flight, guidance and proximity destruction.
3. Restart for a fresh passive-target set. Lock/launch target 01, immediately TAB
   to crossing target 02, aim within 30°, wait for its independent lock, then
   press M. H shows both missiles tracking their original targets despite TAB.
   Destruction must reduce total count without producing an impossible index.
4. Select crossing/receding targets and inspect closure/range advice. Try a long-
   range locked NO RNG shot: the missile may coast, lose seeker geometry or expire
   instead of receiving a guaranteed hit. A very fast receding target is covered
   by the automated poor-launch test; stock targets move at the documented speeds.
5. Move the vehicle laterally in SPACE VTOL, lock and launch. Verify missile
   inherits drift and turns gradually rather than snapping its velocity toward LOS.
6. Deplete inventory: no new missile at MSL 00. F2 restores prototype inventory
   and clears lock; targets already destroyed remain destroyed. Verify basic
   atmospheric missile operation with the same simplified dynamics.
7. Kill a missile's target with the gun: the assigned missile must expire rather
   than transfer to the newly selected target. Destroy all targets: `TGT --/00`,
   NO TARGET, no authorized launch. Restart to respawn.
8. Verify Spacebar gun, G momentum preservation, SAS/HOV, target/gun lead, camera,
   H debug, resize/minimize/restore and ESC remain functional.

New regressions cover target validity/index/count/wrap, TAB/M key edges, inventory,
continuous lock geometry, separate envelope advice, inherited launch momentum,
separation/burnout/coast, PN G/rate bounds, seeker failure, lifetime, swept fuse,
successful crossing intercept, genuine long-range receding miss, multi-target
ownership/count updates, and frame-rate consistency. Gun/FCC/flight regressions
remain in the full test suite. Enemy AI is described below; countermeasures are not implemented.


## Milestone 11: physical enemy pilots

Default: four hostile FIGHTER aircraft plus the ten existing passive targets.
TAB cycles the combined living list; enemy IDs begin at 11. Hostiles are pink/red
fighter wedges, passive targets remain octahedra. F1 respawns enemies in SPACE;
F2 and atmospheric Fighter R reset respawn them in ATMOSPHERE relative to the
reset player. These development controls discard old enemy weapons/inventories;
ordinary flight and transformation do not. Player kills persist across these
resets; enemy crashes reduce HOSTILES without increasing KILLS.

`EnemyAircraft` conservatively inherits the working vehicle state and owns its
own unchanged `FlightController`, FCC, `AIPilot`, and `CombatSystem`. Aircraft
are integrated once per physics substep; combat receives the actual beginning
and ending target positions for relative swept collision. A read-only live
`PlayerTarget` proxy exposes player kinematics to radar and missiles. Hits report
HUD warnings/counts and consume the weapon, without killing or moving the player.
Dead launchers' existing rounds/missiles continue until normal expiry. Player
weapons destroy hostile aircraft immediately and refresh radar target selection.

AI decisions never assign aircraft position, velocity, orientation or angular
state. Only spawn initialization sets starting motion/attitude. Bounded pilot
commands enter exactly the normal FCC and physics. Absolute desired throttle is
converted to a bounded throttle-rate input; the existing persistent throttle
ramp supplies thrust. No player aerodynamic/control/FCC parameters changed.

FSM priority: DEAD, RECOVER (stall/terrain), EVADE (missile/gun threat), PATROL
(outside detection range), ATTACK (valid gun geometry), PURSUIT. Decisions and
feedback run each physics substep, without a slower tactical scheduler.
Pure pursuit uses current relative position at short range. Lead pursuit reuses
the existing quadratic intercept solver with relative target/shooter velocity
and a pursuit-speed estimate. Invalid solutions fall back to pure pursuit.
ATTACK steers toward the existing gun intercept solution. Overshoots lose firing
geometry, return to PURSUIT and require a real reversal.

Atmospheric pilots roll local-up into the desired turn plane, then pitch toward
a bounded target AoA, with speed-dependent lift trim; yaw only corrects sideslip.
SPACE uses local pitch/yaw direction errors without aerodynamic banking.
World-space momentum stays independent of rotation in both environments.
Energy feedback targets 160 m/s, increases throttle below it and cuts throttle
above 220 m/s. Stall recovery uses full throttle and unloads toward 3 degrees AoA;
recovery persists until speed exceeds 85 m/s and AoA is below 10 degrees.
AoA above the existing warning threshold unloads pitch. Ground avoidance takes
priority below 150 m or when a five-second descending projection crosses that
altitude: roll toward an upward turn plane and pull, still respecting AoA.
Insufficient energy/altitude can produce a physical crash.

Perfect-information missile assignment and threatening player gun geometry
trigger deterministic break turns, varying direction every three seconds with
per-enemy phase variation. No missile changes or guaranteed evasion outcomes.
Gun fire requires the existing radar SHOOT criteria (effective range, valid
intercept, flight time and gun boresight tolerance), in 0.6-second bursts with
0.8-second pauses. Every enemy has the same gun/projectile velocity inheritance
and collision path as the player. Missiles reuse lock acquisition, launch
advice, separation, motor, seeker, PN limits and fuse: AI additionally requires
IN RNG, has four rounds, and waits seven seconds between successful launches.

HUD adds HOSTILES, player KILLS (passive targets excluded), timed GUN/MISSILE HIT
warnings with cumulative player hit count, and MISSILE WARNING while any living
hostile missile retains the player proxy as its assigned target. TGT count stays
separate. H on a selected hostile adds AI state/range/speed/AoA/throttle,
bounded axis commands, desired direction, terrain status and missile threat.
Debug vectors: red forward, cyan velocity, green desired direction, magenta lead.
Hostile and player projectiles/missiles share the batched combat visualization;
the extra enemy mesh uses explicit OpenGL cleanup.

Tuning is centralized in `game/ai_config.py`: count, weapons, speed/energy,
altitude, aggression, firing tolerance, throttle-feedback reaction time, burst timing, inventory,
cooldown and break period. Startup options override count/weapons for testing.
No added dependencies, aircraft physics retuning or later-milestone features.

### Reproducible manual tests

From the project directory, use the existing environment:

```sh
source .venv/bin/activate
python main.py --atmosphere --enemies 1 --ai-weapons none
```

1. **Single unarmed flight:** TAB to ID 11; H shows AI controls/vectors. Follow
   for several minutes using C as needed. Observe bank-and-pull, continuous
   velocity, throttle management, terrain recovery and absence of snapping or
   teleportation. Maneuver into turns to provoke high AoA/recovery. Crashes remain
   possible; F2 starts a fresh enemy. Player controls are unchanged.
2. **Dogfight:** restart with `python main.py --atmosphere --enemies 1 --ai-weapons guns`.
   Let the enemy pursue, fire bursts and overshoot, then get behind it to provoke
   EVADE. Enemy rounds should produce a timed WARNING GUN HIT without ending the
   game. Shoot ID 11 with Spacebar: it disappears, HOSTILES becomes 00, KILLS 01,
   and TAB no longer selects it. Passive kills must not increment KILLS.
3. **Player missile/evasion:** use the same one-enemy gun preset. Aim at ID 11,
   acquire MSL LOCK and IN RNG, press M. H should show EVADE while that missile
   remains alive. Repeat after F2 from crossing/poor geometry, including a locked
   NO RNG shot. Both genuine hits and misses are valid; watch continuous turning
   and inherited momentum, never a predetermined survival outcome.
4. **Enemy missile:** `python main.py --atmosphere --enemies 1 --ai-weapons all`.
   Give the enemy a tail-on/forward launch opportunity at 0.2–4 km for at least
   1.5 seconds. MISSILE WARNING should appear on launch; attempt bank-and-pull
   evasion. Enemy launch cadence cannot exceed one per seven seconds and inventory
   is finite. A hit reports WARNING MISSILE HIT; expired/dead missiles clear warning.
5. **Four enemies:** `python main.py --atmosphere --enemies 4 --ai-weapons all`.
   Confirm HOSTILES 04 and TGT total 14 before destruction. TAB through IDs 11–14,
   inspect differing geometry/states, launch at separate enemies, and verify
   independent weapons/inventories, removal and HOSTILES/KILLS counts.
6. **SPACE:** omit `--atmosphere` or press F1. Observe the enemy rotating during
   drift, 3D pursuit, velocity-inheriting rounds and the unchanged missile model.
   Recheck player throttle/strafe/brake, G, SAS, HOV, camera and resize behavior.

Run regressions with:

```sh
python -m unittest discover -s tests -q
```

Headless development checks used one unarmed atmospheric fighter for three
minutes, then sequential gun-only, missile-capable and four-enemy 35-second
engagements. These check physical trajectories and weapon events, not subjective
handling or successful completion of the above visual playtests.

Validation: 114 automated tests pass, including all 98 pre-existing regressions.
The hidden-window OpenGL smoke attempt aborted before window creation in this
execution session; visual rendering/handling still require manual verification.


## Milestone 12: defensive avionics and countermeasures

The player flight integrator, aerodynamic parameters, thrust, Fighter/VTOL
handling, SAS, hover assist and thrust vectoring are unchanged. Gun ballistics,
lead mathematics, missile motor/PN/turn limits and launch advice are unchanged.
Flight state only gains read-only signal properties. No new dependencies.

`DefensiveAvionics` observes live hostile radar tracks and original missile
assignments; it never changes vehicle or missile motion. RWR distinguishes cyan
TRACK, amber LOCK and red MISSILE markers. Bearing is
`atan2(relative dot right, relative dot forward)`: ahead is at the top, right is
right, behind is bottom, regardless of camera view or world heading. IR approach
warnings also appear on this shared threat compass, without claiming IR radar
emissions. LOCK means a hostile RADAR fire-control lock; IR acquisition does not
produce a radar-lock indication. Incoming count includes assigned missiles still
alive during seeker search grace and drops on lost track/death/expiry/detonation.
TTI is range divided by positive finite relative closure; zero/receding/invalid
closure omits the estimate. It is advice, not an intercept prediction.

Player missiles now have independent **RADAR 6 / IR 6** inventories. 1/2 switches
weapon and clears lock progress; M launches the selected type. Existing `.inventory`
is the total for compatibility; selected type authorizes its own pool only.
F2 / atmospheric Fighter R restore both pools, 30 flares, 30 chaff and ECM OFF.
F1 changes environment without replenishing player stores. Development enemy
respawns retire old enemies/hostile weapons; airborne player missiles retain
original ownership and expire if their original enemy was retired.

Both seeker types use the same physical Missile class. Radar preserves normal
cone/range/timed acquisition. IR requires acquisition cone/range and adequate
heat; it has a 6 km seeker/acquisition range. Aircraft heat is
`0.25 + 1.75 * engine_throttle`; idle engines stay warm. Passive targets have no
IR heat source. There is no thermal, aspect, imaging or real RCS simulation.

L is flare because **F already controls local-down thrust**. B is chaff and J
is ECM. All three are edges: holding a key cannot empty stores or repeatedly
toggle ECM. Packages spawn three meters aft and inherit world velocity plus an
8 m/s aft and 4 m/s local-down ejection. They drift linearly in this lightweight
model. Flares live 5 s with `heat = 12 * exp(-age / 1.5)`; chaff lives 8 s with
`radar_signature = 10 * exp(-age / 2.5)` and radius `1 + 4 * age` meters.
Expired/detonated packages are removed from the shared active collection.
Decoys are never TAB targets or gun collision targets.

Seekers score the original aircraft and live matching expendables using:

```
score = signature * angular_factor / (1 + (range / 1000)^2)
angular_factor = 0.2 + 0.8 * (alignment - cone_cosine) / (1 - cone_cosine)
```

Outside the seeker cone/range, dead sources and insufficient IR heat score zero.
IR considers flares only; RADAR considers chaff only. Existing tracking has 20%
hysteresis to reduce candidate jitter. The strongest valid candidate becomes
`current_seeker_target`; PN then uses that source's position and velocity.
`original_target` and compatibility `.target` retain the launch assignment.
Radar selection never changes. Decoy expiration can cause acquisition of the
original aircraft only if it is still valid inside geometry, or normal search/
track loss. A decoy can physically trigger the existing missile proximity fuse;
aircraft also remain eligible for normal collision even while a decoy is tracked.
No probability rolls, forced trajectory changes or guaranteed survival.

ECM is deterministic and RADAR-only: target ECM halves radar lock progress
(1.5 s becomes 3 s in continuous valid geometry) and multiplies that target's
radar seeker score by 0.55, making competing chaff more attractive. It does not
instantly cancel existing locks or disable missiles. Tradeoff: radar bearing
tracking range expands from 10 km to 15 km for an emitting target. Weapon-quality
range limits still apply. IR acquisition/scores are unaffected; own ECM does not
jam one's own weapon systems.

AI uses the same approach-warning observations, 10 flares and 10 chaff each.
Threats inside 2.5 km with estimated TTI no greater than 8 s (or unknown TTI)
permit type-matched dispensing, with 1.5–1.9 s per-aircraft cooldown. Maneuvering
still uses the existing EVADE/RECOVER physics. Default enemy missile loads are
2 radar / 2 IR, with differing initial selections and automatic switch when a
pool empties. `--ai-ecm` enables ECM on alternating enemies; default is OFF.
Inventories/cooldowns do not guarantee successful timing or survival.

HUD keeps flight and firing instruments, adding selected WPN, RDR/IR counts,
type-specific ACQ/LOCK, FLR/CHF, ECM, inbound count/TTI and the RWR circle. H adds
selected enemy acquisition progress and defensive stores. It also shows one
inbound missile (otherwise newest player missile): TYPE, TRACK AIRCRAFT/FLARE/
CHAFF/LOST, candidate score and original target ID. Debug world lines show a
short seeker cone, cyan original target, magenta current track and candidate
rays (amber viable, gray rejected). Primitive bright flare crosses and expanding
chaff rings use the existing streamed line buffer, with no new GPU resources.

### Exact defensive playtests

```sh
cd "/path/to/project-checkout"
source .venv/bin/activate
```

The `--defense-test` presets create one or two physical hostile missiles at
startup using valid launch geometry and pre-elapsed acquisition. Test launchers
continue flying but do not fire additional weapons. This isolates comparisons;
normal AI launches still require ordinary elapsed acquisition and cooldown.
Restart the same command for each trial; F2 resets into ordinary AI rather than
recreating the preset. Add `--defense-range 2500` for a longer approach. Omit
`--atmosphere` to repeat in SPACE. H enables seeker/debug information.

1. **IR test:** `python main.py --atmosphere --defense-test ir`. First do nothing;
   observe warning count/TTI and a possible hit. Restart separately for maneuver
   only; L without maneuver; and a break using A/D then S plus L. Compare actual
   TRACK FLARE with continued missile motion. Repeat at 2500 m with L immediately
   (early flare decay), then at 1000 m dispensing very late. Survival is not
   guaranteed; the early/late cases are also automated. Press B instead of L:
   IR must not track CHAFF. Holding L consumes only one package.
2. **Radar test:** `python main.py --atmosphere --defense-test radar`. Compare no
   action, L, B alone, and break + B. RADAR must ignore FLARE; viable chaff should
   permit TRACK CHAFF without changing original ID. Observe normal fuse/expiry
   and warning clearing. Repeat early/late and different bearings/ranges.
3. **ECM acquisition:** `python main.py --atmosphere --enemies 1 --ai-weapons all`.
   TAB to ID 11 and H. Let it establish valid radar geometry; note its displayed
   acquisition progression with ECM OFF. Restart identically and press J before
   acquisition: valid radar progress should be half-speed and still reach LOCK.
   Turn ECM on after lock: it must not instantly remove that lock or destroy an
   airborne missile. Repeat with IR geometry (or select 2 against ID 12 using
   `python main.py --atmosphere --enemies 2 --ai-weapons all --ai-ecm`): IR acquisition remains 1.5 s in valid heat geometry.
   The automated controlled comparison removes differences caused by maneuvering.
4. **Mixed threats:** `python main.py --atmosphere --defense-test mixed`. Initially
   MISSILE WARNING 2 should appear. H inspects one live inbound missile. Deploy
   L and B separately while maneuvering: each candidate type should influence
   only its matching seeker. One missile may remain dangerous after the other is
   lost/detonated; count must become 1 then 0 as threats resolve.
5. **Bearing:** use the mixed preset (threats initially behind, one slightly right).
   RWR should agree. Roll/yaw/pitch the player and observe local-frame marker
   changes; C must not change the vehicle-relative bearings.
6. **Regression/multi-enemy:** run the normal four-enemy preset; test TAB while
   missiles track decoys, finite AI countermeasures, guns ignoring decoys, F2
   replenishment, and ordinary flight/SAS/HOV/VTOL/camera controls. Battledroid is described below; later systems remain unimplemented.

Validation uses `python -m unittest discover -s tests -q`. New tests cover
inventories/edges, inherited velocity, decay, geometric candidate competition,
wrong types, original ownership, loss/reacquisition, ECM/IR separation, bearings,
TTI, multiple-threat counts, AI cooldowns, reset/removal, early/late failure,
HUD finiteness and mocked streamed rendering/idempotent cleanup. Visual OpenGL
playtests still require a local display; mocked rendering is not a visual check.

Milestone 12 validation: **140 automated tests pass**, including all 114 existing
regressions. Headless scenarios verified correct transfers, wrong-type failures
and early/late flare failure. Local visual playtesting remains outstanding.


## Milestone 13: Battledroid

G now cycles **FIGHTER → VTOL → BATTLEDROID → FIGHTER**. It remains edge-triggered.
This is the same PlayerVehicle, FCC, combat system, radar selection and stores:
transformation itself changes no position, world velocity, body basis, angular
rates, throttle, environment or weapon/countermeasure state. Leaving grounded
Battledroid changes GROUNDED to FLYING so its old support cannot glue the new
configuration down. It does not launch it or add an impulse.

`engine/battledroid_controller.py` and `game/battledroid_physics.py` isolate the new
handling. Existing FlightController/force/FCC entry points dispatch Battledroid to
those paths; Fighter and VTOL integration, parameters, SAS, hover calculations,
AI tactics, ballistics, radar/lead, missiles and seeker/defensive logic retain
existing behavior. Three old tests' expectations were updated for the required
three-mode cycle, implemented Battledroid and extended hover eligibility.

The primitive humanoid has torso, head, two arms, two legs, feet and backpack,
without articulation/animation. Vehicle position is its center-of-mass reference.
The model is 5.4 m tall, soles 3.2 m below COM and head 2.2 m above it when upright.
A conservative oriented body support box supplies the ground clearance while
banked/tipped; this is body/sole contact, not individual-foot or slope simulation.
Chase view uses 12 m aft / 4 m up for the taller configuration. Cockpit remains
the approximate existing forward view. The new mesh uses explicit, idempotent
OpenGL cleanup.

### Airborne controls and forces

In SPACE, W/S pitch, A/D roll, Q/E yaw, Up/Down persistent forward throttle,
Left/Right local strafe, R/F local vertical thrust and X inertial brake apply.
Battledroid uses its own bounded attitude/translation rates and the existing kind
of temporary inertial damping. There is no aerodynamic force, gravity or
world-up correction. SAS damps released angular rates only.

Atmospheric Battledroid also retains attitude and translation controls. Its main
throttle thrust points along **local up**, without using VTOL thrust vector.
It has gravity, quadratic body drag (CD 1.0, area 30 m²) and zero wing lift. Total
engine plus maneuvering thrust is capped at **1.8 times vehicle weight**. Incoming
150–250 m/s momentum is retained at transformation, then this drag reduces speed
through forces. No speed clamp, protected landing or anti-gravity exists.
Z/X vector commands remain VTOL-only; X brakes Battledroid only in SPACE.

Upright zero-speed manual hover is approximately **56% throttle**. V toggles
vertical-velocity hover feedback only while airborne in ATMOSPHERE. It computes
bounded engine throttle through the existing conceptual FCC override/grace
mechanism; it never sets altitude, position or velocity. R/F and throttle inputs
retain manual control and a brief override grace. Inverted/insufficient-thrust
hover can fail. Ground contact clears airborne hover assistance.

### Contact, walking and jumping

Ground remains Y=0. `FlightStatus.GROUNDED` is actual Battledroid contact;
`is_grounded` also requires ATMOSPHERE and BATTLEDROID. Airborne uses FLYING status.
Supported contact is checked geometrically each step. A replaceable flat ground
height query may return None to remove support for future edges; no terrain is
implemented here.

Contact corrects normal penetration only and applies a zero-restitution normal
constraint: downward velocity becomes zero, upward departure is retained and
horizontal velocity is untouched at impact. Supported contact supplies weight
reaction. Tangential aerodynamic drag, ground friction/traction and any thruster
force subsequently change horizontal motion. Large incoming horizontal speed
can therefore skid. Normal/penetration corrections are contact resolution,
not a transformation or locomotion position assignment.

A landing is considered hard above **7 m/s downward** or beyond **45 degrees**
from upright, with a three-second HARD LANDING indication. Downward impact above
35 m/s becomes CRASHED (prototype frozen physics; F2 resets). There is no bounce
or damage model. Thresholds and all new handling values are in the immutable
`BATTLEDROID` parameters in `game/battledroid_physics.py`.

Only when Battledroid is grounded:

| Key | Action |
| --- | --- |
| W / S | Walk forward / backward |
| A / D | Walk left / right |
| Q / E | Turn left / right |
| K | One short thruster jump per press |

Walking projects the body heading onto the ground, generates desired tangential
velocity, then applies bounded traction acceleration. Forward/reverse/strafe
speeds are 6/3/4 m/s; diagonal speed is bounded. Movement acceleration is 3 m/s²,
released-control friction/deceleration is bounded at 4 m/s². No walking code
writes position. The integrator advances acceleration → velocity → position.
Grounded turning has bounded 45°/s rate and 90°/s² angular acceleration. Support
logic rotates body-up toward the ground normal at a bounded rate, never snapping
upright; this correction is never active while airborne or in SPACE.

K was unused; **R remains vertical thrust/reset and Spacebar remains gun**.
K commands full bounded thruster force for 0.5 s from supported contact. It does
not set velocity/altitude or prescribe a jump arc. Contact breaks when upward
forces overcome weight; horizontal momentum remains. With throttle zero, gravity
then returns the robot to normal landing/contact. Persistent throttle above
hover can keep it aloft after the burst. Transforming out of Battledroid cancels
that configuration's remaining jump burst.

HUD adds MODE BATTLEDROID, GROUND/AIRBORNE, WALK speed when supported and HARD LANDING.
The flight-path marker still shows actual velocity, and naturally hides below
its existing low-speed threshold. Artificial horizon/combat/defensive instruments
remain. H shows grounded state, contact depth/normal, vertical/horizontal speed,
traction/friction force, thruster force, desired walking velocity and actual
horizontal velocity. Weapons remain tied to body forward and inherit the same
world-space motion while walking, falling, hovering or drifting. L/B/J and 1/2/M,
TAB and Spacebar retain their defensive/weapon functions. Enemy AI stays Fighter.

### Exact Battledroid manual tests

Activate the existing environment from the project directory:

```sh
cd "/path/to/project-checkout"
source .venv/bin/activate
```

Startup-only `--battledroid-test ground|landing|skid` presets set reproducible initial
conditions; they are not a teleport/control mechanic during play. F2 remains the
existing Fighter test-flight reset. Use `--enemies 0` to isolate flight/contact.

1. **SPACE transformation:** `python main.py --enemies 0`. Accelerate using Up,
   reduce throttle with Down, yaw until nose and velocity marker differ. Press G
   twice to Battledroid, checking speed/trajectory continuity at each transition.
   Roll upside-down and release controls: SAS must not level it. Try local
   translations, X braking and weapons. G again returns Fighter without reset.
2. **Atmospheric transformation/landing:** `python main.py --atmosphere --enemies 0`.
   Accelerate in Fighter to 150–200 m/s, G twice, observe immediate momentum
   preservation then drag. Keep local up near world up; increase throttle toward
   56% to arrest descent, or V to test bounded hover. Turn V off to descend
   manually, trim throttle below hover and approach the ground at less than
   7 m/s downward. Observe GROUND. F2 resets; it does not preserve stores.
   For a near-ground starting point use `python main.py --enemies 0 --battledroid-test landing`;
   this initializes 50% thrust and 2 m/s descent from 12 m, producing a physical
   powered landing. V or R can arrest descent; lowering throttle can make it hard.
3. **Walking:** `python main.py --enemies 0 --battledroid-test ground`. Confirm GROUND,
   throttle zero, H debug. Test W/S, A/D, Q/E, diagonals and release deceleration.
   Spacebar still fires; K is jump. Up above hover should lift off and restore
   airborne pitch/roll input. G releases the ground constraint into Fighter;
   insufficient thrust may subsequently cause normal ground impact.
4. **Skid/hard landing:** `python main.py --enemies 0 --battledroid-test skid`.
   Initial velocity is 40 m/s sideways and 2 m/s downward from 3.7 m COM altitude.
   Release controls. Horizontal momentum must survive touchdown and decay after
   it; H shows forces/depth. Use the landing preset with low throttle and a longer
   descent for HARD LANDING; extreme descent may crash. Check no dramatic bounce.
5. **Jump:** use the ground preset, throttle zero and hover OFF. Press K once;
   hold K to confirm no repeated jumps. Observe thrust, AIRBORNE, gravity and
   landing. Repeat while walking sideways: momentum remains and it lands away
   from its departure point. Do not expect return to the original position.
6. **Battledroid combat:** use the ground preset. TAB to a passive target ahead, walk
   laterally, inspect lead/SHOOT and hold Spacebar. Select 1, acquire RADAR lock,
   M launches an inherited-velocity missile. Use K or thrusters to repeat while
   falling/hovering. L/B/J remain available. For active opponents omit `--enemies 0`;
   Fighter AI should attack normally without transforming into Battledroid.
   Recheck camera, debug, resize, and Fighter/VTOL handling.

Automated checks run with `python -m unittest discover -s tests -q`. Battledroid tests
cover transformation/state preservation, free inverted SPACE attitude, local
forces, zero lift/gravity/drag/thrust bounds, manual/FCC hover, penetration and
normal impulse, retained tangential momentum, friction, pure bounded walking,
angular stabilization/turning, jumps/landing/crash, loss of support, releasing
configuration constraints, inherited weapon/decoy motion, keys, HUD/model/cleanup
and frame-rate consistency. Local visual handling still requires manual testing.
No Battledroid AI, animation, articulated aiming, terrain or later milestone work.

Milestone 13 validation: **162 automated tests pass**. Headless powered-landing,
skid and walking scenarios also reached stable contact through normal physics.
Visible OpenGL rendering and subjective handling remain manual playtests.


## Milestone 14: hierarchical GLB models

The normal game remains keyboard-only and uses the same primitive vehicles,
physics, controls and combat. No dependencies or shader changes were added.
A CPU-only glTF 2.0 GLB reader preserves node hierarchy, TRS/quaternions or node
matrices, shared mesh references, normals/UVs and basic material factors. It
validates offsets/stride/indices and rejects unsupported features clearly.
A single +Z-to--Z model-root Y rotation establishes asset-facing conversion;
units remain meters. The GPU wrapper reuses the existing Mesh/shader/cleanup
lifecycle and uploads once per unique primitive, never per frame.

See [model pipeline documentation](docs/MODELS.md) for the supported subset,
lookup/runtime override/reset APIs, resource ownership, conversion and future
Blender export conventions.

```sh
source .venv/bin/activate
python main.py --enemies 0 --model-test
python main.py --inspect-model assets/models/test/hierarchy_test.glb
python -m unittest discover -s tests -q
```

The original tiny test GLB has named body, wings, leg pivots and parented feet.
The opt-in automatic 16-second exercise demonstrates parent articulation, child
articulation, whole-root motion and exact pose reset beside the unchanged craft.
Move/rotate the player with existing keys to verify vehicle-root composition.
Repeat launch/ESC to check cleanup. A supported custom file can be displayed
using `--model-test --model-path /absolute/file.glb` (rest pose only).
Packaged asset paths work independently of shell working directory.

Manual acceptance: also run normal SPACE/ATMOSPHERE, G through all three modes,
weapons, AI and defensive controls. Automated CPU/mocked GPU checks do not replace
visual rendering or handling playtests. No M15 aircraft or animation system has
been implemented.

Milestone 14 validation: **184 tests pass**, including all 162 unchanged
M1–M13 regression tests. CPU hierarchy/articulation and mocked GPU reuse/cleanup
checks passed. The real OpenGL smoke run aborted during GLFW initialization
before opening a window; local visual acceptance remains outstanding.

## Milestone 15 — original prototype Fighter

The player Fighter now uses an original meter-scale twin-engine GLB with separate
wing, intake, leg, arm, head and torso joints. VTOL, Battledroid and enemy visuals
remain unchanged. Fighter chase framing is 24 m behind and 7 m above; flight,
combat, collision and keyboard controls are unchanged.

See [prototype design, joints and inspection](docs/PROTOTYPE.md). Run
`python main.py --enemies 0 --model-test --prototype-test` for the automatic pivot
exercise, or `python main.py --enemies 0` for normal flight. The original M14
`--model-test` remains available. Reproduce the committed asset with
`python tools/generate_tc167_prototype.py`. No dependencies were added.

Validation: **192 tests pass**, including all M1–M14 tests. A real OpenGL smoke
test passed three upload/draw/delete cycles and all prototype pivot phases, with
no GL errors or surviving model buffer handles. Interactive flight and combat
acceptance remain pending. M16 transformation is not implemented.

## Milestone 16 — continuous transformation

G now unfolds the same approved airframe through Fighter, VTOL and Battledroid.
Fighter/VTOL transitions take 1.5 s; VTOL/Battledroid take 1.9 s. Battledroid's G
return chains through VTOL. Press G while transforming to reverse smoothly.
Physics switches at 60% of each edge; motion, throttle and combat continue.
The HUD displays the source/target and transformation percentage.

Run `python main.py --model-test --enemies 0` for the automatic forward/reverse
cycle. See [transformation timing, validation and limitations](docs/TRANSFORMATION.md).
All **214 tests pass**; real OpenGL endpoint/transition/reversal and repeated GPU
cleanup checks passed. Interactive flight/combat acceptance remains for local
playtesting. The approved M15 GLB is unchanged. No M17 gait animation is included.

## Milestone 16.1 — chase orbit inspection

Gameplay remains keyboard-controlled. **0** selects CHASE or DOLLY. In DOLLY, **left click + drag** (including
MacBook trackpad click-drag) orbits the observing camera; **wheel/two-finger
scroll** zooms; **HOME / BACKSPACE** resets the rear orbit view while staying
in DOLLY. C still toggles external/COCKPIT and remembers CHASE or DOLLY. Mouse input does not control flight, walking, aiming, weapons or
transformation, and is inactive in COCKPIT. No cursor capture is used.

Yaw is unlimited; pitch clamps to ±80°. Zoom is 8–80 m from the vehicle root.
Manual angles remain fixed in the reference frame captured when DOLLY begins,
so vehicle rotation does not pull the view behind it. The camera tracks root
translation directly while smoothing angle/zoom changes. Manual distance stays
fixed across transformations until HOME; C restores the previous orbit on return.
In CHASE, mouse orbit/zoom is inactive and the existing vehicle-following
framing and bank behavior are unchanged. Press 0 to return to that behavior.

Orbit works in normal play and `--model-test`. In the latter, it observes the
player/root; the separate inspection copy still sits beside that root. To inspect
the actual transforming player, run `python main.py --enemies 0` and use G while
orbiting. BACKSPACE is an unused reset alias for compact keyboards without HOME.

Validation: **225 tests pass**, including all prior regression tests. Callback
checks cover drag, release/re-click, scrolling, HOME edges and cockpit isolation.
Real OpenGL orbit-view checks use the existing hidden-window validation tool.
Local trackpad gesture/feel acceptance remains to be checked. Transformation
poses, the approved GLB and gameplay physics are unchanged; M17 is not started.

## Milestone 16.2 — Battledroid endpoint refinement

Battledroid now lowers the complete intake/leg parents into the hip region, narrows
the shoulders, bends the arms and folds full-size wings flat into a backpack.
The horizontal tails fold inward too. Fighter, VTOL, orbit controls and
physics/combat remain unchanged. Exhaust-node caches are refreshed after engine
pose overrides; no player exhaust effect system was added.

`--model-test` now holds Battledroid for seven seconds in a 25.8-second cycle.
See [exact pose changes and compromises](docs/TRANSFORMATION.md#milestone-162-endpoint-refinement).
All **232 tests pass**; real OpenGL orbit/transition/cleanup checks pass.

## Milestone 16.3 — Battledroid leg extension

Battledroid uses the existing full-length leg links, with a 4° thigh lean, 8° knee
bend and level feet. Hip positions relative to the body remain unchanged.
Intake-shell compression is confined to the shell meshes rather than inherited
by the leg chain. A visual root offset preserves the existing ground-contact
datum while increasing standing height to approximately 10.74 m.

Leg extension occurs late in the VTOL-to-Battledroid transition and reverses
continuously. Approved Fighter/VTOL poses, physics, camera, controls and combat
remain unchanged. See [exact joint changes](docs/TRANSFORMATION.md#milestone-163-leg-extension)
and the [updated stance preview](docs/transformation_battledroid.png).

All **235 tests pass**. Hidden-window OpenGL endpoint, transformation, reversal,
orbit and repeated resource-cleanup checks pass. Manually inspect front/side
views, land and walk, and check momentum through transformations in both
environments. M17 is not implemented.

## Milestone 16.4 — Battledroid arm correction

Battledroid upper arms now descend beside the torso, with a forward elbow bend
and hands following the forearms near hip height. Both elbows use local X=-20°;
upper arms use X=-90° and wrists use identity rotation. The approved M16.3 leg
stance and all non-arm poses are unchanged. See the
[axis diagnosis and exact angles](docs/TRANSFORMATION.md#milestone-164-arm-pose-correction).
All **237 tests pass**, along with OpenGL transformation/reversal/orbit/cleanup
checks. No walking animation was added.

## Milestone 17 — Battledroid procedural locomotion

Grounded Battledroid now layers speed-dependent walk/run, backward and lateral
steps, opposing arm swing, modest foot motion and turn steps over its approved
standing pose. Input-free sliding braces into LOC SKID; takeoff and transformation
fade the gait out. H shows locomotion debug state. SPACE drift does not trigger
walking. Animation never drives physical motion.

Start directly on the ground:

```sh
.venv/bin/python main.py --enemies 0 --battledroid-test ground
```

Add `--no-locomotion-animation` to compare the original physical motion. Existing
ground controls remain W/S forward/back, A/D strafe, Q/E yaw, K jump and G
transformation. Use the unchanged orbit camera to inspect side/front views.

All **253 tests pass**, including exact enabled/disabled physical trajectories.
Scripted OpenGL locomotion/transformation/orbit/cleanup checks pass. See
[architecture, tuning, validation and manual checks](docs/LOCOMOTION.md). Foot
sliding/contact remains approximate without IK. No later milestone was started.

## Explicit CHASE / DOLLY camera modes

Start in CHASE: the existing camera follows vehicle position, turns, pitch and
bank continuously. **0** toggles CHASE/DOLLY once per key press. DOLLY starts
from the current chase offset, follows vehicle position and preserves its
observation angle independently of vehicle rotation.

Mouse/trackpad click-drag and scroll operate only in DOLLY. **C** switches
external/cockpit, preserving the selected external mode. **HOME / Fn+Left /
BACKSPACE** resets DOLLY angle/zoom while remaining in DOLLY. Returning to CHASE
requires only **0**. The title shows CHASE, DOLLY or COCKPIT.

Camera correction validation: **257 tests pass**, including key hold/release
edges, continuous chase after subsequent turns, no positional jump into DOLLY,
position-only dolly following, cockpit restoration and camera-only isolation.
Existing OpenGL orbit/locomotion/transformation/cleanup checks pass. Native
MacBook trackpad feel remains a local manual check. M17 animation, vehicle
physics, transformation and models are unchanged.

## Milestone 18 — VTOL landing and ground hover

VTOL can land on its feet, rest under ground support, skid under existing
vectored/strafe thrust, yaw and lift off naturally. Approved airborne forces and
controls remain unchanged. Touchdown preserves horizontal momentum; moderate
load-dependent friction slows sliding. There is no VTOL walking animation.

Upright sole clearance is approximately 5.926 m. Safe descent is up to 3 m/s;
harder contact warns, while descent above 18 m/s, extreme tilt or horizontal
impact can crash. The HUD shows AIRBORNE/LANDING/LANDED/CRASHED and hard landings.
H enables foot/support debug values. Existing V hover assist coexists with contact.
Grounded VTOL/Battledroid transformations preserve momentum and settle through
a shared changing contact envelope. Fighter ground behavior is unchanged.

```sh
.venv/bin/python main.py --enemies 0 --vtol-test landing
.venv/bin/python main.py --enemies 0 --vtol-test ground
.venv/bin/python main.py --enemies 0 --vtol-test skim
.venv/bin/python main.py --enemies 0 --vtol-test hard
.venv/bin/python main.py --enemies 0 --vtol-test crash
```

Use unchanged throttle/vector/strafe controls to skim and take off, Q/E to yaw,
V for hover assist and G to transform. Select DOLLY with 0 for foot inspection.
All **273 tests pass**. Scripted pre-M18 trajectory comparisons and OpenGL ground,
transformation, orbit and cleanup checks pass. See
[contact architecture, tuning and manual acceptance](docs/VTOL_GROUND.md).
Selected custom model edits are preserved. M17 animation is unchanged; M19 is
not implemented.
