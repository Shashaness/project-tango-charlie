# Milestone 18 — VTOL landing and ground hover

VTOL retains its approved airborne forces, integration and controls. M18 adds
foot contact and load-dependent ground forces to that same integrator. There is
no walking, root motion, ground movement command remapping or new flight tuning.
The currently selected user-modified GLB is preserved without regeneration.

## Reused contact architecture

The existing GroundContact record and FLYING/GROUNDED/CRASHED states remain
shared with Battledroid. `game/ground_contact.py` extracts Battledroid's penetration
correction and zero-restitution normal impulse. Battledroid keeps its original
forces, friction, control rates and thresholds. The shared helper permits
VTOL-specific impact limits without introducing a second movement integrator.

`engine/vtol_ground.py` adds unilateral support and friction to the initial
and midpoint force samples already used by FlightController. Position still
comes from velocity integration; only actual penetration is corrected. A normal
collision impulse removes downward vertical velocity, never horizontal momentum.
No force changes are made away from contact or in SPACE.

Internally AIRBORNE is FLYING, LANDED is GROUNDED. LANDING is an informational
approach flag (descending within 2 m of the contact envelope), without forces.
HARD LANDING is a transient warning; catastrophic contact is CRASHED. The normal
VTOL HUD shows AIRBORNE/LANDING/LANDED/CRASHED. H adds foot heights, clearance,
penetration, support and friction. Existing Battledroid debug and gait remain intact.

## Geometry and tuning

The actual foot nodes are LeftFoot and RightFoot. A cached, immutable set of mesh
vertices is extracted from their descendants in the canonical VTOL pose,
including hierarchy transforms and GLB-to-vehicle conversion. Physical clearance
uses their lowest world-up projection under the current vehicle orientation;
it never reads M17 animation. Upright foot clearance is approximately **5.925660 m**
from vehicle root to sole. A 0.8 m body minimum catches inverted/non-foot impacts.

All contact tuning lives in `game/vtol_ground.py`, separate from the approved
`game/vtol_physics.py` flight parameters.

| Parameter | Value |
|---|---|
| Safe descent speed | ≤3 m/s |
| Hard warning | above 3 m/s or tilt above 25° |
| Stronger hard warning | ≥7 m/s: 5 s instead of 3 s |
| Catastrophic descent | >18 m/s |
| Catastrophic attitude | >75° from upright |
| Catastrophic horizontal impact | >100 m/s |
| Contact tolerance | 1 mm |
| Liftoff separation tolerance | 2 cm |
| Approach indication height | 2 m |
| Dynamic/static friction coefficients | 0.12 / 0.25 |
| Sliding velocity damping cap | 0.6 /s |
| Static-speed region | below 0.03 m/s |
| Restitution | zero |

Normal support supplies the negative vertical remainder of gravity, aerodynamic
forces and thrust. Increasing upward thrust unloads the feet and reduces friction.
If upward acceleration is positive, the original integrator raises the craft;
contact clears after separation exceeds 2 cm. Tiny force residuals below 1e-6 m/s²
are treated as numerical equilibrium while supported. No takeoff key is needed.

Sliding friction opposes horizontal velocity, bounded by loaded normal force,
viscous damping and the available momentum per substep. Near zero speed, a capped
static-friction force resists thrust and damps tiny drift. Release causes gradual
deceleration; horizontal velocity is never clamped to zero by contact.

Yaw and thrust-driven skimming use the existing Q/E, throttle, vector and strafe
commands. Pitch/roll remain attitude controls. No automatic ground attitude
leveling or Battledroid-like desired walking velocity was added.

## Hover assist and transformation

The approved hover calculation is unchanged. Eligibility now includes landed
VTOL, allowing the existing vertical-velocity feedback to coexist with support.
Existing manual throttle/vector/lift override behavior still permits descent or
liftoff. Near-ground hover does not become LANDED without foot contact. Battledroid
hover eligibility remains unchanged.

Grounded G/B mode switches retain support status. During their existing staged
transformation, a smooth contact envelope interpolates canonical VTOL foot
clearance and Battledroid's existing 3.2 m clearance. Both controllers use that
same envelope while transformation is active. Expanding geometry receives normal
penetration correction; contracting geometry settles under gravity. Switching
configuration itself never changes world position, velocity, orientation or
combat state. The root height consequently changes gradually as contact resolves
the differing physical shapes. There is no single configuration-switch teleport
or horizontal momentum reset. Some visual contact mismatch during intermediate
folding remains possible; this is not foot IK or a full animated collision hull.
Leaving for Fighter retains its original ground/crash behavior.

## Reproduction and acceptance

From the project root:

```sh
.venv/bin/python main.py --enemies 0 --vtol-test landing
.venv/bin/python main.py --enemies 0 --vtol-test ground
.venv/bin/python main.py --enemies 0 --vtol-test skim
.venv/bin/python main.py --enemies 0 --vtol-test hard
.venv/bin/python main.py --enemies 0 --vtol-test crash
```

Landing starts 3 m above the feet datum, descending at 1 m/s with vertical thrust
approximately balancing weight. Ground starts rested with throttle zero. Skim
starts with 8 m/s lateral momentum. Hard/crash start with -8/-25 m/s vertical
velocity. Presets reset contact history, then run ordinary gameplay physics.

Controls are unchanged: Up/Down throttle, X/Z vector adjustment, Left/Right
lateral thrust, Q/E yaw, W/S pitch, A/D roll, R/F maneuvering lift, V hover assist,
G transformation, C cockpit/external, 0 CHASE/DOLLY. In DOLLY use trackpad
click-drag and two-finger scroll to inspect feet and the side silhouette.

Manually check gentle landing, sustained rest, loaded lateral/forward skimming,
release deceleration, yaw, throttle liftoff and touch-and-go. Try hover assist
followed by commanded descent, both grounded transformation directions, then
Battledroid walking and jumping. Check hard/crash presets and high-altitude VTOL
maneuvering in both environments. No interactive feel approval is claimed here.

## Validation and limits

**273 tests pass**, including all prior regressions and new landing, thresholds,
foot clearance, support/loading, friction/no-reversal, liftoff, approach HUD,
rest/hover stability, intentional descent, skimming/yaw, SPACE exclusion,
ground transformation/momentum, Fighter isolation, touch-and-go, fresh exhaust
hierarchy and reproducible preset checks. M17 animation code is unchanged.

Scripted comparisons with pre-M18 controller code produce exactly matching
position/velocity/orientation for all configurations in SPACE and airborne
ATMOSPHERE. An additional Battledroid ground walk/strafe/yaw/release/jump comparison
matches at every sample. Existing airborne regression tests remain green.

The OpenGL check supports `--vtol-ground` and captures landing, rest, skimming,
hard/crash contact, liftoff and both grounded transformations. Three uploads,
endpoint/orbit/reversal rendering and explicit GPU cleanup pass with no GL errors.
Captured feet/contact, skim, liftoff and transformation endpoints were inspected.
These are scripted render checks; local keyboard/trackpad handling remains a
manual acceptance task.

A few previous tests assumed the original generated asset's mesh ordering,
exhaust offsets or foot spacing. They now validate the selected custom asset's
actual hierarchy and marker transforms; generator reproducibility still checks
the original generated binary. The custom model selection and both binaries
were left untouched.

Limits: flat world-up ground only, approximate support envelope during
transformation, no IK, suspension, terrain or damage. There is no VTOL gait.
Thresholds and friction require gameplay feel feedback. M19 is not implemented.
