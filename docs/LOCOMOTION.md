# Milestone 17 — Battledroid procedural locomotion

The approved M16.4 Battledroid stance remains the configuration pose. Animation
reads existing physical motion and never writes position, velocity, orientation,
throttle, contact state or physical gait phase. No root motion or physics tuning
was introduced. The original GLB, Fighter/VTOL tables and camera are unchanged.

## Ownership and layering

`game/battledroid_locomotion.py` contains `BattledroidLocomotionAnimator`. Each existing
game simulation substep clears the previous visual deltas, advances M16
transformation and existing flight physics, then samples the resulting state.
The animator receives dt, a read-only vehicle sample and current movement input.
It owns its own phase, amplitude and local matrices, separate from M13 walk_phase.

Actual animated nodes are LeftUpperLeg, RightUpperLeg, LeftLowerLeg,
RightLowerLeg, LeftFoot, RightFoot, LeftUpperArm, RightUpperArm and TorsoCore.
The hip pivots remain below LeftIntake/RightIntake. Shoulder positions, elbows
(LeftForearm/RightForearm) and hands stay at their approved transforms and inherit
upper-arm swing. Head and backpack remain stable without root bob.

Effective node transform is imported/configuration matrix multiplied by a local
animation delta. `ModelNode.set_animation_delta()` retains the existing pose
instead of discarding it. Procedural degree values generate quaternions each
sample; no Euler-angle accumulation occurs. The canonical pose tables are never
modified. At zero blend all deltas are identity, recovering exact M16 matrices.
Engines are root siblings, so their visual layer follows each ankle's relative
world change once; exhaust descendant matrices are refreshed afterward.

## Selection and tuning

Ground gait requires completed Battledroid configuration, ATMOSPHERE and GROUNDED.
World horizontal velocity is projected onto the normalized horizontal body
forward vector and its right vector, matching the existing ground controller.
Input supplies only whether locomotion is commanded; direction and state speed
come from actual motion, including residual momentum.

| Parameter | Current value |
|---|---|
| IDLE | speed below 0.3 m/s |
| WALK | commanded speed 0.3–4.5 m/s |
| RUN | commanded speed above 4.5 m/s |
| Run blend | continuous across 3.5–6 m/s |
| Cadence | 0.35–0.75 walk; up to 1.05 run cycles/s |
| Hip fore/aft swing | ±20° walk, up to ±28° run |
| Lateral hip swing | at most ±6° |
| Additional knee flexion | 0–25° walk, up to 38° run |
| Ankle motion | up to ±6°, scaled by forward contribution |
| Opposing arm swing | ±12° walk, up to ±18° run |
| Lateral arm motion | 35% of arm swing |
| Torso counter-yaw | up to ±1.5° |
| Amplitude ramp | 5 units/s: 0.2 s from zero to full |
| Turn stepping | above 3°/s actual yaw, below idle translation speed |
| Turn cadence / hip swing | 0.4 cycles/s; ±7° fore/aft |

All tuning constants are centralized in the animator module. The heavy-machine
cycle is bounded even at extreme speed. Left and right phases differ by π.
Knees flex during recovery and remain extended during the support half-cycle.
Elbows retain their corrected local X=-20° bend throughout.

Signed forward velocity reverses stride direction without discontinuously
resetting phase. Lateral velocity drives a shared local-Y leg weight shift while recovery knees
alternate, preserving clearance instead of pulling both feet inward. Direction
components are normalized and blended continuously for diagonal motion; lateral
amplitude is restricted to preserve foot separation. Turning in place uses a
small alternating counter-step approximation, rather than a separate gait rig.

When movement commands are released but speed exceeds 0.3 m/s, LOC SKID freezes
phase and fades offsets into the neutral braced stance. Airborne gait likewise
freezes and fades; SPACE inertial motion never starts a grounded cycle. Touchdown
ramps into movement or remains idle according to physical speed and command.

On transformation away from Battledroid, the frozen last offsets fade over the
ongoing canonical interpolation. They are cleared by 0.2 seconds and cannot
persist into VTOL. Reverse continues using M16's exact curves. Completion
into Battledroid exposes the exact base pose with zero locomotion amplitude before
starting a new ramp. Transformations and physical mode switches are not delayed.

## Debug and reproduction

H enables the existing debug display, now including LOC IDLE/WALK/RUN/SKID/TURN,
forward/lateral speeds, phase, cycles/s and blend. No new gameplay key is used.
`--no-locomotion-animation` disables the visual layer only.

From the project directory:

```sh
.venv/bin/python main.py --enemies 0 --battledroid-test ground
.venv/bin/python main.py --enemies 0 --battledroid-test ground --no-locomotion-animation
.venv/bin/python main.py --enemies 0 --battledroid-test skid
.venv/bin/python main.py --enemies 0 --battledroid-test landing
```

Use W/S for ground forward/back, A/D for strafe, Q/E for body yaw, K to jump,
G to transform/reverse, F1 for SPACE. Existing chase click-drag and scroll inspect
the model. Check side-view thigh alternation, recovery knee flexion and opposing
arms; check front-view separation and backpack stability. Test diagonal motion,
brief starts/stops, released-input skids, takeoff/landing and mid-gait reversal.
No changes were made to `--model-test`: it still inspects canonical transformations.

## Validation and limitations

253 tests pass, including all earlier regressions. New checks cover actual state
selection, frame-partition-independent phase and wrap, bounded cadence, leg/arm
opposition, knee/ankle limits, backwards/local/diagonal direction, strafe foot
separation, smooth threshold and amplitude transitions, exact idle recovery,
no cumulative overrides, skid/air/SPACE suppression, turn stepping, transformation
fade and exact endpoints, and fresh engine/exhaust attachment transforms.

A 5.33-second full gameplay comparison includes forward motion, strafe, yaw,
release and jump. Enabled versus disabled animation yields exactly identical
position, velocity and orientation arrays at every sample, with matching throttle
and ground status. Separate animator checks confirm contact vectors are untouched.

The hidden-window OpenGL check (`python -m tools.check_transformation_gl
--locomotion --output /tmp/tc167_m17`) exercises live physics, walking/running,
backward motion, strafe, diagonals, skid, turning, takeoff and transformation,
plus all prior endpoints/orbit/reversal checks and three resource cleanup cycles.
Captured side/front, backward and strafe views were visually reviewed. These are
scripted render inspections; interactive keyboard/trackpad feel still requires
local manual acceptance.

This is a procedural approximation without foot locking or IK: feet can slide,
float or clip slightly during support, especially lateral/diagonal transitions. Lateral hip angles are intentionally
bounded to clear the wide stabilizer pods.
Stride and cadence roughly fit movement, but do not enforce contact. Turn steps
are deliberately simple. No root bob, independent aiming, gait clips, terrain,
new physics or later milestone features are implemented.
