# Milestone 16 — continuous canonical vehicle transformation

One unchanged approved M15 GLB supplies Fighter, VTOL and Battledroid. Its
rectangular intake lips/splitters and compact rear engine spacing are preserved.
No asset regeneration, new meshes, dependencies, flight retuning or controls.
Normal rendering now uses this hierarchy for all three configurations. Primitive
VTOL/Battledroid routines remain as a legacy/debug fallback, outside normal Game rendering.

`game/tc167_poses.py` holds absolute parent-local TRS pose overrides in source
GLB coordinates (+Y up, +Z nose). Fighter is the imported rest pose (empty table).
Only changed nodes are listed. Runtime overrides never modify imported data.
`game/transformation.py` supplies the staged reversible controller and existing
quaternion SLERP; `game/transformation_test.py` supplies the isolated viewer cycle.
The controller owns source/target, progress, duration and visual pose. It writes
only pose overrides, the active physics mode, and the existing GROUNDED-to-FLYING
status change when leaving Battledroid. It never writes motion, throttle or combat state.

The actual stable nodes used are Fuselage, Nose, Cockpit, TorsoCore, Head,
Left/RightWing, Left/RightTail, VerticalTail, Left/RightIntake, Left/RightUpperLeg,
Left/RightLowerLeg, Left/RightFoot, Left/RightShoulder, Left/RightUpperArm,
Left/RightForearm, Left/RightHand and Left/RightEngine. `FuselageSurface1` is the
inspected GLB's central shell companion; only it telescopes during Battledroid
assembly. TC167Root remains the visual root; existing gun/hardpoint markers
remain present but gameplay weapon spawning does not follow animated nodes yet.

Fighter -> VTOL unlocks intakes, deploys hip/knee chains and unfolds feet.
Wings retain their aircraft silhouette, with small bank adjustments. Engines
follow the actual foot matrices with their imported attachment offsets, then
move upward/back toward calf collars and vector their nozzles downward. They
remain visible and mechanically associated with the folded nacelle assembly.
No engine spacing correction is applied in Fighter: its imported matrices return exactly.

VTOL -> Battledroid brings the legs beneath the body, establishes an upright
central core, folds the nose down along the torso, moves the canopy to the chest,
deploys shoulders/arms, then reveals the head. Wings/tails fold into a backpack.
The neutral soles reach approximately -3.20 m relative to vehicle COM, matching
the existing 3.2 m contact datum without changing its collider or contact code.
All scales remain positive; no component is hidden or scaled to zero.

[VTOL endpoint sheet](transformation_vtol.png) and
[Battledroid endpoint sheet](transformation_battledroid.png) show top, side, front,
rear, bottom and front three-quarter views. Preview shading is development-only;
the game retains its original unlit color shader.

| Component | Fighter -> VTOL window | VTOL -> Battledroid window |
|---|---|---|
| Intake | 0–25% | 30–90% |
| Upper leg | 10–50% | 35–80% |
| Lower leg | 25–70% | 40–85% |
| Foot | 45–85% | 55–95% |
| Engine collar | 45–85% | 20–60% |
| Visual root height | unchanged | 35–90% |
| Wing | 60–100% | 45–95% |
| Tail | 60–100% | 30–85% |
| Fuselage / torso core | unchanged | 10–60% |
| Nose | unchanged | 15–65% |
| Cockpit | unchanged | 20–65% |
| Shoulder | unchanged | 35–70% |
| Upper arm | unchanged | 40–80% |
| Forearm | unchanged | 50–90% |
| Hand | unchanged | 60–95% |
| Head | unchanged | 75–100% |

Each window clamps its fraction and applies smoothstep `t*t*(3-2*t)`.
Translations/scales interpolate linearly; normalized [x,y,z,w] rotations use
shortest-path SLERP. Endpoint rest matrices restore exactly. Engines compose
ancestor quaternions and inherit ankle motion; no Euler decomposition is used.

Fighter <-> VTOL takes 1.5 seconds; VTOL <-> Battledroid takes 1.9 seconds.
G cycles Fighter -> VTOL -> Battledroid -> Fighter. The last command chains
Battledroid -> VTOL -> Fighter (3.4 seconds). Press G during an active edge to
reverse it, cancel any remaining chain, and retrace the same timing curve without
moving any node at the reversal instant. Repeated reversals remain continuous.
Explicit controller requests also support VTOL -> Fighter and Battledroid -> VTOL.

Physics switches at **60% directional progress of each edge**, retaining the
source force model earlier. On reversal the active force model stays unchanged
until the reverse curve reaches its own 60% threshold; this is deliberate
hysteresis. Configuration is advanced before each existing <=1/120-second flight
substep, so physics/combat never pause. At a switch only mode/status change;
position, world velocity, orientation, throttle, thrust vector, targets and
inventories are untouched. Subsequent acceleration follows existing forces,
gravity and artificial SPACE damping. No aerodynamic interpolation is introduced.
SAS and hover eligibility use existing rules; transformation never enables hover.

The HUD shows `MODE FIGHTER > VTOL` and `XFORM 43%` while active. Camera remains
attached to the physical root. Articulated VTOL uses Fighter's 24/7 m chase
framing so the camera stays outside the 19 m fuselage. Battledroid stays at 12/4 m;
smoothing, bank, cockpit visibility and look-ahead behavior are unchanged.

```sh
source .venv/bin/activate
python main.py --enemies 0
python main.py --atmosphere --enemies 0
python main.py --model-test --enemies 0
python main.py --battledroid-test landing --enemies 0
python -m unittest discover -s tests -q
python -m tools.check_transformation_gl --output /tmp/tc167_m16
```

Default `--model-test` holds Fighter 0–3 s, transforms 3–4.5 s, holds VTOL
until 7.5 s, transforms to Battledroid until 9.4 s, holds until 16.4 s, reverses to
VTOL until 18.3 s, holds until 21.3 s, returns to Fighter until 22.8 s and holds
until its 25.8 s repeat. The Battledroid endpoint now holds for seven seconds. State/percentage appear in the window title. This separate
model/controller never controls physics. `--model-test --prototype-test` retains
the M15 pivot exercise; `--model-path` retains static custom GLB inspection.

Validation: **214 tests pass**. Atmospheric 180 m/s transformation compares
against the existing force integrator, with no pause/reset; SPACE transformation
preserves the original drift direction and inverted attitude, with existing
damping accounted for. Battledroid completes transformation, lands and walks via
M13 contact/locomotion. Guns, countermeasures and ECM operate during transition;
controller-only checks preserve target/inventory state exactly. One old G-key
regression was updated to expect delayed mode switching; its original momentum
and force-behavior checks remain. Other M1–M15 tests pass.

A real hidden-window OpenGL smoke check on Apple M3 Pro passed three repeated
uploads, every endpoint/transition, reversal and double-close. All model
VAO/VBO/EBO handles were confirmed deleted; no GL errors occurred. Captured
endpoint/intermediate frames were inspected. Local interactive acceptance still
needs high-speed atmospheric G, turned/inverted SPACE drift, combat through G,
landing/walking and repeated G reversal. No human playtest or frame-rate benchmark
is claimed.

Mechanical compromises: the torso shell telescopes to 38% longitudinal scale;
this avoids the original 8 m aft fuselage passing through the legs. The nose
folds along the torso, some wing/shoulder packaging overlaps, and engines slide
along their ankle attachments. Battledroid is taller/wider than the unchanged M13
collider; sole placement matches its upright datum but tilted/ground transformations
can clip the ground. M18 preserves ground support through Battledroid/VTOL switches using a smooth
contact envelope; see [VTOL ground integration](VTOL_GROUND.md). Fighter
still uses its original ground/crash rules. Cockpit view still observes the
vehicle COM, not the articulated canopy. Ground walking used a static
pose in M16; M17 now adds a relative visual gait (see [locomotion](LOCOMOTION.md)).

M17 integration follows this constraint: layer a small rest-relative walking pose over the canonical
Battledroid pose, drive gait from existing ground walk_phase/speed, keep ankle soles
near the contact datum and preserve the transformation controller's rest data.
Choose a clear ownership/order for gait overrides versus transformation overrides.
Do not derive physics/contact from animated geometry. M17 is documented in
[LOCOMOTION.md](LOCOMOTION.md).

## Milestone 16.2 endpoint refinement

Only Battledroid pose data, its component phase windows and the inspection hold were
tuned. Fighter/VTOL local matrices were compared against the previous version
and are identical. The binary asset, orbit camera, physics/combat and durations
remain unchanged. Source-coordinate changes (meters) are:

| Component | Before | After |
|---|---|---|
| Intake/leg parent position | (±1.3, 2.62, -0.4) | (±1.1, -0.6, -0.4) |
| Intake parent scale | (1,1,1) | (0.85,0.448,1) |
| Shoulder position | (±2.25,1.7,-0.2) | (±1.5,2.0,-0.2) |
| Upper arm rotation | X=-90°, Z=±8° | X=-75°, Y=∓12° |
| Upper arm scale | (1,1,1) | (0.9,0.9,0.75) |
| Forearm bend | X=10° | X=25° |
| Wing hinge position | (±1.05,1.5,-1.2) | (±1.25,2.8,-1.5±0.1) |
| Wing rotation | X=-80°, Z=±65° | X=-90°, Y=±90° |
| Horizontal tail rotation | X=75° | X=-75°, Y=±65° |

Intake parents move the complete assemblies downward by 3.22 m relative to the previous endpoint, and
0.5 m downward from the approved VTOL carrier position. Their vertical
compression keeps the existing sole datum: the actual feet reach -3.204 m,
without changing child joint translations or contact physics. Battledroid-only engine collar offset rises by 0.11 m to clear the shortened
leg/sole assembly; Fighter and VTOL engine attachments are unchanged. This remains a
prototype telescoping compromise, rather than rigid-link deployment. Shortened
arm assemblies produce a relaxed bent-elbow pose. Wings retain unit scale and
all geometry; a 0.2 m depth stagger avoids overlapping back panels fighting for
depth. Horizontal tails fold inward/downward instead of maintaining the old
wide shoulder outline. Overall Battledroid width falls from 6.8 to 4.9 m. Nose,
cockpit, torso and head overrides are untouched.

Intake travel now occupies 30–90% of VTOL -> Battledroid, upper-leg rotation
15–65%, and wing folding 45–95%. Reverse follows the same curves backward.
Transition durations remain 1.5/1.9 seconds and the physics switch stays at 60%.

Exhaust inspection found no player afterburner effects; the colored end caps are
engine mesh geometry, and the exhaust markers themselves have no mesh. There
was a real stale-cache issue: engine overrides were applied after hierarchy
traversal, leaving engine/exhaust world_matrix caches at an earlier pose until
draw. A final traversal now refreshes these descendants after overrides and
Fighter reset. For an effect in world space, evaluate
`model.world_matrices(vehicle.model_matrix())` and read the named exhaust node;
its matrix includes vehicle root, import conversion, engine pose and marker
local transform exactly once. No special VTOL effect coordinates were added.
This cache fix does not alter approved VTOL geometry; it is not a claim that
every subjective exhaust concern has been resolved.

**232 tests pass**, including preserved poses, parent inheritance, actual sole
height, compact unscaled wings, exact reverse endpoint, fresh nozzle caches and
full world transforms without physical-state writes. OpenGL endpoint/transitions,
all six requested orbit viewpoints and three repeated cleanup cycles pass with
no GL errors. Captured views were inspected. Local visual approval remains with
the user. No gait, aiming or M17 work was started.

## Milestone 16.3 leg extension

Both hip/intake positions remain (±1.1, -0.6, -0.4), relative to the body.
Upper-leg X rotation changes from -90° to -94°, knee from +15° to +8°,
and ankle from -105° to -94°. The resulting chain totals -180°: soles are
horizontal, with thighs and shins only 4° from vertical and 8° knee flexion.
Joint translations and native 2.6 m segment lengths remain unchanged.

Inspection found that M16.2 intake-parent Y scale 0.448 compressed the entire
leg chain. Battledroid now restores that parent axis to 1 and puts the same 0.448
scale on its three intake mesh children instead. The shells retain their shape;
leg joints have unit scale and use their existing geometry, without stretching.
Fighter and VTOL remain unchanged.

The visual model root rises 3.2873 m during 35–90% of the transition. This moves
the body and hips together without changing their relative placement, keeping
the final soles at -3.20003 m relative to the physical vehicle root. Contact
clearance, physics, controls, combat and camera are unchanged. Standing height
increases from 7.45 to 10.74 m; hip-to-sole length is 5.89 m (about 55%).

Leg rotation windows are now 35–80% (hip), 40–85% (knee), 55–95% (ankle).
Reverse retraces the same smooth curves; durations and mode-switch timing are
unchanged. Engine attachment calculations include the visual parent's full
transform so the new root offset is applied exactly once.

Manual acceptance: inspect front/side views with chase orbit, transform and
reverse halfway, land upright and walk. Repeat in SPACE and ATMOSPHERE to check
retained momentum. Walking still uses the existing sliding stance; M17 is not
implemented.

## Milestone 16.4 arm pose correction

The actual chains are LeftShoulder → LeftUpperArm → LeftForearm → LeftHand,
and RightShoulder → RightUpperArm → RightForearm → RightHand. Imported upper-arm
and forearm joint frames have identity rotation; both links extend along local
-Z. The elbow hinge is local X on both sides. With the deployed parent frame,
positive elbow rotation bends backward (+Z in engine coordinates). Negative
rotation bends forward (-Z). Mirrored placement does not reverse this hinge sign.

| Joint | Previous Battledroid rotation | Corrected rotation |
|---|---|---|
| Upper arm | X=-75°, Y=∓12° | X=-90°, Y=∓12° |
| Forearm / elbow | X=+25° | X=-20° |
| Hand / wrist | X=-10° | identity (0°) |

Both sides use the same elbow sign. Shoulders, translations, scales and timing
are untouched. Upper arms descend beside the torso; elbows bend forward and
hands follow forearms at approximately hip height, outside the torso. Only the
six upper-arm, forearm and hand endpoint overrides changed. Fighter, VTOL,
all other Battledroid component poses and M16.3 stance remain identical.

All 237 tests pass, including hierarchy/axis checks, symmetric forward hands,
continuous reversal and endpoint completion. Existing OpenGL validation passes
for endpoints, transitions, reversal, orbit views and three resource cleanup
cycles. Front, both sides, rear and front-three-quarter captures were inspected.
The updated endpoint sheet shows the full stance; close orbit captures can crop
the head at the existing default distance. Physics, camera, combat and
transformation architecture are untouched. No walking animation was added.
