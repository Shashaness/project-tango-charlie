# M23.1 stabilization report

| Defect | Reproduction and cause | Correction |
| --- | --- | --- |
| Seated pose on foot | Rendering a seat pose, resetting only the current clip, and sampling Idle left the shin's 88° manual rotation in the model's delta layer. The animation player clears the sampled pose it owns while the model still multiplies that result by the manual delta. | Use a baked Seated palette and shared mesh data. A new on-foot session resets the complete model and animation player before Idle, even if a handoff occurs between draws. Camera visibility leaves locomotion evaluation unchanged. |
| Off-road exit | On a 26° cross-slope a stationary Jeep with four contacts found no exit. The center-height test rejected the uphill side, the fixed-height path omitted the slope along the route, and the endpoint could still intersect the tilted hull. The wheel check also rejected stale contact flags. | Check live suspension load and stable chassis support. Sample the route from each door step over reachable support, then verify slope, steps, drop, and capsule clearance. Search both sides in a fixed order, with additional forward and rearward candidates. |
| Backward knees | The source GLB sampled about −37° local-X rotation in Walk and −63° in Run. With the feet extending down local −Y and +Z facing forward, these rotations move the shin forward past the extended knee. | Use bounded positive local-X knee flexion, smaller bends in stance, and stronger bends in swing as the hip brings the foot forward. Alternate the legs by half a cycle and counter-rotate the feet. Regenerate the pilot GLB with the same joint hierarchy and four clip names. |

M23.2 moved the pilot source to Blender. Use the
[Blender authoring workflow](M23_2_BLENDER_ASSET_AUTHORING.md) to export its
current GLB. Numerical regressions are in `tests/test_m23_stabilization.py`. They test 12 seat/stand
cycles, camera-independent pose playback, safe and unsafe exits, knee limits
and alternation, and the round trip from TC167 through driving and walking
on actual natural terrain. The baseline was 465 Python tests and 12 Blender
integration tests.

The placeholder keeps a fixed seated pose and procedural gait cycles.
Its feet lift during swing, with terrain-dependent foot placement left
to future animation work. Exit placement remains conservative near
complex obstacle geometry and uses the shared capsule and vehicle bounds.

The final suite passed all 470 Python tests and all 12 Blender integration
tests. Blender imported the rebuilt pilot with 17 bones and all four
locomotion clips. The TC167 runtime comparison passed, and a fresh export
matched the checked-in TC167 GLB byte for byte.

An OpenGL run captured seating and immediate standing, the front and side
of both gaits, the Turn clip, camera zoom and orbit, and re-entry into the
Jeep and the TC167. The pilot exited on level natural terrain and on an
11.2° natural slope, returned to the hangar, and triggered a TC167
transformation after re-entry.
