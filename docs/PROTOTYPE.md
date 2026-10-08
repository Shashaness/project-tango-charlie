# Milestone 15 prototype airframe

An original, procedurally authored folded twin-engine interceptor. The runtime
loads `assets/models/vehicles/tc167_prototype.glb`; it never regenerates geometry.
Reproduce it with `python tools/generate_tc167_prototype.py` (standard library).
It has 1,592 triangles, 68 nodes including visual companions/markers, and 32
mesh primitives. Dimensions are X=12.00 m, Y=4.15 m, Z=19.02 m. Lower polygon
count is intentional: this prototype prioritizes silhouette and joint structure.

All coordinates below are **engine vehicle-local meters**, after the existing
M14 root conversion. Source geometry uses +Z toward the nose and opposite X;
no additional gameplay rotation is applied.

```text
TC167Root
  Fuselage
    Nose
    Cockpit
    TorsoCore
    Head
  LeftWing / RightWing
    MissileLeft01 / MissileRight01
    MissileLeft02 / MissileRight02
  LeftTail / RightTail
  VerticalTail
  LeftIntake / RightIntake
    LeftUpperLeg / RightUpperLeg
      LeftLowerLeg / RightLowerLeg
        LeftFoot / RightFoot
  LeftEngine / RightEngine
    LeftEngineExhaust / RightEngineExhaust
  LeftShoulder / RightShoulder
    LeftUpperArm / RightUpperArm
      LeftForearm / RightForearm
        LeftHand / RightHand
  GunMount
```

Each mesh resides on a generated `ComponentSurfaceN` child, leaving its named
joint unscaled. These companion names are inspectable implementation details;
future pose code should address the stable component names above.

| Joint/marker | Left | Right |
|---|---|---|
| Wing hinge | (-1.3, 0, -0.5) | (1.3, 0, -0.5) |
| Intake/hip carrier | (-1.6, -0.1, -2) | (1.6, -0.1, -2) |
| Hip | (-1.6, -0.1, -0.5) | (1.6, -0.1, -0.5) |
| Knee | (-1.6, -0.1, 2.1) | (1.6, -0.1, 2.1) |
| Ankle | (-1.6, -0.1, 4.7) | (1.6, -0.1, 4.7) |
| Shoulder | (-0.95, -0.5, -1.3) | (0.95, -0.5, -1.3) |
| Elbow | (-0.95, -0.5, 0.7) | (0.95, -0.5, 0.7) |
| Exhaust | (-1.0, 0, 9.52) | (1.0, 0, 9.52) |

The nose hinge is (0,0,-3.8). GunMount is (0,-0.65,-4.2).
Hardpoint markers are at X=±2.6 and ±3.8, Y=-0.35, Z=0.9.
Gun/missile spawning remains unchanged; these markers are preparation only.

The leg shells form the longitudinal intake/nacelle surfaces. Feet are compact
rear fairings in front of the separate dark exhaust structures. Arm segments
form narrow under-wing-root fairings, rather than exposed humanoid limbs. The
head is enclosed inside the central fuselage at (0,0.35,0.9); the torso is an
integrated central lower shell. Both horizontal stabilizers and a single
vertical fin establish a conventional aircraft tail. Gray panels, dark canopy,
dark intake mouths and limited exhaust accents use existing base-color rendering.

Normal Fighter rendering uses the GLB in chase view. Cockpit mode retains its
existing hidden-player visual behavior. VTOL, Battledroid and enemy models remain
primitive. Fighter chase distance/height are now 24/7 m to clear the large model;
VTOL remains 8/3 m and Battledroid 12/4 m. Camera smoothing, orientation, look-ahead
and cockpit behavior are unchanged. Physics, collisions and combat are untouched.

```sh
source .venv/bin/activate
python main.py --enemies 0
python main.py --enemies 0 --model-test --prototype-test
python main.py --inspect-model assets/models/vehicles/tc167_prototype.glb
python -m tools.preview_tc167_prototype /tmp/tc167_prototype_views.png
python -m unittest discover -s tests -q
```

The original M14 `--model-test` sequence remains available unchanged. Add
`--prototype-test` to show the aircraft 18 m ahead and 14 m to the right of the vehicle.
It rotates as a whole for five seconds, then exercises left/right wing, hip,
knee, ankle, shoulder and elbow for three seconds each, followed by three
seconds at imported rest pose. The 29-second cycle repeats. No inspection keys
or gameplay controls were added. A separate model instance prevents debug
articulation affecting the normal Fighter visual.

CPU previews contain top, side, front, rear, bottom and front three-quarter
views (left-to-right, top-to-bottom). Their simple shading is inspection-only;
the game retains its existing unlit color shader. CPU silhouette inspection
shows a swept-wing, twin-engine aircraft with an enclosed central body and
folded limbs. Automated pivot checks validate fixed origins, geometry motion,
child inheritance and exact reset. A real hidden-window OpenGL smoke test passed on Apple M3 Pro: three repeated
upload/draw/delete cycles, Fighter/VTOL/Battledroid draws, all prototype inspection
phases, no GL errors, and every model VAO/VBO/EBO handle deleted. The initial
sandboxed GLFW attempt aborted; window-server access resolved it. Normal Fighter
and articulated inspection frames were captured and inspected. Interactive flight,
combat and handling acceptance still require local playtesting. Frame-rate impact
has not been measured.

[CPU silhouette sheet](prototype_views.png) shows the rest configuration.
To preview a pivot without OpenGL, append its name and angle to the preview command,
e.g. `python -m tools.preview_tc167_prototype /tmp/knee.png LeftLowerLeg 20`.

For M16: this is a viable joint scaffold, not a solved transformation mechanism.
Deployment angles, limb clearance and final humanoid proportions still need
engineering; the intake carrier and separate engine will need coordinated poses.
The stowed head and torso overlap enclosing shells deliberately. The generator
uses fixed rest geometry and does not define transformation poses or animation.

Intake mouths use beveled rectangular profiles and prominent inboard splitter
plates. The aft leg/foot shells taper inward toward exhaust centers at X=±1 m
(2 m center spacing), keeping the rear engine pair compact. Joint origins remain
unchanged except for the two visual engine nodes.
