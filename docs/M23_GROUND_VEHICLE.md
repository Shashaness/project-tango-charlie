# M23 ground vehicle driving

## Architecture

Player control owns occupancy and chooses the nearest eligible access point.
The TC167 and the Jeep remain in the same world in all three control modes.
Aircraft configurations and the flight controller retain their existing
responsibilities. The Jeep has a separate fixed-step dynamics controller
that queries the same terrain and obstacle index as the pilot. Driving uses
its own camera and the existing mouse and scroll input collection.

The M22 Python suite starts with 457 tests. The M23 regression pass also
checks the TC167 Blender integration and the exported animation asset.

## Play

Exit the parked TC167 with **Enter**, walk to the olive Jeep in the same
hangar, and press **Enter** when the HUD names the Jeep. Release the parking
brake with **P**, drive west through the doorway, and continue across the apron
onto the surrounding terrain. Stop before pressing **Enter** to leave the
Jeep. The parking brake engages on exit. Walk to either vehicle and use
**Enter** again.

| Input | While driving |
| --- | --- |
| W | Accelerate forward |
| S | Brake forward motion, then reverse |
| A / D | Steer left / right |
| Space | Service brake |
| P | Toggle parking brake |
| Left mouse drag | Orbit |
| Mouse wheel | Zoom |
| Home / Backspace | Reset driving view |

The HUD shows speed in km/h and the active drive or brake state. Both cameras
retain their mode-specific bindings. Flight controls resume after boarding
the TC167, with **E** still assigned to aircraft yaw.

## Control and world integration

`game/player_control.py` owns the registered vehicle identities, occupancy,
and active control mode. A vehicle access adapter supplies entry points,
eligibility, occupancy activation, collision bounds, and safe exit placement.
The nearest reachable eligible access point wins. Obstacle checks include the
other parked vehicle. Jeep exit requires longitudinal speed at or below
0.25 m/s, low vertical speed, a chassis tilt within 32°, and stable
suspension support. Live wheel compression determines load-bearing corners.
At least three corners must carry load, and their combined spring load must
carry at least 70% of the Jeep's weight. A stale contact flag at one corner
can coexist with stable support.

Exit placement starts at a modeled door step and follows the local terrain
or reachable solid support. The route checks step height, drops, pedestrian
slope limits, and capsule clearance. Candidates on both sides include
forward and rearward offsets, and the final capsule must clear the Jeep
as well as other geometry. The same checks apply in the hangar, on the
runway, and outside. The TC167 retains the M22 support and speed checks.
Its three configurations remain independent of occupancy.

World positions are in meters with +Y up. The scene holds the Jeep and TC167
continuously; the pilot object is retained across every handoff. The Jeep
collides with the indexed world boxes and aircraft hull, and the existing
aircraft obstacle solver sees parked ground vehicles. New ground vehicles can
join the world's vehicle list and register their own access adapter. They can
reuse the driving controller with different parameters and anchor positions.

## Handling and asset workflow

`game/ground_vehicle.py` integrates at 120 Hz. Four independent terrain
samples supply spring compression and chassis pitch and roll. Surface
resistance and the supported-wheel count affect traction. Signed speed and
speed-dependent steering feed a wheelbase turning model. A swept oriented
footprint handles obstacle impacts. The driving camera smooths chase heading
and zooms from 2.5 to 15 m, with the boom shortened by nearby collision shapes.

| Default parameter | Value |
| --- | --- |
| Mass | 1,450 kg |
| Drive / service brake force | 6,400 / 13,000 N |
| Maximum steering angle | 34° |
| Wheelbase / track | 2.65 / 1.65 m |
| Tire radius | 0.43 m |
| Suspension travel | 0.28 m |
| Spring / damping per supported wheel | 38,000 N/m / 4,800 N·s/m |
| Nominal traction | 0.92 |
| Forward / reverse speed limits | 21 / 6 m/s |

Edit the Jeep in `assets/blender/jeep.blend` and export the runtime
`assets/models/vehicles/jeep.glb`. See the
[Blender asset authoring guide](M23_2_BLENDER_ASSET_AUTHORING.md).
The rigid-node glTF asset uses +Y up and +Z forward. Keep `Hub.FL`, `Hub.FR`, `Hub.RL`, and `Hub.RR` as
suspension pivots, each with a corresponding `Wheel.*` child. The front hubs
steer and all wheel children spin around local X. Keep `DriverSeat`,
`Entry.L/R`, and `Exit.L/R` in chassis space. The world reads those anchors
from the GLB. For a body with different dimensions, update the physics
footprint and wheel parameters along with the model. The TC167 authoring
pipeline and the pilot's GLB are independent of the Jeep model.

## Verification and limitations

```sh
.venv/bin/python -m unittest discover -s tests -p test_ground_vehicle.py
.venv/bin/python -m unittest discover -s tests -p test_m23_stabilization.py
.venv/bin/python -m unittest discover -s tests
/Applications/Blender.app/Contents/MacOS/Blender \
  --background --factory-startup --python-exit-code 1 \
  --python tools/test_tc167_authoring_blender.py
```

The lifecycle regression exits the TC167, boards the Jeep, crosses the apron
onto exterior terrain, reverses back, parks, and boards the same TC167 before
taking a flight-controller command. Additional tests exercise competing
interaction points, alternate exits, brakes, wheel poses, suspension, frame
partitioning, terrain traction, camera clipping, and thin-wall collisions.

The chassis uses a single-speed bicycle model with four terrain contacts.
Tire slip is simplified, collisions stop the Jeep without a damage model,
and the driver uses a fixed seated pose. Terrain height and collision bounds
come from the existing world queries.

If the HUD reports that the Jeep needs stable wheel support, come to a
complete stop with the chassis settled. If it reports that there is no safe
exit, look for steep ground, a drop, or nearby obstacles. Move to a more
open spot with support beside a door. See [the stabilization report](M23_1_STABILIZATION.md).
