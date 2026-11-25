# NAV1 Terrestrial Navigation

On foot or in a supported ground vehicle, Kara's helmet displays a translucent compass tape and signed XYZ coordinates at the top of the screen. Mission, inspection, and journal text remain below it. The aircraft keeps its existing display.

- **Person** reports the simulation's feet position. **Vehicle** reports the occupied vehicle's world origin, centered at the Jeep's chassis reference point.
- **X** increases east, **Y** increases upward, and **Z** increases south. Display values are meters to three decimal places. World Y uses the scene datum. Height above local terrain would require subtracting the ground elevation at X/Z.
- The compass measures the simulation actor's forward vector with `atan2(forward.x, -forward.z)` in degrees. It wraps at 360°, with north at 000°, east at 090°, south at 180°, and west at 270°. Camera orbit and zoom leave both readings unchanged.

## Recording survey points

Press **F6** on foot or in a ground vehicle. Arrival writes to `surveys/arrival_locations.csv` and sandbox writes to `surveys/sandbox_locations.csv`. Files are ignored by Git and append across runs. The header includes marker ID, world ID, layout revision, campaign ID, position reference, XYZ in meters, heading in degrees, and simulation time in seconds. Numbers are written to nine decimal places. The campaign clock resumes from its save. The sandbox clock starts per session. This recorder uses POSIX file locking and rejects a malformed header, sequence, or incomplete final line.

## Power and future capabilities

NAV1 keeps navigation online. The helmet canon establishes several days of operation from a rechargeable micro protonic battery. Consumption, charging, replacement, waypoints, terrain mapping, and other equipment functions await future milestones. World coordinates remain the survey and placement reference. See the [technology record](../lore/technology/terrestrial_navigation_helmet.md) and [mission registry](../lore/missions/REGISTRY.md).
