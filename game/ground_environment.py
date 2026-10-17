"""LOD-independent ground queries and spatially indexed static obstacle contact."""

import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from game.flight_state import Environment, FlightStatus, VehicleMode


@dataclass(frozen=True)
class GroundHandlingParameters:
    """Ground tuning in SI units; slope/steering _deg fields use degrees."""

    steering_angle_deg: float = 28.0
    steering_fade_speed: float = 45.0
    wheelbase: float = 10.0
    yaw_inertia: float = 90000.0
    yaw_response: float = 3.0
    lateral_grip: float = 0.65
    max_slope_deg: float = 12.0
    vtol_max_slope_deg: float = 25.0
    battledroid_max_slope_deg: float = 35.0
    obstacle_crash_speed: float = 8.0
    obstacle_crash_deceleration: float = 120.0
    impact_duration: float = 0.1
    terrain_grid: float = 32.0


GROUND_HANDLING = GroundHandlingParameters()
ROLLING_RESISTANCE = {
    "runway": 0.015,
    "marking": 0.015,
    "taxiway": 0.018,
    "apron": 0.02,
    "street": 0.02,
    "sidewalk": 0.035,
    "plaza": 0.025,
    "desert": 0.07,
    "lot": 0.09,
    "park": 0.08,
    "path": 0.05,
    "perimeter": 0.04,
}


class GroundEnvironment:
    """Query terrain support and spatially indexed swept contact with static obstacles."""

    def __init__(self, environment, terrain, parameters=GROUND_HANDLING):
        """Build a spatial obstacle index and initialize bounded ground-height queries.

        Args:
            environment: Physics environment or shared scenery data, as required by the
                component.
            terrain: Terrain height provider and compatibility mask.
            parameters: Configuration parameters; defaults retain the simulation tuning.
        """
        self.environment = environment
        self.terrain = terrain
        self.parameters = parameters
        self.heights = OrderedDict()
        self.cell_size = 128.0
        self.cells = {}
        self.boxes = [
            (np.asarray(center) - np.asarray(size) / 2, np.asarray(center) + np.asarray(size) / 2)
            for center, size in environment.solid_boxes
        ]
        self.boxes.extend(
            (
                np.asarray(b.center) - np.asarray(b.size) / 2,
                np.asarray(b.center) + np.asarray(b.size) / 2,
            )
            for b in environment.buildings
        )
        for index, (lo, hi) in enumerate(self.boxes):
            for x in range(
                math.floor(lo[0] / self.cell_size), math.floor(hi[0] / self.cell_size) + 1
            ):
                for z in range(
                    math.floor(lo[2] / self.cell_size), math.floor(hi[2] / self.cell_size) + 1
                ):
                    self.cells.setdefault((x, z), []).append(index)

    def _flat(self, x, z):
        """Check whether infrastructure protection requires zero ground elevation.

        Args:
            x: Local east-axis coordinate(s) in meters.
            z: Local south-axis coordinate(s) in meters.

        Returns:
            Whether the query lies in the flat compatibility region.
        """
        for a, b, c, d in self.terrain.compatibility.footprints:
            dx = max(a - x, 0.0, x - b)
            dz = max(c - z, 0.0, z - d)
            if dx * dx + dz * dz <= self.terrain.compatibility.clearance**2:
                return True
        return float(self.terrain.compatibility_weight(x, z)) == 0.0

    def _height_node(self, x, z):
        """Query a terrain grid node and refresh the bounded height cache.

        Args:
            x: Local east-axis coordinate(s) in meters.
            z: Local south-axis coordinate(s) in meters.

        Returns:
            Grid-node elevation in meters.
        """
        key = (x, z)
        if key not in self.heights:
            self.heights[key] = self.terrain.height_at(x, z)
            if len(self.heights) > 512:
                self.heights.popitem(last=False)
        self.heights.move_to_end(key)
        return self.heights[key]

    def height(self, position):
        """Interpolate support heights on a fixed grid independent of render LOD.

        Args:
            position: XYZ position in meters.

        Returns:
            Ground elevation in meters; infrastructure regions return zero.
        """
        x, z = float(position[0]), float(position[2])
        if self._flat(x, z):
            return 0.0
        step = self.parameters.terrain_grid
        a, c = math.floor(x / step) * step, math.floor(z / step) * step
        u, v = (x - a) / step, (z - c) / step
        return (
            (1 - u) * (1 - v) * self._height_node(a, c)
            + u * (1 - v) * self._height_node(a + step, c)
            + (1 - u) * v * self._height_node(a, c + step)
            + u * v * self._height_node(a + step, c + step)
        )

    def normal(self, position):
        """Estimate the support normal with central height differences.

        Args:
            position: XYZ position in meters.

        Returns:
            Unit world-space surface normal.
        """
        if self._flat(position[0], position[2]):
            return np.array((0.0, 1.0, 0.0))
        x, y, z = position
        step = 1.0
        dx = (self.height((x + step, y, z)) - self.height((x - step, y, z))) / (2 * step)
        dz = (self.height((x, y, z + step)) - self.height((x, y, z - step))) / (2 * step)
        n = np.array((-dx, 1.0, -dz))
        return n / np.linalg.norm(n)

    def surface(self, position):
        """Find the topmost ground-surface classification at a world position.

        Args:
            position: XYZ position in meters.

        Returns:
            Surface kind string, defaulting to desert.
        """
        x, _, z = position
        for tile in reversed(self.environment.ground_surfaces):
            a, b, c, d = tile.bounds
            if a <= x <= b and c <= z <= d:
                return tile.kind
        return "desert"

    def resistance(self, position):
        """Look up rolling resistance for the surface beneath the query.

        Args:
            position: XYZ position in meters.

        Returns:
            Dimensionless rolling-resistance coefficient.
        """
        return ROLLING_RESISTANCE.get(self.surface(position), 0.07)

    def landing_suitable(self, vehicle):
        """Compare support slope with the current configuration landing limit.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

        Returns:
            Whether the local slope is within the configured limit.
        """
        limit = (
            self.parameters.battledroid_max_slope_deg
            if vehicle.flight_state.mode is VehicleMode.BATTLEDROID
            else self.parameters.vtol_max_slope_deg
            if vehicle.flight_state.mode is VehicleMode.VTOL
            else self.parameters.max_slope_deg
        )
        return (
            math.degrees(math.acos(float(np.clip(self.normal(vehicle.position)[1], -1, 1))))
            <= limit
        )

    def candidates(self, lo, hi):
        """Collect obstacle indices from grid cells intersecting a swept world AABB.

        Args:
            lo: Lower XYZ corner of a world-space bounding box in meters.
            hi: Upper XYZ corner of a world-space bounding box in meters.

        Returns:
            Sorted list of obstacle indices.
        """
        result = set()
        for x in range(math.floor(lo[0] / self.cell_size), math.floor(hi[0] / self.cell_size) + 1):
            for z in range(
                math.floor(lo[2] / self.cell_size), math.floor(hi[2] / self.cell_size) + 1
            ):
                result.update(self.cells.get((x, z), ()))
        return sorted(result)

    def body_bounds(self, vehicle):
        # Conservative hulls exclude abstract tire points so pavement is not a wall.
        """Compute conservative oriented body extents, excluding abstract wheel points.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

        Returns:
            Pair of lower and upper XYZ offsets in meters relative to the vehicle origin.
        """
        if vehicle.flight_state.mode is VehicleMode.FIGHTER:
            from game.fighter_ground import fighter_vertices

            points = fighter_vertices() @ vehicle.orientation.T
            return points.min(axis=0), points.max(axis=0)
        from game.battledroid_physics import BATTLEDROID

        if vehicle.flight_state.mode is VehicleMode.BATTLEDROID:
            extent = np.abs(vehicle.orientation) @ np.array((1.4, 3.2, 1.2))
        else:
            extent = np.abs(vehicle.orientation) @ np.array((6.0, 7.0, 10.5))
        return -extent, extent

    def collide(self, vehicle, previous, dt):
        """Resolve the earliest swept obstacle hit, writing position, velocity, and crash state.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            previous: Vehicle world position before integration, in meters.
            dt: Elapsed simulation time in seconds.
        """
        if vehicle.flight_state.environment is not Environment.ATMOSPHERE:
            return
        lower, upper = self.body_bounds(vehicle)
        lo = np.minimum(previous, vehicle.position) + lower
        hi = np.maximum(previous, vehicle.position) + upper
        delta = vehicle.position - previous
        best = None
        for index in self.candidates(lo, hi):
            boxlo, boxhi = self.boxes[index]
            # Expand the obstacle by the oriented hull extents (Minkowski difference),
            # reducing swept hull contact to a segment-versus-box slab intersection.
            a, b = boxlo - upper, boxhi - lower
            enter, leave = 0.0, 1.0
            normal = None
            valid = True
            for axis in range(3):
                # Intersect each axis's entry/exit interval; the latest entry supplies
                # the contact face normal, and an empty interval rejects the obstacle.
                if abs(delta[axis]) < 1e-12:
                    if previous[axis] <= a[axis] or previous[axis] >= b[axis]:
                        valid = False
                        break
                else:
                    first, last = (
                        (a[axis] - previous[axis]) / delta[axis],
                        (b[axis] - previous[axis]) / delta[axis],
                    )
                    sign = -1 if delta[axis] > 0 else 1
                    if first > last:
                        first, last = last, first
                    if first >= enter:
                        enter = first
                        normal = np.eye(3)[axis] * sign
                    leave = min(leave, last)
                    if enter > leave:
                        valid = False
                        break
            if valid and 0 <= enter <= leave and enter <= 1:
                separation = None
                if normal is None:
                    # Already overlapping: project through the nearest separating face.
                    distances = np.r_[previous - a, b - previous]
                    face = int(np.argmin(distances))
                    axis = face % 3
                    normal = np.eye(3)[axis] * (-1 if face < 3 else 1)
                    separation = vehicle.position.copy()
                    separation[axis] = (a if face < 3 else b)[axis] + normal[axis] * 0.001
                if best is None or enter < best[0]:
                    best = (enter, normal, separation)
        if best is None:
            return
        fraction, normal, separation = best
        impact = max(0.0, -float(np.dot(vehicle.velocity, normal)))
        if separation is not None:
            # Rotation can expand the hull into a wall. Resolve that overlap
            # instead of repeatedly rewinding translation to the same point.
            vehicle.position[:] = separation
        elif np.linalg.norm(delta) > 0:
            vehicle.position[:] = previous + delta * max(0.0, fraction - 1e-5) + normal * 0.001
        if (
            impact > self.parameters.obstacle_crash_speed
            or impact / self.parameters.impact_duration
            > self.parameters.obstacle_crash_deceleration
        ):
            vehicle.flight_state.status = FlightStatus.CRASHED
            vehicle.airfield.crash_reason = "OBSTACLE COLLISION"
            vehicle.velocity[:] = 0
            vehicle.acceleration[:] = 0
            vehicle.engine_throttle = 0
            vehicle.pitch_rate = vehicle.yaw_rate = vehicle.roll_rate = 0.0
        else:
            vehicle.velocity -= normal * min(0.0, float(np.dot(vehicle.velocity, normal)))
