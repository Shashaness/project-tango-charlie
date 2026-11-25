"""Persistent vehicle occupancy and meter-space pedestrian movement."""

import math
from enum import Enum, auto

import numpy as np

from game.flight_state import Environment, FlightStatus, VehicleMode

RADIUS = 0.30
HEIGHT = 1.80
EYE_HEIGHT = 1.63
STEP_HEIGHT = 0.32
WALKABLE_Y = math.cos(math.radians(46))
UP = np.array((0.0, 1.0, 0.0))


class ControlMode(Enum):
    AIRCRAFT = auto()
    VEHICLE = AIRCRAFT  # M22 caller compatibility.
    ON_FOOT = auto()
    GROUND_VEHICLE = auto()


class PedestrianSpace:
    """Query the shared terrain and static obstacle index plus the parked hull."""

    def __init__(self, ground, vehicle):
        self.ground = ground
        self.vehicle = vehicle
        self.parked_bounds = None
        self.other_vehicles = []
        self.interactive_obstacles = None

    def vehicle_bounds(self, vehicle=None):
        vehicle = self.vehicle if vehicle is None else vehicle
        cached = self.parked_bounds if vehicle is self.vehicle else None
        lo, hi = cached or self.ground.body_bounds(vehicle)
        return vehicle.position + lo, vehicle.position + hi

    def boxes(self, lo, hi, exclude=None, *, include_registered=True):
        for index in self.ground.candidates(lo, hi):
            yield self.ground.boxes[index]
        if self.interactive_obstacles is not None:
            yield from self.interactive_obstacles(lo, hi)
        if exclude is not self.vehicle:
            yield self.vehicle_bounds()
        if include_registered:
            for access in self.other_vehicles:
                if access.vehicle is not exclude:
                    yield access.bounds(self)

    def blocked(self, feet, radius=RADIUS, height=HEIGHT, exclude=None):
        """Test an upright capsule via the distance from its axis to each box."""
        axis_lo = feet + UP * radius
        axis_hi = feet + UP * (height - radius)
        for lo, hi in self.boxes(
            axis_lo - radius, axis_hi + radius, exclude, include_registered=False
        ):
            gap = np.maximum(np.maximum(lo - axis_hi, axis_lo - hi), 0)
            if np.dot(gap, gap) < (radius - 1e-5) ** 2:
                return True
        return any(
            access.vehicle is not exclude
            and access.blocks_capsule(self, axis_lo, axis_hi, radius - 1e-5)
            for access in self.other_vehicles
        )

    def terrain(self, feet):
        # Account for the capsule's curved bottom near terrain edges.
        height = self.ground.height(feet)
        for offset in ((RADIUS, 0, 0), (-RADIUS, 0, 0), (0, 0, RADIUS), (0, 0, -RADIUS)):
            height = max(height, self.ground.height(feet + offset) - RADIUS)
        return height

    def support(self, feet, ceiling, exclude=None):
        """Highest reachable floor, including the top of a low solid step."""
        height = self.terrain(feet)
        lo = feet - np.array((RADIUS, HEIGHT, RADIUS))
        hi = feet + np.array((RADIUS, STEP_HEIGHT, RADIUS))
        for lower, upper in self.boxes(lo, hi, exclude):
            dx = max(lower[0] - feet[0], feet[0] - upper[0], 0)
            dz = max(lower[2] - feet[2], feet[2] - upper[2], 0)
            if upper[1] <= ceiling and dx * dx + dz * dz < RADIUS**2:
                height = max(height, upper[1])
        return height

    def clear_exit_path(self, start, end, exclude):
        samples = max(2, math.ceil(np.linalg.norm(end - start) / (RADIUS / 2)) + 1)
        for part in np.linspace(0, 1, samples):
            if self.blocked(start + (end - start) * part, exclude=exclude):
                return False
        return True

    def camera_distance(self, eye, back, distance, exclude=None):
        """Clip a camera sphere along the boom before entering an obstacle."""
        travel = 0.0
        for step in range(1, math.ceil(distance / 0.10) + 1):
            candidate = min(step * 0.10, distance)
            point = eye + back * candidate
            if point[1] - 0.15 < self.ground.height(point) or self.blocked(
                point - UP * 0.15, radius=0.15, height=0.30, exclude=exclude
            ):
                return travel
            travel = candidate
        return distance


class Pilot:
    """Own the capsule pose and fixed-step locomotion state."""

    def __init__(self, position, forward):
        self.position = np.array(position, dtype=float)
        self.velocity = np.zeros(3)
        self.forward = np.array(forward, dtype=float)
        self.grounded = True
        self.clip = "Idle"
        self.pose_revision = 0
        self.elapsed = 0.0
        self._accumulator = 0.0
        self._jump = False
        self._movement_input = None
        self._movement_direction = np.zeros(3)

    def movement_direction(self, strafe, advance):
        keys = (strafe, advance)
        if keys != self._movement_input:
            # Hold the requested direction as the pilot turns to face it.
            direction = self.forward * advance + np.cross(self.forward, UP) * strafe
            self._movement_direction = direction / max(1, np.linalg.norm(direction))
            self._movement_input = keys
        return self._movement_direction

    def update(self, dt, space, direction, run=False, jump=False):
        self._jump |= jump
        self._accumulator += max(dt, 0.0)
        step = 1 / 120
        while self._accumulator + 1e-10 >= step:
            self._step(step, space, direction, run, self._jump)
            self._jump = False
            self._accumulator -= step

    def _step(self, dt, space, direction, run, jump):
        desired = direction * (6 if run else 3)
        horizontal = (0, 2)
        self.velocity[[0, 2]] += (desired[[0, 2]] - self.velocity[[0, 2]]) * (-math.expm1(-18 * dt))
        if jump and self.grounded:
            self.velocity[1] = 5.2
            self.grounded = False
        before = self.position.copy()
        for axis in horizontal:
            trial = self.position.copy()
            trial[axis] += self.velocity[axis] * dt
            floor = space.support(
                trial, self.position[1] + STEP_HEIGHT if self.grounded else trial[1]
            )
            if floor > trial[1]:
                rise = floor - trial[1]
                if (
                    not self.grounded
                    or rise > STEP_HEIGHT
                    or (
                        floor == space.terrain(trial) and space.ground.normal(trial)[1] < WALKABLE_Y
                    )
                ):
                    self.velocity[axis] = 0
                    continue
                trial[1] = floor
            if not space.blocked(trial):
                self.position[:] = trial
            else:
                self.velocity[axis] = 0
        trial = self.position.copy()
        self.velocity[1] = max(self.velocity[1] - 9.81 * dt, -25)
        trial[1] += self.velocity[1] * dt
        floor = space.support(trial, self.position[1] + 1e-5)
        self.grounded = self.velocity[1] <= 0 and trial[1] <= floor
        if self.grounded:
            trial[1] = floor
            self.velocity[1] = 0
        if not space.blocked(trial):
            self.position[:] = trial
        else:
            self.velocity[1] = 0
        moved = self.position - before
        moved[1] = 0
        speed = np.linalg.norm(moved) / dt
        turn = False
        if np.linalg.norm(direction) > 1e-4:
            yaw = math.atan2(self.forward[0], self.forward[2])
            target = math.atan2(direction[0], direction[2])
            angle = math.remainder(target - yaw, math.tau)
            yaw += np.clip(angle, -9 * dt, 9 * dt)
            self.forward[:] = (math.sin(yaw), 0, math.cos(yaw))
            turn = abs(angle) > 0.2
        clip = "Run" if speed > 3.5 else "Walk" if speed > 0.15 else "Turn" if turn else "Idle"
        self.elapsed += dt
        self.clip = clip


def segment_box_distance_squared(start, end, lower, upper):
    """Minimum squared distance from a segment to a box in box-local coordinates."""
    delta = end - start
    # The nearest box face changes only when the segment crosses a face plane.
    breaks = [0.0, 1.0]
    for axis in np.flatnonzero(np.abs(delta) > 1e-12):
        for face in (lower[axis], upper[axis]):
            t = (face - start[axis]) / delta[axis]
            if 0 < t < 1:
                breaks.append(t)
    breaks.sort()
    nearest = float("inf")
    for left, right in zip(breaks, breaks[1:]):
        midpoint = start + delta * ((left + right) / 2)
        active = (midpoint < lower) | (midpoint > upper)
        if not active.any():
            return 0.0
        face = midpoint.clip(lower, upper)
        slope = delta[active]
        speed_squared = slope @ slope
        t = (
            np.clip((face[active] - start[active]) @ slope / speed_squared, left, right)
            if speed_squared > 0
            else left
        )
        point = start + delta * t
        gap = point - point.clip(lower, upper)
        nearest = min(nearest, gap @ gap)
    return nearest


class VehicleAccess:
    """Interaction contract for a persistent drivable entity."""

    mode = ControlMode.GROUND_VEHICLE
    reach = 2.4

    def __init__(self, vehicle, identity=None, label=None):
        self.vehicle = vehicle
        self.identity = vehicle.identity if identity is None else identity
        self.label = vehicle.label if label is None else label
        self.occupant = None

    def activate(self, pilot, space):
        self.occupant = pilot

    def deactivate(self, space):
        self.occupant = None

    def bounds(self, space):
        return self.vehicle.bounds()

    def blocks_capsule(self, space, axis_lo, axis_hi, radius):
        vehicle = self.vehicle
        pose = vehicle.model_matrix()
        axes = pose[:3, :3]
        lower = np.array(
            (
                -vehicle.half_width,
                min(vehicle.body_bottom, -vehicle.parameters.ride_height),
                -vehicle.half_length,
            )
        )
        upper = np.array((vehicle.half_width, vehicle.body_top, vehicle.half_length))
        center = pose[:3, 3] + axes @ ((lower + upper) / 2)
        extent = np.abs(axes) @ ((upper - lower) / 2)
        gap = np.maximum(np.maximum(center - extent - axis_hi, axis_lo - center - extent), 0)
        if gap @ gap >= radius**2:
            return False
        # A world-aligned box includes open corners beside a diagonally parked Jeep.
        local = (np.array([axis_lo, axis_hi]) - pose[:3, 3]) @ axes
        return segment_box_distance_squared(*local, lower, upper) < radius**2

    def exit_warning(self, space):
        if (
            abs(self.vehicle.speed) > self.vehicle.exit_speed
            or abs(self.vehicle.vertical_speed) > 0.4
        ):
            return "Stop the vehicle first"
        if not self.vehicle.safely_supported(space.ground):
            return "Vehicle needs stable wheel support"
        return ""

    def entry_warning(self, space):
        return "Vehicle is occupied" if self.occupant is not None else self.exit_warning(space)

    def entry_points(self, eye, space):
        return tuple(self.vehicle.anchor(f"Entry.{side}") for side in ("L", "R"))

    def exit_position(self, space):
        vehicle = self.vehicle
        for side, outward in (("R", -vehicle.side), ("L", vehicle.side)):
            door = vehicle.anchor(f"Entry.{side}")
            anchor = vehicle.anchor(f"Exit.{side}")
            sill_offset = vehicle.anchors[f"Entry.{side}"][1] - vehicle.anchors[f"Exit.{side}"][1]
            step_height = door[1] - (sill_offset - 0.38) * vehicle.model_matrix()[1, 1]
            for distance, along in ((0, 0), (0.45, 0), (0.25, 0.65), (0.25, -0.65)):
                position = anchor + distance * outward + along * vehicle.forward
                feet = self._exit_route(space, door, position, step_height)
                if feet is not None and not space.blocked(feet):
                    return feet, outward
        return None

    def _exit_route(self, space, door, destination, step_height):
        """Follow actual walkable support from the door rather than a level ray."""
        rise_limit, drop_limit = STEP_HEIGHT, 0.55
        previous = step_height
        for fraction in np.linspace(0, 1, 10):
            point = door + fraction * (destination - door)
            point[1] = previous
            terrain = space.terrain(point)
            floor = space.support(point, previous + rise_limit, exclude=self.vehicle)
            drop = drop_limit if fraction == 0 else STEP_HEIGHT
            if (
                floor < previous - drop
                or floor > previous + rise_limit
                or (floor <= terrain + 1e-5 and space.ground.normal(point)[1] < WALKABLE_Y)
            ):
                return None
            point[1] = previous = floor
            if space.blocked(point, exclude=self.vehicle):
                return None
        return point

    def park(self):
        self.vehicle.parking_brake = True


class AircraftAccess(VehicleAccess):
    mode = ControlMode.AIRCRAFT
    reach = 2.4

    def __init__(self, vehicle, exit_speed):
        super().__init__(vehicle, getattr(vehicle, "identity", "tc167"), "TC167")
        self.exit_speed = exit_speed
        self.entry_disabled = None

    def entry_warning(self, space):
        return self.entry_disabled or super().entry_warning(space)

    def activate(self, pilot, space):
        super().activate(pilot, space)
        if self.vehicle is space.vehicle:
            space.parked_bounds = None

    def deactivate(self, space):
        super().deactivate(space)
        if self.vehicle is space.vehicle:
            space.parked_bounds = space.ground.body_bounds(self.vehicle)

    def bounds(self, space):
        return space.vehicle_bounds(self.vehicle)

    def blocks_capsule(self, space, axis_lo, axis_hi, radius):
        lo, hi = self.bounds(space)
        gap = np.maximum(np.maximum(lo - axis_hi, axis_lo - hi), 0)
        return gap @ gap < radius**2

    def entry_points(self, eye, space):
        lo, hi = self.bounds(space)
        return (np.clip(eye, lo, hi),)

    def park(self):
        pass

    def exit_warning(self, space):
        vehicle = self.vehicle
        state = vehicle.flight_state
        if state.status is FlightStatus.CRASHED or getattr(vehicle, "destroyed", False):
            return "Vehicle is unsafe"
        if vehicle.transformation.active:
            return "Wait for transformation to finish"
        if (
            state.environment is not Environment.ATMOSPHERE
            or state.status is not FlightStatus.GROUNDED
        ):
            return "Land before entering or exiting"
        if vehicle.speed > self.exit_speed:
            return "Stop the vehicle first"
        from game.reference_anchors import floor

        support = floor(vehicle, space.ground.height)
        if support is None:
            from game.battledroid_physics import contact_clearance as battledroid_clearance
            from game.fighter_ground import clearance as fighter_clearance
            from game.vtol_ground import contact_clearance as vtol_clearance

            clearance = {
                VehicleMode.FIGHTER: fighter_clearance,
                VehicleMode.VTOL: vtol_clearance,
                VehicleMode.BATTLEDROID: battledroid_clearance,
            }[state.mode]
            support = space.ground.height(vehicle.position) + clearance(vehicle)
        if abs(vehicle.position[1] - support) > 0.5:
            return "Vehicle is not supported by the ground"
        return ""

    def exit_position(self, space):
        lo, hi = self.bounds(space)
        origin = self.vehicle.position
        right = self.vehicle.right.copy()
        right[1] = 0
        length = np.linalg.norm(right)
        right = right / length if length > 1e-6 else np.array((1.0, 0.0, 0.0))
        for angle in (
            0,
            math.pi,
            math.pi / 4,
            -math.pi / 4,
            3 * math.pi / 4,
            -3 * math.pi / 4,
            math.pi / 2,
            -math.pi / 2,
        ):
            dx, dz = right[0], right[2]
            direction = np.array(
                (
                    dx * math.cos(angle) - dz * math.sin(angle),
                    0,
                    dx * math.sin(angle) + dz * math.cos(angle),
                )
            )
            edge = min(
                ((hi if direction[i] > 0 else lo)[i] - origin[i]) / direction[i]
                for i in (0, 2)
                if abs(direction[i]) > 1e-6
            )
            for margin in (0.7, 1.4):
                feet = origin + direction * (edge + margin)
                feet[1] = space.terrain(feet)
                doorway = origin + direction * (edge + 0.05)
                doorway[1] = feet[1]
                if (
                    space.clear_exit_path(doorway, feet, self.vehicle)
                    and abs(feet[1] - space.ground.height(origin)) < 1.2
                    and space.ground.normal(feet)[1] >= WALKABLE_Y
                    and not space.blocked(feet)
                ):
                    return feet, direction
        return None


class PlayerControl:
    """Route one persistent pilot between registered vehicles and the shared world."""

    def __init__(self, vehicle, ground, exit_speed=0.35):
        self.vehicle = vehicle
        self.space = PedestrianSpace(ground, vehicle)
        self.aircraft = AircraftAccess(vehicle, exit_speed)
        self.registry = {self.aircraft.identity: self.aircraft}
        self.mode = ControlMode.AIRCRAFT
        self._pilot = Pilot(vehicle.position, vehicle.forward)
        self.pilot = None
        self.aircraft.activate(self._pilot, self.space)
        self.selected = None
        self.message = ""

    @property
    def active(self):
        if self.mode is ControlMode.ON_FOOT:
            return None
        if self.selected is not None and self.selected.mode is self.mode:
            return self.selected
        return self.aircraft

    def register(self, access):
        if access.identity in self.registry:
            raise ValueError(f"Duplicate vehicle identity: {access.identity}")
        self.registry[access.identity] = access
        self.space.other_vehicles.append(access)

    def exit_position(self):
        return (self.active or self.aircraft).exit_position(self.space)

    def nearby(self):
        pilot = self.pilot
        if pilot is None or not pilot.grounded:
            return []
        eye = pilot.position + UP * EYE_HEIGHT
        candidates = []
        for access in self.registry.values():
            nearest = min(
                access.entry_points(eye, self.space), key=lambda p: np.linalg.norm(p - eye)
            )
            distance = np.linalg.norm(nearest - eye)
            if distance <= access.reach:
                error = access.entry_warning(self.space)
                if not error:
                    hatch = nearest.copy()
                    hatch[1] = pilot.position[1]
                    if not self.space.clear_exit_path(hatch, pilot.position, access.vehicle):
                        error = "Vehicle access is blocked"
                candidates.append((distance, access, error))
        return sorted(candidates, key=lambda item: item[0])

    def prompt(self):
        if self.message:
            return self.message
        if self.mode is ControlMode.ON_FOOT:
            for _, access, error in self.nearby():
                if not error:
                    return f"ENTER TO BOARD {access.label.upper()}"
            return "ON FOOT"
        if self.mode is ControlMode.GROUND_VEHICLE:
            jeep = self.active.vehicle
            status = (
                "P RELEASE PARKING BRAKE"
                if jeep.parking_brake
                else "REVERSE"
                if jeep.speed < 0
                else "DRIVE"
            )
            return f"{jeep.label.upper()} / {abs(jeep.speed) * 3.6:.0f} KM/H / {status}"

    def interact(self):
        self.message = ""
        if self.mode is ControlMode.ON_FOOT:
            candidates = self.nearby()
            for _, access, error in candidates:
                if not error:
                    access.activate(self._pilot, self.space)
                    self.selected = access
                    self.pilot = None
                    self.mode = access.mode
                    return True
            self.message = candidates[0][2] if candidates else "Stand close to a vehicle to enter"
            return False
        access = self.active
        self.message = access.exit_warning(self.space)
        if self.message:
            return False
        placement = access.exit_position(self.space)
        if placement is None:
            self.message = "No safe exit beside the vehicle"
            return False
        feet, outward = placement
        forward = access.vehicle.forward.copy()
        forward[1] = 0
        length = np.linalg.norm(forward)
        self._pilot.position[:] = feet
        self._pilot.forward[:] = forward / length if length > 1e-6 else outward
        self._pilot.velocity[:] = 0
        self._pilot.grounded = True
        self._pilot.clip = "Idle"
        self._pilot.pose_revision += 1
        self._pilot._jump = False
        access.park()
        access.deactivate(self.space)
        self.mode = ControlMode.ON_FOOT
        self.pilot = self._pilot
        self.selected = None
        return True
