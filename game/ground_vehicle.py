"""Wheel-supported keyboard driving in the shared meter-space world."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DriveParameters:
    mass: float = 1450.0
    drive_force: float = 6400.0
    brake_force: float = 13000.0
    steering_angle: float = math.radians(34)
    wheelbase: float = 2.65
    track: float = 1.65
    wheel_radius: float = 0.43
    suspension: float = 0.28
    spring_rate: float = 38000.0
    damping: float = 4800.0
    traction: float = 0.92
    max_forward: float = 21.0
    max_reverse: float = 6.0

    @property
    def ride_height(self):
        return self.wheel_radius + self.suspension - self.mass * 9.81 / (4 * self.spring_rate)


class GroundVehicle:
    """A persistent four-wheel vehicle with a supported chassis and signed road speed."""

    half_width = 1.04
    half_length = 2.11
    body_bottom = -0.18
    body_top = 1.02
    exit_speed = 0.25

    def __init__(
        self,
        identity,
        position,
        heading,
        parameters=DriveParameters(),
        label="Jeep",
        anchors=None,
    ):
        self.identity = identity
        self.label = label
        self.position = np.array(position, dtype=float)
        self.velocity = np.zeros(3)
        self.heading = heading
        self.parameters = parameters
        self.anchors = anchors or {
            "DriverSeat": (0, 0.38, 0),
            "Entry.L": (self.half_width, 0.65, 0),
            "Entry.R": (-self.half_width, 0.65, 0),
            "Exit.L": (self.half_width + 0.55, 0, 0),
            "Exit.R": (-self.half_width - 0.55, 0, 0),
        }
        self.speed = self.vertical_speed = self.steering = 0.0
        self.pitch = self.roll = 0.0
        self.wheel_spin = np.zeros(4)
        self.wheel_heights = np.full(4, parameters.wheel_radius - parameters.ride_height)
        self.contacts = np.ones(4, dtype=bool)
        self.parking_brake = True
        self._time = 0.0

    @property
    def forward(self):
        return np.array((math.sin(self.heading), 0.0, math.cos(self.heading)))

    @property
    def side(self):
        return np.array((math.cos(self.heading), 0.0, -math.sin(self.heading)))

    @property
    def axles(self):
        p = self.parameters
        return np.array(
            (
                (p.track / 2, p.wheelbase / 2),
                (-p.track / 2, p.wheelbase / 2),
                (p.track / 2, -p.wheelbase / 2),
                (-p.track / 2, -p.wheelbase / 2),
            )
        )

    def model_matrix(self):
        matrix = np.eye(4)
        forward = self.forward.copy()
        forward[1] = math.tan(self.pitch)
        forward /= np.linalg.norm(forward)
        side = self.side.copy()
        side[1] = math.tan(self.roll)
        side -= np.dot(forward, side) * forward
        side /= np.linalg.norm(side)
        matrix[:3, :3] = np.column_stack((side, np.cross(forward, side), forward))
        matrix[:3, 3] = self.position
        return matrix

    def bounds(self):
        # Pedestrians and other drivers query the conservative world AABB.
        basis = self.model_matrix()[:3, :3]
        center_y = (self.body_top + self.body_bottom) / 2
        center = self.position + basis[:, 1] * center_y
        half_height = (self.body_top - self.body_bottom) / 2
        extents = np.abs(basis) @ (self.half_width, half_height, self.half_length)
        lower, upper = center - extents, center + extents
        lower[1] = min(lower[1], self.position[1] - self.parameters.ride_height)
        return lower, upper

    def place(self, ground):
        self.position[1] = ground.height(self.position) + self.parameters.ride_height

    def _heights(self, ground):
        offsets = self.axles[:, :1] * self.side + self.axles[:, 1:] * self.forward
        return np.array([ground.height(self.position + offset) for offset in offsets])

    def _suspension(self, heights):
        p = self.parameters
        offsets = self.axles @ np.tan((self.roll, self.pitch))
        centers = self.position[1] + offsets
        compression = np.clip(heights + p.wheel_radius + p.suspension - centers, 0, p.suspension)
        return offsets, compression

    def safely_supported(self, ground):
        """Check live suspension load and whether the wheel footprint spans the center."""
        if not self.contacts.any() or math.hypot(self.pitch, self.roll) > math.radians(32):
            return False
        _, compression = self._suspension(self._heights(ground))
        load = compression * self.parameters.spring_rate
        p = self.parameters
        supporting = load > 0.03 * p.mass * 9.81
        # Three corners of the rectangle support its center. A two-wheel line
        # can carry weight but leaves the chassis free to rock.
        return supporting.sum() >= 3 and load.sum() >= 0.70 * p.mass * 9.81

    def anchor(self, name):
        matrix = self.model_matrix()
        return matrix[:3, :3] @ self.anchors[name] + matrix[:3, 3]

    def _collides(self, space, previous_position):
        # Sweep the oriented footprint on both rectangle axes and the box axes.
        # Rotation is bounded by the fixed step; use the end heading for the sweep.
        r, f = self.side[[0, 2]], self.forward[[0, 2]]
        extent = self.half_width * np.abs(r) + self.half_length * np.abs(f)
        lo, hi = self.bounds()
        displacement = self.position - previous_position
        lo, hi = np.minimum(lo, lo - displacement), np.maximum(hi, hi - displacement)
        center = previous_position[[0, 2]]
        axes = np.array(((1, 0), (0, 1), r, f))
        footprint = np.array((*extent, self.half_width, self.half_length))
        speeds = axes @ displacement[[0, 2]]
        for lower, upper in space.boxes(lo, hi, exclude=self):
            if lower[1] >= hi[1] or upper[1] <= lo[1] + 1e-5:
                continue
            half = (upper - lower)[[0, 2]] / 2
            offsets = axes @ ((upper + lower)[[0, 2]] / 2 - center)
            margins = footprint + np.abs(axes) @ half
            enter, leave = 0.0, 1.0
            for offset, margin, speed in zip(offsets, margins, speeds):
                if abs(speed) < 1e-10:
                    if abs(offset) >= margin:
                        break
                else:
                    first, last = sorted(((offset - margin) / speed, (offset + margin) / speed))
                    enter, leave = max(enter, first), min(leave, last)
                    if enter >= leave:
                        break
            else:
                return True
        return False

    def update(self, elapsed, space, throttle=0, steer=0, brake=False):
        self._time += max(elapsed, 0)
        step = 1 / 120
        while self._time + 1e-10 >= step:
            self._step(step, space, throttle, steer, brake)
            self._time -= step

    def _step(self, dt, space, throttle, steer, brake):
        p = self.parameters
        old_position, old_heading = self.position.copy(), self.heading
        ground = space.ground
        heights = self._heights(ground)
        axle_pitch = (heights[:2].mean() - heights[2:].mean()) / p.wheelbase
        axle_roll = (heights[::2].mean() - heights[1::2].mean()) / p.track
        smoothing = -math.expm1(-10 * dt)
        self.pitch += (math.atan(axle_pitch) - self.pitch) * smoothing
        self.roll += (math.atan(axle_roll) - self.roll) * smoothing

        offsets, compression = self._suspension(heights)
        self.contacts = compression > 1e-6
        wheel_force = np.maximum(0, p.spring_rate * compression - p.damping * self.vertical_speed)
        self.vertical_speed += (wheel_force[self.contacts].sum() / p.mass - 9.81) * dt
        self.position[1] += self.vertical_speed * dt
        # A bump stop prevents the chassis from dropping through a sharp crest.
        floor = float(np.max(heights + p.wheel_radius - offsets))
        if self.position[1] < floor:
            self.position[1] = floor
            self.vertical_speed = max(0, self.vertical_speed)
        self.wheel_heights = np.clip(
            heights + p.wheel_radius - self.position[1],
            offsets - p.suspension,
            offsets + p.suspension,
        )

        resistance = ground.resistance(self.position)
        traction = p.traction * np.clip(1 - 4 * resistance, 0.35, 1) * self.contacts.mean()
        service_braking = brake or (abs(self.speed) > 0.2 and throttle * self.speed < 0)
        if (self.parking_brake or service_braking) and self.contacts.any():
            deceleration = min(p.brake_force / p.mass, 9.81 * traction)
            self.speed = math.copysign(max(abs(self.speed) - deceleration * dt, 0), self.speed)
        else:
            acceleration = throttle * min(p.drive_force / p.mass, traction * 9.81)
            if self.contacts.any():
                acceleration -= 9.81 * math.sin(self.pitch)
            coast = (resistance * 9.81 if self.contacts.any() else 0) + 0.018 * self.speed**2
            speed = self.speed + acceleration * dt
            self.speed = math.copysign(max(0, abs(speed) - coast * dt), speed)
        self.speed = float(np.clip(self.speed, -p.max_reverse, p.max_forward))

        target_steer = -steer * p.steering_angle / (1 + (abs(self.speed) / 9) ** 2)
        self.steering += (target_steer - self.steering) * smoothing
        curvature = math.tan(self.steering) / p.wheelbase
        turn = self.speed * curvature * min(1, 2 * traction) * dt
        self.heading += turn / 2
        travel = self.speed * dt
        self.position += self.forward * travel
        self.heading = math.remainder(self.heading + turn / 2, math.tau)

        if self._collides(space, old_position):
            self.position[:] = old_position
            self.heading = old_heading
            self.speed = self.vertical_speed = travel = 0.0
        self.wheel_spin = np.remainder(
            self.wheel_spin + travel * (1 - curvature * self.axles[:, 0]) / p.wheel_radius,
            math.tau,
        )
        self.velocity[:] = (self.position - old_position) / dt
