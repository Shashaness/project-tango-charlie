"""Operational wheel support and runway safety; existing force integration is retained."""

from functools import lru_cache

import numpy as np

from game.atmospheric_physics import PARAMETERS
from game.flight_state import FlightStatus


@lru_cache(maxsize=1)
def fighter_vertices():
    """Load and cache fighter rest-pose geometry for conservative hull queries.

    Returns:
        Read-only array of model-space XYZ vertices in meters.
    """
    from game.transformation import TransformationController

    c = TransformationController()
    points = []
    for node in c.model.nodes:
        for index in node.meshes:
            p = c.model.primitives[index].positions
            points.extend((node.world_matrix @ np.column_stack((p, np.ones(len(p)))).T).T[:, :3])
    points = np.asarray(points)
    points.setflags(write=False)
    return points


def clearance(vehicle):
    """Find the lowest oriented gear or body point relative to flat ground.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Required center-of-mass altitude in meters.
    """
    from game.reference_anchors import clearance as anchor_clearance

    value = anchor_clearance(vehicle)
    if value is not None:
        return value + PARAMETERS.ground_altitude
    from game.airfield_operations import GearState

    ops = vehicle.airfield
    if ops.gear is GearState.DEPLOYED:
        points = np.asarray(ops.parameters.wheel_points)
    else:
        wheels = np.asarray(ops.parameters.wheel_points).copy()
        wheels[:, 1] *= ops.extension
        points = np.concatenate((fighter_vertices(), wheels))
    return -float((points @ vehicle.orientation.T)[:, 1].min()) + PARAMETERS.ground_altitude


class FighterGroundContact:
    """Coordinate wheel support, load-dependent friction, steering, and landing safety."""

    def __init__(self, vehicle):
        """Bind the vehicle and initialize wheel support and steering state.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        """
        self.vehicle = vehicle
        self.supported = False
        self.environment = None
        self.steering = 0.0
        self.ground_height = 0.0
        self.normal = np.array((0.0, 1.0, 0.0))

    def on_runway(self):
        """Check whether deployed wheels, or the vehicle origin, lie inside the runway.

        Returns:
            Whether all tested points lie on the runway.
        """
        from game.airfield_operations import GearState
        from game.world_environment import RUNWAY

        v = self.vehicle
        points = v.airfield.wheel_world(v) if v.airfield.gear is GearState.DEPLOYED else v.position
        return bool(np.all(RUNWAY.contains(points)))

    def floor(self):
        """Query the minimum vehicle altitude that clears local wheel/body support.

        Returns:
            Required vehicle Y position in meters.
        """
        v = self.vehicle
        from game.reference_anchors import authored, floor

        if authored(v):
            height = (
                self.environment.height
                if self.environment is not None
                else lambda p: PARAMETERS.ground_altitude
            )
            return floor(v, height)
        if self.environment is None:
            return clearance(v)
        points = v.airfield.wheel_world(v)
        offsets = points - v.position
        return (
            max(
                self.environment.height(point) - offset[1] for point, offset in zip(points, offsets)
            )
            if v.airfield.extension == 1
            else self.environment.height(v.position) + clearance(v)
        )

    def begin(self):
        """Reset contact forces and sample support normal and deployed-wheel support."""
        from game.airfield_operations import GearState

        v = self.vehicle
        g = v.ground_contact
        ops = v.airfield
        g.reaction_force[:] = 0
        g.friction_force[:] = 0
        ops.brakes_active = False
        self.normal = (
            self.environment.normal(v.position)
            if self.environment is not None
            else np.array((0.0, 1.0, 0.0))
        )
        g.normal[:] = self.normal
        self.supported = (
            ops.gear is GearState.DEPLOYED
            and v.flight_state.status is FlightStatus.GROUNDED
            and v.position[1] <= self.floor() + ops.parameters.contact_epsilon
            and float(v.velocity @ self.normal) <= 0.001
        )

    def acceleration(self, forces, velocity, dt):
        """Apply wheel reaction, rolling resistance, braking, and bounded lateral tire forces.

        Args:
            forces: Force telemetry with world-space force vectors in newtons.
            velocity: World velocity vector in m/s.
            dt: Elapsed simulation time in seconds.

        Returns:
            World acceleration vector in m/s²; updates force/braking telemetry.
        """
        force = forces.total_force.copy()
        g = self.vehicle.ground_contact
        if self.supported:
            p = self.vehicle.airfield.parameters
            load = max(0.0, -float(force @ self.normal))
            g.reaction_force[:] = self.normal * load
            force += g.reaction_force
            tangent = np.asarray(velocity).copy()
            tangent -= self.normal * (tangent @ self.normal)
            speed = np.linalg.norm(tangent)
            # Brake strength follows actual normal wheel load; airborne lift unloads it.
            coefficient = (
                self.environment.resistance(self.vehicle.position)
                if self.environment is not None
                else p.rolling_resistance
            )
            resistance = coefficient * load
            if self.vehicle.airfield.brakes and load > 0:
                resistance += min(p.brake_friction * load, PARAMETERS.mass * p.brake_deceleration)
                self.vehicle.airfield.brakes_active = True
            if speed > 1e-8:
                friction = (
                    -tangent / speed * min(resistance, PARAMETERS.mass * speed / max(dt, 1e-8))
                )
                force += friction
                g.friction_force[:] = friction
            # Tire side force bends ground velocity toward the steered aircraft direction.
            from game.ground_environment import GROUND_HANDLING as handling

            if self.environment is not None:
                handling = self.environment.parameters
            forward = self.vehicle.forward.copy()
            forward -= self.normal * (forward @ self.normal)
            forward /= max(np.linalg.norm(forward), 1e-8)
            right = np.cross(forward, self.normal)
            along = float(tangent @ forward)
            lateral = float(tangent @ right)
            angle = (
                np.radians(handling.steering_angle_deg)
                * self.steering
                / (1 + (speed / handling.steering_fade_speed) ** 2)
            )
            target_rate = along * np.tan(angle) / handling.wheelbase
            lateral_force = PARAMETERS.mass * (-lateral / max(dt, 1e-8) - along * target_rate)
            tire = right * np.clip(
                lateral_force, -handling.lateral_grip * load, handling.lateral_grip * load
            )
            force += tire
            g.friction_force += tire
        return force / PARAMETERS.mass

    def steer_rate(self, dt, command):
        """Integrate load-limited yaw response toward a speed-dependent steering target.

        Args:
            dt: Elapsed simulation time in seconds.
            command: Normalized steering command in [-1, 1].

        Returns:
            Updated yaw rate in radians/s; also stores the normalized steering input.
        """
        from game.ground_environment import GROUND_HANDLING as p

        if self.environment is not None:
            p = self.environment.parameters
        v = self.vehicle
        self.steering = float(np.clip(command, -1, 1))
        speed = float(np.dot(v.velocity, v.forward))
        angle = (
            np.radians(p.steering_angle_deg)
            * self.steering
            / (1 + (speed / p.steering_fade_speed) ** 2)
        )
        target = speed * np.tan(angle) / p.wheelbase
        load = float(np.linalg.norm(v.ground_contact.reaction_force))
        if load == 0:
            load = PARAMETERS.mass * PARAMETERS.gravity
        requested_torque = p.yaw_inertia * (target - v.yaw_rate) * p.yaw_response
        torque = np.clip(
            requested_torque,
            -p.lateral_grip * load * p.wheelbase / 2,
            p.lateral_grip * load * p.wheelbase / 2,
        )
        return v.yaw_rate + torque / p.yaw_inertia * dt

    def crash(self, reason):
        """Record a landing failure and stop vehicle motion and active propulsion.

        Args:
            reason: Human-readable crash reason stored on airfield state.
        """
        v = self.vehicle
        v.airfield.crash_reason = reason
        v.flight_state.status = FlightStatus.CRASHED
        v.velocity[:] = 0
        v.acceleration[:] = 0
        v.pitch_rate = v.yaw_rate = v.roll_rate = 0.0
        v.engine_throttle = 0.0
        v.aerodynamics = type(v.aerodynamics)()
        v.airfield.brakes_active = False

    def finish(self):
        """Resolve touchdown clearance, classify landing safety, and update contact state."""
        import math

        from game.airfield_operations import GearState

        v = self.vehicle
        g = v.ground_contact
        ops = v.airfield
        p = ops.parameters
        floor = self.floor()
        g.clearance = floor - PARAMETERS.ground_altitude
        g.depth = 0.0
        # A lifting, supported aircraft may cross an end line during liftoff.
        normal_speed = float(v.velocity @ self.normal)
        contact = v.position[1] <= floor + p.contact_epsilon and normal_speed <= 0.001
        if not contact:
            if v.position[1] > floor + p.contact_epsilon or normal_speed > 0.001:
                v.flight_state.status = FlightStatus.FLYING
            return
        speed = float(np.linalg.norm(v.velocity[[0, 2]]))
        impact = max(0.0, -float(v.velocity @ self.normal))
        pitch = math.degrees(math.asin(float(np.clip(v.forward[1], -1, 1))))
        roll = math.degrees(math.atan2(-v.right[1], v.up[1]))
        body_floor = -float((fighter_vertices() @ v.orientation.T)[:, 1].min()) + (
            self.environment.height(v.position)
            if self.environment is not None
            else PARAMETERS.ground_altitude
        )
        reason = None
        if ops.gear is not GearState.DEPLOYED:
            reason = "GEAR NOT DOWN"
        elif impact > p.hard_landing_speed:
            reason = "HARD LANDING"
        elif abs(pitch) > p.max_pitch_deg or abs(roll) > p.max_roll_deg:
            reason = "UNSAFE ATTITUDE"
        elif body_floor > floor + p.body_margin:
            reason = "BODY STRIKE"
        elif speed > p.max_touchdown_speed and not self.supported:
            reason = "TOUCHDOWN SPEED"
        elif self.environment is not None and not self.environment.landing_suitable(v):
            reason = "UNSAFE SLOPE"
        g.depth = max(0.0, floor - v.position[1])
        v.position[1] = max(v.position[1], floor)
        if reason:
            self.crash(reason)
            return
        if v.flight_state.status is FlightStatus.FLYING:
            g.touchdown_id += 1
            g.touchdown_impact_speed = impact
        g.impact_speed = impact
        v.velocity -= self.normal * min(0.0, float(v.velocity @ self.normal))
        v.flight_state.status = FlightStatus.GROUNDED
