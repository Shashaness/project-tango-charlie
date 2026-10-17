"""Battledroid-only forces/contact/locomotion; vehicle reference is its center of mass."""

import math
from dataclasses import dataclass, field

import numpy as np

from game.atmospheric_physics import PARAMETERS, AeroForces
from game.flight_state import Environment


@dataclass(frozen=True)
class BattledroidParameters:
    """Battledroid tuning in SI units; _deg angles/rates use degrees and degrees/s."""

    height: float = 5.4
    foot_clearance: float = 3.2  # COM to sole when upright (head reaches +2.2)
    body_half_width: float = 1.4
    body_half_depth: float = 1.2
    thrust_to_weight: float = 1.8  # bound on combined atmospheric propulsion
    drag_coefficient: float = 1.0
    frontal_area: float = 30.0
    maneuver_acceleration: float = 8.0
    space_forward_acceleration: float = 16.0
    pitch_rate_deg: float = 70.0
    roll_rate_deg: float = 90.0
    yaw_rate_deg: float = 70.0
    walk_speed: float = 6.0
    reverse_speed: float = 3.0
    strafe_speed: float = 4.0
    ground_acceleration: float = 3.0
    ground_deceleration: float = 4.0
    ground_response: float = 0.8
    friction_damping: float = 1.5
    ground_turn_rate_deg: float = 45.0
    ground_angular_acceleration_deg: float = 90.0
    stabilization_gain: float = 3.0
    stabilization_rate_deg: float = 35.0
    safe_landing_vertical_speed: float = 7.0
    crash_vertical_speed: float = 35.0
    safe_landing_tilt_deg: float = 45.0
    jump_duration: float = 0.5
    space_damping: float = 0.35
    space_brake_damping: float = 2.0


BATTLEDROID = BattledroidParameters()


@dataclass
class GroundContact:
    """Mutable contact telemetry in meters, m/s, seconds, newtons, and unit normals."""

    depth: float = 0.0
    normal: np.ndarray = field(default_factory=lambda: np.array((0.0, 1.0, 0.0)))
    friction_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    reaction_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    desired_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    touchdown_id: int = 0
    touchdown_impact_speed: float = 0.0
    impact_speed: float = 0.0
    hard_landing_time: float = 0.0
    walk_phase: float = 0.0
    left_foot_height: float = 0.0
    right_foot_height: float = 0.0
    clearance: float = 0.0
    landing_approach: bool = False


def contact_clearance(vehicle, parameters=BATTLEDROID):
    """Find orientation-dependent center-of-mass clearance over the support plane.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        parameters: Configuration parameters; defaults retain the simulation tuning.

    Returns:
        Required vertical clearance in meters.
    """
    from game.ground_contact import transformation_clearance

    transition = transformation_clearance(vehicle)
    if transition is not None:
        return transition
    # Conservative oriented body support box; upright sole is exactly -3.2 m.
    return (
        abs(float(vehicle.up[1])) * parameters.foot_clearance
        + abs(float(vehicle.right[1])) * parameters.body_half_width
        + abs(float(vehicle.forward[1])) * parameters.body_half_depth
    )


def forces(vehicle, controls, throttle=None, velocity=None, parameters=BATTLEDROID):
    """Evaluate configuration-specific propulsion, drag, and gravity without moving the vehicle.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        controls: Pilot command fields; signed axes are normalized to [-1, 1].
        throttle: Engine throttle fraction in [0, 1].
        velocity: World velocity vector in m/s.
        parameters: Configuration parameters; defaults retain the simulation tuning.

    Returns:
        AeroForces with world-space forces in newtons; wing lift remains zero.
    """
    result = AeroForces()
    velocity = vehicle.velocity if velocity is None else np.asarray(velocity)
    throttle = vehicle.throttle if throttle is None else throttle
    if (
        not np.isfinite(velocity).all()
        or not np.isfinite(vehicle.orientation).all()
        or not math.isfinite(throttle)
    ):
        raise ValueError("Battledroid force state must be finite.")
    result.airspeed = float(np.linalg.norm(velocity))
    atmosphere = vehicle.flight_state.environment is Environment.ATMOSPHERE
    maneuvers = (
        (vehicle.right * controls.strafe + vehicle.up * controls.lift)
        * parameters.maneuver_acceleration
        * PARAMETERS.mass
    )
    if atmosphere:
        result.gravity_force = np.array((0.0, -PARAMETERS.mass * PARAMETERS.gravity, 0.0))
        maximum = PARAMETERS.mass * PARAMETERS.gravity * parameters.thrust_to_weight
        thrust = vehicle.up * maximum * np.clip(throttle, 0, 1)
        magnitude = float(np.linalg.norm(thrust + maneuvers))
        scale = min(1.0, maximum / max(magnitude, 1e-8))
        result.thrust_force = thrust * scale
        result.maneuver_force = maneuvers * scale
        result.dynamic_pressure = 0.5 * PARAMETERS.air_density * result.airspeed**2
        result.drag_coefficient = parameters.drag_coefficient
        if result.airspeed > 1e-8:
            result.drag_force = (
                -velocity
                / result.airspeed
                * result.dynamic_pressure
                * parameters.frontal_area
                * parameters.drag_coefficient
            )
    else:
        result.thrust_force = (
            vehicle.forward
            * PARAMETERS.mass
            * parameters.space_forward_acceleration
            * np.clip(throttle, 0, 1)
        )
        result.maneuver_force = maneuvers
    return result  # no wing lift, in either environment


def walking_acceleration(vehicle, controls, dt, parameters=BATTLEDROID):
    """Bound horizontal walking response using the current motion and pilot axes.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        controls: Pilot command fields; signed axes are normalized to [-1, 1].
        dt: Elapsed simulation time in seconds.
        parameters: Configuration parameters; defaults retain the simulation tuning.

    Returns:
        Pair of desired world velocity in m/s and bounded acceleration in m/s².
    """
    forward = vehicle.forward.copy()
    forward[1] = 0
    length = np.linalg.norm(forward)
    forward = forward / length if length > 1e-8 else np.array((0.0, 0.0, -1.0))
    right = np.cross(forward, np.array((0.0, 1.0, 0.0)))
    walk = -controls.pitch
    strafe = controls.roll
    desired = (
        forward * walk * (parameters.walk_speed if walk >= 0 else parameters.reverse_speed)
        + right * strafe * parameters.strafe_speed
    )
    length = np.linalg.norm(desired)
    if length > parameters.walk_speed:
        desired *= parameters.walk_speed / length
    horizontal = vehicle.velocity.copy()
    horizontal[1] = 0
    moving = abs(walk) + abs(strafe) > 1e-8
    acceleration = (
        (desired - horizontal) / parameters.ground_response
        if moving
        else -horizontal * parameters.friction_damping
    )
    limit = parameters.ground_acceleration if moving else parameters.ground_deceleration
    length = np.linalg.norm(acceleration)
    if length > limit:
        acceleration *= limit / length
    return desired, acceleration
