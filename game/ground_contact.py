"""Shared flat-ground penetration/normal-impulse resolution; no tangential reset."""

import math

import numpy as np

from game.flight_state import FlightStatus, VehicleMode


def resolve_contact(
    vehicle,
    clearance,
    *,
    supported,
    safe_speed,
    crash_speed,
    safe_tilt,
    crash_tilt=180.0,
    crash_horizontal_speed=math.inf,
    release_epsilon=0.03,
    hysteresis=False,
    hard_speed=math.inf,
):
    """Resolve zero-restitution ground contact, updating motion, status, and landing telemetry.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        clearance: Required vehicle reference altitude or clearance in meters.
        supported: Whether the vehicle had support before this integration step.
        safe_speed: Safe landing vertical speed limit in m/s.
        crash_speed: Crash vertical speed limit in m/s.
        safe_tilt: Safe body tilt limit in degrees.
        crash_tilt: Crash body tilt limit in degrees.
        crash_horizontal_speed: Horizontal landing crash threshold in m/s.
        release_epsilon: Liftoff clearance tolerance in meters.
        hysteresis: Whether to use supported-state liftoff hysteresis.
        hard_speed: Vertical speed threshold for hard-landing telemetry in m/s.
    """
    v = vehicle
    ground = v.ground_contact
    ground.depth = 0.0
    if v.position[1] <= clearance:
        ground.depth = max(0.0, clearance - v.position[1])
        v.position[1] = clearance
        if v.velocity[1] <= 0:
            impact = max(0.0, -float(v.velocity[1]))
            horizontal = float(np.linalg.norm(v.velocity[[0, 2]]))
            v.velocity[1] = 0.0  # normal collision impulse only
            tilt = math.degrees(math.acos(float(np.clip(v.up[1], -1, 1))))
            if not supported:
                if v.flight_state.status is FlightStatus.FLYING:
                    ground.touchdown_id += 1
                    ground.touchdown_impact_speed = impact
                ground.impact_speed = impact
                if impact > safe_speed or tilt > safe_tilt:
                    ground.hard_landing_time = 5.0 if impact >= hard_speed else 3.0
            crashed = (
                impact > crash_speed or tilt > crash_tilt or horizontal > crash_horizontal_speed
            )
            v.flight_state.status = FlightStatus.CRASHED if crashed else FlightStatus.GROUNDED
        elif not hysteresis:
            v.flight_state.status = FlightStatus.FLYING
    elif v.position[1] > clearance + release_epsilon or (not hysteresis and v.velocity[1] > 0):
        v.flight_state.status = FlightStatus.FLYING


def transformation_clearance(vehicle):
    """Evaluate the canonical support envelope during VTOL/battledroid deployment.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Clearance in meters, or None outside the relevant active transition.
    """
    c = getattr(vehicle, "transformation", None)
    if c is None or not c.active or c._edge != (VehicleMode.VTOL, VehicleMode.BATTLEDROID):
        return None
    from game.battledroid_physics import BATTLEDROID
    from game.transformation import phase
    from game.vtol_ground import foot_geometry

    u = phase(c._coordinate)
    return (1 - u) * foot_geometry()[1] + u * BATTLEDROID.foot_clearance
