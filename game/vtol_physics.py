"""Small VTOL configuration parameters and reusable force helpers (SI)."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VTOLParameters:
    """Immutable VTOL propulsion tuning in newtons and angular rates in degrees/s."""

    max_thrust: float = 110000.0  # N; 1.87 times the 6000 kg vehicle's weight
    wing_lift_factor: float = 0.30
    drag_factor: float = 3.5
    maneuver_thrust: float = 48000.0  # N per lateral/vertical axis
    vector_rate: float = 0.5  # fraction / second
    pitch_rate_deg: float = 60.0
    roll_rate_deg: float = 90.0
    yaw_rate_deg: float = 60.0


VTOL = VTOLParameters()


def thrust_direction(vehicle, vector=None):
    """Blend forward/up axes and normalize the vectored engine direction.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        vector: VTOL thrust-vector fraction; None uses the vehicle value.

    Returns:
        Unit world-space thrust direction.
    """
    vector = vehicle.thrust_vector if vector is None else vector
    if not math.isfinite(vector):
        raise ValueError("Thrust vector must be finite.")
    vector = float(np.clip(vector, 0, 1))
    direction = vehicle.forward * (1 - vector) + vehicle.up * vector
    length = float(np.linalg.norm(direction))
    if not math.isfinite(length) or length < 1e-8:
        raise ValueError("Thrust direction requires a finite nonzero vehicle basis.")
    return direction / length


def engine_force(vehicle, throttle, vector=None):
    """Evaluate vectored VTOL thrust at the requested throttle.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        throttle: Engine throttle fraction in [0, 1].
        vector: VTOL thrust-vector fraction; None uses the vehicle value.

    Returns:
        World thrust vector in newtons.
    """
    return thrust_direction(vehicle, vector) * VTOL.max_thrust * np.clip(throttle, 0, 1)


def maneuver_force(vehicle, controls=None):
    """Evaluate local lateral and vertical maneuvering thrusters.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
        controls: Pilot command fields; signed axes are normalized to [-1, 1].

    Returns:
        World force vector in newtons, or zeros when controls are absent.
    """
    if controls is None:
        return np.zeros(3, dtype=np.float64)
    return (vehicle.right * controls.strafe + vehicle.up * controls.lift) * VTOL.maneuver_thrust
