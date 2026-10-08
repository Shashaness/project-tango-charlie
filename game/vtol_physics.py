"""Small VTOL configuration parameters and reusable force helpers (SI)."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class VTOLParameters:
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
    return thrust_direction(vehicle, vector) * VTOL.max_thrust * np.clip(throttle, 0, 1)


def maneuver_force(vehicle, controls=None):
    if controls is None:
        return np.zeros(3, dtype=np.float64)
    return (vehicle.right * controls.strafe + vehicle.up * controls.lift) * VTOL.maneuver_thrust
