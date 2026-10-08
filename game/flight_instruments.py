"""Read-only SI flight instruments derived from world state and vehicle basis."""

import math

import numpy as np

from game.atmospheric_physics import PARAMETERS
from game.flight_state import Environment

STANDARD_GRAVITY = 9.80665


def heading_degrees(vehicle):
    forward = vehicle.forward
    if not np.isfinite(forward).all() or math.hypot(forward[0], forward[2]) < 1e-6:
        return None  # horizontal heading undefined when pointing vertically
    return math.degrees(math.atan2(forward[0], -forward[2])) % 360


def vertical_speed(vehicle):
    return float(vehicle.velocity[1])


def normal_g_load(vehicle):
    gravity = np.array((0.0, -PARAMETERS.gravity, 0.0)) if vehicle.flight_state.environment is Environment.ATMOSPHERE else np.zeros(3)
    # Specific force excludes free-fall gravity; local-up sign preserves negative G.
    return float(np.dot(vehicle.acceleration - gravity, vehicle.up) / STANDARD_GRAVITY)


def attitude_angles(vehicle):
    """Pitch and bank in radians relative to world +Y, including inverted attitude.

    At vertical pitch, bank is conventionally undefined: return None rather
    than invent a horizon reference. This ambiguity is instrumentation only.
    """
    pitch = math.asin(float(np.clip(vehicle.forward[1], -1, 1)))
    if math.hypot(vehicle.forward[0], vehicle.forward[2]) < 1e-6:
        return pitch, None
    bank = math.atan2(-vehicle.right[1], vehicle.up[1])
    return pitch, bank
