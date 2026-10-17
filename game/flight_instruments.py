"""Read-only SI flight instruments derived from world state and vehicle basis."""

import math

import numpy as np

from game.atmospheric_physics import PARAMETERS
from game.flight_state import Environment

STANDARD_GRAVITY = 9.80665


def heading_degrees(vehicle):
    """Measure horizontal heading clockwise from geographic north.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Heading in [0, 360) degrees, or None for invalid/vertical direction.
    """
    forward = vehicle.forward
    frame = getattr(vehicle, "geographic_frame", None)
    east, north, _ = (
        frame.horizontal_basis()
        if frame is not None
        else (np.array((1.0, 0.0, 0.0)), np.array((0.0, 0.0, -1.0)), np.array((0.0, 1.0, 0.0)))
    )
    e, n = float(np.dot(forward, east)), float(np.dot(forward, north))
    if not np.isfinite(forward).all() or math.hypot(e, n) < 1e-6:
        return None  # horizontal heading undefined when pointing vertically
    return math.degrees(math.atan2(e, n)) % 360


def vertical_speed(vehicle):
    """Read world-up velocity without altering motion.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Vertical speed in m/s, positive upward.
    """
    return float(vehicle.velocity[1])


def normal_g_load(vehicle):
    """Project nongravitational specific force onto vehicle up.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Signed normal acceleration in standard-gravity units.
    """
    gravity = (
        np.array((0.0, -PARAMETERS.gravity, 0.0))
        if vehicle.flight_state.environment is Environment.ATMOSPHERE
        else np.zeros(3)
    )
    # Specific force excludes free-fall gravity; local-up sign preserves negative G.
    return float(np.dot(vehicle.acceleration - gravity, vehicle.up) / STANDARD_GRAVITY)


def attitude_angles(vehicle):
    """Measure pitch and bank relative to world +Y, including inverted attitude.

    At vertical pitch, bank is conventionally undefined: return None rather
    than invent a horizon reference. This ambiguity is instrumentation only.

    Args:
        vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

    Returns:
        Pitch and bank in radians; bank is None at vertical pitch.
    """
    pitch = math.asin(float(np.clip(vehicle.forward[1], -1, 1)))
    if math.hypot(vehicle.forward[0], vehicle.forward[2]) < 1e-6:
        return pitch, None
    bank = math.atan2(-vehicle.right[1], vehicle.up[1])
    return pitch, bank
