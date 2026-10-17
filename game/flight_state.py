"""Shared flight states for all three configurations."""

from dataclasses import dataclass
from enum import Enum, auto


class Environment(Enum):
    """Physics environment selector for inertial space or atmospheric flight."""

    SPACE = auto()
    ATMOSPHERE = auto()


class VehicleMode(Enum):
    """Public vehicle configurations used by controls, physics, and transformation."""

    FIGHTER = auto()
    VTOL = auto()
    BATTLEDROID = auto()


class FlightStatus(Enum):
    """Flight/contact lifecycle state, including terminal crash state."""

    FLYING = auto()
    GROUNDED = auto()
    CRASHED = auto()


@dataclass
class FlightState:
    """Shared mutable environment, configuration, and contact lifecycle selectors."""

    environment: Environment = Environment.SPACE
    mode: VehicleMode = VehicleMode.FIGHTER
    status: FlightStatus = FlightStatus.FLYING
