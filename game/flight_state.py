"""Shared flight states for all three configurations."""

from dataclasses import dataclass
from enum import Enum, auto


class Environment(Enum):
    SPACE = auto()
    ATMOSPHERE = auto()


class VehicleMode(Enum):
    FIGHTER = auto()
    VTOL = auto()
    BATTLEDROID = auto()


class FlightStatus(Enum):
    FLYING = auto()
    GROUNDED = auto()
    CRASHED = auto()


@dataclass
class FlightState:
    environment: Environment = Environment.SPACE
    mode: VehicleMode = VehicleMode.FIGHTER
    status: FlightStatus = FlightStatus.FLYING
