"""Fighter-only operational gear/flaps and configurable wheel-contact parameters."""

import math
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np


class GearState(Enum):
    """Discrete gear states, including extension and retraction transitions."""

    RETRACTED = auto()
    DEPLOYING = auto()
    DEPLOYED = auto()
    RETRACTING = auto()


class FlapState(Enum):
    """Discrete flap states for fighter aerodynamic adjustments."""

    RETRACTED = auto()
    DEPLOYED = auto()


@dataclass(frozen=True)
class AirfieldParameters:
    """Immutable gear timing, wheel geometry, and landing limits in SI/_deg units."""

    gear_seconds: float = 2.5
    gear_drag: float = 0.025
    flap_drag: float = 0.08
    flap_lift: float = 0.35
    flap_lift_alpha_deg: float = 10.0
    rolling_resistance: float = 0.015
    brake_deceleration: float = 5.0
    brake_friction: float = 0.65
    hard_landing_speed: float = 4.0
    max_touchdown_speed: float = 120.0
    max_pitch_deg: float = 18.0
    max_roll_deg: float = 8.0
    off_runway_safe_speed: float = 5.0
    contact_epsilon: float = 0.02
    body_margin: float = 0.01
    configuration_warning_height: float = 150.0
    flap_warning_speed: float = 95.0
    wheel_points: tuple = ((-2.0, -3.0, 4.0), (2.0, -3.0, 4.0), (0.0, -3.0, -6.0))

    def __post_init__(self):
        """Validate configuration fields after dataclass initialization."""
        values = [value for name, value in self.__dict__.items() if name != "wheel_points"]
        if not all(math.isfinite(v) and v >= 0 for v in values) or self.flap_lift_alpha_deg <= 0:
            raise ValueError("Airfield parameters must be finite and nonnegative")
        if self.gear_seconds <= 0 or self.brake_deceleration <= 0 or self.brake_friction < 0:
            raise ValueError("Invalid gear timing/braking parameters")
        if (
            np.asarray(self.wheel_points).shape != (3, 3)
            or not np.isfinite(self.wheel_points).all()
        ):
            raise ValueError("Expected three finite aircraft-relative wheel points")


AIRFIELD = AirfieldParameters()


class AirfieldOperations:
    """Track fighter gear, flaps, brake requests, and operational crash messages."""

    def __init__(self, parameters=AIRFIELD):
        """Retain operational tuning and reset gear/flap/brake state.

        Args:
            parameters: Configuration parameters; defaults retain the simulation tuning.
        """
        self.parameters = parameters
        self.reset()

    def reset(self, *, deployed=False):
        """Reset gear deployment, retract flaps, and clear brake/crash telemetry.

        Args:
            deployed: Whether gear should reset fully deployed.
        """
        self.gear = GearState.DEPLOYED if deployed else GearState.RETRACTED
        self.extension = 1.0 if deployed else 0.0
        self.flaps = FlapState.RETRACTED
        self.brakes = False
        self.brakes_active = False
        self.crash_reason = None

    def toggle_gear(self):
        """Reverse or start the gear deployment transition."""
        self.gear = (
            GearState.RETRACTING
            if self.gear in (GearState.DEPLOYED, GearState.DEPLOYING)
            else GearState.DEPLOYING
        )

    def toggle_flaps(self):
        self.flaps = (
            FlapState.DEPLOYED if self.flaps is FlapState.RETRACTED else FlapState.RETRACTED
        )

    def update(self, dt):
        """Advance gear extension by its configured deployment time without altering vehicle
        motion.
        motion.
        motion.

        Args:
            dt: Elapsed simulation time in seconds.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Gear timestep must be finite and nonnegative")
        if self.gear in (GearState.DEPLOYING, GearState.RETRACTING):
            direction = 1 if self.gear is GearState.DEPLOYING else -1
            self.extension = float(
                np.clip(self.extension + direction * dt / self.parameters.gear_seconds, 0, 1)
            )
            if self.extension == 1:
                self.gear = GearState.DEPLOYED
            elif self.extension == 0:
                self.gear = GearState.RETRACTED

    def wheel_world(self, vehicle):
        """Transform configured aircraft-relative wheel points into world space.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.

        Returns:
            Three XYZ wheel positions in meters.
        """
        return np.asarray(self.parameters.wheel_points) @ vehicle.orientation.T + vehicle.position
