"""Finite deterministic expendables; no aircraft or missile motion writes."""

import math
from enum import Enum, auto

import numpy as np


class CountermeasureType(Enum):
    """Expendable infrared flare or radar chaff classification."""

    FLARE = auto()
    CHAFF = auto()


class Countermeasure:
    """Track finite-life decoy motion and deterministic heat/radar signatures."""

    radius = 0.5

    def __init__(self, vehicle, kind):
        """Initialize a decoy behind the aircraft with inherited motion and finite lifetime.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            kind: Countermeasure classification: flare or chaff.
        """
        self.kind = kind
        self.position = vehicle.position.copy() - vehicle.forward * 3
        self.velocity = vehicle.velocity.copy() - vehicle.forward * 8 - vehicle.up * 4
        self.age = 0.0
        self.lifetime = 5.0 if kind is CountermeasureType.FLARE else 8.0
        self.alive = True
        self.target_id = -1

    @property
    def heat_signature(self):
        return (
            12.0 * math.exp(-self.age / 1.5)
            if self.alive and self.kind is CountermeasureType.FLARE
            else 0.0
        )

    @property
    def radar_signature(self):
        return (
            10.0 * math.exp(-self.age / 2.5)
            if self.alive and self.kind is CountermeasureType.CHAFF
            else 0.0
        )

    @property
    def dispersion_radius(self):
        return 1.0 + self.age * 4 if self.kind is CountermeasureType.CHAFF else 0.5

    def update(self, dt):
        """Advance decoy constant-velocity travel and expire it at the lifetime boundary.

        Args:
            dt: Elapsed simulation time in seconds.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Invalid countermeasure timestep.")
        travel = min(dt, max(0.0, self.lifetime - self.age))
        self.position += self.velocity * travel
        self.age += travel
        self.alive = self.age < self.lifetime


class CountermeasureDispenser:
    """Own finite flare/chaff inventories, ECM state, and defensive cooldown."""

    def __init__(self, flares=30, chaff=30):
        """Retain capacities and reset inventories, ECM, and cooldown.

        Args:
            flares: Initial flare count.
            chaff: Initial chaff count.
        """
        self.capacity = (flares, chaff)
        self.reset()

    def reset(self):
        self.flares, self.chaff = self.capacity
        self.ecm_enabled = False
        self.cooldown = 0.0

    def dispense(self, vehicle, kind, collection):
        """Create and append one decoy while consuming its available inventory.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            kind: Countermeasure classification: flare or chaff.
            collection: Mutable shared decoy list receiving new expendables.

        Returns:
            New Countermeasure, or None when that inventory is empty.
        """
        name = "flares" if kind is CountermeasureType.FLARE else "chaff"
        count = getattr(self, name)
        if count <= 0:
            return None
        package = Countermeasure(vehicle, kind)
        setattr(self, name, count - 1)
        collection.append(package)
        return package

    def defend(self, dt, vehicle, avionics, collection):
        """Dispense a matching decoy for an imminent missile and update the cooldown.

        Args:
            dt: Elapsed simulation time in seconds.
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            avionics: Observed incoming-threat telemetry.
            collection: Mutable shared decoy list receiving new expendables.
        """
        self.cooldown = max(0.0, self.cooldown - dt)
        if self.cooldown > 0 or not avionics.incoming:
            return
        threat = min(
            avionics.incoming,
            key=lambda t: t.time_to_impact if t.time_to_impact is not None else math.inf,
        )
        if threat.range > 2500 or (threat.time_to_impact is not None and threat.time_to_impact > 8):
            return
        from game.missile_seeker import SeekerType

        kind = (
            CountermeasureType.FLARE
            if threat.seeker_type is SeekerType.IR
            else CountermeasureType.CHAFF
        )
        if self.dispense(vehicle, kind, collection) is not None:
            self.cooldown = 1.5 + (vehicle.target_id % 3) * 0.2
