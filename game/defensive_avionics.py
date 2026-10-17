"""Observational RWR/approach warning; perfect information, no guidance writes."""

import math
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from game.missile_fire_control import LockState
from game.missile_seeker import SeekerState, SeekerType


class ThreatState(Enum):
    """Observed emitter tracking, lock, or inbound missile warning category."""

    TRACK = auto()
    LOCK = auto()
    MISSILE = auto()


@dataclass(frozen=True)
class Threat:
    """Read-only threat source, local bearing in radians, range in meters, and impact time in
    seconds.
    """

    source: object
    state: ThreatState
    bearing: float
    range: float
    time_to_impact: float | None = None
    seeker_type: SeekerType | None = None


def geometry(observer, source):
    """Measure aircraft-relative bearing, range, and constant-closure impact time.

    Args:
        observer: Aircraft whose local basis defines bearing and warning geometry.
        source: Threat emitter or missile with world kinematics.

    Returns:
        (bearing radians, range meters, impact seconds or None), or None for invalid range.
    """
    relative = source.position - observer.position
    distance = float(np.linalg.norm(relative))
    if not math.isfinite(distance):
        return None
    # Zero = ahead, positive = right, +/-pi = behind, measured in aircraft frame.
    bearing = math.atan2(
        float(np.dot(relative, observer.right)), float(np.dot(relative, observer.forward))
    )
    closure = -float(np.dot(source.velocity - observer.velocity, relative / max(distance, 1e-8)))
    tti = (
        distance / closure
        if math.isfinite(closure) and closure > 1e-6 and distance > 1e-8
        else None
    )
    return bearing, distance, tti


class DefensiveAvionics:
    """Observe assigned radar/missile threats without writing guidance or motion."""

    def __init__(self):
        """Initialize empty radar-warning and incoming-missile lists."""
        self.rwr = []
        self.incoming = []

    def update(self, observer, assigned_entity, emitters=(), missiles=()):
        """Rebuild radar-warning and incoming-missile lists for the observed aircraft.

        Args:
            observer: Aircraft whose local basis defines bearing and warning geometry.
            assigned_entity: Target identity used to match emitters and missiles to the
                observer.
            emitters: Aircraft with combat radar state.
            missiles: Missile collection to observe.
        """
        self.rwr = []
        self.incoming = []
        for emitter in emitters:
            if not getattr(emitter, "alive", True):
                continue
            combat = emitter.combat
            radar = combat.radar
            track = radar.track
            if radar.current_target is not assigned_entity or track is None or not track.in_range:
                continue
            g = geometry(observer, emitter)
            if g is None:
                continue
            fc = combat.missile_fire_control
            state = (
                ThreatState.LOCK
                if fc.selected_type is SeekerType.RADAR and fc.lock_state is LockState.LOCKED
                else ThreatState.TRACK
            )
            self.rwr.append(Threat(emitter, state, g[0], g[1]))
        for missile in missiles:
            if (
                not missile.alive
                or missile.original_target is not assigned_entity
                or missile.seeker_state is SeekerState.LOST
            ):
                continue
            g = geometry(observer, missile)
            if g is None:
                continue
            threat = Threat(missile, ThreatState.MISSILE, *g, seeker_type=missile.seeker_type)
            self.incoming.append(threat)
            self.rwr.append(threat)
