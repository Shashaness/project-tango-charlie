"""Fixed-forward automatic gun shared by all configurations/environments."""

from dataclasses import dataclass
import math
import numpy as np

from game.projectile import Projectile


@dataclass(frozen=True)
class GunParameters:
    muzzle_velocity: float = 1000.0  # m/s relative to shooter
    rounds_per_second: float = 12.0
    projectile_lifetime: float = 5.0  # seconds
    projectile_radius: float = .5  # m
    muzzle_offset: tuple = (0.0, 0.0, -3.0)  # local right/up/backward coordinates
    effective_range: float = 2000.0  # m
    firing_tolerance_deg: float = 1.5


GUN = GunParameters()


class Gun:
    def __init__(self, parameters=GUN):
        if parameters.rounds_per_second <= 0 or parameters.muzzle_velocity <= 0:
            raise ValueError('Gun fire rate and muzzle speed must be positive.')
        self.parameters = parameters
        self.cooldown = 0.0

    def muzzle_position(self, vehicle):
        return vehicle.position + vehicle.orientation @ np.array(self.parameters.muzzle_offset)

    def fire(self, vehicle, position=None, velocity=None):
        muzzle = self.muzzle_position(vehicle) if position is None else position
        inherited_velocity = vehicle.velocity if velocity is None else velocity
        return Projectile(muzzle, inherited_velocity + vehicle.forward * self.parameters.muzzle_velocity,
                          self.parameters.projectile_lifetime, self.parameters.projectile_radius)

    def update(self, dt, firing, vehicle, previous_position=None, previous_velocity=None):
        """Return (round, birth offset) events; no backlog accumulates while idle.

        Half-open frame intervals avoid double-firing shots on frame boundaries.
        Birth offsets allow rounds to advance only for their actual age this step.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Gun timestep must be finite and nonnegative.')
        if dt == 0:
            return []
        if not firing:
            self.cooldown = max(0.0, self.cooldown - dt)
            return []
        events = []
        offset = max(0.0, self.cooldown)
        interval = 1.0 / self.parameters.rounds_per_second
        local_muzzle = vehicle.orientation @ np.array(self.parameters.muzzle_offset)
        while offset < dt - 1e-12:
            fraction = offset / dt
            position = vehicle.position if previous_position is None else previous_position + (vehicle.position-previous_position)*fraction
            velocity = vehicle.velocity if previous_velocity is None else previous_velocity + (vehicle.velocity-previous_velocity)*fraction
            events.append((self.fire(vehicle, position+local_muzzle, velocity), offset))
            offset += interval
        self.cooldown = max(0.0, offset - dt)
        return events
