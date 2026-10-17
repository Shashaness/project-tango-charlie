"""Constant-velocity rounds; swept collision helpers include target motion."""

import math

import numpy as np


class Projectile:
    """Track an unguided projectile with inherited launch velocity and finite lifetime."""

    def __init__(self, position, velocity, lifetime=5.0, radius=0.5):
        """Validate finite launch kinematics and positive lifetime/collision radius.

        Args:
            position: XYZ position in meters.
            velocity: World velocity vector in m/s.
            lifetime: Maximum remaining travel time in seconds.
            radius: Collision radius in meters.
        """
        self.position = np.array(position, dtype=np.float64)
        self.previous_position = self.position.copy()
        self.velocity = np.array(velocity, dtype=np.float64)
        if (
            self.position.shape != (3,)
            or self.velocity.shape != (3,)
            or not np.isfinite(self.position).all()
            or not np.isfinite(self.velocity).all()
            or not math.isfinite(lifetime)
            or lifetime <= 0
            or not math.isfinite(radius)
            or radius <= 0
        ):
            raise ValueError("Projectile requires finite vectors and positive lifetime/radius.")
        self.lifetime = lifetime
        self.radius = radius
        self.alive = True

    def update(self, dt):
        """Advance constant-velocity travel up to the remaining projectile lifetime.

        Args:
            dt: Elapsed simulation time in seconds.

        Returns:
            Elapsed travel time in seconds; updates position, age, and alive state.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Projectile timestep must be finite and nonnegative.")
        self.previous_position = self.position.copy()
        travel_time = min(dt, self.lifetime) if self.alive else 0.0
        self.position += self.velocity * travel_time
        self.lifetime = max(0.0, self.lifetime - travel_time)
        self.alive = self.alive and self.lifetime > 1e-10
        return travel_time


def segment_sphere_hit(start, end, center, radius):
    """Find the earliest swept segment contact with a sphere, including initial overlap.

    Args:
        start: World segment start in meters.
        end: World segment end in meters.
        center: World sphere center in meters.
        radius: Collision radius in meters.

    Returns:
        Contact fraction in [0, 1], or None when the segment misses.
    """
    relative_start = np.asarray(start) - np.asarray(center)
    delta = np.asarray(end) - np.asarray(start)
    quadratic_c = float(np.dot(relative_start, relative_start) - radius * radius)
    if quadratic_c <= 0:
        return 0.0
    quadratic_a = float(np.dot(delta, delta))
    if quadratic_a < 1e-20:
        return None
    quadratic_b = 2 * float(np.dot(relative_start, delta))
    discriminant = quadratic_b * quadratic_b - 4 * quadratic_a * quadratic_c
    if discriminant < 0:
        return None
    fraction = (-quadratic_b - math.sqrt(discriminant)) / (2 * quadratic_a)
    return fraction if 0 <= fraction <= 1 else None
