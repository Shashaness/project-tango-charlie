"""Exact constant-velocity gun intercept with inherited shooter velocity."""

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class FireControlSolution:
    intercept_time: float
    intercept_position: np.ndarray
    required_direction: np.ndarray  # muzzle direction, not world projectile velocity


def solve_intercept(shooter_position, shooter_velocity, target_position, target_velocity, muzzle_speed):
    vectors = [np.asarray(value, dtype=float) for value in
               (shooter_position, shooter_velocity, target_position, target_velocity)]
    if any(value.shape != (3,) or not np.isfinite(value).all() for value in vectors):
        return None
    if not math.isfinite(muzzle_speed) or muzzle_speed <= 0:
        return None
    shooter_position, shooter_velocity, target_position, target_velocity = vectors
    relative_position = target_position - shooter_position
    relative_velocity = target_velocity - shooter_velocity
    quadratic_a = float(np.dot(relative_velocity, relative_velocity) - muzzle_speed*muzzle_speed)
    quadratic_b = 2 * float(np.dot(relative_position, relative_velocity))
    quadratic_c = float(np.dot(relative_position, relative_position))
    if not all(math.isfinite(value) for value in (quadratic_a,quadratic_b,quadratic_c)) or quadratic_c < 1e-16:
        return None
    # |r + v*t| = s*t; scale the near-linear test to velocity squared units.
    a_scale = max(float(np.dot(relative_velocity,relative_velocity)), muzzle_speed*muzzle_speed, 1.0)
    if abs(quadratic_a) <= 1e-10 * a_scale:
        if abs(quadratic_b) <= 1e-12 * max(math.sqrt(quadratic_c)*muzzle_speed,1):
            return None
        roots = [-quadratic_c / quadratic_b]
    else:
        discriminant = quadratic_b*quadratic_b - 4 * quadratic_a * quadratic_c
        scale = max(quadratic_b*quadratic_b,abs(4*quadratic_a*quadratic_c),1)
        if not math.isfinite(discriminant) or discriminant < -1e-12*scale:
            return None
        root = math.sqrt(max(0.0,discriminant))
        # Stable quadratic evaluation avoids subtracting nearly equal roots.
        q = -.5 * (quadratic_b + math.copysign(root,quadratic_b))
        roots = [q/quadratic_a]
        if abs(q) > 1e-20:
            roots.append(quadratic_c/q)
    positive_roots = [time for time in roots if math.isfinite(time) and time > 1e-8]
    if not positive_roots:
        return None
    intercept_time = min(positive_roots)
    aim_displacement = relative_position + relative_velocity * intercept_time
    length = float(np.linalg.norm(aim_displacement))
    intercept_position = target_position + target_velocity * intercept_time
    if length < 1e-8 or not math.isfinite(length) or not np.isfinite(intercept_position).all():
        return None
    return FireControlSolution(intercept_time,intercept_position,aim_displacement/length)
