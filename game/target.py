"""Independent constant-velocity test target (SI units)."""

import math
import numpy as np


class Target:
    def __init__(self, target_id, position, velocity=(0, 0, 0), radius=25.0):
        self.target_id = target_id
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        if (self.position.shape != (3,) or self.velocity.shape != (3,) or
                not np.isfinite(self.position).all() or not np.isfinite(self.velocity).all() or
                not math.isfinite(radius) or radius <= 0):
            raise ValueError('Target requires finite vectors and a positive radius.')
        self.radius = radius
        self.alive = True

    def update(self, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Target timestep must be finite and nonnegative.')
        if self.alive:
            self.position += self.velocity * dt
