"""Orbit and eye-level observation for the on-foot controller."""

import math

import numpy as np

from engine.camera import ORBIT_PITCH_LIMIT, ORBIT_SENSITIVITY, Camera
from game.player_control import EYE_HEIGHT, UP


class PedestrianCamera(Camera):
    """Use the vehicle camera matrices with independent pilot orbit state."""

    def __init__(self):
        super().__init__()
        self.mode = "PILOT"
        self.reset_orbit()

    @property
    def label(self):
        return "ON FOOT · EYES" if self.first_person else "ON FOOT"

    @property
    def first_person(self):
        return self._distance < 0.35

    def reset_orbit(self):
        self.orbit_yaw = 0.0
        self.orbit_pitch = -0.18
        self.orbit_distance = self._distance = 4.5

    def begin(self, pilot):
        self.reset_orbit()
        self.orbit_yaw = math.atan2(pilot.forward[0], pilot.forward[2])

    def orbit_drag(self, dx, dy):
        self.orbit_yaw -= dx * ORBIT_SENSITIVITY
        self.orbit_pitch = float(
            np.clip(
                self.orbit_pitch - dy * ORBIT_SENSITIVITY, -ORBIT_PITCH_LIMIT, ORBIT_PITCH_LIMIT
            )
        )

    def orbit_zoom(self, scroll):
        self.orbit_distance = float(
            np.clip(
                (self.orbit_distance + 0.3) * math.exp(np.clip(-0.24 * scroll, -20, 20)) - 0.3, 0, 8
            )
        )

    def look_direction(self):
        return np.array((math.sin(self.orbit_yaw), 0, math.cos(self.orbit_yaw)))

    def observe(self, pilot, space, dt):
        horizontal = self.look_direction()
        self.forward = horizontal * math.cos(self.orbit_pitch) + UP * math.sin(self.orbit_pitch)
        self.right = np.cross(horizontal, UP)
        self.up = np.cross(self.right, self.forward)
        eye = pilot.position + UP * EYE_HEIGHT
        desired = space.camera_distance(eye, -self.forward, self.orbit_distance)
        alpha = -math.expm1(-14 * max(dt, 0))
        smoothed = self._distance + (desired - self._distance) * alpha
        # Contract immediately for obstructions, smoothly for deliberate zoom.
        self._distance = min(smoothed, desired) if desired < self.orbit_distance else smoothed
        if desired == 0 and self._distance < 0.005:
            self._distance = 0
        self.position[:] = eye - self.forward * self._distance
