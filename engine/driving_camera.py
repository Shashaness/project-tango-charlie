"""Independent, collision-aware chase camera for ground vehicles."""

import math

import numpy as np

from engine.pedestrian_camera import PedestrianCamera
from game.player_control import UP


class DrivingCamera(PedestrianCamera):
    def __init__(self):
        self.vehicle = None
        super().__init__()
        self.mode = "DRIVING"

    @property
    def label(self):
        return "DRIVING"

    def reset_orbit(self):
        self.orbit_yaw = 0.0
        self.orbit_pitch = -0.23
        self.orbit_distance = self._distance = 7.0
        self._heading = self.vehicle.heading if self.vehicle is not None else 0.0

    def begin(self, vehicle):
        self.vehicle = vehicle
        self.reset_orbit()

    def orbit_zoom(self, scroll):
        self.orbit_distance = float(
            np.clip(self.orbit_distance * math.exp(np.clip(-scroll * 0.14, -10, 10)), 2.5, 15)
        )

    def observe(self, vehicle, space, dt):
        alpha = -math.expm1(-6 * max(0, dt))
        self._heading += math.remainder(vehicle.heading - self._heading, math.tau) * alpha
        yaw = self._heading + self.orbit_yaw
        horizontal = np.array((math.sin(yaw), 0, math.cos(yaw)))
        self.forward = horizontal * math.cos(self.orbit_pitch) + UP * math.sin(self.orbit_pitch)
        self.right = np.cross(horizontal, UP)
        self.up = np.cross(self.right, self.forward)
        target = vehicle.position + UP * 1.22
        clear = space.camera_distance(target, -self.forward, self.orbit_distance, exclude=vehicle)
        eased = self._distance + (self.orbit_distance - self._distance) * alpha
        self._distance = min(clear, eased) if clear < self.orbit_distance else eased
        self.position[:] = target - self.forward * self._distance
