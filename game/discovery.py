"""Physical visibility and deliberate inspection for world entities."""

from dataclasses import dataclass

import numpy as np

from game.player_control import EYE_HEIGHT, UP, segment_box_distance_squared


@dataclass(frozen=True)
class Inspectable:
    title: str
    description: str
    condition: str
    actions: str
    discovery_range: float = 5.5
    inspection_range: float = 2.8

    def visible(self, pilot, access, space, *, inspect=False):
        distance = self.inspection_range if inspect else self.discovery_range
        eye = pilot.position + UP * EYE_HEIGHT
        target = access.vehicle.position + UP * 0.45
        if np.linalg.norm(eye - target) > distance:
            return False
        lower, upper = np.minimum(eye, target), np.maximum(eye, target)
        if any(
            segment_box_distance_squared(eye, target, lo, hi) < 1e-8
            for lo, hi in space.boxes(lower, upper, exclude=access.vehicle)
        ):
            return False
        return all(point[1] > space.ground.height(point) for point in np.linspace(eye, target, 9))
