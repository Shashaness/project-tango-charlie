"""Vector PN normal acceleration: bounded forces, never velocity steering."""

import math
import numpy as np

STANDARD_GRAVITY = 9.80665


def proportional_navigation(relative_position, relative_velocity, missile_velocity,
                            navigation_constant, max_g, max_turn_rate_deg):
    distance_squared = float(np.dot(relative_position,relative_position))
    speed = float(np.linalg.norm(missile_velocity))
    if distance_squared < 1e-12 or speed < 1e-6:
        return np.zeros(3)
    line_of_sight = relative_position / math.sqrt(distance_squared)
    closing_speed = max(0.0,-float(np.dot(relative_velocity,line_of_sight)))
    los_angular_velocity = np.cross(relative_position,relative_velocity) / distance_squared
    velocity_direction = missile_velocity / speed
    acceleration = navigation_constant * closing_speed * np.cross(los_angular_velocity,velocity_direction)
    # PN acceleration is normal to motion: steering comes from acceleration integration.
    maximum = min(max_g*STANDARD_GRAVITY,speed*math.radians(max_turn_rate_deg))
    magnitude = float(np.linalg.norm(acceleration))
    if magnitude > maximum:
        acceleration *= maximum/magnitude
    return acceleration
