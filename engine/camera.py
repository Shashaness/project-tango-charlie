"""Vehicle-observing perspective camera, independent of flight physics."""

import math
from dataclasses import dataclass

import numpy as np

from engine.transform import Transform
from game.flight_state import VehicleMode

CHASE_DISTANCE = 24.0
CHASE_HEIGHT = 7.0
CHASE_LOOK_AHEAD = 2.0
ORBIT_SENSITIVITY = math.radians(.25)  # radians per cursor pixel
ORBIT_PITCH_LIMIT = math.radians(80)
ORBIT_MIN_DISTANCE = 8.0
ORBIT_MAX_DISTANCE = 80.0
ORBIT_ZOOM_RATE = .12  # exponential distance change per scroll unit

CHASE_SMOOTHING = 8.0  # exponential position response / second

@dataclass(frozen=True)
class ChaseSettings:
    distance: float
    elevation: float
    minimum: float
    maximum: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.distance,self.elevation,self.minimum,self.maximum)) or not (
                self.distance>0 and self.elevation>=0 and 0<self.minimum<=math.hypot(self.distance,self.elevation)<=self.maximum):
            raise ValueError('Invalid chase camera settings')

CHASE_SETTINGS = {
    VehicleMode.FIGHTER: ChaseSettings(24.,7.,8.,80.),
    VehicleMode.VTOL: ChaseSettings(18.,6.,8.,70.),
    VehicleMode.BATTLEDROID: ChaseSettings(32.,12.,18.,100.),
}


class Camera(Transform):
    def __init__(self, position=(0, 0, 3), aspect=1280 / 720, chase_settings=None):
        super().__init__(position)
        self.aspect = aspect
        self.fov = np.radians(60.0)
        self.near = 0.1
        self.far = 20000.0
        self.mode = "CHASE"
        self.chase_settings = dict(CHASE_SETTINGS)
        if chase_settings is not None:self.chase_settings.update(chase_settings)
        self._vehicle_mode = VehicleMode.FIGHTER
        self._zoom = {mode:math.hypot(s.distance,s.elevation) for mode,s in self.chase_settings.items()}
        self.external_camera_mode = "CHASE"
        initial = self.chase_settings[VehicleMode.FIGHTER]
        self._external_offset = np.array((0., initial.elevation, initial.distance))
        self._snap = True
        self.orbit_yaw = 0.0
        self.orbit_pitch = math.atan2(initial.elevation, initial.distance)
        self.orbit_distance = math.hypot(initial.distance, initial.elevation)
        self._manual_orbit = False
        self._orbit_basis = np.eye(3)
        self._default_basis = np.eye(3)
        self._default_pitch = self.orbit_pitch
        self._default_distance = self.orbit_distance
        self._view_yaw = self.orbit_yaw
        self._view_pitch = self.orbit_pitch
        self._view_distance = self.orbit_distance

    @property
    def label(self):
        return "COCKPIT" if self.mode == "COCKPIT" else self.external_camera_mode

    def _begin_orbit(self):
        # Capture the actual smoothed chase offset, not merely its target.
        self._manual_orbit = True
        self._orbit_basis = self._default_basis.copy()
        offset = self._orbit_basis.T @ self._external_offset
        distance = float(np.linalg.norm(offset))
        if distance < 1e-8:
            offset = np.array((0., math.sin(self._default_pitch), math.cos(self._default_pitch)))
            distance = self._default_distance
        self.orbit_yaw = self._view_yaw = math.atan2(offset[0], offset[2])
        self.orbit_pitch = self._view_pitch = math.atan2(offset[1], math.hypot(offset[0], offset[2]))
        self.orbit_distance = self._view_distance = distance

    def toggle_external_mode(self):
        if self.external_camera_mode == "CHASE":
            self.external_camera_mode = "DOLLY"
            self._begin_orbit()
        else:
            self.external_camera_mode = "CHASE"
            self._manual_orbit = False
            self._snap = True

    def orbit_drag(self, dx, dy):
        if self.mode != "CHASE" or self.external_camera_mode != "DOLLY" or not math.isfinite(dx) or not math.isfinite(dy):return
        if dx == 0 and dy == 0:return
        self.orbit_yaw += -dx * ORBIT_SENSITIVITY
        self.orbit_pitch = float(np.clip(self.orbit_pitch + dy * ORBIT_SENSITIVITY,
                                         -ORBIT_PITCH_LIMIT, ORBIT_PITCH_LIMIT))

    def orbit_zoom(self, scroll):
        if self.mode != "CHASE" or not math.isfinite(scroll) or scroll == 0:return
        factor = math.exp(float(np.clip(-scroll * ORBIT_ZOOM_RATE, -20, 20)))
        settings = self.chase_settings[self._vehicle_mode]
        self.orbit_distance = float(np.clip(self.orbit_distance * factor,
                                            settings.minimum, settings.maximum))
        self._zoom[self._vehicle_mode] = self.orbit_distance

    def reset_orbit(self):
        self._manual_orbit = self.external_camera_mode == "DOLLY"
        self._orbit_basis = self._default_basis.copy()
        self.orbit_yaw = 0.0
        self.orbit_pitch = self._default_pitch
        self.orbit_distance = self._default_distance
        self._zoom[self._vehicle_mode] = self._default_distance
        self._snap = True

    def snap_to_vehicle(self):
        """Snap the next observation after a development reset/teleport."""
        self._snap = True

    def toggle_mode(self):
        self.mode = "COCKPIT" if self.mode == "CHASE" else "CHASE"
        self._snap = True

    def follow(self, vehicle, dt):
        mode = vehicle.flight_state.mode
        transformation = getattr(vehicle, 'transformation', None)
        if getattr(transformation, 'active', False):mode = transformation.target
        settings = self.chase_settings[mode]
        mode_changed = mode is not self._vehicle_mode
        self._vehicle_mode = mode
        self._default_pitch = math.atan2(settings.elevation, settings.distance)
        self._default_distance = math.hypot(settings.distance, settings.elevation)
        if mode_changed:
            self.orbit_distance = self._zoom[mode]
            # Keep orbit azimuth, but restore the new mode's useful elevation.
            self.orbit_pitch = self._default_pitch
        if self.mode == "COCKPIT":
            self.position = vehicle.position.copy()
            self.forward = vehicle.forward.copy()
            self.right = vehicle.right.copy()
            self.up = vehicle.up.copy()
        else:
            ratio = self._zoom[mode]/self._default_distance
            distance, height = settings.distance*ratio, settings.elevation*ratio
            self._default_basis = vehicle.orientation.copy()
            self._default_pitch = math.atan2(height, distance)
            self._default_distance = math.hypot(settings.distance, settings.elevation)
            if self.external_camera_mode == "DOLLY":
                alpha = 1.0 if self._snap else -math.expm1(-CHASE_SMOOTHING * max(0.0, dt))
                self._view_yaw += (self.orbit_yaw - self._view_yaw) * alpha
                self._view_pitch += (self.orbit_pitch - self._view_pitch) * alpha
                self._view_distance += (self.orbit_distance - self._view_distance) * alpha
                yaw, pitch = self._view_yaw, self._view_pitch
                offset = self._orbit_basis @ np.array((math.sin(yaw)*math.cos(pitch),
                           math.sin(pitch), math.cos(yaw)*math.cos(pitch)))
                self.position = vehicle.position + offset * self._view_distance
                self.forward = -offset
                right = np.cross(self.forward, self._orbit_basis[:,1])
                self.right = right / np.linalg.norm(right)
                self.up = np.cross(self.right, self.forward)
                self._external_offset = self.position - vehicle.position
                self._snap = False
                return
            self.orbit_yaw = 0.0
            self.orbit_pitch = self._default_pitch
            self.orbit_distance = self._zoom[mode]
            desired = (vehicle.position - vehicle.forward * distance
                       + vehicle.up * height)
            alpha = 1.0 if self._snap else -math.expm1(-CHASE_SMOOTHING * max(0.0, dt))
            self.position += (desired - self.position) * alpha
            target = vehicle.position + vehicle.forward * CHASE_LOOK_AHEAD
            direction = target - self.position
            length = np.linalg.norm(direction)
            self.forward = direction / length if length > 1e-8 else vehicle.forward.copy()
            # Preserve vehicle bank, never use fixed world +Y.
            right = vehicle.right - self.forward * np.dot(vehicle.right, self.forward)
            if np.linalg.norm(right) < 1e-8:
                right = np.cross(self.forward, vehicle.up)
            self.right = right / np.linalg.norm(right)
            self.up = np.cross(self.right, self.forward)
        if self.mode == "CHASE":
            self._external_offset = self.position - vehicle.position
        self._snap = False

    def view_matrix(self):
        view = np.eye(4, dtype=np.float32)
        view[0, :3] = self.right
        view[1, :3] = self.up
        view[2, :3] = -self.forward
        view[:3, 3] = -view[:3, :3] @ self.position
        return view

    def projection_matrix(self):
        scale = 1.0 / np.tan(self.fov / 2.0)
        projection = np.zeros((4, 4), dtype=np.float32)
        projection[0, 0] = scale / self.aspect
        projection[1, 1] = scale
        projection[2, 2] = (self.far + self.near) / (self.near - self.far)
        projection[2, 3] = 2 * self.far * self.near / (self.near - self.far)
        projection[3, 2] = -1.0
        return projection
