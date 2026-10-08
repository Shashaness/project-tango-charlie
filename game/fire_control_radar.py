"""Perfect-information selected-target tracking; radar never fires the gun."""

from dataclasses import dataclass
import math
import numpy as np

from game.fire_control_solution import solve_intercept
from game.gun import Gun
from game.target_manager import TargetManager
from game.missile_seeker import ECM_DETECTION_FACTOR

RADAR_MAX_RANGE = 10000.0  # m; independent of the gun's useful engagement range


@dataclass(frozen=True)
class RadarTrack:
    target_id: int
    range: float
    closure: float  # positive means range decreasing
    relative_position: np.ndarray
    relative_velocity: np.ndarray
    line_of_sight: np.ndarray
    in_range: bool


class FireControlRadar:
    def __init__(self, max_range=RADAR_MAX_RANGE):
        self.max_range = max_range
        self.target_manager = TargetManager()
        self.track = None
        self.solution = None
        self.shoot = False

    @property
    def current_target(self):
        return self.target_manager.current_target

    @current_target.setter
    def current_target(self,target):
        self.target_manager.current_target=target

    def select_next(self, targets):
        return self.target_manager.cycle(targets)

    def update(self, vehicle, targets, gun=None):
        self.target_manager.refresh(targets)
        self.track = self.solution = None
        self.shoot = False
        target = self.current_target
        if target is None:
            return
        gun = Gun() if gun is None else gun
        muzzle_position = gun.muzzle_position(vehicle)
        relative_position = target.position - muzzle_position
        relative_velocity = target.velocity - vehicle.velocity
        distance = float(np.linalg.norm(relative_position))
        if not math.isfinite(distance) or distance < 1e-8:
            return
        line_of_sight = relative_position / distance
        closure = -float(np.dot(relative_velocity,line_of_sight))
        self.track = RadarTrack(target.target_id,distance,closure,relative_position,relative_velocity,
                                line_of_sight,distance<=self.max_range*(ECM_DETECTION_FACTOR if getattr(target,"ecm_enabled",False) else 1.0))
        if not self.track.in_range:
            return
        self.solution = solve_intercept(muzzle_position,vehicle.velocity,target.position,target.velocity,
                                        gun.parameters.muzzle_velocity)
        if self.solution is not None:
            alignment = float(np.dot(vehicle.forward,self.solution.required_direction))
            self.shoot = (distance<=gun.parameters.effective_range
                          and self.solution.intercept_time<=gun.parameters.projectile_lifetime
                          and alignment>=math.cos(math.radians(gun.parameters.firing_tolerance_deg)))
