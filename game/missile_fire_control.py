"""Prelaunch timed acquisition and closure/crossing-adjusted launch advice."""

from enum import Enum,auto
import math
import numpy as np

from game.missile import MISSILE,Missile
from game.missile_seeker import SeekerType,IR_SEEKER_RANGE,IR_MIN_HEAT,heat_signature,ECM_LOCK_FACTOR


class LockState(Enum):
    NO_TARGET=auto()
    ACQUIRING=auto()
    LOCKED=auto()
    OUT_OF_RANGE=auto()


class MissileFireControl:
    def __init__(self,parameters=MISSILE):
        self.parameters=parameters
        self.selected_type=SeekerType.RADAR
        self.inventory=parameters.max_inventory
        self.lock_state=LockState.NO_TARGET
        self.lock_progress=0.0
        self.lock_target=None
        self.in_envelope=False
        self.advised_range=0.0

    @property
    def inventory(self):
        return sum(self.inventories.values())

    @inventory.setter
    def inventory(self, value):
        value=max(0,int(value))
        # Tiny custom loadouts retain radar-only compatibility; standard load splits.
        radar=value if value<=2 else (value+1)//2
        self.inventories={SeekerType.RADAR:radar,SeekerType.IR:value-radar}

    @property
    def selected_inventory(self):
        return self.inventories[self.selected_type]

    def select(self, seeker_type):
        if seeker_type is not self.selected_type:
            self.selected_type=seeker_type
            self.lock_progress=0.;self.lock_state=LockState.ACQUIRING
            self.in_envelope=False

    def reset(self):
        self.inventory=self.parameters.max_inventory
        self.lock_target=None;self.lock_progress=0;self.lock_state=LockState.NO_TARGET
        self.in_envelope=False

    def update(self,dt,vehicle,radar):
        if not math.isfinite(dt) or dt<0:raise ValueError('Lock timestep must be finite and nonnegative.')
        target=radar.current_target
        self.in_envelope=False
        if target is not self.lock_target:
            self.lock_progress=0;self.lock_target=target
        if target is None or not target.alive or radar.track is None:
            self.lock_state=LockState.NO_TARGET;self.lock_progress=0;return
        relative_position=target.position-(vehicle.position+vehicle.orientation@np.array(self.parameters.hardpoint))
        distance=float(np.linalg.norm(relative_position))
        ir=self.selected_type is SeekerType.IR
        limit=min(self.parameters.max_range,IR_SEEKER_RANGE) if ir else self.parameters.max_range
        if (not ir and not radar.track.in_range) or distance>limit:
            self.lock_state=LockState.OUT_OF_RANGE;self.lock_progress=0;return
        valid=distance>1e-8 and float(np.dot(relative_position/distance,vehicle.forward))>=math.cos(math.radians(self.parameters.lock_half_cone_deg))
        if ir and heat_signature(target)<IR_MIN_HEAT: valid=False
        if not valid:
            self.lock_progress=0;self.lock_state=LockState.ACQUIRING;return
        self.lock_progress=min(self.parameters.lock_time,self.lock_progress+dt*(ECM_LOCK_FACTOR if not ir and getattr(target,"ecm_enabled",False) else 1.0))
        self.lock_state=LockState.LOCKED if self.lock_progress>=self.parameters.lock_time-1e-9 else LockState.ACQUIRING
        closure=radar.track.closure
        lateral_velocity=radar.track.relative_velocity-radar.track.line_of_sight*np.dot(radar.track.relative_velocity,radar.track.line_of_sight)
        recession_factor=float(np.clip(1+closure/self.parameters.nominal_engagement_speed,.25,1.25))
        crossing_factor=1/math.sqrt(1+(np.linalg.norm(lateral_velocity)/self.parameters.nominal_engagement_speed)**2)
        self.advised_range=min(self.parameters.max_range,self.parameters.effective_range*recession_factor*crossing_factor)
        estimated_time=distance/max(50,self.parameters.nominal_engagement_speed+closure)
        self.in_envelope=(self.parameters.min_range<=distance<=self.advised_range
                          and estimated_time<=self.parameters.max_lifetime-self.parameters.separation_time)

    def launch(self,vehicle):
        if self.selected_inventory<=0 or self.lock_state is not LockState.LOCKED or self.lock_target is None or not self.lock_target.alive:
            return None
        missile=Missile(vehicle,self.lock_target,self.parameters,self.selected_type)
        self.inventories[self.selected_type]-=1
        return missile
