"""Finite deterministic expendables; no aircraft or missile motion writes."""
from enum import Enum, auto
import math
import numpy as np

class CountermeasureType(Enum):
    FLARE = auto()
    CHAFF = auto()

class Countermeasure:
    radius = .5
    def __init__(self, vehicle, kind):
        self.kind = kind
        self.position = vehicle.position.copy()-vehicle.forward*3
        self.velocity = vehicle.velocity.copy()-vehicle.forward*8-vehicle.up*4
        self.age = 0.
        self.lifetime = 5. if kind is CountermeasureType.FLARE else 8.
        self.alive = True
        self.target_id = -1

    @property
    def heat_signature(self):
        return 12.*math.exp(-self.age/1.5) if self.alive and self.kind is CountermeasureType.FLARE else 0.

    @property
    def radar_signature(self):
        return 10.*math.exp(-self.age/2.5) if self.alive and self.kind is CountermeasureType.CHAFF else 0.

    @property
    def dispersion_radius(self):
        return 1.+self.age*4 if self.kind is CountermeasureType.CHAFF else .5

    def update(self, dt):
        if not math.isfinite(dt) or dt < 0: raise ValueError('Invalid countermeasure timestep.')
        travel = min(dt,max(0.,self.lifetime-self.age))
        self.position += self.velocity*travel
        self.age += travel
        self.alive = self.age < self.lifetime

class CountermeasureDispenser:
    def __init__(self, flares=30, chaff=30):
        self.capacity = (flares,chaff)
        self.reset()

    def reset(self):
        self.flares,self.chaff = self.capacity
        self.ecm_enabled = False
        self.cooldown = 0.

    def dispense(self, vehicle, kind, collection):
        name = 'flares' if kind is CountermeasureType.FLARE else 'chaff'
        count = getattr(self,name)
        if count <= 0: return None
        package = Countermeasure(vehicle,kind)
        setattr(self,name,count-1)
        collection.append(package)
        return package

    def defend(self, dt, vehicle, avionics, collection):
        self.cooldown = max(0.,self.cooldown-dt)
        if self.cooldown > 0 or not avionics.incoming: return
        threat = min(avionics.incoming,key=lambda t: t.time_to_impact if t.time_to_impact is not None else math.inf)
        if threat.range > 2500 or (threat.time_to_impact is not None and threat.time_to_impact > 8): return
        from game.missile_seeker import SeekerType
        kind = CountermeasureType.FLARE if threat.seeker_type is SeekerType.IR else CountermeasureType.CHAFF
        if self.dispense(vehicle,kind,collection) is not None:
            self.cooldown = 1.5 + (vehicle.target_id % 3)*.2
