"""Permanent geodetic addresses; regional meter frames remain explicit."""
from dataclasses import dataclass
import math
import re
import numpy as np
from game.hgt import LocalProjection

def wrap_longitude(longitude):
    if not math.isfinite(longitude): raise ValueError('Longitude must be finite')
    return (longitude+180)%360-180

@dataclass(frozen=True,order=True)
class GeographicTile:
    south: int
    west: int
    def __post_init__(self):
        if not isinstance(self.south,int) or not isinstance(self.west,int) or not -90 <= self.south < 90 or not -180 <= self.west < 180:
            raise ValueError('Invalid southwest geographic cell corner')
    @property
    def identifier(self):
        return f"{'N' if self.south>=0 else 'S'}{abs(self.south):02d}{'E' if self.west>=0 else 'W'}{abs(self.west):03d}"
    @classmethod
    def at(cls, latitude, longitude):
        if not math.isfinite(latitude) or not -90 <= latitude <= 90: raise ValueError('Latitude must be within [-90,90]')
        return cls(min(math.floor(latitude),89),math.floor(wrap_longitude(longitude)))
    @classmethod
    def parse(cls, identifier):
        match=re.fullmatch(r'([NS])(\d{2})([EW])(\d{3})',identifier)
        if not match: raise ValueError('Invalid geographic tile identifier')
        ns,lat,ew,lon=match.groups()
        tile=cls(int(lat)*(1 if ns=='N' else -1),int(lon)*(1 if ew=='E' else -1))
        if tile.identifier!=identifier: raise ValueError('Use canonical geographic tile identifiers')
        return tile

@dataclass(frozen=True)
class GeographicPosition:
    latitude: float
    longitude: float
    elevation: float  # source-reference meters, not an ellipsoidal height
    def __post_init__(self):
        GeographicTile.at(self.latitude,self.longitude)
        if not math.isfinite(self.elevation): raise ValueError('Elevation must be finite')
        object.__setattr__(self,'longitude',wrap_longitude(self.longitude))
    @property
    def tile(self): return GeographicTile.at(self.latitude,self.longitude)

class GeographicFrame:
    """Fixed regional projection + explicit vertical reference; no physics rebase."""
    def __init__(self, projection, elevation_offset=0.):
        self.projection=projection;self.elevation_offset=elevation_offset
    def from_local(self, position):
        latitude,longitude=self.projection.to_geographic(position[0],position[2])
        return GeographicPosition(float(latitude),float(longitude),float(position[1])+self.elevation_offset)
    def horizontal_basis(self):
        """True east/north/up in this regional frame, including translated rebases."""
        return np.array((1.,0.,0.)),np.array((0.,0.,-1.)),np.array((0.,1.,0.))
    def to_local(self, position):
        x,z=self.projection.to_world(position.latitude,position.longitude)
        return np.asarray((x,position.elevation-self.elevation_offset,z),dtype=np.float64)
    def rebase_plan(self, position):
        """Return an explicit proposed frame/shift; never mutate game objects."""
        new_projection=LocalProjection(position.latitude,position.longitude)
        return GeographicFrame(new_projection,self.elevation_offset),self.to_local(position)
