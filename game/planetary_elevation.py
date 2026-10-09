"""Continuous geographic relief and real-source confidence blending."""
import numpy as np
from game.geography import GeographicTile

class ProceduralElevation:
    def __init__(self, seed=205): self.seed=int(seed)
    def _noise(self, points):
        base=np.floor(points).astype(np.int64);fraction=points-base
        smooth=fraction**3*(10+fraction*(-15+6*fraction));result=np.zeros(points.shape[:-1])
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    corner=(base+np.array([x,y,z])).astype(np.uint64) & np.uint64(0xffffffff)
                    h=(corner[...,0]*np.uint64(73856093))^(corner[...,1]*np.uint64(19349663))^(corner[...,2]*np.uint64(83492791))^np.uint64(self.seed & 0xffffffff)
                    h^=h>>np.uint64(13);h &= np.uint64(0xffffffff);h*=np.uint64(1274126177);h^=h>>np.uint64(16)
                    value=(h & np.uint64(0xffffffff)).astype(float)/2147483647.5-1
                    weight=np.ones(result.shape)
                    for axis,offset in enumerate((x,y,z)):weight*=smooth[...,axis] if offset else 1-smooth[...,axis]
                    result+=value*weight
        return result
    def sample(self, latitude, longitude):
        latitude,longitude=np.broadcast_arrays(np.asarray(latitude,float),np.asarray(longitude,float))
        if not np.isfinite(latitude).all() or not np.isfinite(longitude).all() or np.any(np.abs(latitude)>90): raise ValueError('Invalid procedural geographic coordinates')
        phi=np.radians(latitude);theta=np.radians((longitude+180)%360-180)
        cosine=np.where(np.abs(latitude)==90,0.,np.cos(phi))
        direction=np.stack((cosine*np.cos(theta),cosine*np.sin(theta),np.sin(phi)),axis=-1)
        result=np.full(latitude.shape,1200.)
        for frequency,amplitude in ((64,900),(128,450),(256,200),(512,80)):
            noise=self._noise(direction*frequency)
            result+=amplitude*noise
            if frequency==256:result+=350*(1-np.abs(noise))**2
        return result

class PlanetaryElevation:
    def __init__(self, dataset, projection, seed=205, blend_width=750.):
        self.dataset=dataset;self.projection=projection
        self.procedural=ProceduralElevation(seed);self.blend_width=blend_width
    def sample(self, latitude, longitude):
        latitude,longitude=np.broadcast_arrays(np.asarray(latitude,float),np.asarray(longitude,float))
        procedural=self.procedural.sample(latitude,longitude)
        real=self.dataset.sample(latitude,longitude)
        weight=self.dataset.coverage_weight(latitude,longitude,self.projection.east_scale,self.projection.north_scale,self.blend_width)
        weight*=self.dataset.confidence(latitude,longitude,self.projection.east_scale,self.projection.north_scale,self.blend_width)
        weight=np.where(np.isfinite(real),weight,0.)
        return procedural*(1-weight)+np.where(np.isfinite(real),real,0)*weight
    def source_at(self, latitude, longitude):
        real=float(self.dataset.sample(latitude,longitude))
        if not np.isfinite(real):return 'PROCEDURAL'
        weight=float(self.dataset.coverage_weight(latitude,longitude,self.projection.east_scale,self.projection.north_scale,self.blend_width))
        weight*=float(self.dataset.confidence(latitude,longitude,self.projection.east_scale,self.projection.north_scale,self.blend_width))
        return 'SRTM' if weight>=1-1e-8 else 'BLEND'
