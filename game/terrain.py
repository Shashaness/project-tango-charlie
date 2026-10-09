"""LOD-independent height field and bounded quadtree terrain geometry."""
from dataclasses import dataclass
from pathlib import Path
from functools import lru_cache
import numpy as np
from engine.asset_paths import asset_path
from game.hgt import HgtDataset, LocalProjection
from game.world_environment import TERRAIN_BOUNDS

@dataclass(frozen=True)
class TerrainConfig:
    directory: Path = asset_path('terrain/hgt')
    latitude: float = 34.5
    longitude: float = -111.5
    elevation_offset: float | None = None
    side: float = 32000.
    view_distance: float = 16000.
    budget: int = 64
    max_level: int = 5
    cells: int = 32
    cache_size: int = 96
    blend_width: float = 1500.
    hysteresis: float = .15

    def __post_init__(self):
        if not 22000 <= self.side <= 64000 or not 1000 <= self.view_distance <= 20000:
            raise ValueError('Terrain side must be 22–64 km and view distance 1–20 km')
        if not 4 <= self.budget <= 128 or not self.budget+1 <= self.cache_size <= 256:
            raise ValueError('Cache must exceed the 4–128 patch budget')
        if not 1 <= self.max_level <= 6 or not 4 <= self.cells <= 64 or not 0 <= self.hysteresis < .5 or not 0 < self.blend_width <= 5000:
            raise ValueError('Invalid terrain subdivision or blend configuration')
        LocalProjection(self.latitude,self.longitude)
        if self.elevation_offset is not None and not np.isfinite(self.elevation_offset):
            raise ValueError('Elevation offset must be finite')

class Terrain:
    def __init__(self, config=None):
        self.config = config or TerrainConfig()
        self.projection = LocalProjection(self.config.latitude, self.config.longitude)
        self.dataset = HgtDataset(self.config.directory)
        self.enabled = bool(self.dataset.paths)
        origin = float(self.dataset.sample(self.config.latitude, self.config.longitude))
        self.offset = self.config.elevation_offset if self.config.elevation_offset is not None else (origin if np.isfinite(origin) else 0.)
        half = self.config.side/2
        north,west = self.projection.to_geographic(-half,-half)
        south,east = self.projection.to_geographic(half,half)
        keys = {key for key in self.dataset.paths if key[0] <= north and key[0]+1 >= south
                and (west > east or (key[1] <= east and key[1]+1 >= west))}
        low, high = self.dataset.elevation_range(keys)
        self.elevation_bounds = min(0., low-self.offset), max(0., high-self.offset)
        self.skirt_depth = self.elevation_bounds[1]-self.elevation_bounds[0]+64.

    def heights(self, x, z):
        x, z = np.broadcast_arrays(np.asarray(x, float), np.asarray(z, float))
        lat, lon = self.projection.to_geographic(x, z)
        raw = np.nan_to_num(self.dataset.sample(lat, lon)-self.offset, nan=0.)
        # Fade unavailable tile edges to flat ground rather than an abrupt data cliff.
        coverage = np.zeros(x.shape)
        for south,west in self.dataset.paths:
            mask = (lat >= south) & (lat <= south+1) & (lon >= west) & (lon <= west+1)
            fade = np.ones(x.shape)
            for neighbor,distance in (((south,west-1),(lon-west)*self.projection.east_scale),
                                      ((south,west+1),(west+1-lon)*self.projection.east_scale),
                                      ((south-1,west),(lat-south)*self.projection.north_scale),
                                      ((south+1,west),(south+1-lat)*self.projection.north_scale)):
                if neighbor not in self.dataset.paths:
                    fade = np.minimum(fade,np.clip(distance/750.,0,1))
            fade = fade*fade*(3-2*fade)
            coverage = np.maximum(coverage,np.where(mask,fade,0))
        return raw * self.compatibility_weight(x,z) * coverage

    def compatibility_weight(self, x, z):
        x,z = np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        xmin,xmax,zmin,zmax = TERRAIN_BOUNDS
        distance = np.maximum.reduce((xmin-x,x-xmax,zmin-z,z-zmax,np.zeros(x.shape)))
        t = np.clip(distance/self.config.blend_width,0,1)
        return t*t*(3-2*t)

    def height_at(self, x, z): return float(self.heights(x, z))

    def normals(self, x, z):
        step = 10.
        dx = (self.heights(np.asarray(x)+step,z)-self.heights(np.asarray(x)-step,z))/(2*step)
        dz = (self.heights(x,np.asarray(z)+step)-self.heights(x,np.asarray(z)-step))/(2*step)
        normal = np.stack((-dx, np.ones_like(dx), -dz), axis=-1)
        return normal/np.linalg.norm(normal, axis=-1, keepdims=True)

    def normal_at(self, x, z): return self.normals(x,z)

    def bounds(self, key):
        level, ix, iz = key; size = self.config.side/(2**level); start = -self.config.side/2
        return start+ix*size, start+(ix+1)*size, start+iz*size, start+(iz+1)*size

    def excluded(self, key):
        a,b,c,d = self.bounds(key); e,f,g,h = TERRAIN_BOUNDS
        return e <= a and b <= f and g <= c and d <= h

    @staticmethod
    def children(key):
        level,x,z = key
        return [(level+1,2*x+i,2*z+j) for i in range(2) for j in range(2)]

    def mesh(self, key):
        a,b,c,d = self.bounds(key); e,f,g,h = TERRAIN_BOUNDS
        # Include exact compatibility edges at every level; cut out the old ground.
        xs = np.unique(np.r_[np.linspace(a,b,self.config.cells+1), [v for v in (e,f) if a < v < b]])
        zs = np.unique(np.r_[np.linspace(c,d,self.config.cells+1), [v for v in (g,h) if c < v < d]])
        x,z = np.meshgrid(xs,zs); y = self.heights(x,z); normal = self.normals(x,z)
        sun = np.array([-.4,.8,-.3]); sun /= np.linalg.norm(sun)
        light = .45+.55*np.maximum(normal@sun,0)
        rock = np.clip((1-normal[...,1])*2 + np.abs(y)/5000,0,1)[...,None]
        color = ((1-rock)*np.array([.57,.48,.36])+rock*np.array([.36,.35,.32]))*light[...,None]
        blend = self.compatibility_weight(x,z)[...,None]
        color = color*blend + np.array([.57,.48,.36])*(1-blend)
        vertices = np.concatenate((x[...,None],y[...,None],z[...,None],color),axis=-1).reshape(-1,6).tolist()
        indices = []; width = len(xs)
        for j in range(len(zs)-1):
            for i in range(len(xs)-1):
                cx,cz = (xs[i]+xs[i+1])/2,(zs[j]+zs[j+1])/2
                if e < cx < f and g < cz < h: continue
                k = j*width+i; indices.extend((k,k+width,k+1,k+1,k+width,k+width+1))
        edges = [list(range(width)), list(range((len(zs)-1)*width,len(zs)*width)), [j*width for j in range(len(zs))], [j*width+width-1 for j in range(len(zs))]]
        for edge in edges:
            for first,second in zip(edge,edge[1:]):
                cx,cz = (vertices[first][0]+vertices[second][0])/2,(vertices[first][2]+vertices[second][2])/2
                if e <= cx <= f and g <= cz <= h: continue
                start = len(vertices)
                for index in (first,second):
                    vertex = vertices[index].copy(); vertex[1] -= self.skirt_depth; vertices.append(vertex)
                indices.extend((first,second,start,second,start+1,start))
        return np.asarray(vertices,np.float32), np.asarray(indices,np.uint32)

class TerrainSelector:
    def __init__(self, terrain): self.terrain = terrain; self.split = set()

    def select(self, position, view_projection=None):
        terrain = self.terrain; config = terrain.config; position = np.asarray(position)
        planes = None
        if view_projection is not None:
            matrix = np.asarray(view_projection)
            planes = [matrix[3]+sign*matrix[axis] for axis in range(3) for sign in (-1,1)]
        @lru_cache(maxsize=None)
        def visible(key):
            if terrain.excluded(key): return False
            a,b,c,d = terrain.bounds(key)
            distance = np.linalg.norm([max(a-position[0],0,position[0]-b),max(c-position[2],0,position[2]-d)])
            if distance > config.view_distance: return False
            if planes is not None:
                # Conservative elevation bounds include every legal source height and flat zero.
                low,high = terrain.elevation_bounds
                for p in planes:
                    corner = np.array([b if p[0]>=0 else a, high if p[1]>=0 else low, d if p[2]>=0 else c,1])
                    if p@corner < 0: return False
            return True
        @lru_cache(maxsize=None)
        def refinement(key):
            a,b,c,d = terrain.bounds(key)
            distance = np.linalg.norm([max(a-position[0],0,position[0]-b),max(0,abs(position[1])-1000),max(c-position[2],0,position[2]-d)])
            threshold = (b-a)*1.3*(1+config.hysteresis if key in self.split else 1-config.hysteresis)
            children = tuple(child for child in terrain.children(key) if visible(child))
            return distance,threshold,b-a,children
        leaves = [(0,0,0)] if visible((0,0,0)) else []; split = set()
        while True:
            candidates = []
            for key in leaves:
                if key[0] >= config.max_level: continue
                distance,threshold,size,children = refinement(key)
                if distance < threshold and len(leaves)-1+len(children) <= config.budget:
                    candidates.append((distance/size,key,children))
            if not candidates: break
            _,key,children = min(candidates); leaves.remove(key); leaves.extend(children); split.add(key)
        self.split = split
        return sorted(leaves)
