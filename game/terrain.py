"""LOD-independent height field and bounded quadtree terrain geometry."""
from dataclasses import dataclass
from pathlib import Path
from functools import lru_cache
import numpy as np
from engine.asset_paths import PROJECT_ROOT
from game.hgt import LocalProjection
from game.elevation import ElevationDataset
from game.geographic_registry import GeographicRegistry
from game.planetary_elevation import PlanetaryElevation
from game.geography import GeographicFrame,GeographicTile
from game.terrain_compatibility import CompatibilityMask

@lru_cache(maxsize=1)
def default_footprints():
    from game.world_environment import WorldEnvironment
    return WorldEnvironment().protected_footprints

@dataclass(frozen=True)
class TerrainConfig:
    directory: Path = PROJECT_ROOT / 'hgt'
    registry_directory: Path = PROJECT_ROOT / 'assets/planets/earth/tiles'
    planetary_seed: int = 205
    real_blend_width: float = 750.
    prefetch_seconds: float = 4.
    cpu_cache_size: int = 32
    cpu_cache_bytes: int = 32*1024**2
    latitude: float = 34.5
    longitude: float = -111.5
    elevation_offset: float | None = None
    side: float = 32000.
    view_distance: float = 16000.
    budget: int = 64
    max_level: int = 5
    cells: int = 32
    cache_size: int = 96
    blend_width: float = 450.
    flat_clearance: float = 150.
    transition_variation: float = .15
    transition_wavelength: float = 1800.
    compatibility_seed: int = 20
    compatibility_join_width: float = 100.
    compatibility_cell_size: float = 50.
    hysteresis: float = .15

    def __post_init__(self):
        if not 22000 <= self.side <= 64000 or not 1000 <= self.view_distance <= 20000:
            raise ValueError('Terrain side must be 22–64 km and view distance 1–20 km')
        if self.side < 2*self.view_distance:
            raise ValueError('Root patch span must cover the view diameter')
        if not 4 <= self.budget <= 128 or not self.budget+1 <= self.cache_size <= 256:
            raise ValueError('Cache must exceed the 4–128 patch budget')
        if not 1 <= self.max_level <= 6 or not 4 <= self.cells <= 64 or not 0 <= self.hysteresis < .5 or not 0 < self.blend_width <= 5000:
            raise ValueError('Invalid terrain subdivision or blend configuration')
        if not 0 <= self.flat_clearance <= 500 or not 0 <= self.transition_variation <= .3 or not 1000 <= self.transition_wavelength <= 10000:
            raise ValueError('Invalid compatibility clearance or low-frequency transition variation')
        if not 0 <= self.compatibility_join_width <= 200 or not 25 <= self.compatibility_cell_size <= 100:
            raise ValueError('Invalid compatibility join smoothing or mesh sample spacing')
        if not isinstance(self.planetary_seed,int) or not 100 <= self.real_blend_width <= 5000 or not 0 <= self.prefetch_seconds <= 10 or not 4 <= self.cpu_cache_size <= 128 or self.cpu_cache_bytes < 1024**2:
            raise ValueError('Invalid planetary source/streaming settings')
        LocalProjection(self.latitude,self.longitude)
        if self.elevation_offset is not None and not np.isfinite(self.elevation_offset):
            raise ValueError('Elevation offset must be finite')

class Terrain:
    def __init__(self, config=None, footprints=None):
        self.config = config or TerrainConfig()
        self.compatibility = CompatibilityMask(footprints or default_footprints(),
            self.config.flat_clearance,self.config.blend_width,self.config.transition_variation,
            self.config.transition_wavelength,self.config.compatibility_seed,self.config.compatibility_join_width)
        self.projection = LocalProjection(self.config.latitude, self.config.longitude)
        self.registry = GeographicRegistry(self.config.registry_directory,self.config.directory)
        self.dataset = ElevationDataset(self.config.directory,paths=self.registry.elevation_paths)
        self.dataset.warnings[:0]=self.registry.warnings
        self.provider = PlanetaryElevation(self.dataset,self.projection,self.config.planetary_seed,self.config.real_blend_width)
        self.enabled = True
        origin = float(self.provider.sample(self.config.latitude,self.config.longitude))
        self.offset = self.config.elevation_offset if self.config.elevation_offset is not None else origin
        self.frame = GeographicFrame(self.projection,self.offset)
        # Conservative universal source bounds avoid rescanning when the window moves.
        self.elevation_bounds = min(0.,-32768-self.offset),max(0.,32767-self.offset)
        self.skirt_depth = self.elevation_bounds[1]-self.elevation_bounds[0]+64.

    def heights(self, x, z, procedural_only=False):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        lat,lon=self.projection.to_geographic(x,z)
        raw=(self.provider.procedural.sample(lat,lon) if procedural_only else self.provider.sample(lat,lon))-self.offset
        return raw*self.compatibility_weight(x,z)

    def compatibility_weight(self, x, z):
        return self.compatibility.weight(x,z)

    def height_at(self, x, z): return float(self.heights(x, z))

    def normals(self, x, z, procedural_only=False):
        step = 10.
        dx = (self.heights(np.asarray(x)+step,z,procedural_only)-self.heights(np.asarray(x)-step,z,procedural_only))/(2*step)
        dz = (self.heights(x,np.asarray(z)+step,procedural_only)-self.heights(x,np.asarray(z)-step,procedural_only))/(2*step)
        normal = np.stack((-dx, np.ones_like(dx), -dz), axis=-1)
        return normal/np.linalg.norm(normal, axis=-1, keepdims=True)

    def normal_at(self, x, z): return self.normals(x,z)

    def bounds(self, key):
        level, ix, iz = key; size = self.config.side/(2**level); start = -self.config.side/2
        return start+ix*size, start+(ix+1)*size, start+iz*size, start+(iz+1)*size

    def root_at(self, x, z):
        half=self.config.side/2
        return (0,int(np.floor((x+half)/self.config.side)),int(np.floor((z+half)/self.config.side)))

    def roots_near(self, position, margin=0.):
        distance=self.config.view_distance+margin
        first=self.root_at(position[0]-distance,position[2]-distance)
        last=self.root_at(position[0]+distance,position[2]+distance)
        return [(0,x,z) for x in range(first[1],last[1]+1) for z in range(first[2],last[2]+1)]

    def geographic_identity(self, key):
        a,b,c,d=self.bounds(key);lat,lon=self.projection.to_geographic((a+b)/2,(c+d)/2)
        return ('earth',self.config.latitude,self.config.longitude,self.config.side,GeographicTile.at(float(lat),float(lon)).identifier,*key)

    def excluded(self, key):
        return self.compatibility.excludes_patch(self.bounds(key))

    @staticmethod
    def children(key):
        level,x,z = key
        return [(level+1,2*x+i,2*z+j) for i in range(2) for j in range(2)]

    def mesh(self, key, procedural_only=False, cells=None, local=False, coarse_fallback=False):
        a,b,c,d = self.bounds(key)
        # Insert every retained ground edge, so exclusion is an exact partition
        # at all levels rather than dropping triangles by an approximate mask.
        footprints = self.compatibility.footprints
        xs = np.unique(np.r_[np.linspace(a,b,(self.config.cells if cells is None else cells)+1),
            [v for r in footprints for v in r[:2] if a < v < b]])
        zs = np.unique(np.r_[np.linspace(c,d,(self.config.cells if cells is None else cells)+1),
            [v for r in footprints for v in r[2:] if c < v < d]])
        # Stable subgrid near infrastructure resolves grading even on the pinned
        # root fallback; rows outside this compact neighborhood retain normal LOD.
        radius = self.config.flat_clearance+self.config.blend_width*(1+self.config.transition_variation)+self.config.compatibility_join_width/4
        detail_step = self.config.compatibility_cell_size
        for e,f,g,h in footprints:
            if f+radius < a or e-radius > b or h+radius < c or g-radius > d: continue
            xs = np.unique(np.r_[xs,np.arange(np.ceil(max(a,e-radius)/detail_step)*detail_step,min(b,f+radius)+1e-8,detail_step)])
            zs = np.unique(np.r_[zs,np.arange(np.ceil(max(c,g-radius)/detail_step)*detail_step,min(d,h+radius)+1e-8,detail_step)])
        x,z = np.meshgrid(xs,zs); y = self.heights(x,z,procedural_only); normal = np.broadcast_to([0.,1.,0.],(*x.shape,3)) if coarse_fallback else self.normals(x,z,procedural_only)
        sun = np.array([-.4,.8,-.3]); sun /= np.linalg.norm(sun)
        light = .45+.55*np.maximum(normal@sun,0)
        rock = np.clip((1-normal[...,1])*2 + np.abs(y)/5000,0,1)[...,None]
        color = ((1-rock)*np.array([.57,.48,.36])+rock*np.array([.36,.35,.32]))*light[...,None]
        blend = self.compatibility_weight(x,z)[...,None]
        color = color*blend + np.array([.57,.48,.36])*(1-blend)
        vertices = np.concatenate((x[...,None],y[...,None],z[...,None],color),axis=-1).reshape(-1,6).tolist()
        indices = []; width = len(xs)
        cx,cz = np.meshgrid((xs[:-1]+xs[1:])/2,(zs[:-1]+zs[1:])/2)
        excluded = self.compatibility.contains(cx,cz,strict=True)
        for j in range(len(zs)-1):
            for i in range(len(xs)-1):
                if excluded[j,i]: continue
                k = j*width+i; indices.extend((k,k+width,k+1,k+1,k+width,k+width+1))
        edges = [list(range(width)), list(range((len(zs)-1)*width,len(zs)*width)), [j*width for j in range(len(zs))], [j*width+width-1 for j in range(len(zs))]]
        for edge in edges:
            for first,second in zip(edge,edge[1:]):
                cx,cz = (vertices[first][0]+vertices[second][0])/2,(vertices[first][2]+vertices[second][2])/2
                if self.compatibility.contains(cx,cz): continue
                start = len(vertices)
                for index in (first,second):
                    vertex = vertices[index].copy(); vertex[1] -= self.skirt_depth; vertices.append(vertex)
                indices.extend((first,second,start,second,start+1,start))
        vertices=np.asarray(vertices,float)
        if local:
            vertices[:,0]-=(a+b)/2;vertices[:,2]-=(c+d)/2
        return vertices.astype(np.float32),np.asarray(indices,np.uint32)

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
        leaves = [key for key in terrain.roots_near(position) if visible(key)]; split = set()
        while True:
            candidates = []
            for key in leaves:
                if key[0] >= config.max_level: continue
                distance,threshold,size,children = refinement(key)
                if distance < threshold and len(leaves)-1+len(children) <= config.budget:
                    candidates.append((0 if key in self.split else 1,distance/size,key,children))
            if not candidates: break
            _,_,key,children = min(candidates); leaves.remove(key); leaves.extend(children); split.add(key)
        self.split = split
        return sorted(leaves)
