"""Streaming forest: bounded CPU jobs, GL ownership, exclusive parent fallback."""
from collections import OrderedDict,Counter
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from engine.mesh import Mesh
from game.terrain import TerrainSelector
from game.geography import GeographicTile

class TerrainResources:
    def __init__(self, terrain, mesh_factory=Mesh):
        self.terrain=terrain;self.factory=mesh_factory;self.selector=TerrainSelector(terrain)
        self.cache=OrderedDict();self.cpu_cache=OrderedDict();self.cpu_bytes=0
        self.pending={};self.fallbacks=set();self.active=[];self.stats={}
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='terrain')
        self.closed=False;self.diagnostic=None;self.diagnostic_pending=None;self.diagnostic_position=None
        self._upload((0,0,0),terrain.mesh((0,0,0),local=True))

    def _upload(self, key, data):
        old=self.cache.pop(key,None)
        if old is not None:old.close()
        self.cache[key]=self.factory(*data)
        self.cache[key].terrain_bytes=sum(array.nbytes for array in data)

    def _remember(self, key, data):
        if key in self.cpu_cache:self.cpu_bytes-=sum(a.nbytes for a in self.cpu_cache.pop(key))
        size=sum(a.nbytes for a in data)
        if size>self.terrain.config.cpu_cache_bytes:return
        self.cpu_cache[key]=data;self.cpu_bytes+=size
        while len(self.cpu_cache)>self.terrain.config.cpu_cache_size or self.cpu_bytes>self.terrain.config.cpu_cache_bytes:
            _,old=self.cpu_cache.popitem(last=False);self.cpu_bytes-=sum(a.nbytes for a in old)

    def _diagnose(self, position):
        lat,lon=self.terrain.projection.to_geographic(position[0],position[2])
        return (position.copy(),self.terrain.height_at(position[0],position[2]),self.terrain.provider.source_at(float(lat),float(lon)))

    def update(self, position, view_projection=None, vehicle_position=None, velocity=None):
        position=np.asarray(position,float)
        desired=self.selector.select(position,view_projection)
        current_roots={self.terrain.root_at(*((self.terrain.bounds(k)[0]+self.terrain.bounds(k)[1])/2,
                                                (self.terrain.bounds(k)[2]+self.terrain.bounds(k)[3])/2)) for k in desired}
        target=position if vehicle_position is None else np.asarray(vehicle_position,float)
        velocity=np.zeros(3) if velocity is None else np.asarray(velocity,float)
        predicted=target+velocity*self.terrain.config.prefetch_seconds
        # Clamp speculative lead, so a debug teleport/extreme speed cannot queue a planet.
        lead=predicted-target;distance=np.linalg.norm(lead)
        if distance>self.terrain.config.side:predicted=target+lead*self.terrain.config.side/distance
        prefetched=set(self.terrain.roots_near(predicted))|{self.terrain.root_at(target[0],target[2])}
        wanted={(0,0,0)}|current_roots|prefetched
        for level,x,z in desired:
            while level>0:
                wanted.add((level,x,z));level,x,z=level-1,x//2,z//2
        priority=lambda key:(0 if key in current_roots else 1 if key in prefetched else 2,key[0],
            np.hypot((sum(self.terrain.bounds(key)[:2])/2)-position[0],(sum(self.terrain.bounds(key)[2:])/2)-position[2]))
        wanted=set(sorted(wanted,key=priority)[:self.terrain.config.cache_size-1])|{(0,0,0)}
        protected=wanted|current_roots
        def make_room():
            while len(self.cache)>=self.terrain.config.cache_size:
                victim=next((key for key in self.cache if key not in protected),None)
                if victim is None:return False
                self.cache.pop(victim).close();self.fallbacks.discard(victim)
            return True
        for key,future in list(self.pending.items()):
            if key not in wanted and future.cancel():del self.pending[key]
        uploaded=0
        for key,future in list(self.pending.items()):
            if uploaded>=2:break
            if not future.done():continue
            del self.pending[key];data=future.result() # programming errors propagate
            self._remember(key,data)
            if key in wanted and (key in self.cache or make_room()):
                self._upload(key,data);self.fallbacks.discard(key);uploaded+=1
        # Immediate bounded safety geometry: nine procedural vertices, no raster IO,
        # no normal scan. Ordinary motion generally reaches prefetched real roots.
        for key in sorted(current_roots):
            if key not in self.cache:
                if not make_room():raise RuntimeError('Cache cannot hold visible root coverage')
                self._upload(key,self.terrain.mesh(key,procedural_only=True,cells=2,local=True,coarse_fallback=True))
                self.fallbacks.add(key)
        for key in sorted(wanted,key=priority):
            if len(self.pending)>=8:break
            if (key not in self.cache or key in self.fallbacks) and key not in self.pending:
                if key in self.cpu_cache:
                    if uploaded<2 and (key in self.cache or make_room()):
                        self._upload(key,self.cpu_cache[key]);self.fallbacks.discard(key);uploaded+=1
                        self.cpu_cache.move_to_end(key)
                else:self.pending[key]=self.executor.submit(self.terrain.mesh,key,local=True)
        def cover(key):
            descendants=[k for k in desired if k[0]>=key[0] and k[1]//2**(k[0]-key[0])==key[1] and k[2]//2**(k[0]-key[0])==key[2]]
            if not descendants:return []
            if key in desired:return [key] if key in self.cache else None
            covered=[]
            for child in self.terrain.children(key):
                result=cover(child)
                if result is None:return [key] if key in self.cache else None
                covered.extend(result)
            return covered
        self.active=[key for root in sorted(current_roots) for key in (cover(root) or [root])]
        for key in self.active:self.cache.move_to_end(key)
        if self.diagnostic_pending is not None and self.diagnostic_pending.done():
            self.diagnostic=self.diagnostic_pending.result();self.diagnostic_pending=None
        if self.diagnostic_pending is None and (self.diagnostic_position is None or np.linalg.norm(position-self.diagnostic_position)>25):
            self.diagnostic_position=position.copy()
            self.diagnostic_pending=self.executor.submit(self._diagnose,position.copy())
        lat,lon=self.terrain.projection.to_geographic(position[0],position[2]);levels=[k[0] for k in self.active]
        fresh=self.diagnostic is not None and np.linalg.norm(position-self.diagnostic[0])<=100
        sampled_height=self.diagnostic[1] if fresh else float(self.terrain.heights(position[0],position[2],procedural_only=True))
        self.stats=dict(lod=f'{min(levels)}-{max(levels)}' if levels else 'NONE',patches=len(self.active),
            triangles=sum(self.cache[k].count//3 for k in self.active),agl=float(position[1])-sampled_height,
            tiles=len(self.terrain.dataset.cache),raster_bytes=self.terrain.dataset.resident_bytes,
            cached=len(self.cache),cpu_cached=len(self.cpu_cache),cpu_bytes=self.cpu_bytes,
            gpu_bytes=sum(m.terrain_bytes for m in self.cache.values()),pending=len(self.pending)+(self.diagnostic_pending is not None),
            latitude=float(lat),longitude=float(lon),tile=GeographicTile.at(float(lat),float(lon)).identifier,
            source=self.diagnostic[2] if fresh else 'PENDING',agl_estimated=not fresh,warnings=len(self.terrain.dataset.warnings),lod_distribution=dict(Counter(levels)),fallbacks=len(self.fallbacks))

    def draw(self, shader=None, camera=None):
        if shader is not None:
            view=camera.view_matrix().copy();view[:3,3]=0
            shader.set_matrix('view',view)
        for key in self.active:
            if shader is not None:
                a,b,c,d=self.terrain.bounds(key)
                model=np.eye(4,dtype=np.float32)
                model[:3,3]=np.array([(a+b)/2,0.,(c+d)/2])-camera.position
                shader.set_matrix('model',model)
            self.cache[key].draw()
        if shader is not None:
            shader.set_matrix('view',camera.view_matrix());shader.set_matrix('model',np.eye(4,dtype=np.float32))

    def close(self):
        if self.closed:return
        self.closed=True
        for future in self.pending.values():future.cancel()
        if self.diagnostic_pending is not None:self.diagnostic_pending.cancel()
        self.executor.shutdown(wait=True,cancel_futures=True)
        self.pending.clear()
        for mesh in self.cache.values():mesh.close()
        self.cache.clear();self.cpu_cache.clear();self.cpu_bytes=0;self.active.clear()
        self.terrain.dataset.close()
