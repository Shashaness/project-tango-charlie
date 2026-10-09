"""CPU meshes on one worker; bounded uploads and GL ownership on render thread."""
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from engine.mesh import Mesh
from game.terrain import TerrainSelector

class TerrainResources:
    def __init__(self, terrain, mesh_factory=Mesh):
        self.terrain = terrain; self.factory = mesh_factory
        self.selector = TerrainSelector(terrain)
        self.cache = OrderedDict(); self.pending = {}; self.active = []
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='terrain')
        self.closed = False; self.stats = {}
        # A pinned startup mesh supplies complete coverage while children load.
        self.cache[(0,0,0)] = mesh_factory(*terrain.mesh((0,0,0)))

    def update(self, position, view_projection=None):
        desired = self.selector.select(position, view_projection)
        wanted = {(0,0,0)}
        for key in desired:
            level,x,z = key
            while level:
                wanted.add((level,x,z)); level,x,z = level-1,x//2,z//2
        # Restrict the working set too: ancestors count against the cache budget.
        wanted = set(sorted(wanted)[:self.terrain.config.cache_size])
        for key,future in list(self.pending.items()):
            if key not in wanted and future.cancel(): del self.pending[key]
        uploaded = 0
        for key,future in list(self.pending.items()):
            if uploaded >= 2: break
            if not future.done(): continue
            del self.pending[key]
            vertices,indices = future.result()
            if key not in wanted: continue
            while len(self.cache) >= self.terrain.config.cache_size:
                victim = next((k for k in self.cache if k != (0,0,0) and k not in wanted), None)
                if victim is None: break
                self.cache.pop(victim).close()
            if len(self.cache) < self.terrain.config.cache_size:
                self.cache[key] = self.factory(vertices,indices); uploaded += 1
        for key in sorted(wanted):
            if len(self.pending) >= 8: break
            if key not in self.cache and key not in self.pending:
                self.pending[key] = self.executor.submit(self.terrain.mesh,key)
        # Parent fallback is exclusive: never draw it together with descendants.
        def cover(key):
            descendants = [k for k in desired if k[0] >= key[0] and k[1]//2**(k[0]-key[0]) == key[1] and k[2]//2**(k[0]-key[0]) == key[2]]
            if not descendants: return []
            if key in desired: return [key] if key in self.cache else None
            covered = []
            for child in self.terrain.children(key):
                result = cover(child)
                if result is None: return [key] if key in self.cache else None
                covered.extend(result)
            return covered
        self.active = cover((0,0,0)) or []
        for key in self.active: self.cache.move_to_end(key)
        levels = [key[0] for key in self.active]
        self.stats = dict(lod=f'{min(levels)}-{max(levels)}' if levels else 'NONE', patches=len(self.active), triangles=sum(self.cache[k].count//3 for k in self.active), agl=float(position[1])-self.terrain.height_at(position[0],position[2]), tiles=len(self.terrain.dataset.cache), cached=len(self.cache), pending=len(self.pending))

    def draw(self):
        for key in self.active: self.cache[key].draw()

    def close(self):
        if self.closed: return
        self.closed = True
        for future in self.pending.values(): future.cancel()
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.pending.clear()
        for mesh in self.cache.values(): mesh.close()
        self.cache.clear(); self.active.clear()
        self.terrain.dataset.close()
