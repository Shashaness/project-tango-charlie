"""Strict, standalone SRTM reader. Source heights are EGM96 meters."""
from collections import OrderedDict
from pathlib import Path
import re
import threading
import numpy as np

VOID = -32768

class HgtTile:
    def __init__(self, path):
        path = Path(path)
        match = re.fullmatch(r'([NS])(\d{2})([EW])(\d{3})\.hgt', path.name, re.I)
        if not match:
            raise ValueError('HGT filename must identify its southwest corner')
        ns, lat, ew, lon = match.groups()
        self.latitude = int(lat) * (1 if ns.upper() == 'N' else -1)
        self.longitude = int(lon) * (1 if ew.upper() == 'E' else -1)
        if not -90 <= self.latitude < 90 or not -180 <= self.longitude < 180:
            raise ValueError('Invalid tile coordinate')
        sizes = {1201 * 1201 * 2: 1201, 3601 * 3601 * 2: 3601}
        self.size = sizes.get(path.stat().st_size)
        if self.size is None:
            raise ValueError('HGT must contain exactly 1201² or 3601² signed samples')
        self.samples = np.memmap(path, dtype='>i2', mode='r', shape=(self.size, self.size))

    def sample(self, latitude, longitude):
        lat, lon = np.broadcast_arrays(np.asarray(latitude, float), np.asarray(longitude, float))
        valid = np.isfinite(lat) & np.isfinite(lon) & (lat >= self.latitude) & (lat <= self.latitude+1) & (lon >= self.longitude) & (lon <= self.longitude+1)
        row = np.clip(np.nan_to_num(self.latitude+1-lat), 0, 1)*(self.size-1)
        col = np.clip(np.nan_to_num(lon-self.longitude), 0, 1)*(self.size-1)
        row = np.where(np.abs(row-np.rint(row)) < 1e-8, np.rint(row), row)
        col = np.where(np.abs(col-np.rint(col)) < 1e-8, np.rint(col), col)
        r, c = np.minimum(row.astype(int), self.size-2), np.minimum(col.astype(int), self.size-2)
        fy, fx = row-r, col-c
        numerator = np.zeros(lat.shape); denominator = np.zeros(lat.shape)
        for dr, dc, weight in ((0,0,(1-fy)*(1-fx)), (0,1,(1-fy)*fx), (1,0,fy*(1-fx)), (1,1,fy*fx)):
            value = self.samples[r+dr, c+dc].astype(float)
            weight = weight * (value != VOID)
            numerator += value * weight; denominator += weight
        return np.divide(numerator, denominator, out=np.full(lat.shape, np.nan), where=valid & (denominator > 0))

class HgtDataset:
    """Memory mapped tiles with a bounded LRU; sampling is thread safe."""
    def __init__(self, directory, capacity=4):
        self.paths = {}; self.warnings = []; self.cache = OrderedDict()
        self.capacity = capacity; self.lock = threading.RLock()
        for path in sorted(p for p in Path(directory).glob('*') if p.suffix.lower() == '.hgt'):
            try:
                tile = HgtTile(path)
                self.paths[tile.latitude, tile.longitude] = path
                tile.samples._mmap.close()
            except ValueError as error:
                self.warnings.append(f'{path.name}: {error}')

    def sample(self, latitude, longitude):
        lat, lon = np.broadcast_arrays(np.asarray(latitude, float), np.asarray(longitude, float))
        result = np.full(lat.shape, np.nan)
        with self.lock:
            # Inclusive edges let adjacent tiles supply border samples if one is absent.
            for key, path in self.paths.items():
                south, west = key
                mask = np.isnan(result) & (lat >= south) & (lat <= south+1) & (lon >= west) & (lon <= west+1)
                if not np.any(mask): continue
                if key not in self.cache:
                    self.cache[key] = HgtTile(path)
                tile = self.cache[key]; self.cache.move_to_end(key)
                result[mask] = tile.sample(lat[mask], lon[mask])
                while len(self.cache) > self.capacity:
                    _, old = self.cache.popitem(last=False); old.samples._mmap.close()
        return result

    def elevation_range(self, keys=None):
        low = high = 0.
        for key, path in self.paths.items():
            if keys is not None and key not in keys: continue
            tile = HgtTile(path)
            for start in range(0, tile.size, 128):
                samples = tile.samples[start:start+128]
                values = samples[samples != VOID]
                if values.size:
                    low = min(low, float(values.min())); high = max(high, float(values.max()))
            tile.samples._mmap.close()
        return low, high

    def close(self):
        with self.lock:
            for tile in self.cache.values(): tile.samples._mmap.close()
            self.cache.clear()

class LocalProjection:
    """Local WGS84 ellipsoid linearization: east X, north -Z, meters."""
    def __init__(self, latitude=34.5, longitude=-111.5):
        if not np.isfinite(latitude) or not np.isfinite(longitude) or not -80 <= latitude <= 80 or not -180 <= longitude <= 180:
            raise ValueError('Origin must be finite, nonpolar geographic coordinates')
        self.latitude, self.longitude = latitude, longitude
        phi = np.radians(latitude); e2 = 6.69437999014e-3; a = 6378137.
        factor = 1-e2*np.sin(phi)**2
        self.east_scale = a*np.cos(phi)/np.sqrt(factor)*np.pi/180
        self.north_scale = a*(1-e2)/factor**1.5*np.pi/180

    def to_world(self, latitude, longitude):
        delta = (np.asarray(longitude)-self.longitude+180)%360-180
        return delta*self.east_scale, -(np.asarray(latitude)-self.latitude)*self.north_scale

    def to_geographic(self, x, z):
        return self.latitude-np.asarray(z)/self.north_scale, (self.longitude+np.asarray(x)/self.east_scale+180)%360-180
