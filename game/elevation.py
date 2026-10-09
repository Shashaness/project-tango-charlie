"""Metadata-indexed HGT/GeoTIFF elevation sources with bounded raster residency.

Supported GeoTIFFs are single-band, north-up EPSG:4326 rasters in degrees.
No CRS transformation is implicit. Geometry remains independent of raster tiling.
"""
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
import threading
import numpy as np
import tifffile
from game.hgt import HgtTile, VOID

_TOLERANCE = 1e-10
_SUPPORTED_COMPRESSION = {1, 8, 32946}  # uncompressed, Adobe/legacy DEFLATE

@dataclass(frozen=True)
class RasterInfo:
    path: Path
    format: str
    shape: tuple
    dtype: np.dtype
    west: float
    east: float
    south: float
    north: float
    x_step: float
    y_step: float
    raster_type: int = 2  # GeoTIFF RasterPixelIsPoint
    nodata: float | None = None

    @property
    def nbytes(self):
        return int(np.prod(self.shape)) * self.dtype.itemsize

    @property
    def sample_west(self):
        return self.west + (self.x_step/2 if self.raster_type == 1 else 0)

    @property
    def sample_north(self):
        return self.north - (self.y_step/2 if self.raster_type == 1 else 0)

    @property
    def sample_bounds(self):
        return (self.sample_west, self.sample_west+(self.shape[1]-1)*self.x_step,
                self.sample_north-(self.shape[0]-1)*self.y_step, self.sample_north)

    def contains(self, latitude, longitude, centers=False):
        west,east,south,north = self.sample_bounds if centers else (self.west,self.east,self.south,self.north)
        return ((latitude >= south-_TOLERANCE) & (latitude <= north+_TOLERANCE)
                & (longitude >= west-_TOLERANCE) & (longitude <= east+_TOLERANCE))

    def intersects(self, south, west, north, east):
        longitude_overlap = ((self.west <= east and self.east >= west) if west <= east
                             else (self.west <= east or self.east >= west))
        return self.south <= north and self.north >= south and longitude_overlap


def inspect_raster(path):
    """Read metadata only (HGT validation briefly opens and closes its mapping)."""
    path = Path(path)
    if path.suffix.lower() == '.hgt':
        tile = HgtTile(path)
        try:
            return RasterInfo(path,'hgt',tile.samples.shape,tile.samples.dtype,
                              tile.longitude,tile.longitude+1,tile.latitude,tile.latitude+1,
                              1/(tile.size-1),1/(tile.size-1),nodata=VOID)
        finally:
            tile.samples._mmap.close()
    if path.suffix.lower() not in ('.tif','.tiff'):
        raise ValueError('Unsupported elevation file extension')
    with tifffile.TiffFile(path) as file:
        if len(file.pages) != 1:
            raise ValueError('GeoTIFF must contain one elevation image')
        page = file.pages[0]
        if len(page.shape) != 2 or min(page.shape) < 2 or page.samplesperpixel != 1 or page.dtype.kind not in 'iuf':
            raise ValueError('GeoTIFF must be a numeric single-band raster of at least 2 × 2 pixels')
        geo = file.geotiff_metadata or {}
        if (int(geo.get('GTModelTypeGeoKey',0)) != 2
                or int(geo.get('GeographicTypeGeoKey',0)) != 4326
                or int(geo.get('GeogAngularUnitsGeoKey',9102)) != 9102
                or geo.get('ProjectedCSTypeGeoKey') is not None):
            raise ValueError('Unsupported GeoTIFF CRS: only geographic WGS84 EPSG:4326 in degrees is supported')
        if int(geo.get('VerticalUnitsGeoKey',9001)) != 9001 or int(geo.get('VerticalCSTypeGeoKey',5773)) != 5773:
            raise ValueError('Unsupported GeoTIFF vertical reference: expected SRTM EGM96 heights in meters')
        raster_type = int(geo.get('GTRasterTypeGeoKey',1))
        if raster_type not in (1,2):
            raise ValueError('Unsupported GeoTIFF raster type')
        if int(page.compression) not in _SUPPORTED_COMPRESSION:
            raise ValueError('Unsupported TIFF compression: use uncompressed or DEFLATE elevation rasters')
        if int(page.predictor) not in (1,2):
            raise ValueError('Unsupported TIFF predictor')
        transform = geo.get('ModelTransformation')
        if transform is not None:
            matrix = np.asarray(transform,float).reshape(4,4)
            expected = np.diag([matrix[0,0],matrix[1,1],matrix[2,2],1.])
            expected[:3,3] = matrix[:3,3]
            if not np.allclose(matrix,expected,rtol=0,atol=1e-12):
                raise ValueError('Rotated or sheared GeoTIFF transforms are unsupported')
            x_step,y_step = matrix[0,0],-matrix[1,1]
            west,north = matrix[0,3],matrix[1,3]
        else:
            scale,tiepoint = geo.get('ModelPixelScale'),geo.get('ModelTiepoint')
            if scale is None or tiepoint is None or len(scale) != 3 or len(tiepoint) < 6 or len(tiepoint)%6:
                raise ValueError('GeoTIFF requires pixel scale and tiepoint, or an axis-aligned transform')
            x_step,y_step = scale[:2]
            i,j,_,x,y,_ = tiepoint[:6]
            west,north = x-i*x_step,y+j*y_step
            for k in range(6,len(tiepoint),6):
                i,j,_,x,y,_ = tiepoint[k:k+6]
                if not np.allclose((x-i*x_step,y+j*y_step),(west,north),rtol=0,atol=1e-10):
                    raise ValueError('Inconsistent GeoTIFF tiepoints')
        if not np.isfinite([west,north,x_step,y_step]).all() or x_step <= 0 or y_step <= 0:
            raise ValueError('GeoTIFF must have finite north-up geographic pixel dimensions')
        rows,cols = page.shape
        east = west+x_step*(cols if raster_type == 1 else cols-1)
        south = north-y_step*(rows if raster_type == 1 else rows-1)
        if west < -180-_TOLERANCE or east > 180+_TOLERANCE or south < -90-_TOLERANCE or north > 90+_TOLERANCE:
            raise ValueError('GeoTIFF bounds are outside geographic coordinates')
        nodata = page.tags.get(42113)
        try:
            nodata = float(str(nodata.value).strip('\x00 ')) if nodata else None
        except (ValueError,TypeError) as error:
            raise ValueError('Invalid GeoTIFF NoData marker') from error
        return RasterInfo(path,'geotiff',page.shape,page.dtype,west,east,south,north,
                          x_step,y_step,raster_type,nodata)


class ElevationTile:
    """One raster, mapped when possible, decoded only within the byte budget."""
    def __init__(self, info):
        self.info = info
        if info.format == 'hgt':
            self.samples = HgtTile(info.path).samples
        else:
            try:
                self.samples = tifffile.memmap(info.path,page=0,mode='r')
            except ValueError:
                # Supported DEFLATE is decoded to disk-backed memory, not a huge
                # resident ndarray. tifffile processes strips with one worker.
                with tifffile.TiffFile(info.path) as file:
                    self.samples = file.pages[0].asarray(out='memmap',maxworkers=1)
        self.closed = False

    def valid(self, values):
        mask = np.isfinite(values)
        if self.info.nodata is not None:
            mask &= values != self.info.nodata
        return mask

    def sample(self, latitude, longitude, neighbor_sample=None):
        lat,lon = np.broadcast_arrays(np.asarray(latitude,float),np.asarray(longitude,float))
        info = self.info; rows,cols = info.shape
        valid = np.isfinite(lat) & np.isfinite(lon) & info.contains(lat,lon)
        row = np.nan_to_num((info.sample_north-lat)/info.y_step,nan=0.,posinf=0.,neginf=0.)
        col = np.nan_to_num((lon-info.sample_west)/info.x_step,nan=0.,posinf=0.,neginf=0.)
        row = np.clip(row,-.5,rows-.5); col = np.clip(col,-.5,cols-.5)
        row = np.where(np.abs(row-np.rint(row))<1e-8,np.rint(row),row)
        col = np.where(np.abs(col-np.rint(col))<1e-8,np.rint(col),col)
        r,c = np.floor(row).astype(int),np.floor(col).astype(int)
        fy,fx = row-r,col-c
        numerator = np.zeros(lat.shape); denominator = np.zeros(lat.shape)
        for dr,dc,weight in ((0,0,(1-fy)*(1-fx)),(0,1,(1-fy)*fx),(1,0,fy*(1-fx)),(1,1,fy*fx)):
            rr,cc = r+dr,c+dc
            inside = (rr>=0)&(rr<rows)&(cc>=0)&(cc<cols)
            values = np.where(inside,self.samples[np.clip(rr,0,rows-1),np.clip(cc,0,cols-1)],np.nan).astype(float)
            outside = ~inside & valid & (weight>0)
            if neighbor_sample is not None and np.any(outside):
                values[outside] = neighbor_sample(info.sample_north-rr[outside]*info.y_step,
                                                 info.sample_west+cc[outside]*info.x_step)
            accepted = self.valid(values)
            numerator += np.where(accepted,values,0)*weight
            denominator += accepted*weight
        return np.divide(numerator,denominator,out=np.full(lat.shape,np.nan),where=valid&(denominator>0))

    def elevation_range(self):
        low=high=0.
        for start in range(0,self.info.shape[0],128):
            block=self.samples[start:start+128]; values=block[self.valid(block)]
            if values.size: low=min(low,float(values.min())); high=max(high,float(values.max()))
        return low,high

    def close(self):
        if self.closed: return
        self.closed=True
        mapping=getattr(self.samples,'_mmap',None)
        if mapping is not None: mapping.close()


class ElevationDataset:
    """Common elevation interface; metadata indexing does not decode rasters."""
    def __init__(self, directory, capacity=4, cache_bytes=128*1024**2, max_tile_bytes=64*1024**2):
        if capacity < 1 or cache_bytes < 1 or max_tile_bytes < 1:
            raise ValueError('Elevation cache limits must be positive')
        self.capacity=capacity; self.cache_bytes=cache_bytes
        self.max_tile_bytes=min(max_tile_bytes,cache_bytes)
        self.rasters={}; self.paths={}; self.warnings=[]; self.cache=OrderedDict()
        self.lock=threading.RLock(); self.resident_bytes=0
        for path in sorted(p for p in Path(directory).glob('*') if p.suffix.lower() in ('.hgt','.tif','.tiff')):
            try:
                info=inspect_raster(path)
                if info.nbytes > self.max_tile_bytes:
                    raise ValueError(f'Raster exceeds {self.max_tile_bytes} byte per-tile limit')
                self.rasters[path]=info; self.paths[path]=path
            except (ValueError,OSError,tifffile.TiffFileError) as error:
                self.warnings.append(f'{path.name}: {error}')
        self._neighbors={}
        for path,info in self.rasters.items():
            others=[other for key,other in self.rasters.items() if key!=path]
            self._neighbors[path]=(
                [o for o in others if o.west < info.west-_TOLERANCE and o.east >= info.west-_TOLERANCE],
                [o for o in others if o.east > info.east+_TOLERANCE and o.west <= info.east+_TOLERANCE],
                [o for o in others if o.south < info.south-_TOLERANCE and o.north >= info.south-_TOLERANCE],
                [o for o in others if o.north > info.north+_TOLERANCE and o.south <= info.north+_TOLERANCE])

    def _tile(self, path):
        if path in self.cache:
            self.cache.move_to_end(path); return self.cache[path]
        info=self.rasters[path]
        while self.cache and (len(self.cache)>=self.capacity or self.resident_bytes+info.nbytes>self.cache_bytes):
            _,old=self.cache.popitem(last=False)
            self.resident_bytes-=old.info.nbytes; old.close()
        tile=ElevationTile(info); self.cache[path]=tile; self.resident_bytes+=info.nbytes
        return tile

    def _center_samples(self, latitude, longitude):
        # Copy corner values before any further cache lookup can evict their tile.
        result=np.full(latitude.shape,np.nan)
        for path,info in self.rasters.items():
            mask=np.isnan(result)&info.contains(latitude,longitude,centers=True)
            if np.any(mask): result[mask]=self._tile(path).sample(latitude[mask],longitude[mask])
        return result

    def sample(self, latitude, longitude):
        lat,lon=np.broadcast_arrays(np.asarray(latitude,float),np.asarray(longitude,float))
        result=np.full(lat.shape,np.nan)
        with self.lock:
            for path,info in self.rasters.items():
                mask=np.isnan(result)&info.contains(lat,lon)
                if not np.any(mask): continue
                # PixelIsArea boundary interpolation can access neighboring rasters.
                # Gather those samples first to avoid evicting a live current mapping.
                row=(info.sample_north-lat[mask])/info.y_step
                col=(lon[mask]-info.sample_west)/info.x_step
                corners={}
                for dr,dc in ((0,0),(0,1),(1,0),(1,1)):
                    rr=np.floor(row+1e-10).astype(int)+dr; cc=np.floor(col+1e-10).astype(int)+dc
                    outside=(rr<0)|(rr>=info.shape[0])|(cc<0)|(cc>=info.shape[1])
                    if np.any(outside):
                        corner_lat=info.sample_north-rr[outside]*info.y_step
                        corner_lon=info.sample_west+cc[outside]*info.x_step
                        values=self._center_samples(corner_lat,corner_lon)
                        for a,b,value in zip(corner_lat,corner_lon,values): corners[(round(float(a),12),round(float(b),12))]=value
                def neighbor_sample(a,b):
                    return np.asarray([corners.get((round(float(x),12),round(float(y),12)),np.nan) for x,y in zip(a,b)])
                result[mask]=self._tile(path).sample(lat[mask],lon[mask],neighbor_sample)
        return result

    def coverage_weight(self, latitude, longitude, east_scale, north_scale, width=750.):
        lat,lon=np.broadcast_arrays(latitude,longitude); coverage=np.zeros(lat.shape)
        for path,info in self.rasters.items():
            mask=info.contains(lat,lon); fade=np.ones(lat.shape)
            distances=((lon-info.west)*east_scale,(info.east-lon)*east_scale,
                       (lat-info.south)*north_scale,(info.north-lat)*north_scale)
            for side,(neighbors,distance) in enumerate(zip(self._neighbors[path],distances)):
                connected=np.zeros(lat.shape,dtype=bool)
                for neighbor in neighbors:
                    connected |= ((lat>=neighbor.south-_TOLERANCE)&(lat<=neighbor.north+_TOLERANCE) if side<2
                                  else (lon>=neighbor.west-_TOLERANCE)&(lon<=neighbor.east+_TOLERANCE))
                fade=np.minimum(fade,np.where(connected,1.,np.clip(distance/width,0,1)))
            fade=fade*fade*(3-2*fade)
            coverage=np.maximum(coverage,np.where(mask,fade,0))
        return coverage

    def intersecting(self, south, west, north, east):
        return {path for path,info in self.rasters.items() if info.intersects(south,west,north,east)}

    def elevation_range(self, keys=None):
        low=high=0.
        with self.lock:
            for path in self.rasters:
                if keys is not None and path not in keys: continue
                tile_low,tile_high=self._tile(path).elevation_range()
                low=min(low,tile_low); high=max(high,tile_high)
        return low,high

    def close(self):
        with self.lock:
            for tile in self.cache.values(): tile.close()
            self.cache.clear(); self.resident_bytes=0
