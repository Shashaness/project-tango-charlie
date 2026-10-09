"""Small synthetic GeoTIFFs; no downloaded data or GL context required."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
import numpy as np
import tifffile
from game.elevation import ElevationDataset, ElevationTile, inspect_raster
from game.hgt import HgtTile, LocalProjection
from game.terrain import Terrain, TerrainConfig


def write_fixture(path, samples, west=-112., north=35., x_step=.5, y_step=.5,
                  raster_type=2, nodata=-32767, crs=4326, model_type=2,
                  byteorder=None, compression=None, tie_i=0., tie_j=0., transform=None):
    keys=(1,1,0,4,1024,0,1,model_type,1025,0,1,raster_type,
          2048,0,1,crs,2054,0,1,9102)
    tags=[(34735,'H',len(keys),keys,False)]
    if transform is None:
        tags.extend([(33550,'d',3,(x_step,y_step,0),False),
                     (33922,'d',6,(tie_i,tie_j,0,west+tie_i*x_step,north-tie_j*y_step,0),False)])
    else:
        tags.append((34264,'d',16,np.asarray(transform).ravel(),False))
    if nodata is not None: tags.append((42113,'s',0,str(nodata),False))
    tifffile.imwrite(path,samples,metadata=None,extratags=tags,byteorder=byteorder,compression=compression)


class GeoTiffTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.directory=Path(self.temp.name)
        self.datasets=[]

    def tearDown(self):
        for dataset in self.datasets: dataset.close()
        self.temp.cleanup()

    def dataset(self, **options):
        result=ElevationDataset(self.directory,**options); self.datasets.append(result); return result

    def test_metadata_bounds_dimensions_and_no_filename_assumptions(self):
        path=self.directory/'arbitrary-name.TIFF'
        samples=np.arange(20,dtype=np.int16).reshape(4,5)-10
        write_fixture(path,samples,west=-111.3,north=34.7,x_step=.02,y_step=.03,tie_i=10,tie_j=20)
        info=inspect_raster(path)
        self.assertEqual(info.shape,(4,5)); self.assertEqual(info.dtype,np.dtype('int16'))
        np.testing.assert_allclose((info.west,info.east,info.south,info.north),(-111.3,-111.22,34.61,34.7))
        self.assertEqual(info.raster_type,2); self.assertEqual(info.nodata,-32767)
        dataset=self.dataset(); self.assertEqual(len(dataset.paths),1)
        self.assertEqual(float(dataset.sample(34.7,-111.3)),-10)
        self.assertEqual(float(dataset.sample(34.61,-111.22)),9)

    def test_endianness_orientation_negative_elevations_and_bilinear(self):
        samples=np.array([[-300,-200,-100],[-100,0,100],[100,200,300]],np.int16)
        write_fixture(self.directory/'east.tif',samples,byteorder='>')
        dataset=self.dataset()
        self.assertEqual(float(dataset.sample(35,-112)),-300)
        self.assertEqual(float(dataset.sample(34,-111)),300)
        self.assertAlmostEqual(float(dataset.sample(34.75,-111.75)),-150)
        self.assertTrue(np.isnan(dataset.sample(33,-111)))

    def test_declared_void_and_nan_renormalization(self):
        write_fixture(self.directory/'void.tif',np.array([[-32767,100],[200,300]],np.int16))
        dataset=self.dataset()
        self.assertAlmostEqual(float(dataset.sample(34.75,-111.75)),200)
        self.assertTrue(np.isnan(dataset.sample(35,-112)))
        # -32768 is a valid signed value if the declared GeoTIFF marker differs.
        write_fixture(self.directory/'negative.tiff',np.full((2,2),-32768,np.int16),west=-110)
        write_fixture(self.directory/'nan.tif',np.full((2,2),np.nan,np.float32),west=-108,nodata='nan')
        dataset=self.dataset()
        self.assertEqual(float(dataset.sample(34.75,-109.75)),-32768)
        self.assertTrue(np.isnan(dataset.sample(34.75,-107.75)))

    def test_point_tiles_share_samples_and_do_not_fade_internal_edge(self):
        write_fixture(self.directory/'west.tif',np.tile([0,10,20],(3,1)).astype(np.int16))
        write_fixture(self.directory/'east.tiff',np.tile([20,30,40],(3,1)).astype(np.int16),west=-111)
        dataset=self.dataset(capacity=1)
        values=dataset.sample(np.full(3,34.5),[-111-1e-7,-111,-111+1e-7])
        np.testing.assert_allclose(values,[20-2e-6,20,20+2e-6],atol=1e-8)
        self.assertEqual(float(dataset.coverage_weight(34.5,-111,90000,110000)),1)
        self.assertLessEqual(len(dataset.cache),1)

    def test_area_pixel_centers_and_cross_tile_bilinear_continuity(self):
        write_fixture(self.directory/'west.tif',np.array([[0,10],[0,10]],np.int16),
                      west=-112,north=35,raster_type=1)
        write_fixture(self.directory/'east.tif',np.array([[20,30],[20,30]],np.int16),
                      west=-111,north=35,raster_type=1)
        dataset=self.dataset(capacity=1)
        info=inspect_raster(self.directory/'west.tif')
        np.testing.assert_allclose(info.sample_bounds,(-111.75,-111.25,34.25,34.75))
        values=dataset.sample(np.full(5,34.5),[-111.25,-111.1,-111,-110.9,-110.75])
        np.testing.assert_allclose(values,[10,13,15,17,20],atol=1e-7)
        self.assertEqual(float(dataset.coverage_weight(34.5,-111,90000,110000)),1)
        self.assertEqual(float(dataset.sample(34.5,-112)),0)

    def test_area_four_tile_corner_and_cache_byte_bound(self):
        for name,west,north,value in (('nw',-112,35,0),('ne',-111,35,10),
                                      ('sw',-112,34,20),('se',-111,34,30)):
            write_fixture(self.directory/f'{name}.tif',np.full((2,2),value,np.int16),
                          west=west,north=north,raster_type=1)
        dataset=self.dataset(capacity=4,cache_bytes=8,max_tile_bytes=8)
        self.assertAlmostEqual(float(dataset.sample(34,-111)),15)
        self.assertEqual(dataset.resident_bytes,8)
        self.assertEqual(len(dataset.cache),1)
        self.assertEqual(float(dataset.coverage_weight(34,-111,90000,110000)),1)
        old=next(iter(dataset.cache.values())); dataset.sample(34.5,-111.5)
        if old is not next(iter(dataset.cache.values())): self.assertTrue(old.closed)
        dataset.close(); self.assertEqual(dataset.resident_bytes,0)

    def test_missing_tiles_and_void_footprint(self):
        write_fixture(self.directory/'void.tif',np.full((3,3),-32767,np.int16))
        dataset=self.dataset()
        self.assertTrue(np.isnan(dataset.sample(34.5,-111.5)))
        self.assertTrue(np.isnan(dataset.sample(40,-100)))
        self.assertEqual(float(dataset.coverage_weight(40,-100,90000,110000)),0)
        self.assertEqual(float(dataset.coverage_weight(34.5,-111,90000,110000)),0)
        terrain=Terrain(TerrainConfig(directory=self.directory))
        self.datasets.append(terrain.dataset)
        lat,lon=terrain.projection.to_geographic(13000,12000)
        self.assertAlmostEqual(terrain.height_at(13000,12000),float(terrain.provider.procedural.sample(lat,lon))-terrain.offset)

    def test_reject_unsupported_crs_and_malformed_georeferencing(self):
        for name,options in (('projected',{'crs':32612,'model_type':1}),
                             ('nad83',{'crs':4269}),('badscale',{'x_step':-1}),
                             ('rastertype',{'raster_type':3})):
            path=self.directory/f'{name}.tif'; write_fixture(path,np.zeros((2,2),np.int16),**options)
            with self.assertRaises(ValueError): inspect_raster(path)
        path=self.directory/'not-geographic.tif'; tifffile.imwrite(path,np.zeros((2,2),np.int16))
        with self.assertRaisesRegex(ValueError,'CRS'): inspect_raster(path)
        dataset=self.dataset(); self.assertFalse(dataset.paths); self.assertEqual(len(dataset.warnings),5)
        self.assertTrue(any('CRS' in warning for warning in dataset.warnings))

    def test_axis_aligned_transform_and_reject_rotation(self):
        matrix=np.diag([.5,-.5,1,1]); matrix[:2,3]=[-112,35]
        path=self.directory/'transform.tif'; write_fixture(path,np.zeros((3,3),np.int16),transform=matrix)
        info=inspect_raster(path); self.assertAlmostEqual(info.east,-111)
        matrix[0,1]=.1
        write_fixture(self.directory/'rotated.tif',np.zeros((3,3),np.int16),transform=matrix)
        with self.assertRaisesRegex(ValueError,'Rotated'): inspect_raster(self.directory/'rotated.tif')

    def test_deflate_disk_mapping_and_size_rejection(self):
        path=self.directory/'compressed.tif'
        write_fixture(path,np.arange(9,dtype=np.int16).reshape(3,3),compression='deflate')
        dataset=self.dataset(); self.assertEqual(float(dataset.sample(34.5,-111.5)),4)
        self.assertIsInstance(next(iter(dataset.cache.values())).samples,np.memmap)
        rejected=self.dataset(cache_bytes=16,max_tile_bytes=16)
        self.assertFalse(rejected.paths); self.assertIn('byte',rejected.warnings[0])

    def test_mixed_hgt_and_geotiff_sources(self):
        path=self.directory/'N34W112.hgt'
        with path.open('wb') as file: file.truncate(1201*1201*2)
        write_fixture(self.directory/'second.tif',np.full((3,3),100,np.int16),west=-111)
        dataset=self.dataset()
        self.assertEqual({info.format for info in dataset.rasters.values()},{'hgt','geotiff'})
        np.testing.assert_array_equal(dataset.sample([34.5,34.5],[-111.5,-110.5]),[0,100])
        tile=HgtTile(path); self.assertEqual(tile.size,1201); tile.samples._mmap.close()

    def test_terrain_mesh_seam_and_city_exclusion_with_geotiff(self):
        projection=LocalProjection(); _,seam=projection.to_geographic(12000,0)
        for name,west in (('west',seam-.5),('east',seam)):
            lat=35-np.arange(9)*.125; lon=west+np.arange(9)*.0625
            samples=(1000+(lat[:,None]-34.5)*100+(lon[None,:]+111.5)*200).astype(np.float32)
            write_fixture(self.directory/f'{name}.tif',samples,west=west,x_step=.0625,y_step=.125)
        terrain=Terrain(TerrainConfig(directory=self.directory)); self.datasets.append(terrain.dataset)
        left,li=terrain.mesh((5,27,28)); right,ri=terrain.mesh((5,28,28))
        l=left[(left[:,0]==12000)&(left[:,1]>terrain.elevation_bounds[0]-1)]
        r=right[(right[:,0]==12000)&(right[:,1]>terrain.elevation_bounds[0]-1)]
        self.assertTrue(len(l)); np.testing.assert_array_equal(l,r)
        self.assertEqual(terrain.height_at(0,3580),0); self.assertEqual(terrain.height_at(0,-300),0)
        first=terrain.mesh((0,0,0)); second=terrain.mesh((0,0,0))
        for a,b in zip(first,second): np.testing.assert_array_equal(a,b)
        for vertices,indices,is_left in ((left,li,True),(right,ri,False)):
            triangles=vertices[indices.reshape(-1,3),:3]
            top=np.abs(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])[:,1])>1
            centers=triangles[top].mean(axis=1)
            self.assertTrue(np.all(centers[:,0]<12000) if is_left else np.all(centers[:,0]>12000))

if __name__=='__main__': unittest.main()
