"""Synthetic SRTM fixtures; no external dataset or GL context required."""
import tempfile
import time
import unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
from game.hgt import HgtTile, HgtDataset, LocalProjection, VOID
from game.terrain import Terrain, TerrainConfig, TerrainSelector
from game.world_environment import TERRAIN_BOUNDS, RUNWAY
from engine.terrain_renderer import TerrainResources

class FakeMesh:
    def __init__(self, vertices, indices):
        self.count = len(indices); self.closed = False
    def close(self): self.closed = True
    def draw(self): pass

class TerrainTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.directory = Path(self.temp.name)
        self.path = self.directory/'N34W112.hgt'
        # A continuous eastward slope, northward slope, with negative elevations.
        rows,cols = np.indices((1201,1201))
        self.samples = (cols-rows-200).astype('>i2'); self.samples.tofile(self.path)
        self.terrain = Terrain(TerrainConfig(directory=self.directory, elevation_offset=0))

    def tearDown(self):
        self.terrain.dataset.close(); self.temp.cleanup()

    def test_byte_order_dimensions_and_signed_orientation(self):
        tile = HgtTile(self.path)
        self.assertEqual(tile.size,1201)
        self.assertEqual(float(tile.sample(35,-112)),-200)
        self.assertEqual(float(tile.sample(34,-111)),-200)
        self.assertEqual(float(tile.sample(34,-112)),-1400)
        self.assertEqual(float(tile.sample(35,-111)),1000)
        self.assertTrue(np.isnan(tile.sample(33,-112)))
        tile.samples._mmap.close()
        path = self.directory/'S01E002.hgt'
        with path.open('wb') as file: file.truncate(3601*3601*2)
        tile = HgtTile(path); self.assertEqual(tile.size,3601)
        self.assertEqual(float(tile.sample(-.5,2.5)),0)
        tile.samples._mmap.close()

    def test_malformed_name_and_size(self):
        for name in ('N34W112bad.hgt','N91W112.hgt','N34W180.hgt'):
            path=self.directory/name; path.write_bytes(b'bad')
            with self.assertRaises(ValueError): HgtTile(path)
        broken=self.directory/'N33W112.hgt'; broken.write_bytes(b'bad')
        dataset=HgtDataset(self.directory)
        self.assertEqual(len(dataset.paths),1); self.assertEqual(len(dataset.warnings),4)
        dataset.close()

    def test_bilinear_and_void_renormalization(self):
        tile=HgtTile(self.path)
        self.assertAlmostEqual(float(tile.sample(35-100.5/1200,-112+300.5/1200)),0)
        tile.samples._mmap.close()
        self.samples[100:102,300:302] = [[VOID,100],[200,300]]
        self.samples.tofile(self.path); tile=HgtTile(self.path)
        self.assertAlmostEqual(float(tile.sample(35-100.5/1200,-112+300.5/1200)),200,places=6)
        self.assertTrue(np.isnan(tile.sample(35-100/1200,-112+300/1200)))
        tile.samples._mmap.close()

    def test_all_void_footprint_and_bounded_tile_cache(self):
        self.samples[100:102,300:302]=VOID
        self.samples.tofile(self.path)
        tile=HgtTile(self.path)
        self.assertTrue(np.isnan(tile.sample(35-100.5/1200,-112+300.5/1200)))
        tile.samples._mmap.close()
        for lon in range(113,118):
            with (self.directory/f'N34W{lon:03d}.hgt').open('wb') as file:
                file.truncate(1201*1201*2)
        dataset=HgtDataset(self.directory,capacity=2)
        for lon in range(113,118):
            self.assertEqual(float(dataset.sample(34.5,-lon+.5)),0)
            self.assertLessEqual(len(dataset.cache),2)
        dataset.close(); self.assertFalse(dataset.cache)

    def test_projection_roundtrip_and_longitude_scale(self):
        projection=LocalProjection(34.5,-111.5)
        x,z=projection.to_world(34.51,-111.49)
        self.assertGreater(x,900); self.assertLess(z,-1100)
        np.testing.assert_allclose(projection.to_geographic(x,z),(34.51,-111.49))
        self.assertLess(LocalProjection(60,0).east_scale,LocalProjection(0,0).east_scale*.51)
        for origin in ((90,0),(0,float('nan'))):
            with self.assertRaises(ValueError): LocalProjection(*origin)

    def test_source_queries_and_flat_compatibility(self):
        lat,lon=self.terrain.projection.to_geographic(13000,12000)
        expected=float(self.terrain.dataset.sample(lat,lon))
        self.assertAlmostEqual(self.terrain.height_at(13000,12000),expected)
        self.assertEqual(self.terrain.height_at(*RUNWAY.spawn_xz),0)
        self.assertEqual(self.terrain.height_at(0,-300),0)
        np.testing.assert_allclose(self.terrain.normal_at(0,0),(0,1,0))
        # Smooth transition on the east edge of the unchanged ground rectangle.
        self.assertEqual(self.terrain.height_at(9000,0),0)
        self.assertLess(abs(self.terrain.height_at(9001,0)),.01)
        normal=self.terrain.normal_at(13000,12000)
        self.assertAlmostEqual(np.linalg.norm(normal),1)
        self.assertLess(normal[0],0); self.assertGreater(normal[2],0)

    def test_deterministic_mesh_and_no_duplicate_ground(self):
        vertices,indices=self.terrain.mesh((0,0,0))
        vertices2,indices2=self.terrain.mesh((0,0,0))
        np.testing.assert_array_equal(vertices,vertices2); np.testing.assert_array_equal(indices,indices2)
        self.assertTrue(np.isfinite(vertices).all())
        self.assertEqual(vertices.dtype,np.float32)
        triangles=vertices[indices.reshape(-1,3),:3]
        # Only top triangles have nonzero XZ area. Skirts are vertical.
        cross=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        centers=triangles.mean(axis=1); top=np.abs(cross[:,1])>1
        a,b,c,d=TERRAIN_BOUNDS
        self.assertFalse(np.any(top & (centers[:,0]>a)&(centers[:,0]<b)&(centers[:,2]>c)&(centers[:,2]<d)))
        self.assertTrue(np.all(cross[top,1]>0))
        edge=vertices[(vertices[:,0]==9000)&(vertices[:,2]>=c)&(vertices[:,2]<=d)]
        self.assertTrue(len(edge)); self.assertTrue(np.all(edge[:,1]==0))
        np.testing.assert_allclose(edge[:,3:],np.tile([.57,.48,.36],(len(edge),1)),atol=1e-7)

    def test_patch_edges_and_skirts(self):
        left,_=self.terrain.mesh((2,0,3)); right,_=self.terrain.mesh((2,1,3))
        a,b,c,d=self.terrain.bounds((2,0,3))
        l=left[(left[:,0]==b)&(left[:,1]>self.terrain.elevation_bounds[0]-1)]
        r=right[(right[:,0]==b)&(right[:,1]>self.terrain.elevation_bounds[0]-1)]
        np.testing.assert_array_equal(l,r)
        self.assertLess(left[:,1].min(),self.terrain.elevation_bounds[0])

    def test_lod_budget_distance_hysteresis_and_frustum(self):
        selector=TerrainSelector(self.terrain)
        near=selector.select((13000,500,12000))
        far=TerrainSelector(self.terrain).select((13000,15000,12000))
        self.assertLessEqual(len(near),self.terrain.config.budget)
        self.assertGreater(max(k[0] for k in near),max(k[0] for k in far))
        self.assertEqual(near,selector.select((13001,500,12001)))
        # Force all boxes behind a rejecting clip plane.
        matrix=np.eye(4); matrix[3]=[0,0,0,-1e9]
        self.assertEqual(selector.select((0,0,0),matrix),[])
        self.assertTrue(all(not self.terrain.excluded(k) for k in near))
        self.assertNotEqual(TerrainSelector(self.terrain).select((0,0,0)),near)

    def test_hysteresis_has_distinct_enter_exit_thresholds(self):
        terrain = Terrain(replace(self.terrain.config,max_level=1))
        selector = TerrainSelector(terrain)
        self.assertGreater(len(selector.select((0,30000,0))),1)
        self.assertGreater(len(selector.select((0,44000,0))),1)
        self.assertEqual(TerrainSelector(terrain).select((0,44000,0)),[(0,0,0)])
        self.assertEqual(selector.select((0,51000,0)),[(0,0,0)])
        terrain.dataset.close()

    def test_no_data_and_partial_tile_fallback(self):
        empty=Terrain(replace(self.terrain.config,directory=self.directory/'missing'))
        self.assertFalse(empty.enabled); self.assertEqual(empty.height_at(13000,12000),0)
        np.testing.assert_array_equal(empty.normal_at(13000,12000),(0,1,0))
        self.assertEqual(self.terrain.height_at(100000,100000),0)
        x,z=self.terrain.projection.to_world(34.5,-112)
        self.assertEqual(self.terrain.height_at(x,z),0)
        empty.dataset.close()

    def test_async_cache_reuse_and_cleanup(self):
        resources=TerrainResources(self.terrain,FakeMesh)
        for _ in range(100):
            resources.update((13000,500,12000))
            self.assertLessEqual(len(resources.pending),8)
            self.assertLessEqual(len(resources.cache),self.terrain.config.cache_size)
            if not resources.pending and len(resources.active)>1: break
            time.sleep(.01)
        self.assertGreater(len(resources.active),1)
        self.assertLessEqual(len(resources.active),self.terrain.config.budget)
        active=list(resources.active); objects=[resources.cache[k] for k in active]
        resources.update((13000,500,12000))
        self.assertEqual(active,resources.active)
        self.assertEqual(objects,[resources.cache[k] for k in active])
        # No parent/child overlap in rendered coverage.
        for key in active:
            self.assertFalse(any(k[0]<key[0] and key[1]//2**(key[0]-k[0])==k[1] and key[2]//2**(key[0]-k[0])==k[2] for k in active))
        meshes=list(resources.cache.values()); resources.close(); resources.close()
        self.assertTrue(all(m.closed for m in meshes)); self.assertFalse(resources.cache)

    def test_cache_evicts_resources_when_camera_moves(self):
        self.terrain.config = replace(self.terrain.config,budget=16,cache_size=20)
        meshes=[]
        def factory(vertices,indices):
            mesh=FakeMesh(vertices,indices); meshes.append(mesh); return mesh
        resources=TerrainResources(self.terrain,factory)
        for position in ((13000,500,12000),(-13000,500,-12000),(13000,500,-12000)):
            for _ in range(100):
                resources.update(position)
                self.assertLessEqual(len(resources.cache),20)
                if not resources.pending: break
                time.sleep(.005)
            self.assertLessEqual(len(resources.active),16)
        self.assertTrue(any(mesh.closed for mesh in meshes))
        self.assertFalse(meshes[0].closed)
        resources.close(); self.assertTrue(all(mesh.closed for mesh in meshes))

    def test_f4_and_ground_support_with_source_data(self):
        from engine.game import Game
        from game.flight_state import FlightStatus, VehicleMode, Environment
        game = Game(enemy_count=0,terrain_config=self.terrain.config)
        vehicle = game.player_vehicle
        original_environment = game.world.environment.vertices.copy()
        for mode in VehicleMode:
            vehicle.flight_state.mode = mode
            vehicle.flight_state.environment = Environment.SPACE
            vehicle.position[:] = (13000,1000,12000)
            vehicle.velocity[:] = 20
            game.input.runway_pressed = True; game.update(.1)
            self.assertEqual(tuple(vehicle.position[[0,2]]),RUNWAY.spawn_xz)
            np.testing.assert_array_equal(vehicle.forward,RUNWAY.forward)
            self.assertIs(vehicle.flight_state.status,FlightStatus.GROUNDED)
            np.testing.assert_array_equal(vehicle.velocity,0)
        for _ in range(120): game.flight_controller.update(1/120,game.input)
        self.assertIs(vehicle.flight_state.status,FlightStatus.GROUNDED)
        np.testing.assert_array_equal(original_environment,game.world.environment.vertices)
        game.world.terrain.dataset.close()

    def test_configuration_bounds(self):
        for options in ({'budget':0},{'side':1000},{'max_level':20},{'cells':0},{'elevation_offset':float('nan')}):
            with self.assertRaises(ValueError): TerrainConfig(**options)

if __name__=='__main__': unittest.main()
