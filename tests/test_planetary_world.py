"""Synthetic planetary registry, source blending and streaming regressions."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
import tifffile
from game.geography import GeographicTile,GeographicPosition,GeographicFrame
from game.geographic_registry import GeographicRegistry
from game.planetary_elevation import ProceduralElevation
from game.terrain import Terrain,TerrainConfig,TerrainSelector
from engine.terrain_renderer import TerrainResources
from game.hgt import LocalProjection

class FakeMesh:
    def __init__(self,vertices,indices):self.count=len(indices);self.closed=False
    def draw(self):pass
    def close(self):self.closed=True

def write_tile(path,values,west=-112,north=35,step=.25):
    keys=(1,1,0,4,1024,0,1,2,1025,0,1,2,2048,0,1,4326,2054,0,1,9102)
    tifffile.imwrite(path,np.asarray(values,np.int16),metadata=None,extratags=[
        (33550,'d',3,(step,step,0),False),(33922,'d',6,(0,0,0,west,north,0),False),
        (34735,'H',len(keys),keys,False),(42113,'s',0,'-32767',False)])

class PlanetaryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.registry=self.root/'tiles';self.legacy=self.root/'hgt';self.legacy.mkdir()
        self.terrain=Terrain(TerrainConfig(directory=self.legacy,registry_directory=self.registry,elevation_offset=0))
    def tearDown(self):self.terrain.dataset.close();self.temp.cleanup()
    def config(self,**values):
        return TerrainConfig(directory=self.legacy,registry_directory=self.registry,elevation_offset=0,**values)

    def test_tile_flooring_hemispheres_wrapping_and_poles(self):
        for lat,lon,expected in ((34.8,-110.1,'N34W111'),(34,-112,'N34W112'),
            (35.5,-110.5,'N35W111'),(-.1,36.2,'S01E036'),(-1.01,-.01,'S02W001'),
            (0,180,'N00W180'),(0,540,'N00W180'),(90,20,'N89E020'),(-90,-180,'S90W180')):
            key=GeographicTile.at(lat,lon);self.assertEqual(key.identifier,expected)
            self.assertEqual(GeographicTile.parse(expected),key)
        for args in ((91,0),(float('nan'),0),(0,float('inf'))):
            with self.assertRaises(ValueError):GeographicTile.at(*args)
        for name in ('N90W111','N34E180','S00E000','bad'):
            with self.assertRaises(ValueError):GeographicTile.parse(name)

    def test_real_rasters_connect_across_antimeridian(self):
        write_tile(self.legacy/'east.tif',np.full((5,5),1000),west=179)
        write_tile(self.legacy/'west.tif',np.full((5,5),1000),west=-180)
        terrain=Terrain(self.config())
        try:
            for longitude in (179.99999,-180.,-179.99999,180.00001):
                self.assertAlmostEqual(float(terrain.dataset.sample(34.5,longitude)),1000)
                self.assertAlmostEqual(float(terrain.provider.sample(34.5,longitude)),1000)
        finally:terrain.dataset.close()

    def test_geographic_debug_hud_geometry(self):
        from engine.hud import HUD
        from engine.camera import Camera
        from game.player_vehicle import PlayerVehicle
        hud=HUD();hud.resize(1280,720)
        hud.terrain_debug=dict(lod='0-5',patches=64,triangles=100000,agl=100,
            agl_estimated=True,tiles=4,latitude=34.5,longitude=-111.5,tile='N34W112',
            source='PROCEDURAL',cpu_cached=32,cached=96,pending=9,
            lod_distribution={0:1,5:63},cpu_bytes=2000000,gpu_bytes=8000000,warnings=0)
        self.assertTrue(np.isfinite(hud.geometry(PlayerVehicle(),Camera(),debug=True)).all())

    def test_geographic_local_roundtrip_without_physics_mutation(self):
        frame=GeographicFrame(LocalProjection(),1937)
        local=np.asarray([130000.,1500.,-240000.]);copy=local.copy()
        geographic=frame.from_local(local)
        np.testing.assert_allclose(frame.to_local(geographic),local,atol=1e-8)
        np.testing.assert_array_equal(local,copy)
        proposed,shift=frame.rebase_plan(geographic)
        np.testing.assert_allclose(proposed.to_local(geographic)[[0,2]],0,atol=1e-8)
        np.testing.assert_allclose(shift,local,atol=1e-8)
        self.assertEqual(frame.projection.latitude,34.5)

    def test_registry_optional_manifests_and_legacy_priority(self):
        cell=self.registry/'N34W112';cell.mkdir(parents=True)
        write_tile(cell/'elevation.tif',np.full((5,5),2000))
        write_tile(self.legacy/'legacy.tif',np.full((5,5),1000))
        for name in ('city.json','airport.json','objects.json'):(cell/name).write_text('{}')
        (cell/'manifest.json').write_text(json.dumps(dict(version=1,elevation=['elevation.tif'],
            cities=['city.json'],airports=['airport.json'],objects=['objects.json'])))
        registry=GeographicRegistry(self.registry,self.legacy)
        assets=registry.assets_at(34.5,-111.5)
        self.assertEqual(len(assets.cities),1);self.assertEqual(len(assets.airports),1)
        self.assertEqual(len(assets.objects),1);self.assertEqual(len(registry.elevation_paths),2)
        terrain=Terrain(self.config());self.assertEqual(float(terrain.dataset.sample(34.5,-111.5)),2000)
        with patch.object(terrain.dataset, '_tile', side_effect=AssertionError('HUD must not decode rasters')):
            self.assertTrue(terrain.dataset.has_coverage(34.5, -111.5))
            self.assertFalse(terrain.dataset.has_coverage(34.5, -113.5))
        self.assertEqual(float(terrain.provider.sample(34.5,-111.5)),2000)
        terrain.dataset.close()
        self.assertTrue((self.legacy/'legacy.tif').exists())

    def test_manifest_and_corrupt_source_diagnostics(self):
        cell=self.registry/'N34W112';cell.mkdir(parents=True)
        (cell/'manifest.json').write_text('{bad')
        (self.legacy/'bad.tif').write_bytes(b'not a TIFF')
        terrain=Terrain(self.config())
        self.assertEqual(len(terrain.dataset.warnings),2)
        self.assertTrue(np.isfinite(terrain.height_at(20000,20000)))
        terrain.dataset.close()
        (cell/'manifest.json').write_text(json.dumps({'objects':['../../escape.json']}))
        self.assertTrue(GeographicRegistry(self.registry).warnings)

    def test_procedural_determinism_shared_edges_antimeridian_poles(self):
        a=ProceduralElevation(205);b=ProceduralElevation(205);c=ProceduralElevation(206)
        lat=np.linspace(34,35,101);lon=np.full(lat.shape,-111.)
        np.testing.assert_array_equal(a.sample(lat,lon),b.sample(lat,lon))
        self.assertFalse(np.array_equal(a.sample(lat,lon),c.sample(lat,lon)))
        np.testing.assert_allclose(a.sample(lat,180),a.sample(lat,-180),atol=1e-8)
        np.testing.assert_allclose(a.sample(90,np.linspace(-180,180,100)),a.sample(90,0),atol=1e-8)
        west=a.sample(lat,np.full(lat.shape,-111));east=b.sample(lat,np.full(lat.shape,-111))
        np.testing.assert_array_equal(west,east)
        self.assertLess(np.max(np.abs(np.diff(a.sample(34.5,np.linspace(-111.001,-110.999,1000))))),1)

    def test_real_to_procedural_edge_blending_is_continuous(self):
        write_tile(self.legacy/'real.tif',np.full((5,5),2000))
        terrain=Terrain(self.config())
        lon=np.linspace(-112.002,-111.98,2001);lat=np.full(lon.shape,34.5)
        values=terrain.provider.sample(lat,lon)
        self.assertLess(np.max(np.abs(np.diff(values))),5)
        self.assertAlmostEqual(float(terrain.provider.sample(34.5,-112)),float(terrain.provider.procedural.sample(34.5,-112)))
        self.assertEqual(float(terrain.provider.sample(34.5,-111.5)),2000)
        self.assertEqual(terrain.provider.source_at(34.5,-111.5),'SRTM')
        self.assertEqual(terrain.provider.source_at(34.5,-113),'PROCEDURAL')
        terrain.dataset.close()

    def test_void_feather_is_continuous_and_query_lod_independent(self):
        values=np.full((17,17),2000);values[8,8]=-32767
        write_tile(self.legacy/'void.tif',values,step=1/16)
        terrain=Terrain(self.config())
        longitude=np.linspace(-111.7,-111.3,2001);latitude=np.full(longitude.shape,34.5)
        result=terrain.provider.sample(latitude,longitude)
        self.assertLess(np.max(np.abs(np.diff(result))),20)
        self.assertAlmostEqual(float(terrain.provider.sample(34.5,-111.5)),float(terrain.provider.procedural.sample(34.5,-111.5)))
        before=terrain.height_at(20000,20000)
        TerrainSelector(terrain).select((20000,20000,20000))
        self.assertEqual(terrain.height_at(20000,20000),before)
        terrain.dataset.close()

    def test_source_disappearance_and_programming_errors(self):
        path=self.legacy/'real.tif';write_tile(path,np.full((5,5),2000))
        terrain=Terrain(self.config());terrain.dataset.close();path.unlink()
        self.assertTrue(np.isfinite(terrain.height_at(20000,20000)))
        self.assertTrue(terrain.dataset.warnings)
        with patch.object(terrain.provider.procedural,'sample',side_effect=RuntimeError('programming failure')):
            with self.assertRaisesRegex(RuntimeError,'programming failure'):terrain.height_at(20000,20000)
        terrain.dataset.close()

    def test_streaming_crosses_cells_with_bounded_caches_and_no_holes(self):
        resources=TerrainResources(self.terrain,FakeMesh);old=[];identities=[]
        for position in ((0,1500,0),(120000,1500,0),(240000,1500,-120000),(-160000,1500,80000)):
            roots=set(self.terrain.roots_near(position));resources.update(position,velocity=(300,0,-100))
            self.assertTrue(resources.active)
            for key in resources.active:
                a,b,c,d=self.terrain.bounds(key)
                self.assertTrue(np.isfinite(resources.cache[key].terrain_bytes))
                identities.append(self.terrain.geographic_identity(key))
            for key in resources.selector.select(position):
                a,b,c,d=self.terrain.bounds(key);cx,cz=(a+b)/2,(c+d)/2
                self.assertTrue(any(e<=cx<=f and g<=cz<=h for e,f,g,h in map(self.terrain.bounds,resources.active)))
            for _ in range(20):resources.update(position,velocity=(300,0,-100));time.sleep(.005)
            self.assertLessEqual(len(resources.active),self.terrain.config.budget)
            self.assertLessEqual(len(resources.cache),self.terrain.config.cache_size)
            self.assertLessEqual(len(resources.cpu_cache),self.terrain.config.cpu_cache_size)
            self.assertLessEqual(resources.cpu_bytes,self.terrain.config.cpu_cache_bytes)
            old.extend(resources.cache.values())
        self.assertGreater(len({identity[4] for identity in identities}),2)
        # The stable origin mesh persists for F4; other roots are evictable.
        self.assertIn((0,0,0),resources.cache)
        resources.close();self.assertTrue(all(mesh.closed for mesh in old))

    def test_delayed_worker_uses_procedural_root_without_raster_io(self):
        resources=TerrainResources(self.terrain,FakeMesh)
        original=self.terrain.mesh
        def delayed(*args,**kwargs):
            if not kwargs.get('procedural_only'):time.sleep(.04)
            return original(*args,**kwargs)
        with patch.object(self.terrain,'mesh',side_effect=delayed),patch.object(self.terrain.dataset,'sample',side_effect=RuntimeError('render thread IO')):
            # Background builds may fail deliberately, but the first update must
            # install immediate root coverage without reading any raster.
            resources.update((200000,1000,200000))
            self.assertTrue(resources.active);self.assertTrue(resources.fallbacks)
        resources.close()

    def test_prefetch_velocity_and_camera_relative_vertices(self):
        resources=TerrainResources(self.terrain,FakeMesh)
        resources.update((12000,1000,0),vehicle_position=(12000,1000,0),velocity=(2000,0,0))
        predicted=self.terrain.root_at(20000,0)
        self.assertTrue(predicted in resources.pending or predicted in resources.cache)
        key=self.terrain.root_at(1000000,0)
        vertices,_=self.terrain.mesh(key,procedural_only=True,cells=2,local=True,coarse_fallback=True)
        self.assertLessEqual(np.abs(vertices[:,0]).max(),self.terrain.config.side/2)
        resources.close()

    def test_city_airport_anchors_and_f4_remain_stable(self):
        from engine.game import Game
        from game.world_environment import RUNWAY
        game=Game(enemy_count=0,terrain_config=self.config());v=game.player_vehicle
        vertices=game.world.environment.vertices.copy();anchors=game.world.geographic_anchors.copy()
        v.position[:]=(240000,1000,-100000)
        geographic=v.geographic_position
        self.assertNotEqual(geographic.tile,anchors['airport'].tile)
        game.input.runway_pressed=True;game.update(.1)
        self.assertEqual(tuple(v.position[[0,2]]),RUNWAY.spawn_xz)
        np.testing.assert_array_equal(v.velocity,0)
        self.assertEqual(anchors,game.world.geographic_anchors)
        np.testing.assert_array_equal(vertices,game.world.environment.vertices)
        self.assertEqual(game.world.terrain.height_at(*RUNWAY.spawn_xz),0)
        game.world.terrain.dataset.close()

if __name__=='__main__':unittest.main()
