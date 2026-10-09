"""M20.4 compact footprint, grading and ground/terrain partition regressions."""
import tempfile
import unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
from game.terrain import Terrain,TerrainConfig
from game.terrain_compatibility import CompatibilityMask
from game.world_environment import WorldEnvironment,RUNWAY,TERRAIN_BOUNDS

class CompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.environment=WorldEnvironment()
        cls.temp=tempfile.TemporaryDirectory()
        np.full((1201,1201),1000,dtype='>i2').tofile(Path(cls.temp.name)/'N34W112.hgt')

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def setUp(self):
        self.terrain=Terrain(TerrainConfig(directory=Path(self.temp.name),elevation_offset=0),self.environment.protected_footprints)

    def tearDown(self): self.terrain.dataset.close()

    def test_actual_infrastructure_is_protected_and_flat(self):
        for rectangles in self.environment.infrastructure_footprints.values():
            points=[]
            for a,b,c,d in rectangles:
                points.extend(((a,c),(a,d),(b,c),(b,d),((a+b)/2,(c+d)/2)))
            points=np.asarray(points)
            np.testing.assert_array_equal(self.terrain.heights(points[:,0],points[:,1]),0)
        for building in self.environment.buildings:
            a,b,c,d=building.footprint
            self.assertEqual(self.terrain.height_at(a,c),0)
            self.assertEqual(self.terrain.height_at(b,d),0)
        x,_,z=RUNWAY.center
        for zz in np.linspace(z-RUNWAY.length/2,z+RUNWAY.length/2,101):
            self.assertEqual(self.terrain.height_at(x-RUNWAY.width/2,zz),0)
            self.assertEqual(self.terrain.height_at(x+RUNWAY.width/2,zz),0)
            np.testing.assert_array_equal(self.terrain.normal_at(x,zz),(0,1,0))

    def test_flat_region_is_compact_and_disconnected_infrastructure(self):
        footprints=self.environment.protected_footprints
        self.assertEqual(len(footprints),2)
        self.assertEqual(footprints,((-750.,750.,-1050.,450.),(-76.,396.,1194.,3806.)))
        area=sum((b-a)*(d-c) for a,b,c,d in footprints)
        old=(TERRAIN_BOUNDS[1]-TERRAIN_BOUNDS[0])*(TERRAIN_BOUNDS[3]-TERRAIN_BOUNDS[2])
        self.assertLess(area,old*.02)
        xs,zs=np.meshgrid(np.arange(-1500,1501,25),np.arange(-1800,4601,25))
        flat=np.count_nonzero(self.terrain.compatibility_weight(xs,zs)==0)*25**2
        self.assertLess(flat,old*.02)
        self.assertGreater(self.terrain.height_at(0,825),0) # gap is graded, not one big rectangle

    def test_configurable_clearance_and_width_restore_natural_elevations(self):
        for clearance,width in ((100.,300.),(200.,600.)):
            mask=CompatibilityMask(self.environment.protected_footprints,clearance,width)
            self.assertEqual(float(mask.weight(750+clearance,0)),0)
            self.assertEqual(float(mask.weight(750+clearance+width*1.3+1,0)),1)
        lat,lon=self.terrain.projection.to_geographic(2000,0)
        self.assertEqual(self.terrain.height_at(2000,0),float(self.terrain.dataset.sample(lat,lon)))
        self.assertEqual(self.terrain.height_at(9000,0),1000) # old flat region is gone
        self.assertLess(self.terrain.config.flat_clearance+self.terrain.config.blend_width*(1+self.terrain.config.transition_variation),1000)

    def test_determinism_irregularity_and_continuous_endpoints(self):
        mask=self.terrain.compatibility
        x=np.linspace(900,1600,2001); z=np.zeros_like(x)
        a=mask.weight(x,z)
        b=CompatibilityMask(mask.footprints).weight(x,z)
        np.testing.assert_array_equal(a,b)
        self.assertLess(np.max(np.abs(np.diff(a))),.01)
        self.assertEqual(a[0],0);self.assertEqual(a[-1],1)
        widths=mask.transition_width(np.zeros(100),np.linspace(-3000,3000,100))
        self.assertGreater(np.ptp(widths),20)
        self.assertGreaterEqual(widths.min(),450*.85)
        self.assertLessEqual(widths.max(),450*1.15)
        for edge in (900.,900.+float(mask.transition_width(1550,0))):
            self.assertLess(abs(self.terrain.height_at(edge+.01,0)-self.terrain.height_at(edge-.01,0)),.2)
        np.testing.assert_array_equal(self.terrain.normal_at(750,0),(0,1,0))

    def test_exact_ground_and_terrain_area_partition(self):
        vertices,indices=self.terrain.mesh((0,0,0))
        triangles=vertices[indices.reshape(-1,3),:3].astype(float)
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        top=normals[:,1]>0
        area=normals[top,1].sum()/2
        ground_area=sum((b-a)*(d-c) for a,b,c,d in self.environment.protected_footprints)
        self.assertAlmostEqual(area+ground_area,self.terrain.config.side**2,places=5)
        centers=triangles[top].mean(axis=1)
        self.assertFalse(np.any(self.terrain.compatibility.contains(centers[:,0],centers[:,2],strict=True)))
        # Edge insertion guarantees no top triangle straddles any protected edge.
        for a,b,c,d in self.environment.protected_footprints:
            bounds=np.stack((triangles[top,:,0].min(axis=1),triangles[top,:,0].max(axis=1),
                             triangles[top,:,2].min(axis=1),triangles[top,:,2].max(axis=1)),axis=1)
            self.assertFalse(np.any((bounds[:,0]<b)&(bounds[:,1]>a)&(bounds[:,2]<d)&(bounds[:,3]>c)))

    def test_transition_lod_edges_share_query_heights(self):
        left,_=self.terrain.mesh((5,15,15)); right,_=self.terrain.mesh((5,16,15))
        l=left[(left[:,0]==0)&(left[:,1]>self.terrain.elevation_bounds[0]-1)]
        r=right[(right[:,0]==0)&(right[:,1]>self.terrain.elevation_bounds[0]-1)]
        np.testing.assert_array_equal(l,r)
        vertices,_=self.terrain.mesh((4,7,7))
        top=vertices[vertices[:,1]>=0]
        np.testing.assert_allclose(top[:,1],self.terrain.heights(top[:,0],top[:,2]),atol=1e-4)

    def test_join_is_smooth_and_invalid_dimensions_rejected(self):
        # Near the midpoint of the city/airport gap, smooth-min removes the
        # sharp derivative reversal of a literal nearest-rectangle distance.
        mask=self.terrain.compatibility
        zs=np.linspace(819,825,601)
        derivative=np.gradient(mask.weight(np.zeros_like(zs),zs),zs)
        self.assertLess(np.max(np.abs(np.diff(derivative))),1e-5)
        for options in ({'flat_clearance':-1},{'transition_variation':1},
                        {'compatibility_join_width':-1},{'compatibility_cell_size':0}):
            with self.assertRaises(ValueError): replace(self.terrain.config,**options)
        with self.assertRaises(ValueError): CompatibilityMask(mask.footprints,blend_width=0)

    def test_empty_data_still_has_complete_static_fallback(self):
        from engine.terrain_renderer import TerrainResources
        class Mesh:
            def __init__(self,vertices,indices): self.count=len(indices)
            def close(self): pass
            def draw(self): pass
        terrain=Terrain(replace(self.terrain.config,directory=Path(self.temp.name)/'missing'))
        resources=TerrainResources(terrain,Mesh)
        resources.update((0,100,0))
        self.assertEqual(resources.active,[(0,0,0)])
        self.assertFalse(resources.pending)
        self.assertGreater(resources.stats['triangles'],0)
        resources.close()

if __name__=='__main__':unittest.main()
