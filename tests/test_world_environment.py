"""M20 layout, reset and force-based runway operation regressions."""
from collections import Counter
import unittest
from unittest.mock import patch
import glfw
import numpy as np
from engine.game import Game
from engine.input import Input
from game.world_environment import (WorldEnvironment, RUNWAY, CITY_SIZE, CITY_CENTER,
                                    BUILDING_PALETTE, GROUND_PALETTE, GROUND_ELEVATION,
                                    TERRAIN_BOUNDS, GroundSurface, downtown_distance)
from game.urban_layout import ROAD_WIDTHS, center_size, intersects, generate_layout
from game.fighter_ground import fighter_vertices, clearance
from game.flight_state import VehicleMode, Environment, FlightStatus
from game.atmospheric_physics import PARAMETERS, calculate_forces


class EnvironmentTests(unittest.TestCase):
    def test_determinism_and_bounded_mesh(self):
        a=WorldEnvironment();b=WorldEnvironment()
        np.testing.assert_array_equal(a.vertices,b.vertices)
        self.assertEqual(a.buildings,b.buildings)
        self.assertFalse(np.array_equal(a.vertices,WorldEnvironment(21).vertices))
        self.assertEqual(a.layout,b.layout)
        self.assertEqual(a.parcels,b.parcels)
        self.assertNotEqual(a.layout,WorldEnvironment(21).layout)
        self.assertGreater(len(a.buildings),200)
        self.assertLess(len(a.vertices),80000)
        self.assertTrue(np.isfinite(a.vertices).all())

    def test_blocks_setbacks_and_airfield_separation(self):
        e=WorldEnvironment()
        self.assertGreater(len(e.blocks),80)
        for block in e.blocks:
            x,z = block.center;w,d = block.size
            if block.kind=='ordinary':
                self.assertTrue(60<=w<=140 and 80<=d<=180)
            self.assertLessEqual(abs(x)+w/2,CITY_SIZE/2+1e-7)
            self.assertLessEqual(abs(z-CITY_CENTER[1])+d/2,CITY_SIZE/2+1e-7)
        for b in e.buildings:
            self.assertGreaterEqual(b.size[1],8)
            self.assertLessEqual(b.size[1],420 if b.landmark else 320)
            block = e.blocks[b.block_id]
            self.assertTrue(block.bounds[0]+3 <= b.footprint[0] < b.footprint[1] <= block.bounds[1]-3)
            self.assertTrue(block.bounds[2]+3 <= b.footprint[2] < b.footprint[3] <= block.bounds[3]-3)
        self.assertEqual((RUNWAY.length,RUNWAY.width),(2400,45))
        self.assertGreater(RUNWAY.center[2]-RUNWAY.length/2,CITY_CENTER[1]+CITY_SIZE/2)
        self.assertTrue(2000<=RUNWAY.center[2]-CITY_CENTER[1]<=3000)

    def test_palette_and_irregular_downtown_distribution(self):
        e = WorldEnvironment()
        self.assertTrue(all(b.color in BUILDING_PALETTE for b in e.buildings))
        self.assertTrue(all(tile.color in GROUND_PALETTE.values() for tile in e.ground_surfaces))
        regular = [b for b in e.buildings if not b.landmark]
        bands = ((0., .3), (.3, .58), (.58, .85), (.85, 1.08), (1.08, 2.))
        means = []
        for low, high in bands:
            heights = [b.size[1] for b in regular
                       if low <= downtown_distance(b.center[0], b.center[2]) < high]
            self.assertGreater(len(heights), 4)
            self.assertGreater(np.std(heights), 5.)
            means.append(float(np.mean(heights)))
        self.assertTrue(all(a > b for a, b in zip(means, means[1:])))
        self.assertGreater(means[0], 180.)
        self.assertLess(means[-1], 30.)
        self.assertGreater(means[0], means[-1]*8.)
        self.assertEqual([b.size[1] for b in e.buildings[-3:]], [420., 385., 350.])
        self.assertNotEqual(downtown_distance(300,-100), downtown_distance(300,-500))
        for b in e.buildings:
            self.assertGreater(b.center[2]-b.size[2]/2,
                               TERRAIN_BOUNDS[2])
            self.assertLess(b.center[2]+b.size[2]/2, RUNWAY.center[2]-RUNWAY.length/2)

    def test_planar_ground_partition_has_no_overlaps_or_missing_area(self):
        e = WorldEnvironment()
        bounds = np.array([tile.bounds for tile in e.ground_surfaces])
        self.assertTrue(all(tile.elevation == GROUND_ELEVATION for tile in e.ground_surfaces))
        x0, x1, z0, z1 = TERRAIN_BOUNDS
        self.assertTrue(np.all(bounds[:,0] >= x0) and np.all(bounds[:,1] <= x1))
        self.assertTrue(np.all(bounds[:,2] >= z0) and np.all(bounds[:,3] <= z1))
        area = np.sum((bounds[:,1]-bounds[:,0])*(bounds[:,3]-bounds[:,2]))
        self.assertAlmostEqual(area, (x1-x0)*(z1-z0), places=4)
        # Strict positive-area intersections are forbidden, including markings,
        # connector intersections and paving/desert boundaries at the airport.
        for index, (left, right, near, far) in enumerate(bounds):
            following = bounds[index+1:]
            overlap = ((following[:,0] < right-1e-9) & (following[:,1] > left+1e-9) &
                       (following[:,2] < far-1e-9) & (following[:,3] > near+1e-9))
            self.assertFalse(np.any(overlap), msg=f'overlapping ground tile {index}')
        ground = e.vertices[:e.ground_vertex_count].reshape(-1,3,6)
        np.testing.assert_array_equal(ground[:,:,1], GROUND_ELEVATION)
        normals = np.cross(ground[:,1,:3]-ground[:,0,:3], ground[:,2,:3]-ground[:,0,:3])
        self.assertTrue(np.all(normals[:,1] > 0))
        # No negative/zero thickness terrain boxes or downward-facing bases.
        triangles = e.vertices.reshape(-1,3,6)
        normal = np.cross(triangles[:,1,:3]-triangles[:,0,:3],triangles[:,2,:3]-triangles[:,0,:3])
        self.assertTrue(np.all(normal[:,1] >= 0))
        self.assertTrue(np.all(np.linalg.norm(normal,axis=1) > 0))

    def test_ground_mesh_is_watertight_at_tile_and_marking_seams(self):
        e = WorldEnvironment()
        triangles = e.vertices[:e.ground_vertex_count,:3].reshape(-1,3,3)
        edges = Counter()
        for triangle in triangles:
            for index, point in enumerate(triangle):
                edge = tuple(sorted((tuple(point), tuple(triangle[(index+1)%3]))))
                edges[edge] += 1
        x0, x1, z0, z1 = TERRAIN_BOUNDS
        for (a,b), count in edges.items():
            boundary = ((a[0] == b[0] and a[0] in (x0,x1)) or
                        (a[2] == b[2] and a[2] in (z0,z1)))
            self.assertEqual(count, 1 if boundary else 2, msg=f'open or duplicate edge {a}, {b}')

    def test_ground_rectangle_subtraction_and_pavement_materials(self):
        tile = GroundSurface((0,10,0,10),'desert')
        self.assertEqual(tile.subtract((20,30,20,30)), (tile,))
        self.assertEqual(tile.subtract((-1,11,-1,11)), ())
        pieces = tile.subtract((2,8,2,8))
        self.assertEqual(len(pieces),4)
        self.assertEqual(sum((p.bounds[1]-p.bounds[0])*(p.bounds[3]-p.bounds[2]) for p in pieces),64)
        e = WorldEnvironment()
        def kind(x,z):
            matches = [t.kind for t in e.ground_surfaces
                       if t.bounds[0] < x < t.bounds[1] and t.bounds[2] < z < t.bounds[3]]
            self.assertEqual(len(matches),1)
            return matches[0]
        self.assertEqual(kind(10,2501),'runway')
        self.assertEqual(kind(0,2471),'marking')
        self.assertEqual(kind(99,2501),'taxiway')
        self.assertEqual(kind(225,2999),'apron')
        self.assertEqual(kind(600,2001),'desert')
        block = next(b for b in e.blocks if b.kind=='ordinary' and b.district=='outer')
        x,z = block.center
        self.assertEqual(kind(block.bounds[0]+1.13,z+.137),'sidewalk')
        self.assertEqual(kind(x+.137,z+.137),'lot')
        road = next(r for r in e.roads if r.axis=='z' and r.kind=='major')
        (x,z),_ = center_size(road.bounds)
        self.assertEqual(kind(x+.137,z+.137),'street')

    def test_variable_blocks_road_classes_and_shared_layout(self):
        e = WorldEnvironment()
        widths = {round(b.size[0],2) for b in e.blocks if b.kind=='ordinary'}
        lengths = {round(b.size[1],2) for b in e.blocks if b.kind=='ordinary'}
        self.assertGreater(len(widths),6);self.assertGreater(len(lengths),6)
        self.assertTrue({'merged','subdivided','park_edge'} <= {b.kind for b in e.blocks})
        self.assertEqual({r.kind for r in e.roads},set(ROAD_WIDTHS))
        for road in e.roads:
            low,high = ROAD_WIDTHS[road.kind]
            self.assertTrue(low-1e-7<=road.width<=high+1e-7)
            _,size = center_size(road.bounds)
            self.assertAlmostEqual(size[1 if road.axis=='x' else 0],road.width)
            for block in e.blocks:self.assertFalse(intersects(block.bounds,road.bounds))
        self.assertTrue(any(r.kind=='major' and max(center_size(r.bounds)[1])>800 for r in e.roads))
        for index,a in enumerate(e.blocks):
            self.assertTrue(all(not intersects(a.bounds,b.bounds) for b in e.blocks[index+1:]))
        # Alternate seeds exercise the same bounded layout rules, not just seed 20.
        for seed in (0,7,21,99):
            layout = generate_layout(seed)
            self.assertEqual(len(layout.parks),5)
            self.assertTrue(all(not intersects(b.bounds,r.bounds) for b in layout.blocks for r in layout.roads))

    def test_parks_parcels_and_building_separation(self):
        e = WorldEnvironment()
        self.assertEqual(e.parks[0].size,(250.,450.))
        self.assertTrue(3<=len(e.parks)<=6)
        self.assertGreater(sum(p.vacant for p in e.parcels),20)
        self.assertTrue({'slab','setback','sections','warehouse','landmark'} <= {b.style for b in e.buildings})
        self.assertTrue(any(b.size[0]>65 or b.size[2]>80 for b in e.buildings))
        for park in e.parks:
            self.assertTrue(all(not intersects(park.bounds,r.bounds) for r in e.roads))
        for i,b in enumerate(e.buildings):
            self.assertTrue(all(not intersects(b.footprint,r.bounds) for r in e.roads))
            self.assertTrue(all(not intersects(b.footprint,p.bounds) for p in e.parks))
            self.assertTrue(all(not intersects(b.footprint,other.footprint) for other in e.buildings[i+1:]))
            self.assertLess(b.footprint[1],b.parcel[1]+1e-7)
            self.assertGreater(b.footprint[0],b.parcel[0]-1e-7)
            self.assertLess(b.footprint[3],b.parcel[3]+1e-7)
            self.assertGreater(b.footprint[2],b.parcel[2]-1e-7)
        self.assertTrue(all(not intersects(p.bounds,park.bounds) for p in e.parcels for park in e.parks))
        x,z = e.parks[0].center
        self.assertTrue(any(t.kind=='path' and t.bounds[0]<=x<=t.bounds[1] and t.bounds[2]<=z<=t.bounds[3]
                            for t in e.ground_surfaces))

    def test_district_density_and_vacancy(self):
        e = WorldEnvironment();density=[];height=[];vacancy=[]
        for district in ('downtown','midtown','outer'):
            blocks=[b for b in e.blocks if b.district==district and b.kind!='open_space']
            buildings=[b for b in e.buildings if e.blocks[b.block_id].district==district]
            parcels=[p for p in e.parcels if p.district==district]
            area=sum(b.size[0]*b.size[1] for b in blocks)
            density.append(len(buildings)/area)
            height.append(float(np.mean([b.size[1] for b in buildings])))
            vacancy.append(sum(p.vacant for p in parcels)/len(parcels))
        self.assertGreater(density[0],density[1]);self.assertGreater(density[1],density[2]*2)
        self.assertGreater(height[0],height[1]*2);self.assertGreater(height[1],height[2]*2)
        self.assertLess(vacancy[0],vacancy[1]);self.assertLess(vacancy[1],vacancy[2])
        counts=[sum(b.block_id==block.id for b in e.buildings) for block in e.blocks]
        self.assertGreater(len(set(counts)),5)

    def test_spawn_clearance_heading_and_rest(self):
        g=Game(enemy_count=0);v=g.player_vehicle
        self.assertEqual(tuple(v.position[[0,2]]),RUNWAY.spawn_xz)
        np.testing.assert_array_equal(v.forward,RUNWAY.forward)
        self.assertAlmostEqual((fighter_vertices()@v.orientation.T)[:,1].min()+v.position[1],0)
        for _ in range(120):g.flight_controller.update(1/120,g.input)
        self.assertIs(v.flight_state.status,FlightStatus.GROUNDED)
        np.testing.assert_array_equal(v.velocity,0)
        self.assertAlmostEqual(v.ground_contact.reaction_force[1],PARAMETERS.mass*PARAMETERS.gravity)
        self.assertEqual(v.ground_contact.touchdown_id,0)
        self.assertEqual(np.linalg.norm(v.aerodynamics.lift_force),0)

    def test_f4_from_all_modes_and_environments(self):
        g=Game(enemy_count=0);v=g.player_vehicle
        for mode in VehicleMode:
            for environment in Environment:
                v.flight_state.mode=mode;v.flight_state.environment=environment
                v.transformation.reset(mode);v.transformation.cycle();v.transformation.update(.1)
                v.velocity[:]=10;v.acceleration[:]=5;v.throttle=1
                v.pitch_rate=v.yaw_rate=v.roll_rate=1
                v.ground_contact.touchdown_id=9;v.ground_contact.hard_landing_time=4
                v.animation.state='LANDING';v.animation.weight=1;v.locomotion.blend=1
                g.input.thrust=g.input.pitch=1;g.input.fire_gun=True
                g.camera.mode='COCKPIT';g.camera.external_camera_mode='DOLLY'
                g.camera_input.scroll=3;g.camera_input.toggle_pressed=True
                g.input.runway_pressed=True;g.update(.1)
                self.assertIs(v.flight_state.mode,VehicleMode.FIGHTER)
                self.assertIs(v.flight_state.environment,Environment.ATMOSPHERE)
                self.assertIs(v.flight_state.status,FlightStatus.GROUNDED)
                np.testing.assert_array_equal(v.velocity,0);np.testing.assert_array_equal(v.acceleration,0)
                self.assertEqual(v.throttle,0);self.assertEqual(v.pitch_rate,0)
                self.assertFalse(v.transformation.active);self.assertEqual(v.transformation.queue,[])
                self.assertEqual(v.transformation._coordinate,0)
                self.assertEqual(v.animation.state,'IDLE');self.assertEqual(v.animation.weight,0)
                self.assertEqual(v.locomotion.blend,0);self.assertEqual(v.ground_contact.touchdown_id,0)
                self.assertEqual(g.input.pitch,0);self.assertFalse(g.input.fire_gun)
                self.assertEqual(g.camera_input.scroll,0);self.assertFalse(g.camera_input.toggle_pressed)
                self.assertEqual(g.camera.label,'CHASE');self.assertEqual(g.camera.mode,'CHASE')

    def test_f4_edge_binding(self):
        controls=Input()
        with patch('glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key==glfw.KEY_F4 else glfw.RELEASE):
            controls.poll(None);self.assertTrue(controls.runway_pressed)
            controls.poll(None);self.assertFalse(controls.runway_pressed)
            self.assertFalse(controls.space_pressed or controls.atmosphere_pressed or controls.toggle_sas)

    def test_ground_roll_rotation_and_aerodynamic_liftoff(self):
        g=Game(enemy_count=0);v=g.player_vehicle;g.input.thrust=1
        for _ in range(600):g.flight_controller.update(1/120,g.input)
        self.assertGreater(v.speed,75)
        self.assertIs(v.flight_state.status,FlightStatus.GROUNDED)
        self.assertEqual(v.velocity[1],0)
        for _ in range(360):
            g.input.pitch=.15 if v.forward[1]<.17 else 0
            g.flight_controller.update(1/120,g.input)
            if v.position[1]>clearance(v)+5:break
        self.assertIs(v.flight_state.status,FlightStatus.FLYING)
        self.assertGreater(v.position[1],clearance(v)+5)
        self.assertGreater(v.velocity[1],0)
        self.assertGreater(v.aerodynamics.lift_force[1],PARAMETERS.mass*PARAMETERS.gravity*.8)
        self.assertEqual(v.ground_contact.reaction_force[1],0)
        self.assertGreater(v.position[2],RUNWAY.center[2]-RUNWAY.length/2)

    def test_airborne_forces_and_space(self):
        g=Game(enemy_count=0);v=g.player_vehicle;g.flight_controller.reset_atmosphere()
        before=calculate_forces(v)
        g.flight_controller.fighter_ground.begin()
        np.testing.assert_array_equal(g.flight_controller.fighter_ground.acceleration(before,v.velocity,.01),
                                      before.total_force/PARAMETERS.mass)
        g.flight_controller.switch_space();v.throttle=0
        g.flight_controller.update(.1,g.input)
        self.assertEqual(v.velocity[1],0)
        self.assertGreater(g.camera.far,8000)
        self.assertEqual(g.camera.near,.1)

    def test_existing_ground_presets(self):
        g=Game(enemy_count=0)
        g.setup_vtol_test('ground');g.update(.1);self.assertTrue(g.player_vehicle.is_grounded)
        g.player_vehicle.throttle=1;g.update(.5);self.assertFalse(g.player_vehicle.is_grounded)
        g.setup_battledroid_test('ground');g.input.pitch=-1;g.update(1)
        self.assertTrue(g.player_vehicle.is_grounded)
        self.assertGreater(g.player_vehicle.speed,0)
