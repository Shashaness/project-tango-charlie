"""Continuous canonical poses, reversal and independent simulation regressions."""
import math
import unittest
from unittest.mock import patch,Mock
import numpy as np
from engine.game import Game
from engine.flight_controller import FlightController,DAMPING,MAX_STEP
from engine.hud import instrument_labels
from engine.model import ModelResources
from game.flight_state import VehicleMode as M,Environment,FlightStatus
from game.transformation import TransformationController,phase
from game.transformation_test import TransformationTest
from game.tc167_poses import POSES,DURATIONS,PHYSICS_SWITCH
from game.battledroid_physics import BATTLEDROID
from test_vehicle import commands

class TransformationTests(unittest.TestCase):
    def setUp(self):self.c=TransformationController()

    def matrices(self,c=None):return [n.local_matrix.copy() for n in (c or self.c).model.nodes]

    def test_poses_exist_and_names_are_real(self):
        self.assertEqual(set(POSES),set(M))
        for pose in POSES.values():
            for name in pose:self.assertIsNotNone(self.c.model.find_node(name))

    def test_fighter_exact_approved_rest_and_no_primitive_or_visibility_swap(self):
        model=self.c.model;primitives=model.primitives;nodes=tuple(model.nodes)
        self.c.reset(M.BATTLEDROID);self.c.reset(M.FIGHTER)
        self.assertIs(model.primitives,primitives);self.assertEqual(tuple(model.nodes),nodes)
        for node in model.nodes:np.testing.assert_array_equal(node.local_matrix,node.base_matrix)
        self.assertEqual(len(model.world_matrices()),32)

    def test_endpoints_exact_and_neutral_feet_match_contact_height(self):
        for mode in (M.VTOL,M.BATTLEDROID):
            self.c.reset(mode);target=self.matrices();self.c.reset(M.FIGHTER)
            self.c.request(mode);self.c.update(10)
            for actual,expected in zip(self.matrices(),target):np.testing.assert_array_equal(actual,expected)
        lo,hi=self.c.model.bounds();self.assertAlmostEqual(lo[1],-BATTLEDROID.foot_clearance,delta=.03)
        self.assertGreater(hi[1],3)

    def test_timing_clamps_and_smoothstep(self):
        self.assertEqual(phase(-1,.2,.8),0);self.assertEqual(phase(2,.2,.8),1)
        self.assertAlmostEqual(phase(.5,.2,.8),.5)
        self.assertLess(phase(.21,.2,.8),.01)
        with self.assertRaises(ValueError):phase(.5,.8,.2)

    def test_sequence_delays_later_parts(self):
        self.c.request(M.VTOL);self.c.update(.2*1.5)
        for name in ('LeftFoot','LeftWing'):
            np.testing.assert_array_equal(self.c.model.find_node(name).local_matrix,self.c.model.find_node(name).base_matrix)
        self.assertFalse(np.allclose(self.c.model.find_node('LeftUpperLeg').local_matrix,self.c.model.find_node('LeftUpperLeg').base_matrix))

    def test_norm_quaternions_positive_scales_and_hierarchy_every_sample(self):
        for edge in DURATIONS:
            for t in np.linspace(0,1,11):
                self.c.apply_curve(*edge,t);self.c.model.world_matrices()
                for value in self.c.current_pose.values():
                    self.assertAlmostEqual(np.linalg.norm(value.rotation),1)
                    self.assertTrue(all(s>0 for s in value.scale))
                for n in self.c.model.nodes:
                    if n.parent is not None:np.testing.assert_allclose(n.world_matrix,n.parent.world_matrix@n.local_matrix,atol=1e-12)

    def test_reverse_has_no_snap_and_retraces_same_curve(self):
        self.c.request(M.VTOL);self.c.update(.675);before=self.matrices()
        self.c.reverse()
        for a,b in zip(self.matrices(),before):np.testing.assert_array_equal(a,b)
        self.c.update(.15);reverse=self.matrices()
        reference=TransformationController();reference.apply_curve(M.FIGHTER,M.VTOL,.35)
        for a,b in zip(reverse,self.matrices(reference)):np.testing.assert_allclose(a,b,atol=1e-12)
        self.c.update(5);self.assertIs(self.c.configuration,M.FIGHTER)
        for n in self.c.model.nodes:np.testing.assert_array_equal(n.local_matrix,n.base_matrix)

    def test_battledroid_to_fighter_chains_through_vtol(self):
        self.c.reset(M.BATTLEDROID);self.c.cycle()
        self.assertIs(self.c.target,M.VTOL)
        self.c.update(1.9);self.assertIs(self.c.source,M.VTOL);self.assertIs(self.c.target,M.FIGHTER)
        expected=TransformationController();expected.reset(M.VTOL)
        for a,b in zip(self.matrices(),self.matrices(expected)):np.testing.assert_array_equal(a,b)
        self.c.update(1.5);self.assertFalse(self.c.active);self.assertEqual(self.c.progress,1)
        for n in self.c.model.nodes:np.testing.assert_array_equal(n.local_matrix,n.base_matrix)

    def test_physics_switch_only_at_sixty_percent_preserves_everything(self):
        g=Game(enemy_count=0);v=g.player_vehicle;c=v.transformation
        v.position[:]=(10,1000,-20);v.velocity[:]=(150,8,-90);v.rotate(yaw=.8,roll=2.1)
        v.throttle=.73;v.thrust_vector=.4
        before=(v.position.copy(),v.velocity.copy(),v.orientation.copy(),v.throttle,v.thrust_vector)
        target=g.combat.radar.current_target;inventory=g.combat.missile_fire_control.inventories.copy()
        stores=(v.defenses.flares,v.defenses.chaff,v.defenses.ecm_enabled)
        c.cycle();c.update(.899);self.assertIs(v.flight_state.mode,M.FIGHTER)
        c.update(.002);self.assertIs(v.flight_state.mode,M.VTOL)
        for a,b in zip((v.position,v.velocity,v.orientation,v.throttle,v.thrust_vector),before):np.testing.assert_array_equal(a,b)
        self.assertIs(g.combat.radar.current_target,target)
        self.assertEqual(g.combat.missile_fire_control.inventories,inventory)
        self.assertEqual((v.defenses.flares,v.defenses.chaff,v.defenses.ecm_enabled),stores)

    def test_reverse_physics_hysteresis(self):
        g=Game(enemy_count=0);c=g.player_vehicle.transformation
        c.cycle();c.update(1.2);self.assertIs(g.player_vehicle.flight_state.mode,M.VTOL)
        c.reverse();c.update(.1);self.assertIs(g.player_vehicle.flight_state.mode,M.VTOL)
        c.update(.51);self.assertIs(g.player_vehicle.flight_state.mode,M.FIGHTER)

    def test_dt_partition_independence_and_bad_dt(self):
        results=[]
        for fps in (30,60,144):
            c=TransformationController();c.request(M.BATTLEDROID)
            for _ in range(fps*4):c.update(1/fps)
            results.append(self.matrices(c));self.assertEqual(c.progress,1)
        for result in results[1:]:
            for a,b in zip(result,results[0]):np.testing.assert_array_equal(a,b)
        for dt in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.c.update(dt)

    def test_space_inertial_transformation_continues_physics(self):
        g=Game(enemy_count=0);g.flight_controller.switch_space();v=g.player_vehicle
        v.velocity[:]=(140,12,-90);v.rotate(yaw=math.pi/2,roll=math.pi)
        vv=v.velocity.copy();basis=v.orientation.copy();position=v.position.copy()
        g.input.toggle_configuration=True;g.update(0);g.input.toggle_configuration=False
        g.update(1.5)
        np.testing.assert_allclose(v.velocity,vv*math.exp(-DAMPING*1.5),atol=1e-9)
        np.testing.assert_allclose(v.orientation,basis,atol=1e-12)
        self.assertGreater(np.linalg.norm(v.position-position),100)
        self.assertIs(v.flight_state.mode,M.VTOL)
        self.assertEqual(v.throttle,0)

    def test_atmospheric_high_speed_transform_compares_existing_force_models(self):
        g=Game(enemy_count=0);g.flight_controller.reset_atmosphere();v=g.player_vehicle
        v.velocity[:]=(0,0,-180);v.throttle=0.;before=v.position.copy()
        g.input.toggle_configuration=True;g.update(0);g.input.toggle_configuration=False
        reference=Game(enemy_count=0);reference.flight_controller.reset_atmosphere()
        rv=reference.player_vehicle;rv.velocity[:]=(0,0,-180);rv.throttle=0.
        for i in range(180):
            g.update(MAX_STEP)
            if (i+1)*MAX_STEP>=.9-1e-10:rv.flight_state.mode=M.VTOL
            reference.flight_controller.update(MAX_STEP,reference.input)
        self.assertIs(v.flight_state.mode,M.VTOL)
        self.assertLess(v.speed,180);self.assertGreater(np.linalg.norm(v.position-before),100)
        np.testing.assert_allclose(v.velocity,rv.velocity,atol=.1)
        np.testing.assert_allclose(v.orientation,rv.orientation,atol=.001)
        self.assertEqual(v.throttle,0.)

    def test_battledroid_lands_and_walks_with_new_pose(self):
        g=Game(enemy_count=0);g.setup_battledroid_test('landing');v=g.player_vehicle
        v.throttle=0;v.transformation.reset(M.VTOL);v.flight_state.mode=M.VTOL
        v.transformation.request(M.BATTLEDROID)
        for _ in range(480):g.update(MAX_STEP)
        self.assertIs(v.flight_state.mode,M.BATTLEDROID);self.assertTrue(v.is_grounded)
        np.testing.assert_allclose(v.position[1],BATTLEDROID.foot_clearance,atol=1e-8)
        previous=v.position.copy();g.input.pitch=-1
        for _ in range(120):g.update(MAX_STEP)
        self.assertGreater(np.linalg.norm(v.position-previous),.1)
        self.assertEqual(len(v.transformation.model.world_matrices()),32)

    def test_viewer_full_cycle_rest_and_labels(self):
        with patch('builtins.print'):t=TransformationTest();t.update(3.75)
        self.assertTrue(t.controller.active);self.assertIn('50%',t.phase)
        with patch('builtins.print'):t.update(TransformationTest.PERIOD-3.75)
        self.assertFalse(t.controller.active);self.assertEqual(t.phase,'FIGHTER')
        for n in t.model.nodes:np.testing.assert_array_equal(n.local_matrix,n.base_matrix)

    def test_hud_during_transform_and_game_reset(self):
        g=Game(enemy_count=0);v=g.player_vehicle;c=v.transformation
        c.cycle();c.update(.5);labels=instrument_labels(v)
        self.assertTrue(any('FIGHTER > VTOL' in x for x in labels))
        self.assertTrue(any('XFORM' in x for x in labels))
        g.input.atmosphere_pressed=True;g.update(0)
        self.assertFalse(c.active);self.assertIs(c.configuration,M.FIGHTER)

    def test_same_gpu_resources_all_three_configurations(self):
        with patch('engine.mesh.Mesh') as mesh:
            resources=ModelResources(self.c.model);count=mesh.call_count
            for mode in M:self.c.reset(mode);resources.draw(self.c.model,Mock())
            self.assertEqual(mesh.call_count,count);resources.close();resources.close()
            self.assertEqual(mesh.return_value.close.call_count,count)

    def test_combat_keeps_firing_and_countermeasures_work_during_transform(self):
        g=Game(enemy_count=0);v=g.player_vehicle
        g.input.toggle_configuration=True;g.update(0);g.input.toggle_configuration=False
        g.input.fire_gun=True;g.update(.1)
        count=len(g.combat.projectiles);self.assertGreater(count,0)
        g.input.dispense_flare=True;g.input.dispense_chaff=True;g.input.toggle_ecm=True
        flares=v.defenses.flares;chaff=v.defenses.chaff
        g.update(.1)
        self.assertGreater(len(g.combat.projectiles),count)
        self.assertEqual(v.defenses.flares,flares-1);self.assertEqual(v.defenses.chaff,chaff-1)
        self.assertTrue(v.defenses.ecm_enabled);self.assertTrue(v.transformation.active)
        self.assertEqual(len(g.world.countermeasures),2)

    def test_ground_switch_has_no_transform_teleport(self):
        g=Game(enemy_count=0);g.setup_battledroid_test('ground');v=g.player_vehicle
        before=v.position.copy();vv=v.velocity.copy();o=v.orientation.copy()
        v.transformation.cycle();v.transformation.update(1.2)
        np.testing.assert_array_equal(v.position,before);np.testing.assert_array_equal(v.velocity,vv)
        np.testing.assert_array_equal(v.orientation,o)
        self.assertIs(v.flight_state.mode,M.VTOL);self.assertIs(v.flight_state.status,FlightStatus.GROUNDED)

    def test_renderer_draws_one_canonical_model_in_every_mode(self):
        from engine.renderer import Renderer
        g=Game(enemy_count=0);r=Renderer(g.world,g.player_vehicle,g.camera)
        r.environment_mesh=Mock();r.shader=Mock();r.grid=Mock();r.mesh=Mock();r.hud=Mock();r._draw_combat_scene=Mock()
        r.fighter_resources=Mock();r.fighter_model=g.player_vehicle.transformation.model
        r.vehicle_mesh=Mock();r.vtol_mesh=Mock();r.battledroid_mesh=Mock()
        with patch('engine.renderer.GL.glClear'), patch('engine.renderer.GL.glClearColor'):
            for mode in M:
                g.player_vehicle.flight_state.mode=mode;r.render()
        self.assertEqual(r.environment_mesh.draw.call_count,3)
        self.assertEqual(r.fighter_resources.draw.call_count,3)
        r.vehicle_mesh.draw.assert_not_called();r.vtol_mesh.draw.assert_not_called();r.battledroid_mesh.draw.assert_not_called()

    def test_double_reversal_and_chained_dt_overshoot(self):
        self.c.request(M.VTOL);self.c.update(.8);self.c.reverse();self.c.update(.1);self.c.reverse()
        self.c.update(5);self.assertIs(self.c.configuration,M.VTOL)
        self.c.reset(M.BATTLEDROID);self.c.request(M.FIGHTER);self.c.update(100)
        self.assertIs(self.c.configuration,M.FIGHTER);self.assertFalse(self.c.active)
        self.assertEqual(self.c.progress,1)

    def test_transform_hud_geometry_has_glyphs_and_finite_vertices(self):
        from engine.hud import HUD
        g=Game(enemy_count=0);hud=HUD();hud.resize(1280,720)
        g.player_vehicle.transformation.cycle();g.player_vehicle.transformation.update(.5)
        vertices=hud.geometry(g.player_vehicle,g.camera,g.flight_controller.fcc,combat=g.combat)
        self.assertGreater(len(vertices),0);self.assertTrue(np.isfinite(vertices).all())
