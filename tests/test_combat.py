"""Intercept solutions, gun lifecycle, swept moving-target hits and combat HUD."""

from dataclasses import replace
import math
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from engine.camera import Camera
from engine.game import Game
from engine.hud import HUD, project_position
from engine.input import Input
from game.fire_control_radar import FireControlRadar
from game.fire_control_solution import solve_intercept
from game.flight_state import Environment,VehicleMode
from game.gun import GUN,Gun
from game.projectile import Projectile,segment_sphere_hit
from game.target import Target
from game.weapons import CombatSystem
from game.player_vehicle import PlayerVehicle


class InterceptTests(unittest.TestCase):
    def verify(self,shooter_velocity,target_position,target_velocity,expected_time=None):
        solution=solve_intercept((0,0,0),shooter_velocity,target_position,target_velocity,1000)
        self.assertIsNotNone(solution)
        if expected_time is not None:self.assertAlmostEqual(solution.intercept_time,expected_time)
        t=solution.intercept_time
        projectile_position=(np.array(shooter_velocity)+solution.required_direction*1000)*t
        np.testing.assert_allclose(projectile_position,solution.intercept_position,atol=1e-8)
        np.testing.assert_allclose(solution.intercept_position,np.array(target_position)+np.array(target_velocity)*t)
        self.assertAlmostEqual(np.linalg.norm(solution.required_direction),1)
        return solution

    def test_stationary_directly_ahead(self):
        solution=self.verify((0,0,0),(0,0,-1000),(0,0,0),1)
        np.testing.assert_array_equal(solution.required_direction,(0,0,-1))

    def test_crossing(self):self.verify((0,0,0),(0,0,-1000),(100,0,0),1000/math.sqrt(1000**2-100**2))
    def test_receding(self):self.verify((0,0,0),(0,0,-1000),(0,0,-75),1000/925)
    def test_shooter_lateral(self):
        solution=self.verify((300,0,0),(0,0,-1000),(0,0,0))
        self.assertLess(solution.required_direction[0],0)
    def test_shooter_approaching(self):self.verify((0,0,-250),(0,0,-1000),(0,0,0),.8)
    def test_near_linear(self):self.verify((0,0,0),(0,0,-1000),(0,0,1000),.5)
    def test_tangent_double_root(self):self.verify((0,0,0),(1000,0,0),(-1000,1000,0),1)
    def test_no_solution_and_invalid_inputs(self):
        for target_position,target_velocity in (((0,0,-1000),(0,0,-1500)),((0,0,-1000),(0,0,-1000)),((0,0,0),(0,0,0))):
            self.assertIsNone(solve_intercept((0,0,0),(0,0,0),target_position,target_velocity,1000))
        for speed in (0,-1,float('nan'),float('inf')):
            self.assertIsNone(solve_intercept((0,0,0),(0,0,0),(0,0,-1000),(0,0,0),speed))
        self.assertIsNone(solve_intercept((float('nan'),0,0),(0,0,0),(0,0,-1000),(0,0,0),1000))


class GunTests(unittest.TestCase):
    def test_muzzle_transform_and_inherited_velocity_both_modes(self):
        vehicle=PlayerVehicle(position=(10,20,30));vehicle.velocity[:]=(300,-20,-70)
        vehicle.rotate(pitch=.3,yaw=.8,roll=1)
        gun=Gun()
        for mode in (VehicleMode.FIGHTER,VehicleMode.VTOL):
            vehicle.flight_state.mode=mode
            projectile=gun.fire(vehicle)
            np.testing.assert_allclose(projectile.position,vehicle.position+vehicle.orientation@np.array(GUN.muzzle_offset))
            np.testing.assert_allclose(projectile.velocity,vehicle.velocity+vehicle.forward*1000)

    def test_automatic_cadence_and_birth_ages_across_dt(self):
        for fps in (20,30,60,144):
            gun=Gun();vehicle=PlayerVehicle();shots=[]
            for frame in range(fps*2):
                events=gun.update(1/fps,True,vehicle)
                shots.extend(frame/fps+offset for _,offset in events)
            self.assertEqual(len(shots),24)
            np.testing.assert_allclose(shots,np.arange(24)/12,atol=1e-10)
        gun=Gun();events=gun.update(1,True,PlayerVehicle())
        self.assertEqual(len(events),12)
        self.assertAlmostEqual(events[-1][1],11/12)

    def test_idle_does_not_bank_shots(self):
        gun=Gun();vehicle=PlayerVehicle()
        gun.update(100,False,vehicle)
        self.assertEqual(len(gun.update(.01,True,vehicle)),1)
        self.assertEqual(len(gun.update(.01,True,vehicle)),0)

    def test_constant_velocity_and_lifetime_endpoint(self):
        projectile=Projectile((0,0,0),(1000,50,0),lifetime=.1)
        duration=projectile.update(1)
        self.assertEqual(duration,.1)
        np.testing.assert_allclose(projectile.position,(100,5,0))
        self.assertFalse(projectile.alive)
        projectile.update(1);np.testing.assert_allclose(projectile.position,(100,5,0))

    def test_swept_collision_tunneling_and_nearest_target(self):
        self.assertIsNotNone(segment_sphere_hit((-100,0,0),(100,0,0),(0,0,0),1))
        self.assertIsNone(segment_sphere_hit((-100,2,0),(100,2,0),(0,0,0),1))
        self.assertEqual(segment_sphere_hit((0,0,0),(0,0,0),(0,0,0),1),0)
        combat=CombatSystem();vehicle=PlayerVehicle()
        near=Target(1,(100,0,0),radius=2);far=Target(2,(200,0,0),radius=2)
        combat.projectiles=[Projectile((0,0,0),(1000,0,0))]
        combat.update(.3,vehicle,[far,near])
        self.assertFalse(near.alive);self.assertTrue(far.alive)
        self.assertEqual(combat.projectiles,[])
        self.assertGreater(combat.hit_flash,0)

    def test_moving_target_relative_sweep(self):
        target=Target(1,(50,-50,0),(0,100,0),radius=1)
        combat=CombatSystem();combat.projectiles=[Projectile((0,0,0),(100,0,0),radius=.1)]
        combat.update(1,PlayerVehicle(),[target])
        self.assertFalse(target.alive)

    def test_expiry_limits_collision_interval(self):
        target=Target(1,(200,0,0),radius=2)
        combat=CombatSystem();combat.projectiles=[Projectile((0,0,0),(1000,0,0),lifetime=.1)]
        combat.update(1,PlayerVehicle(),[target])
        self.assertTrue(target.alive);self.assertEqual(combat.projectiles,[])

    def test_late_shots_only_travel_since_birth(self):
        combat=CombatSystem();combat.gun=Gun(replace(GUN,muzzle_offset=(0,0,0)))
        combat.update(.1,PlayerVehicle(position=(0,0,0)),[],True)
        self.assertEqual(len(combat.projectiles),2)
        self.assertAlmostEqual(combat.projectiles[0].position[2],-100)
        self.assertAlmostEqual(combat.projectiles[1].position[2],-1000*(.1-1/12))

    def test_end_to_end_crossing_with_moving_shooter(self):
        for parameters in (GUN,replace(GUN,muzzle_offset=(0,0,0))):
            vehicle=PlayerVehicle(position=(0,0,0));vehicle.velocity[:]=(150,0,0)
            target=Target(1,(0,0,-1000),(50,10,0),radius=3)
            combat=CombatSystem();combat.gun=Gun(parameters)
            # Test pilot aligns the fixed-forward body; the gun never articulates.
            # Recompute because rotating the vehicle also rotates the muzzle offset.
            for _ in range(5):
                combat.radar.update(vehicle,[target],combat.gun)
                direction=combat.radar.solution.required_direction
                vehicle.forward[:]=direction
                vehicle.right=np.cross(direction,(0,1,0));vehicle.right/=np.linalg.norm(vehicle.right)
                vehicle.up=np.cross(vehicle.right,vehicle.forward)
            combat.projectiles=[combat.gun.fire(vehicle)]
            for _ in range(120):
                vehicle.position+=vehicle.velocity/60
                combat.update(1/60,vehicle,[target])
                if not target.alive:break
            self.assertFalse(target.alive)

    def test_shared_atmosphere_space_ballistics(self):
        results=[]
        for environment in (Environment.SPACE,Environment.ATMOSPHERE):
            vehicle=PlayerVehicle();vehicle.flight_state.environment=environment
            projectile=Gun().fire(vehicle);projectile.update(.5);results.append(projectile.position)
        np.testing.assert_array_equal(*results)


class RadarHUDTests(unittest.TestCase):
    def test_closure_ranges_and_firing_cue(self):
        vehicle=PlayerVehicle(position=(0,0,0));vehicle.velocity[:]=(0,0,-100)
        target=Target(1,(0,0,-1000));radar=FireControlRadar();radar.update(vehicle,[target])
        self.assertAlmostEqual(radar.track.closure,100);self.assertTrue(radar.shoot)
        target.position[2]=-2500;radar.update(vehicle,[target])
        self.assertIsNotNone(radar.solution);self.assertFalse(radar.shoot)
        target.position[2]=-11000;radar.update(vehicle,[target])
        self.assertFalse(radar.track.in_range);self.assertIsNone(radar.solution)
        target.position[:]=(500,0,-1000);radar.update(vehicle,[target]);self.assertFalse(radar.shoot)

    def test_selection_skips_dead_and_auto_reselects(self):
        targets=[Target(index,(0,0,-1000-index)) for index in range(3)]
        radar=FireControlRadar();vehicle=PlayerVehicle();radar.update(vehicle,targets)
        self.assertIs(radar.current_target,targets[0])
        targets[1].alive=False;radar.select_next(targets)
        self.assertIs(radar.current_target,targets[2])
        targets[2].alive=False;radar.update(vehicle,targets)
        self.assertIs(radar.current_target,targets[0])
        targets[0].alive=False;radar.update(vehicle,targets)
        self.assertIsNone(radar.current_target);self.assertIsNone(radar.track)
        radar.select_next([]);self.assertIsNone(radar.current_target)

    def test_target_cycles_once_per_tab_press(self):
        controls=Input();held={glfw.KEY_TAB,glfw.KEY_SPACE}
        with patch('engine.input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None);self.assertTrue(controls.select_target and controls.fire_gun)
            controls.poll(None);self.assertFalse(controls.select_target);self.assertTrue(controls.fire_gun)
            held.clear();controls.poll(None);held.add(glfw.KEY_TAB);controls.poll(None)
            self.assertTrue(controls.select_target)

    def test_world_point_projector_rear_edge_and_combat_geometry(self):
        camera=Camera();camera.aspect=800/600
        point,edge=project_position((0,0,-1000),camera,800,600)
        np.testing.assert_allclose(point,(400,300));self.assertFalse(edge)
        for position in ((0,0,100),(5000,0,-1000)):
            point,edge=project_position(position,camera,800,600)
            self.assertTrue(edge);self.assertTrue(np.isfinite(point).all())
        self.assertIsNone(project_position((float('nan'),0,0),camera,800,600))
        game=Game();hud=HUD();hud.resize(800,600)
        for debug in (False,True):
            geometry=hud.geometry(game.player_vehicle,camera,game.flight_controller.fcc,debug,game.combat)
            self.assertTrue(np.isfinite(geometry).all());self.assertEqual(len(geometry)%3,0)

    def test_game_fires_destroys_and_auto_selects_without_touching_flight(self):
        game=Game();game.flight_controller.switch_space()
        game.player_vehicle.position[:]=(0,0,3);game.input.fire_gun=True
        initial_position=game.player_vehicle.position.copy()
        for _ in range(75):game.update(1/60)
        self.assertFalse(game.world.targets[0].alive)
        self.assertIs(game.combat.radar.current_target,game.world.targets[1])
        np.testing.assert_array_equal(game.player_vehicle.position,initial_position)
        np.testing.assert_array_equal(game.player_vehicle.velocity,0)
