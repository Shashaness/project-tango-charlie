"""Missile lock/inventory, finite flight, independent targeting and failure cases."""

from dataclasses import replace
import math
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from engine.game import Game
from engine.hud import HUD
from engine.input import Input
from game.fire_control_radar import FireControlRadar
from game.missile import MISSILE,Missile,MotorState
from game.missile_fire_control import MissileFireControl,LockState
from game.missile_guidance import proportional_navigation,STANDARD_GRAVITY
from game.missile_seeker import MissileSeeker,SeekerState
from game.player_vehicle import PlayerVehicle
from game.target import Target
from game.target_manager import TargetManager
from game.weapons import CombatSystem


def acquire(combat,vehicle,targets,seconds=1.5):
    combat.radar.update(vehicle,targets,combat.gun)
    for _ in range(round(seconds*120)):combat.update(1/120,vehicle,targets)


class TargetManagerTests(unittest.TestCase):
    def test_cycle_wrap_exclude_dead_indices_and_zero(self):
        targets=[Target(index,(0,0,-1000)) for index in (17,43,99)]
        manager=TargetManager();manager.refresh(targets)
        self.assertEqual((manager.index,manager.count),(1,3))
        manager.cycle(targets);self.assertEqual(manager.current_target.target_id,43)
        manager.cycle(targets);self.assertEqual(manager.index,3)
        manager.cycle(targets);self.assertEqual(manager.index,1)
        targets[0].alive=False;manager.refresh(targets)
        self.assertEqual((manager.index,manager.count),(1,2))
        self.assertEqual(manager.current_target.target_id,43)
        for target in targets:target.alive=False
        manager.refresh(targets);self.assertIsNone(manager.current_target)
        self.assertIsNone(manager.index);self.assertEqual(manager.count,0)

    def test_invalid_targets_are_not_available(self):
        target=Target(1,(0,0,-1000));target.position[0]=float('nan')
        manager=TargetManager();manager.refresh([target]);self.assertEqual(manager.count,0)

    def test_tab_and_m_are_edges_and_t_is_unassigned(self):
        controls=Input();held={glfw.KEY_TAB,glfw.KEY_M}
        with patch('engine.input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None);self.assertTrue(controls.select_target and controls.launch_missile)
            controls.poll(None);self.assertFalse(controls.select_target or controls.launch_missile)
            held.clear();controls.poll(None);held.add(glfw.KEY_T);controls.poll(None)
            self.assertFalse(controls.select_target)


class MissileControlTests(unittest.TestCase):
    def test_lock_requires_continuity_and_resets_on_geometry_or_selection(self):
        vehicle=PlayerVehicle(position=(0,0,0));targets=[Target(1,(0,0,-1000)),Target(2,(100,0,-1500))]
        radar=FireControlRadar();control=MissileFireControl();radar.update(vehicle,targets)
        control.update(1,vehicle,radar);self.assertIs(control.lock_state,LockState.ACQUIRING)
        targets[0].position[:]=(1000,0,0);radar.update(vehicle,targets);control.update(.1,vehicle,radar)
        self.assertEqual(control.lock_progress,0)
        targets[0].position[:]=(0,0,-1000);radar.update(vehicle,targets);control.update(1.5,vehicle,radar)
        self.assertIs(control.lock_state,LockState.LOCKED)
        radar.select_next(targets);radar.update(vehicle,targets);control.update(.1,vehicle,radar)
        self.assertIs(control.lock_state,LockState.ACQUIRING)

    def test_inventory_authorization_and_reset(self):
        vehicle=PlayerVehicle(position=(0,0,0));target=Target(1,(0,0,-1000))
        radar=FireControlRadar();control=MissileFireControl(replace(MISSILE,max_inventory=2))
        self.assertIsNone(control.launch(vehicle))
        radar.update(vehicle,[target]);control.update(1.5,vehicle,radar)
        self.assertIsNotNone(control.launch(vehicle));self.assertEqual(control.inventory,1)
        self.assertIsNotNone(control.launch(vehicle));self.assertEqual(control.inventory,0)
        self.assertIsNone(control.launch(vehicle));control.reset();self.assertEqual(control.inventory,2)
        self.assertIsNone(control.launch(vehicle))

    def test_envelope_separate_from_lock_and_recession_reduces_range(self):
        vehicle=PlayerVehicle(position=(0,0,0));target=Target(1,(0,0,-3000))
        radar=FireControlRadar();control=MissileFireControl()
        radar.update(vehicle,[target]);control.update(1.5,vehicle,radar)
        self.assertTrue(control.in_envelope);baseline=control.advised_range
        target.velocity[:]=(0,0,-400);radar.update(vehicle,[target]);control.update(0,vehicle,radar)
        self.assertFalse(control.in_envelope);self.assertLess(control.advised_range,baseline)
        self.assertIs(control.lock_state,LockState.LOCKED)
        self.assertIsNotNone(control.launch(vehicle))  # poor locked shots allowed, never guaranteed
        target.position[2]=-9000;radar.update(vehicle,[target]);control.update(.1,vehicle,radar)
        self.assertIs(control.lock_state,LockState.OUT_OF_RANGE)
        self.assertIsNone(control.launch(vehicle))

    def test_game_launch_once_per_edge_not_per_substep(self):
        game=Game();game.update(1.5)
        self.assertIs(game.combat.missile_fire_control.lock_state,LockState.LOCKED)
        game.input.launch_missile=True;game.update(.1)
        self.assertEqual(len(game.combat.missiles),1)
        self.assertEqual(game.combat.missile_fire_control.inventory,11)
        game.input.launch_missile=False;game.update(.1)
        self.assertEqual(len(game.combat.missiles),1)
        game.input.atmosphere_pressed=True;game.update(0)
        self.assertEqual(game.combat.missile_fire_control.inventory,12)


class MissilePhysicsTests(unittest.TestCase):
    def test_hardpoint_and_inherited_velocity(self):
        vehicle=PlayerVehicle(position=(10,20,30));vehicle.velocity[:]=(200,-5,-140)
        vehicle.rotate(pitch=.2,yaw=.7,roll=1)
        missile=Missile(vehicle,Target(1,(0,0,-1000)))
        np.testing.assert_allclose(missile.position,vehicle.position+vehicle.orientation@np.array(MISSILE.hardpoint))
        np.testing.assert_allclose(missile.velocity,vehicle.velocity+vehicle.forward*MISSILE.launch_speed)
        np.testing.assert_allclose(missile.orientation,vehicle.orientation)

    def test_separation_motor_burnout_and_coast(self):
        parameters=replace(MISSILE,burn_time=.5,navigation_constant=0)
        vehicle=PlayerVehicle(position=(0,0,0));missile=Missile(vehicle,Target(1,(0,0,-10000)),parameters)
        initial=missile.velocity.copy();missile.update(.1)
        self.assertIs(missile.motor_state,MotorState.SEPARATING)
        np.testing.assert_array_equal(missile.velocity,initial)
        # Use generous seeker range so only motor behavior is under test.
        missile.seeker.max_range=20000
        missile.update(.7)
        self.assertIs(missile.motor_state,MotorState.COASTING)
        self.assertAlmostEqual(missile.speed,80+.5*6000/100,places=7)
        velocity=missile.velocity.copy();position=missile.position.copy()
        missile.update(1)
        np.testing.assert_allclose(missile.velocity,velocity,atol=1e-9)
        np.testing.assert_allclose(missile.position,position+velocity,atol=1e-8)

    def test_pn_limits_normal_acceleration_and_turn_rate(self):
        acceleration=proportional_navigation(np.array((0,0,-100)),np.array((1000,0,500)),
                                            np.array((0,0,-1000)),4,35,90)
        self.assertLessEqual(np.linalg.norm(acceleration),35*STANDARD_GRAVITY+1e-9)
        self.assertGreater(np.linalg.norm(acceleration),300)
        self.assertAlmostEqual(np.dot(acceleration,(0,0,-1000)),0)
        slow=proportional_navigation(np.array((0,0,-100)),np.array((1000,0,500)),np.array((0,0,-10)),4,35,90)
        self.assertLessEqual(np.linalg.norm(slow),10*math.radians(90)+1e-9)

    def test_seeker_geometry_and_track_loss_timeout(self):
        seeker=MissileSeeker(1000,45,.8)
        self.assertTrue(seeker.update(np.array((0,0,-500)),np.array((0,0,-1)),True,.1))
        self.assertFalse(seeker.update(np.array((0,0,500)),np.array((0,0,-1)),True,.4))
        self.assertIs(seeker.state,SeekerState.SEARCHING)
        seeker.update(np.array((0,0,500)),np.array((0,0,-1)),True,.4)
        self.assertIs(seeker.state,SeekerState.LOST)
        seeker.update(np.array((0,0,-2000)),np.array((0,0,-1)),True,.1)
        self.assertFalse(seeker.state is SeekerState.TRACKING)

    def test_lifetime_and_target_destruction_remove_missile(self):
        target=Target(1,(0,0,-1000));missile=Missile(PlayerVehicle(),target,replace(MISSILE,max_lifetime=.1))
        missile.update(1);self.assertFalse(missile.alive);self.assertEqual(missile.miss_reason,'EXPIRED')
        missile=Missile(PlayerVehicle(),target);target.alive=False;missile.update(.1)
        self.assertFalse(missile.alive);self.assertEqual(missile.miss_reason,'TARGET DESTROYED')

    def test_swept_proximity_fuse_high_speed(self):
        vehicle=PlayerVehicle(position=(0,0,0));vehicle.rotate(yaw=-math.pi/2);vehicle.velocity[:]=(20000,0,0)
        target=Target(1,(100,0,0),radius=1)
        parameters=replace(MISSILE,hardpoint=(0,0,0),launch_speed=0,separation_time=0,
                           burn_time=0,navigation_constant=0,max_travel=100000)
        missile=Missile(vehicle,target,parameters)
        combat=CombatSystem();combat.missiles=[missile];combat.update(.01,vehicle,[target])
        self.assertFalse(target.alive);self.assertEqual(combat.missiles,[])
        self.assertGreater(combat.hit_flash,0)

    def test_good_crossing_intercept_and_poor_receding_miss(self):
        for position,velocity,should_hit in (((-500,100,-1500),(50,0,0),True),((0,0,-5000),(0,0,-700),False)):
            vehicle=PlayerVehicle(position=(0,0,0));target=Target(1,position,velocity)
            combat=CombatSystem();acquire(combat,vehicle,[target])
            combat.update(1/120,vehicle,[target],launch_missile=True)
            self.assertEqual(len(combat.missiles),1)
            missile=combat.missiles[0]
            for _ in range(2400):
                combat.update(1/120,vehicle,[target])
                self.assertTrue(np.isfinite(missile.position).all())
                self.assertLessEqual(np.linalg.norm(missile.guidance_acceleration),35*STANDARD_GRAVITY+1e-8)
                if not combat.missiles:break
            self.assertFalse(combat.missiles)
            self.assertEqual(not target.alive,should_hit)
            if not should_hit:self.assertIsNotNone(missile.miss_reason)

    def test_two_assigned_targets_survive_selection_changes_and_count_updates(self):
        vehicle=PlayerVehicle(position=(0,0,0));targets=[Target(index,position) for index,position in
                    ((17,(0,0,-1000)),(43,(100,0,-1500)),(99,(-100,0,-2000)),(105,(0,0,-2500)))]
        combat=CombatSystem();acquire(combat,vehicle,targets)
        combat.update(1/120,vehicle,targets,launch_missile=True);first=combat.missiles[0]
        combat.radar.select_next(targets);acquire(combat,vehicle,targets)
        combat.update(1/120,vehicle,targets,launch_missile=True)
        self.assertEqual(len(combat.missiles),2)
        second=combat.missiles[1]
        self.assertIs(first.target,targets[0]);self.assertIs(second.target,targets[1])
        combat.radar.select_next(targets)
        self.assertIs(first.target,targets[0]);self.assertIs(second.target,targets[1])
        for _ in range(1200):
            combat.update(1/120,vehicle,targets)
            manager=combat.radar.target_manager
            self.assertTrue(manager.index is None or manager.index<=manager.count)
            if not combat.missiles:break
        self.assertFalse(targets[0].alive);self.assertFalse(targets[1].alive)
        self.assertEqual(combat.radar.target_manager.count,2)
        self.assertIn(combat.radar.current_target,targets[2:])

    def test_crossing_intercept_is_consistent_across_frame_rates(self):
        results=[]
        for fps in (30,60,144):
            vehicle=PlayerVehicle(position=(0,0,0));target=Target(1,(-400,100,-1500),(50,0,0))
            combat=CombatSystem();combat.missiles=[Missile(vehicle,target)]
            missile=combat.missiles[0]
            for _ in range(fps*15):
                combat.update(1/fps,vehicle,[target])
                if not combat.missiles:break
            self.assertFalse(target.alive)
            results.append(missile.age)
        self.assertLess(max(results)-min(results),.08)

    def test_combat_hud_zero_targets_is_finite(self):
        game=Game()
        for target in game.world.targets:target.alive=False
        game.combat.radar.update(game.player_vehicle,game.world.targets,game.combat.gun)
        game.combat.missile_fire_control.update(0,game.player_vehicle,game.combat.radar)
        geometry=HUD().geometry(game.player_vehicle,game.camera,game.flight_controller.fcc,True,game.combat)
        self.assertTrue(np.isfinite(geometry).all())
