"""AI intent, shared physics/combat and target lifecycle regressions."""
import unittest
from dataclasses import replace
import math
import numpy as np
from game.ai_config import AI
from game.ai_pilot import AIState
from game.enemy_aircraft import EnemyAircraft, PlayerTarget
from game.player_vehicle import PlayerVehicle
from game.flight_state import Environment
from game.atmospheric_physics import reset_test_flight
from game.missile import Missile
from game.missile_fire_control import LockState
from game.target_manager import TargetManager
from game.weapons import CombatSystem
from game.world import World
from engine.game import Game

class AITests(unittest.TestCase):
    def setUp(self):
        self.player = PlayerVehicle((0,1000,-1000))
        self.proxy = PlayerTarget(self.player)
        self.enemy = EnemyAircraft(11,Environment.ATMOSPHERE,(0,1000,0))

    def command(self,dt=.01,incoming=(),firing=False):
        e=self.enemy
        e.combat.radar.update(e,[self.proxy],e.combat.gun)
        return e.pilot.update(dt,e,self.proxy,e.combat.radar,e.combat.missile_fire_control,incoming,firing)

    def test_decisions_do_not_mutate_kinematics_and_bound_commands(self):
        e=self.enemy
        p,v,o=e.position.copy(),e.velocity.copy(),e.orientation.copy()
        for location in ((1000,1500,0),(-1000,10,2000),(0,1000,-500)):
            self.player.position[:]=location
            c=self.command()
            for axis in ('pitch','yaw','roll','thrust'):
                self.assertLessEqual(abs(getattr(c,axis)),1)
            self.assertTrue(0<=c.throttle<=1)
        np.testing.assert_array_equal(p,e.position)
        np.testing.assert_array_equal(v,e.velocity)
        np.testing.assert_array_equal(o,e.orientation)

    def test_lead_accounts_for_relative_velocity(self):
        self.player.position[:]=(0,1000,-3000)
        self.player.velocity[:]=(80,0,0)
        d=self.enemy.pilot.pursuit(self.enemy,self.proxy)
        self.assertGreater(d[0],0)
        self.enemy.velocity[:]=(80,0,0)
        d=self.enemy.pilot.pursuit(self.enemy,self.proxy)
        self.assertAlmostEqual(d[0],0)

    def test_atmospheric_bank_turn(self):
        self.player.position[:]=(1000,1000,-1000)
        c=self.command()
        self.assertGreater(c.roll,.5)
        self.assertLess(abs(c.yaw),.1)

    def test_stall_recovery_unloads(self):
        self.enemy.rotate(pitch=math.radians(35))
        c=self.command()
        self.assertIs(self.enemy.pilot.state,AIState.RECOVER)
        self.assertLess(c.pitch,0)
        self.assertEqual(c.throttle,1)

    def test_terrain_overrides_pursuit(self):
        self.enemy.position[1]=100
        self.enemy.velocity[1]=-10
        self.player.position[:]=(1000,-1000,-1000)
        c=self.command()
        self.assertIs(self.enemy.pilot.state,AIState.RECOVER)
        self.assertTrue(self.enemy.pilot.terrain_danger)
        self.assertGreater(self.enemy.pilot.desired_direction[1],0)
        self.assertGreater(c.pitch,0)

    def test_missile_threat_evades(self):
        missile=Missile(self.player,self.enemy)
        self.command(incoming=[missile])
        self.assertIs(self.enemy.pilot.state,AIState.EVADE)
        self.assertTrue(self.enemy.pilot.missile_threat)

    def align_space(self):
        e=self.enemy
        e.controller.switch_space()
        e.forward[:]=(0,0,-1);e.right[:]=(1,0,0);e.up[:]=(0,1,0)
        e.velocity[:]=0
        self.player.position[:]=(0,1000,-1000)
        self.player.velocity[:]=0

    def test_gun_geometry_bursts_rate_and_inheritance(self):
        self.align_space()
        self.assertTrue(self.command().fire_gun)
        self.assertFalse(self.command(.7).fire_gun)
        self.player.position[0]=1000
        self.assertFalse(self.command().fire_gun)
        e=self.enemy;e.velocity[:]=(20,5,-30)
        events=e.combat.gun.update(1,True,e)
        self.assertEqual(len(events),12)
        np.testing.assert_allclose(events[0][0].velocity,e.velocity+e.forward*1000)

    def test_missile_requires_lock_envelope_inventory_and_cooldown(self):
        self.align_space();e=self.enemy
        self.assertFalse(self.command().fire_missile)
        e.combat.radar.update(e,[self.proxy],e.combat.gun)
        fc=e.combat.missile_fire_control
        fc.update(2,e,e.combat.radar)
        self.assertIs(fc.lock_state,LockState.LOCKED)
        c=self.command()
        self.assertTrue(c.fire_missile)
        m=fc.launch(e)
        self.assertIs(m.target,self.proxy)
        self.assertEqual(fc.inventory,AI.missile_inventory-1)
        e.pilot.missile_launched()
        self.assertFalse(self.command().fire_missile)
        fc.in_envelope=False;e.pilot._missile_wait=0
        self.assertFalse(self.command().fire_missile)
        fc.lock_state=LockState.ACQUIRING
        self.assertIsNone(fc.launch(e))

    def test_destruction_target_counts_and_kills(self):
        world=World(enemy_count=1)
        enemy=world.enemies[0]
        combat=CombatSystem();combat.hit_target(enemy,'GUN')
        manager=TargetManager();manager.refresh(world.targets)
        self.assertNotIn(enemy,manager.available)
        self.assertEqual(world.hostile_count,0)
        self.assertEqual(combat.kills,1)
        self.assertIs(enemy.pilot.state,AIState.DEAD)
        combat.hit_target(world.targets[0],'GUN')
        self.assertEqual(combat.kills,1)

    def test_player_hit_proxy_stays_alive_and_warning(self):
        game=Game(enemy_count=1)
        e=game.world.enemies[0]
        e.combat.hit_target(game.world.player_target,'GUN')
        self.assertEqual(game.world.player_hits,1)
        self.assertTrue(game.world.player_target.alive)
        e.combat.missiles.append(Missile(e,game.world.player_target))
        self.assertTrue(game.world.missile_warning)

    def test_space_rotation_does_not_rotate_velocity(self):
        self.align_space();e=self.enemy
        e.velocity[:]=(80,0,0);e.throttle=0
        e.pilot.parameters=replace(AI,guns_enabled=False,missiles_enabled=False)
        c=self.command()
        c.thrust=0
        e.controller.update(.01,c)
        self.assertGreater(e.velocity[0],79)
        self.assertAlmostEqual(e.velocity[1],0)
        self.assertAlmostEqual(e.velocity[2],0)

    def test_crash_and_dead_stop_flight(self):
        e=self.enemy;e.position[1]=.01;e.velocity[:]=(0,-200,0)
        e.fly(.01,self.proxy)
        self.assertFalse(e.alive)
        p=e.position.copy();e.fly(1,self.proxy)
        np.testing.assert_array_equal(e.position,p)

    def test_independent_enemy_state_inventory_and_environment_reset(self):
        g=Game(enemy_count=4)
        a,b=g.world.enemies[:2]
        a.combat.missile_fire_control.inventory=0
        self.assertEqual(b.combat.missile_fire_control.inventory,4)
        g.input.atmosphere_pressed=True;g.update(0)
        self.assertTrue(all(e.flight_state.environment is Environment.ATMOSPHERE for e in g.world.enemies))
        self.assertEqual(g.world.hostile_count,4)

    def test_recovery_and_terrain_use_real_forces(self):
        for terrain in (False, True):
            e=EnemyAircraft(11,Environment.ATMOSPHERE,(0,1000,0),
                            replace(AI,guns_enabled=False,missiles_enabled=False))
            if terrain:
                e.position[1]=140;e.velocity[1]=-25
            else:
                e.rotate(pitch=math.radians(35))
            for _ in range(600): e.fly(1/120,self.proxy)
            self.assertTrue(e.alive)
            self.assertLess(abs(math.degrees(e.aerodynamics.alpha)),10)
            if terrain:
                self.assertGreater(e.position[1],150)
                self.assertGreater(e.velocity[1],0)

    def test_enemy_projectile_sweeps_into_player_without_killing(self):
        self.align_space()
        world=World(enemy_count=0)
        combat=self.enemy.combat;combat.on_hit=world.player_hit
        combat.projectiles.append(combat.gun.fire(self.enemy))
        combat.update(1,self.enemy,[self.proxy])
        self.assertEqual(world.player_hits,1)
        self.assertTrue(self.proxy.alive)
        self.assertEqual(combat.projectiles,[])

    def test_selected_enemy_hud_geometry_is_finite(self):
        from engine.hud import HUD
        g=Game(enemy_count=1)
        g.combat.radar.current_target=g.world.enemies[0]
        g.update(.01)
        g.world.player_hit(g.world.player_target,'MISSILE')
        g.world.enemies[0].combat.missiles.append(Missile(g.world.enemies[0],g.world.player_target))
        hud=HUD();hud.resize(1280,720)
        vertices=hud.geometry(g.player_vehicle,g.camera,g.flight_controller.fcc,True,g.combat)
        self.assertTrue(np.isfinite(vertices).all())
        self.assertGreater(len(vertices),0)

if __name__=='__main__':unittest.main()
