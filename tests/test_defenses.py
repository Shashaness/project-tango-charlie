"""Deterministic defensive sensors, seeker competition and expendable lifecycle."""
import math
import unittest
from unittest.mock import patch
from dataclasses import replace
import glfw
import numpy as np
from engine.game import Game
from engine.input import Input
from game.player_vehicle import PlayerVehicle
from game.enemy_aircraft import PlayerTarget, EnemyAircraft
from game.countermeasure import Countermeasure, CountermeasureType as Kind, CountermeasureDispenser
from game.defensive_avionics import DefensiveAvionics, ThreatState, geometry
from game.missile import Missile, MISSILE
from game.missile_seeker import SeekerType as Type, SeekerState, select_candidate, candidate_score
from game.missile_fire_control import MissileFireControl, LockState
from game.fire_control_radar import FireControlRadar
from game.flight_state import Environment
from game.ai_config import AI
from game.target import Target

class DefensiveTests(unittest.TestCase):
    def setUp(self):
        self.launcher=PlayerVehicle((0,0,0))
        self.aircraft=PlayerVehicle((0,0,-1000));self.aircraft.engine_throttle=.5
        self.aircraft.defenses=CountermeasureDispenser()
        self.target=PlayerTarget(self.aircraft)

    def missile(self,kind=Type.IR):
        m=Missile(self.launcher,self.target,seeker_type=kind)
        m.age=MISSILE.separation_time
        return m

    def decoy(self,kind):
        d=Countermeasure(self.aircraft,kind)
        d.position[:]=self.target.position+np.array((30,0,0))
        return d

    def test_inventory_decrements_nonnegative_and_empty_creates_nothing(self):
        dispenser=CountermeasureDispenser(1,1);active=[]
        for kind in (Kind.FLARE,Kind.CHAFF):
            self.assertIsNotNone(dispenser.dispense(self.aircraft,kind,active))
            self.assertIsNone(dispenser.dispense(self.aircraft,kind,active))
        self.assertEqual((dispenser.flares,dispenser.chaff),(0,0))
        self.assertEqual(len(active),2)
        dispenser.reset();self.assertEqual((dispenser.flares,dispenser.chaff),(1,1))
        self.assertFalse(dispenser.ecm_enabled)

    def test_both_packages_inherit_world_velocity(self):
        self.aircraft.velocity[:]=(40,-20,300)
        for kind in (Kind.FLARE,Kind.CHAFF):
            d=Countermeasure(self.aircraft,kind)
            np.testing.assert_allclose(d.velocity,self.aircraft.velocity-self.aircraft.forward*8-self.aircraft.up*4)

    def test_signatures_decay_expire_and_cloud_expands(self):
        flare=self.decoy(Kind.FLARE);chaff=self.decoy(Kind.CHAFF)
        heat=flare.heat_signature;radar=chaff.radar_signature;radius=chaff.dispersion_radius
        flare.update(2);chaff.update(2)
        self.assertLess(flare.heat_signature,heat)
        self.assertLess(chaff.radar_signature,radar)
        self.assertGreater(chaff.dispersion_radius,radius)
        flare.update(20);chaff.update(20)
        self.assertFalse(flare.alive or chaff.alive)
        self.assertEqual(flare.heat_signature,0)
        self.assertEqual(chaff.radar_signature,0)

    def test_ir_transfers_to_flare(self):
        m=self.missile();d=self.decoy(Kind.FLARE)
        self.assertIs(select_candidate(m,[d]),d)
        m.update(.01,countermeasures=[d])
        self.assertIs(m.current_seeker_target,d)
        self.assertIs(m.original_target,self.target)
        self.assertIs(m.target,self.target)
        self.assertGreater(abs(m.guidance_acceleration[0]),0)

    def test_radar_transfers_to_chaff(self):
        m=self.missile(Type.RADAR);d=self.decoy(Kind.CHAFF)
        m.update(.01,countermeasures=[d])
        self.assertIs(m.current_seeker_target,d)
        self.assertIs(m.original_target,self.target)

    def test_wrong_countermeasure_cannot_deceive(self):
        for seeker,wrong in ((Type.IR,Kind.CHAFF),(Type.RADAR,Kind.FLARE)):
            m=self.missile(seeker);d=self.decoy(wrong)
            self.assertIs(select_candidate(m,[d]),self.target)
            self.assertNotIn(d,m.seeker.candidate_scores)

    def test_cone_range_and_heat_gate(self):
        m=self.missile();d=self.decoy(Kind.FLARE)
        d.position[:]=(0,0,1000)
        self.assertEqual(candidate_score(m,d),0)
        d.position[:]=(0,0,-7000)
        self.assertEqual(candidate_score(m,d),0)
        self.aircraft.engine_throttle=0
        # Passive targets deliberately have no heat source.
        self.assertEqual(candidate_score(m,Target(1,(0,0,-100))),0)

    def test_expired_decoy_loses_track_without_omniscient_reacquisition(self):
        m=self.missile();d=self.decoy(Kind.FLARE)
        m.update(.01,countermeasures=[d]);d.update(10)
        self.aircraft.position[:]=(0,0,1000)
        m.update(1,countermeasures=[d])
        self.assertFalse(m.alive)
        self.assertIs(m.seeker_state,SeekerState.LOST)
        self.assertEqual(m.miss_reason,'TRACK LOST')

    def test_reacquires_only_original_aircraft_when_visible(self):
        m=self.missile();d=self.decoy(Kind.FLARE)
        m.update(.01,countermeasures=[d]);d.update(10)
        m.update(.01,countermeasures=[d])
        self.assertIs(m.current_seeker_target,self.target)

    def acquire(self,kind,ecm):
        radar=FireControlRadar();fc=MissileFireControl();fc.select(kind)
        self.aircraft.defenses.ecm_enabled=ecm
        radar.update(self.launcher,[self.target]);fc.update(1.5,self.launcher,radar)
        return fc

    def test_ecm_slows_radar_lock_but_allows_eventual_lock(self):
        off=self.acquire(Type.RADAR,False);on=self.acquire(Type.RADAR,True)
        self.assertIs(off.lock_state,LockState.LOCKED)
        self.assertIs(on.lock_state,LockState.ACQUIRING)
        self.assertAlmostEqual(on.lock_progress,.75)
        radar=FireControlRadar();radar.update(self.launcher,[self.target])
        on.update(1.5,self.launcher,radar)
        self.assertIs(on.lock_state,LockState.LOCKED)

    def test_ecm_does_not_change_ir_lock_or_score(self):
        m=self.missile();score=candidate_score(m,self.target)
        off=self.acquire(Type.IR,False);on=self.acquire(Type.IR,True)
        self.assertEqual(off.lock_progress,on.lock_progress)
        self.assertEqual(candidate_score(m,self.target),score)

    def test_ecm_reduces_radar_score_and_extends_detection(self):
        m=self.missile(Type.RADAR);score=candidate_score(m,self.target)
        self.aircraft.defenses.ecm_enabled=True
        self.assertLess(candidate_score(m,self.target),score)
        self.aircraft.position[:]=(0,0,-12000)
        radar=FireControlRadar();radar.update(self.launcher,[self.target])
        self.assertTrue(radar.track.in_range)
        self.aircraft.defenses.ecm_enabled=False;radar.update(self.launcher,[self.target])
        self.assertFalse(radar.track.in_range)

    def test_ir_lock_requires_heat_and_range(self):
        radar=FireControlRadar();fc=MissileFireControl();fc.select(Type.IR)
        passive=Target(1,(0,0,-1000));radar.update(self.launcher,[passive])
        fc.update(2,self.launcher,radar);self.assertIsNot(fc.lock_state,LockState.LOCKED)
        self.aircraft.position[2]=-7000;radar.update(self.launcher,[self.target])
        fc.update(2,self.launcher,radar);self.assertIs(fc.lock_state,LockState.OUT_OF_RANGE)

    def test_separate_inventory_and_weapon_switch_clear_lock(self):
        fc=self.acquire(Type.RADAR,False)
        self.assertEqual(fc.inventories,{Type.RADAR:6,Type.IR:6})
        self.assertIsNotNone(fc.launch(self.launcher));self.assertEqual(fc.inventories[Type.RADAR],5)
        fc.select(Type.IR);self.assertEqual(fc.lock_progress,0)
        self.assertIsNone(fc.launch(self.launcher))
        fc.inventories[Type.IR]=0
        fc.lock_state=LockState.LOCKED
        self.assertIsNone(fc.launch(self.launcher))
        fc.reset();self.assertEqual(fc.inventory,12)

    def test_warning_count_updates_on_death_and_lost_track(self):
        av=DefensiveAvionics();a=self.missile(Type.IR);b=self.missile(Type.RADAR)
        av.update(self.aircraft,self.target,missiles=[a,b]);self.assertEqual(len(av.incoming),2)
        a.alive=False;av.update(self.aircraft,self.target,missiles=[a,b]);self.assertEqual(len(av.incoming),1)
        b.seeker.state=SeekerState.LOST
        av.update(self.aircraft,self.target,missiles=[a,b]);self.assertEqual(len(av.incoming),0)

    def test_bearing_rotates_with_observer(self):
        self.launcher.position[:]=(100,0,100)
        bearing=geometry(self.aircraft,self.launcher)[0]
        self.assertGreater(bearing,math.pi/2)
        self.aircraft.rotate(yaw=math.pi/2)
        expected=math.atan2(np.dot(self.launcher.position-self.aircraft.position,self.aircraft.right),
                            np.dot(self.launcher.position-self.aircraft.position,self.aircraft.forward))
        self.assertAlmostEqual(geometry(self.aircraft,self.launcher)[0],expected)
        self.assertNotAlmostEqual(bearing,expected)

    def test_rwr_track_lock_and_missile(self):
        e=EnemyAircraft(11,Environment.SPACE,(0,0,0));e.combat.radar.update(e,[self.target])
        av=DefensiveAvionics();av.update(self.aircraft,self.target,[e])
        self.assertIs(av.rwr[0].state,ThreatState.TRACK)
        e.combat.missile_fire_control.update(1.5,e,e.combat.radar)
        av.update(self.aircraft,self.target,[e]);self.assertIs(av.rwr[0].state,ThreatState.LOCK)
        av.update(self.aircraft,self.target,[e],[self.missile()])
        self.assertIs(av.rwr[-1].state,ThreatState.MISSILE)

    def test_zero_and_negative_closure_have_no_tti(self):
        m=self.missile();m.velocity[:]=(0,0,0)
        self.assertIsNone(geometry(self.aircraft,m)[2])
        m.velocity[:]=(0,0,100)
        self.assertIsNone(geometry(self.aircraft,m)[2])

    def test_ai_dispense_is_finite_and_not_every_frame(self):
        e=EnemyAircraft(11,Environment.SPACE,(0,0,-400))
        m=Missile(self.launcher,e,seeker_type=Type.IR)
        e.avionics.update(e,e,missiles=[m]);active=[]
        for _ in range(120):e.defenses.defend(1/120,e,e.avionics,active)
        self.assertEqual(len(active),1)
        self.assertEqual(e.defenses.flares,9)
        for _ in range(5000):e.defenses.defend(1/120,e,e.avionics,active)
        self.assertEqual(e.defenses.flares,0)
        self.assertEqual(len(active),10)
        self.assertEqual(e.defenses.chaff,10)

    def test_keys_are_edges_and_existing_f_translation_survives(self):
        controls=Input();held={glfw.KEY_L,glfw.KEY_B,glfw.KEY_J,glfw.KEY_2,glfw.KEY_F}
        with patch('engine.input.glfw.get_key',side_effect=lambda w,k:glfw.PRESS if k in held else glfw.RELEASE):
            controls.poll(None)
            self.assertTrue(controls.dispense_flare and controls.dispense_chaff and controls.toggle_ecm and controls.select_ir)
            self.assertEqual(controls.lift,-1)
            controls.poll(None)
            self.assertFalse(controls.dispense_flare or controls.dispense_chaff or controls.toggle_ecm or controls.select_ir)

    def test_game_cleanup_and_reset(self):
        g=Game(enemy_count=0);g.input.dispense_flare=True;g.update(0)
        self.assertEqual(g.player_vehicle.defenses.flares,29)
        g.input.dispense_flare=False;g.update(6)
        self.assertEqual(g.world.countermeasures,[])
        g.input.atmosphere_pressed=True;g.update(0)
        self.assertEqual(g.player_vehicle.defenses.flares,30)

    def test_decoy_transfer_does_not_change_selected_target(self):
        g=Game(enemy_count=0);g.combat.radar.current_target=g.world.targets[0]
        selected=g.combat.radar.current_target;m=self.missile();d=self.decoy(Kind.FLARE)
        m.update(.01,countermeasures=[d])
        self.assertIs(g.combat.radar.current_target,selected)
        self.assertIs(m.original_target,self.target)

    def test_mixed_defense_preset_and_decoy_ownership(self):
        for atmosphere in (False,True):
            g=Game(enemy_count=0)
            if atmosphere:
                g.input.atmosphere_pressed=True;g.update(0);g.input.atmosphere_pressed=False
            g.setup_defense_test((Type.RADAR,Type.IR))
            self.assertEqual(len(g.player_vehicle.avionics.incoming),2)
            self.assertEqual([e.combat.missiles[0].seeker_type for e in g.world.enemies],[Type.RADAR,Type.IR])
            for e in g.world.enemies:
                self.assertIs(e.combat.missiles[0].original_target,g.world.player_target)
                self.assertEqual(e.combat.missile_fire_control.inventory,3)
            g.update(.1)
            self.assertEqual(len(g.player_vehicle.avionics.incoming),2)

    def test_flares_do_not_guarantee_survival_when_early_or_late(self):
        for distance,dispense_time in ((2500,0),(1000,6)):
            g=Game(enemy_count=0)
            g.input.atmosphere_pressed=True;g.update(0);g.input.atmosphere_pressed=False
            g.setup_defense_test((Type.IR,),distance)
            deployed=False
            for frame in range(360):
                if frame/30>=dispense_time and not deployed:
                    g.player_vehicle.defenses.dispense(g.player_vehicle,Kind.FLARE,g.world.countermeasures)
                    deployed=True
                g.update(1/30)
            self.assertGreater(g.world.player_hits,0)

    def test_defense_render_geometry_and_explicit_cleanup_without_gl_context(self):
        from unittest.mock import MagicMock
        from engine.renderer import Renderer
        g=Game(enemy_count=0);g.setup_defense_test((Type.RADAR,Type.IR))
        g.player_vehicle.defenses.dispense(g.player_vehicle,Kind.FLARE,g.world.countermeasures)
        g.player_vehicle.defenses.dispense(g.player_vehicle,Kind.CHAFF,g.world.countermeasures)
        g.update(.3)
        r=Renderer(g.world,g.player_vehicle,g.camera,g.flight_controller.fcc,g.combat)
        r.show_axes=True;r.shader=MagicMock()
        meshes=[]
        for name in ('enemy_mesh','target_mesh','missile_mesh','combat_lines'):
            mesh=MagicMock();mesh.vbo=1;setattr(r,name,mesh);meshes.append(mesh)
        with patch('engine.renderer.GL.glBindBuffer'),patch('engine.renderer.GL.glBufferData') as upload,patch('engine.renderer.GL.glUseProgram'):
            r._draw_combat_scene()
            self.assertTrue(np.isfinite(upload.call_args.args[2]).all())
            r.close();r.close()
        for mesh in meshes:mesh.close.assert_called_once()

    def test_ir_and_radar_geometry_hud_is_finite(self):
        from engine.hud import HUD
        g=Game(enemy_count=0);g.setup_defense_test((Type.RADAR,Type.IR))
        hud=HUD();hud.resize(1280,720)
        for kind in (Type.RADAR,Type.IR):
            g.combat.missile_fire_control.select(kind)
            geometry=hud.geometry(g.player_vehicle,g.camera,g.flight_controller.fcc,True,g.combat)
            self.assertTrue(np.isfinite(geometry).all())

if __name__=='__main__':unittest.main()
