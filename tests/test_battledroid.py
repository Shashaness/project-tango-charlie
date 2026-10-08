"""Battledroid-only physics/contact regressions alongside the established flight core."""
import math
import unittest
from unittest.mock import patch, MagicMock
import numpy as np
import glfw
from engine.game import Game
from engine.flight_controller import FlightController
from engine.input import Input
from engine.hud import HUD, instrument_labels
from game.player_vehicle import PlayerVehicle
from game.flight_state import VehicleMode, Environment, FlightStatus
from game.battledroid_physics import BATTLEDROID, forces, contact_clearance, walking_acceleration
from game.atmospheric_physics import PARAMETERS, calculate_forces
from game.countermeasure import Countermeasure, CountermeasureType
from game.gun import Gun
from game.missile import Missile
from game.target import Target
from test_vehicle import commands


def battledroid(environment=Environment.ATMOSPHERE, height=50.):
    v=PlayerVehicle((0,height,0));v.flight_state.environment=environment
    v.toggle_configuration();v.toggle_configuration()
    return v,FlightController(v)

class BattledroidTests(unittest.TestCase):
    def test_all_transformations_preserve_kinematics_and_inventories(self):
        g=Game(enemy_count=0);v=g.player_vehicle
        v.position[:]=(30,80,-50);v.velocity[:]=(120,-5,-40)
        v.rotate(pitch=.3,yaw=.8,roll=2.4)
        v.pitch_rate=.4;v.yaw_rate=.2;v.roll_rate=-.7
        p,vv,o=v.position.copy(),v.velocity.copy(),v.orientation.copy()
        radar=g.combat.radar.current_target;fc=g.combat.missile_fire_control
        inventory=fc.inventories.copy();stores=(v.defenses.flares,v.defenses.chaff)
        for mode in (VehicleMode.VTOL,VehicleMode.BATTLEDROID,VehicleMode.FIGHTER):
            v.toggle_configuration();self.assertIs(v.flight_state.mode,mode)
            np.testing.assert_array_equal(v.position,p);np.testing.assert_array_equal(v.velocity,vv)
            np.testing.assert_array_equal(v.orientation,o)
            self.assertEqual((v.pitch_rate,v.yaw_rate,v.roll_rate),(.4,.2,-.7))
            self.assertIs(g.combat.radar.current_target,radar)
            self.assertEqual(fc.inventories,inventory)
            self.assertEqual((v.defenses.flares,v.defenses.chaff),stores)

    def test_inverted_space_sas_has_no_world_up_correction(self):
        v,c=battledroid(Environment.SPACE);v.rotate(roll=math.pi)
        basis=v.orientation.copy();v.velocity[:]=(40,10,-50)
        c.update(.5,commands())
        np.testing.assert_allclose(v.orientation,basis,atol=1e-12)
        np.testing.assert_allclose(v.velocity,np.array((40,10,-50))*math.exp(-BATTLEDROID.space_damping*.5))
        self.assertFalse(v.is_grounded)
        np.testing.assert_array_equal(v.aerodynamics.gravity_force,0)

    def test_space_local_thrust_changes_velocity_not_facing_momentum(self):
        v,c=battledroid(Environment.SPACE);v.rotate(yaw=math.pi/2);v.velocity[:]=(0,0,-40)
        right=v.right.copy();c.update(.1,commands(strafe=1,lift=1))
        self.assertGreater(np.dot(v.velocity-np.array((0,0,-40))*math.exp(-BATTLEDROID.space_damping*.1),right),0)
        self.assertLess(v.velocity[2],-38)

    def test_gravity_zero_lift_drag_and_bounded_thrust(self):
        v,c=battledroid();v.velocity[:]=(0,0,-200)
        f=forces(v,commands());self.assertEqual(f.lift_coefficient,0)
        np.testing.assert_array_equal(calculate_forces(v).lift_force,0)
        np.testing.assert_array_equal(f.lift_force,0)
        self.assertLess(f.gravity_force[1],0)
        self.assertGreater(np.linalg.norm(f.drag_force),100000)
        v.throttle=1;f=forces(v,commands(lift=1,strafe=1))
        self.assertGreater((f.thrust_force+f.maneuver_force)[1],PARAMETERS.mass*PARAMETERS.gravity)
        self.assertLessEqual(np.linalg.norm(f.thrust_force+f.maneuver_force),PARAMETERS.mass*PARAMETERS.gravity*BATTLEDROID.thrust_to_weight+1e-8)
        v.throttle=0;c.update(.1,commands());self.assertLess(v.velocity[1],0)

    def test_high_speed_transform_keeps_momentum_then_drag_slows_it(self):
        v=PlayerVehicle((0,1000,0));v.flight_state.environment=Environment.ATMOSPHERE
        v.velocity[:]=(0,0,-200);before=v.velocity.copy()
        v.toggle_configuration();v.toggle_configuration()
        np.testing.assert_array_equal(v.velocity,before)
        FlightController(v).update(.1,commands())
        self.assertLess(v.speed,200);self.assertGreater(v.speed,170)

    def test_hover_fcc_never_writes_position_velocity(self):
        v,c=battledroid();v.velocity[:]=(3,-4,5)
        c.fcc.toggle_hover(v);p=v.position.copy();vv=v.velocity.copy()
        output=c.fcc.commands(commands(),v,.01,{axis:1. for axis in ('pitch','yaw','roll')})
        self.assertIsNotNone(output.throttle_override)
        np.testing.assert_array_equal(v.position,p);np.testing.assert_array_equal(v.velocity,vv)
        for _ in range(240):c.update(1/120,commands())
        self.assertGreater(v.velocity[1],-1)
        self.assertFalse(v.is_grounded)

    def test_manual_hover_uses_force_equilibrium(self):
        v,c=battledroid();v.throttle=1/BATTLEDROID.thrust_to_weight
        c.update(2,commands())
        np.testing.assert_allclose(v.position,(0,50,0),atol=1e-9)
        np.testing.assert_allclose(v.velocity,0,atol=1e-9)

    def test_contact_removes_only_normal_velocity_and_no_bounce(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance+.001);v.velocity[:]=(30,-2,-4)
        c.update(.001,commands())
        self.assertTrue(v.is_grounded)
        self.assertGreaterEqual(v.position[1],contact_clearance(v))
        self.assertEqual(v.velocity[1],0)
        self.assertGreater(v.velocity[0],29)
        self.assertLess(v.velocity[2],-3.9)
        self.assertGreater(v.ground_contact.depth,0)

    def test_friction_slows_skid_smoothly(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance)
        v.flight_state.status=FlightStatus.GROUNDED;v.velocity[0]=20
        c.update(.1,commands());self.assertTrue(0<v.velocity[0]<20)
        c.update(12,commands());self.assertLess(abs(v.velocity[0]),.01)
        self.assertGreater(v.position[0],10)

    def test_walking_force_is_pure_bounded_and_accelerates(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance);v.flight_state.status=FlightStatus.GROUNDED
        p=v.position.copy();vv=v.velocity.copy()
        desired,accel=walking_acceleration(v,commands(pitch=-1,roll=1),.01)
        self.assertLessEqual(np.linalg.norm(accel),BATTLEDROID.ground_acceleration+1e-9)
        np.testing.assert_array_equal(v.position,p);np.testing.assert_array_equal(v.velocity,vv)
        c.update(.1,commands(pitch=-1));self.assertLess(v.velocity[2],0)
        self.assertLess(v.speed,1);self.assertLess(v.position[2],0)
        c.update(5,commands(pitch=-1));self.assertGreater(v.speed,4)
        c.update(.1,commands());self.assertGreater(v.speed,3)

    def test_ground_yaw_and_stabilization_are_bounded(self):
        v,c=battledroid();v.rotate(roll=.2);v.position[1]=contact_clearance(v)
        v.flight_state.status=FlightStatus.GROUNDED;up=v.up.copy()
        c.update(.01,commands(yaw=1,pitch=-1,roll=1))
        self.assertLess(v.yaw_rate,math.radians(BATTLEDROID.ground_turn_rate_deg))
        self.assertLess(np.linalg.norm(v.up-up),.02)
        for _ in range(600):c.update(1/120,commands())
        self.assertGreater(v.up[1],.999)

    def test_jump_is_short_force_burst_and_lands_without_scripted_arc(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance);v.flight_state.status=FlightStatus.GROUNDED
        v.velocity[0]=3;pilot=commands();pilot.jump_pressed=True
        before=v.position.copy();c.update(.01,pilot)
        self.assertIs(v.flight_state.status,FlightStatus.FLYING)
        self.assertGreater(v.velocity[1],0);self.assertGreater(v.velocity[0],2.9)
        self.assertLess(v.position[1]-before[1],.01)
        pilot.jump_pressed=False
        for _ in range(600):c.update(1/120,pilot)
        self.assertTrue(v.is_grounded);self.assertEqual(v.velocity[1],0)
        self.assertGreater(v.position[0],0)
        self.assertEqual(v.ground_contact.hard_landing_time,0)

    def test_hard_landing_and_extreme_crash(self):
        for speed,status in ((10,FlightStatus.GROUNDED),(50,FlightStatus.CRASHED)):
            v,c=battledroid(height=BATTLEDROID.foot_clearance+.01);v.velocity[1]=-speed
            c.update(.01,commands())
            self.assertIs(v.flight_state.status,status)
            self.assertGreater(v.ground_contact.hard_landing_time,0)
            self.assertEqual(v.velocity[1],0)

    def test_leaving_grounded_mode_releases_ground_constraint(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance);v.flight_state.status=FlightStatus.GROUNDED
        before=v.position.copy();v.toggle_configuration()
        self.assertIs(v.flight_state.status,FlightStatus.FLYING)
        np.testing.assert_array_equal(v.position,before)
        c.update(.1,commands());self.assertLess(v.position[1],before[1])

    def test_ground_query_can_remove_support(self):
        v,c=battledroid(height=BATTLEDROID.foot_clearance);v.flight_state.status=FlightStatus.GROUNDED
        c.battledroid.ground_height=lambda pos:None
        c.update(.1,commands());self.assertIs(v.flight_state.status,FlightStatus.FLYING)
        self.assertLess(v.velocity[1],0)

    def test_combat_and_countermeasures_inherit_battledroid_motion(self):
        v,_=battledroid();v.velocity[:]=(6,-2,3)
        gun=Gun().fire(v);target=Target(1,(0,50,-1000));missile=Missile(v,target)
        np.testing.assert_allclose(gun.velocity,v.velocity+v.forward*1000)
        np.testing.assert_allclose(missile.velocity,v.velocity+v.forward*missile.parameters.launch_speed)
        for kind in CountermeasureType:
            package=Countermeasure(v,kind)
            np.testing.assert_allclose(package.velocity,v.velocity-v.forward*8-v.up*4)

    def test_k_and_g_edges_preserve_spacebar_gun_and_r_lift(self):
        controls=Input();held={glfw.KEY_K,glfw.KEY_G,glfw.KEY_SPACE,glfw.KEY_R}
        with patch('engine.input.glfw.get_key',side_effect=lambda w,k:glfw.PRESS if k in held else glfw.RELEASE):
            controls.poll(None);self.assertTrue(controls.jump_pressed and controls.toggle_configuration and controls.fire_gun)
            self.assertEqual(controls.lift,1)
            controls.poll(None);self.assertFalse(controls.jump_pressed or controls.toggle_configuration)
            self.assertTrue(controls.fire_gun)

    def test_hud_camera_and_robot_geometry_without_context(self):
        from engine.renderer import Renderer
        g=Game(enemy_count=0);g.setup_battledroid_test('ground');v=g.player_vehicle
        labels=instrument_labels(v,g.flight_controller.fcc,True)
        self.assertIn('MODE BATTLEDROID',labels);self.assertIn('GROUND',labels)
        self.assertFalse(any('RESET' in label for label in labels))
        hud=HUD();hud.resize(1280,720)
        self.assertTrue(np.isfinite(hud.geometry(v,g.camera,g.flight_controller.fcc,True,g.combat)).all())
        self.assertGreater(np.linalg.norm(g.camera.position-v.position),12)
        with patch('engine.renderer.Mesh') as mesh:
            Renderer._create_battledroid_mesh()
            vertices=np.array(mesh.call_args.args[0]);self.assertEqual(len(vertices),9*36)
            self.assertAlmostEqual(vertices[:,1].min(),-BATTLEDROID.foot_clearance)
            self.assertAlmostEqual(vertices[:,1].max()-vertices[:,1].min(),BATTLEDROID.height)
        r=Renderer(g.world,v,g.camera,g.flight_controller.fcc,g.combat)
        resource=MagicMock();r.battledroid_mesh=resource
        r.close();r.close();resource.close.assert_called_once()

    def test_frame_rate_consistency(self):
        for grounded in (False,True):
            results=[]
            for fps in (30,60,144):
                v,c=battledroid(height=BATTLEDROID.foot_clearance if grounded else 50)
                if grounded:v.flight_state.status=FlightStatus.GROUNDED
                for _ in range(fps*2):c.update(1/fps,commands(pitch=-.4 if grounded else .1,yaw=.2,thrust=.2))
                results.append((v.position.copy(),v.velocity.copy()))
            for result in results[1:]:np.testing.assert_allclose(result,results[0],atol=.01)

    def test_upward_penetration_is_corrected_without_removing_upward_motion(self):
        v,c=battledroid(height=1.);v.velocity[1]=2
        c.update(.001,commands())
        self.assertGreaterEqual(v.position[1],contact_clearance(v))
        self.assertGreater(v.velocity[1],0)
        self.assertIs(v.flight_state.status,FlightStatus.FLYING)

    def test_jump_and_ground_contact_at_multiple_frame_rates(self):
        results=[]
        for fps in (30,60,144):
            v,c=battledroid(height=BATTLEDROID.foot_clearance);v.flight_state.status=FlightStatus.GROUNDED
            v.velocity[0]=4
            for frame in range(fps*3):
                pilot=commands();pilot.jump_pressed=frame==0
                c.update(1/fps,pilot)
            self.assertTrue(v.is_grounded)
            results.append((v.position.copy(),v.velocity.copy()))
        for result in results[1:]:np.testing.assert_allclose(result,results[0],atol=.06)

    def test_powered_landing_and_safe_skid_presets(self):
        for preset in ('landing','skid'):
            g=Game(enemy_count=0);g.setup_battledroid_test(preset)
            for _ in range(180):g.update(1/60)
            v=g.player_vehicle
            self.assertTrue(v.is_grounded)
            self.assertLess(v.ground_contact.impact_speed,BATTLEDROID.safe_landing_vertical_speed)
            if preset=='skid':
                self.assertGreater(v.velocity[0],10)
                self.assertGreater(v.position[0],40)

if __name__=='__main__':unittest.main()
