"""M18 contact forces and preserved airborne/SPACE dynamics."""
import math
import unittest
import numpy as np
from engine.flight_controller import FlightController
from engine.game import Game
from engine.hud import instrument_labels
from game.flight_state import Environment as E,VehicleMode as M,FlightStatus as S
from game.player_vehicle import PlayerVehicle
from game.vtol_ground import VTOL_GROUND as P,contact_clearance,foot_geometry
from game.atmospheric_physics import PARAMETERS
from game.vtol_physics import VTOL
from test_vehicle import commands


def vtol(height=None,grounded=False):
    v=PlayerVehicle((0,foot_geometry()[1] if height is None else height,0))
    v.flight_state.environment=E.ATMOSPHERE;v.flight_state.mode=M.VTOL
    v.flight_state.status=S.GROUNDED if grounded else S.FLYING
    v.thrust_vector=1.
    return v,FlightController(v)

class VTOLGroundTests(unittest.TestCase):
    def test_geometry_actual_named_feet_clearance(self):
        feet,height=foot_geometry();self.assertEqual(len(feet),2)
        self.assertAlmostEqual(height,5.92566,places=4)
        v,c=vtol(grounded=True);c.update(.1,commands())
        self.assertAlmostEqual(v.ground_contact.left_foot_height,0,places=9)
        self.assertAlmostEqual(v.ground_contact.right_foot_height,0,places=9)
        self.assertAlmostEqual(v.position[1],height,places=9)

    def test_gentle_touchdown_normal_impulse_preserves_horizontal_momentum(self):
        for descent in (.5,1,2):
            v,c=vtol(foot_geometry()[1]+.001);v.velocity[:]=(12,-descent,4)
            c.update(.002,commands())
            self.assertTrue(v.is_grounded);self.assertIs(v.flight_state.status,S.GROUNDED)
            self.assertGreater(v.velocity[0],11.9);self.assertGreater(v.velocity[2],3.9)
            self.assertEqual(v.velocity[1],0)
            self.assertGreaterEqual(v.position[1],contact_clearance(v))

    def test_hard_landing_warning_and_catastrophic_speed_attitude_horizontal(self):
        v,c=vtol();v.velocity[1]=-8;c.update(.001,commands())
        self.assertTrue(v.is_grounded);self.assertGreater(v.ground_contact.hard_landing_time,0)
        self.assertIn('HARD LANDING',instrument_labels(v))
        for case in ('vertical','attitude','horizontal'):
            v,c=vtol()
            if case=='vertical':v.velocity[1]=-25
            if case=='attitude':v.rotate(roll=math.pi);v.position[1]=contact_clearance(v);v.velocity[1]=-1
            if case=='horizontal':v.velocity[:]=(120,-1,0)
            c.update(.001,commands());self.assertIs(v.flight_state.status,S.CRASHED,case)

    def test_long_rest_support_load_no_chatter_or_penetration(self):
        v,c=vtol(grounded=True)
        for _ in range(360):
            c.update(1/120,commands());self.assertTrue(v.is_grounded)
        np.testing.assert_allclose(v.velocity,0,atol=1e-12)
        self.assertAlmostEqual(v.position[1],foot_geometry()[1],places=9)
        self.assertAlmostEqual(v.ground_contact.reaction_force[1],PARAMETERS.mass*PARAMETERS.gravity,places=7)
        self.assertEqual(v.ground_contact.friction_force[1],0)

    def test_horizontal_friction_decelerates_without_reversal(self):
        v,c=vtol(grounded=True);v.velocity[:]=(5,0,-2)
        for _ in range(1500):
            previous=v.velocity.copy();c.update(1/120,commands())
            self.assertGreaterEqual(v.velocity[0],-1e-10);self.assertLessEqual(v.velocity[2],1e-10)
            self.assertLessEqual(v.speed,np.linalg.norm(previous)+1e-10)
        self.assertLess(v.speed,.02)

    def test_load_decreases_with_thrust_and_existing_controls_lift_off(self):
        v,c=vtol(grounded=True);c.update(.01,commands());full=v.ground_contact.reaction_force[1]
        v.throttle=.25;c.update(.01,commands());partial=v.ground_contact.reaction_force[1]
        self.assertLess(partial,full);self.assertGreater(partial,0)
        before=v.position.copy();v.throttle=1.;c.update(.25,commands())
        self.assertIs(v.flight_state.status,S.FLYING);self.assertGreater(v.velocity[1],0)
        self.assertGreater(v.position[1],before[1]+P.liftoff_epsilon)
        self.assertEqual(v.ground_contact.reaction_force[1],0)

    def test_near_ground_hover_not_contact_and_landing_hud(self):
        v,c=vtol(foot_geometry()[1]+.5);v.throttle=PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust
        c.update(1,commands());self.assertIs(v.flight_state.status,S.FLYING)
        self.assertAlmostEqual(v.position[1],foot_geometry()[1]+.5,places=8)
        self.assertEqual(v.ground_contact.reaction_force[1],0)
        v.velocity[1]=-1;c.update(.01,commands())
        self.assertIn('LANDING',instrument_labels(v));self.assertFalse(v.is_grounded)

    def test_hover_assist_landed_stable_and_intentional_descent_lands(self):
        v,c=vtol(grounded=True);c.fcc.toggle_hover(v)
        self.assertTrue(c.fcc.hover_assist_enabled)
        for _ in range(240):
            c.update(1/120,commands());self.assertTrue(v.is_grounded)
        self.assertTrue(c.fcc.hover_assist_enabled)
        v.position[1]+=.3;v.flight_state.status=S.FLYING;v.throttle=0
        c.update(.5,commands(thrust=-1))
        self.assertTrue(v.is_grounded)
        for _ in range(240):c.update(1/120,commands())
        self.assertTrue(v.is_grounded);self.assertTrue(c.fcc.hover_assist_enabled)

    def test_ground_thrust_slide_release_and_yaw(self):
        v,c=vtol(grounded=True);c.update(1,commands(strafe=1))
        self.assertGreater(v.velocity[0],5);self.assertTrue(v.is_grounded)
        speed=v.speed;c.update(1,commands());self.assertLess(v.speed,speed);self.assertGreater(v.speed,0)
        orientation=v.orientation.copy();c.update(.4,commands(yaw=.5))
        self.assertTrue(v.is_grounded);self.assertFalse(np.allclose(v.orientation,orientation))
        v.throttle=.35;v.thrust_vector=.5;c.update(.5,commands())
        self.assertGreater(abs(v.velocity[2]),.1)

    def test_ground_force_branch_has_exact_airborne_equivalence(self):
        v,c=vtol(1000);reference,rc=vtol(1000)
        rc.vtol_ground.ground_height=lambda p:None
        v.velocity[:]=reference.velocity[:]=(30,-5,-80)
        v.throttle=reference.throttle=.55;v.thrust_vector=reference.thrust_vector=.7
        for i in range(120):
            pilot=commands(pitch=.1,roll=-.15,yaw=.2,strafe=.3,vector_command=-.05,thrust=.05)
            c.update(1/120,pilot);rc.update(1/120,pilot)
        np.testing.assert_array_equal(v.position,reference.position)
        np.testing.assert_array_equal(v.velocity,reference.velocity)
        np.testing.assert_array_equal(v.orientation,reference.orientation)
        self.assertEqual(v.ground_contact.reaction_force[1],0)

    def test_space_never_applies_contact_or_friction_even_below_ground(self):
        v,c=vtol(grounded=True);c.update(.1,commands());c.switch_space();v.position[1]=-10
        v.velocity[:]=(4,-2,3);v.throttle=0;c.update(.1,commands())
        self.assertIs(v.flight_state.status,S.FLYING)
        np.testing.assert_allclose(v.velocity,np.array((4,-2,3))*math.exp(-.35*.1),atol=1e-12)
        np.testing.assert_array_equal(v.ground_contact.reaction_force,0)
        np.testing.assert_array_equal(v.ground_contact.friction_force,0)

    def test_both_ground_transformation_directions_preserve_contact_and_momentum(self):
        for source,target,height in ((M.BATTLEDROID,M.VTOL,3.2),(M.VTOL,M.BATTLEDROID,foot_geometry()[1])):
            g=Game(enemy_count=0);g.setup_battledroid_test('ground');v=g.player_vehicle
            v.transformation.reset(source);v.flight_state.mode=source;v.position[1]=height;v.velocity[:]=(2,0,-1)
            before=(v.position.copy(),v.velocity.copy(),v.orientation.copy())
            inventory=g.combat.missile_fire_control.inventories.copy();v.transformation.request(target)
            for x,y in zip((v.position,v.velocity,v.orientation),before):np.testing.assert_array_equal(x,y)
            for _ in range(240):g.update(1/120)
            self.assertIs(v.flight_state.mode,target);self.assertTrue(v.is_grounded)
            self.assertGreater(v.speed,0)
            expected=foot_geometry()[1] if target is M.VTOL else 3.2
            self.assertAlmostEqual(v.position[1],expected,places=8)
            self.assertEqual(g.combat.missile_fire_control.inventories,inventory)
            if target is M.BATTLEDROID:
                g.input.pitch=-1;g.update(.5);self.assertGreater(v.locomotion.blend,0)

    def test_exhaust_attachment_landing_liftoff_and_skim(self):
        g=Game(enemy_count=0);g.setup_battledroid_test('ground');v=g.player_vehicle
        v.transformation.reset(M.VTOL);v.flight_state.mode=M.VTOL;v.position[1]=foot_geometry()[1]
        v.thrust_vector=1.
        for controls in ((0,0),(.4,1),(1,0)):
            v.throttle,g.input.strafe=controls;g.update(.3)
            v.transformation.model.world_matrices(v.model_matrix())
            for side in ('Left','Right'):
                engine=v.transformation.model.find_node(side+'Engine');marker=v.transformation.model.find_node(side+'EngineExhaust')
                np.testing.assert_allclose(marker.world_matrix,engine.world_matrix@marker.local_matrix,atol=1e-12)
            self.assertEqual(v.locomotion.blend,0)

    def test_fighter_ground_path_remains_original_and_gets_no_support(self):
        v,c=vtol(.001);v.flight_state.mode=M.FIGHTER;v.velocity[1]=-1
        c.update(.01,commands())
        self.assertIs(v.flight_state.status,S.GROUNDED)
        np.testing.assert_array_equal(v.velocity,0)
        np.testing.assert_array_equal(v.aerodynamics.total_force,0)
        np.testing.assert_array_equal(v.ground_contact.reaction_force,0)
        self.assertEqual(v.engine_throttle,0)

    def test_touch_and_go_sustained_thrust_and_relanding(self):
        v,c=vtol(foot_geometry()[1]+.1);v.velocity[1]=-1
        c.update(.2,commands());self.assertTrue(v.is_grounded)
        c.update(.5,commands());self.assertTrue(v.is_grounded)
        v.throttle=1.;c.update(.4,commands());self.assertIs(v.flight_state.status,S.FLYING)
        v.throttle=0.
        for _ in range(240):c.update(1/120,commands())
        self.assertTrue(v.is_grounded);self.assertIsNot(v.flight_state.status,S.CRASHED)

    def test_startup_presets_reset_contact_history_and_use_normal_physics(self):
        g=Game(enemy_count=0)
        for case in ('ground','landing','skim','hard','crash'):
            g.player_vehicle.ground_contact.hard_landing_time=5
            g.setup_vtol_test(case);v=g.player_vehicle
            self.assertEqual(v.ground_contact.hard_landing_time,0)
            self.assertIs(v.flight_state.mode,M.VTOL)
            self.assertAlmostEqual(v.thrust_vector,1)
            if case in ('hard','crash'):
                g.update(.03)
                self.assertIs(v.flight_state.status,S.CRASHED if case=='crash' else S.GROUNDED)
            elif case=='landing':self.assertIs(v.flight_state.status,S.FLYING)
            else:self.assertTrue(v.is_grounded)
