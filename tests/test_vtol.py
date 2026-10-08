"""Momentum-preserving transformation, vectored forces, and manual hover physics."""

import math
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from engine.flight_controller import FlightController, DAMPING
from engine.game import Game
from engine.hud import instrument_labels
from engine.input import Input
from game.atmospheric_physics import PARAMETERS, calculate_forces
from game.flight_state import Environment, VehicleMode, FlightStatus
from game.vtol_physics import VTOL, thrust_direction
from game.player_vehicle import PlayerVehicle
from test_vehicle import commands


def vtol(environment=Environment.ATMOSPHERE):
    vehicle=PlayerVehicle(position=(0,1000,0))
    vehicle.flight_state.environment=environment
    vehicle.toggle_configuration()
    return vehicle,FlightController(vehicle)


class VTOLTests(unittest.TestCase):
    def test_transformation_round_trip_preserves_all_flight_state(self):
        vehicle=PlayerVehicle(position=(12,1000,-34))
        vehicle.velocity[:]=(20,-5,-140)
        vehicle.rotate(pitch=.2,yaw=.8,roll=2.4)
        vehicle.throttle=.71;vehicle.thrust_vector=.4
        vehicle.pitch_rate=.1;vehicle.yaw_rate=.2;vehicle.roll_rate=.3
        snapshot=(vehicle.position.copy(),vehicle.velocity.copy(),vehicle.orientation.copy(),
                  vehicle.throttle,vehicle.pitch_rate,vehicle.yaw_rate,vehicle.roll_rate,vehicle.thrust_vector)
        for expected in (VehicleMode.VTOL,VehicleMode.BATTLEDROID,VehicleMode.FIGHTER):
            vehicle.toggle_configuration()
            self.assertIs(vehicle.flight_state.mode,expected)
            actual=(vehicle.position,vehicle.velocity,vehicle.orientation,vehicle.throttle,
                    vehicle.pitch_rate,vehicle.yaw_rate,vehicle.roll_rate,vehicle.thrust_vector)
            for value,before in zip(actual,snapshot):np.testing.assert_array_equal(value,before)

    def test_game_toggle_does_not_reset_atmospheric_vehicle(self):
        game=Game();game.flight_controller.reset_atmosphere()
        position=game.player_vehicle.position.copy();velocity=game.player_vehicle.velocity.copy()
        game.input.toggle_configuration=True;game.update(0)
        self.assertIs(game.player_vehicle.flight_state.mode,VehicleMode.FIGHTER)
        self.assertTrue(game.player_vehicle.transformation.active)
        self.assertIs(game.player_vehicle.transformation.target,VehicleMode.VTOL)
        np.testing.assert_array_equal(game.player_vehicle.position,position)
        np.testing.assert_array_equal(game.player_vehicle.velocity,velocity)
        game.input.toggle_configuration=False
        game.player_vehicle.transformation.update(1.5)
        game.input.reset_pressed=True;game.input.lift=1
        game.update(.1)
        self.assertIs(game.player_vehicle.flight_state.mode,VehicleMode.VTOL)
        self.assertNotEqual(game.player_vehicle.position[1],1000)

    def test_vector_endpoints_and_normalized_interpolation(self):
        vehicle,_=vtol()
        for vector,expected in ((0,vehicle.forward),(1,vehicle.up),(.5,(vehicle.forward+vehicle.up)/math.sqrt(2))):
            np.testing.assert_allclose(thrust_direction(vehicle,vector),expected)
            self.assertAlmostEqual(np.linalg.norm(thrust_direction(vehicle,vector)),1)
        vehicle.rotate(roll=math.pi)
        self.assertLess(thrust_direction(vehicle,1)[1],0)

    def test_vector_controls_are_gradual_clamped_persistent(self):
        vehicle,controller=vtol()
        controller.update(1,commands(vector_command=1))
        self.assertAlmostEqual(vehicle.thrust_vector,.5)
        controller.update(.1,commands())
        self.assertAlmostEqual(vehicle.thrust_vector,.5)
        controller.update(2,commands(vector_command=1))
        self.assertEqual(vehicle.thrust_vector,1)
        controller.update(3,commands(vector_command=-1))
        self.assertEqual(vehicle.thrust_vector,0)

    def test_vertical_thrust_exceeds_weight_and_zero_speed_climbs(self):
        vehicle,controller=vtol();vehicle.thrust_vector=1;vehicle.throttle=1
        forces=calculate_forces(vehicle)
        self.assertGreater(forces.thrust_force[1],1.5*PARAMETERS.mass*PARAMETERS.gravity)
        controller.update(.2,commands())
        self.assertGreater(vehicle.velocity[1],1)
        self.assertGreater(vehicle.position[1],1000)

    def test_balanced_hover_is_force_equilibrium_not_velocity_reset(self):
        vehicle,controller=vtol();vehicle.thrust_vector=1
        vehicle.throttle=PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust
        for _ in range(600):controller.update(1/60,commands())
        np.testing.assert_allclose(vehicle.position,(0,1000,0),atol=1e-9)
        np.testing.assert_allclose(vehicle.velocity,0,atol=1e-9)
        vehicle.velocity[:]=(0,3,0)
        controller.update(.1,commands())
        self.assertGreater(vehicle.velocity[1],2.9)
        self.assertGreater(vehicle.position[1],1000)

    def test_vtol_has_reduced_lift_and_greater_high_speed_drag(self):
        vehicle=PlayerVehicle();vehicle.velocity[:]=(0,0,-200);vehicle.rotate(pitch=.1)
        fighter=calculate_forces(vehicle);vehicle.toggle_configuration();walk=calculate_forces(vehicle)
        np.testing.assert_allclose(walk.lift_force,fighter.lift_force*VTOL.wing_lift_factor)
        np.testing.assert_allclose(walk.drag_force,fighter.drag_force*VTOL.drag_factor)
        vehicle.throttle=0;vehicle.position[1]=1000
        FlightController(vehicle).update(.25,commands())
        self.assertLess(vehicle.speed,195)
        self.assertGreater(vehicle.speed,150)

    def test_low_speed_attitude_and_translation_apply_local_forces(self):
        vehicle,controller=vtol();vehicle.rotate(yaw=-math.pi/2)
        right=vehicle.right.copy()
        controller.update(.01,commands(strafe=1,pitch=1))
        self.assertGreater(vehicle.pitch_rate,0)
        self.assertGreater(np.dot(vehicle.velocity,right),0)
        self.assertGreater(np.linalg.norm(vehicle.velocity),0)
        forces=calculate_forces(vehicle,controls=commands(lift=1))
        np.testing.assert_allclose(forces.maneuver_force,vehicle.up*VTOL.maneuver_thrust)

    def test_space_has_no_aerodynamics_or_gravity_and_x_does_not_brake(self):
        vehicle,controller=vtol(Environment.SPACE)
        vehicle.thrust_vector=1;vehicle.throttle=1
        controller.update(.1,commands())
        self.assertGreater(vehicle.velocity[1],0)
        np.testing.assert_array_equal(vehicle.aerodynamics.gravity_force,0)
        np.testing.assert_array_equal(vehicle.aerodynamics.lift_force,0)
        np.testing.assert_array_equal(vehicle.aerodynamics.drag_force,0)
        vehicle.throttle=0;vehicle.velocity[:]=(10,0,0)
        controller.update(.1,commands(brake=True,vector_command=1))
        self.assertAlmostEqual(vehicle.velocity[0],10*math.exp(-DAMPING*.1))

    def test_fighter_does_not_use_vector_controls(self):
        vehicle=PlayerVehicle();controller=FlightController(vehicle)
        controller.update(.2,commands(vector_command=1))
        self.assertEqual(vehicle.thrust_vector,0)

    def test_vtol_suppresses_fighter_stall_warnings(self):
        vehicle,_=vtol();vehicle.aerodynamics.alpha=math.radians(80)
        labels=instrument_labels(vehicle)
        self.assertIn('VEC 0%',labels)
        self.assertNotIn('STALL',labels);self.assertNotIn('HIGH AOA',labels)

    def test_g_press_is_edge_triggered_and_x_z_cancel(self):
        controls=Input();held={glfw.KEY_G,glfw.KEY_X}
        with patch('engine.input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None)
            self.assertTrue(controls.toggle_configuration);self.assertEqual(controls.vector_command,1)
            controls.poll(None);self.assertFalse(controls.toggle_configuration)
            held.add(glfw.KEY_Z);controls.poll(None);self.assertEqual(controls.vector_command,0)
            held.clear();controls.poll(None)
            held.update((glfw.KEY_G,glfw.KEY_Z));controls.poll(None)
            self.assertTrue(controls.toggle_configuration);self.assertEqual(controls.vector_command,-1)

    def test_both_environments_are_frame_rate_consistent(self):
        for environment in (Environment.SPACE,Environment.ATMOSPHERE):
            results=[]
            for fps in (30,60,144):
                vehicle,controller=vtol(environment);vehicle.throttle=.7
                for _ in range(fps*2):controller.update(1/fps,commands(vector_command=1,strafe=.2,pitch=.1))
                results.append((vehicle.position.copy(),vehicle.velocity.copy()))
            for result in results[1:]:np.testing.assert_allclose(result,results[0],atol=.003)
