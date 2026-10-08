"""Force directions, atmosphere integration and SPACE regressions."""

import math
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from engine.flight_controller import FlightController, DAMPING
from engine.game import Game
from engine.hud import HUD
from engine.input import Input
from game.atmospheric_physics import PARAMETERS, calculate_forces, coefficients, control_authority
from game.flight_state import Environment, FlightStatus
from game.player_vehicle import PlayerVehicle
from test_vehicle import commands


class AtmosphericTests(unittest.TestCase):
    def test_angles_follow_airflow_and_orientation(self):
        vehicle = PlayerVehicle()
        vehicle.velocity[:] = (0, 0, -120)
        vehicle.rotate(pitch=math.radians(10))
        forces = calculate_forces(vehicle)
        self.assertAlmostEqual(math.degrees(forces.alpha), 10)
        self.assertAlmostEqual(forces.beta, 0)
        vehicle.rotate(pitch=math.radians(-10))
        vehicle.velocity[:] = (10, 0, -100)
        self.assertAlmostEqual(calculate_forces(vehicle).beta, math.atan2(10, 100))
        vehicle.velocity[:] = (0, 10, -100)
        self.assertLess(calculate_forces(vehicle).alpha, 0)

    def test_lift_banks_drag_opposes_flow_and_gravity_is_world_down(self):
        vehicle = PlayerVehicle()
        vehicle.velocity[:] = (0, 0, -120)
        vehicle.rotate(pitch=math.radians(5))
        upright = calculate_forces(vehicle)
        vehicle.rotate(roll=math.radians(60))
        banked = calculate_forces(vehicle)
        self.assertGreater(abs(banked.lift_force[0]), 1000)
        self.assertAlmostEqual(np.dot(banked.lift_force, vehicle.velocity), 0, places=7)
        self.assertLess(np.dot(banked.drag_force, vehicle.velocity), 0)
        np.testing.assert_allclose(banked.gravity_force, (0, -PARAMETERS.mass * PARAMETERS.gravity, 0))
        # Maintain positive body AoA while inverted to check lift's body-relative sign.
        vehicle.forward[:] = (0, 0, -1); vehicle.right[:] = (-1, 0, 0); vehicle.up[:] = (0, -1, 0)
        vehicle.velocity[:] = (0, 10, -120)
        self.assertLess(calculate_forces(vehicle).lift_force[1], 0)
        self.assertGreater(upright.lift_force[1], 0)

    def test_stall_is_symmetric_progressive_and_continuous(self):
        cl18, cd18, _ = coefficients(math.radians(PARAMETERS.stall_angle_deg))
        cl28, cd28, _ = coefficients(math.radians(28))
        cl45, cd45, _ = coefficients(math.radians(45))
        self.assertGreater(cl18, cl28); self.assertGreater(cl28, cl45)
        self.assertGreater(cl45, 0)
        self.assertLess(cd18, cd28); self.assertLess(cd28, cd45)
        negative = coefficients(math.radians(-28))
        self.assertAlmostEqual(negative[0], -cl28)
        self.assertAlmostEqual(negative[1], cd28)
        np.testing.assert_allclose(coefficients(math.radians(PARAMETERS.stall_angle_deg + .00001)), coefficients(math.radians(PARAMETERS.stall_angle_deg - .00001)), atol=1e-6)

    def test_zero_reverse_and_sideways_flow_are_finite(self):
        vehicle = PlayerVehicle()
        for velocity in ((0,0,0),(0,0,120),(120,0,0),(0,120,0)):
            vehicle.velocity[:] = velocity
            forces = calculate_forces(vehicle)
            self.assertTrue(np.isfinite(forces.total_force).all())
            self.assertTrue(math.isfinite(forces.alpha))
        vehicle.velocity[:] = 0
        forces = calculate_forces(vehicle)
        np.testing.assert_allclose(forces.lift_force, 0)
        np.testing.assert_allclose(forces.drag_force, 0)
        self.assertEqual(control_authority(forces), 0)
        vehicle.velocity[:] = (0,0,-120)
        self.assertGreater(control_authority(calculate_forces(vehicle)), 1)
        vehicle.velocity[0] = float('nan')
        with self.assertRaises(ValueError): calculate_forces(vehicle)

    def test_trim_and_long_level_flight(self):
        vehicle = PlayerVehicle(); controller = FlightController(vehicle)
        controller.reset_atmosphere()
        self.assertGreater(vehicle.throttle, .1)
        self.assertLess(vehicle.throttle, .2)
        np.testing.assert_allclose(vehicle.aerodynamics.total_force, 0, atol=1e-8)
        for _ in range(600): controller.update(1/60, commands())
        np.testing.assert_allclose(vehicle.velocity, (0,0,-120), atol=1e-8)
        self.assertAlmostEqual(vehicle.position[1], 1000, places=7)
        controller.update(.5, commands(thrust=1))
        self.assertGreater(vehicle.speed, 120)

    def test_no_atmospheric_strafe_lift_or_space_brake(self):
        results=[]
        for control in (commands(),commands(strafe=1,lift=1,brake=True)):
            vehicle=PlayerVehicle();controller=FlightController(vehicle);controller.reset_atmosphere()
            controller.update(1,control)
            results.append((vehicle.position.copy(),vehicle.velocity.copy()))
        np.testing.assert_allclose(results[0],results[1],atol=1e-12)

    def test_bank_changes_velocity_through_force(self):
        vehicle=PlayerVehicle();controller=FlightController(vehicle);controller.reset_atmosphere()
        vehicle.rotate(roll=math.radians(60))
        before=vehicle.velocity.copy()
        controller.update(.5,commands(pitch=.2))
        self.assertGreater(abs(vehicle.velocity[0]-before[0]),1)
        self.assertNotEqual(vehicle.aerodynamics.lift_force[0],0)

    def test_ground_freezes_and_reset_restores_state(self):
        vehicle=PlayerVehicle();controller=FlightController(vehicle);controller.reset_atmosphere()
        vehicle.position[1]=.1;vehicle.velocity[:]=(0,-20,0)
        controller.update(.1,commands())
        self.assertIs(vehicle.flight_state.status,FlightStatus.CRASHED)
        self.assertEqual(vehicle.position[1],0)
        position=vehicle.position.copy()
        controller.update(2,commands(thrust=1,pitch=1))
        np.testing.assert_array_equal(vehicle.position,position)
        controller.reset_atmosphere()
        self.assertIs(vehicle.flight_state.status,FlightStatus.FLYING)
        self.assertEqual(vehicle.position[1],1000)
        HUD().geometry(vehicle,Game().camera)
        vehicle.position[1]=.001;vehicle.velocity[:]=(0,-.1,0)
        controller.update(.01,commands())
        self.assertIs(vehicle.flight_state.status,FlightStatus.GROUNDED)
        HUD().geometry(vehicle,Game().camera)

    def test_space_remains_inertial_and_has_no_gravity(self):
        vehicle=PlayerVehicle();controller=FlightController(vehicle)
        vehicle.velocity[:]=(4,0,-10)
        controller.update(1,commands())
        np.testing.assert_allclose(vehicle.velocity,np.array((4,0,-10))*math.exp(-DAMPING),atol=1e-12)
        self.assertEqual(vehicle.position[1],0)
        controller.reset_atmosphere();before=vehicle.velocity.copy();controller.switch_space()
        vehicle.throttle=0
        controller.update(.01,commands())
        self.assertIs(vehicle.flight_state.environment,Environment.SPACE)
        self.assertEqual(vehicle.velocity[1],0)
        self.assertGreater(np.dot(vehicle.velocity,before),0)

    def test_frame_rate_and_camera_independence(self):
        results=[]
        for fps, cockpit in ((30,False),(60,True),(144,False)):
            game=Game();game.flight_controller.reset_atmosphere()
            if cockpit: game.camera.toggle_mode()
            game.input.roll=.15;game.input.pitch=.1
            for _ in range(fps*3): game.update(1/fps)
            results.append((game.player_vehicle.position.copy(),game.player_vehicle.velocity.copy()))
        for result in results[1:]: np.testing.assert_allclose(result,results[0],atol=.01)

    def test_low_speed_controls_and_reduced_throttle(self):
        rates=[]
        for speed in (1,120):
            vehicle=PlayerVehicle();controller=FlightController(vehicle);controller.reset_atmosphere()
            vehicle.velocity[:]=(0,0,-speed)
            controller.update(1/120,commands(pitch=1))
            rates.append(vehicle.pitch_rate)
        self.assertLess(rates[0],rates[1]/100)
        vehicle=PlayerVehicle();controller=FlightController(vehicle);controller.reset_atmosphere()
        vehicle.throttle=0
        controller.update(1,commands())
        self.assertLess(vehicle.speed,120)
        self.assertGreater(vehicle.speed,0)

    def test_game_environment_reset_snaps_observer_and_preserves_mode(self):
        game=Game();game.camera.toggle_mode()
        game.input.atmosphere_pressed=True
        game.update(0)
        self.assertIs(game.player_vehicle.flight_state.environment,Environment.ATMOSPHERE)
        np.testing.assert_allclose(game.camera.position,game.player_vehicle.position)
        game.input.atmosphere_pressed=False
        game.player_vehicle.flight_state.status=FlightStatus.CRASHED
        game.input.reset_pressed=True
        game.update(0)
        self.assertIs(game.player_vehicle.flight_state.status,FlightStatus.FLYING)
        self.assertEqual(game.camera.mode,'COCKPIT')
        game.input.reset_pressed=False;game.input.space_pressed=True
        position=game.player_vehicle.position.copy();velocity=game.player_vehicle.velocity.copy()
        game.update(0)
        self.assertIs(game.player_vehicle.flight_state.environment,Environment.SPACE)
        np.testing.assert_allclose(game.player_vehicle.position,position)
        np.testing.assert_allclose(game.player_vehicle.velocity,velocity)

    def test_debug_switches_are_edge_triggered(self):
        controls=Input();held={glfw.KEY_F2,glfw.KEY_R}
        with patch('engine.input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None)
            self.assertTrue(controls.atmosphere_pressed and controls.reset_pressed)
            controls.poll(None)
            self.assertFalse(controls.atmosphere_pressed or controls.reset_pressed)
            held.clear();controls.poll(None)
            held.add(glfw.KEY_F1);controls.poll(None)
            self.assertTrue(controls.space_pressed)
