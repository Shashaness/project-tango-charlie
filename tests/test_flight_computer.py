"""FCC command-only feedback and read-only flight-instrument regressions."""

import math
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from engine.camera import Camera
from engine.flight_controller import FlightController
from engine.game import Game
from engine.hud import HUD, instrument_groups, project_direction
from engine.input import Input
from game.atmospheric_physics import PARAMETERS
from game.flight_control_computer import FlightControlComputer, SAS_RATE_DAMPING
from game.flight_instruments import heading_degrees, vertical_speed, normal_g_load, attitude_angles
from game.flight_state import Environment, VehicleMode
from game.vtol_physics import VTOL
from game.player_vehicle import PlayerVehicle
from test_vehicle import commands

LIMITS=dict(pitch=math.radians(60),yaw=math.radians(60),roll=math.radians(90))


def hover_vehicle():
    vehicle=PlayerVehicle(position=(0,1000,0))
    vehicle.flight_state.environment=Environment.ATMOSPHERE
    vehicle.toggle_configuration();vehicle.thrust_vector=1
    vehicle.throttle=PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust
    return vehicle


class FlightComputerTests(unittest.TestCase):
    def test_defaults_and_heading_cardinals_vertical_guard(self):
        fcc=FlightControlComputer()
        self.assertTrue(fcc.stability_assist_enabled);self.assertFalse(fcc.hover_assist_enabled)
        vehicle=PlayerVehicle()
        for direction,heading in (((0,0,-1),0),((1,0,0),90),((0,0,1),180),((-1,0,0),270)):
            vehicle.forward[:]=direction;self.assertEqual(heading_degrees(vehicle),heading)
        vehicle.forward[:]=(0,1,0);self.assertIsNone(heading_degrees(vehicle))

    def test_vertical_speed_is_world_velocity(self):
        vehicle=PlayerVehicle();vehicle.position[1]=1000;vehicle.velocity[1]=-12.3
        self.assertEqual(vertical_speed(vehicle),-12.3)

    def test_hover_fcc_never_modifies_vehicle_and_bounds_throttle(self):
        vehicle=hover_vehicle();fcc=FlightControlComputer();fcc.toggle_hover(vehicle)
        self.assertEqual(fcc.hover_target_altitude,1000)
        for speed in (-1000,-8,0,8,1000):
            vehicle.velocity[:]=(20,speed,-30)
            snapshot=(vehicle.position.copy(),vehicle.velocity.copy(),vehicle.orientation.copy(),
                      vehicle.throttle,vehicle.thrust_vector,vehicle.pitch_rate)
            command=fcc.commands(commands(),vehicle,.01,LIMITS)
            self.assertGreaterEqual(command.throttle_override,0);self.assertLessEqual(command.throttle_override,1)
            actual=(vehicle.position,vehicle.velocity,vehicle.orientation,vehicle.throttle,vehicle.thrust_vector,vehicle.pitch_rate)
            for value,before in zip(actual,snapshot):np.testing.assert_array_equal(value,before)
        vehicle.velocity[:]=0
        self.assertAlmostEqual(fcc.commands(commands(),vehicle,.01,LIMITS).throttle_override,
                               PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust)

    def test_hover_velocity_hold_works_through_physics_and_has_no_altitude_lock(self):
        for initial_speed in (-8,8):
            vehicle=hover_vehicle();vehicle.velocity[1]=initial_speed
            controller=FlightController(vehicle);controller.fcc.toggle_hover(vehicle)
            for _ in range(300):controller.update(1/60,commands())
            self.assertLess(abs(vehicle.velocity[1]),.02)
            self.assertGreater(abs(vehicle.position[1]-1000),2)
            self.assertAlmostEqual(vehicle.throttle,PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust)
            self.assertTrue(controller.fcc.hover_correction_active)

    def test_hover_pilot_override_and_horizontal_freedom(self):
        vehicle=hover_vehicle();fcc=FlightControlComputer();fcc.toggle_hover(vehicle)
        for pilot in (commands(thrust=1),commands(vector_command=-1),commands(lift=1)):
            self.assertIsNone(fcc.commands(pilot,vehicle,.01,LIMITS).throttle_override)
        self.assertIsNone(fcc.commands(commands(),vehicle,.1,LIMITS).throttle_override)
        for _ in range(100):command=fcc.commands(commands(strafe=1,yaw=1),vehicle,.01,LIMITS)
        self.assertIsNotNone(command.throttle_override)
        self.assertEqual(command.strafe,1);self.assertEqual(command.yaw,1)
        controller=FlightController(vehicle);controller.fcc.toggle_hover(vehicle)
        controller.update(.5,commands(lift=1))
        self.assertGreater(vehicle.velocity[1],2)

    def test_hover_only_operates_in_atmospheric_vtol_with_upward_thrust(self):
        vehicle=PlayerVehicle();fcc=FlightControlComputer();fcc.toggle_hover(vehicle)
        self.assertFalse(fcc.hover_assist_enabled)
        vehicle=hover_vehicle();fcc.toggle_hover(vehicle)
        vehicle.rotate(roll=math.pi)
        self.assertIsNone(fcc.commands(commands(),vehicle,.01,LIMITS).throttle_override)
        vehicle.toggle_configuration()  # BATTLEDROID now supports airborne hover
        self.assertIsNone(fcc.commands(commands(),vehicle,.01,LIMITS).throttle_override)
        vehicle.toggle_configuration()  # FIGHTER remains ineligible
        fcc.commands(commands(),vehicle,.01,LIMITS)
        self.assertFalse(fcc.hover_assist_enabled)

    def test_sas_damps_only_rates_and_leaves_inverted_attitude(self):
        vehicle=hover_vehicle();vehicle.flight_state.environment=Environment.SPACE
        vehicle.rotate(roll=math.pi);vehicle.roll_rate=1
        basis=vehicle.orientation.copy();fcc=FlightControlComputer()
        output=fcc.commands(commands(),vehicle,.1,LIMITS)
        self.assertAlmostEqual(output.roll*LIMITS['roll'],math.exp(-SAS_RATE_DAMPING*.1))
        np.testing.assert_array_equal(vehicle.orientation,basis)
        vehicle.roll_rate=0
        self.assertEqual(fcc.commands(commands(),vehicle,.1,LIMITS).roll,0)
        controller=FlightController(vehicle)
        for _ in range(60):controller.update(1/60,commands())
        np.testing.assert_allclose(vehicle.up,(0,-1,0),atol=1e-12)

    def test_sas_rate_tail_manual_raw_and_pilot_priority(self):
        for enabled in (False,True):
            vehicle=hover_vehicle();vehicle.flight_state.environment=Environment.SPACE
            controller=FlightController(vehicle);controller.fcc.stability_assist_enabled=enabled
            controller.update(.1,commands(roll=1))
            before=vehicle.orientation.copy()
            controller.update(.1,commands())
            if enabled:
                self.assertGreater(vehicle.roll_rate,0)
                self.assertLess(vehicle.roll_rate,LIMITS['roll'])
                self.assertFalse(np.allclose(vehicle.orientation,before))
            else:
                self.assertEqual(vehicle.roll_rate,0);np.testing.assert_array_equal(vehicle.orientation,before)
        fcc=FlightControlComputer();vehicle.roll_rate=1
        self.assertEqual(fcc.commands(commands(roll=-1),vehicle,.1,LIMITS).roll,-1)

    def test_signed_g_excludes_gravity_and_attitude_does_not_fake_load(self):
        vehicle=PlayerVehicle();vehicle.flight_state.environment=Environment.ATMOSPHERE
        vehicle.acceleration[:]=0
        self.assertAlmostEqual(normal_g_load(vehicle),9.81/9.80665)
        vehicle.acceleration[:]=(0,-9.81,0)
        self.assertAlmostEqual(normal_g_load(vehicle),0)
        vehicle.acceleration[:]=(0,-20,0)
        self.assertLess(normal_g_load(vehicle),0)
        vehicle.rotate(roll=math.pi)
        self.assertGreater(normal_g_load(vehicle),0)

    def test_attitude_bank_pitch_inverted_and_vertical_guard(self):
        vehicle=PlayerVehicle()
        np.testing.assert_allclose(attitude_angles(vehicle),(0,0))
        vehicle.rotate(roll=math.pi/4)
        self.assertAlmostEqual(attitude_angles(vehicle)[1],math.pi/4)
        vehicle=PlayerVehicle();vehicle.rotate(pitch=math.radians(20))
        self.assertAlmostEqual(attitude_angles(vehicle)[0],math.radians(20))
        vehicle=PlayerVehicle();vehicle.rotate(roll=math.pi)
        self.assertAlmostEqual(abs(attitude_angles(vehicle)[1]),math.pi)
        vehicle=PlayerVehicle();vehicle.rotate(pitch=math.pi/2)
        self.assertIsNone(attitude_angles(vehicle)[1])

    def test_velocity_marker_near_zero_rearward_and_nonfinite_guards(self):
        camera=Camera()
        for direction in ((0,0,0),(0,0,-.001),(float('nan'),0,0),(float('inf'),0,0)):
            self.assertIsNone(project_direction(direction,camera,800,600))
        point,edge=project_direction((0,0,10),camera,800,600)
        self.assertTrue(edge);self.assertTrue(np.isfinite(point).all())
        self.assertTrue(0<=point[0]<=800 and 0<=point[1]<=600)

    def test_hud_groups_debug_gating_and_geometry_are_read_only(self):
        vehicle=hover_vehicle();fcc=FlightControlComputer()
        normal=instrument_groups(vehicle,fcc)
        self.assertEqual(normal['debug'],[])
        self.assertIn('SAS ON',normal['right']);self.assertIn('HOV OFF',normal['right'])
        self.assertTrue(instrument_groups(vehicle,fcc,debug=True)['debug'])
        snapshot=(vehicle.position.copy(),vehicle.velocity.copy(),vehicle.orientation.copy())
        hud=HUD()
        for size in ((1280,720),(400,300),(1600,900)):
            hud.resize(*size)
            for debug in (False,True):
                data=hud.geometry(vehicle,Camera(),fcc,debug)
                self.assertTrue(np.isfinite(data).all());self.assertEqual(len(data)%3,0)
        for actual,before in zip((vehicle.position,vehicle.velocity,vehicle.orientation),snapshot):
            np.testing.assert_array_equal(actual,before)

    def test_f3_v_toggle_once_per_press(self):
        controls=Input();held={glfw.KEY_F3,glfw.KEY_V}
        with patch('engine.input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None);self.assertTrue(controls.toggle_sas and controls.toggle_hover)
            controls.poll(None);self.assertFalse(controls.toggle_sas or controls.toggle_hover)
            held.clear();controls.poll(None)
            held.update((glfw.KEY_F3,glfw.KEY_V));controls.poll(None)
            self.assertTrue(controls.toggle_sas and controls.toggle_hover)

    def test_hover_and_sas_feedback_are_frame_rate_consistent(self):
        results=[]
        for fps in (30,60,144):
            vehicle=hover_vehicle();vehicle.velocity[1]=-6;vehicle.roll_rate=.2
            controller=FlightController(vehicle);controller.fcc.toggle_hover(vehicle)
            for _ in range(fps*3):controller.update(1/fps,commands())
            results.append((vehicle.position.copy(),vehicle.velocity.copy()))
        for result in results[1:]:np.testing.assert_allclose(result,results[0],atol=.01)
