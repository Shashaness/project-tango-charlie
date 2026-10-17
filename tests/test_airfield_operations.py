"""Runway operations use real forces and contact, with no renderer requirement."""

import unittest
from unittest.mock import patch

import glfw
import numpy as np
from test_vehicle import commands

from engine.flight_controller import FlightController
from engine.game import Game
from engine.input import Input
from game.airfield_operations import AirfieldOperations, AirfieldParameters, FlapState, GearState
from game.atmospheric_physics import PARAMETERS, calculate_forces
from game.fighter_ground import clearance
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.player_vehicle import PlayerVehicle
from game.world_environment import RUNWAY, Runway


class AirfieldTests(unittest.TestCase):
    """Airfield regression tests."""

    def aircraft(self):
        v = PlayerVehicle((0, 10, 2500))
        v.flight_state.environment = Environment.ATMOSPHERE
        v.airfield.reset(deployed=True)
        return v, FlightController(v)

    def touchdown(self, *, gear=True, descent=1, speed=70, z=2500, pitch=0, roll=0):
        v, c = self.aircraft()
        if not gear:
            v.airfield.reset()
        v.rotate(pitch=np.radians(pitch))
        v.rotate(roll=np.radians(roll))
        v.position[:] = (0, clearance(v) + 0.001, z)
        v.velocity[:] = (0, -descent, -speed)
        c.update(1 / 120, commands())
        return v, c

    def test_bindings_edges_hold_and_existing_controls(self):
        """Verify bindings edges hold and existing controls."""
        i = Input()
        down = {glfw.KEY_6, glfw.KEY_7, glfw.KEY_8, glfw.KEY_X}
        with patch(
            "glfw.get_key", side_effect=lambda w, k: glfw.PRESS if k in down else glfw.RELEASE
        ):
            i.poll(None)
            self.assertTrue(i.wheel_brake and i.toggle_flaps and i.toggle_gear and i.brake)
            self.assertFalse(
                i.toggle_configuration or i.select_radar or i.select_ir or i.toggle_camera
            )
            i.poll(None)
            self.assertTrue(i.wheel_brake)
            self.assertFalse(i.toggle_flaps or i.toggle_gear)
            down.clear()
            i.poll(None)
            self.assertFalse(i.wheel_brake or i.brake)

    def test_gear_timing_and_reversal(self):
        """Verify gear timing and reversal."""
        a = AirfieldOperations(AirfieldParameters(gear_seconds=2))
        a.toggle_gear()
        a.update(1)
        self.assertIs(a.gear, GearState.DEPLOYING)
        self.assertEqual(a.extension, 0.5)
        a.toggle_gear()
        a.update(0.5)
        self.assertIs(a.gear, GearState.RETRACTING)
        self.assertEqual(a.extension, 0.25)
        a.update(0.5)
        self.assertIs(a.gear, GearState.RETRACTED)
        a.toggle_gear()
        a.update(2)
        self.assertIs(a.gear, GearState.DEPLOYED)
        a.toggle_gear()
        a.update(2)
        self.assertIs(a.gear, GearState.RETRACTED)

    def test_gear_world_points(self):
        v, _ = self.aircraft()
        v.rotate(yaw=0.5, pitch=0.1, roll=0.2)
        local = np.asarray(v.airfield.parameters.wheel_points)
        expected = (v.model_matrix() @ np.c_[local, np.ones(3)].T).T[:, :3]
        np.testing.assert_allclose(v.airfield.wheel_world(v), expected, atol=1e-5)

    def test_gear_drag_and_flap_lift_drag_space_and_vtol(self):
        """Verify gear drag and flap lift drag space and vtol."""
        v, _ = self.aircraft()
        v.airfield.reset()
        v.velocity[:] = (0, 0, -70)
        v.rotate(pitch=0.1)
        clean = calculate_forces(v)
        v.airfield.reset(deployed=True)
        gear = calculate_forces(v)
        self.assertGreater(gear.drag_coefficient, clean.drag_coefficient)
        v.airfield.toggle_flaps()
        flaps = calculate_forces(v)
        self.assertGreater(flaps.lift_force[1], gear.lift_force[1])
        self.assertGreater(flaps.drag_coefficient, gear.drag_coefficient)
        v.flight_state.environment = Environment.SPACE
        self.assertEqual(calculate_forces(v).lift_coefficient, clean.lift_coefficient)
        v.flight_state.environment = Environment.ATMOSPHERE
        v.flight_state.mode = VehicleMode.VTOL
        before = calculate_forces(v).total_force.copy()
        v.airfield.toggle_flaps()
        v.airfield.reset()
        np.testing.assert_allclose(calculate_forces(v).total_force, before)

    def test_brakes_reduce_ground_speed_without_reversing(self):
        """Verify brakes reduce ground speed without reversing."""
        v, c = self.aircraft()
        v.position[1] = clearance(v)
        v.flight_state.status = FlightStatus.GROUNDED
        v.velocity[:] = (0, 0, -15)
        previous = v.speed
        for _ in range(600):
            c.update(1 / 120, commands(wheel_brake=True))
            self.assertGreaterEqual(-v.velocity[2], 0)
            self.assertLessEqual(v.speed, previous + 1e-8)
            previous = v.speed
        self.assertLess(v.speed, 0.01)
        self.assertTrue(v.airfield.brakes_active)

    def test_brakes_do_not_work_airborne(self):
        a, ca = self.aircraft()
        b, cb = self.aircraft()
        a.position[1] = b.position[1] = 500
        a.velocity[:] = b.velocity[:] = (0, 0, -70)
        ca.update(0.1, commands(wheel_brake=True))
        cb.update(0.1, commands())
        np.testing.assert_allclose(a.velocity, b.velocity)
        self.assertFalse(a.airfield.brakes_active)

    def test_takeoff_using_pitch_and_throttle_then_retract(self):
        """Verify takeoff using pitch and throttle then retract."""
        g = Game(enemy_count=0)
        g.reset_runway()
        v = g.player_vehicle
        for _ in range(600):
            g.flight_controller.update(1 / 120, commands(thrust=1))
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        for _ in range(600):
            g.flight_controller.update(1 / 120, commands(pitch=0.15 if v.forward[1] < 0.17 else 0))
            if v.position[1] > clearance(v) + 5:
                break
        self.assertIs(v.flight_state.status, FlightStatus.FLYING)
        self.assertGreater(v.velocity[1], 0)
        self.assertGreater(v.aerodynamics.lift_force[1], PARAMETERS.mass * PARAMETERS.gravity * 0.8)
        v.airfield.toggle_gear()
        g.flight_controller.update(3, commands())
        self.assertIs(v.airfield.gear, GearState.RETRACTED)

    def test_landing_rollout_and_braking(self):
        v, c = self.touchdown()
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.assertGreater(v.speed, 60)
        self.assertEqual(v.velocity[1], 0)
        for _ in range(1800):
            c.update(1 / 120, commands(wheel_brake=True))
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.assertLess(v.speed, 0.01)

    def test_gear_up_touchdown_crashes(self):
        v, _ = self.touchdown(gear=False)
        self.assertIs(v.flight_state.status, FlightStatus.CRASHED)
        self.assertEqual(v.airfield.crash_reason, "GEAR NOT DOWN")

    def test_partial_gear_touchdown_crashes(self):
        v, c = self.aircraft()
        v.airfield.toggle_gear()
        v.airfield.update(1)
        v.position[1] = clearance(v)
        v.velocity[:] = (0, -1, -60)
        c.update(1 / 120, commands())
        self.assertIs(v.flight_state.status, FlightStatus.CRASHED)

    def test_hard_landing_crashes(self):
        v, _ = self.touchdown(descent=6)
        self.assertIs(v.flight_state.status, FlightStatus.CRASHED)

    def test_pitch_roll_and_excessive_speed_crash(self):
        for kw in (dict(pitch=25), dict(roll=20), dict(speed=150)):
            v, _ = self.touchdown(**kw)
            self.assertIs(v.flight_state.status, FlightStatus.CRASHED)

    def test_off_runway_flat_landing_is_allowed(self):
        v, _ = self.touchdown(z=3800)
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.assertIsNone(v.airfield.crash_reason)

    def test_leaving_flat_runway_does_not_crash(self):
        v, c = self.aircraft()
        v.flight_state.status = FlightStatus.GROUNDED
        v.position[:] = (0, clearance(v), 1306.1)
        v.velocity[:] = (0, 0, -60)
        c.update(0.1, commands())
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.assertIsNone(v.airfield.crash_reason)

    def test_ground_departure_and_no_uncommanded_flap_takeoff(self):
        """Verify ground departure and no uncommanded flap takeoff."""
        v, c = self.aircraft()
        v.flight_state.status = FlightStatus.GROUNDED
        v.position[:] = (0, clearance(v), 1290)
        v.velocity[:] = (0, 0, -30)
        c.update(0.1, commands())
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        v, c = self.aircraft()
        v.position[1] = clearance(v)
        v.flight_state.status = FlightStatus.GROUNDED
        v.airfield.toggle_flaps()
        v.velocity[:] = (0, 0, -100)
        c.update(0.1, commands())
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.assertEqual(v.velocity[1], 0)

    def test_stationary_boundary_and_oriented_bounds(self):
        v, c = self.aircraft()
        v.position[:] = (0, clearance(v), 1306.01)
        v.flight_state.status = FlightStatus.GROUNDED
        c.update(1, commands())
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        r = Runway(center=(20, 0, 30), forward=(1, 0, 0), length=100, width=10)
        self.assertTrue(r.contains((60, 0, 30)))
        self.assertFalse(r.contains((20, 0, 40)))
        self.assertFalse(r.contains((80, 0, 30)))

    def test_f4_and_mode_restrictions(self):
        """Verify f4 and mode restrictions."""
        g = Game(enemy_count=0)
        v = g.player_vehicle
        v.airfield.toggle_flaps()
        v.airfield.toggle_gear()
        g.input.runway_pressed = True
        g.update(0.1)
        self.assertIs(v.airfield.gear, GearState.DEPLOYED)
        self.assertIs(v.airfield.flaps, FlapState.RETRACTED)
        np.testing.assert_array_equal(v.velocity, 0)
        self.assertFalse(v.airfield.brakes)
        v.flight_state.mode = VehicleMode.VTOL
        g.input.toggle_gear = g.input.toggle_flaps = True
        g.input.runway_pressed = False
        g.update(0)
        self.assertIs(v.airfield.gear, GearState.DEPLOYED)
        self.assertIs(v.airfield.flaps, FlapState.RETRACTED)

    def test_hud_indicators(self):
        from engine.hud import instrument_groups

        v, _ = self.aircraft()
        v.airfield.brakes_active = True
        labels = instrument_groups(v)["airfield"]
        self.assertTrue({"GEAR DOWN", "FLAPS UP", "BRAKES ACTIVE", "AIRBORNE"}.issubset(labels))
        v.airfield.toggle_gear()
        v.velocity[1] = -1
        self.assertIn("GEAR NOT DOWN", instrument_groups(v)["warnings"])


if __name__ == "__main__":
    unittest.main()
