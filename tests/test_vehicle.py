"""Headless checks for vehicle physics and observer camera separation."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from engine.camera import Camera, CHASE_DISTANCE, CHASE_HEIGHT
from engine.flight_controller import FlightController, DAMPING
from engine.input import Input
from game.player_vehicle import PlayerVehicle


def commands(**changes):
    values = dict(thrust=0, pitch=0, yaw=0, roll=0, lift=0, strafe=0, brake=False, vector_command=0)
    values.update(changes)
    return SimpleNamespace(**values)


class VehicleTests(unittest.TestCase):
    def test_turning_preserves_world_velocity_and_new_thrust_adds_sideways(self):
        vehicle = PlayerVehicle()
        vehicle.velocity[:] = (0, 0, -10)
        controller = FlightController(vehicle)
        controller.update(1.5, commands(yaw=-1))
        np.testing.assert_allclose(vehicle.forward, (1, 0, 0), atol=1e-12)
        np.testing.assert_allclose(vehicle.velocity, (0, 0, -10 * np.exp(-DAMPING * 1.5)))
        controller.update(.5, commands(thrust=1))
        self.assertGreater(vehicle.velocity[0], 0)
        self.assertLess(vehicle.velocity[2], 0)

    def test_upside_down_remains_and_basis_stays_orthonormal(self):
        vehicle = PlayerVehicle()
        vehicle.rotate(roll=np.pi)
        FlightController(vehicle).update(5, commands())
        np.testing.assert_allclose(vehicle.up, (0, -1, 0), atol=1e-12)
        for _ in range(2000):
            vehicle.rotate(pitch=.013, yaw=.021, roll=-.035)
        basis = vehicle.orientation
        np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(basis), 1)

    def test_frame_rate_consistency(self):
        results = []
        for fps in (30, 60, 144):
            vehicle = PlayerVehicle()
            controller = FlightController(vehicle)
            for _ in range(fps * 3):
                controller.update(1 / fps, commands(thrust=1, yaw=.3, roll=.5, lift=.2))
            results.append((vehicle.position.copy(), vehicle.velocity.copy(), vehicle.orientation))
        for result in results[1:]:
            for actual, expected in zip(result, results[0]):
                np.testing.assert_allclose(actual, expected, atol=.001)

    def test_chase_offset_and_cockpit_do_not_modify_vehicle(self):
        vehicle = PlayerVehicle()
        vehicle.rotate(pitch=.4, yaw=.8, roll=np.pi)
        camera = Camera()
        position, orientation = vehicle.position.copy(), vehicle.orientation.copy()
        camera.follow(vehicle, 0)
        np.testing.assert_allclose(camera.position,
                                   position - vehicle.forward * CHASE_DISTANCE + vehicle.up * CHASE_HEIGHT)
        np.testing.assert_allclose(camera.orientation.T @ camera.orientation, np.eye(3), atol=1e-12)
        camera.toggle_mode()
        camera.follow(vehicle, .1)
        np.testing.assert_allclose(camera.position, position)
        np.testing.assert_allclose(camera.orientation, orientation)
        camera.rotate(yaw=1)
        np.testing.assert_allclose(vehicle.orientation, orientation)
        np.testing.assert_allclose(vehicle.position, position)
        camera.toggle_mode()
        camera.follow(vehicle, 0)
        self.assertEqual(camera.mode, 'CHASE')

    def test_chase_smoothing_matches_elapsed_time_for_fixed_target(self):
        vehicle = PlayerVehicle()
        results = []
        for fps in (30, 144):
            camera = Camera()
            camera.follow(vehicle, 0)
            camera.position += (20, 0, 0)
            for _ in range(fps):
                camera.follow(vehicle, 1 / fps)
            results.append(camera.position)
        np.testing.assert_allclose(*results, atol=1e-12)

    def test_toggle_keys_require_release(self):
        import glfw
        controls = Input()
        held = {glfw.KEY_C, glfw.KEY_H}
        with patch('engine.input.glfw.get_key', side_effect=lambda window, key: glfw.PRESS if key in held else glfw.RELEASE):
            controls.poll(None)
            self.assertTrue(controls.toggle_camera and controls.toggle_axes)
            controls.poll(None)
            self.assertFalse(controls.toggle_camera or controls.toggle_axes)
            held.clear()
            controls.poll(None)
            held.update((glfw.KEY_C, glfw.KEY_H))
            controls.poll(None)
            self.assertTrue(controls.toggle_camera and controls.toggle_axes)


if __name__ == '__main__':
    unittest.main()
