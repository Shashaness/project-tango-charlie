"""Flight instrumentation, throttle and core rendering regression checks."""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from engine.camera import Camera
from engine.flight_controller import FlightController
from engine.hud import HUD, project_direction
from engine.mesh import Mesh
from game.flight_state import Environment, VehicleMode
from game.player_vehicle import PlayerVehicle
from test_vehicle import commands


class InstrumentTests(unittest.TestCase):
    def test_throttle_persists_clamps_and_does_not_assign_velocity(self):
        vehicle = PlayerVehicle()
        controller = FlightController(vehicle)
        controller.update(1, commands(thrust=1))
        self.assertAlmostEqual(vehicle.throttle, .5)
        controller.update(3, commands(thrust=1))
        self.assertEqual(vehicle.throttle, 1)
        speed = vehicle.speed
        controller.update(.5, commands())
        self.assertEqual(vehicle.throttle, 1)
        self.assertGreater(vehicle.speed, speed)
        controller.update(3, commands(thrust=-1))
        self.assertEqual(vehicle.throttle, 0)
        self.assertGreater(vehicle.speed, 0)

    def test_brake_decelerates_gradually(self):
        normal, brake = PlayerVehicle(), PlayerVehicle()
        normal.velocity[:] = brake.velocity[:] = (0, 0, -20)
        FlightController(normal).update(.5, commands())
        FlightController(brake).update(.5, commands(brake=True))
        self.assertGreater(brake.speed, 0)
        self.assertLess(brake.speed, normal.speed)

    def test_battledroid_configuration_is_implemented(self):
        vehicle = PlayerVehicle()
        self.assertIs(vehicle.flight_state.environment, Environment.SPACE)
        self.assertIs(vehicle.flight_state.mode, VehicleMode.FIGHTER)
        for environment, mode in ((Environment.SPACE, VehicleMode.BATTLEDROID),):
            vehicle.flight_state.environment, vehicle.flight_state.mode = environment, mode
            FlightController(vehicle).update(.1, commands())
            self.assertIs(vehicle.flight_state.mode,VehicleMode.BATTLEDROID)

    def test_marker_uses_camera_orientation_not_position(self):
        camera = Camera()
        self.assertIsNone(project_direction((0, 0, 0), camera, 800, 600))
        point, edge = project_direction((0, 0, -10), camera, 800, 600)
        np.testing.assert_allclose(point, (400, 300))
        self.assertFalse(edge)
        camera.position[:] = (1000, -2000, 100)
        np.testing.assert_allclose(project_direction((0,0,-10), camera, 800,600)[0], point)
        point, edge = project_direction((3,0,-10), camera,800,600)
        self.assertGreater(point[0],400)
        self.assertFalse(edge)
        camera.rotate(yaw=-np.pi/2)
        point, edge = project_direction((0,0,-10), camera,800,600)
        self.assertTrue(edge)
        self.assertLess(point[0],400)
        self.assertTrue(project_direction(-camera.forward*10,camera,800,600)[1])

    def test_hud_resizes_and_only_reads_vehicle(self):
        hud, vehicle, camera = HUD(), PlayerVehicle(), Camera()
        vehicle.velocity[:] = (4, 2, -10)
        snapshot = (vehicle.position.copy(), vehicle.velocity.copy(), vehicle.orientation.copy(), vehicle.throttle)
        for width, height in ((800,600),(1600,900),(400,900)):
            hud.resize(width,height)
            camera.aspect = width/height
            geometry = hud.geometry(vehicle,camera)
            self.assertTrue(np.isfinite(geometry).all())
            self.assertEqual(geometry.shape[1],6)
            self.assertEqual(len(geometry)%3,0)
            center = geometry[:24,:2].mean(axis=0)
            np.testing.assert_allclose(center,(width/2,height/2),atol=.001)
        for actual, expected in zip((vehicle.position,vehicle.velocity,vehicle.orientation,vehicle.throttle),snapshot):
            np.testing.assert_allclose(actual,expected)
        self.assertIsNone(project_direction(vehicle.velocity,camera,0,0))

    def test_hud_restores_depth_and_releases_buffer(self):
        hud = HUD()
        with patch('engine.hud.Mesh') as mesh, patch('engine.hud.GL') as gl:
            gl.glIsEnabled.return_value=True
            hud.initialize()
            hud.render(PlayerVehicle(),Camera(),MagicMock())
            gl.glDisable.assert_called_once_with(gl.GL_DEPTH_TEST)
            gl.glEnable.assert_called_once_with(gl.GL_DEPTH_TEST)
            self.assertGreater(mesh.return_value.count,0)
            hud.close(); hud.close()
            mesh.return_value.close.assert_called_once()

    def test_nonindexed_mesh_preserves_line_primitive(self):
        with patch('engine.mesh.GL') as gl:
            gl.GL_LINES=1
            gl.glGenVertexArrays.return_value=7
            gl.glGenBuffers.return_value=8
            mesh=Mesh([(0,0,0,1,1,1),(1,0,0,1,1,1)],primitive=gl.GL_LINES)
            mesh.draw()
            gl.glDrawArrays.assert_called_once_with(gl.GL_LINES,0,2)
            mesh.close()
