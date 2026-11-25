"""M23 world, occupancy, driving, and animation regressions."""

import math
import unittest
from types import SimpleNamespace

import numpy as np

from engine.camera import ORBIT_SENSITIVITY
from engine.driving_camera import DrivingCamera
from engine.game import Game
from engine.jeep_asset import JeepVisual, jeep_access_points
from game.ground_environment import GroundEnvironment
from game.ground_vehicle import DriveParameters, GroundVehicle
from game.player_control import ControlMode, PedestrianSpace, VehicleAccess


def ground(terrain=lambda _: 0.0, obstacles=(), resistance=0.015):
    environment = SimpleNamespace(solid_boxes=obstacles, buildings=[])
    result = GroundEnvironment(environment, None)
    result.height = terrain
    result.resistance = lambda _: resistance
    result.normal = lambda _: np.array((0.0, 1.0, 0.0))
    return result


class DrivingTests(unittest.TestCase):
    def setup_course(self, surface=None):
        obstacle = SimpleNamespace(position=np.array((1000.0, 1.0, 1000.0)))
        surface = surface or ground()
        surface.body_bounds = lambda _: (np.full(3, -3.0), np.full(3, 3.0))
        space = PedestrianSpace(surface, obstacle)
        vehicle = GroundVehicle("test", (0, 0, 0), 0)
        vehicle.place(space.ground)
        vehicle.parking_brake = False
        return vehicle, space

    def test_fixed_step_acceleration_speed_dependent_steering_and_wheels(self):
        first, space = self.setup_course()
        second, _ = self.setup_course()
        for _ in range(60):
            first.update(1 / 30, space, throttle=1, steer=0.35)
        for _ in range(288):
            second.update(1 / 144, space, throttle=1, steer=0.35)
        np.testing.assert_allclose(first.position, second.position, atol=1e-8)
        np.testing.assert_allclose(first.wheel_spin, second.wheel_spin, atol=1e-8)
        self.assertGreater(first.speed, 5)
        self.assertLess(first.heading, -0.2)
        speed_steer = abs(first.steering)
        stationary, _ = self.setup_course()
        stationary.update(2, space, steer=0.35)
        self.assertGreater(abs(stationary.steering), speed_steer)
        self.assertFalse(np.allclose(first.wheel_spin, 0))
        self.assertNotAlmostEqual(first.wheel_spin[0], first.wheel_spin[1])

        visual = JeepVisual()
        visual.pose(first)
        self.assertNotAlmostEqual(visual.hubs[0].local_matrix[0, 2], 0)
        self.assertAlmostEqual(visual.hubs[2].local_matrix[0, 2], 0)
        self.assertNotAlmostEqual(visual.wheels[0].local_matrix[1, 2], 0)
        self.assertEqual(visual.wheels[0].parent.name, "Hub.FL")
        self.assertEqual(len(jeep_access_points()), 5)

    def test_brakes_reverse_parking_and_traction(self):
        vehicle, space = self.setup_course()
        vehicle.update(1.5, space, throttle=1)
        speed = vehicle.speed
        start = vehicle.position.copy()
        vehicle.parking_brake = True
        vehicle.update(0.15, space, throttle=1)
        self.assertLess(vehicle.speed, speed)
        self.assertGreater(vehicle.speed, 0)
        vehicle.update(2, space, throttle=1)
        self.assertAlmostEqual(vehicle.speed, 0)
        parked = vehicle.position.copy()
        vehicle.update(1, space, throttle=-1)
        np.testing.assert_allclose(vehicle.position, parked, atol=1e-8)
        self.assertGreater(parked[2], start[2])
        vehicle.parking_brake = False
        vehicle.update(1, space, throttle=-1)
        self.assertLess(vehicle.speed, -1)
        vehicle.update(1.5, space, brake=True, throttle=-1)
        self.assertAlmostEqual(vehicle.speed, 0)
        sand, desert = self.setup_course(ground(resistance=0.25))
        road, pavement = self.setup_course()
        sand.update(3, desert, throttle=1)
        road.update(3, pavement, throttle=1)
        self.assertGreater(road.speed, sand.speed + 2)

    def test_four_wheels_suspension_slope_and_gravity(self):
        def profile(point):
            return point[0] * 0.12 + point[2] * 0.18

        vehicle, space = self.setup_course(ground(terrain=profile))
        vehicle.parking_brake = True
        vehicle.update(2, space)
        self.assertTrue(vehicle.contacts.all())
        self.assertAlmostEqual(vehicle.pitch, math.atan(0.18), places=3)
        self.assertAlmostEqual(vehicle.roll, math.atan(0.12), places=3)
        self.assertGreater(vehicle.wheel_heights[0], vehicle.wheel_heights[3])
        vehicle.position[1] += 3
        vehicle.update(0.1, space)
        self.assertFalse(vehicle.contacts.any())
        self.assertLess(vehicle.vertical_speed, 0)
        vehicle.update(2, space)
        self.assertTrue(vehicle.contacts.all())
        self.assertAlmostEqual(vehicle.position[1], DriveParameters().ride_height, places=2)

    def test_swept_wall_collision_and_camera_clipping(self):
        wall = ((0, 1, 7), (8, 5, 0.08))
        rear = ((0, 2, -3.5), (9, 6, 0.12))
        vehicle, space = self.setup_course(ground(obstacles=(wall, rear)))
        camera = DrivingCamera()
        camera.begin(vehicle)
        camera.observe(vehicle, space, 0.2)
        self.assertGreater(camera.position[2], -3.5)
        camera.orbit_drag(180, 12)
        camera.orbit_zoom(100)
        self.assertEqual(camera.orbit_distance, 2.5)
        camera.orbit_zoom(-100)
        self.assertEqual(camera.orbit_distance, 15)
        vehicle.update(4, space, throttle=1)
        self.assertLess(vehicle.position[2] + vehicle.half_length, 7)
        self.assertAlmostEqual(vehicle.speed, 0)
        vehicle.update(0.4, space, throttle=-1)
        self.assertLess(vehicle.speed, 0)

    def test_parked_aircraft_is_a_driving_obstacle(self):
        vehicle, space = self.setup_course()
        space.vehicle.position[:] = (0, 1, 9)
        vehicle.update(4, space, throttle=1)
        self.assertLess(vehicle.position[2] + vehicle.half_length, 6.001)
        self.assertAlmostEqual(vehicle.speed, 0)


class OccupancyTests(unittest.TestCase):
    def setUp(self):
        self.game = Game(enemy_count=0)
        self.addCleanup(self.game.world.terrain.dataset.close)
        self.control = self.game.player_control
        self.aircraft = self.game.player_vehicle
        self.jeep = self.game.world.ground_vehicles[0]

    def press_enter(self):
        self.game.input.interact_pressed = True
        self.game.update(1 / 60)
        self.game.input.interact_pressed = False

    def walk(self, target, limit=350):
        pilot = self.control.pilot
        heading = target - pilot.position
        heading[1] = 0
        if np.linalg.norm(heading) < 0.35:
            return
        pilot.forward[:] = heading / np.linalg.norm(heading)
        self.game.input.pilot_advance = 1
        self.game.input.pilot_run = True
        for _ in range(limit):
            offset = target - self.control.pilot.position
            if np.linalg.norm(offset[[0, 2]]) < 0.35:
                break
            self.game.update(1 / 60)
        self.game.input.pilot_advance = 0
        self.game.input.pilot_run = False

    def test_nearest_eligible_vehicle_and_no_boarding_in_air(self):
        self.press_enter()
        pilot = self.control.pilot
        alternative = GroundVehicle("other-jeep", (100, 0, 2900), 0, label="Utility")
        closer = GroundVehicle("close-jeep", (100, 0, 2904.4), 0, label="Service")
        for vehicle in (alternative, closer):
            vehicle.place(self.game.world.ground)
            self.control.register(VehicleAccess(vehicle))
        pilot.position[:] = (98.5, 0, 2902.25)
        self.assertEqual(self.control.prompt(), "ENTER TO BOARD SERVICE")
        closer.speed = 2
        self.assertEqual(self.control.prompt(), "ENTER TO BOARD UTILITY")
        pilot.grounded = False
        self.assertFalse(self.control.interact())
        pilot.grounded = True
        self.assertTrue(self.control.interact())
        self.assertIs(self.control.active.vehicle, alternative)
        self.assertIs(self.control.active.occupant, pilot)

    def test_full_walk_drive_park_and_resume_aircraft(self):
        self.assertIs(self.control.mode, ControlMode.AIRCRAFT)
        flight_position = self.aircraft.position.copy()
        self.press_enter()
        pilot = self.control.pilot
        exit_from_plane = pilot.position.copy()
        target = self.jeep.anchor("Exit.L")
        self.walk(target)
        self.assertIn("JEEP", self.control.prompt())
        self.press_enter()
        self.assertIs(self.control.mode, ControlMode.GROUND_VEHICLE)
        self.assertIs(self.control.active.occupant, pilot)
        self.assertIs(self.control.active.vehicle, self.jeep)
        heading = self.jeep.heading
        self.game.camera_input.drag = (180, 12)
        self.game.update(1 / 60)
        self.assertAlmostEqual(self.game.driving_camera.orbit_yaw, -180 * ORBIT_SENSITIVITY)
        self.assertAlmostEqual(
            self.game.driving_camera.orbit_pitch, -0.23 - 12 * ORBIT_SENSITIVITY
        )
        self.assertEqual(self.jeep.heading, heading)

        self.game.input.toggle_parking_brake = True
        self.game.update(1 / 30)
        self.game.input.toggle_parking_brake = False
        self.game.input.pilot_advance = 1
        self.game.input.toggle_configuration = True
        for _ in range(550):
            self.game.update(1 / 30)
            if self.jeep.position[0] < 100:
                break
        self.assertLess(self.jeep.position[0], 100)
        self.assertNotEqual(self.game.world.ground.surface(self.jeep.position), "apron")
        self.assertFalse(self.control.interact())
        self.assertIn("Stop", self.control.message)
        self.assertFalse(self.aircraft.transformation.active)
        np.testing.assert_array_equal(self.aircraft.position, flight_position)

        self.game.input.pilot_advance = -1
        for _ in range(1800):
            self.game.update(1 / 30)
            if self.jeep.position[0] > 295:
                break
        self.assertGreater(self.jeep.position[0], 295)
        self.game.input.pilot_advance = 0
        self.game.input.fire_gun = True
        for _ in range(60):
            self.game.update(1 / 30)
        self.game.input.fire_gun = False
        self.press_enter()
        self.assertIs(self.control.mode, ControlMode.ON_FOOT)
        self.assertTrue(self.jeep.parking_brake)
        self.assertIs(self.control.pilot, pilot)
        self.assertFalse(self.control.space.blocked(pilot.position))
        parked = self.jeep.position.copy()
        # Exit on either side, route around the bumper before heading for TC167.
        if pilot.position[2] < self.jeep.position[2]:
            self.walk(parked + (-3.5, 0, 0))
            self.walk(parked + (-3.5, 0, 3))
        self.walk(exit_from_plane)
        self.game.input.toggle_configuration = False
        self.press_enter()
        self.assertIs(self.control.mode, ControlMode.AIRCRAFT)
        self.assertIs(self.game.player_vehicle, self.aircraft)
        self.assertIs(self.control.active.occupant, pilot)
        np.testing.assert_array_equal(self.jeep.position, parked)
        self.game.input.toggle_configuration = True
        self.game.update(1 / 60)
        self.assertTrue(self.aircraft.transformation.active)

    def test_jeep_alternate_exit_and_blocked_exit(self):
        self.press_enter()
        self.control.pilot.position[:] = self.jeep.anchor("Exit.L")
        self.control.pilot.position[1] = 0
        self.press_enter()
        self.assertIs(self.control.mode, ControlMode.GROUND_VEHICLE)
        self.jeep.contacts[:] = False
        self.assertFalse(self.control.interact())
        self.jeep.contacts[:] = True
        space = self.control.space
        blocked = self.jeep.anchor("Exit.R")
        space.ground = ground(obstacles=((blocked, (2, 5, 2)),))
        self.assertGreater(
            (self.control.exit_position()[0] - self.jeep.position) @ self.jeep.side, 0
        )
        both = self.jeep.anchor("Exit.L")
        space.ground = ground(obstacles=((blocked, (2, 5, 2)), (both, (2, 5, 2))))
        self.assertFalse(self.control.interact())
        self.assertIn("No safe exit", self.control.message)
