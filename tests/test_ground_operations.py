"""Physical ground steering, traversable surfaces and open hangar regression tests."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from test_vehicle import commands

from engine.flight_controller import FlightController
from engine.game import Game
from game.airfield_operations import GearState
from game.fighter_ground import clearance
from game.flight_instruments import heading_degrees
from game.flight_state import FlightStatus, VehicleMode
from game.ground_environment import GroundEnvironment
from game.world_environment import HANGAR_SPAWN, RUNWAY


class GroundOperationsTests(unittest.TestCase):
    """Ground Operations regression tests."""

    def setUp(self):
        self.game = Game(enemy_count=0)
        self.v = self.game.player_vehicle
        self.c = self.game.flight_controller

    def tearDown(self):
        self.game.world.terrain.dataset.close()

    def drive(self, heading, stop, limit=8000, speed=5):
        for _ in range(limit):
            error = (heading - heading_degrees(self.v) + 180) % 360 - 180
            pilot = commands(
                thrust=float(np.clip((speed - self.v.speed) * 0.12, -1, 1)),
                yaw=float(np.clip(-error / 12, -1, 1)),
            )
            self.c.update(1 / 60, pilot)
            self.assertIsNot(self.v.flight_state.status, FlightStatus.CRASHED)
            if stop(self.v):
                return
        self.fail("Taxi route did not reach waypoint")

    def test_safe_hangar_spawn_and_open_exit(self):
        v = self.v
        ground = self.game.world.ground
        self.assertEqual(tuple(v.position[[0, 2]]), (HANGAR_SPAWN[0], HANGAR_SPAWN[2]))
        self.assertIs(v.airfield.gear, GearState.DEPLOYED)
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        np.testing.assert_array_equal(v.velocity, 0)
        ground.collide(v, v.position.copy(), 1 / 120)
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        self.drive(270, lambda v: v.position[0] < 245)
        self.assertEqual(ground.surface(v.position), "apron")

    def test_taxi_to_runway_and_takeoff(self):
        """Verify taxi to runway and takeoff."""
        self.drive(270, lambda v: v.position[0] < 120)
        self.drive(0, lambda v: abs((heading_degrees(v) + 180) % 360 - 180) < 0.5)
        self.assertEqual(self.game.world.ground.surface(self.v.position), "taxiway")
        self.drive(0, lambda v: v.position[2] < 2500)
        self.drive(270, lambda v: abs((heading_degrees(v) - 270 + 180) % 360 - 180) < 0.5)
        self.drive(270, lambda v: v.position[0] < 20)
        self.drive(0, lambda v: abs((heading_degrees(v) + 180) % 360 - 180) < 0.5)
        self.assertTrue(self.c.fighter_ground.on_runway())
        for _ in range(800):
            self.c.update(
                1 / 120,
                commands(
                    thrust=1, pitch=0.15 if self.v.speed > 80 and self.v.forward[1] < 0.17 else 0
                ),
            )
            if self.v.position[1] > clearance(self.v) + 5:
                break
        self.assertIs(self.v.flight_state.status, FlightStatus.FLYING)
        self.assertGreater(self.v.velocity[1], 0)

    def test_return_taxi_into_hangar(self):
        # Set up an airborne final; rollout and return are physical, not teleports.
        """Verify return taxi into hangar."""
        v = self.v
        v.forward[:] = (0, 0, -1)
        v.right[:] = (1, 0, 0)
        v.up[:] = (0, 1, 0)
        v.position[:] = (0, 3.001, 3500)
        v.velocity[:] = (0, -1, -65)
        v.flight_state.status = FlightStatus.FLYING
        self.c.update(1 / 120, commands())
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        for _ in range(1800):
            self.c.update(1 / 120, commands(wheel_brake=True))
        self.assertLess(v.speed, 0.01)
        self.drive(90, lambda v: abs((heading_degrees(v) - 90 + 180) % 360 - 180) < 0.5)
        self.drive(90, lambda v: v.position[0] > 170)
        self.drive(0, lambda v: abs((heading_degrees(v) + 180) % 360 - 180) < 0.5)
        self.drive(0, lambda v: v.position[2] < 3020)
        self.drive(90, lambda v: abs((heading_degrees(v) - 90 + 180) % 360 - 180) < 0.5)
        self.drive(90, lambda v: v.position[0] > 300)
        for _ in range(360):
            self.c.update(1 / 120, commands(wheel_brake=True, thrust=-1))
        self.assertLess(v.speed, 0.1)
        self.assertTrue(255 < v.position[0] < 340 and 2947 < v.position[2] < 3053)

    def test_q_e_steering_needs_motion_and_does_not_directly_set_yaw(self):
        """Verify q e steering needs motion and does not directly set yaw."""
        orientation = self.v.orientation.copy()
        self.c.update(0.2, commands(yaw=1))
        np.testing.assert_allclose(self.v.orientation, orientation)
        self.v.velocity[:] = self.v.forward * 5
        self.c.update(0.2, commands(yaw=1))
        self.assertGreater(self.v.yaw_rate, 0)
        self.assertLess(self.v.yaw_rate, 0.3)
        self.assertLess(heading_degrees(self.v), 270)
        self.v.velocity[:] = self.v.forward * 5
        self.c.update(0.5, commands(yaw=-1))
        self.assertLess(self.v.yaw_rate, 0)

    def test_repeated_open_apron_circles_at_multiple_frame_rates(self):
        """Verify repeated open apron circles at multiple frame rates."""
        for fps in (30, 60, 120):
            for steering in (1, -1):
                with self.subTest(fps=fps, steering=steering):
                    v = self.v
                    v.position[:] = (185, 3, 3000)
                    v.forward[:] = (0, 0, -1)
                    v.right[:] = (1, 0, 0)
                    v.up[:] = (0, 1, 0)
                    v.velocity[:] = (0, 0, -5)
                    v.yaw_rate = 0
                    v.throttle = 0.01
                    v.flight_state.status = FlightStatus.GROUNDED
                    last = heading_degrees(v)
                    accumulated = 0.0
                    positions = []
                    for frame in range(52 * fps):
                        self.c.update(
                            1 / fps,
                            commands(
                                yaw=steering, thrust=float(np.clip((5 - v.speed) * 0.12, -1, 1))
                            ),
                        )
                        heading = heading_degrees(v)
                        accumulated += (heading - last + 180) % 360 - 180
                        last = heading
                        positions.append(v.position.copy())
                        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
                        self.assertTrue(self.c.fighter_ground.supported)
                        self.assertIsNone(v.airfield.crash_reason)
                        self.assertEqual(self.game.world.ground.surface(v.position), "apron")
                        self.assertLess(abs(float(v.velocity @ v.right)), 0.1)
                        self.assertGreater(float(v.velocity @ v.forward), 4.5)
                    self.assertGreater(-steering * accumulated, 720)
                    points = np.asarray(positions)
                    self.assertGreater(np.ptp(points[:, 0]), 30)
                    self.assertGreater(np.ptp(points[:, 2]), 30)
                    # Centered wheel damps yaw and supports ordinary straight taxi.
                    start = v.position.copy()
                    for _ in range(3 * fps):
                        self.c.update(
                            1 / fps, commands(thrust=float(np.clip((5 - v.speed) * 0.12, -1, 1)))
                        )
                    self.assertLess(abs(v.yaw_rate), 0.001)
                    self.assertGreater(np.linalg.norm(v.position - start), 10)
                    self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)

    def test_reverse_wheel_geometry_and_speed_dependent_radius(self):
        """Verify reverse wheel geometry and speed dependent radius."""
        v = self.v
        v.position[:] = (185, 3, 3000)
        v.forward[:] = (0, 0, -1)
        v.right[:] = (1, 0, 0)
        v.up[:] = (0, 1, 0)
        rates = []
        for speed in (-5, 5, 15):
            v.velocity[:] = v.forward * speed
            v.yaw_rate = 0
            rate = self.c.fighter_ground.steer_rate(1 / 120, 1)
            rates.append(rate)
        self.assertLess(rates[0], 0)
        self.assertGreater(rates[1], 0)
        self.assertGreater(15 / rates[2], 5 / rates[1])

    def test_rotating_hull_overlap_separates_and_can_taxi_away(self):
        """Verify rotating hull overlap separates and can taxi away."""
        v = self.v
        ground = self.game.world.ground
        v.forward[:] = (1, 0, 0)
        v.right[:] = (0, 0, 1)
        v.up[:] = (0, 1, 0)
        # A shallow overlap with the east wall, as a turning hull can produce.
        v.position[:] = (335, 3, 3000)
        v.velocity[:] = (-1, 0, 0)
        previous = v.position.copy()
        v.position += v.velocity / 60
        ground.collide(v, previous, 1 / 60)
        self.assertLess(v.position[0] + ground.body_bounds(v)[1][0], 343)
        self.assertIs(v.flight_state.status, FlightStatus.GROUNDED)
        start = v.position.copy()
        for _ in range(60):
            previous = v.position.copy()
            v.position += v.velocity / 60
            ground.collide(v, previous, 1 / 60)
        self.assertAlmostEqual(start[0] - v.position[0], 1, places=5)

    def test_surface_resistance_and_off_runway_braking(self):
        ground = self.game.world.ground
        self.assertLess(ground.resistance((0, 0, 2500)), ground.resistance((450, 0, 2500)))
        self.v.position[:] = (50, 3, 2000)
        self.v.forward[:] = (0, 0, -1)
        self.v.right[:] = (1, 0, 0)
        self.v.velocity[:] = (0, 0, -10)
        for _ in range(400):
            self.c.update(1 / 120, commands(wheel_brake=True))
        self.assertIs(self.v.flight_state.status, FlightStatus.GROUNDED)
        self.assertLess(self.v.speed, 0.05)

    def test_severe_and_gentle_wall_impacts_swept_without_tunneling(self):
        """Verify severe and gentle wall impacts swept without tunneling."""
        ground = self.game.world.ground
        self.v.forward[:] = (1, 0, 0)
        self.v.right[:] = (0, 0, 1)
        for speed, expected in ((1, FlightStatus.GROUNDED), (30, FlightStatus.CRASHED)):
            self.v.position[:] = (320, 3, 3000)
            previous = self.v.position.copy()
            self.v.position[0] = 380
            self.v.velocity[:] = (speed, 0, 0)
            self.v.flight_state.status = FlightStatus.GROUNDED
            ground.collide(self.v, previous, 0.1)
            self.assertIs(self.v.flight_state.status, expected)
            self.assertLess(self.v.position[0], 344)
            self.assertEqual(self.v.velocity[0], 0)

    def test_natural_terrain_height_slope_and_cache(self):
        """Verify natural terrain height slope and cache."""
        env = self.game.world.environment
        plane = SimpleNamespace(
            compatibility=SimpleNamespace(footprints=(), clearance=0),
            compatibility_weight=lambda x, z: 1,
            height_at=lambda x, z: 0.05 * x,
        )
        ground = GroundEnvironment(env, plane)
        self.c.fighter_ground.environment = ground
        self.v.position[:] = (1000, 53, 4000)
        self.v.velocity[:] = 0
        self.c.update(0.1, commands())
        self.assertIs(self.v.flight_state.status, FlightStatus.GROUNDED)
        self.assertGreater(self.v.position[1], 50)
        self.v.forward[:] = (1, 0, 0)
        self.v.right[:] = (0, 0, 1)
        self.v.position[1] = self.c.fighter_ground.floor()
        self.v.velocity[:] = (5, 0.25, 0)
        self.c.update(0.5, commands())
        self.assertIs(self.v.flight_state.status, FlightStatus.GROUNDED)
        self.assertLessEqual(len(ground.heights), 512)
        with patch.object(ground, "normal", return_value=np.array((-0.5, 0.8660254, 0))):
            self.c.update(0.1, commands())
            self.assertIs(self.v.flight_state.status, FlightStatus.CRASHED)

    def test_off_airfield_vtol_and_robot_land_without_runway_rules(self):
        """Verify off airfield vtol and robot land without runway rules."""
        for mode in (VehicleMode.VTOL, VehicleMode.BATTLEDROID):
            if mode is VehicleMode.VTOL:
                self.game.setup_vtol_test("landing")
            else:
                self.game.setup_battledroid_test("landing")
            self.v.position[0] = -140
            self.v.position[2] = 2500
            height = self.game.world.ground.height(self.v.position)
            from game.battledroid_physics import contact_clearance as robot_clearance
            from game.vtol_ground import contact_clearance as vtol_clearance

            floor = (
                vtol_clearance(self.v) if mode is VehicleMode.VTOL else robot_clearance(self.v)
            ) + height
            self.v.position[1] = floor + 0.01
            self.v.velocity[:] = (0, -0.1, 0)
            self.v.throttle = 0
            self.c.update(0.1, commands())
            self.assertIs(self.v.flight_state.status, FlightStatus.GROUNDED)

    def test_f4_runway_reset_remains_available(self):
        self.game.reset_runway()
        self.assertEqual(tuple(self.v.position[[0, 2]]), RUNWAY.spawn_xz)
        self.assertIs(self.v.airfield.gear, GearState.DEPLOYED)


if __name__ == "__main__":
    unittest.main()
