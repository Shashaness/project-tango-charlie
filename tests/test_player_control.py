"""Acceptance coverage for the parked TC167 and on-foot world lifecycle."""

import unittest
from types import SimpleNamespace

import numpy as np

from engine.asset_paths import asset_path
from engine.camera import ORBIT_SENSITIVITY
from engine.game import Game
from engine.pedestrian_camera import PedestrianCamera
from engine.pilot_asset import PilotAsset
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.ground_environment import GroundEnvironment
from game.player_control import EYE_HEIGHT, RADIUS, ControlMode, PedestrianSpace, Pilot

ZERO = np.zeros(3)
WEST = np.array((-1.0, 0.0, 0.0))
EAST = -WEST


class PlayerControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.game = Game(enemy_count=0)

    @classmethod
    def tearDownClass(cls):
        cls.game.world.terrain.dataset.close()

    def setUp(self):
        self.game.reset_runway()
        self.game.spawn_hangar()
        self.vehicle = self.game.player_vehicle
        self.control = self.game.player_control
        self.control.mode = ControlMode.VEHICLE
        self.control.pilot = None
        self.control.message = ""
        self.control.space.ground = self.game.world.ground
        self.control.space.parked_bounds = None

    def test_exit_eligibility(self):
        self.vehicle.flight_state.status = FlightStatus.FLYING
        self.assertFalse(self.control.interact())
        self.assertIn("Land", self.control.message)
        self.vehicle.flight_state.status = FlightStatus.GROUNDED
        self.vehicle.velocity[:] = (1, 0, 0)
        self.assertFalse(self.control.interact())
        self.assertIn("Stop", self.control.message)
        self.vehicle.velocity[:] = 0
        self.vehicle.transformation.active = True
        self.assertFalse(self.control.interact())
        self.assertIn("transformation", self.control.message)
        self.vehicle.transformation.active = False
        self.vehicle.position[1] += 4
        self.assertFalse(self.control.interact())
        self.assertIn("supported", self.control.message)
        self.vehicle.position[1] -= 4
        self.vehicle.flight_state.status = FlightStatus.CRASHED
        self.assertFalse(self.control.interact())
        self.assertIn("unsafe", self.control.message)
        self.vehicle.flight_state.status = FlightStatus.GROUNDED
        self.assertTrue(self.control.interact())
        self.assertIs(self.control.mode, ControlMode.ON_FOOT)

    def test_no_safe_exit_and_alternative(self):
        position = self.vehicle.position.copy()
        preferred, _ = self.control.exit_position()
        self.control.space.ground = ground_with(
            self.game.world.ground,
            [
                (preferred, (1, 4, 1)),
            ],
        )
        other, _ = self.control.exit_position()
        self.assertGreater(np.linalg.norm(other - preferred), 1)
        self.control.space.ground = ground_with(
            self.game.world.ground,
            [
                (position, (38, 12, 38)),
            ],
        )
        self.assertFalse(self.control.interact())
        self.assertIn("No safe exit", self.control.message)
        self.assertIs(self.control.mode, ControlMode.VEHICLE)

    def test_game_loop_walks_out_and_returns_to_same_vehicle(self):
        v = self.vehicle
        snapshot = (v.position.copy(), v.orientation.copy(), v.transformation.action)
        self.game.input.interact_pressed = True
        self.game.update(1 / 60)
        self.game.input.interact_pressed = False
        pilot = self.control.pilot
        exit_pos = pilot.position.copy()

        # Walk alongside the tail through the existing west doorway and onto the apron.
        pilot.forward[:] = WEST
        self.game.input.pilot_advance = 1
        self.game.input.pilot_run = True
        self.game.input.toggle_configuration = self.game.input.space_pressed = True
        self.game.input.thrust = 1
        for _ in range(600):
            self.game.update(1 / 60)
        self.assertLess(pilot.position[0], 252)
        self.assertFalse(self.control.interact())
        self.assertIn("Stand close", self.control.message)
        self.assertIs(self.vehicle.flight_state.environment, Environment.ATMOSPHERE)
        self.assertAlmostEqual(v.throttle, 0)
        np.testing.assert_array_equal(v.position, snapshot[0])
        np.testing.assert_array_equal(v.orientation, snapshot[1])
        self.assertEqual(v.transformation.action, snapshot[2])

        self.game.input.pilot_advance = -1
        for _ in range(600):
            self.game.update(1 / 60)
        self.assertLess(np.linalg.norm(pilot.position - exit_pos), 1)
        self.game.input.interact_pressed = True
        self.game.update(1 / 60)
        self.assertIs(self.control.mode, ControlMode.VEHICLE)
        self.assertIs(self.game.player_vehicle, v)
        np.testing.assert_array_equal(v.position, snapshot[0])
        np.testing.assert_array_equal(v.orientation, snapshot[1])
        self.assertEqual(v.transformation.action, snapshot[2])

    def test_runway_exit_and_reentry_eligibility(self):
        self.game.reset_runway()
        self.assertTrue(self.control.interact())
        self.control.pilot.grounded = False
        self.assertFalse(self.control.interact())
        self.control.pilot.grounded = True
        self.vehicle.flight_state.status = FlightStatus.CRASHED
        self.assertFalse(self.control.interact())
        self.vehicle.flight_state.status = FlightStatus.GROUNDED
        self.assertTrue(self.control.interact())

    def test_vtol_and_battledroid_use_their_landed_configuration(self):
        for mode, setup in (
            (VehicleMode.VTOL, self.game.setup_vtol_test),
            (VehicleMode.BATTLEDROID, self.game.setup_battledroid_test),
        ):
            with self.subTest(mode=mode):
                setup("ground")
                self.vehicle.position[0] = -20
                self.vehicle.position[2] = 2500
                self.assertTrue(self.control.interact(), self.control.message)
                self.assertFalse(self.control.space.blocked(self.control.pilot.position))
                self.assertTrue(self.control.interact(), self.control.message)
                self.assertIs(self.vehicle.flight_state.mode, mode)
                self.assertIs(self.vehicle.transformation.configuration, mode)

    def test_hangar_wall_and_vehicle_stop_capsule(self):
        self.assertTrue(self.control.interact())
        pilot = self.control.pilot
        start = pilot.position.copy()
        center = self.vehicle.position.copy()
        direction = center - start
        direction[1] = 0
        direction /= np.linalg.norm(direction)
        pilot.update(2, self.control.space, direction, run=True)
        self.assertGreater(np.linalg.norm(pilot.position - center), 5)
        self.assertFalse(self.control.space.blocked(pilot.position))
        pilot.update(12, self.control.space, -direction, run=True)
        self.assertLess(abs(pilot.position[2] - 3000), 54 - RADIUS)
        self.assertFalse(self.control.space.blocked(pilot.position))
        self.assertAlmostEqual(pilot.position[1], 0)

    def test_jump_ground_slope_step_and_timestep(self):
        ground = ground_with(self.game.world.ground, [((2.0, 0.10, 2900), (1.0, 0.2, 2))])
        space = PedestrianSpace(ground, self.vehicle)
        pilot = Pilot((0, 0, 2900), EAST)
        pilot.update(0.65, space, EAST)
        self.assertGreater(pilot.position[0], 1.5)
        self.assertAlmostEqual(pilot.position[1], 0.2)
        pilot.update(0.22, space, ZERO, jump=True)
        self.assertGreater(pilot.position[1], 0.8)
        self.assertFalse(pilot.grounded)
        pilot.update(1.1, space, ZERO)
        self.assertTrue(pilot.grounded)
        self.assertAlmostEqual(pilot.position[1], 0.2)

        ground.height = lambda p: p[0] * 2.2
        ground.normal = lambda _: np.array((-0.91, 0.41, 0))
        steep = Pilot((0, 0.36, 2900), EAST)
        steep.update(1, space, EAST)
        self.assertLess(steep.position[0], 0.1)
        ground.height = lambda p: p[0] * 0.35
        ground.normal = lambda _: np.array((-0.33, 0.94, 0))
        for dt, count in ((1 / 30, 60), (1 / 144, 288)):
            current = Pilot((-5, -1.75, 2910), EAST)
            for _ in range(count):
                current.update(dt, space, EAST, run=True)
            self.assertAlmostEqual(current.position[1], ground.height(current.position), places=2)
            if count == 60:
                baseline = current.position.copy()
            else:
                np.testing.assert_allclose(current.position, baseline, atol=1e-5)

    def test_camera_orbit_zoom_and_obstruction(self):
        self.assertTrue(self.control.interact())
        pilot, space = self.control.pilot, self.control.space
        camera = PedestrianCamera()
        camera.begin(pilot)
        before = camera.look_direction().copy()
        camera.orbit_drag(70, 10)
        self.assertGreater(np.linalg.norm(camera.look_direction() - before), 0.1)
        camera.orbit_zoom(50)
        camera.observe(pilot, space, 0.1)
        self.assertFalse(camera.first_person)
        camera.observe(pilot, space, 0.5)
        self.assertTrue(camera.first_person)
        np.testing.assert_allclose(camera.position, pilot.position + (0, EYE_HEIGHT, 0), atol=0.01)
        pilot.position[:] = (290, 0, 2980)
        camera.orbit_zoom(-50)
        camera.observe(pilot, space, 1)
        self.assertFalse(camera.first_person)
        camera.orbit_yaw = np.pi / 2
        pilot.position[:] = (292, 0, 2980)
        ground = ground_with(
            self.game.world.ground,
            [
                ((290, 2, 2980), (0.2, 4, 8)),
            ],
        )
        camera.observe(pilot, PedestrianSpace(ground, self.vehicle), 1)
        self.assertGreater(camera.position[0], 290.2)

    def on_foot_trial(self, heading=(0, 0, 1)):
        pilot = self.control.pilot = Pilot((1000, 0, 3000), heading)
        ground = ground_with(self.game.world.ground, [])
        ground.height = lambda _: 0
        self.control.space.ground = ground
        self.control.mode = ControlMode.ON_FOOT
        self.game.pedestrian_camera.begin(pilot)
        self.game.input.pilot_jump = False
        self.game.input.interact_pressed = False
        return pilot

    def test_forward_walk_and_run_are_straight_through_a_full_mouse_orbit(self):
        game = self.game
        camera = game.pedestrian_camera
        steps = 60
        try:
            for running, clip, speed in ((False, "Walk", 3), (True, "Run", 6)):
                with self.subTest(clip=clip):
                    paths = []
                    for orbit in (False, True):
                        pilot = self.on_foot_trial()
                        game.input.pilot_advance = 1
                        game.input.pilot_strafe = 0
                        game.input.pilot_run = running
                        frames = []
                        for _ in range(steps):
                            if orbit:
                                game.camera_input.drag = (np.pi * 2 / steps / ORBIT_SENSITIVITY, 0)
                            game.update(1 / 60)
                            frames.append(np.r_[pilot.position, pilot.forward, pilot.velocity])
                        paths.append(frames)
                        self.assertEqual(pilot.clip, clip)
                        self.assertAlmostEqual(pilot.velocity[2], speed, places=5)
                    np.testing.assert_array_equal(*paths)
                    self.assertAlmostEqual(camera.orbit_yaw, -2 * np.pi)
        finally:
            game.input.pilot_advance = 0
            game.input.pilot_run = False

    def test_wasd_uses_avatar_heading_at_all_camera_quarter_turns(self):
        game = self.game
        cases = (
            ((0, 1), WEST),
            ((0, -1), EAST),
            ((1, 0), (0, 0, -1)),
            ((-1, 0), (0, 0, 1)),
            ((1, 1), (-1 / np.sqrt(2), 0, -1 / np.sqrt(2))),
        )
        try:
            for offset in np.arange(4) * np.pi / 2:
                for keys, expected in cases:
                    with self.subTest(offset=offset, keys=keys):
                        pilot = self.on_foot_trial(WEST)
                        start = pilot.position.copy()
                        game.pedestrian_camera.orbit_yaw += offset
                        game.input.pilot_strafe, game.input.pilot_advance = keys
                        for _ in range(36):
                            game.update(1 / 60)
                        delta = pilot.position - start
                        np.testing.assert_allclose(
                            delta / np.linalg.norm(delta), expected, atol=1e-10
                        )
                        np.testing.assert_allclose(pilot.forward, expected, atol=1e-10)
        finally:
            game.input.pilot_strafe = game.input.pilot_advance = 0

    def test_mouse_orbit_while_idle_leaves_avatar_pose_unchanged(self):
        game = self.game
        pilot = self.on_foot_trial(WEST)
        game.input.pilot_strafe = game.input.pilot_advance = 0
        start = pilot.position.copy()
        camera = game.pedestrian_camera
        initial = camera.orbit_yaw, camera.orbit_pitch
        game.camera_input.drag = (120, 65)
        game.update(1 / 30)
        self.assertNotEqual((camera.orbit_yaw, camera.orbit_pitch), initial)
        np.testing.assert_array_equal(pilot.position, start)
        np.testing.assert_array_equal(pilot.forward, WEST)
        np.testing.assert_array_equal(pilot.velocity, ZERO)
        self.assertEqual(pilot.clip, "Idle")

    def test_original_skinned_pilot_animates(self):
        asset = PilotAsset(asset_path("models/characters/pilot.glb"))
        self.assertEqual(set(asset.model.animations), {"Idle", "Walk", "Run", "Turn"})
        self.assertEqual(asset.model.find_node("Shin.L").parent.name, "Thigh.L")
        bind = np.concatenate([part for part, _ in asset.vertices()])
        asset.update("Walk", 0.2)
        walking = np.concatenate([part for part, _ in asset.vertices()])
        self.assertGreater(np.max(np.abs(walking - bind)), 0.1)
        self.assertTrue(np.isfinite(walking).all())


def ground_with(source, boxes):
    """Real spatial acceleration and hull evaluation on a deterministic site."""
    world = SimpleNamespace(solid_boxes=boxes, buildings=[])
    ground = GroundEnvironment(world, None)
    ground.height = source.height
    ground.normal = source.normal
    return ground
