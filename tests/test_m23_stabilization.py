"""M23.1 regressions for exit placement and the actual skinned pilot asset."""

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import test_ground_vehicle as fixtures

from engine.asset_paths import asset_path
from engine.pilot_asset import PilotAsset
from engine.renderer import Renderer
from game.flight_state import VehicleMode
from game.player_control import ControlMode
from tools.validate_ground_assets import hinge_angle


def mesh(asset, seated=False):
    return np.concatenate([part[:, :3] for part, _ in asset.vertices(seated=seated)])


class PilotPoseTests(unittest.TestCase):
    def setUp(self):
        self.asset = PilotAsset(asset_path("models/characters/pilot.glb"))

    def test_renderer_seating_and_camera_never_contaminate_locomotion(self):
        renderer = Renderer(None, None, None)
        renderer.pilot_asset = self.asset
        renderer._draw_pilot_mesh = Mock()
        jeep = SimpleNamespace(model_matrix=lambda: np.eye(4), anchor=lambda _: np.zeros(3))
        pilot = SimpleNamespace(
            position=np.zeros(3),
            forward=np.array((0, 0, 1)),
            elapsed=0.0,
            clip="Idle",
            pose_revision=0,
        )
        self.asset.update("Idle", 0)
        idle = mesh(self.asset)
        seated = mesh(self.asset, seated=True)
        self.assertGreater(np.max(np.abs(idle - seated)), 0.45)
        for index in range(12):
            if index != 11:  # A control transition can also happen between draws.
                renderer._draw_driver(jeep)
                self.assertTrue(renderer._draw_pilot_mesh.call_args.kwargs["seated"])
            pilot.pose_revision += 1
            np.testing.assert_array_equal(mesh(self.asset, seated=True), seated)
            pilot.clip = "Idle"
            renderer._draw_pilot(pilot)
            np.testing.assert_array_equal(mesh(self.asset), idle)
            pilot.clip = "Walk"
            pilot.elapsed += 0.55
            renderer._draw_pilot(pilot, visible=False)
            hidden = mesh(self.asset)
            renderer._draw_pilot(pilot)
            np.testing.assert_array_equal(mesh(self.asset), hidden)
            self.assertGreater(np.max(np.abs(hidden - idle)), 0.10)
            self.assertGreater(np.max(np.abs(hidden - seated)), 0.25)

    def test_anatomical_knee_direction_limits_and_alternating_swing(self):
        model = self.asset.model
        self.assertEqual(len(self.asset.joints), 19)
        self.assertEqual(set(model.animations), {"Idle", "Walk", "Run", "Turn"})
        shins = [model.find_node("Shin." + side) for side in ("L", "R")]
        feet = [model.find_node("Foot." + side) for side in ("L", "R")]
        for side, shin, foot in zip(("L", "R"), shins, feet):
            self.assertEqual(shin.parent.name, "Thigh." + side)
            self.assertIs(foot.parent, shin)
            direction = foot.world_matrix[:3, 3] - shin.world_matrix[:3, 3]
            self.assertLess(direction[1], -0.32)
            self.assertGreater(
                direction @ shin.world_matrix[:3, 1] / np.linalg.norm(direction), 0.995
            )
        for name, clip in model.animations.items():
            self.assertGreater(clip.duration, 0)
            self.asset.reset_locomotion()
            self.asset.player.play(name)
            angles, heights = [], []
            for phase in np.linspace(0, 1, 65):
                self.asset.player.time = clip.duration * phase
                self.asset.player.apply()
                local = [shin.local_matrix for shin in shins]
                angles.append(
                    [hinge_angle(shin, self.asset.hinge_axes[shin.name]) for shin in shins]
                )
                heights.append([foot.world_matrix[1, 3] for foot in feet])
                self.assertTrue(np.isfinite(local).all())
            angles = np.asarray(angles)
            self.assertGreaterEqual(angles.min(), -1e-6, name)
            self.assertLess(angles.max(), math.radians(90), name)
            if name in ("Walk", "Run"):
                np.testing.assert_allclose(angles[:33, 0], angles[32:, 1], atol=1e-6)
                np.testing.assert_allclose(angles[:33, 1], angles[32:, 0], atol=1e-6)
                self.assertGreater(angles[48, 0] - angles[48, 1], 0.55)
                self.assertGreater(heights[48][0] - heights[48][1], 0.07)
            else:
                self.assertLess(angles.max(), 0.20)


class JeepExitStabilizationTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.OccupancyTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.game = self.fixture.game
        self.jeep = self.fixture.jeep
        self.control = self.fixture.control

    def board_jeep(self):
        self.fixture.press_enter()
        self.control.pilot.position[:] = self.jeep.anchor("Exit.L")
        self.control.pilot.position[1] = 0
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.GROUND_VEHICLE)

    def test_diagonal_hangar_exit_and_reentry(self):
        self.board_jeep()
        jeep, control = self.jeep, self.control
        jeep.heading = -math.pi / 4
        pilot = control.active.occupant
        self.assertTrue(control.interact(), control.message)
        np.testing.assert_allclose(pilot.position, jeep.anchor("Exit.R"), atol=0.001)
        lo, hi = jeep.bounds()
        self.assertTrue(np.all(pilot.position[[0, 2]] > lo[[0, 2]]))
        self.assertTrue(np.all(pilot.position[[0, 2]] < hi[[0, 2]]))
        self.assertFalse(control.space.blocked(pilot.position))
        self.assertTrue(control.space.blocked(jeep.position - (0, jeep.position[1], 0)))
        self.assertTrue(control.interact(), control.message)
        self.assertIs(control.active.occupant, pilot)

    def test_hangar_wall_blocks_one_door_but_allows_the_other(self):
        self.board_jeep()
        jeep, control = self.jeep, self.control
        jeep.position[[0, 2]] = (320, 2948.45)
        self.assertTrue(control.space.blocked(jeep.anchor("Exit.R")))
        self.assertTrue(control.interact(), control.message)
        self.assertGreater(control.pilot.position[2], jeep.position[2])
        self.assertAlmostEqual(control.pilot.position[1], 0)

    def test_solid_obstacles_can_block_both_routes(self):
        self.board_jeep()
        jeep, control = self.jeep, self.control
        obstacles = [
            ((jeep.position[0], 1.5, jeep.position[2] + side * 1.65), (5, 3, 0.15))
            for side in (-1, 1)
        ]
        control.space.ground = fixtures.ground(obstacles=obstacles)
        self.assertFalse(control.interact())
        self.assertEqual(control.message, "No safe exit beside the vehicle")
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)

    def test_diagonal_exit_on_apron_and_level_natural_ground(self):
        self.board_jeep()
        jeep, control = self.jeep, self.control
        jeep.heading = -math.pi / 4
        for x, z, kind in ((240, 2960, "apron"), (-160, 2600, "desert")):
            with self.subTest(surface=kind):
                jeep.position[[0, 2]] = (x, z)
                jeep.place(control.space.ground)
                self.assertEqual(control.space.ground.surface(jeep.position), kind)
                self.assertTrue(control.interact(), control.message)
                self.assertAlmostEqual(control.pilot.position[1], 0)
                self.assertFalse(control.space.blocked(control.pilot.position))
                self.assertTrue(control.interact(), control.message)

    def test_moderate_cross_slope_and_one_unloaded_wheel(self):
        self.board_jeep()
        jeep = self.jeep
        jeep.position[:] = (100, 0, 2900)
        jeep.roll = math.atan(0.48)

        def terrain(point):
            grade = 0.48 * (point[2] - 2900)
            # A shallow depression under a front wheel leaves three loaded corners.
            if point[0] < 99 and 2900.6 < point[2] < 2901:
                grade -= 0.35
            return grade

        surface = fixtures.ground(terrain=terrain)
        surface.normal = lambda _: np.array((0, 1.0, -0.48)) / math.hypot(1, 0.48)
        self.control.space.ground = surface
        jeep.place(surface)
        jeep.contacts[:2] = False  # Stale flags must not override live support geometry.
        _, compression = jeep._suspension(jeep._heights(surface))
        self.assertEqual(np.count_nonzero(compression > 0), 3)
        self.assertTrue(self.control.interact(), self.control.message)
        pilot = self.control.pilot
        self.assertAlmostEqual(pilot.position[1], terrain(pilot.position))
        self.assertFalse(self.control.space.blocked(pilot.position))

    def test_airborne_moving_steep_and_drop_are_rejected(self):
        self.board_jeep()
        space, jeep = self.control.space, self.jeep
        jeep.position[:] = (100, 0, 2900)
        surface = fixtures.ground()
        space.ground = surface
        jeep.place(surface)
        access = self.control.active
        jeep.position[1] += 1
        self.assertFalse(self.control.interact())
        jeep.position[1] -= 1
        jeep.vertical_speed = -1
        self.assertFalse(self.control.interact())
        jeep.vertical_speed = 0
        jeep.speed = 1
        self.assertFalse(self.control.interact())
        jeep.speed = 0
        jeep.roll = math.radians(45)
        self.assertFalse(self.control.interact())
        jeep.roll = 0
        # A high supported ridge ends before either door.
        surface.height = lambda pos: 0 if abs(pos[2] - 2900) <= 0.9 else -2
        self.assertFalse(self.control.interact())
        self.assertFalse(access.exit_position(space))

    def test_lifecycle_on_actual_natural_terrain(self):
        def tick():
            self.game.update(1 / 15)

        def drive_toward(target_x, limit):
            controls = self.game.input
            controls.pilot_advance = 1 if target_x < self.jeep.position[0] else -1
            self.jeep.parking_brake = False
            for _ in range(limit):
                tick()
                if (target_x - self.jeep.position[0]) * controls.pilot_advance >= 0:
                    break
            controls.pilot_advance = 0
            controls.fire_gun = True
            for _ in range(36):
                tick()
            controls.fire_gun = False

        aircraft, jeep = self.game.player_vehicle, self.jeep
        aircraft_position = aircraft.position.copy()
        self.fixture.press_enter()
        pilot = self.control.pilot
        aircraft_door = pilot.position.copy()
        self.fixture.walk(jeep.anchor("Exit.L"))
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.GROUND_VEHICLE)
        drive_toward(-385, 900)
        self.assertLess(jeep.position[0], -385)
        self.assertEqual(self.game.world.ground.surface(jeep.position), "desert")
        self.assertLess(self.game.world.ground.normal(jeep.position)[1], 0.999)
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.ON_FOOT, self.control.message)
        self.assertIs(self.control.pilot, pilot)
        self.assertEqual(pilot.clip, "Idle")
        self.assertAlmostEqual(pilot.position[1], self.control.space.terrain(pilot.position))
        self.assertFalse(self.control.space.blocked(pilot.position))
        camera = self.game.pedestrian_camera
        camera.orbit_yaw = jeep.heading
        self.game.input.pilot_advance = 1
        for running, clip in ((False, "Walk"), (True, "Run")):
            self.game.input.pilot_run = running
            for _ in range(3):
                self.game.update(1 / 30)
            self.assertEqual(pilot.clip, clip)
        self.game.input.pilot_advance = 0
        self.game.input.pilot_run = False
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.GROUND_VEHICLE, self.control.message)
        drive_toward(296, 2200)
        self.assertGreater(jeep.position[0], 296)
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.ON_FOOT, self.control.message)
        parked = jeep.position.copy()
        if pilot.position[2] < jeep.position[2]:
            self.fixture.walk(parked + (-3.5, 0, 0))
            self.fixture.walk(parked + (-3.5, 0, 3))
        self.fixture.walk(aircraft_door)
        self.fixture.press_enter()
        self.assertIs(self.control.mode, ControlMode.AIRCRAFT)
        self.assertIs(self.game.player_vehicle, aircraft)
        np.testing.assert_array_equal(aircraft.position, aircraft_position)
        self.assertIs(aircraft.transformation.configuration, VehicleMode.FIGHTER)
        self.game.input.toggle_configuration = True
        self.game.update(1 / 60)
        self.assertTrue(aircraft.transformation.active)
