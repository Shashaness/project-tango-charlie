"""Geometric and compatibility checks for the revised Arrival layout."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from engine.asset_paths import PROJECT_ROOT
from engine.game import Game
from game.arrival import ExpandedArrival, open_campaign
from game.navigation import heading
from game.player_control import EYE_HEIGHT, UP, ControlMode, segment_box_distance_squared
from tools.map_arrival import generate


class ArrivalLayoutTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "save.json"

    def game(self, factory=open_campaign, **options):
        campaign = factory(self.path, **options)
        game = Game(campaign=campaign)
        self.addCleanup(game.world.terrain.dataset.close)
        return game

    def test_distances_yard_road_and_spawn(self):
        game = self.game()
        scene = game.world.scene

        def position(key):
            return np.asarray(scene.instances[key].position)

        crash, garage, airport = (position(k) for k in ("tc167", "service_garage", "hangar_01"))
        np.testing.assert_array_equal(game.player_vehicle.position[[0, 2]], (39.169, 4854.771))
        self.assertAlmostEqual(heading(game.player_vehicle.forward), 123, places=5)
        lower, _ = game.world.ground.body_bounds(game.player_vehicle)
        self.assertAlmostEqual(
            game.player_vehicle.position[1] + lower[1],
            game.world.ground.height(game.player_vehicle.position),
        )
        self.assertTrue(109 < np.linalg.norm(crash - garage) < 111)
        self.assertTrue(558 < np.linalg.norm(crash - airport) < 561)
        self.assertEqual(game.world.environment.city_footprint[3], 450)
        yard, apron = (
            scene.definition.airport["service_yards"][0],
            scene.definition.airport["apron"],
        )
        self.assertEqual(yard[0], (86.770, 4779.158))
        self.assertEqual(game.navigation.layout, "q12c")
        (road,) = game.world.environment.access_roads
        for endpoint, (center, size) in zip((road.points[0], road.points[-1]), (yard, apron)):
            self.assertTrue(np.all(np.abs(endpoint - center) < np.asarray(size) / 2))
        self.assertEqual(road.width, 18)
        self.assertGreater(road.length, np.linalg.norm(road.points[-1] - road.points[0]) + 15)
        elevations = [game.world.ground.height((x, 0, z)) for x, z in road.points]
        self.assertLess(max(np.abs(np.diff(elevations)) / np.sqrt(road.lengths_squared)), 0.06)
        self.assertTrue(
            all(game.world.ground.surface((x, 0, z)) == "apron" for x, z in road.points)
        )
        pilot = game.player_control.pilot
        self.assertIs(game.player_control.mode, ControlMode.ON_FOOT)
        np.testing.assert_array_equal(pilot.position, (36.537, 0, 4835.701))
        self.assertEqual(pilot.position[1], game.world.ground.height(pilot.position))
        self.assertAlmostEqual(heading(pilot.forward), 144.7, places=5)
        camera = game.pedestrian_camera
        camera.observe(pilot, game.player_control.space, 0)
        self.assertFalse(camera.first_person)
        np.testing.assert_allclose(camera.look_direction(), pilot.forward)
        self.assertLess(np.dot(camera.position - pilot.position, pilot.forward), 0)
        self.assertFalse(game.player_control.space.blocked(pilot.position))
        self.assertEqual(game.world.ground.surface(crash), "desert")
        jeep = game.world.ground_vehicles[0]
        eye, target = pilot.position + UP * EYE_HEIGHT, jeep.position + UP * 0.45
        self.assertTrue(
            any(
                segment_box_distance_squared(eye, target, lo, hi) < 1e-8
                for lo, hi in game.player_control.space.boxes(
                    np.minimum(eye, target), np.maximum(eye, target), exclude=jeep
                )
            )
        )
        self.assertFalse(game.campaign.facts)
        self.assertEqual(game.campaign.damage, game.player_vehicle.subsystem_health)

    def test_legacy_layout_and_local_transforms(self):
        legacy = self.game(ExpandedArrival, previous_layout=True)
        saved = self.path.read_bytes()
        restored = self.game()
        self.assertTrue(restored.campaign.previous_layout)
        self.assertEqual(saved, self.path.read_bytes())
        self.path.unlink()
        revised = self.game()
        self.assertFalse(revised.campaign.previous_layout)
        for game in (legacy, revised):
            matrix = np.asarray(game.world.scene.instances["service_garage"].matrix)
            jeep = game.world.scene.instances["hangar-jeep"]
            local = np.linalg.solve(matrix, (*jeep.position, 1))
            np.testing.assert_allclose(local, (16, 0, 0, 1), atol=1e-9)
            np.testing.assert_allclose(
                np.linalg.solve(matrix[:3, :3], jeep.forward), (-4, 0, 0), atol=1e-9
            )
        for name, obj in revised.campaign.objects.objects.items():
            old = legacy.campaign.objects.objects[name]
            np.testing.assert_allclose(obj.position - old.position, (0, 0, 1360))
            self.assertEqual(obj.spec["size"], old.spec["size"])
        self.assertFalse(self.game().campaign.previous_layout)

    def test_q12c_save_keeps_old_spawn(self):
        old = self.game()
        pilot = old.player_control.pilot
        pilot.position[:] = (38, 0, 4841.4)
        yaw = np.deg2rad(148)
        pilot.forward[:] = (np.sin(yaw), 0, np.cos(yaw))
        old.campaign.save(old)
        saved = self.path.read_bytes()
        restored = self.game()
        self.assertEqual(self.path.read_bytes(), saved)
        np.testing.assert_array_equal(restored.player_control.pilot.position, pilot.position)
        np.testing.assert_allclose(restored.player_control.pilot.forward, pilot.forward)

    def test_q12b_save_retains_actors_and_airport(self):
        old = self.game(ExpandedArrival, layout_revision="q12b")
        old.player_control.pilot.position[0] += 2
        old.world.ground_vehicles[0].position[2] += 1
        old.campaign.save(old)
        previous_bytes = self.path.read_bytes()
        restored = self.game()
        self.assertEqual(restored.navigation.layout, "q12b")
        self.assertEqual(previous_bytes, self.path.read_bytes())
        np.testing.assert_array_equal(old.player_vehicle.position, restored.player_vehicle.position)
        np.testing.assert_array_equal(
            old.player_control.pilot.position, restored.player_control.pilot.position
        )
        np.testing.assert_array_equal(
            old.world.ground_vehicles[0].position, restored.world.ground_vehicles[0].position
        )
        self.path.unlink()
        new = self.game()
        for key in (
            "service_garage",
            "service_shed",
            "service_store",
            "service_workshop",
            "hangar-jeep",
        ):
            prior, current = (np.asarray(g.world.scene.instances[key].matrix) for g in (old, new))
            np.testing.assert_array_equal(current[:3, :3], prior[:3, :3])
            np.testing.assert_allclose(current[:3, 3] - prior[:3, 3], (-63.23, 0, 48.158))
        for key in ("hangar_01", "hangar_b", "repair_station_01"):
            self.assertEqual(
                old.world.scene.instances[key].matrix, new.world.scene.instances[key].matrix
            )
        for key in ("runway", "taxiways", "apron", "apron_link"):
            self.assertEqual(
                old.world.scene.definition.airport[key], new.world.scene.definition.airport[key]
            )
        for key in old.campaign.objects.objects:
            np.testing.assert_array_equal(
                old.campaign.objects.objects[key].position,
                new.campaign.objects.objects[key].position,
            )

    def test_map_is_current(self):
        chart = PROJECT_ROOT / "docs/images/arrival_world_layout.svg"
        self.assertEqual(chart.read_text(), generate())
