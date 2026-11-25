"""M24 manifest validation, scene placement, and shared collision regressions."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from engine.game import Game
from game.flight_state import VehicleMode
from game.player_control import ControlMode
from game.world_loader import SceneRegistry, default_scene
from game.world_manifest import DEFAULT_WORLD, WorldError, load_manifest
from tools.validate_environment_assets import ENVIRONMENT_RUNTIME, validate_environment


class WorldManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.template = json.loads(DEFAULT_WORLD.read_text())

    def write(self, spec):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "world.json"
            path.write_text(json.dumps(spec))
            return load_manifest(path)

    def test_valid_world_and_deterministic_dependency_order(self):
        spec = copy.deepcopy(self.template)
        spec["entities"].reverse()
        scene = SceneRegistry(self.write(spec))
        self.assertEqual(scene.definition.identity, "tango_charlie")
        self.assertEqual(next(iter(scene.instances)), "hangar_01")
        self.assertEqual(tuple(scene.instances), tuple(SceneRegistry(self.write(spec)).instances))
        for name, placement in scene.instances.items():
            self.assertEqual(placement.matrix, default_scene().instances[name].matrix)

    def test_rejects_invalid_world_definitions(self):
        def entity(spec, index, **updates):
            spec["entities"][index].update(updates)

        cases = (
            ("Unsupported world schema", lambda s: s.update(schema_version=2)),
            ("Missing GLB", lambda s: s["assets"]["jeep"].update(path="missing.glb")),
            ("missing asset", lambda s: entity(s, 3, asset="absent")),
            ("Duplicate entity", lambda s: entity(s, 3, id="hangar_01")),
            ("needs 3 coordinates", lambda s: entity(s, 3, transform={"scale": [1, 2]})),
            ("missing or invalid anchor", lambda s: entity(s, 3, anchor="invalid")),
            ("Circular entity reference", lambda s: entity(s, 0, parent="repair_station_01")),
            ("Missing parent", lambda s: entity(s, 3, parent="unknown")),
            ("controlled entity", lambda s: s["player"].update(controlled_entity="hangar_01")),
            ("Unknown asset type", lambda s: s["assets"]["jeep"].update(type="unknown")),
            ("requires upright unit scale", lambda s: entity(s, 0, transform={"scale": [2, 2, 2]})),
        )
        for error, change in cases:
            with self.subTest(error=error), self.assertRaisesRegex(WorldError, error):
                spec = copy.deepcopy(self.template)
                change(spec)
                SceneRegistry(self.write(spec))

    def test_parent_anchor_then_local_rotation_translation_and_scale(self):
        spec = copy.deepcopy(self.template)
        spec["entities"][0]["transform"].update(rotation=[0, 90, 0])
        spec["entities"][-1]["transform"] = {
            "position": [2, 0, 3],
            "rotation": [0, 90, 0],
            "scale": [2, 1, 1],
        }
        shop = SceneRegistry(self.write(spec)).instances["repair_station_01"]
        np.testing.assert_allclose(shop.position, (341, 0, 2958))
        np.testing.assert_allclose(np.asarray(shop.matrix)[:3, :3], np.diag([-2, 1, -1]), atol=1e-7)
        np.testing.assert_allclose(shop.anchor("interaction")[:3, 3], (345, 0, 2958), atol=1e-6)

    def test_shared_asset_vehicles_reset_and_selected_driver_state(self):
        spec = copy.deepcopy(self.template)
        spec["entities"].append(
            {
                "id": "apron-jeep",
                "asset": "jeep",
                "initial_state": {"parking_brake": False},
                "transform": {"position": [220, 0, 3000], "rotation": [0, 45, 0]},
            }
        )
        spec["player"] = {"starting_mode": "GROUND_VEHICLE", "controlled_entity": "apron-jeep"}
        definition = self.write(spec)
        first, second = [Game(enemy_count=0, world_manifest=definition) for _ in range(2)]
        for game in (first, second):
            self.addCleanup(game.world.terrain.dataset.close)
        self.assertIs(first.player_control.mode, ControlMode.GROUND_VEHICLE)
        jeep = first.player_control.active.vehicle
        self.assertIs(jeep, first.world.live_entities["apron-jeep"])
        self.assertFalse(jeep.parking_brake)
        self.assertTrue(first.world.live_entities["hangar-jeep"].parking_brake)
        self.assertEqual(tuple(jeep.position[[0, 2]]), (220, 3000))
        self.assertAlmostEqual(jeep.heading, np.pi / 4)
        jeep.position[0] += 1
        self.assertEqual(second.world.live_entities["apron-jeep"].position[0], 220)
        self.assertEqual(first.world.scene.instances["apron-jeep"].position[0], 220)

    def test_on_foot_and_non_fighter_initial_state(self):
        spec = copy.deepcopy(self.template)
        spec["player"] = {"starting_mode": "ON_FOOT", "controlled_entity": "pilot_01"}
        spec["entities"][1]["initial_state"]["configuration"] = "VTOL"
        game = Game(enemy_count=0, world_manifest=self.write(spec))
        self.addCleanup(game.world.terrain.dataset.close)
        self.assertIs(game.player_control.mode, ControlMode.ON_FOOT)
        self.assertIs(game.player_vehicle.flight_state.mode, VehicleMode.VTOL)
        self.assertEqual(tuple(game.player_control.pilot.position), (305, 0, 3009))
        self.assertIs(game.player_control.pilot, game.world.live_entities["pilot_01"])

    def test_authored_floor_walls_workbench_and_occupancy(self):
        for kind, path in ENVIRONMENT_RUNTIME.items():
            model = validate_environment(kind, path)
            self.assertGreater(len({part.material.base_color for part in model.primitives}), 3)
        game = Game(enemy_count=0)
        self.addCleanup(game.world.terrain.dataset.close)
        world, control = game.world, game.player_control
        scene = world.scene.instances
        jeep = world.live_entities["hangar-jeep"]
        self.assertIs(control.mode, ControlMode.AIRCRAFT)
        self.assertEqual(tuple(game.player_vehicle.position[[0, 2]]), (310, 3000))
        self.assertEqual(tuple(jeep.position[[0, 2]]), (298, 2986))
        self.assertTrue(jeep.parking_brake)
        self.assertEqual(scene["hangar_01"].position, (300, 0, 3000))
        self.assertEqual(scene["repair_station_01"].position, (340, 0, 3038))
        self.assertEqual(world.environment.walkable_floors, [(255, 345, 2945, 3055)])
        boxes = [tuple(center) for center, _ in world.environment.solid_boxes]
        for center in ((344, 12, 3000), (300, 23.5, 3000), (340, 0.55, 3038)):
            self.assertEqual(sum(np.allclose(box, center, atol=1e-6, rtol=0) for box in boxes), 1)
        approach = scene["repair_station_01"].anchor("pilot_standing")[:3, 3]
        self.assertTrue(control.space.blocked(np.array((340, 0, 3038))))
        self.assertTrue(control.space.blocked(np.array((300, 0, 2946))))
        self.assertFalse(control.space.blocked(np.array((255, 0, 3000))))
        self.assertFalse(control.space.blocked(approach))
        self.assertEqual(world.ground.height(approach), 0)
        np.testing.assert_array_equal(world.ground.normal(approach), (0, 1, 0))
        self.assertTrue(control.interact(), control.message)
        aircraft_exit = control.pilot.position.copy()
        control.pilot.position[:] = jeep.anchor("Exit.L")
        self.assertTrue(control.interact(), control.message)
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)
        self.assertTrue(control.interact(), control.message)
        self.assertIs(control.mode, ControlMode.ON_FOOT)
        self.assertFalse(control.space.blocked(control.pilot.position))
        self.assertTrue(control.interact(), control.message)
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)
        self.assertTrue(control.interact(), control.message)
        control.pilot.position[:] = aircraft_exit
        self.assertTrue(control.interact(), control.message)
        self.assertIs(control.mode, ControlMode.AIRCRAFT)
