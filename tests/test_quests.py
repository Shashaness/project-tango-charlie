"""Quest definition, lifecycle, campaign save, and damaged-aircraft coverage."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from engine.game import Game
from game.campaign import DAMAGE, ArrivalCampaign
from game.player_control import ControlMode
from game.quests.definitions import load_quests
from game.quests.manager import Event, QuestManager, Status


class QuestTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name) / "registry.json"
        self.quests = [
            {
                "id": "first",
                "title": "First",
                "description": "Recover access",
                "owner": "kara",
                "classification": "main",
                "activation": "started",
                "completion": "confirmed",
                "journal": {"initial": "Begin", "complete": "Done"},
                "objectives": [
                    {
                        "id": "reach",
                        "title": "Reach",
                        "type": "reach",
                        "target": "base",
                        "count": 2,
                    },
                    {"id": "inspect", "title": "Inspect", "type": "inspect", "target": "base"},
                    {
                        "id": "option",
                        "title": "Look",
                        "type": "discover",
                        "target": "base",
                        "optional": True,
                    },
                    {
                        "id": "finish",
                        "title": "Finish",
                        "type": "interact",
                        "target": "base",
                        "stage": 1,
                    },
                ],
            },
            {
                "id": "next",
                "title": "Next",
                "description": "Follow up",
                "owner": "kara",
                "classification": "secondary",
                "prerequisites": ["first"],
                "objectives": [
                    {"id": "report", "title": "Report", "type": "interact", "target": "base"}
                ],
            },
        ]

    def load(self):
        self.path.write_text(json.dumps({"schema_version": 1, "quests": self.quests}))
        return load_quests(self.path, {"base", "kara", "other"})

    def test_validation(self):
        self.assertEqual(set(self.load()), {"first", "next"})
        self.quests.append(self.quests[0])
        with self.assertRaisesRegex(ValueError, "Duplicate quest"):
            self.load()
        self.quests.pop()
        self.quests[1]["prerequisites"] = ["unknown"]
        with self.assertRaisesRegex(ValueError, "Unknown prerequisite"):
            self.load()
        self.quests[1]["prerequisites"] = ["first"]
        self.quests[0]["prerequisites"] = ["next"]
        with self.assertRaisesRegex(ValueError, "Circular"):
            self.load()
        self.quests[0].pop("prerequisites")
        self.quests[0]["entities"] = ["unknown"]
        with self.assertRaisesRegex(ValueError, "world entity"):
            self.load()

    def test_stages_owner_completion_and_replay(self):
        manager = QuestManager(self.load())

        def event(kind, owner="kara"):
            return Event(kind, owner, "base", (0, 0, 0), 1)

        self.assertEqual(manager.status, {"first": Status.AVAILABLE, "next": Status.LOCKED})
        manager.set_condition("started")
        early = event("OBJECT_INTERACTED")
        self.assertFalse(manager.accept(early))
        self.assertFalse(manager.accept(event("LOCATION_REACHED", "other")))
        self.assertTrue(manager.accept(event("ENTITY_INSPECTED")))
        reach = event("LOCATION_REACHED")
        self.assertTrue(manager.accept(reach))
        self.assertFalse(manager.accept(reach))
        self.assertTrue(manager.accept(event("LOCATION_REACHED")))
        self.assertFalse(manager.accept(early))
        manager.accept(event("OBJECT_INTERACTED"))
        self.assertEqual(manager.status["first"], Status.ACTIVE)
        manager.set_condition("confirmed")
        self.assertEqual(manager.status["first"], Status.COMPLETED)
        self.assertEqual(manager.status["next"], Status.ACTIVE)
        self.assertEqual(manager.journal, [("first", "initial"), ("first", "complete")])
        restored = QuestManager(manager.definitions)
        snapshot = manager.snapshot()
        restored.restore(snapshot)
        self.assertEqual(restored.snapshot(), snapshot)
        restored.fail("next")
        self.assertEqual(restored.status["next"], Status.FAILED)
        self.assertFalse(restored.accept(event("OBJECT_INTERACTED")))


class ArrivalTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "arrival.json"

    def create(self, **options):
        campaign = ArrivalCampaign(self.path, **options)
        game = Game(campaign=campaign)
        self.addCleanup(game.world.terrain.dataset.close)
        return game, campaign

    def test_initial_damage_and_resets(self):
        game, campaign = self.create()
        control = game.player_control
        vehicle = game.player_vehicle
        original = vehicle.position.copy()
        self.assertIs(control.mode, ControlMode.ON_FOOT)
        self.assertEqual(vehicle.subsystem_health, DAMAGE)
        self.assertEqual(len(game.world.enemies), 0)
        self.assertIsNotNone(control.aircraft.entry_warning(control.space))
        self.assertFalse(control.space.blocked(control.pilot.position))
        hangar = game.world.scene.instances["hangar_01"].position
        self.assertTrue(400 <= np.linalg.norm((hangar - original)[[0, 2]]) <= 600)
        for button in (
            "space_pressed",
            "atmosphere_pressed",
            "runway_pressed",
            "toggle_configuration",
            "reset_pressed",
        ):
            setattr(game.input, button, True)
            game.update(1 / 60)
            setattr(game.input, button, False)
            np.testing.assert_array_equal(vehicle.position, original)
            self.assertEqual(vehicle.flight_state.mode.name, "FIGHTER")
        game.reset_runway()
        game.spawn_hangar()
        game.setup_battledroid_test("ground")
        game.setup_vtol_test("ground")
        game.setup_defense_test(())
        np.testing.assert_array_equal(vehicle.position, original)
        self.assertEqual(campaign.manager.progress["Q001"]["airbase"], 0)

    def test_arrival_and_occupancy_resume(self):
        game, campaign = self.create()
        control = game.player_control
        jeep = game.world.ground_vehicles[0]
        control.pilot.position[:] = jeep.anchor("Exit.L")
        game.input.interact_pressed = True
        game.update(1 / 60)
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)
        self.assertEqual(campaign.manager.progress["Q001"]["airbase"], 1)
        self.assertEqual(campaign.manager.status["Q001"], Status.ACTIVE)
        resumed, restored = self.create()
        self.assertEqual(restored.manager.snapshot(), campaign.manager.snapshot())
        self.assertEqual(restored.damage, DAMAGE)
        self.assertEqual(resumed.player_control.active.identity, jeep.identity)
        np.testing.assert_allclose(resumed.player_control.active.vehicle.position, jeep.position)
        self.assertEqual(len(restored.manager.journal), 1)
        _, fresh = self.create(new=True)
        self.assertEqual(fresh.manager.progress["Q001"]["airbase"], 0)

    def test_invalid_save_is_preserved(self):
        self.path.write_text('{"schema_version": 200}')
        original = self.path.read_bytes()
        with self.assertRaises(ValueError):
            ArrivalCampaign(self.path)
        self.assertEqual(original, self.path.read_bytes())
        self.path.write_text("{bad")
        with self.assertRaises(ValueError):
            ArrivalCampaign(self.path)

    def test_default_sandbox_is_unchanged(self):
        game = Game(enemy_count=0)
        self.addCleanup(game.world.terrain.dataset.close)
        self.assertIsNone(game.campaign)
        self.assertIs(game.player_control.mode, ControlMode.AIRCRAFT)
        self.assertEqual(game.world.scene.definition.identity, "tango_charlie")
        self.assertTrue(game.player_control.interact())
        self.assertFalse(game.player_control.aircraft.entry_warning(game.player_control.space))
        self.assertFalse(self.path.exists())

    def test_exploration_discovery_inspection_and_driving_resume(self):
        game, campaign = self.create()
        control = game.player_control

        def stand(x, z):
            control.pilot.position[:] = (x, 0, z)
            control.pilot.velocity[:] = 0
            game.update(0)

        def reload():
            nonlocal game, campaign, control
            game, campaign = self.create()
            control = game.player_control

        reload()  # before reaching the airbase
        self.assertEqual(campaign.facts, {})
        game.input.inspect_pressed = True
        game.update(0)
        game.input.inspect_pressed = False
        self.assertNotIn("inspect", campaign.facts)

        stand(300, 3076)  # Service garage before the main hangar.
        self.assertIn("discover", campaign.facts)
        self.assertEqual(campaign.manager.progress["Q001"]["discover"], 0)
        self.assertIn("A utility vehicle.", campaign._journal_entries()[-1])
        reload()
        self.assertIn("discover", campaign.facts)
        self.assertNotIn("inspect", campaign.facts)

        stand(304, 3078)
        game.input.inspect_pressed = True
        game.update(0)
        game.update(0)  # repeat the interaction while the key is held in the test
        game.input.inspect_pressed = False
        self.assertIn("inspect", campaign.facts)
        self.assertEqual(sum("better condition" in j for j in campaign._journal_entries()), 1)
        reload()
        self.assertIn("inspect", campaign.facts)

        stand(280, 3000)
        self.assertTrue(
            all(
                campaign.manager.progress["Q001"][key]
                for key in ("airbase", "hangar", "service", "discover", "inspect")
            )
        )
        reload()
        self.assertIn("hangar", campaign.facts)
        stand(304, 3078)
        game.input.interact_pressed = True
        game.update(1 / 60)
        game.input.interact_pressed = False
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)
        self.assertEqual(campaign.manager.status["Q001"], Status.ACTIVE)
        reload()
        self.assertIs(control.mode, ControlMode.GROUND_VEHICLE)
        jeep = control.active.vehicle
        original = jeep.position.copy()
        game.input.pilot_advance = 1
        for _ in range(20):
            game.update(1 / 30)
        self.assertEqual(campaign.driven, 0)
        game.input.toggle_parking_brake = True
        game.update(1 / 30)
        game.input.toggle_parking_brake = False
        for _ in range(100):
            game.update(1 / 30)
            if campaign.driven >= 4:
                break
        self.assertTrue(4 <= campaign.driven < campaign.drive_distance)
        campaign.save(game)
        driven = campaign.driven
        speed = jeep.speed
        reload()
        self.assertEqual(campaign.driven, driven)
        self.assertEqual(control.active.vehicle.speed, speed)
        jeep = control.active.vehicle
        game.input.pilot_advance = 1
        for _ in range(180):
            game.update(1 / 30)
            if campaign.manager.status["Q001"] == Status.COMPLETED:
                break
        self.assertLess(jeep.position[0], original[0] - 11)
        self.assertEqual(campaign.manager.status["Q001"], Status.COMPLETED)
        self.assertEqual(campaign.manager.status["Q002"], Status.AVAILABLE)
        self.assertEqual(len(campaign._journal_entries()), 5)
        reload()
        self.assertEqual(campaign.manager.status["Q001"], Status.COMPLETED)
        self.assertEqual(len(campaign._journal_entries()), 5)
        self.assertAlmostEqual(control.active.vehicle.position[0], jeep.position[0])
        self.assertIn("AWAITING", campaign.objective_label)

    def test_hangar_first_resume_and_context_controls(self):
        game, campaign = self.create()
        game.player_control.pilot.position[:] = (275, 0, 3000)
        game.update(0)
        self.assertIn("hangar", campaign.facts)
        self.assertNotIn("discover", campaign.facts)
        game, campaign = self.create()
        self.assertIn("hangar", campaign.facts)
        self.assertNotIn("discover", campaign.facts)
        game.input.journal_pressed = True
        game.update(0)
        game.input.journal_pressed = False
        self.assertIn("JOURNAL", campaign.hud_lines()[0])
        game.input.journal_next_pressed = True
        game.update(0)
        self.assertIn("ENTRY 2/2", campaign.hud_lines())

    def test_discovery_checks_solid_wall_and_inspection_range(self):
        from game.campaign import JEEP_PROFILE

        game, campaign = self.create()
        control = game.player_control
        access = control.registry["hangar-jeep"]
        access.vehicle.position[0] = 309  # Bring the test target near the rear wall.
        control.pilot.position[:] = (311.8, 0, 3076)
        self.assertFalse(JEEP_PROFILE.visible(control.pilot, access, control.space))
        game.update(0)
        self.assertNotIn("discover", campaign.facts)
        control.pilot.position[:] = (305, 0, 3076)
        self.assertTrue(JEEP_PROFILE.visible(control.pilot, access, control.space))
        game.update(0)
        self.assertIn("discover", campaign.facts)
        self.assertFalse(campaign.inspect(control.pilot, access, control.space))
        control.pilot.position[:] = (309, 0, 3078)
        self.assertTrue(campaign.inspect(control.pilot, access, control.space))

    def test_q11_save_migration_preserves_progress_and_moved_jeep(self):
        from game.world_loader import default_scene

        _, _campaign = self.create()
        original = json.loads(self.path.read_text())
        original["schema_version"] = 1
        del original["facts"], original["driven"]
        quests = original["quests"]
        del quests["progress"]["Q001"]["board"], quests["progress"]["Q002"]
        del quests["status"]["Q002"]
        quests["progress"]["Q001"]["airbase"] = 1
        original["pilot"]["position"] = [230, 0, 3000]
        jeep = original["jeeps"]["hangar-jeep"]
        position = default_scene().instances["hangar-jeep"].position
        jeep["position"] = [float(position[0]), jeep["position"][1], float(position[2])]
        self.path.write_text(json.dumps(original))
        game, campaign = self.create()
        np.testing.assert_allclose(game.player_control.pilot.position, [230, 0, 3000])
        self.assertEqual(campaign.manager.progress["Q001"]["airbase"], 1)
        self.assertEqual(game.world.ground_vehicles[0].position[2], 3076)
        self.assertEqual(json.loads(self.path.read_text())["schema_version"], 2)

        jeep["position"][0] += 4
        original["occupied"] = "hangar-jeep"
        self.path.write_text(json.dumps(original))
        game, campaign = self.create()
        self.assertIn("board", campaign.facts)
        np.testing.assert_allclose(game.player_control.active.vehicle.position, jeep["position"])
