"""Q1.3A state, contextual actions, and expanded save compatibility."""

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from engine.game import Game
from game.arrival import open_campaign
from game.equipment import Battery, Condition, Slot
from game.player_control import ControlMode
from game.quests.manager import Status


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name) / "recovery.json"
        self.load()

    def load(self):
        self.c = open_campaign(self.path)
        self.game = Game(campaign=self.c)
        self.addCleanup(self.game.world.terrain.dataset.close)
        self.control = self.game.player_control
        self.scene = self.c.objects
        self.jeep = self.game.world.ground_vehicles[0]
        self.pilot = self.control._pilot
        self.cmd = self.game.input

    def stand(self, x, z, heading=(-1, 0, 0)):
        self.pilot.position[:] = (x, 0, z)
        self.pilot.forward[:] = heading
        self.game.update(0)

    def use(self, key, action, *, inspect=False):
        obj = self.scene.objects[key]
        self.assertTrue(
            self.scene.in_reach(obj, self.pilot, self.control.space),
            (key, self.pilot.position, self.scene._point(obj)),
        )
        self.assertEqual(self.scene.action(obj), action)
        self.c._selected_id = key
        self.cmd.pilot_run = inspect
        self.cmd.inspect_pressed = True
        self.game.update(0.01)
        self.cmd.inspect_pressed = self.cmd.pilot_run = False
        return self.c.notice_detail

    def park(self, x, z, heading):
        self.jeep.position[:] = (x, 0, z)
        self.jeep.heading = heading
        self.jeep.place(self.game.world.ground)

    def test_condition_and_charge_are_independent(self):
        original = self.scene.batteries["battery_original"]
        self.assertIs(original.condition, Condition.SERVICEABLE)
        self.assertEqual(original.charge, 0)
        self.assertFalse(self.scene.compatible("battery_original"))
        self.assertTrue(self.scene.compatible("battery_a"))
        self.assertFalse(self.scene.compatible("battery_b"))
        for condition in (Condition.FAILED, Condition.DETERIORATED, Condition.UNKNOWN):
            self.assertFalse(
                Battery(original.interface, condition, 1).supplies(original.interface, 0.25)
            )
        self.assertFalse(
            Battery("different", Condition.SERVICEABLE, 1).supplies(original.interface, 0.25)
        )
        self.assertTrue(self.scene.readiness["towable"])
        self.assertFalse(self.scene.electrically_operational)
        self.assertFalse(self.scene.readiness["ready_for_loaded_transport"])

    def test_handle_load_deliver_remove_install_and_resume(self):
        wreck = self.game.player_vehicle.position.copy()
        self.stand(312, 4470)
        self.assertIn("DEPLETED", self.use("trolley", "INSPECT"))
        self.assertIn("battery_fault", self.c.facts)
        self.park(302, 4596, math.pi)
        self.stand(303.25, 4598.55)
        self.use("battery_station", "INSPECT")
        self.assertIn("SERVICEABLE", self.use("battery_a", "LIFT", inspect=True))
        self.assertIn("battery_eligible", self.c.facts)
        self.assertNotIn("batteries_exam", self.c.facts)
        self.stand(307.35, 4598.1)
        self.assertIn("DETERIORATED", self.use("battery_b", "LIFT", inspect=True))
        self.assertIn("batteries_exam", self.c.facts)
        self.stand(303.25, 4598.55)
        self.use("battery_a", "LIFT")
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.CARRIED)
        self.assertEqual(self.scene.movement_scale, 0.5)
        self.assertNotIn("battery_a", self.scene.inventory(self.c.owner))
        self.stand(302, 4600.55)
        self.jeep.speed = 1
        self.assertIn("STOP THE JEEP", self.use("jeep_cargo_bay", "LOAD BATTERY"))
        self.jeep.speed = 0
        self.use("jeep_cargo_bay", "LOAD BATTERY")
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.VEHICLE)
        offset = self.scene.settings["jeep_load"]
        np.testing.assert_allclose(
            self.scene.transform("battery_a")[:3, 3], (self.jeep.model_matrix() @ (*offset, 1))[:3]
        )
        self.c.save(self.game)
        self.load()
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.VEHICLE)
        self.assertEqual(self.scene.cargo["battery_a"].transport_m, 0)
        self.park(306.5, 4474, 0)
        self.stand(311, 4470)
        self.assertNotIn("battery_delivered", self.c.facts)
        self.use("trolley_battery_bay", "REMOVE BATTERY")
        self.assertIs(self.scene.cargo["battery_original"].slot, Slot.CARRIED)
        self.use("battery_original", "SET DOWN")
        position = self.scene.transform("battery_original")[:3, 3].copy()
        self.stand(306.5, 4469)
        self.use("jeep_cargo_bay", "UNLOAD BATTERY")
        self.stand(310.3, 4469)
        self.assertIn("POWER RESTORED", self.use("trolley_battery_bay", "INSTALL BATTERY"))
        self.assertTrue(self.scene.electrically_operational)
        self.assertFalse(self.scene.readiness["stabilized"])
        self.stand(312.35, 4474.1)
        self.assertIn("ONLINE", self.use("trolley", "INSPECT"))
        self.assertIn("trolley_online", self.c.facts)
        self.assertNotEqual(self.c.manager.status["Q002"], Status.COMPLETED)
        self.c.save(self.game)
        self.load()
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.INSTALLED)
        self.assertIs(self.scene.cargo["battery_original"].slot, Slot.WORLD)
        np.testing.assert_array_equal(self.scene.transform("battery_original")[:3, 3], position)
        np.testing.assert_array_equal(self.game.player_vehicle.position, wreck)

    def test_invalid_install_and_exclusive_slots(self):
        self.stand(311, 4470)
        self.use("trolley_battery_bay", "REMOVE BATTERY")
        self.assertIn("COMPATIBLE", self.use("trolley_battery_bay", "INSTALL BATTERY"))
        self.assertFalse(self.scene.electrically_operational)
        raw = self.scene.cargo_snapshot()
        raw["items"]["battery_a"]["slot"] = "carried"
        with self.assertRaisesRegex(ValueError, "Duplicate cargo"):
            self.scene.restore_cargo(raw)
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.RACK)
        self.assertIs(self.scene.cargo["battery_original"].slot, Slot.CARRIED)

    def test_card_scopes_early_collection_and_sandbox(self):
        self.assertFalse(self.scene.credentials["office_key"].permits("vehicles", "crn-329"))
        self.stand(185.4, 4269.8)
        self.assertIn("crane_card_seen", self.c.facts)
        self.use("crane_access_card", "COLLECT")
        self.assertTrue(self.scene.authorized(self.c.owner, "vehicles", "crn-329"))
        self.assertFalse(self.scene.authorized(self.c.owner, "facilities", "hangar_b"))
        self.assertNotEqual(self.c.manager.status["Q002"], Status.COMPLETED)
        self.load()
        self.assertIn("crane_card_seen", self.c.facts)
        self.assertIn("crane_access_card", self.scene.inventory(self.c.owner))
        self.assertFalse(self.scene.visible(self.scene.objects["crane_access_card"]))
        self.assertEqual(
            sum("I found an access card near" in x for x in self.c._journal_entries()), 1
        )
        sandbox = Game(enemy_count=0)
        self.addCleanup(sandbox.world.terrain.dataset.close)
        self.assertNotIn("hangar_c", sandbox.world.scene.instances)
        self.assertIs(sandbox.player_control.mode, ControlMode.AIRCRAFT)

    def test_schema_three_adds_new_entities_without_moving_actors(self):
        raw = json.loads(self.path.read_text())
        raw["schema_version"] = 3
        raw.pop("cargo")
        raw["objects"] = {
            k: v for k, v in raw["objects"].items() if k in self.scene.legacy_object_ids
        }
        removed = {
            "battery_fault",
            "visit_c",
            "station_found",
            "batteries_exam",
            "battery_eligible",
            "jeep_service",
            "battery_loaded",
            "battery_delivered",
            "battery_removed",
            "battery_installed",
            "trolley_online",
        }
        raw["quests"]["progress"]["Q002"] = {
            k: v for k, v in raw["quests"]["progress"]["Q002"].items() if k not in removed
        }
        raw["pilot"]["position"][0] += 2
        self.path.write_text(json.dumps(raw))
        self.load()
        np.testing.assert_array_equal(self.pilot.position, raw["pilot"]["position"])
        np.testing.assert_array_equal(
            self.jeep.position, raw["jeeps"][self.jeep.identity]["position"]
        )
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.RACK)
        self.c.save(self.game)
        self.assertEqual(json.loads(self.path.read_text())["schema_version"], 4)

    def test_handling_budget_sets_down_cargo(self):
        self.stand(303.25, 4598.55)
        self.use("battery_a", "LIFT")
        self.c.save(self.game)
        self.load()
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.CARRIED)
        self.stand(294, 4598.55)
        self.assertIs(self.scene.cargo["battery_a"].slot, Slot.WORLD)
        self.assertEqual(self.scene.movement_scale, 1)
        self.assertIn("USE THE JEEP", self.c.notice_detail)
        self.assertTrue(self.scene.solid(self.scene.objects["battery_a"]))
