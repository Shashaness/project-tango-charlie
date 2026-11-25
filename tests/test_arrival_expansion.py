"""Arrival geography, shared object state, and access progression."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from engine.game import Game
from game.arrival import ExpandedArrival, open_campaign
from game.campaign import ArrivalCampaign
from game.player_control import ControlMode
from game.quests.manager import Status


class ArrivalExpansionTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name) / "save.json"

    def make(self, kind=ExpandedArrival):
        campaign = (
            kind(self.path, previous_layout=True) if kind is ExpandedArrival else kind(self.path)
        )
        game = Game(campaign=campaign)
        self.addCleanup(game.world.terrain.dataset.close)
        return game, campaign

    def stand(self, game, x, z):
        game.player_control.pilot.position[:] = (x, 0, z)
        game.update(0)

    def use(self, game, obj, x, z, action=None):
        campaign = game.campaign
        self.stand(game, x, z)
        if action is None:
            campaign._selected_id = obj
            game.input.inspect_pressed = True
            game.update(0)
            game.input.inspect_pressed = False
            self.assertIsNotNone(campaign.selection, (obj, campaign.notice_detail))
            self.assertEqual(campaign.selection.identity, obj)
            return
        control = game.player_control
        return campaign.objects.interact(
            obj, campaign.owner, control.pilot, control.space, campaign.time, [], action=action
        )

    def test_layout_and_legacy_compatibility(self):
        game, campaign = self.make()
        crash = game.player_vehicle.position[[0, 2]]
        scene = game.world.scene.instances

        def distance(name):
            return np.linalg.norm(np.asarray(scene[name].position)[[0, 2]] - crash)

        self.assertTrue(200 < distance("service_garage") < 350)
        self.assertTrue(1500 < distance("hangar_01") < 2000)
        self.assertFalse(game.world.enemies)
        self.stand(game, 270, 3000)
        self.assertIn("hangar", campaign.facts)
        self.assertEqual(campaign.manager.progress["Q001"]["hangar"], 0)
        resumed, state = self.make()
        np.testing.assert_allclose(resumed.player_control.pilot.position, (270, 0, 3000))
        self.assertIn("hangar", state.facts)
        self.path.unlink()
        legacy, old = self.make(ArrivalCampaign)
        content = self.path.read_bytes()
        restored, restored_state = self.make(open_campaign)
        self.assertIs(type(restored_state), ArrivalCampaign)
        np.testing.assert_allclose(restored.player_vehicle.position, legacy.player_vehicle.position)
        self.assertEqual(content, self.path.read_bytes())
        sandbox = Game(enemy_count=0)
        self.addCleanup(sandbox.world.terrain.dataset.close)
        self.assertNotIn("hangar_b", sandbox.world.scene.instances)

    def test_drawer_key_door_and_gate_persistence(self):
        game, campaign = self.make()
        objects = campaign.objects.objects
        control = game.player_control
        self.stand(game, 304.35, 3020)
        self.assertFalse(campaign.objects.visible(objects["office_key"]))
        self.assertFalse(self.use(game, "office_key", 304.35, 3020, "COLLECT")[0])
        self.use(game, "office_drawer", 304.35, 3020)
        game.update(0.25)
        self.assertEqual(objects["office_drawer"].state, "OPENING")
        campaign.save(game)
        game, campaign = self.make()
        control = game.player_control
        objects = campaign.objects.objects
        self.assertEqual(objects["office_drawer"].state, "OPENING")
        game.update(0.5)
        self.assertTrue(campaign.objects.visible(objects["office_key"]))
        game.update(0)
        self.assertIn("key_seen", campaign.facts)
        self.stand(game, 253.5, 3155)
        self.use(game, "personnel_door", 253.5, 3155)
        self.assertIn("KEY REQUIRED", campaign.notice_detail)
        with patch.object(campaign.objects, "inventory", return_value={"unrelated_key"}):
            events, message = self.use(game, "personnel_door", 253.5, 3155, "UNLOCK")
            self.assertFalse(events)
            self.assertIn("KEY REQUIRED", message)
        self.assertTrue(control.space.blocked(np.array((255.0, 0, 3155))))
        self.assertFalse(self.use(game, "gate_control", 253.5, 3155, "ACTIVATE")[0])
        self.use(game, "office_key", 304.35, 3020)
        self.assertFalse(campaign.objects.visible(objects["office_key"]))
        self.assertIn("office_key", campaign.objects.inventory(campaign.owner))
        self.use(game, "office_drawer", 304.35, 3020)
        game.update(0.6)
        self.assertEqual(objects["office_drawer"].state, "CLOSED")
        self.use(game, "personnel_door", 253.5, 3155)
        self.assertFalse(objects["personnel_door"].locked)
        self.assertEqual(objects["personnel_door"].state, "CLOSED")
        self.use(game, "personnel_door", 253.5, 3155)
        self.assertEqual(objects["personnel_door"].state, "OPENING")
        self.assertTrue(control.space.blocked(np.array((255.0, 0, 3155))))
        game.update(1.25)
        self.assertFalse(control.space.blocked(np.array((255.0, 0, 3155))))
        self.use(game, "gate_control", 261.5, 3157)
        self.assertEqual(objects["main_gate"].state, "OPENING")
        self.assertTrue(control.space.blocked(np.array((255.0, 0, 3115))))
        game.update(4.1)
        self.assertEqual(objects["main_gate"].state, "OPEN")
        self.assertFalse(control.space.blocked(np.array((255.0, 0, 3115))))
        jeep = game.world.ground_vehicles[0]
        jeep.position[:] = (255.0, jeep.position[1], 3115.0)
        self.use(game, "gate_control", 261.5, 3157)
        self.assertIn("OBSTRUCTED", campaign.notice_detail)
        self.assertEqual(objects["main_gate"].state, "OPEN")
        self.assertEqual(objects["gate_control"].activations, 1)
        jeep.position[0] = 265.0
        self.use(game, "gate_control", 261.5, 3157)
        self.assertEqual(objects["main_gate"].state, "CLOSING")
        game.update(0.5)
        jeep.position[0] = 255.0
        game.update(0.25)
        self.assertEqual(objects["main_gate"].state, "OPENING")
        game.update(0.5)
        game, campaign = self.make()
        self.assertEqual(campaign.objects.objects["gate_control"].activations, 2)
        self.assertEqual(campaign.objects.objects["main_gate"].state, "OPEN")
        self.assertIn("office_key", campaign.objects.inventory(campaign.owner))
        self.assertEqual(sum("I found a key" in s for s in campaign._journal_entries()), 1)

    def test_powered_operation_and_out_of_order_progress(self):
        game, campaign = self.make()
        self.stand(game, 275, 3000)
        self.stand(game, 129, 4491.7)
        self.assertIn("discover", campaign.facts)
        game.input.inspect_pressed = True
        game.update(0)
        game.input.inspect_pressed = False
        game.input.interact_pressed = True
        game.update(1 / 30)
        game.input.interact_pressed = False
        self.assertIs(game.player_control.mode, ControlMode.GROUND_VEHICLE)
        self.assertEqual(campaign.manager.status["Q001"], Status.ACTIVE)
        game.input.toggle_parking_brake = True
        game.update(1 / 30)
        game.input.toggle_parking_brake = False
        game.input.pilot_advance = 1
        for _ in range(180):
            game.update(1 / 30)
            if game.world.ground_vehicles[0].position[0] < 111:
                break
        self.assertEqual(campaign.manager.status["Q001"], Status.COMPLETED)
        self.assertEqual(campaign.manager.status["Q002"], Status.ACTIVE)
        self.assertEqual(campaign.driven, campaign.drive_distance)
        self.assertLess(game.world.ground_vehicles[0].position[0], 111)
