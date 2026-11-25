"""Presentation state for the terrestrial information panel."""

import tempfile
import unittest
from pathlib import Path

from engine.game import Game
from engine.helmet_information import HelmetInformation, paginate
from game.arrival import open_campaign
from game.quests.manager import Event


class HelmetInformationTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        campaign = open_campaign(Path(folder.name) / "arrival.json")
        self.game = Game(campaign=campaign)
        self.addCleanup(self.game.world.terrain.dataset.close)
        self.campaign = campaign
        self.control = self.game.player_control
        self.input = self.game.input

    def test_tabs_do_not_reveal_locked_missions_or_mutate_progress(self):
        info = self.game.helmet_information
        self.assertEqual(info.content(self.control, self.campaign)[0], "GENERAL")
        before = self.campaign.manager.snapshot()
        self.input.helmet_tab_pressed = True
        info.observe(self.campaign, self.control, self.input, 0)
        label, lines = info.content(self.control, self.campaign)
        self.assertEqual(label, "QUESTS")
        self.assertIn("CURRENT / Locate the outlying service compound", lines)
        self.assertTrue(all("Q002" not in line and "key" not in line.lower() for line in lines))
        info.observe(self.campaign, self.control, self.input, 0)
        self.assertEqual(info.content(self.control, self.campaign)[0], "LOG")
        self.assertEqual(before, self.campaign.manager.snapshot())
        self.campaign.journal_open = True
        self.assertEqual(info.content(self.control, self.campaign)[0], "JOURNAL")
        self.campaign.journal_open = False
        self.assertEqual(info.content(self.control, self.campaign)[0], "LOG")

    def test_completion_notification_fades_and_does_not_replay_on_resume(self):
        info = self.game.helmet_information
        info.observe(self.campaign, self.control, self.input, 0)
        objective = self.campaign.manager.ready("Q001")[0]
        self.campaign.manager.accept(
            Event(
                objective.event,
                self.campaign.owner,
                objective.target,
                (0, 0, 0),
                1,
            )
        )
        info.observe(self.campaign, self.control, self.input, 1)
        self.assertEqual(info.banner(), ("OBJECTIVE COMPLETED", objective.title, 1.0))
        info.observe(self.campaign, self.control, self.input, 4.2)
        self.assertTrue(0 < info.banner()[2] < 1)
        info.observe(self.campaign, self.control, self.input, 5)
        self.assertIsNone(info.banner())
        self.assertEqual(sum(objective.title in line for line in info.history), 1)
        reopened = HelmetInformation()
        reopened.observe(self.campaign, self.control, self.input, 5)
        self.assertIsNone(reopened.banner())

    def test_pagination_wraps_and_preserves_all_lines(self):
        text = "PERSISTENT CONTEXTUAL FEEDBACK"
        first, number, total = paginate([text], columns=11, rows=2, page=0)
        last, last_number, _ = paginate([text], columns=11, rows=2, page=1)
        self.assertEqual((number, total, last_number), (1, 2, 2))
        self.assertEqual(" ".join(first + last), text)
        self.assertLessEqual(max(map(len, first + last)), 11)

    def test_context_fades_independently_and_deduplicates_repeated_attempts(self):
        info = self.game.helmet_information
        campaign = self.campaign
        info.observe(campaign, self.control, self.input, 0)
        objective = campaign.manager.ready("Q001")[0]
        campaign.manager.accept(
            Event(objective.event, campaign.owner, objective.target, (0, 0, 0), 1)
        )
        message = "DOOR LOCKED - KEY REQUIRED"
        campaign.notice_detail = message
        campaign.detail_until = 6
        info.observe(campaign, self.control, self.input, 1)
        self.assertEqual(info.context_banner(), ("ACCESS DENIED", message, 0))
        info.observe(campaign, self.control, self.input, 1.1)
        self.assertAlmostEqual(info.context_banner()[2], 0.5)
        self.assertIsNotNone(info.banner())
        campaign.detail_until = 6.2
        info.observe(campaign, self.control, self.input, 1.2)
        self.assertEqual(info.context_started, 1)
        self.assertEqual(info.history.count(message), 1)
        info.observe(campaign, self.control, self.input, 4.6)
        self.assertIsNone(info.banner())
        self.assertTrue(0 < info.context_banner()[2] < 1)
        info.observe(campaign, self.control, self.input, 4.9)
        self.assertIsNone(info.context_banner())
        self.assertIn(message, info.content(self.control, campaign)[1])
        self.assertIn(message, info.history)

    def test_selection_replaces_card_and_inspection_remains_in_general(self):
        info = self.game.helmet_information
        campaign = self.campaign
        drawer = campaign.objects.objects["office_drawer"]
        desk = campaign.objects.objects["office_desk"]
        campaign.selection = desk
        campaign.selection_position = (1, 2)
        info.observe(campaign, self.control, self.input, 0)
        self.assertIsNone(info.context_banner())
        history = tuple(info.history)
        self.input.object_next_pressed = True
        info.observe(campaign, self.control, self.input, 1)
        campaign.selection = drawer
        campaign.selection_position = (2, 2)
        info.observe(campaign, self.control, self.input, 1.1)
        self.input.object_next_pressed = False
        self.assertEqual(info.context_started, 1.1)
        title, body, _ = info.context_banner()
        self.assertEqual(title, "OBJECT SELECTED\t2 / 2")
        self.assertIn("[T] OPEN", body)
        self.assertEqual(tuple(info.history), history)
        drawer.target = 1
        info.observe(campaign, self.control, self.input, 1.2)
        self.assertIn("[T] CLOSE", info.context_banner()[1])
        result = "DRAWER INSPECTION RESULT"
        campaign.notice_detail = result
        campaign.inspected_object = drawer.identity
        campaign.detail_until = 5
        info.observe(campaign, self.control, self.input, 1.3)
        self.assertEqual(info.context_banner()[0], "OBJECT INSPECTION")
        campaign.time = 6
        info.observe(campaign, self.control, self.input, 6.1)
        self.assertIn(result, info.context_banner()[1])
        info.observe(campaign, self.control, self.input, 7.2)
        self.assertIsNone(info.context_banner())
        self.assertIn(result, info.content(self.control, campaign)[1])
        self.assertIn(result, info.history)
