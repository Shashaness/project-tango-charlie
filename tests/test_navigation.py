"""Helmet pose, compass wrap, and local survey integration."""

import csv
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from engine.game import Game
from game.arrival import ExpandedArrival
from game.navigation import append_marker, heading, read_fix, tape
from game.player_control import ControlMode


class NavigationTests(unittest.TestCase):
    def test_world_headings_and_wrap(self):
        for degrees in (0, 45, 90, 135, 180, 225, 270, 315, 359.9):
            angle = math.radians(degrees)
            self.assertAlmostEqual(heading((math.sin(angle), 42, -math.cos(angle))), degrees)
        self.assertIsNone(heading((0, 1, 0)))
        self.assertAlmostEqual(next(x for x, label in tape(359.9) if label == "N"), 0.1)
        self.assertAlmostEqual(next(x for x, label in tape(0.1) if label == "N"), -0.1)

    def test_position_ignores_camera_and_uses_occupied_vehicle(self):
        pilot = SimpleNamespace(position=(107.123456, 1.25, 4739), forward=(1, 0, 0))
        jeep = SimpleNamespace(position=(200, 0.42, 4400), forward=(0, 0, 1))
        control = SimpleNamespace(
            mode=ControlMode.ON_FOOT,
            pilot=pilot,
            active=SimpleNamespace(vehicle=jeep),
            camera=SimpleNamespace(position=(0, 0, 0)),
        )
        fix = read_fix(control)
        self.assertEqual(fix.position, pilot.position)
        self.assertEqual(fix.reference, "person")
        control.camera.position = (1, 2, 3)
        self.assertEqual(read_fix(control), fix)
        control.mode = ControlMode.GROUND_VEHICLE
        self.assertEqual(read_fix(control).position, jeep.position)
        self.assertEqual(read_fix(control).heading, 180)
        self.assertEqual(read_fix(control).reference, "vehicle")
        control.mode = ControlMode.AIRCRAFT
        self.assertIsNone(read_fix(control))

    def test_arrival_survey_and_sandbox(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            path = directory / "save.json"
            campaign = ExpandedArrival(path, layout_revision="q12b")
            game = Game(campaign=campaign)
            self.addCleanup(game.world.terrain.dataset.close)
            game.navigation.directory = directory
            fix = read_fix(game.player_control)
            before = path.read_bytes()
            game.input.survey_pressed = True
            game.update(0)
            self.assertEqual(before, path.read_bytes())
            self.assertEqual(fix, read_fix(game.player_control))
            survey = directory / "arrival_locations.csv"
            with survey.open(newline="") as stream:
                (record,) = tuple(csv.DictReader(stream))
            self.assertEqual(record["marker_id"], "001")
            self.assertEqual(record["world_id"], "arrival")
            self.assertEqual(record["layout_revision"], "q12b")
            self.assertEqual(record["campaign_id"], "arrival")
            self.assertEqual(record["x_m"], "107.000000000")
            self.assertEqual(record["z_m"], "4739.000000000")
            self.assertEqual(float(record["heading_deg"]), round(fix.heading, 9))
            self.assertAlmostEqual(fix.heading, 51, places=5)
            self.assertEqual(append_marker(survey, fix, "arrival", "q12b", "arrival", 12), 2)
            saved = survey.read_bytes()
            survey.write_bytes(saved.rstrip())
            with self.assertRaises(ValueError):
                append_marker(survey, fix, "arrival", "q12b", "arrival", 14)
            self.assertEqual(survey.read_bytes(), saved.rstrip())
            sandbox = Game(enemy_count=0)
            self.addCleanup(sandbox.world.terrain.dataset.close)
            sandbox.navigation.directory = directory
            sandbox.navigation.record()
            self.assertFalse((directory / "sandbox_locations.csv").exists())
            self.assertTrue(sandbox.player_control.interact())
            sandbox.navigation.record()
            self.assertIn(
                "tango_charlie,sandbox-v1,,person",
                (directory / "sandbox_locations.csv").read_text(),
            )
