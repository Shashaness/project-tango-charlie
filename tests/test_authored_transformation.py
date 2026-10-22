"""M21.4 GLB authority, continuity, hierarchy and physical-reference regressions."""

import unittest
from unittest.mock import patch

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_animation import AnimationChannel, AnimationClip, AnimationSampler
from engine.gltf_loader import load_glb
from engine.model import ModelNode, trs
from game.authored_transformation import EDGES, REQUIRED_ACTIONS
from game.authored_transformation import AuthoredTransformationController as Controller
from game.flight_state import Environment, FlightStatus
from game.flight_state import VehicleMode as M
from game.player_vehicle import PlayerVehicle
from game.reference_anchors import clearance, floor, positions, validate_anchors
from game.transformation import TransformationController

PATH = asset_path("models/vehicles/tc167_authoring.glb")


class AuthoredTests(unittest.TestCase):
    def setUp(self):
        self.model = load_glb(PATH)
        self.vehicle = PlayerVehicle()
        self.c = self.vehicle.transformation = Controller(self.vehicle, self.model)

    def matrices(self):
        return [n.local_matrix.copy() for n in self.model.nodes]

    def assert_pose(self, matrices, tolerance=1e-10):
        for n, expected in zip(self.model.nodes, matrices):
            np.testing.assert_allclose(n.local_matrix, expected, atol=tolerance)

    def test_actions_and_every_edge_actual_duration_and_intermediate_channels(self):
        self.assertTrue(set(REQUIRED_ACTIONS) <= self.model.animations.keys())
        for (source, target), action in EDGES.items():
            self.c.reset(source)
            self.c.request(target)
            clip = self.model.animations[action]
            self.assertEqual(self.c.duration, clip.duration)
            self.c.update(clip.duration * 0.43)
            for (node, path), value in clip.sample(clip.duration * 0.43).items():
                component = {"translation": 0, "rotation": 1, "scale": 2}[path]
                np.testing.assert_allclose(self.c.pose[node][component], value, atol=1e-10)
            self.c.update(clip.duration * 0.57 + 1e-8)
            self.assertFalse(self.c.active)
            self.assertIs(self.c.configuration, target)
            self.assertIs(self.vehicle.flight_state.mode, target)
            for node, pose in self.c.configuration_pose(target).items():
                np.testing.assert_allclose(node.local_matrix, trs(*pose), atol=1e-10)

    def test_exported_configuration_endpoints(self):
        for (source, target), name in EDGES.items():
            clip = self.model.animations[name]
            for mode, time in ((source, 0), (target, clip.duration)):
                sample = self.c._sample(name, time, self.c.configuration_pose(source))
                for node, expected in self.c.configuration_pose(mode).items():
                    np.testing.assert_allclose(trs(*sample[node]), trs(*expected), atol=3e-5)

    def test_reversal_uses_separate_action_and_is_continuous(self):
        self.c.request(M.VTOL)
        self.c.update(0.6)
        before = self.matrices()
        self.c.reverse()
        self.assertEqual(self.c.action, "TC_VTOLToFighter")
        self.assert_pose(before)
        self.c.update(0)
        self.assert_pose(before)
        self.c.update(self.c.bridge_seconds + self.c.duration + 1e-8)
        self.assertIs(self.c.configuration, M.FIGHTER)

    def test_queue_pause_restart_interrupt_and_large_dt(self):
        self.c.request(M.BATTLEDROID)
        self.c.update(0.3)
        before = self.matrices()
        self.c.pause()
        self.c.update(5)
        self.assert_pose(before)
        self.c.resume()
        self.c.restart()
        self.assert_pose(before)
        self.c.update(20)
        self.assertIs(self.c.configuration, M.BATTLEDROID)
        self.c.request(M.VTOL)
        self.c.update(0.4)
        before = self.matrices()
        self.c.interrupt(M.FIGHTER)
        self.assert_pose(before)
        self.c.update(30)
        self.assertIs(self.c.configuration, M.FIGHTER)

    def test_missing_actions_and_references_fail_without_fallback(self):
        self.model.animations.pop("TC_FighterToVTOL")
        with self.assertRaisesRegex(ValueError, "TC_FighterToVTOL.*legacy-animation"):
            Controller(model=self.model)
        self.model.nodes = [n for n in self.model.nodes if not n.name.startswith("TC_THRUST_")]
        self.assertTrue(any("THRUST" in e for e in validate_anchors(self.model)))

    def test_no_legacy_visual_writes_and_world_motion_remains_physical(self):
        before = self.vehicle.position.copy()
        with (
            patch.object(TransformationController, "apply_curve", side_effect=AssertionError),
            patch.object(ModelNode, "set_pose", side_effect=AssertionError),
            patch.object(ModelNode, "set_delta", side_effect=AssertionError),
        ):
            self.c.request(M.BATTLEDROID)
            self.c.update(10)
            self.c.locomotion(0.1, self.vehicle)
        np.testing.assert_array_equal(before, self.vehicle.position)

    def test_new_node_animated_without_pose_tables_and_stale_channels_reset(self):
        parent = self.model.find_node("LeftWing")
        node = ModelNode("ArtistScorpionSegment", base_trs=((0, 0, 0), (0, 0, 0, 1), (1, 1, 1)))
        node.parent = parent
        parent.children.append(node)
        self.model.nodes.append(node)
        clip = self.model.animations["TC_FighterToVTOL"]
        channel = AnimationChannel(
            len(self.model.nodes) - 1,
            node,
            "translation",
            AnimationSampler([0, 0.5, clip.duration], [[0, 0, 0], [0, 3, 1], [0, 0, 0]]),
        )
        self.model.animations[clip.name] = AnimationClip(clip.name, (*clip.channels, channel))
        self.c = self.vehicle.transformation = Controller(self.vehicle, self.model)
        self.c.request(M.VTOL)
        self.c.update(0.5)
        np.testing.assert_allclose(node.local_matrix[:3, 3], [0, 3, 1])
        self.model.world_matrices()
        np.testing.assert_allclose(node.world_matrix, parent.world_matrix @ node.local_matrix)
        self.c.reset()
        np.testing.assert_allclose(node.local_matrix, np.eye(4))

    def test_anchor_hierarchy_clearance_and_sloped_terrain(self):
        self.assertEqual(validate_anchors(self.model), [])
        self.c.reset(M.BATTLEDROID)
        contacts = positions(self.vehicle, "TC_CONTACT_BATTLEDROID_")
        self.assertGreaterEqual(len(contacts), 2)
        self.assertAlmostEqual(
            clearance(self.vehicle), -min(p[1] for p in contacts.values()), places=4
        )
        self.vehicle.rotate(roll=0.15)

        def height(p):
            return 0.05 * p[0] + 0.02 * p[2]

        required = floor(self.vehicle, height)
        self.vehicle.position[1] = required
        for p in positions(self.vehicle).values():
            self.assertTrue(np.isfinite(p).all())
        for p in contacts.values():
            world = self.vehicle.orientation @ p + self.vehicle.position
            self.assertGreaterEqual(world[1] - height(world), -1e-6)

    def test_absolute_locomotion_blending_no_double_configuration(self):
        self.vehicle.flight_state.environment = Environment.ATMOSPHERE
        self.vehicle.flight_state.status = FlightStatus.GROUNDED
        self.c.reset(M.BATTLEDROID)
        self.c.locomotion(0.2, self.vehicle)
        before = self.matrices()
        self.vehicle.velocity[2] = -2
        self.c.locomotion(0, self.vehicle)
        self.assert_pose(before)
        self.c.locomotion(0.2, self.vehicle)
        self.assertEqual(self.c.action, "BD_Walk")
        sample = self.c._sample(
            "BD_Walk", self.c._gait_time, self.c.configuration_pose(M.BATTLEDROID)
        )
        for n, pose in sample.items():
            np.testing.assert_allclose(n.local_matrix, trs(*pose), atol=1e-10)
        before = self.matrices()
        self.vehicle.velocity[2] = -6
        self.c.locomotion(0, self.vehicle)
        self.assert_pose(before)
        self.c.locomotion(0.2, self.vehicle)
        self.assertEqual(self.c.action, "BD_Run")

    def test_animation_debug_labels_have_renderable_glyphs(self):
        from engine.hud import FONT, instrument_groups

        self.vehicle.animation_debug = True
        for mode in M:
            self.c.reset(mode)
            labels = instrument_groups(self.vehicle)["right"]
            self.assertTrue(any("GLB TC_" in label for label in labels))
            for label in labels:
                self.assertTrue(set(label) <= FONT.keys(), label)

    def test_legacy_option_and_default_game(self):
        from engine.game import Game

        for legacy in (True, False):
            game = Game(enemy_count=0, legacy_animation=legacy)
            self.assertEqual(
                game.player_vehicle.transformation.authority, "Legacy" if legacy else "GLB"
            )
            game.world.terrain.dataset.close()


class CubicTests(unittest.TestCase):
    def test_hermite_tangents_are_scaled_by_key_interval(self):
        sampler = AnimationSampler(
            [0, 2],
            [[0, 0, 0], [0, 0, 0], [2, 0, 0], [0, 0, 0], [2, 0, 0], [0, 0, 0]],
            "CUBICSPLINE",
        )
        np.testing.assert_allclose(sampler.sample(1), [1.5, 0, 0])
        np.testing.assert_array_equal(sampler.sample(0), [0, 0, 0])
        np.testing.assert_array_equal(sampler.sample(5), [2, 0, 0])

    def test_cubic_quaternion_preserves_tangents_and_normalizes_result(self):
        node = ModelNode("new", base_trs=((0, 0, 0), (0, 0, 0, 1), (1, 1, 1)))
        values = [
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0.2, 0, 0],
            [0, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 0],
        ]
        channel = AnimationChannel(
            0, node, "rotation", AnimationSampler([0, 1], values, "CUBICSPLINE")
        )
        np.testing.assert_array_equal(channel.sampler.values, values)
        self.assertAlmostEqual(np.linalg.norm(channel.sampler.sample(0.5, True)), 1)
