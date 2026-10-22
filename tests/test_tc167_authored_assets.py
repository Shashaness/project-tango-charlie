"""Production M21.1 rigid-node animation assets retain rest geometry and closed in-place loops."""

import unittest

import numpy as np

from engine.asset_paths import asset_path
from engine.game import Game
from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import load_glb, read_glb
from game.flight_state import VehicleMode
from game.transformation import TransformationController

ORIGINAL = asset_path("models/vehicles/tc167.glb")
ANIMATED = asset_path("models/vehicles/tc167_animated.glb")


class AuthoredAssetTests(unittest.TestCase):
    """Check actual exported artist assets rather than synthetic animation fixtures."""

    def setUp(self):
        self.model = load_glb(ANIMATED)
        self.player = AnimationPlayer(self.model)

    def test_production_geometry_materials_and_fighter_rest_are_exact(self):
        original = load_glb(ORIGINAL)
        self.assertEqual(len(original.nodes), len(self.model.nodes))
        for a, b in zip(original.nodes, self.model.nodes):
            self.assertEqual(a.name, b.name)
            self.assertEqual(
                a.parent.name if a.parent else None, b.parent.name if b.parent else None
            )
            np.testing.assert_array_equal(a.base_matrix, b.base_matrix)
        for a, b in zip(original.primitives, self.model.primitives):
            self.assertEqual(a.material, b.material)
            for name in ("positions", "normals", "texcoords", "indices"):
                np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
        document, binary = read_glb(ORIGINAL)
        _, animated_binary = read_glb(ANIMATED)
        self.assertEqual(binary, animated_binary[: len(binary)])
        self.assertEqual(len(document["nodes"]), 68)

    def test_clips_have_closed_loops_and_no_root_displacement(self):
        self.assertEqual(set(self.model.animations), {"BD_Idle", "BD_Walk", "BD_Run"})
        for name, duration in (("BD_Idle", 2), ("BD_Walk", 1), ("BD_Run", 0.75)):
            clip = self.model.animations[name]
            self.assertEqual(clip.duration, duration)
            self.assertEqual(len(clip.channels), 108)
            first, last = clip.sample(0), clip.sample(duration)
            for key in first:
                np.testing.assert_allclose(first[key], last[key], atol=1e-7)
            root = self.model.find_node("TC167Root")
            for time in np.linspace(0, duration, 49):
                sample = clip.sample(float(time))
                np.testing.assert_allclose(sample[root, "translation"], (0, 3.2873, 0), atol=1e-6)
                np.testing.assert_allclose(sample[root, "rotation"], (0, 0, 0, 1), atol=1e-6)

    def test_idle_starts_at_the_authoritative_canonical_pose(self):
        controller = TransformationController(model=self.model)
        controller.reset(VehicleMode.BATTLEDROID)
        canonical = [node.local_matrix.copy() for node in self.model.nodes]
        self.player.play("BD_Idle")
        self.player.apply()
        for node, matrix in zip(self.model.nodes, canonical):
            np.testing.assert_allclose(node.local_matrix, matrix, atol=2e-5)

    def test_idle_has_subtle_upper_motion_and_stable_legs(self):
        clip = self.model.animations["BD_Idle"]
        start, middle = clip.sample(0), clip.sample(0.5)
        changed = []
        for key in start:
            if not np.allclose(start[key], middle[key], atol=1e-6):
                changed.append(key[0].name)
            if any(part in key[0].name for part in ("Leg", "Foot", "Intake", "Engine")):
                np.testing.assert_allclose(start[key], middle[key], atol=1e-6)
        self.assertTrue({"TorsoCore", "Head", "LeftShoulder", "RightShoulder"} <= set(changed))

    def test_gaits_lift_feet_without_ground_penetration_or_accumulation(self):
        for name in ("BD_Walk", "BD_Run"):
            self.player.play(name, loop=False)
            heights = []
            for time in np.linspace(0, self.player.clip.duration, 97):
                self.player.time = float(time)
                self.player.apply()
                for side in ("Left", "Right"):
                    for node in self.model.nodes:
                        if not node.name.startswith(side + "Foot"):
                            continue
                        for index in node.meshes:
                            vertices = self.model.primitives[index].positions
                            points = (
                                node.world_matrix
                                @ np.column_stack((vertices, np.ones(len(vertices)))).T
                            ).T
                            heights.append(float(points[:, 1].min()))
            self.assertGreater(min(heights), -3.21)
            self.assertGreater(max(heights), -3.08)
            before = [node.local_matrix.copy() for node in self.model.nodes]
            self.player.apply()
            for node, matrix in zip(self.model.nodes, before):
                np.testing.assert_array_equal(node.local_matrix, matrix)

    def test_full_game_uses_authored_clips_without_changing_physics(self):
        native = Game(legacy_animation=True, enemy_count=0, vehicle_model_path=ANIMATED)
        fallback = Game(legacy_animation=True, enemy_count=0, vehicle_model_path=ANIMATED, gltf_animation=False)
        for game in (native, fallback):
            game.setup_battledroid_test("ground")
            game.input.pitch = -1
        for _ in range(60):
            native.update(1 / 60)
            fallback.update(1 / 60)
        self.assertEqual(native.player_vehicle.animation.native_clip, "BD_Walk")
        for field in ("position", "velocity", "orientation", "acceleration"):
            np.testing.assert_array_equal(
                getattr(native.player_vehicle, field), getattr(fallback.player_vehicle, field)
            )

    def test_reversing_transformation_retains_production_endpoints(self):
        controller = TransformationController(model=self.model)
        controller.request(VehicleMode.VTOL)
        controller.update(0.5)
        current = [node.local_matrix.copy() for node in self.model.nodes]
        controller.reverse()
        for node, matrix in zip(self.model.nodes, current):
            np.testing.assert_array_equal(node.local_matrix, matrix)
        controller.update(10)
        for node in self.model.nodes:
            np.testing.assert_array_equal(node.local_matrix, node.base_matrix)
