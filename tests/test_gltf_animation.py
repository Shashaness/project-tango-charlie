"""Synthetic native glTF animation import, playback, and TC167 integration regressions."""

import copy
import io
import math
import struct
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from engine.asset_paths import asset_path
from engine.game import Game
from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import AssetError, build_model, load_glb, read_glb
from engine.model import trs
from game.flight_state import VehicleMode
from game.prototype_model import PROTOTYPE_PATH
from tools.inspect_glb_animations import inspect


def append_accessor(document, binary, values, kind):
    """Append a float32 animation accessor to a synthetic embedded GLB buffer."""
    binary += b"\0" * (-len(binary) % 4)
    values = np.asarray(values, dtype="<f4")
    view = len(document["bufferViews"])
    document["bufferViews"].append(
        {"buffer": 0, "byteOffset": len(binary), "byteLength": values.nbytes}
    )
    index = len(document["accessors"])
    document["accessors"].append(
        {"bufferView": view, "componentType": 5126, "count": len(values), "type": kind}
    )
    binary += values.tobytes()
    document["buffers"][0]["byteLength"] = len(binary)
    return index, binary


def add_clip(document, binary, name, tracks):
    """Append (node index, path, times, values, interpolation) tracks to a document."""
    animation = {"name": name, "samplers": [], "channels": []}
    for node, path, times, values, interpolation in tracks:
        input_index, binary = append_accessor(document, binary, times, "SCALAR")
        output_index, binary = append_accessor(
            document, binary, values, "VEC4" if path == "rotation" else "VEC3"
        )
        animation["channels"].append(
            {"sampler": len(animation["samplers"]), "target": {"node": node, "path": path}}
        )
        animation["samplers"].append(
            {"input": input_index, "output": output_index, "interpolation": interpolation}
        )
    document.setdefault("animations", []).append(animation)
    return binary


def write_glb(path, document, binary):
    """Write synthetic JSON/BIN chunks without touching repository assets."""
    import json

    encoded = json.dumps(document, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    binary += b"\0" * (-len(binary) % 4)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 28 + len(encoded) + len(binary))
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
        + struct.pack("<II", len(binary), 0x004E4942)
        + binary
    )


class NativeImportTests(unittest.TestCase):
    """Exercise the runtime loader with real GLB chunks and malformed animation metadata."""

    def setUp(self):
        self.document, self.binary = read_glb(asset_path("models/test/hierarchy_test.glb"))
        self.binary = add_clip(
            self.document,
            self.binary,
            "Move",
            [(5, "translation", [0, 1], [(0, 0, 0), (2, 0, 0)], "LINEAR")],
        )

    def load(self, document=None, binary=None):
        return build_model(
            self.document if document is None else document,
            self.binary if binary is None else binary,
        )

    def test_valid_glb_geometry_materials_and_cpu_only_import(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "animated.glb"
            write_glb(path, self.document, self.binary)
            with patch("engine.mesh.Mesh", side_effect=AssertionError("GPU allocation")):
                model = load_glb(path)
            static = load_glb(asset_path("models/test/hierarchy_test.glb"))
            self.assertEqual(list(model.animations), ["Move"])
            self.assertEqual(model.animations["Move"].duration, 1)
            for a, b in zip(model.primitives, static.primitives):
                np.testing.assert_array_equal(a.positions, b.positions)
                np.testing.assert_array_equal(a.indices, b.indices)
                self.assertEqual(a.material, b.material)
            with redirect_stdout(io.StringIO()) as output:
                inspect(path)
            self.assertIn("Move: 1 s, 1 channel", output.getvalue())
            self.assertIn("node[5] LeftLeg: translation, LINEAR, 2 keys", output.getvalue())

    def test_node_indices_are_original_not_scene_traversal_indices(self):
        self.document["nodes"][0]["children"].reverse()
        model = self.load()
        channel = model.animations["Move"].channels[0]
        self.assertEqual(channel.node_index, 5)
        self.assertIs(channel.node, model.find_node("LeftLeg"))
        self.assertIsNot(channel.node, model.nodes[5])

    def test_unnamed_clip_has_stable_identifier_without_renaming_nodes(self):
        del self.document["animations"][0]["name"]
        model = self.load()
        self.assertIn("Animation_0", model.animations)
        self.assertEqual(model.animations["Animation_0"].channels[0].node.name, "LeftLeg")

    def test_normalized_quaternions_and_immutable_keys(self):
        self.binary = add_clip(
            self.document,
            self.binary,
            "Rotate",
            [(5, "rotation", [0, 1], [(0, 0, 0, 2), (0, 0, 3, 0)], "LINEAR")],
        )
        channel = self.load().animations["Rotate"].channels[0]
        np.testing.assert_allclose(np.linalg.norm(channel.sampler.values, axis=1), 1)
        self.assertFalse(channel.sampler.times.flags.writeable)
        self.assertFalse(channel.sampler.values.flags.writeable)

    def test_inspector_can_require_complete_battledroid_clip_set(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.glb"
            write_glb(path, self.document, self.binary)
            with redirect_stdout(io.StringIO()), self.assertRaisesRegex(AssetError, "BD_Idle"):
                inspect(path, require_battledroid=True)

    def test_malformed_targets_samplers_and_unsupported_features(self):
        mutations = [
            lambda a, d: a["channels"][0]["target"].update(node=999),
            lambda a, d: a["channels"][0]["target"].update(node=-1),
            lambda a, d: a["channels"][0]["target"].update(node=True),
            lambda a, d: a["channels"][0]["target"].pop("node"),
            lambda a, d: a["channels"][0]["target"].update(path="weights"),
            lambda a, d: a["channels"][0].update(sampler=99),
            lambda a, d: a["samplers"][0].update(interpolation="CUBICSPLINE"),
            lambda a, d: a["samplers"][0].update(interpolation="UNKNOWN"),
            lambda a, d: a["channels"].append(copy.deepcopy(a["channels"][0])),
            lambda a, d: a.update(channels=[]),
            lambda a, d: a.update(samplers=[]),
            lambda a, d: a.update(name=""),
            lambda a, d: d["animations"].append(copy.deepcopy(a)),
            lambda a, d: d.update(skins=[{}]),
            lambda a, d: d["nodes"][5].update(skin=0),
            lambda a, d: d["nodes"][5].update(matrix=list(np.eye(4).ravel())),
            lambda a, d: a["channels"][0].update(target=[]),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                document = copy.deepcopy(self.document)
                mutate(document["animations"][0], document)
                with self.assertRaises(AssetError):
                    self.load(document)

    def test_wrong_accessor_type_component_count_alignment_and_bounds(self):
        output_index = self.document["animations"][0]["samplers"][0]["output"]
        input_index = self.document["animations"][0]["samplers"][0]["input"]
        for index, update in (
            (input_index, {"type": "VEC3"}),
            (input_index, {"componentType": 5123}),
            (input_index, {"byteOffset": 1}),
            (output_index, {"type": "VEC2"}),
            (output_index, {"type": "VEC4"}),
            (output_index, {"normalized": True}),
            (output_index, {"count": 1}),
            (output_index, {"count": 999}),
            (output_index, {"byteOffset": 999999}),
        ):
            with self.subTest(index=index, update=update):
                document = copy.deepcopy(self.document)
                document["accessors"][index].update(update)
                with self.assertRaises(AssetError):
                    self.load(document)
        document = copy.deepcopy(self.document)
        document["bufferViews"][-1]["byteLength"] += 4
        with self.assertRaises(AssetError):
            self.load(document)

    def test_invalid_key_times_values_and_zero_rotation(self):
        for times, values, path in (
            ([0, 0], [(0, 0, 0)] * 2, "translation"),
            ([1, 0], [(0, 0, 0)] * 2, "translation"),
            ([-1, 0], [(0, 0, 0)] * 2, "translation"),
            ([0, math.inf], [(0, 0, 0)] * 2, "translation"),
            ([0, math.nan], [(0, 0, 0)] * 2, "translation"),
            ([0, 1], [(0, 0, 0), (math.nan, 0, 0)], "translation"),
            ([0, 1], [(0, 0, 0), (math.inf, 0, 0)], "scale"),
            ([0, 1], [(0, 0, 0, 0), (0, 0, 0, 1)], "rotation"),
        ):
            with self.subTest(times=times, path=path):
                document, binary = read_glb(asset_path("models/test/hierarchy_test.glb"))
                binary = add_clip(document, binary, "Invalid", [(5, path, times, values, "LINEAR")])
                with self.assertRaises(AssetError):
                    self.load(document, binary)

    def test_errors_include_filename(self):
        self.document["animations"][0]["channels"][0]["target"]["node"] = 999
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid-animation.glb"
            write_glb(path, self.document, self.binary)
            with self.assertRaisesRegex(AssetError, "invalid-animation.glb.*animation"):
                load_glb(path)


class NativePlayerTests(unittest.TestCase):
    """Verify exact-time TRS sampling, transitions, controls, and nonaccumulating application."""

    def setUp(self):
        document, binary = read_glb(asset_path("models/test/hierarchy_test.glb"))
        binary = add_clip(
            document,
            binary,
            "A",
            [
                (5, "translation", [0, 1], [(0, 0, 0), (2, 0, 0)], "LINEAR"),
                (5, "rotation", [0, 1], [(0, 0, 0, 1), (0, 0, 1, 0)], "LINEAR"),
                (5, "scale", [0, 1], [(1, 1, 1), (3, 5, 7)], "LINEAR"),
            ],
        )
        binary = add_clip(
            document, binary, "B", [(5, "translation", [0, 1], [(4, 0, 0)] * 2, "STEP")]
        )
        binary = add_clip(
            document, binary, "C", [(3, "translation", [0, 1], [(8, 0, 0)] * 2, "LINEAR")]
        )
        self.model = build_model(document, binary)
        self.node = self.model.find_node("LeftLeg")
        self.player = AnimationPlayer(self.model)

    def test_translation_scale_and_quaternion_interpolation(self):
        self.player.play("A", loop=False)
        self.player.update(0.5)
        values = self.player.sample()
        np.testing.assert_allclose(values[self.node, "translation"], (1, 0, 0))
        np.testing.assert_allclose(values[self.node, "scale"], (2, 3, 4))
        np.testing.assert_allclose(values[self.node, "rotation"], (0, 0, 2**-0.5, 2**-0.5))

    def test_step_holds_until_exact_key_and_clamps_endpoints(self):
        sampler = self.model.animations["A"].channels[0].sampler
        from engine.gltf_animation import AnimationSampler

        step = AnimationSampler([0.25, 1], sampler.values, "STEP")
        np.testing.assert_allclose(step.sample(0), (0, 0, 0))
        np.testing.assert_allclose(step.sample(0.999), (0, 0, 0))
        np.testing.assert_allclose(step.sample(1), (2, 0, 0))
        np.testing.assert_allclose(step.sample(10), (2, 0, 0))

    def test_antipodal_quaternions_take_shortest_arc_and_step_holds_rotation(self):
        from engine.gltf_animation import AnimationSampler

        values = [(0, 0, 0, 1), (0, 0, 0, -1)]
        linear = AnimationSampler([0, 1], values)
        np.testing.assert_allclose(linear.sample(0.5, rotation=True), values[0])
        step = AnimationSampler([0, 1], values, "STEP")
        np.testing.assert_allclose(step.sample(0.9, rotation=True), values[0])
        np.testing.assert_allclose(step.sample(1, rotation=True), values[1])

    def test_pause_resume_stop_and_speed(self):
        self.player.play("A", speed=2)
        self.player.update(0.1)
        self.player.pause()
        self.player.update(5)
        self.assertAlmostEqual(self.player.time, 0.2)
        self.player.resume()
        self.player.update(0.1)
        self.assertAlmostEqual(self.player.time, 0.4)
        rest = self.node.base_matrix
        self.player.apply()
        self.assertFalse(np.allclose(self.node.local_matrix, rest))
        self.player.stop()
        np.testing.assert_allclose(self.node.local_matrix, rest)
        self.assertEqual(self.player.sample(), {})
        self.player.play("A", speed=0)
        self.player.update(2)
        self.assertEqual(self.player.time, 0)

    def test_loop_boundary_multiple_cycles_and_one_shot(self):
        self.player.play("A")
        self.player.update(3)
        self.assertEqual(self.player.time, 0)
        self.player.update(0.25)
        self.assertAlmostEqual(self.player.time, 0.25)
        self.player.play("A", loop=False)
        self.player.update(3)
        self.assertEqual(self.player.time, 1)
        self.assertTrue(self.player.finished)
        self.assertFalse(self.player.playing)
        self.player.resume()
        self.assertFalse(self.player.playing)

    def test_single_key_clip_with_zero_duration(self):
        from engine.gltf_animation import AnimationChannel, AnimationClip, AnimationSampler

        self.model.animations["Still"] = AnimationClip(
            "Still", (AnimationChannel(5, self.node, "scale", AnimationSampler([0], [[0, 1, 1]])),)
        )
        self.player.play("Still")
        self.player.update(10)
        self.player.apply()
        self.assertTrue(self.player.finished)
        self.assertEqual(self.node.local_matrix[0, 0], 0)
        self.player.stop()
        np.testing.assert_allclose(self.node.local_matrix, self.node.base_matrix)

    def test_smooth_transition_blends_all_components_and_missing_paths(self):
        self.player.play("A", loop=False)
        self.player.update(0.5)
        outgoing = self.player.sample()
        self.player.play("B", transition=1)
        for key, value in outgoing.items():
            np.testing.assert_allclose(self.player.sample()[key], value)
        self.player.update(0.5)
        sample = self.player.sample()
        np.testing.assert_allclose(sample[self.node, "translation"], (2.5, 0, 0))
        np.testing.assert_allclose(sample[self.node, "scale"], (1.5, 2, 2.5))
        self.assertAlmostEqual(np.linalg.norm(sample[self.node, "rotation"]), 1)
        self.player.update(0.5)
        self.player.apply()
        np.testing.assert_allclose(self.node.local_matrix, trs((4, 0, 0)))

    def test_interrupted_transition_is_continuous_and_releases_previous_nodes(self):
        self.player.play("A")
        self.player.update(0.25)
        self.player.play("B", transition=1)
        self.player.update(0.3)
        current = self.player.sample()
        self.player.play("C", transition=1)
        for key, value in current.items():
            np.testing.assert_allclose(self.player.sample()[key], value)
        self.player.apply()
        self.player.update(1)
        self.player.apply()
        np.testing.assert_allclose(self.node.local_matrix, self.node.base_matrix)
        self.assertIn(self.model.find_node("LeftWing"), self.player._owned)

    def test_no_accumulated_transform_drift_and_rest_configuration_preservation(self):
        rest = self.node.base_matrix
        self.node.set_pose((9, 8, 7))
        canonical = self.node.configuration_matrix
        self.player.play("A", speed=0)
        self.player.apply()
        expected = self.node.local_matrix.copy()
        for _ in range(200):
            self.player.update(1 / 60)
            self.player.apply()
            np.testing.assert_array_equal(self.node.local_matrix, expected)
        np.testing.assert_array_equal(self.node.base_matrix, rest)
        np.testing.assert_array_equal(self.node.configuration_matrix, canonical)
        self.player.stop()
        np.testing.assert_array_equal(self.node.local_matrix, canonical)

    def test_frame_rate_independence_including_crossfade(self):
        def run(steps):
            player = AnimationPlayer(self.model)
            player.play("A")
            player.update(0.25)
            player.play("B", transition=0.8)
            for dt in steps:
                player.update(dt)
            return player.sample()

        expected = run([0.5])
        for steps in ([1 / 60] * 30, [1 / 120] * 60, [0.1, 0.05, 0.2, 0.15]):
            actual = run(steps)
            for key in expected:
                np.testing.assert_allclose(actual[key], expected[key], atol=1e-12)

    def test_loop_partition_and_paused_transition(self):
        for fps in (30, 60, 120):
            self.player.play("A")
            for _ in range(fps * 3):
                self.player.update(1 / fps)
            self.assertAlmostEqual(self.player.time, 0, places=10)
        self.player.play("B", transition=1)
        self.player.update(0.3)
        before = self.player.sample()
        self.player.pause()
        self.player.update(10)
        for key, value in before.items():
            np.testing.assert_array_equal(self.player.sample()[key], value)
        self.player.resume()
        self.player.update(0.7)
        self.assertFalse(self.player._source)

    def test_short_one_shot_completes_its_longer_transition(self):
        self.player.play("A")
        self.player.update(0.2)
        self.player.play("B", loop=False, transition=2)
        self.player.update(1.5)
        self.assertTrue(self.player.finished)
        self.assertTrue(self.player.playing)
        self.player.update(0.5)
        self.assertFalse(self.player.playing)
        self.player.apply()
        np.testing.assert_allclose(self.node.local_matrix, trs((4, 0, 0)))

    def test_invalid_controls_and_unknown_name(self):
        with self.assertRaises(KeyError):
            self.player.play("Absent")
        for value in (-1, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.player.play("A", speed=value)
                with self.assertRaises(ValueError):
                    self.player.play("A", transition=value)
                with self.assertRaises(ValueError):
                    self.player.update(value)


class NativeBattledroidTests(unittest.TestCase):
    """Keep physics, transformations, and procedural fallback independent of native gait."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "tc167-animated.glb"
        document, binary = read_glb(PROTOTYPE_PATH)
        indices = {node["name"]: index for index, node in enumerate(document["nodes"])}
        for name, node in (
            ("BD_Idle", "TorsoCore"),
            ("BD_Walk", "LeftUpperLeg"),
            ("BD_Run", "RightUpperLeg"),
        ):
            binary = add_clip(
                document,
                binary,
                name,
                [
                    (
                        indices[node],
                        "rotation",
                        [0, 1],
                        [(0, 0, 0, 1), (math.sin(0.1), 0, 0, math.cos(0.1))],
                        "LINEAR",
                    )
                ],
            )
        write_glb(self.path, document, binary)
        self.game = Game(enemy_count=0, vehicle_model_path=self.path)
        self.game.setup_battledroid_test("ground")
        self.vehicle = self.game.player_vehicle
        self.animation = self.vehicle.animation
        self.pilot = SimpleNamespace(pitch=-1, roll=0)

    def sample(self, speed=0, dt=0.05):
        self.vehicle.velocity[:] = (0, 0, -speed)
        self.animation.clear_layer()
        self.vehicle.locomotion.clear_layer()
        self.animation.prepare(dt, self.vehicle, self.pilot)
        self.vehicle.locomotion.update(dt, self.vehicle, self.pilot)
        self.animation.update()

    def test_clips_selected_by_actual_state_and_velocity(self):
        self.sample()
        self.assertEqual(self.animation.native_clip, "BD_Idle")
        self.sample(2)
        self.assertEqual(self.animation.native_clip, "BD_Walk")
        self.assertEqual(self.animation.native_player.speed, 1)
        self.sample(6)
        self.assertEqual(self.animation.native_clip, "BD_Run")
        self.assertEqual(self.animation.native_player.speed, 1)
        self.assertEqual(self.vehicle.locomotion.state, "RUN")
        self.assertEqual(self.vehicle.locomotion.forward_speed, 6)
        self.assertEqual(self.vehicle.locomotion.offsets, {})
        self.assertEqual(self.vehicle.locomotion.blend, 0)

    def test_missing_clip_and_explicit_disable_use_procedural_fallback(self):
        del self.animation.model.animations["BD_Walk"]
        self.sample(2)
        self.assertIsNone(self.animation.native_clip)
        self.assertFalse(self.vehicle.locomotion.native_active)
        self.assertTrue(self.vehicle.locomotion.offsets)
        self.sample(6)
        self.assertEqual(self.animation.native_clip, "BD_Run")
        self.animation.native_enabled = False
        self.sample(6)
        self.assertIsNone(self.animation.native_clip)
        self.assertTrue(self.vehicle.locomotion.offsets)
        self.game.spawn_hangar()
        self.assertFalse(self.vehicle.animation.native_enabled)

    def test_zero_default_clips_preserve_procedural_backend(self):
        game = Game(enemy_count=0)
        game.setup_battledroid_test("ground")
        vehicle = game.player_vehicle
        vehicle.velocity[:] = (0, 0, -2)
        vehicle.animation.prepare(0.05, vehicle, self.pilot)
        vehicle.locomotion.update(0.05, vehicle, self.pilot)
        vehicle.animation.update()
        self.assertIsNone(vehicle.animation.native_clip)
        self.assertTrue(vehicle.locomotion.offsets)

    def test_missing_components_use_configuration_not_imported_fighter_rest(self):
        torso = self.animation.model.find_node("TorsoCore")
        canonical = torso.configuration_trs
        self.sample(dt=0.25)
        expected = self.animation.native_player.sample()[torso, "rotation"]
        np.testing.assert_allclose(torso.local_matrix, trs(canonical[0], expected, canonical[2]))
        self.assertEqual(torso.configuration_trs, canonical)

    def test_visual_sampling_does_not_touch_motion_or_accumulate(self):
        self.sample(2, dt=0.2)
        position = self.vehicle.position.copy()
        velocity = self.vehicle.velocity.copy()
        orientation = self.vehicle.orientation.copy()
        self.animation.native_player.speed = 0
        for _ in range(100):
            self.animation.clear_layer()
            self.vehicle.locomotion.clear_layer()
            self.animation.update()
        np.testing.assert_array_equal(self.vehicle.position, position)
        np.testing.assert_array_equal(self.vehicle.velocity, velocity)
        np.testing.assert_array_equal(self.vehicle.orientation, orientation)
        before = [node.local_matrix.copy() for node in self.animation.model.nodes]
        self.animation.clear_layer()
        self.vehicle.locomotion.clear_layer()
        self.animation.update()
        for node, matrix in zip(self.animation.model.nodes, before):
            np.testing.assert_allclose(node.local_matrix, matrix, atol=1e-12)

    def test_transformation_disables_native_and_keeps_exact_canonical_endpoint(self):
        self.sample(2)
        controller = self.vehicle.transformation
        controller.request(VehicleMode.VTOL)
        self.animation.clear_layer()
        self.vehicle.locomotion.clear_layer()
        controller.update(0.1)
        expected = [node.local_matrix.copy() for node in controller.model.nodes]
        self.animation.prepare(0.1, self.vehicle, self.pilot)
        self.vehicle.locomotion.update(0.1, self.vehicle, self.pilot)
        self.animation.update()
        self.assertIsNone(self.animation.native_clip)
        for node, matrix in zip(controller.model.nodes, expected):
            np.testing.assert_allclose(node.local_matrix, matrix, atol=1e-12)

        controller.update(10)
        controller.reset(VehicleMode.VTOL)
        expected = [node.local_matrix.copy() for node in controller.model.nodes]
        self.sample(2, dt=0)
        for node, matrix in zip(controller.model.nodes, expected):
            np.testing.assert_allclose(node.local_matrix, matrix, atol=1e-12)

    def test_zero_root_scale_and_foot_scale_do_not_break_engine_attachments(self):
        from engine.gltf_animation import AnimationChannel, AnimationClip, AnimationSampler

        model = self.animation.model
        clip = model.animations["BD_Walk"]
        channels = list(clip.channels)
        for name in ("TC167Root", "LeftFoot"):
            node = model.find_node(name)
            channels.append(AnimationChannel(0, node, "scale", AnimationSampler([0], [[0, 0, 0]])))
        model.animations["BD_Walk"] = AnimationClip("BD_Walk", tuple(channels))
        self.sample(2)
        for node in model.nodes:
            self.assertTrue(np.isfinite(node.world_matrix).all())
        self.animation.native_enabled = False
        self.sample(2)
        self.assertFalse(self.animation._native_attachments)

    def test_engine_attachment_relative_to_foot_is_preserved(self):
        model = self.animation.model
        offsets = {
            side: np.linalg.inv(model.find_node(side + "Foot").world_matrix)
            @ model.find_node(side + "Engine").world_matrix
            for side in ("Left", "Right")
        }
        self.sample(2, dt=0.5)
        for side, offset in offsets.items():
            actual = (
                np.linalg.inv(model.find_node(side + "Foot").world_matrix)
                @ model.find_node(side + "Engine").world_matrix
            )
            np.testing.assert_allclose(actual, offset, atol=1e-10)

    def test_full_game_updates_match_disabled_backend_physics(self):
        fallback = Game(enemy_count=0, vehicle_model_path=self.path, gltf_animation=False)
        fallback.setup_battledroid_test("ground")
        self.game.input.pitch = fallback.input.pitch = -1
        for _ in range(20):
            self.game.update(1 / 60)
            fallback.update(1 / 60)
        for name in ("position", "velocity", "acceleration", "orientation"):
            np.testing.assert_array_equal(
                getattr(self.vehicle, name), getattr(fallback.player_vehicle, name)
            )
        self.assertEqual(self.vehicle.throttle, fallback.player_vehicle.throttle)
        self.assertEqual(self.vehicle.flight_state, fallback.player_vehicle.flight_state)
