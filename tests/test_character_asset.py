"""M24.2 runtime skin, texture, seating, and manifest coverage."""

import json
import struct
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_loader import AssetError, read_glb
from engine.jeep_asset import jeep_access_points
from engine.pilot_asset import CLIPS, SEMANTICS, PilotAsset
from game.world_manifest import DEFAULT_WORLD, load_manifest

FEMALE = asset_path("models/characters/female_pilot.glb")
LEGACY = asset_path("models/characters/pilot.glb")


def write_glb(path, document, binary):
    description = json.dumps(document, separators=(",", ":")).encode()
    description += b" " * (-len(description) % 4)
    binary += b"\x00" * (-len(binary) % 4)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 28 + len(description) + len(binary))
        + struct.pack("<I4s", len(description), b"JSON")
        + description
        + struct.pack("<I4s", len(binary), b"BIN\0")
        + binary
    )


class CharacterAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.female = PilotAsset(FEMALE)
        cls.original = PilotAsset(LEGACY)

    def test_independent_fitted_rest_and_animated_start(self):
        female, original = self.female, self.original
        female.reset_locomotion()
        original.reset_locomotion()
        self.assertEqual(set(female.semantic_joints), set(SEMANTICS))
        for name in ("Forearm.L", "Thigh.L", "Shin.R"):
            target = female.semantic_joints[name]
            source = original.semantic_joints[name]
            self.assertFalse(np.allclose(target.world_matrix[:3, :3], source.world_matrix[:3, :3]))
            self.assertNotAlmostEqual(
                np.linalg.norm(target.base_trs[0]),
                np.linalg.norm(source.base_trs[0]),
                places=3,
            )
        bind = next(female.vertices())[0][:, :3]
        self.assertAlmostEqual(bind[:, 1].min(), 0, places=5)
        for clip in CLIPS:
            female.reset_locomotion()
            female.update(clip, 0)
            animated = next(female.vertices())[0][:, :3]
            self.assertGreater(np.max(np.abs(bind - animated)), 0.07, clip)
            self.assertAlmostEqual(animated[:, 1].min(), 0, places=4, msg=clip)
            female.player.time = female.player.clip.duration
            female.player.apply()
            np.testing.assert_allclose(next(female.vertices())[0][:, :3], animated, atol=1e-5)

    def test_texture_and_character_authored_jeep_pose(self):
        female, original = self.female, self.original
        texture, _ = female.base_colors[0]
        self.assertEqual(texture.shape, (4096, 4096, 4))
        self.assertEqual(set(female.model.animations), {*CLIPS, "Seated"})
        self.assertEqual(set(original.model.animations), set(CLIPS))
        driver_seat = jeep_access_points()["DriverSeat"]
        fit = next(female.vertices(seated=True))[0][:, :3]
        fit = fit - female.seat_hips + female.seat_offset + driver_seat
        # Jeep floor top and front of the existing hood geometry, in Jeep local space.
        self.assertGreaterEqual(fit[:, 1].min(), 0.375 - 0.008)
        self.assertLess(fit[:, 2].max(), 0.42)
        female.reset_locomotion()
        female.update("Seated", 0)
        np.testing.assert_allclose(
            next(female.vertices(seated=True))[0],
            next(female.vertices())[0],
            atol=1e-6,
        )
        female.reset_locomotion()
        female.update("Idle", 0)
        self.assertAlmostEqual(next(female.vertices())[0][:, 1].min(), 0, places=4)

    def test_manifest_selects_both_assets(self):
        def selected(definition):
            return next(e.asset.path for e in definition.entities if e.identity == "pilot_01")

        default = load_manifest()
        self.assertEqual(selected(default), FEMALE)
        self.assertEqual(default.assets["pilot"].path, LEGACY)
        document = json.loads(DEFAULT_WORLD.read_text())
        next(e for e in document["entities"] if e["id"] == "pilot_01")["asset"] = "pilot"
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "world.json"
            manifest.write_text(json.dumps(document))
            self.assertEqual(selected(load_manifest(manifest)), LEGACY)

    def test_renamed_joint_and_additional_skin_joint(self):
        document, binary = read_glb(LEGACY)
        skin = document["skins"][0]
        nodes = document["nodes"]
        mesh = next(node for node in nodes if "mesh" in node)
        hips = next(node for node in nodes if node.get("name") == "Hips")
        hips["name"] = "IndependentPelvis"
        mesh["name"] = "IndependentMesh"
        mapping = {role: role for role in SEMANTICS}
        mapping["Hips"] = hips["name"]
        mesh["extras"] = {"ptc_character_version": 1, "ptc_semantics": json.dumps(mapping)}
        seat = deepcopy(next(c for c in document["animations"] if c["name"] == "Idle"))
        seat["name"] = "Seated"
        document["animations"].append(seat)
        extra = len(nodes)
        hips.setdefault("children", []).append(extra)
        nodes.append({"name": "Harness", "translation": [0, 0.12, 0.10]})
        view = len(document["bufferViews"])
        document["bufferViews"].append(
            {
                "buffer": 0,
                "byteOffset": len(binary),
                "byteLength": 64,
            }
        )
        # Move the inverse-bind accessor to a new contiguous view with one more entry.
        accessor = document["accessors"][skin["inverseBindMatrices"]]
        old_view = document["bufferViews"][accessor["bufferView"]]
        offset = old_view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
        matrices = binary[offset : offset + accessor["count"] * 64]
        document["bufferViews"][view]["byteLength"] += len(matrices)
        binary += matrices + np.eye(4, dtype="<f4").tobytes()
        document["buffers"][0]["byteLength"] = len(binary)
        accessor.update({"bufferView": view, "byteOffset": 0, "count": accessor["count"] + 1})
        skin["joints"].append(extra)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "independent.glb"
            write_glb(path, document, binary)
            asset = PilotAsset(path)
            self.assertEqual(len(asset.joints), 20)
            self.assertEqual(asset.semantic_joints["Hips"].name, "IndependentPelvis")
            self.assertEqual(asset.joints[-1].name, "Harness")
            bad = deepcopy(document)
            del bad["animations"][-1]
            write_glb(path, bad, binary)
            with self.assertRaisesRegex(AssetError, "missing a required"):
                PilotAsset(path)
            bad = deepcopy(document)
            mapping["Hips"] = "Unmapped"
            next(n for n in bad["nodes"] if "mesh" in n)["extras"]["ptc_semantics"] = json.dumps(
                mapping
            )
            write_glb(path, bad, binary)
            with self.assertRaisesRegex(AssetError, "semantic mapping"):
                PilotAsset(path)

    def test_invalid_weights_are_rejected(self):
        document, binary = read_glb(LEGACY)
        primitive = document["meshes"][0]["primitives"][0]
        weights = document["accessors"][primitive["attributes"]["WEIGHTS_0"]]
        view = document["bufferViews"][weights["bufferView"]]
        start = view.get("byteOffset", 0) + weights.get("byteOffset", 0)
        self.assertEqual(weights["componentType"], 5126)
        damaged = binary[:start] + bytes(16) + binary[start + 16 :]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unweighted.glb"
            write_glb(path, document, damaged)
            with self.assertRaisesRegex(AssetError, "Invalid pilot joint indices or skin weights"):
                PilotAsset(path)
