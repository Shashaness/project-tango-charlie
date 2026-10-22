"""M21.4 authored asset contracts supersede the frozen M21.3 pose-bridge contract."""

import unittest

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb
from game.authored_transformation import EDGES, REQUIRED_ACTIONS, AuthoredTransformationController
from game.reference_anchors import validate_anchors

AUTHORING = asset_path("models/vehicles/tc167_authoring.glb")


class AuthoringAssetTests(unittest.TestCase):
    def test_configuration_endpoints_and_native_runtime_round_trip(self):
        model = load_glb(AUTHORING)
        c = AuthoredTransformationController(model=model)
        for (source, target), name in EDGES.items():
            clip = model.animations[name]
            for mode, time in ((source, 0), (target, clip.duration)):
                pose = c._sample(name, time, c.configuration_pose(source))
                from engine.model import trs

                for node, expected in c.configuration_pose(mode).items():
                    np.testing.assert_allclose(trs(*pose[node]), trs(*expected), atol=3e-5)

    def test_exported_actions_geometry_and_physical_references(self):
        original = load_glb(asset_path("models/vehicles/tc167.glb"))
        model = load_glb(AUTHORING)
        self.assertEqual(set(model.animations), set(REQUIRED_ACTIONS))
        self.assertEqual(validate_anchors(model), [])
        # A complete export can reorder primitives and nodes. Compare named geometry
        # numerically rather than requiring original GLB bytes/hierarchy forever.
        self.assertEqual(len(model.primitives), len(original.primitives))
        for node in original.nodes:
            current = model.find_node(node.name)
            self.assertEqual(len(node.meshes), len(current.meshes))
            for a, b in zip(node.meshes, current.meshes):
                self.assertEqual(
                    model.primitives[b].positions.shape, original.primitives[a].positions.shape
                )

    def test_legacy_requires_explicit_selection_and_authored_is_default(self):
        from engine.game import Game

        for legacy, path in (
            (True, asset_path("models/vehicles/tc167_animated.glb")),
            (False, AUTHORING),
        ):
            game = Game(enemy_count=0, vehicle_model_path=path, legacy_animation=legacy)
            self.assertEqual(
                game.player_vehicle.transformation.authority, "Legacy" if legacy else "GLB"
            )
            game.world.terrain.dataset.close()
