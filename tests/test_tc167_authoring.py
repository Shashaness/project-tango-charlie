"""M21.4 authored asset contracts supersede the frozen M21.3 pose-bridge contract."""

import json
import unittest
from pathlib import Path

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb
from engine.model import trs
from game.authored_transformation import EDGES, REQUIRED_ACTIONS, AuthoredTransformationController
from game.flight_state import VehicleMode
from game.reference_anchors import validate_anchors

AUTHORING = asset_path("models/vehicles/tc167_authoring.glb")


FUSELAGE = "FuselageSurface1"
TAIL = tuple(f"{FUSELAGE}.{i:03}" for i in range(1, 7))
FUSELAGE_REFERENCE = Path(__file__).parent / "fixtures/tc167_fighter_fuselage.json"


def surfaces(model, names):
    triangles = []
    for name in names:
        node = model.find_node(name)
        rotation, translation = node.world_matrix[:3, :3], node.world_matrix[:3, 3]
        for index in node.meshes:
            primitive = model.primitives[index]
            points = primitive.positions @ rotation.T + translation
            indices = primitive.indices
            if indices is None:
                indices = np.arange(len(points))
            triangles.extend(points[indices.reshape(-1, 3)])
    return np.asarray(triangles)


def surface_area(triangles):
    a, b, c = triangles.transpose(1, 0, 2)
    return np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() / 2


def surface_distance(source, target):
    """Largest sampled distance to the target triangle surface, independent of tessellation."""
    samples = np.concatenate(
        (
            source.reshape(-1, 3),
            ((source + np.roll(source, -1, axis=1)) / 2).reshape(-1, 3),
            source.mean(axis=1),
        )
    )
    a, b, c = target.transpose(1, 0, 2)
    normal = np.cross(b - a, c - a)
    squared_norm = np.sum(normal**2, axis=1)
    height = np.einsum("pfi,fi->pf", samples[:, None, :] - a, normal)
    inside = np.ones(height.shape, dtype=bool)
    distance_squared = np.full(height.shape, np.inf)
    for start, end in ((a, b), (b, c), (c, a)):
        offset = samples[:, None, :] - start
        edge = end - start
        side = np.einsum("pfi,fi->pf", np.cross(edge, offset), normal)
        inside &= side >= -1e-12
        squared_length = np.sum(edge**2, axis=1)
        along = np.einsum("pfi,fi->pf", offset, edge) / np.maximum(squared_length, 1e-24)
        delta = offset - np.clip(along, 0, 1)[..., None] * edge
        distance_squared = np.minimum(distance_squared, np.sum(delta**2, axis=2))
    projected = height**2 / np.maximum(squared_norm, 1e-24)
    np.minimum(
        distance_squared,
        np.where(inside & (squared_norm > 1e-24), projected, np.inf),
        out=distance_squared,
    )
    return np.sqrt(distance_squared.min(axis=1).max())


class AuthoringAssetTests(unittest.TestCase):
    def test_configuration_endpoints_and_native_runtime_round_trip(self):
        model = load_glb(AUTHORING)
        c = AuthoredTransformationController(model=model)
        for (source, target), name in EDGES.items():
            clip = model.animations[name]
            for mode, time in ((source, 0), (target, clip.duration)):
                pose = c._sample(name, time, c.configuration_pose(source))
                for node, expected in c.configuration_pose(mode).items():
                    np.testing.assert_allclose(trs(*pose[node]), trs(*expected), atol=3e-5)

    def test_exported_actions_geometry_and_physical_references(self):
        original = load_glb(asset_path("models/vehicles/tc167.glb"))
        model = load_glb(AUTHORING)
        self.assertEqual(set(model.animations), set(REQUIRED_ACTIONS))
        self.assertEqual(validate_anchors(model), [])
        controller = AuthoredTransformationController(model=model)
        geometry_nodes = [node.name for node in original.nodes if node.meshes]
        self.assertCountEqual(
            [node.name for node in model.nodes if node.meshes], [*geometry_nodes, *TAIL]
        )
        for index, name in enumerate(TAIL):
            node = model.find_node(name)
            self.assertEqual(node.parent.name, (*TAIL[1:], FUSELAGE)[index])
        for name in geometry_nodes:
            if name == FUSELAGE:
                continue
            original_node = original.find_node(name)
            current = model.find_node(name)
            self.assertEqual(len(current.meshes), len(original_node.meshes), name)
            for previous, latest in zip(original_node.meshes, current.meshes):
                a, b = original.primitives[previous], model.primitives[latest]
                self.assertEqual(a.material, b.material, name)
                np.testing.assert_array_equal(a.indices, b.indices, err_msg=name)
                for field in ("positions", "normals", "texcoords"):
                    previous, current_values = getattr(a, field), getattr(b, field)
                    if previous is None:
                        self.assertIsNone(current_values, name)
                    else:
                        np.testing.assert_allclose(
                            previous, current_values, rtol=0, atol=2e-6, err_msg=name
                        )
        controller.reset(VehicleMode.FIGHTER)
        actual = surfaces(model, (FUSELAGE, *TAIL))
        reference = json.loads(FUSELAGE_REFERENCE.read_text())
        expected = np.asarray(reference["vertices"])[reference["triangles"]]
        self.assertAlmostEqual(surface_area(actual), surface_area(expected), delta=2e-4)
        self.assertLess(surface_distance(expected, actual), 2e-4)
        self.assertLess(surface_distance(actual, expected), 2e-4)

        fighter = controller.configuration_pose(VehicleMode.FIGHTER)
        vtol = controller.configuration_pose(VehicleMode.VTOL)
        battledroid = controller.configuration_pose(VehicleMode.BATTLEDROID)
        for name in TAIL:
            node = model.find_node(name)
            np.testing.assert_allclose(trs(*fighter[node]), trs(*battledroid[node]), atol=3e-5)
            delta = np.max(np.abs(trs(*fighter[node]) - trs(*vtol[node])))
            if name in TAIL[:2]:
                self.assertGreater(delta, 0.05, name)
            else:
                self.assertLess(delta, 3e-5, name)

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
