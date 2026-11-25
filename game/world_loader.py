"""Resolve authored scene transforms and extract explicit environment geometry."""

from dataclasses import dataclass
from functools import lru_cache
from itertools import product
from types import MappingProxyType

import numpy as np

from engine.gltf_loader import load_glb
from game.world_manifest import default_definition, require

STATIC_KINDS = {"structure", "prop"}
ACTOR_KINDS = {"aircraft", "character", "ground_vehicle"}
CUBE = np.array(list(product((-1, 1), repeat=3)))


def transform(entity):
    x, y, z = np.radians(entity.rotation)
    cx, cy, cz = np.cos((x, y, z))
    sx, sy, sz = np.sin((x, y, z))
    rx = ((1, 0, 0), (0, cx, -sx), (0, sx, cx))
    ry = ((cy, 0, sy), (0, 1, 0), (-sy, 0, cy))
    rz = ((cz, -sz, 0), (sz, cz, 0), (0, 0, 1))
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ (np.asarray(ry) @ rx) @ np.diag(entity.scale)
    matrix[:3, 3] = entity.position
    return matrix


def points(matrix, coordinates):
    return coordinates @ matrix[:3, :3].T + matrix[:3, 3]


@lru_cache(maxsize=32)
def scene_model(path):
    return load_glb(path, convert_coordinates=False)


@dataclass(frozen=True)
class Placement:
    definition: object
    matrix: tuple

    @property
    def identity(self):
        return self.definition.identity

    @property
    def position(self):
        return tuple(row[3] for row in self.matrix[:3])

    @property
    def forward(self):
        return tuple(round(row[2], 7) for row in self.matrix[:3])

    def anchor(self, name):
        anchors = [
            node
            for node in scene_model(self.definition.asset.path).nodes
            if node.name == f"Anchor.{name}"
        ]
        require(
            len(anchors) == 1 and not anchors[0].meshes,
            f"{self.identity}: missing or invalid anchor {name}",
        )
        return np.asarray(self.matrix) @ anchors[0].world_matrix


class SceneRegistry:
    """Read-only resolved starting placements, in parent-before-child order."""

    def __init__(self, definition):
        instances = {}
        for entity in definition.entities:
            matrix = transform(entity)
            if entity.parent:
                parent = instances[entity.parent]
                basis = parent.anchor(entity.anchor) if entity.anchor else np.asarray(parent.matrix)
                matrix = basis @ matrix
            require(np.all(np.isfinite(matrix)), f"{entity.identity}: nonfinite placement")
            if entity.asset.kind in ACTOR_KINDS:
                basis = matrix[:3, :3]
                require(
                    np.allclose(basis.T @ basis, np.eye(3), atol=1e-6)
                    and np.allclose(basis[:, 1], (0, 1, 0), atol=1e-6),
                    f"{entity.identity}: gameplay placement requires upright unit scale",
                )
            instances[entity.identity] = Placement(entity, tuple(map(tuple, matrix)))
        self.instances = MappingProxyType(instances)
        self.definition = definition

    def of_kind(self, *kinds):
        return tuple(p for p in self.instances.values() if p.definition.asset.kind in kinds)


@lru_cache(maxsize=1)
def default_scene():
    return SceneRegistry(default_definition())


def environment_geometry(placement):
    """Yield render triangles, marked surfaces, and collision boxes in world coordinates."""
    model = scene_model(placement.definition.asset.path)
    vertices, surfaces, obstacles = [], [], []
    markers = set()
    for node in model.nodes:
        name = node.name
        matrix = np.asarray(placement.matrix) @ node.world_matrix
        if name.startswith(("Anchor.", "Collision.", "Surface.")):
            require(name not in markers, f"{placement.identity}: duplicate marker {name}")
            markers.add(name)
        if name.startswith("Anchor."):
            require(not node.meshes, f"{name}: anchor must be an empty")
            continue
        if name.startswith("Collision."):
            require(not node.meshes, f"{name}: collision marker must be an empty")
            box = points(matrix, CUBE)
            lower, upper = box.min(axis=0), box.max(axis=0)
            require(
                abs(np.linalg.det(matrix[:3, :3])) > 1e-10,
                f"{name}: collision box must have positive volume",
            )
            obstacles.append((tuple((lower + upper) / 2), tuple(upper - lower)))
            continue
        if name.startswith("Surface."):
            kind = "apron" if name == "Surface.Floor" else "marking"
            require(
                node.meshes and (name == "Surface.Floor" or name.startswith("Surface.Marking.")),
                f"{name}: invalid ground surface marker",
            )
            mesh = np.concatenate([model.primitives[i].positions for i in node.meshes])
            projected = points(matrix, mesh)
            lo, hi = projected.min(axis=0), projected.max(axis=0)
            require(
                np.allclose(projected[:, 1], 0, atol=1e-5)
                and hi[0] > lo[0]
                and hi[2] > lo[2]
                and all(
                    np.all(
                        np.isclose(projected[:, axis], lo[axis], atol=1e-5, rtol=0)
                        | np.isclose(projected[:, axis], hi[axis], atol=1e-5, rtol=0)
                    )
                    for axis in (0, 2)
                )
                and len(np.unique(np.round(projected[:, (0, 2)], 5), axis=0)) == 4,
                f"{name}: expected a horizontal, axis-aligned ground rectangle at Y=0",
            )
            surfaces.append((tuple(((lo + hi) / 2)[[0, 2]]), tuple((hi - lo)[[0, 2]]), kind))
            continue
        for index in node.meshes:
            mesh = model.primitives[index]
            transformed = points(matrix, mesh.positions)
            triangles = transformed if mesh.indices is None else transformed[mesh.indices.ravel()]
            vertices.extend((*p, *mesh.material.base_color[:3]) for p in triangles)
    require(vertices and obstacles, f"{placement.identity}: asset needs geometry and collision")
    return vertices, surfaces, obstacles
