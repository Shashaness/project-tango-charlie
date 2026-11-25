#!/usr/bin/env python3
"""Build an original open-cab utility Jeep as a rigid-node GLB."""

import argparse
import json
import math
import struct
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FACES = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
SHADING = (0.76, 0.82, 0.60, 1.0, 0.65, 0.92)


def box(groups, center, size, color):
    points = [
        np.asarray(center) + (np.array((x, y, z)) - 0.5) * size
        for x in (0, 1)
        for y in (0, 1)
        for z in (0, 1)
    ]
    for face, shade in zip(FACES, SHADING):
        target = groups[tuple(channel * shade for channel in color)]
        target.extend(points[face[k]] for k in (0, 1, 2, 0, 2, 3))


def wheel(groups, center=(0, 0, 0), axis=0):
    for i in range(20):
        a, b = (math.tau * j / 20 for j in (i, i + 1))
        colors = (0.065, 0.068, 0.065) if i % 2 else (0.10, 0.105, 0.10)

        def point(axial, radial, angle):
            local = (axial, radial * math.cos(angle), radial * math.sin(angle))
            return np.asarray(center) + np.roll(local, axis)

        tread = groups[colors]
        left_a, left_b = point(-0.205, 0.43, a), point(-0.205, 0.43, b)
        right_a, right_b = point(0.205, 0.43, a), point(0.205, 0.43, b)
        tread.extend((left_a, left_b, right_a, left_b, right_b, right_a))
        for side in (-1, 1):
            outer = side * 0.205
            for color, radius, depth in (
                ((0.085, 0.09, 0.085), 0.43, outer),
                ((0.37, 0.39, 0.32) if i % 4 else (0.64, 0.65, 0.56), 0.22, outer * 1.02),
                ((0.17, 0.18, 0.16), 0.095, outer * 1.04),
            ):
                face = groups[color]
                face.extend((point(depth, 0, a), point(depth, radius, a), point(depth, radius, b)))


def build():
    olive, dark, rubber = (0.39, 0.46, 0.29), (0.22, 0.28, 0.20), (0.09, 0.10, 0.085)
    ivory, glass = (0.94, 0.86, 0.60), (0.25, 0.40, 0.44)
    body, tire = defaultdict(list), defaultdict(list)
    shapes = [
        ((0, 0, 0), (1.35, 0.24, 3.8), dark),
        ((0, 0.22, 0.15), (1.52, 0.31, 3.35), olive),
        ((0, 0.41, 1.13), (1.45, 0.31, 1.46), olive),
        ((0, 0.40, 1.87), (1.39, 0.56, 0.12), dark),
        ((0, 0.10, 1.99), (1.95, 0.17, 0.18), rubber),
        ((0, 0.15, -1.98), (1.86, 0.17, 0.18), rubber),
        ((0, 0.41, -1.72), (1.58, 0.40, 0.13), olive),
        ((0, 0.63, 0.35), (1.53, 0.10, 0.12), olive),
        ((0, 0.96, 0.37), (1.53, 0.065, 0.071), dark),
        ((0, 0.795, 0.38), (1.37, 0.26, 0.028), glass),
    ]
    for sign in (-1, 1):
        shapes.extend(
            [
                ((sign * 0.74, 0.38, -0.92), (0.15, 0.38, 1.53), olive),
                ((sign * 0.73, 0.80, 0.37), (0.055, 0.36, 0.077), dark),
                ((sign * 0.41, 0.30, -0.38), (0.45, 0.13, 0.53), rubber),
                ((sign * 0.41, 0.49, -0.63), (0.44, 0.43, 0.11), rubber),
                ((sign * 0.57, 0.52, 1.95), (0.22, 0.18, 0.045), ivory),
                ((sign * 0.68, 0.44, -1.8), (0.17, 0.12, 0.035), (0.71, 0.16, 0.10)),
                ((sign * 0.85, 0.46, 1.27), (0.32, 0.12, 1.10), olive),
                ((sign * 0.85, 0.46, -1.35), (0.31, 0.12, 0.99), olive),
                ((sign * 0.87, 0.70, 0.41), (0.18, 0.10, 0.08), dark),
            ]
        )
    for i in range(-3, 4):
        shapes.append(((i * 0.106, 0.42, 1.944), (0.043, 0.30, 0.024), rubber))
    for center, size, color in shapes:
        box(body, center, size, color)
    wheel(body, (0.04, 0.44, -1.77), axis=2)
    wheel(tire)
    binary = bytearray()
    views, accessors, materials = [], [], []

    def mesh(groups):
        primitives = []
        for color, vertices in groups.items():
            array = np.asarray(vertices, dtype="<f4")
            views.append({"buffer": 0, "byteOffset": len(binary), "byteLength": array.nbytes})
            binary.extend(array.tobytes())
            accessors.append(
                {
                    "bufferView": len(views) - 1,
                    "componentType": 5126,
                    "count": len(array),
                    "type": "VEC3",
                    "min": array.min(0).tolist(),
                    "max": array.max(0).tolist(),
                }
            )
            materials.append(
                {
                    "doubleSided": True,
                    "pbrMetallicRoughness": {"baseColorFactor": [*color, 1.0], "metallicFactor": 0},
                }
            )
            primitives.append(
                {"attributes": {"POSITION": len(accessors) - 1}, "material": len(materials) - 1}
            )
        return {"primitives": primitives}

    meshes = [mesh(body), mesh(tire)]
    nodes = [{"name": "Jeep", "mesh": 0, "children": []}]
    for name, point in (
        ("DriverSeat", (0.41, 0.38, -0.38)),
        ("Entry.L", (1.04, 0.65, 0)),
        ("Entry.R", (-1.04, 0.65, 0)),
        ("Exit.L", (1.59, -0.616, 0)),
        ("Exit.R", (-1.59, -0.616, 0)),
    ):
        nodes[0]["children"].append(len(nodes))
        nodes.append({"name": name, "translation": point})
    for name, x, z in (
        ("FL", 0.825, 1.325),
        ("FR", -0.825, 1.325),
        ("RL", 0.825, -1.325),
        ("RR", -0.825, -1.325),
    ):
        nodes[0]["children"].append(len(nodes))
        nodes.extend(
            (
                {"name": "Hub." + name, "translation": (x, -0.19, z), "children": [len(nodes) + 1]},
                {"name": "Wheel." + name, "mesh": 1},
            )
        )
    doc = {
        "asset": {"version": "2.0", "generator": "Project Tango Charlie utility Jeep"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "bufferViews": views,
        "accessors": accessors,
        "buffers": [{"byteLength": len(binary)}],
    }
    text = json.dumps(doc, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    return (
        struct.pack("<4sII", b"glTF", 2, 28 + len(text) + len(binary))
        + struct.pack("<I4s", len(text), b"JSON")
        + text
        + struct.pack("<I4s", len(binary), b"BIN\0")
        + binary
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Legacy jeep bootstrap for scratch exports.")
    parser.add_argument("--output", type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if output.suffix.lower() != ".glb" or output.is_relative_to(ROOT / "assets"):
        parser.error("Choose a new .glb outside assets. Export production assets from Blender.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(build())
