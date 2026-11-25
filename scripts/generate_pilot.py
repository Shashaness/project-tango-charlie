#!/usr/bin/env python3
"""Build the original low-poly +Y-up, +Z-forward TC167 pilot placeholder."""

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BONES = [
    ("Hips", None, (0, 0.94, 0)),
    ("Spine", "Hips", (0, 1.12, 0)),
    ("Chest", "Spine", (0, 1.36, 0)),
    ("Neck", "Chest", (0, 1.53, 0)),
    ("Head", "Neck", (0, 1.63, 0)),
]
for side, sign in (("L", 1), ("R", -1)):
    BONES.extend(
        [
            ("UpperArm." + side, "Chest", (sign * 0.23, 1.42, 0)),
            ("Forearm." + side, "UpperArm." + side, (sign * 0.27, 1.16, 0)),
            ("Hand." + side, "Forearm." + side, (sign * 0.29, 0.96, 0)),
            ("Thigh." + side, "Hips", (sign * 0.105, 0.93, 0)),
            ("Shin." + side, "Thigh." + side, (sign * 0.105, 0.51, 0)),
            ("Foot." + side, "Shin." + side, (sign * 0.105, 0.10, 0)),
        ]
    )

COLORS = [(0.17, 0.28, 0.26), (0.37, 0.52, 0.37), (0.09, 0.11, 0.14), (0.24, 0.63, 0.76)]
FACES = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))


def build():
    names = [name for name, _, _ in BONES]
    positions = {name: np.asarray(pos) for name, _, pos in BONES}
    buffer = bytearray()
    views = []
    accessors = []

    def accessor(data, kind, code=5126, bounds=False):
        dtype = {5126: "<f4", 5123: "<u2"}[code]
        array = np.asarray(data, dtype=dtype)
        payload = array.tobytes()
        views.append({"buffer": 0, "byteOffset": len(buffer), "byteLength": len(payload)})
        buffer.extend(payload)
        buffer.extend(b"\0" * (-len(buffer) % 4))
        info = {
            "bufferView": len(views) - 1,
            "componentType": code,
            "count": len(array),
            "type": kind,
        }
        if bounds:
            info.update(
                min=np.min(array, axis=0).reshape(-1).tolist(),
                max=np.max(array, axis=0).reshape(-1).tolist(),
            )
        accessors.append(info)
        return len(accessors) - 1

    nodes = []
    inverse = []
    for name, parent, position in BONES:
        relative = positions[name] - positions[parent] if parent else positions[name]
        nodes.append({"name": name, "translation": relative.tolist()})
        matrix = np.eye(4)
        matrix[:3, -1] = -np.asarray(position)
        inverse.append(matrix.T.ravel())
    for i, (_, parent, _) in enumerate(BONES):
        if parent:
            nodes[names.index(parent)].setdefault("children", []).append(i)

    primitives = []
    parts = [
        ("Hips", (0, 0.95, 0), (0.36, 0.24, 0.25), 0),
        ("Spine", (0, 1.16, 0), (0.34, 0.29, 0.24), 0),
        ("Chest", (0, 1.37, 0), (0.44, 0.25, 0.31), 1),
        ("Chest", (0, 1.34, -0.18), (0.30, 0.28, 0.12), 2),
        ("Neck", (0, 1.54, 0), (0.13, 0.12, 0.13), 2),
        ("Head", (0, 1.67, 0.008), (0.23, 0.26, 0.26), 1),
        ("Head", (0, 1.66, 0.138), (0.19, 0.10, 0.03), 3),
    ]
    for side, sign in (("L", 1), ("R", -1)):
        for bone, center, size, color in (
            ("UpperArm", (0.24, 1.31, 0), (0.13, 0.27, 0.17), 1),
            ("Forearm", (0.275, 1.07, 0), (0.12, 0.24, 0.14), 0),
            ("Hand", (0.29, 0.94, 0.015), (0.105, 0.10, 0.12), 2),
            ("Thigh", (0.105, 0.73, 0), (0.18, 0.43, 0.22), 0),
            ("Shin", (0.105, 0.32, 0), (0.16, 0.44, 0.17), 1),
            ("Foot", (0.105, 0.077, 0.065), (0.18, 0.154, 0.28), 2),
        ):
            parts.append((bone + "." + side, (sign * center[0], *center[1:]), size, color))
    materials = []
    for base in COLORS:
        for shade in (0.72, 0.85, 0.62, 1, 0.64, 0.93):
            materials.append(
                {
                    "pbrMetallicRoughness": {
                        "baseColorFactor": [*(shade * np.asarray(base)), 1],
                        "metallicFactor": 0,
                    }
                }
            )
    groups = [[] for _ in materials]
    for bone, center, size, color in parts:
        corners = [
            np.asarray(center) + np.asarray(size) * (np.array((x, y, z)) - 0.5)
            for x in (0, 1)
            for y in (0, 1)
            for z in (0, 1)
        ]
        for i, face in enumerate(FACES):
            groups[color * 6 + i].extend(
                (corners[face[k]], names.index(bone)) for k in (0, 1, 2, 0, 2, 3)
            )
    for material, group in enumerate(groups):
        pos, joints = zip(*group)
        primitives.append(
            {
                "material": material,
                "attributes": {
                    "POSITION": accessor(pos, "VEC3", bounds=True),
                    "JOINTS_0": accessor([(j, 0, 0, 0) for j in joints], "VEC4", 5123),
                    "WEIGHTS_0": accessor([(1, 0, 0, 0)] * len(joints), "VEC4"),
                },
            }
        )
    animations = []
    for name, length, amplitude in (
        ("Idle", 2.4, 0.035),
        ("Walk", 0.8, 0.52),
        ("Run", 0.56, 0.88),
        ("Turn", 0.5, 0.20),
    ):
        phases = np.linspace(0, 1, 9)
        input_index = accessor([length * phase for phase in phases], "SCALAR", bounds=True)
        samplers, channels = [], []

        def track(bone, angles, axis):
            values = []
            for angle in angles:
                quat = [0.0, 0.0, 0.0, math.cos(angle / 2)]
                quat[axis] = math.sin(angle / 2)
                values.append(quat)
            channels.append(
                {
                    "sampler": len(samplers),
                    "target": {"node": names.index(bone), "path": "rotation"},
                }
            )
            samplers.append({"input": input_index, "output": accessor(values, "VEC4")})

        wave = np.sin(math.tau * phases)
        track("Chest", amplitude * wave * (0.2 if name != "Idle" else 1), 1)
        for side, sign in (("L", 1), ("R", -1)):
            if name in ("Walk", "Run"):
                # Negative hip X swings the leg forward. Positive shin X bends
                # the lower leg back. Start in stance and lift during swing.
                pace = 1.45 if name == "Run" else 1.0
                hip = np.radians((-23, -12, 0, 15, 24, 9, -17, -28, -23)) * pace
                knee = np.radians((4, 14, 9, 5, 12, 52, 58, 26, 4)) * pace
                offset = 0 if side == "L" else 4
                hip = np.roll(hip[:-1], offset)
                knee = np.roll(knee[:-1], offset)
                hip = np.append(hip, hip[0])
                knee = np.append(knee, knee[0])
            else:
                hip = amplitude * wave * sign
                knee = np.maximum(-hip, 0) * 0.7
            track("Thigh." + side, hip, 0)
            track("Shin." + side, knee, 0)
            if name in ("Walk", "Run"):
                track("Foot." + side, -hip - knee, 0)
            track("UpperArm." + side, -hip * 0.75, 0)
            track("Forearm." + side, np.maximum(hip, 0) * 0.6, 0)
        animations.append({"name": name, "channels": channels, "samplers": samplers})

    nodes.append({"name": "PilotMesh", "mesh": 0, "skin": 0})
    skin = {
        "joints": list(range(len(BONES))),
        "skeleton": 0,
        "inverseBindMatrices": accessor(inverse, "MAT4"),
    }
    document = {
        "asset": {"version": "2.0", "generator": "Project Tango Charlie placeholder"},
        "scene": 0,
        "scenes": [{"nodes": [0, len(BONES)]}],
        "nodes": nodes,
        "meshes": [{"name": "Pilot", "primitives": primitives}],
        "skins": [skin],
        "materials": materials,
        "animations": animations,
        "bufferViews": views,
        "accessors": accessors,
        "buffers": [{"byteLength": len(buffer)}],
    }
    text = json.dumps(document, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    return (
        struct.pack("<4sII", b"glTF", 2, 28 + len(text) + len(buffer))
        + struct.pack("<I4s", len(text), b"JSON")
        + text
        + struct.pack("<I4s", len(buffer), b"BIN\0")
        + buffer
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Legacy pilot bootstrap for scratch exports.")
    parser.add_argument("--output", type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if output.suffix.lower() != ".glb" or output.is_relative_to(ROOT / "assets"):
        parser.error("Choose a new .glb outside assets. Export production assets from Blender.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(build())
