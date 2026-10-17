"""Original meter-scale folded airframe; deterministic standard-library GLB source."""

import json
import math
import struct
from pathlib import Path

OUTPUT = Path(__file__).resolve().parents[1] / "assets/models/vehicles/tc167_prototype.glb"


def build():
    """Generate and write the original hierarchical TC167 GLB prototype asset."""
    nodes = []
    meshes = []
    views = []
    accessors = []
    binary = bytearray()
    palette = [
        ("Airframe", (0.62, 0.66, 0.69, 1)),
        ("Panels", (0.43, 0.49, 0.54, 1)),
        ("Canopy", (0.08, 0.16, 0.23, 1)),
        ("Mechanisms", (0.12, 0.15, 0.17, 1)),
        ("Markings", (0.55, 0.25, 0.12, 1)),
    ]

    def node(name, parent=None, pivot=(0, 0, 0)):
        """Append a named local pivot and optionally attach it to a parent.

        Args:
            name: Model node or state name.
            parent: Optional parent node index.
            pivot: Local joint translation in meters.

        Returns:
            New zero-based node index.
        """
        i = len(nodes)
        nodes.append({"name": name, "translation": list(pivot)})
        if parent is not None:
            nodes[parent].setdefault("children", []).append(i)
        return i

    def geometry(owner, points, faces, material=0):
        # Flat face normals: readable panel facets without introducing lighting.
        """Triangulate faceted geometry and append glTF buffers, accessors, and a visual child.

        Args:
            owner: Index of the unscaled model joint receiving a visual geometry child.
            points: Local XYZ vertex positions in meters.
            faces: Ordered face vertex-index sequences.
            material: Zero-based material index.
        """
        vertices = []
        normals = []
        indices = []
        for face in faces:
            a, b, c = [points[i] for i in face[:3]]
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(x * x for x in n))
            n = [x / length for x in n] if length else [0, 1, 0]
            start = len(vertices)
            vertices.extend(points[i] for i in face)
            normals.extend([n] * len(face))
            for j in range(1, len(face) - 1):
                indices.extend((start, start + j, start + j + 1))
        attrs = {}
        for name, data in [("POSITION", vertices), ("NORMAL", normals)]:
            while len(binary) % 4:
                binary.append(0)
            offset = len(binary)
            binary.extend(b"".join(struct.pack("<3f", *p) for p in data))
            view = len(views)
            views.append(
                {"buffer": 0, "byteOffset": offset, "byteLength": len(data) * 12, "target": 34962}
            )
            acc = {"bufferView": view, "componentType": 5126, "count": len(data), "type": "VEC3"}
            if name == "POSITION":
                acc.update(
                    min=[min(p[i] for p in data) for i in range(3)],
                    max=[max(p[i] for p in data) for i in range(3)],
                )
            attrs[name] = len(accessors)
            accessors.append(acc)
        offset = len(binary)
        binary.extend(struct.pack("<" + str(len(indices)) + "H", *indices))
        view = len(views)
        views.append(
            {"buffer": 0, "byteOffset": offset, "byteLength": len(indices) * 2, "target": 34963}
        )
        index = len(accessors)
        accessors.append(
            {"bufferView": view, "componentType": 5123, "count": len(indices), "type": "SCALAR"}
        )
        meshes.append(
            {
                "name": nodes[owner]["name"] + "Shell",
                "primitives": [{"attributes": attrs, "indices": index, "material": material}],
            }
        )
        # Geometry companions keep the actual moving joint unscaled and addressable.
        visual = node(nodes[owner]["name"] + "Surface" + str(len(meshes)), owner)
        nodes[visual]["mesh"] = len(meshes) - 1

    def loft(owner, rings, material=0, segments=12, rectangular=False):
        # Elliptical longitudinal shells, source +Z points toward the nose.
        """Build capped elliptical or flattened longitudinal rings and append their geometry.

        Args:
            owner: Index of the unscaled model joint receiving a visual geometry child.
            rings: Longitudinal (Z, center X, center Y, radius X, radius Y) ring tuples in
                meters.
            material: Zero-based material index.
            segments: Number of vertices around each ring.
            rectangular: Whether to flatten the ring profile toward a rectangle.
        """

        def profile(value):
            return math.copysign(abs(value) ** 0.3, value) if rectangular else value

        points = [
            (
                cx + rx * profile(math.cos(i * 2 * math.pi / segments)),
                cy + ry * profile(math.sin(i * 2 * math.pi / segments)),
                z,
            )
            for z, cx, cy, rx, ry in rings
            for i in range(segments)
        ]
        faces = [
            tuple(reversed(range(segments))),
            tuple((len(rings) - 1) * segments + i for i in range(segments)),
        ]
        for r in range(len(rings) - 1):
            for i in range(segments):
                j = (i + 1) % segments
                faces.append(
                    (
                        r * segments + i,
                        r * segments + j,
                        (r + 1) * segments + j,
                        (r + 1) * segments + i,
                    )
                )
        geometry(owner, points, faces, material)

    def slab(owner, outline, thickness, material=0):
        """Extrude a local X/Z outline through its Y thickness and append the resulting geometry.

        Args:
            owner: Index of the unscaled model joint receiving a visual geometry child.
            outline: Local X/Z polygon vertices in meters.
            thickness: Extrusion thickness in meters.
            material: Zero-based material index.
        """
        points = [(x, y, z) for y in (-thickness / 2, thickness / 2) for x, z in outline]
        k = len(outline)
        faces = [tuple(reversed(range(k))), tuple(range(k, 2 * k))]
        faces.extend((i, (i + 1) % k, (i + 1) % k + k, i + k) for i in range(k))
        geometry(owner, points, faces, material)

    root = node("TC167Root")
    body = node("Fuselage", root)
    loft(
        body,
        [(-8, 0, 0, 0.65, 0.55), (-5, 0, 0, 1.2, 0.8), (0, 0, 0, 1.35, 0.9), (3.8, 0, 0, 0.9, 0.7)],
        0,
    )
    nose = node("Nose", body, (0, 0, 3.8))
    loft(
        nose,
        [
            (0, 0, 0, 0.9, 0.7),
            (2.5, 0, -0.12, 0.65, 0.5),
            (4.2, 0, -0.2, 0.35, 0.28),
            (5.7, 0, -0.25, 0.035, 0.035),
        ],
        0,
    )
    canopy = node("Cockpit", body, (0, 0.55, 2))
    loft(
        canopy,
        [
            (-1.7, 0, 0, 0.55, 0.12),
            (-0.7, 0, 0.25, 0.65, 0.65),
            (1, 0, 0.2, 0.48, 0.5),
            (2.1, 0, 0, 0.1, 0.08),
        ],
        2,
    )
    torso = node("TorsoCore", body, (0, -0.2, -0.6))
    loft(torso, [(-1.5, 0, 0, 1, 0.5), (1.5, 0, 0, 1.05, 0.5)], 1)
    head = node("Head", body, (0, 0.35, -0.9))
    loft(head, [(-0.35, 0, 0, 0.3, 0.32), (0.35, 0, 0, 0.3, 0.32)], 3)
    for label, side in [("Left", 1), ("Right", -1)]:
        wing = node(label + "Wing", root, (side * 1.3, 0, 0.5))
        slab(
            wing,
            [(0, 1.2), (side * 4.7, -2), (side * 4.7, -3.1), (side * 2.4, -3.7), (0, -3.2)],
            0.22,
            1,
        )
        tail = node(label + "Tail", root, (side * 0.8, 0.3, -6))
        slab(tail, [(0, 0.8), (side * 2.4, -0.5), (side * 2.6, -1.7), (0, -1.3)], 0.16, 0)
        intake = node(label + "Intake", root, (side * 1.6, -0.1, 2))
        loft(
            intake,
            [(-1.6, 0, 0, 0.75, 0.72), (-0.25, 0, 0, 0.82, 0.82), (0.25, 0, 0, 0.85, 0.85)],
            1,
            rectangular=True,
        )
        # Dark intake face and lip; legs continue aft as nacelle panels.
        loft(intake, [(0.26, 0, 0, 0.68, 0.67), (0.27, 0, 0, 0.68, 0.67)], 3, rectangular=True)
        # Thin inboard splitter/ramp: a separate lip carried by the intake joint.
        splitter = [
            (x, y, z)
            for x in (-side * 0.78 - 0.06, -side * 0.78 + 0.06)
            for y in (-0.85, 0.85)
            for z in (-1.1, 0.55)
        ]
        geometry(
            intake,
            splitter,
            [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)],
            0,
        )
        upper = node(label + "UpperLeg", intake, (0, 0, -1.5))
        loft(upper, [(-2.6, 0, 0, 0.75, 0.7), (0, 0, 0, 0.75, 0.7)], 0)
        lower = node(label + "LowerLeg", upper, (0, 0, -2.6))
        loft(lower, [(-2.6, -side * 0.25, 0.05, 0.7, 0.65), (0, 0, 0, 0.75, 0.7)], 1)
        foot = node(label + "Foot", lower, (0, 0, -2.6))
        loft(foot, [(-1.8, -side * 0.6, 0.1, 0.58, 0.5), (0, -side * 0.25, 0.05, 0.7, 0.65)], 0)
        engine = node(label + "Engine", root, (side * 1.0, 0, -7.8))
        loft(
            engine, [(-1.7, 0, 0, 0.58, 0.5), (-0.3, 0, 0, 0.63, 0.55), (0, 0, 0, 0.58, 0.5)], 3, 20
        )
        loft(engine, [(-1.71, 0, 0, 0.43, 0.36), (-1.72, 0, 0, 0.43, 0.36)], 4, 20)
        node(label + "EngineExhaust", engine, (0, 0, -1.72))
        shoulder = node(label + "Shoulder", root, (side * 0.95, -0.5, 1.3))
        arm = node(label + "UpperArm", shoulder)
        loft(arm, [(-2, 0, 0, 0.32, 0.3), (0, 0, 0, 0.32, 0.3)], 1)
        forearm = node(label + "Forearm", arm, (0, 0, -2))
        loft(forearm, [(-1.8, 0, 0, 0.34, 0.32), (0, 0, 0, 0.32, 0.3)], 0)
        hand = node(label + "Hand", forearm, (0, 0, -1.8))
        loft(hand, [(-0.65, 0, 0, 0.27, 0.25), (0, 0, 0, 0.3, 0.28)], 3)
        for number, x in [("01", 2.6), ("02", 3.8)]:
            node("Missile" + label + number, wing, (side * (x - 1.3), -0.35, -1.4))
    fin = node("VerticalTail", root, (0, 0.6, -5.8))
    geometry(
        fin,
        [
            (-0.09, 0, 0),
            (0.09, 0, 0),
            (-0.06, 2.65, -1.5),
            (0.06, 2.65, -1.5),
            (-0.05, 2.4, -2.8),
            (0.05, 2.4, -2.8),
            (-0.09, 0, -2.6),
            (0.09, 0, -2.6),
        ],
        [(0, 2, 4, 6), (1, 7, 5, 3), (0, 1, 3, 2), (2, 3, 5, 4), (4, 5, 7, 6), (6, 7, 1, 0)],
        1,
    )
    node("GunMount", root, (0, -0.65, 4.2))
    doc = {
        "asset": {
            "version": "2.0",
            "generator": "Project Tango Charlie original folded airframe generator",
        },
        "scene": 0,
        "scenes": [{"name": "FighterRestPose", "nodes": [root]}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": [
            {
                "name": name,
                "pbrMetallicRoughness": {
                    "baseColorFactor": color,
                    "metallicFactor": 0,
                    "roughnessFactor": 0.8,
                },
            }
            for name, color in palette
        ],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": views,
        "accessors": accessors,
    }
    encoded = json.dumps(doc, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    binary.extend(b"\0" * (-len(binary) % 4))
    return (
        struct.pack("<4sII", b"glTF", 2, 28 + len(encoded) + len(binary))
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
        + struct.pack("<II", len(binary), 0x004E4942)
        + binary
    )


if __name__ == "__main__":
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(build())
    print(OUTPUT)
