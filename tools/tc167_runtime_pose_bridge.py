"""Explicit offline Blender-GLB → runtime PoseTransform candidate; never installs gameplay data."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import load_glb
from game.flight_state import VehicleMode
from game.tc167_poses import PoseTransform
from game.transformation import TransformationController

CONFIGURATIONS = {
    "FIGHTER": "TC_Pose_Fighter",
    "VTOL": "TC_Pose_VTOL",
    "BATTLEDROID": "TC_Pose_Battledroid",
}
EDGES = {
    "TC_FighterToVTOL": ("FIGHTER", "VTOL"),
    "TC_VTOLToFighter": ("VTOL", "FIGHTER"),
    "TC_VTOLToBattledroid": ("VTOL", "BATTLEDROID"),
    "TC_BattledroidToVTOL": ("BATTLEDROID", "VTOL"),
}
PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT / "assets/models/vehicles/tc167.glb"


def matrices(model, clip, time):
    player = AnimationPlayer(model)
    player.play(clip, loop=False)
    player.time = time
    player.apply()
    return {n.name: n.local_matrix.copy() for n in model.nodes}


def apply_candidate(model, candidate, configuration):
    """Offline full-local pose application, including engines. Not the procedural controller."""
    model.reset_pose()
    for node in model.nodes:
        node.set_animation_pose()
        pose = candidate["configurations"][configuration][node.name]
        transform = PoseTransform(
            tuple(pose["translation"]), tuple(pose["rotation"]), tuple(pose["scale"])
        )
        node.set_pose(transform.translation, transform.rotation, transform.scale)
    model.world_matrices()


def build(path):
    model, original = load_glb(path), load_glb(SOURCE)
    if len(model.nodes) != len(original.nodes):
        raise ValueError("Node inventory differs from production")
    for a, b in zip(model.nodes, original.nodes):
        if (
            a.name != b.name
            or (a.parent.name if a.parent else None) != (b.parent.name if b.parent else None)
            or not np.array_equal(a.base_matrix, b.base_matrix)
        ):
            raise ValueError("Hierarchy or immutable production rest changed: " + a.name)
    candidate = {
        "schema": "tc167-runtime-pose-candidate-v1",
        "coordinates": "glTF parent-local +Y up +Z nose; xyzw quaternion",
        "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "production_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "parents": {n.name: n.parent.name if n.parent else None for n in model.nodes},
        "configurations": {},
    }
    report = {"endpoint_errors": {}, "roundtrip_errors": {}, "python_configuration_errors": {}}
    references = {}
    for config, clip_name in CONFIGURATIONS.items():
        clip = model.animations[clip_name]
        sample = clip.sample(0)
        candidate["configurations"][config] = {
            node.name: {
                path: [float(v) for v in sample.get((node, path), node.base_trs[index])]
                for index, path in enumerate(("translation", "rotation", "scale"))
            }
            for node in model.nodes
        }
        references[config] = matrices(model, clip_name, 0)
    for name, configs in EDGES.items():
        clip = model.animations[name]
        for config, time in zip(configs, (0, clip.duration)):
            actual = matrices(model, name, time)
            error = max(float(np.max(np.abs(actual[n] - references[config][n]))) for n in actual)
            report["endpoint_errors"][name + ":" + config] = error
            if error > 2e-5:
                raise ValueError(f"{name} {config}: endpoint matrix mismatch {error}")
    # JSON → actual runtime PoseTransform → model TRS round trip, independently of glTF player.
    decoded = json.loads(json.dumps(candidate, allow_nan=False))
    controller = TransformationController(model=load_glb(SOURCE))
    for config in CONFIGURATIONS:
        apply_candidate(model, decoded, config)
        error = max(
            float(np.max(np.abs(n.local_matrix - references[config][n.name]))) for n in model.nodes
        )
        report["roundtrip_errors"][config] = error
        if error > 2e-5:
            raise ValueError(f"{config}: runtime candidate round trip failed: {error}")
        controller.reset(VehicleMode[config])
        report["python_configuration_errors"][config] = {
            n.name: float(np.max(np.abs(n.local_matrix - references[config][n.name])))
            for n in controller.model.nodes
            if np.max(np.abs(n.local_matrix - references[config][n.name])) > 2e-5
        }
    return decoded, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("glb", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--python-output", type=Path)
    args = parser.parse_args()
    # Candidate output is intentionally outside the runtime module tree.
    for path in (args.output, args.python_output):
        if path and any(
            path.resolve().is_relative_to(PROJECT / root) for root in ("game", "engine", "assets")
        ):
            parser.error("Candidate outputs must be outside game/, engine/, and assets/")
    candidate, report = build(args.glb)
    args.output.write_text(
        json.dumps({"candidate": candidate, "validation": report}, indent=2, allow_nan=False) + "\n"
    )
    if args.python_output:
        args.python_output.write_text(
            '"""Offline validated pose candidate. Explicit developer integration required."""\n'
            "from game.tc167_poses import PoseTransform\n"
            "from game.flight_state import VehicleMode\n\nPOSES = {\n"
            + "".join(
                f"    VehicleMode.{config}: {{\n"
                + "".join(
                    f"        {node!r}: PoseTransform({tuple(pose['translation'])!r}, "
                    f"{tuple(pose['rotation'])!r}, {tuple(pose['scale'])!r}),\n"
                    for node, pose in nodes.items()
                )
                + "    },\n"
                for config, nodes in candidate["configurations"].items()
            )
            + "}\n"
        )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
