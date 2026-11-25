"""Validate Blender-authored pilot and Jeep GLBs with the game loaders."""

import argparse
import sys
from pathlib import Path

import numpy as np

from engine.jeep_asset import contract_components, load_jeep
from engine.pilot_asset import PilotAsset
from game.ground_vehicle import DriveParameters

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = {
    "pilot": ROOT / "assets/models/characters/pilot.glb",
    "jeep": ROOT / "assets/models/vehicles/jeep.glb",
}
CLIPS = {"Idle", "Walk", "Run", "Turn"}
PARENTS = {
    "Hips": None,
    "Spine": "Hips",
    "Chest": "Spine",
    "Neck": "Chest",
    "Head": "Neck",
}
for side in ("L", "R"):
    PARENTS.update(
        {
            f"Clavicle.{side}": "Chest",
            f"UpperArm.{side}": f"Clavicle.{side}",
            f"Forearm.{side}": f"UpperArm.{side}",
            f"Hand.{side}": f"Forearm.{side}",
            f"Thigh.{side}": "Hips",
            f"Shin.{side}": f"Thigh.{side}",
            f"Foot.{side}": f"Shin.{side}",
        }
    )
WHEELS = ("FL", "FR", "RL", "RR")
ANCHORS = ("DriverSeat", "Entry.L", "Entry.R", "Exit.L", "Exit.R")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def named(model, name):
    try:
        return model.find_node(name)
    except (KeyError, ValueError) as error:
        raise ValueError(f"Expected exactly one node named {name}") from error


def hinge_angle(node, axis):
    """Signed rotational component about a character-space hinge in bind-local axes."""
    relative = node.base_matrix[:3, :3].T @ node.local_matrix[:3, :3]
    skew = np.array(
        (
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        )
    )
    return np.arctan2(skew @ axis, np.trace(relative) - 1)


def check_anatomy(asset):
    asset.reset_locomotion()
    model = asset.model
    nodes = {node.name: node for node in asset.joints}
    bind = np.asarray([joint.world_matrix for joint in asset.joints])
    require(
        np.allclose(bind @ asset.inverse, np.eye(4), atol=2e-5),
        "Pilot inverse bind matrices do not match the exported rest pose",
    )
    positions = {name: node.world_matrix[:3, 3] for name, node in nodes.items()}
    hips = positions["Hips"]
    head = positions["Head"]
    require(
        0.7 < hips[1] < 1.05 and head[1] - hips[1] > 0.6,
        "Place Hips at pelvis height beneath the spine and head",
    )
    for side in ("L", "R"):
        for first, second, length in (
            ("Clavicle", "UpperArm", 0.12),
            ("UpperArm", "Forearm", 0.20),
            ("Forearm", "Hand", 0.16),
            ("Thigh", "Shin", 0.32),
            ("Shin", "Foot", 0.32),
        ):
            parent, child = f"{first}.{side}", f"{second}.{side}"
            vector = positions[child] - positions[parent]
            require(
                np.linalg.norm(vector) >= length
                and (first == "Clavicle" or vector[1] < -length * 0.85),
                f"{parent} must extend anatomically toward {child}",
            )
            require(
                np.dot(vector / np.linalg.norm(vector), nodes[parent].world_matrix[:3, 1]) > 0.995,
                f"{parent} local +Y must point toward {child}",
            )
        require(
            0 < hips[1] - positions[f"Thigh.{side}"][1] < 0.10,
            f"Thigh.{side} must start beside and below the pelvis",
        )
        require(
            nodes[f"Foot.{side}"].world_matrix[2, 1] > 0.75,
            f"Foot.{side} must point forward from the ankle",
        )
    for name, point in positions.items():
        if name.endswith(".L"):
            require(
                np.allclose(point * (-1, 1, 1), positions[name[:-1] + "R"], atol=1e-5),
                f"Rest placement must mirror {name}",
            )
    for source, dest in (
        ("Hips", "Spine"),
        ("Spine", "Chest"),
        ("Chest", "Neck"),
        ("Neck", "Head"),
    ):
        require(positions[dest][1] > positions[source][1], f"{dest} must be above {source}")
    require(
        {n.name for n in model.nodes} == PARENTS.keys() | {"PilotMesh"},
        "Export only the deform skeleton and PilotMesh",
    )


def check_pilot(path):
    asset = PilotAsset(path)
    model = asset.model
    joints = {joint.name for joint in asset.joints}
    require(
        joints == PARENTS.keys(),
        f"Pilot skin joints differ. Missing {PARENTS.keys() - joints}, "
        f"extra {joints - PARENTS.keys()}",
    )
    for name, parent in PARENTS.items():
        node = named(model, name)
        actual = node.parent.name if node.parent else None
        require(actual == parent, f"Pilot joint {name} must have parent {parent}, got {actual}")
        rotation = node.base_matrix[:3, :3]
        require(
            np.allclose(rotation.T @ rotation, np.eye(3), atol=5e-5)
            and np.linalg.det(rotation) > 0.999,
            f"{name} has scaled or mirrored bind axes",
        )
    check_anatomy(asset)
    require(set(model.animations) == CLIPS, "Pilot must export exactly Idle, Walk, Run and Turn")
    strengths = {}
    for name, clip in model.animations.items():
        asset.reset_locomotion()
        asset.player.play(name)
        samples = {}
        for kind in ("Thigh", "Shin", "UpperArm", "Forearm"):
            samples[kind] = np.zeros((2, 33))
        for frame, phase in enumerate(np.linspace(0, 1, 33)):
            asset.player.time = clip.duration * phase
            asset.player.apply()
            for kind in samples:
                for side, label in enumerate(("L", "R")):
                    node = named(model, f"{kind}.{label}")
                    samples[kind][side, frame] = hinge_angle(node, asset.hinge_axes[node.name])
        for channel in clip.channels:
            require(
                np.allclose(
                    channel.sampler.sample(0), channel.sampler.sample(clip.duration), atol=2e-5
                ),
                f"{name}: {channel.node.name} {channel.path} loop is open",
            )
        knees = samples["Shin"]
        require(
            knees.min() >= -1e-5 and knees.max() < np.pi / 2,
            f"{name}: knee flexion must stay between 0 and 90 degrees about the lateral axis",
        )
        if name not in ("Walk", "Run"):
            continue
        arms, elbows = samples["UpperArm"], samples["Forearm"]
        require(np.max(abs(arms)) < np.radians(45), f"{name}: shoulder swing exceeds 45 degrees")
        require(
            elbows.min() > -np.radians(60) and elbows.max() < -np.radians(5),
            f"{name}: elbows must stay flexed toward the front between 5 and 60 degrees",
        )
        for side in (0, 1):
            phase = np.corrcoef(arms[side, :-1], samples["Thigh"][side, :-1])[0, 1]
            require(phase < -0.75, f"{name}: arm {('L', 'R')[side]} must oppose its thigh")
        for label, values in (("arms", arms), ("knees", knees)):
            require(
                np.max(abs(values[0, :17] - values[1, 16:])) < 0.04,
                f"{name}: left and right {label} must alternate by half a cycle",
            )
        strengths[name] = np.ptp(arms, axis=1).mean()
    require(
        strengths["Run"] > strengths["Walk"] * 1.15,
        "Pilot Run shoulder swing must be stronger than Walk",
    )
    return asset


def check_jeep(path):
    model = load_jeep(path)
    if contract_components(model):
        return model
    root = named(model, "Jeep")
    require(
        model.roots == [root] and np.allclose(root.base_matrix, np.eye(4), atol=1e-6),
        "Jeep must be the sole export root with identity transform",
    )
    require(bool(root.meshes), "Jeep root needs chassis geometry")
    require(not model.animations, "Jeep wheel animation is runtime-driven. Export without Actions")
    p = DriveParameters()
    for suffix, x, z in zip(WHEELS, (1, -1, 1, -1), (1, 1, -1, -1), strict=True):
        hub, wheel = named(model, f"Hub.{suffix}"), named(model, f"Wheel.{suffix}")
        require(
            hub.parent is root and wheel.parent is hub,
            f"Required hierarchy: Jeep > Hub.{suffix} > Wheel.{suffix}",
        )
        require(not hub.meshes and bool(wheel.meshes), f"Wheel.{suffix} needs geometry on its hub")
        require(
            np.allclose(wheel.base_matrix, np.eye(4), atol=1e-6),
            f"Wheel.{suffix} must be centered on its hub with identity local transform",
        )
        require(
            np.allclose(hub.base_matrix[:3, :3], np.eye(3), atol=1e-6)
            and np.allclose(
                hub.base_matrix[:3, 3], (x * p.track / 2, -0.19, z * p.wheelbase / 2), atol=1e-5
            ),
            f"Hub.{suffix} pivot differs from the authored suspension dimensions",
        )
    for name in ANCHORS:
        node = named(model, name)
        require(node.parent is root and not node.meshes, f"{name} must be an empty under Jeep")
        require(
            np.allclose(node.base_matrix[:3, :3], np.eye(3), atol=1e-6),
            f"{name} must have identity rotation and scale",
        )
        if name.endswith((".L", ".R")):
            require(
                node.base_matrix[0, 3] * (1 if name.endswith(".L") else -1) > 0,
                f"{name} must remain on its named side of the chassis",
            )
    return model


def validate(kind, path):
    return {"pilot": check_pilot, "jeep": check_jeep}[kind](path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", choices=RUNTIME)
    parser.add_argument("path", type=Path, nargs="?")
    args = parser.parse_args()
    try:
        validate(args.asset, args.path or RUNTIME[args.asset])
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(f"{args.asset}: {error}", file=sys.stderr)
        return 1
    print(f"{args.asset}: valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
