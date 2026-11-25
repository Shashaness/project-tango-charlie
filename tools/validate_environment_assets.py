"""Check environmental GLBs with the runtime loader."""

import argparse
from pathlib import Path
from types import MappingProxyType

from engine.asset_paths import ASSET_ROOT
from game.world_loader import Placement, environment_geometry, scene_model
from game.world_manifest import Asset, Entity, require

ENVIRONMENT_RUNTIME = {
    name: ASSET_ROOT / "models/environment" / f"{name}.glb" for name in ("hangar", "repair_station")
}
ENVIRONMENT_COLLECTIONS = {
    "hangar": ("Structure", "Floor", "Collision", "Anchors", "Details"),
    "repair_station": ("Equipment", "Collision", "Anchors"),
}
REQUIRED = {
    "hangar": {
        "Anchor.aircraft_parking",
        "Anchor.vehicle_parking",
        "Anchor.pilot_spawn",
        "Anchor.workshop",
        "Anchor.entrance",
        "Surface.Floor",
        "BackWall",
        "NorthWall",
        "SouthWall",
        "Roof",
        "Collision.BackWall",
        "Collision.NorthWall",
        "Collision.SouthWall",
        "Collision.Roof",
    },
    "repair_station": {
        "Anchor.interaction",
        "Anchor.pilot_standing",
        "Workbench",
        "Cabinet",
        "ToolCase",
        "Collision.Workbench",
    },
}


def validate_environment(kind, path):
    path = Path(path)
    entity = Entity(
        kind,
        Asset("structure" if kind == "hangar" else "prop", path),
        (0, 0, 0),
        (0, 0, 0),
        (1, 1, 1),
        None,
        None,
        MappingProxyType({}),
    )
    unit = tuple(tuple(int(i == j) for j in range(4)) for i in range(4))
    placement = Placement(entity, unit)
    model = scene_model(path)
    names = {node.name for node in model.nodes}
    missing = REQUIRED[kind] - names
    require(not missing, f"{kind}: missing nodes {sorted(missing)}")
    require(not model.animations, f"{kind}: environmental assets must be static")
    for name in REQUIRED[kind]:
        if "." not in name:
            require(model.find_node(name).meshes, f"{kind}: {name} must contain geometry")
        if name.startswith("Anchor."):
            placement.anchor(name.removeprefix("Anchor."))
    environment_geometry(placement)
    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", choices=ENVIRONMENT_RUNTIME)
    parser.add_argument("path", nargs="?", type=Path)
    args = parser.parse_args()
    validate_environment(args.asset, args.path or ENVIRONMENT_RUNTIME[args.asset])
    print(f"{args.asset}: runtime validation passed")
