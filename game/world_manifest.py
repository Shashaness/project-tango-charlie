"""Validate immutable, versioned world composition before initializing systems."""

import json
import math
from dataclasses import dataclass, fields
from functools import lru_cache
from pathlib import Path
from types import MappingProxyType

from engine.asset_paths import PROJECT_ROOT, asset_path

DEFAULT_WORLD = asset_path("worlds/tango_charlie/world.json")
COORDINATES = {"up": "Y", "units": "meters", "rotation": "XYZ_DEGREES", "forward": "+Z"}
KINDS = {"aircraft", "character", "ground_vehicle", "structure", "prop"}


class WorldError(ValueError):
    """Invalid authored world data."""


def require(condition, message):
    if not condition:
        raise WorldError(message)


def object_fields(value, label, required, optional=()):
    require(isinstance(value, dict), f"{label} must be an object")
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    require(
        not (missing or unknown), f"{label}: missing {sorted(missing)}, unknown {sorted(unknown)}"
    )
    return value


def vector(value, label, size=3):
    require(isinstance(value, list) and len(value) == size, f"{label} needs {size} coordinates")
    require(
        all(type(v) in (int, float) and math.isfinite(v) for v in value),
        f"{label} needs finite numbers",
    )
    return tuple(float(v) for v in value)


def pairs(items):
    obj = {}
    for key, value in items:
        require(key not in obj, f"Duplicate JSON key {key}")
        obj[key] = value
    return obj


def project_path(value):
    require(isinstance(value, str) and value, "Path must be a nonempty project-relative string")
    path = Path(value)
    resolved = (PROJECT_ROOT / path).resolve()
    require(
        not path.is_absolute() and resolved.is_relative_to(PROJECT_ROOT),
        f"Path escapes the project root: {value}",
    )
    return resolved


@dataclass(frozen=True)
class Asset:
    kind: str
    path: Path


@dataclass(frozen=True)
class Entity:
    identity: str
    asset: Asset
    position: tuple
    rotation: tuple
    scale: tuple
    parent: str | None
    anchor: str | None
    initial_state: MappingProxyType


@dataclass(frozen=True)
class WorldDefinition:
    identity: str
    assets: MappingProxyType
    entities: tuple
    airport: MappingProxyType
    town_seed: int
    terrain: object
    player_mode: str
    controlled_entity: str


def read_airport(airport, entities):
    object_fields(
        airport,
        "airport",
        {
            "runway",
            "taxiways",
            "apron",
            "apron_link",
            "side_hangars",
            "primary_hangar",
        },
        {"service_yards", "access_roads"},
    )
    runway = airport["runway"]
    object_fields(runway, "runway", {"center", "length", "width", "designation"})
    center = vector(runway["center"], "runway.center")
    for key in ("length", "width"):
        number = runway[key]
        require(
            type(number) in (int, float) and math.isfinite(number) and number > 0,
            f"runway.{key} must be positive",
        )
    require(
        center[1] == 0
        and isinstance(runway["designation"], str)
        and len(runway["designation"]) == 2
        and runway["designation"].isascii()
        and runway["designation"].isdigit(),
        "Runway needs ground elevation zero and two digits",
    )
    primary = airport["primary_hangar"]
    require(
        isinstance(primary, str)
        and primary in entities
        and entities[primary].asset.kind == "structure",
        "airport.primary_hangar must reference a structure",
    )

    def rectangle(item, label, count):
        object_fields(item, label, {"center", "size"})
        position = vector(item["center"], f"{label}.center", count)
        size = vector(item["size"], f"{label}.size", count)
        require(min(size) > 0, f"{label}.size must be positive")
        return position, size

    result = {"runway": MappingProxyType({**runway, "center": center}), "primary_hangar": primary}
    for key in ("taxiways", "side_hangars"):
        require(isinstance(airport[key], list), f"airport.{key} must be an array")
        result[key] = tuple(
            rectangle(item, key, 2 if key == "taxiways" else 3) for item in airport[key]
        )
    for key in ("apron", "apron_link"):
        result[key] = rectangle(airport[key], key, 2)
    yards = airport.get("service_yards", [])
    require(isinstance(yards, list), "service_yards must be an array")
    result["service_yards"] = tuple(rectangle(item, "service yard", 2) for item in yards)
    roads = airport.get("access_roads", [])
    require(isinstance(roads, list), "access_roads must be an array")
    authored = {}
    for road in roads:
        object_fields(road, "access road", {"id", "width", "points"})
        key, width, points = road["id"], road["width"], road["points"]
        require(isinstance(key, str) and key and key not in authored, "Invalid access road ID")
        require(
            type(width) in (float, int) and math.isfinite(width) and width > 0,
            "Invalid access road width",
        )
        require(isinstance(points, list) and len(points) >= 2, "Road needs at least two points")
        controls = tuple(vector(point, key, 2) for point in points)
        require(
            all(a != b for a, b in zip(controls, controls[1:])),
            "Consecutive road points must differ",
        )
        authored[key] = (width, controls)
    result["access_roads"] = MappingProxyType(authored)
    return MappingProxyType(result)


def load_manifest(path=DEFAULT_WORLD):
    from game.terrain import TerrainConfig

    try:
        data = json.loads(Path(path).read_text(), object_pairs_hook=pairs)
        object_fields(
            data,
            "world",
            {
                "schema_version",
                "world_id",
                "coordinates",
                "assets",
                "entities",
                "player",
                "environment",
            },
        )
        require(
            type(data["schema_version"]) is int and data["schema_version"] == 1,
            "Unsupported world schema version",
        )
        require(isinstance(data["world_id"], str) and data["world_id"], "Missing world_id")
        require(data["coordinates"] == COORDINATES, "Unsupported world coordinates")
        require(isinstance(data["assets"], dict) and data["assets"], "assets must be a registry")
        assets = {}
        for key, spec in data["assets"].items():
            object_fields(spec, f"asset {key}", {"type", "path"})
            kind = spec["type"]
            require(
                isinstance(kind, str) and kind in KINDS, f"Unknown asset type for {key}: {kind}"
            )
            resolved = project_path(spec["path"])
            require(
                resolved.suffix.lower() == ".glb" and resolved.is_file(),
                f"Missing GLB for asset {key}: {spec['path']}",
            )
            assets[key] = Asset(kind, resolved)
        entities = {}
        require(isinstance(data["entities"], list), "entities must be an array")
        for spec in data["entities"]:
            object_fields(
                spec, "entity", {"id", "asset"}, {"transform", "parent", "anchor", "initial_state"}
            )
            key, ref = spec["id"], spec["asset"]
            require(isinstance(key, str) and key and "." not in key, "Invalid entity ID")
            require(key not in entities, f"Duplicate entity ID {key}")
            require(isinstance(ref, str) and ref in assets, f"{key}: missing asset {ref}")
            kind = assets[ref].kind
            transform = spec.get("transform", {})
            object_fields(transform, f"{key}.transform", set(), {"position", "rotation", "scale"})
            position, rotation, scale = (
                vector(transform.get(name, default), f"{key}.{name}")
                for name, default in (
                    ("position", [0, 0, 0]),
                    ("rotation", [0, 0, 0]),
                    ("scale", [1, 1, 1]),
                )
            )
            require(min(scale) > 0, f"{key}: scale must be positive")
            if kind in {"aircraft", "ground_vehicle", "character"}:
                require(scale == (1, 1, 1), f"{key}: gameplay actors require unit scale")
            parent, anchor = spec.get("parent"), spec.get("anchor")
            require(parent is None or isinstance(parent, str) and parent, f"{key}: invalid parent")
            require(
                anchor is None or (isinstance(anchor, str) and anchor and parent),
                f"{key}: an anchor requires a parent",
            )
            state = spec.get("initial_state", {})
            if kind == "aircraft":
                object_fields(
                    state, f"{key}.initial_state", {"configuration", "environment", "status"}
                )
                require(
                    state["configuration"] in ("FIGHTER", "VTOL", "BATTLEDROID")
                    and state["environment"] == "ATMOSPHERE"
                    and state["status"] == "GROUNDED",
                    f"{key}: expected a configuration parked in atmosphere",
                )
            elif kind == "ground_vehicle":
                object_fields(state, f"{key}.initial_state", {"parking_brake"})
                require(
                    type(state["parking_brake"]) is bool, f"{key}: parking_brake must be boolean"
                )
            else:
                object_fields(state, f"{key}.initial_state", set())
            entities[key] = Entity(
                key, assets[ref], position, rotation, scale, parent, anchor, MappingProxyType(state)
            )
        ordered, visiting = {}, set()

        def visit(key):
            if key in ordered:
                return
            require(key in entities, f"Missing parent entity {key}")
            require(key not in visiting, f"Circular entity reference at {key}")
            visiting.add(key)
            entity = entities[key]
            if entity.parent:
                visit(entity.parent)
            visiting.remove(key)
            ordered[key] = entity

        for key in entities:
            visit(key)
        player = data["player"]
        object_fields(player, "player", {"starting_mode", "controlled_entity"})
        mode, controlled = player["starting_mode"], player["controlled_entity"]
        mode_kinds = {
            "AIRCRAFT": "aircraft",
            "ON_FOOT": "character",
            "GROUND_VEHICLE": "ground_vehicle",
        }
        require(
            isinstance(mode, str)
            and mode in mode_kinds
            and isinstance(controlled, str)
            and controlled in entities
            and entities[controlled].asset.kind == mode_kinds[mode],
            "Player controlled entity does not match starting mode",
        )
        for kind in ("aircraft", "character"):
            require(
                sum(e.asset.kind == kind for e in entities.values()) == 1,
                f"World requires one playable {kind}",
            )
        env = data["environment"]
        object_fields(env, "environment", {"terrain", "town", "airport"})
        town = env["town"]
        object_fields(town, "town", {"generator", "seed"})
        require(
            town["generator"] == "legacy_blocks"
            and type(town["seed"]) is int
            and town["seed"] >= 0,
            "Unknown town generator or invalid seed",
        )
        options = dict(
            object_fields(env["terrain"], "terrain", set(), {f.name for f in fields(TerrainConfig)})
        )
        for name in ("directory", "registry_directory"):
            if name in options:
                options[name] = project_path(options[name])
        require(
            all(
                value is None
                and key == "elevation_offset"
                or isinstance(value, Path)
                or type(value) in (int, float)
                and math.isfinite(value)
                for key, value in options.items()
            ),
            "Invalid terrain configuration value",
        )
        integer_options = {
            "planetary_seed",
            "cpu_cache_size",
            "cpu_cache_bytes",
            "budget",
            "max_level",
            "cells",
            "cache_size",
            "compatibility_seed",
        }
        require(
            all(type(options[key]) is int for key in integer_options & options.keys()),
            "Terrain seeds, subdivision counts, and cache sizes must be integers",
        )
        terrain = TerrainConfig(**options)
        airport = read_airport(env["airport"], entities)
        return WorldDefinition(
            data["world_id"],
            MappingProxyType(assets),
            tuple(ordered.values()),
            airport,
            town["seed"],
            terrain,
            mode,
            controlled,
        )
    except (OSError, ValueError, TypeError, OverflowError) as error:
        raise WorldError(f"{Path(path)}: {error}") from error


@lru_cache(maxsize=1)
def default_definition():
    return load_manifest()
