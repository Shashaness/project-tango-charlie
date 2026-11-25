"""Validate immutable versioned quest definitions before starting a campaign."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

from game.world_manifest import object_fields, pairs, require

OBJECTIVE_EVENTS = dict(
    reach="LOCATION_REACHED",
    discover="ENTITY_DISCOVERED",
    inspect="ENTITY_INSPECTED",
    collect="ITEM_COLLECTED",
    install="COMPONENT_INSTALLED",
    repair="EQUIPMENT_REPAIRED",
    enter="VEHICLE_ENTERED",
    open="OBJECT_OPENED",
    unlock="LOCK_UNLOCKED",
    operate="VEHICLE_OPERATED",
    attach="TROLLEY_ATTACHED",
    load="CARGO_LOADED",
    secure="CARGO_SECURED",
    transport="CARGO_TRANSPORTED",
    deliver="CARGO_DELIVERED",
    interact="OBJECT_INTERACTED",
    investigate="SIGNAL_INVESTIGATED",
)


def identifier(value):
    return isinstance(value, str) and bool(value.strip()) and value == value.strip()


def positive(value, minimum=1):
    return type(value) is int and value >= minimum


@dataclass(frozen=True)
class Objective:
    id: str
    title: str
    event: str
    target: str
    count: int
    stage: int
    optional: bool
    distance_m: float | None = None


@dataclass(frozen=True)
class Quest:
    id: str
    title: str
    description: str
    owner: str
    classification: str
    prerequisites: tuple
    objectives: tuple
    entities: tuple
    activation: str | None
    completion: str | None
    journal: tuple
    hints: tuple


def load_quests(path, entities, registry=None):
    registry = OBJECTIVE_EVENTS if registry is None else registry
    data = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=pairs)
    object_fields(data, "quest registry", {"schema_version", "quests"})
    require(
        type(data["schema_version"]) is int and data["schema_version"] == 1,
        "Unsupported quest schema",
    )
    require(isinstance(data["quests"], list), "quests must be a list")
    quests = {}
    for raw in data["quests"]:
        object_fields(
            raw,
            "quest",
            {"id", "title", "description", "owner", "classification", "objectives"},
            {"prerequisites", "entities", "activation", "completion", "journal", "hints"},
        )
        for key in ("id", "title", "description", "owner", "classification"):
            require(identifier(raw[key]), f"Invalid quest {key}")
        key = raw["id"]
        require(key not in quests, f"Duplicate quest identifier {key}")
        require(raw["classification"] in ("main", "secondary"), f"{key}: invalid classification")
        require(raw["owner"] in entities, f"{key}: unknown owner")
        for field in ("prerequisites", "entities", "hints"):
            values = raw.get(field, [])
            require(
                isinstance(values, list) and all(map(identifier, values)), f"{key}: invalid {field}"
            )
        require(
            all(entity in entities for entity in raw.get("entities", [])),
            f"{key}: invalid world entity reference",
        )
        for field in ("activation", "completion"):
            require(raw.get(field) is None or identifier(raw[field]), f"{key}: invalid {field}")
        journal = raw.get("journal", {})
        objectives = {}
        require(isinstance(raw["objectives"], list), f"{key}: objectives must be a list")
        for obj in raw["objectives"]:
            object_fields(
                obj,
                "objective",
                {"id", "title", "type", "target"},
                {"count", "stage", "optional", "distance_m"},
            )
            require(
                all(map(identifier, (obj["id"], obj["title"], obj["type"], obj["target"]))),
                f"{key}: invalid objective",
            )
            require(
                obj["id"] not in objectives and obj["type"] in registry,
                f"{key}: duplicate objective or unknown objective type",
            )
            require(obj["target"] in entities, f"{key}: unknown objective entity {obj['target']}")
            count, stage, optional = (
                obj.get("count", 1),
                obj.get("stage", 0),
                obj.get("optional", False),
            )
            require(
                positive(count) and positive(stage, 0) and type(optional) is bool,
                f"{key}: invalid objective settings",
            )
            distance = obj.get("distance_m")
            require(
                "distance_m" not in obj
                or (
                    obj["type"] == "operate"
                    and type(distance) in (int, float)
                    and math.isfinite(distance)
                    and distance > 0
                ),
                f"{key}: invalid operating distance",
            )
            objectives[obj["id"]] = Objective(
                obj["id"],
                obj["title"],
                registry[obj["type"]],
                obj["target"],
                count,
                stage,
                optional,
                distance,
            )
        require(
            any(not o.optional for o in objectives.values()), f"{key}: needs a required objective"
        )
        require(
            not ({"initial", "complete"} & objectives.keys()),
            f"{key}: reserved objective identifier",
        )
        object_fields(journal, "journal", set(), {"initial", "complete", *objectives})
        require(all(map(identifier, journal.values())), f"{key}: invalid journal entry")
        quests[key] = Quest(
            key,
            raw["title"],
            raw["description"],
            raw["owner"],
            raw["classification"],
            tuple(raw.get("prerequisites", [])),
            tuple(objectives.values()),
            tuple(raw.get("entities", [])),
            raw.get("activation"),
            raw.get("completion"),
            tuple(journal.items()),
            tuple(raw.get("hints", [])),
        )
    ordered, visiting = {}, set()

    def visit(key):
        require(key in quests, f"Unknown prerequisite {key}")
        require(key not in visiting, f"Circular quest prerequisites at {key}")
        if key in ordered:
            return
        visiting.add(key)
        for parent in quests[key].prerequisites:
            visit(parent)
        visiting.remove(key)
        ordered[key] = quests[key]

    for key in quests:
        visit(key)
    return ordered
