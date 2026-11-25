"""Authored interactive objects with shared animation, visibility, and collision."""

import json
import math
from dataclasses import dataclass

import numpy as np

from game.player_control import EYE_HEIGHT, HEIGHT, RADIUS, UP, segment_box_distance_squared
from game.quests.manager import Event
from game.world_manifest import project_path, require, vector


@dataclass
class InteractiveObject:
    identity: str
    spec: dict
    progress: float
    target: int
    locked: bool
    owner: str | None = None
    activations: int = 0

    @property
    def position(self):
        return np.asarray(self.spec["position"]) + self.progress * np.asarray(
            self.spec.get("travel", (0, 0, 0))
        )

    @property
    def state(self):
        if self.progress == self.target:
            return "OPEN" if self.target else "CLOSED"
        return "OPENING" if self.target else "CLOSING"

    def bounds(self, progress=None):
        center = (
            self.position
            if progress is None
            else (
                np.asarray(self.spec["position"])
                + progress * np.asarray(self.spec.get("travel", (0, 0, 0)))
            )
        )
        half = np.asarray(self.spec["size"]) / 2
        return center - half, center + half


class InteractiveScene:
    def __init__(self, path):
        data = json.loads(path.read_text())
        require(data["schema_version"] == 1, "Unsupported interactive scene version")
        self.objects = {}
        for identity, spec in data["objects"].items():
            position = vector(spec["position"], identity)
            size = vector(spec["size"], identity)
            color = vector(spec["color"], identity)
            require(min(size) > 0 and min(color) >= 0 and max(color) <= 1, f"Invalid {identity}")
            spec.update(position=position, size=size, color=color)
            if "visual" in spec:
                visual = project_path(spec["visual"])
                require(
                    visual.suffix == ".glb" and visual.is_file(),
                    f"Missing GLB visual for {identity}",
                )
                spec["visual"] = visual
            if "travel" in spec:
                vector(spec["travel"], identity)
                require(spec.get("duration", 1) > 0, f"Invalid animation duration for {identity}")
            self.objects[identity] = InteractiveObject(
                identity, spec, 0.0, 0, bool(spec.get("key"))
            )
        for obj in self.objects.values():
            for ref in ("container", "key", "control"):
                require(
                    ref not in obj.spec or obj.spec[ref] in self.objects,
                    f"{obj.identity}: unknown {ref}",
                )
        self.anchors = {key: vector(value, key) for key, value in data["anchors"].items()}
        self.revision = 0
        self._collision_revision = -1
        self._collision = np.empty((0, 2, 3))

    def visible(self, obj):
        container = self.objects.get(obj.spec.get("container"))
        return obj.owner is None and (container is None or container.progress >= 0.8)

    def obstacles(self, lo, hi):
        if self._collision_revision != self.revision:
            self._collision = np.asarray(
                [
                    self._bounds(obj)
                    for obj in self.objects.values()
                    if obj.spec.get("solid") and self.visible(obj)
                ]
            ).reshape((-1, 2, 3))
            self._collision_revision = self.revision
        near = np.all(self._collision[:, 0] <= hi, axis=1) & np.all(
            self._collision[:, 1] >= lo, axis=1
        )
        return self._collision[near]

    def inventory(self, actor):
        return frozenset(obj.identity for obj in self.objects.values() if obj.owner == actor)

    def _point(self, obj):
        point = obj.position.copy()
        container = self.objects.get(obj.spec.get("container"))
        if container is not None:
            point += container.position - np.asarray(container.spec["position"])
        return point

    def _bounds(self, obj):
        point = self._point(obj)
        half = np.asarray(obj.spec["size"]) / 2
        return point - half, point + half

    def render_rows(self):
        for obj in self.objects.values():
            if self.visible(obj) and "visual" not in obj.spec:
                yield (*self._bounds(obj), obj.spec["color"])

    def visual_rows(self):
        for obj in self.objects.values():
            if "visual" in obj.spec and self.visible(obj):
                yield obj.spec["visual"], self._point(obj)

    def in_reach(self, obj, pilot, space):
        if not self.visible(obj) or not pilot.grounded:
            return False
        eye = pilot.position + EYE_HEIGHT * UP
        lo, hi = self._bounds(obj)
        anchor = obj.spec.get("anchor")
        if anchor is not None:
            point = np.asarray(obj.spec["position"]) + np.asarray(anchor)
        else:
            point = np.clip(eye, lo, hi)
        if np.linalg.norm(eye - point) > obj.spec.get("reach", 2.3):
            return False
        for a, b in space.boxes(np.minimum(eye, point), np.maximum(eye, point)):
            if np.array_equal(a, lo) and np.array_equal(b, hi):
                continue
            if segment_box_distance_squared(eye, point, a, b) < 1e-9:
                return False
        return True

    def candidates(self, pilot, space):
        return sorted(
            (
                o
                for o in self.objects.values()
                if o.spec.get("name") and self.in_reach(o, pilot, space)
            ),
            key=lambda o: (
                np.linalg.norm(self._point(o) - (pilot.position + EYE_HEIGHT * UP))
                + o.spec.get("priority_offset", 0)
            ),
        )

    def action(self, obj):
        kind = obj.spec["type"]
        if obj.locked:
            return "UNLOCK"
        if kind in ("drawer", "container", "door", "gate"):
            return "CLOSE" if obj.target else "OPEN"
        if kind in ("key", "item"):
            return "COLLECT"
        return "ACTIVATE" if "control" in obj.spec else "INSPECT"

    def interact(self, identity, actor, pilot, space, now, occupants, *, action=None):
        """Validate a requested action and return structured events and feedback."""
        obj = self.objects.get(identity)
        if obj is None or not obj.spec.get("name") or not self.in_reach(obj, pilot, space):
            return (), "OBJECT IS OUT OF REACH OR OBSTRUCTED"
        if actor not in obj.spec.get("actors", (actor,)):
            return (), "ACCESS DENIED"
        action = self.action(obj) if action is None else action
        if action != self.action(obj) and action != "INSPECT":
            return (), "ACTION UNAVAILABLE"
        if action == "INSPECT":
            return (Event("ENTITY_INSPECTED", actor, identity, tuple(self._point(obj)), now),), (
                obj.spec.get("description", obj.spec["name"])
            )
        if obj.locked:
            if obj.spec["key"] not in self.inventory(actor):
                return (), "DOOR LOCKED - KEY REQUIRED"
            obj.locked = False
            self.revision += 1
            return (Event("LOCK_UNLOCKED", actor, identity, tuple(pilot.position), now),), (
                "DOOR UNLOCKED - PRESS T TO OPEN"
            )
        if action == "COLLECT":
            obj.owner = actor
            self.revision += 1
            return (Event("ITEM_COLLECTED", actor, identity, tuple(pilot.position), now),), (
                f"COLLECTED: {obj.spec['name']}"
            )
        moved = self.objects.get(obj.spec.get("control"), obj)
        target = 1 - moved.target
        if (
            moved.spec.get("solid")
            and (not target or moved.spec.get("block_opening"))
            and (self.obstructed(moved, occupants, target))
        ):
            return (), "OPENING OBSTRUCTED"
        moved.target = target
        if action == "ACTIVATE":
            obj.activations += 1
        self.revision += 1
        return (
            Event(
                "OBJECT_INTERACTED",
                actor,
                identity,
                tuple(pilot.position),
                now,
                data={"cue": moved.spec.get("sound"), "motion": moved.state},
            ),
        ), (f"{moved.spec.get('name', moved.identity.replace('_', ' '))} {moved.state}")

    def obstructed(self, obj, occupants, target=0):
        lo, hi = obj.bounds(target)
        for lower, upper in occupants:
            if np.all(lower < hi) and np.all(upper > lo):
                return True
        for other in self.objects.values():
            if other is not obj and other.spec.get("solid") and self.visible(other):
                lower, upper = other.bounds()
                if np.all(lower < hi) and np.all(upper > lo):
                    return True
        return False

    def tick(self, dt, occupants):
        completed = []
        for obj in self.objects.values():
            if obj.progress == obj.target:
                continue
            if obj.target and obj.spec.get("block_opening") and self.obstructed(obj, occupants, 1):
                obj.target = 0
                self.revision += 1
            if not obj.target and obj.spec.get("solid") and self.obstructed(obj, occupants):
                obj.target = 1
                self.revision += 1
            step = max(0.0, dt) / obj.spec.get("duration", 1)
            value = min(obj.progress + step, 1) if obj.target else max(obj.progress - step, 0)
            if value != obj.progress:
                self.revision += 1
                obj.progress = value
                if value == 1 and obj.spec.get("travel"):
                    completed.append(obj.identity)
        return completed

    def snapshot(self):
        return {
            key: [o.progress, o.target, o.locked, o.owner, o.activations]
            for key, o in self.objects.items()
        }

    def restore(self, raw, actor):
        require(
            isinstance(raw, dict) and raw.keys() == self.objects.keys(), "Invalid saved objects"
        )
        for identity, obj in self.objects.items():
            state = raw[identity]
            require(isinstance(state, list) and len(state) == 5, f"Invalid {identity} state")
            progress, target, locked, owner, activations = state
            require(
                type(progress) in (int, float)
                and math.isfinite(progress)
                and 0 <= progress <= 1
                and type(target) is int
                and target in (0, 1)
                and type(activations) is int
                and activations >= 0
                and (not activations or "control" in obj.spec)
                and type(locked) is bool
                and (owner is None or owner == actor)
                and (owner is None or obj.spec["type"] in ("key", "item"))
                and ("travel" in obj.spec or (progress == target == 0))
                and (not locked or ("key" in obj.spec and progress == target == 0)),
                f"Invalid {identity} state",
            )
            obj.progress, obj.target, obj.locked, obj.owner, obj.activations = state
        self.revision += 1


def actor_bounds(control, vehicles):
    bounds = [vehicle.bounds() for vehicle in vehicles]
    if control.pilot is not None:
        feet = control.pilot.position
        bounds.append(
            (feet - np.array((RADIUS, 0, RADIUS)), feet + np.array((RADIUS, HEIGHT, RADIUS)))
        )
    return bounds
