"""Headless quest lifecycle and idempotent event processing."""

from dataclasses import dataclass, field
from enum import Enum
from uuid import uuid4

from game.world_manifest import object_fields, require

from .definitions import identifier, positive


class Status(str, Enum):
    LOCKED = "LOCKED"
    AVAILABLE = "AVAILABLE"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Event:
    type: str
    actor: str
    target: str
    location: tuple
    timestamp: float
    data: dict = field(default_factory=dict)
    id: str = field(default_factory=lambda: uuid4().hex)


class QuestManager:
    def __init__(self, definitions):
        self.definitions = definitions
        self.status = dict.fromkeys(definitions, Status.LOCKED)
        self.progress = {
            key: dict.fromkeys((o.id for o in quest.objectives), 0)
            for key, quest in definitions.items()
        }
        self.conditions = set()
        self.processed = set()
        self.journal = []
        self.refresh()

    def refresh(self):
        for key, quest in self.definitions.items():
            if self.status[key] in (Status.LOCKED, Status.AVAILABLE):
                available = all(
                    self.status[parent] == Status.COMPLETED for parent in quest.prerequisites
                )
                self.status[key] = Status.AVAILABLE if available else Status.LOCKED
                if available and (quest.activation is None or quest.activation in self.conditions):
                    self.status[key] = Status.ACTIVE
                    self._entry(quest, "initial")
            if (
                self.status[key] == Status.ACTIVE
                and all(
                    self.progress[key][o.id] == o.count for o in quest.objectives if not o.optional
                )
                and (quest.completion is None or quest.completion in self.conditions)
            ):
                self.status[key] = Status.COMPLETED
                self._entry(quest, "complete")

    def set_condition(self, condition):
        require(identifier(condition), "Invalid quest condition")
        self.conditions.add(condition)
        self.refresh()

    def fail(self, key):
        if self.status[key] == Status.ACTIVE:
            self.status[key] = Status.FAILED

    def _entry(self, quest, phase):
        if phase in dict(quest.journal):
            self.journal.append((quest.id, phase))

    def ready(self, key):
        if self.status[key] != Status.ACTIVE:
            return ()
        objectives = self.definitions[key].objectives
        progress = self.progress[key]
        pending = [o for o in objectives if progress[o.id] < o.count]
        stage = min(
            (o.stage for o in pending if not o.optional), default=max(o.stage for o in objectives)
        )
        return tuple(o for o in pending if o.stage == stage or (o.optional and o.stage <= stage))

    def accept(self, event):
        if event.id in self.processed:
            return False
        changed = False
        for key, quest in self.definitions.items():
            if quest.owner != event.actor or self.status[key] != Status.ACTIVE:
                continue
            progress = self.progress[key]
            for objective in self.ready(key):
                if (objective.event, objective.target) == (event.type, event.target):
                    progress[objective.id] += 1
                    if progress[objective.id] == objective.count:
                        self._entry(quest, objective.id)
                    changed = True
        # Record unmatched events too, so replay cannot turn old observations into new progress.
        self.processed.add(event.id)
        self.refresh()
        return changed

    def snapshot(self):
        return {
            "status": {key: state.value for key, state in self.status.items()},
            "progress": {key: values.copy() for key, values in self.progress.items()},
            "conditions": sorted(self.conditions),
            "processed": sorted(self.processed),
            "journal": [list(entry) for entry in self.journal],
        }

    def restore(self, raw):
        object_fields(
            raw, "quest state", {"status", "progress", "conditions", "processed", "journal"}
        )
        object_fields(raw["status"], "quest statuses", set(self.definitions))
        object_fields(raw["progress"], "quest progress", set(self.definitions))
        statuses = {key: Status(value) for key, value in raw["status"].items()}
        for key in ("conditions", "processed"):
            values = raw[key]
            require(
                isinstance(values, list)
                and all(map(identifier, values))
                and len(set(values)) == len(values),
                f"Invalid saved {key}",
            )
        expected_entries = set()
        for key, quest in self.definitions.items():
            values = object_fields(raw["progress"][key], key, {o.id for o in quest.objectives})
            require(
                all(
                    positive(values[o.id], 0) and values[o.id] <= o.count for o in quest.objectives
                ),
                f"Invalid progress for {key}",
            )
            pending_stages = [
                o.stage for o in quest.objectives if not o.optional and values[o.id] < o.count
            ]
            if pending_stages:
                first_pending = min(pending_stages)
                require(
                    not any(values[o.id] for o in quest.objectives if o.stage > first_pending),
                    f"Out-of-order saved progress for {key}",
                )
            state = statuses[key]
            prereqs = all(statuses[p] == Status.COMPLETED for p in quest.prerequisites)
            activated = prereqs and (
                quest.activation is None or quest.activation in raw["conditions"]
            )
            complete = all(values[o.id] == o.count for o in quest.objectives if not o.optional)
            complete &= quest.completion is None or quest.completion in raw["conditions"]
            if state == Status.FAILED:
                require(activated, f"Inconsistent failure for {key}")
            else:
                expected = (
                    Status.LOCKED
                    if not prereqs
                    else Status.AVAILABLE
                    if not activated
                    else Status.COMPLETED
                    if complete
                    else Status.ACTIVE
                )
                require(state == expected, f"Inconsistent status for {key}")
            if not activated:
                require(not any(values.values()), f"Inactive progress for {key}")
            if activated and "initial" in dict(quest.journal):
                expected_entries.add((key, "initial"))
            expected_entries.update(
                (key, o.id)
                for o in quest.objectives
                if values[o.id] == o.count and o.id in dict(quest.journal)
            )
            if state == Status.COMPLETED and "complete" in dict(quest.journal):
                expected_entries.add((key, "complete"))
        entries = raw["journal"]
        require(
            isinstance(entries, list)
            and all(
                isinstance(e, list) and len(e) == 2 and all(map(identifier, e)) for e in entries
            ),
            "Invalid journal",
        )
        journal = [tuple(e) for e in entries]
        require(
            set(journal) == expected_entries and len(set(journal)) == len(journal),
            "Inconsistent journal",
        )
        self.status = statuses
        self.progress = raw["progress"]
        self.conditions = set(raw["conditions"])
        self.processed = set(raw["processed"])
        self.journal = journal
