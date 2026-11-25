"""Q1.2A campaign integration and explicit legacy layout compatibility."""

from pathlib import Path
from textwrap import wrap

import numpy as np

from engine.asset_paths import asset_path
from game.campaign import DEFAULT_SAVE, JEEP, JEEP_PROFILE, OPENING, ArrivalCampaign
from game.interactive import InteractiveScene, actor_bounds
from game.quests.manager import Event, Status
from game.quests.persistence import read_save
from game.world_manifest import require

WORLD = asset_path("worlds/arrival/world.json")
QUESTS = asset_path("quests/arrival.json")
OBJECTS = asset_path("worlds/arrival/objects.json")
Q12B_WORLD = asset_path("worlds/arrival/world_q12b.json")
PREVIOUS_WORLD = asset_path("worlds/arrival/world_q12a.json")
PREVIOUS_OBJECTS = asset_path("worlds/arrival/objects_q12a.json")
JOURNEY = (
    "The Jeep works. I still can't quite believe it. As I drive toward the airport, the city is "
    "becoming clearer. The buildings are enormous. I've seen cities like this in the archives, "
    "but standing here, seeing one with my own eyes, is something entirely different."
)
KEY_WITH_CONTEXT = (
    "I found a key inside one of the desk drawers. There's another hangar nearby, but its "
    "personnel entrance is locked. Perhaps this belongs to it."
)
EQUIPMENT = {
    "office_desk": "office",
    "office_key": "key_seen",
    "gate_control": "panel",
    "crane_station": "crane_seen",
    "trolley": "trolley_seen",
}
ACTIONS = {
    ("ENTITY_INSPECTED", "office_desk"): "desk",
    ("ITEM_COLLECTED", "office_key"): "key_taken",
    ("LOCK_UNLOCKED", "personnel_door"): "unlock",
    ("OBJECT_INTERACTED", "gate_control"): "control_used",
    ("ENTITY_INSPECTED", "crane_station"): "crane_exam",
    ("ENTITY_INSPECTED", "trolley"): "trolley_exam",
}


def open_campaign(path=DEFAULT_SAVE, *, new=False):
    """Keep v1/v2 campaigns on their original geography, without moving actors."""
    path = Path(path)
    if not new and path.exists():
        raw = read_save(path)
        if isinstance(raw, dict) and raw.get("schema_version") in (1, 2):
            return ArrivalCampaign(path)
        if isinstance(raw, dict) and raw.get("schema_version") == 3 and "layout" not in raw:
            return ExpandedArrival(path, previous_layout=True)
        if isinstance(raw, dict) and raw.get("layout") == "q12b":
            return ExpandedArrival(path, layout_revision="q12b")
    return ExpandedArrival(path, new=new)


class ExpandedArrival(ArrivalCampaign):
    def __init__(
        self, path=DEFAULT_SAVE, *, new=False, previous_layout=False, layout_revision="q12c"
    ):
        self.previous_layout = previous_layout
        self.layout_revision = "q12a" if previous_layout else layout_revision
        world = {"q12a": PREVIOUS_WORLD, "q12b": Q12B_WORLD, "q12c": WORLD}[self.layout_revision]
        self.objects = InteractiveScene(PREVIOUS_OBJECTS if previous_layout else OBJECTS)
        self.selection = None
        self.selection_position = (0, 0)
        self._selected_id = None
        self.notice_detail = ""
        self.inspected_object = None
        self.detail_until = 0.0
        super().__init__(
            path,
            new=new,
            world=world,
            quests=QUESTS,
            extra_entities=self.objects.objects,
        )

    def observation_ids(self):
        return {o.id for q in self.manager.definitions.values() for o in q.objectives} | {
            "journey",
            "control_used",
            "equipment_found",
            "q1_complete",
            "locked_attempt",
            "key_with_context",
        }

    def _validate(self, raw):
        require(
            isinstance(raw, dict)
            and type(raw.get("schema_version")) is int
            and raw["schema_version"] == 3,
            "Unsupported expanded Arrival save",
        )
        require("objects" in raw, "Expanded save requires object state")
        raw = raw.copy()
        objects = raw.pop("objects")
        require(
            raw.pop("layout", None) == (None if self.previous_layout else self.layout_revision),
            "Save belongs to a different Arrival layout",
        )
        raw["schema_version"] = 2
        validated = super()._validate(raw)
        self.objects.restore(objects, self.owner)
        require(
            ("key_taken" in self.facts) == ("office_key" in self.objects.inventory(self.owner)),
            "Inconsistent key ownership",
        )
        require(
            ("unlock" in self.facts) == (not self.objects.objects["personnel_door"].locked),
            "Inconsistent personnel lock state",
        )
        q2 = self.manager.progress["Q002"]
        require(
            all(not value or key in self.facts for key, value in q2.items()),
            "Quest progress requires a matching observation",
        )
        return validated

    def bind(self, game):
        game.player_control.space.interactive_obstacles = self.objects.obstacles
        game.world.live_entities.update(self.objects.objects)
        super().bind(game)

    def payload(self, game):
        payload = super().payload(game) | {"schema_version": 3, "objects": self.objects.snapshot()}
        if not self.previous_layout:
            payload["layout"] = self.layout_revision
        return payload

    def _advance(self):
        shown = []
        while True:
            advanced = False
            for key in self.manager.definitions:
                for objective in self.manager.ready(key):
                    if objective.id not in self.facts:
                        continue
                    record = self.facts[objective.id]
                    if self.manager.accept(
                        Event(
                            objective.event,
                            self.owner,
                            objective.target,
                            tuple(record["position"]),
                            record["time"],
                            id=f"arrival:{key.lower()}:{objective.id}",
                        )
                    ):
                        advanced = True
                        shown.append(objective.title)
            if not advanced:
                break
        if shown:
            self.notice = f"COMPLETED: {shown[-1]}"
            self.notice_until = self.time + 5

    def _record(self, events):
        """Store only validated outputs, preserving their original world position."""
        for event in events:
            fact = ACTIONS.get((event.type, event.target))
            if fact:
                self._observe(fact, event.location)
                if fact == "key_taken" and "locked_attempt" in self.facts:
                    self._observe("key_with_context", event.location)

    def _explore(self, game, position, occupied):
        def local(identity):
            return np.asarray(game.world.scene.instances[identity].position)

        if np.linalg.norm((position - local("service_garage"))[[0, 2]]) <= 45:
            self._observe("compound", position)
        if np.linalg.norm((position - local("hangar_01"))[[0, 2]]) <= 100:
            self._observe("airbase", position)
        if np.linalg.norm((position - local("hangar_b"))[[0, 2]]) <= 65:
            self._observe("locate_b", position)
        if self._inside(game, "hangar_01", position) and not occupied:
            self._observe("hangar", position)
            self._observe("explore_a", position)
        if not occupied and self._inside(game, "hangar_b", position):
            self._observe("enter_b", position)

    def _foot(self, game, occupants):
        control = game.player_control
        pilot, space = control.pilot, control.space
        if any(
            self._inside(game, identity, pilot.position)
            for identity in (
                "service_garage",
                "service_shed",
                "service_store",
                "service_workshop",
            )
        ):
            self._observe("service", pilot.position)
        jeep = control.registry[JEEP]
        self.can_inspect = "discover" in self.facts and JEEP_PROFILE.visible(
            pilot, jeep, space, inspect=True
        )
        if "discover" not in self.facts and JEEP_PROFILE.visible(pilot, jeep, space):
            self._observe("discover", pilot.position)
        candidates = self.objects.candidates(pilot, space)
        for obj in candidates:
            if obj.identity in EQUIPMENT:
                self._observe(EQUIPMENT[obj.identity], obj.position)
        ids = [o.identity for o in candidates]
        index = ids.index(self._selected_id) if self._selected_id in ids else 0
        if game.input.object_next_pressed and ids:
            index = (index + 1) % len(ids)
        self.selection = candidates[index] if candidates else None
        self.selection_position = (index + 1, len(candidates)) if candidates else (0, 0)
        self._selected_id = self.selection.identity if self.selection else None
        if game.input.inspect_pressed:
            if self.selection:
                events, self.notice_detail = self.objects.interact(
                    self.selection.identity,
                    self.owner,
                    pilot,
                    space,
                    self.time,
                    occupants,
                    action="INSPECT" if game.input.pilot_run else None,
                )
                self.inspected_object = next(
                    (event.target for event in events if event.type == "ENTITY_INSPECTED"),
                    None,
                )
                self._record(events)
                if (
                    self.selection.identity == "personnel_door"
                    and self.notice_detail == "DOOR LOCKED - KEY REQUIRED"
                ):
                    self._observe("locked_attempt", pilot.position)
                self.detail_until = self.time + 5
            else:
                self.inspect(pilot, jeep, space)

    def update(self, game, dt, previous_access, previous_drive_position):
        self.time += max(dt, 0)
        control = game.player_control
        current = control.active
        position = current.vehicle.position if current else control.pilot.position
        old_count = len(self.facts)
        revision = self.objects.revision
        command = game.input
        if command.journal_pressed:
            self.journal_open = not self.journal_open
        if command.journal_next_pressed and self.journal_open:
            self.journal_page += 1
        if command.hint_pressed:
            active = "Q002" if self.manager.status["Q001"] == Status.COMPLETED else "Q001"
            hints = self.manager.definitions[active].hints
            self.hint_index = (self.hint_index + 1) % len(hints)
            self.notice = hints[self.hint_index]
            self.notice_until = self.time + 7
        occupants = actor_bounds(control, game.world.ground_vehicles)
        self._explore(game, position, current is not None)
        if current is None:
            self._foot(game, occupants)
        else:
            self.selection = None
            self.can_inspect = False
            if current is not previous_access:
                self._observe("board", position)
            elif (
                dt > 0
                and command.pilot_advance
                and not command.fire_gun
                and not current.vehicle.parking_brake
                and abs(current.vehicle.speed) > 0.1
                and current.vehicle.contacts.any()
            ):
                delta = position - previous_drive_position
                self.driven = min(self.drive_distance, self.driven + np.linalg.norm(delta[[0, 2]]))
                if self.driven >= self.drive_distance:
                    self._observe("drive", position)
                    hangar = np.asarray(game.world.scene.instances["hangar_01"].position)
                    garage = np.asarray(game.world.scene.instances["service_garage"].position)
                    if (
                        np.linalg.norm((position - garage)[[0, 2]]) > 50
                        and np.linalg.norm((position - hangar)[[0, 2]]) > 100
                        and delta[[0, 2]] @ (hangar - position)[[0, 2]] > 0
                    ):
                        self._observe("journey", position)
        completed = self.objects.tick(dt, occupants)
        for identity in completed:
            fact = {"office_drawer": "drawer", "main_gate": "gate"}.get(identity)
            if fact:
                self._observe(fact, self.objects.objects[identity].position)
        if (
            "control_used" in self.facts
            and self.objects.objects["main_gate"].state == "OPEN"
            and current is not None
            and self._inside(game, "hangar_b", position)
        ):
            self._observe("jeep_inside", position)
        if "crane_exam" in self.facts and "trolley_exam" in self.facts:
            self._observe("equipment_found", position)
        if len(self.facts) != old_count:
            self._advance()
        if self.manager.status["Q001"] == Status.COMPLETED:
            self._observe("q1_complete", position)
        changed = len(self.facts) != old_count or bool(completed)
        actions = command.inspect_pressed and revision != self.objects.revision
        if changed or actions or current is not previous_access or self.time - self.last_save >= 10:
            self.save(game)

    def _journal_entries(self):
        q1, q2 = (dict(self.manager.definitions[key].journal) for key in ("Q001", "Q002"))
        templates = (
            q1
            | q2
            | {
                "journey": JOURNEY,
                "q1_complete": q1["complete"],
                "equipment_found": q2["trolley_exam"],
            }
        )
        if "key_with_context" in self.facts:
            templates["key_taken"] = KEY_WITH_CONTEXT
        return (
            q1["initial"],
            *(templates[key] for key in self.facts if key in templates and key != "trolley_exam"),
        )

    @property
    def objective_label(self):
        key = "Q002" if self.manager.status["Q001"] == Status.COMPLETED else "Q001"
        return self.manager.ready(key)[0].title

    def hud_lines(self):
        key = "Q002" if self.manager.status["Q001"] == Status.COMPLETED else "Q001"
        quest = self.manager.definitions[key]
        if self.journal_open:
            progress = self.manager.progress[key]
            objectives = [
                f"{'DONE' if progress[o.id] else 'OPEN'} / {o.title}" for o in quest.objectives
            ]
            pages = [tuple(objectives[i : i + 7]) for i in range(0, len(objectives), 7)]
            pages.extend(tuple(wrap(entry, 66)) for entry in self._journal_entries())
            page = self.journal_page % len(pages)
            return (
                f"JOURNAL / Q001 {self.manager.status['Q001'].value} / I CLOSE / N NEXT",
                f"{key} / {quest.title}",
                f"PAGE {page + 1}/{len(pages)}",
                *pages[page],
            )
        lines = [f"{key} / {quest.title}", self.objective_label, "I JOURNAL / U HINT"]
        if self.selection and self.objects.visible(self.selection):
            action = self.objects.action(self.selection)
            name = self.selection.spec["name"]
            lines.append(f"T {action} {name} / O SELECT OBJECT")
            if action != "INSPECT":
                lines.append("SHIFT+T INSPECT SELECTED OBJECT")
        elif self.can_inspect:
            lines.append("T INSPECT UTILITY JEEP")
        if self.inspection_until > self.time and self.can_inspect:
            lines.extend((JEEP_PROFILE.title, JEEP_PROFILE.condition, JEEP_PROFILE.actions))
        if self.detail_until > self.time:
            lines.extend(wrap(self.notice_detail, 66))
        if self.notice_until > self.time:
            lines.extend(wrap(self.notice, 66))
        if self.time < 12:
            lines.extend(OPENING)
        return tuple(lines)
