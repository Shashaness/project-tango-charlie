"""Connect the Arrival scenario to the existing world and player controllers."""

import math
from pathlib import Path
from textwrap import wrap

import numpy as np

from engine.asset_paths import PROJECT_ROOT, asset_path
from game.discovery import Inspectable
from game.player_control import ControlMode
from game.quests.definitions import load_quests
from game.quests.manager import Event, QuestManager, Status
from game.quests.persistence import read_save, write_save
from game.world_loader import default_scene, scene_model
from game.world_manifest import load_manifest, object_fields, require, vector

WORLD = asset_path("worlds/arrival/world_q12.json")
QUESTS = asset_path("quests/arrival_q12.json")
DEFAULT_SAVE = PROJECT_ROOT / "saves" / "arrival.json"
JEEP = "hangar-jeep"
JEEP_PROFILE = Inspectable(
    "UTILITY JEEP",
    "GROUND TRANSPORT FOR THE AIRBASE.",
    "APPEARS OPERATIONAL.",
    "ENTER BOARD / P PARKING BRAKE / WASD DRIVE",
)
DAMAGE = dict(
    airframe=0.20,
    landing_gear=0.0,
    main_propulsion=0.0,
    vtol_propulsion=0.0,
    flight_controls=0.08,
    electrical=0.25,
    avionics=0.15,
    communications=0.10,
    transformation=0.0,
)
OPENING = (
    "KARA SAW A CITY AND AIMED FOR THE AIRPORT.",
    "SHE BLACKED OUT BEFORE IMPACT.",
    "TC167 LIES WRECKED. THE HANGAR IS STILL STANDING.",
)


def number(value, label, lower, upper):
    require(
        type(value) in (int, float) and math.isfinite(value) and lower <= value <= upper,
        f"Invalid {label}",
    )
    return float(value)


class ArrivalCampaign:
    """A new or resumed Arrival campaign, independent of window construction."""

    def __init__(
        self, path=DEFAULT_SAVE, *, new=False, world=WORLD, quests=QUESTS, extra_entities=()
    ):
        self.path = Path(path).resolve()
        require(
            not any(
                self.path.is_relative_to(PROJECT_ROOT / directory)
                for directory in ("assets", "lore", ".git")
            ),
            "Choose a runtime save path outside authored content and Git metadata",
        )
        self.definition = load_manifest(world)
        self.owner = self.definition.controlled_entity
        self.jeep_ids = {
            e.identity for e in self.definition.entities if e.asset.kind == "ground_vehicle"
        }
        self.manager = QuestManager(
            load_quests(
                quests, {e.identity for e in self.definition.entities} | set(extra_entities)
            )
        )
        self.drive_distance = next(
            o.distance_m for o in self.manager.definitions["Q001"].objectives if o.id == "drive"
        )
        require(self.drive_distance is not None, "Q001 needs an operating distance")
        self.facts = {}
        self.driven = 0.0
        self.journal_open = False
        self.journal_page = 0
        self.hint_index = -1
        self.notice = ""
        self.notice_until = self.inspection_until = 0.0
        self.can_inspect = False
        self.time = 0.0
        self.damage = DAMAGE.copy()
        self.migrated = False
        self.saved = None if new or not self.path.exists() else self._validate(read_save(self.path))
        self.last_save = self.time

    def _migrate(self, raw):
        """Carry Q1.1 observations forward and place a still-pristine Jeep in storage."""
        quests = raw["quests"]
        progress = quests["progress"]["Q001"]
        legacy = {"airbase", "hangar", "service", "discover", "inspect", "drive"}
        object_fields(progress, "Q1.1 progress", legacy)
        require(
            type(progress["airbase"]) is int
            and progress["airbase"] in (0, 1)
            and all(
                type(progress[key]) is int and progress[key] == 0 for key in legacy - {"airbase"}
            ),
            "Invalid Q1.1 objective progress",
        )
        stamp = {"position": raw["pilot"]["position"], "time": raw["elapsed"]}
        raw["facts"] = {"airbase": stamp} if progress["airbase"] else {}
        raw["driven"] = 0.0
        state = raw["jeeps"][JEEP]
        legacy_position = vector(state["position"], "legacy Jeep position")
        require(type(state["brake"]) is bool, "Invalid legacy parking brake")
        number(state["speed"], "legacy speed", -100, 100)
        number(state["heading"], "legacy heading", -1e8, 1e8)
        if raw["occupied"] == JEEP:
            raw["facts"]["board"] = {"position": state["position"], "time": raw["elapsed"]}
        else:
            old = default_scene().instances[JEEP]
            if (
                raw["occupied"] is None
                and state["brake"]
                and state["speed"] == 0
                and np.allclose(
                    legacy_position[::2],
                    old.position[::2],
                    atol=1e-6,
                    rtol=0,
                )
                and math.isclose(
                    state["heading"], math.atan2(old.forward[0], old.forward[2]), abs_tol=1e-6
                )
            ):
                state["position"] = None  # Use the new authored spawn in bind.
        progress["board"] = 0
        quests["progress"]["Q002"] = {"find_equipment": 0}
        quests["status"]["Q002"] = "LOCKED"
        raw["schema_version"] = 2
        return raw

    def _validate(self, raw):
        require(isinstance(raw, dict), "Invalid campaign save")
        version = raw.get("schema_version")
        require(type(version) is int and version in (1, 2), "Unsupported campaign save")
        migrating = version == 1
        if migrating:
            object_fields(
                raw,
                "Q1.1 campaign save",
                {
                    "schema_version",
                    "campaign",
                    "elapsed",
                    "pilot",
                    "occupied",
                    "jeeps",
                    "aircraft",
                    "quests",
                },
            )
            try:
                raw = self._migrate(raw)
            except (KeyError, TypeError, OverflowError) as error:
                raise ValueError("Invalid Q1.1 campaign save") from error
        object_fields(
            raw,
            "campaign save",
            {
                "schema_version",
                "campaign",
                "elapsed",
                "pilot",
                "occupied",
                "facts",
                "driven",
                "jeeps",
                "aircraft",
                "quests",
            },
        )
        require(
            type(raw["schema_version"]) is int
            and raw["schema_version"] == 2
            and raw["campaign"] == self.definition.identity,
            "Unsupported campaign save",
        )
        elapsed = number(raw["elapsed"], "elapsed time", 0, 1e12)
        require(
            raw["occupied"] is None
            or (isinstance(raw["occupied"], str) and raw["occupied"] in self.jeep_ids),
            "Invalid campaign occupancy",
        )
        pilot = object_fields(raw["pilot"], "pilot", {"position", "forward"})
        vector(pilot["position"], "pilot position")
        forward = vector(pilot["forward"], "pilot forward")
        require(
            abs(forward[1]) < 1e-5
            and math.isclose(math.hypot(forward[0], forward[2]), 1, abs_tol=1e-5),
            "Invalid pilot facing",
        )
        object_fields(raw["jeeps"], "Jeeps", self.jeep_ids)
        for identity, state in raw["jeeps"].items():
            object_fields(state, identity, {"position", "heading", "brake", "speed", "steering"})
            if not migrating or state["position"] is not None:
                vector(state["position"], "Jeep position")
            number(state["heading"], "Jeep heading", -1e8, 1e8)
            number(state["speed"], "Jeep speed", -100, 100)
            number(state["steering"], "Jeep steering", -10, 10)
            require(type(state["brake"]) is bool, "Invalid Jeep parking brake")
        aircraft = object_fields(raw["aircraft"], "aircraft", {"position", "damage"})
        vector(aircraft["position"], "aircraft position")
        object_fields(aircraft["damage"], "aircraft damage", set(DAMAGE))
        require(
            all(
                number(value, key, 0, 1) == DAMAGE[key] for key, value in aircraft["damage"].items()
            ),
            "Arrival supports the initial aircraft damage profile",
        )
        facts = raw["facts"]
        driven = number(raw["driven"], "driven distance", 0, self.drive_distance)
        valid = self.observation_ids()
        object_fields(facts, "observations", set(), valid)
        for record in facts.values():
            object_fields(record, "observation", {"position", "time"})
            vector(record["position"], "observation position")
            number(record["time"], "observation time", 0, elapsed)
        require("inspect" not in facts or "discover" in facts, "Inspection requires discovery")
        require(("drive" in facts) == (driven >= self.drive_distance), "Invalid driving progress")
        self.manager.restore(raw["quests"])
        require(
            all(not value or key in facts for key, value in self.manager.progress["Q001"].items()),
            "Quest progress requires a matching observation",
        )
        self.facts, self.driven = facts.copy(), driven
        self.time = elapsed
        self.damage = aircraft["damage"].copy()
        self.migrated = migrating
        return raw

    def observation_ids(self):
        return {o.id for o in self.manager.definitions["Q001"].objectives}

    def bind(self, game):
        control = game.player_control
        game.player_vehicle.subsystem_health = self.damage
        control.aircraft.entry_disabled = "TC167 IS SEVERELY DAMAGED"
        raw = self.saved
        if raw is None:
            self.save(game)
            return
        game.player_vehicle.position[:] = raw["aircraft"]["position"]
        control.aircraft.deactivate(control.space)
        pilot = control._pilot
        pilot.position[:] = raw["pilot"]["position"]
        pilot.forward[:] = raw["pilot"]["forward"]
        for vehicle in game.world.ground_vehicles:
            state = raw["jeeps"][vehicle.identity]
            if state["position"] is None:
                continue
            vehicle.position[:] = state["position"]
            vehicle.heading = state["heading"]
            vehicle.parking_brake = state["brake"]
            vehicle.speed = state["speed"]
            vehicle.steering = state["steering"]
            vehicle.velocity[:] = vehicle.forward * vehicle.speed
        if raw["occupied"] is not None:
            control.selected = control.registry[raw["occupied"]]
            control.selected.activate(pilot, control.space)
            control.mode = ControlMode.GROUND_VEHICLE
            control.pilot = None
            game.driving_camera.begin(control.selected.vehicle)
        else:
            game.pedestrian_camera.begin(pilot)
        if self.migrated:
            self.save(game)

    def _inside(self, game, identity, position):
        placement = game.world.scene.instances[identity]
        lo, hi = scene_model(placement.definition.asset.path).bounds()
        local = np.linalg.solve(np.asarray(placement.matrix), (*position, 1))[:3]
        return (
            np.all(local[[0, 2]] > lo[[0, 2]] + 1)
            and np.all(local[[0, 2]] < hi[[0, 2]] - 1)
            and lo[1] - 0.1 <= local[1] < hi[1] - 0.1
        )

    def _observe(self, fact, position):
        if fact in self.facts:
            return False
        self.facts[fact] = {"position": list(position), "time": self.time}
        return True

    def _advance(self):
        shown = []
        while True:
            progress = False
            for objective in self.manager.ready("Q001"):
                if objective.id in self.facts:
                    record = self.facts[objective.id]
                    if self.manager.accept(
                        Event(
                            objective.event,
                            self.owner,
                            objective.target,
                            tuple(record["position"]),
                            record["time"],
                            id=f"arrival:q001:{objective.id}",
                        )
                    ):
                        shown.append(objective.title)
                        progress = True
            if not progress:
                break
        if shown:
            self.notice = (
                "Q001 COMPLETE"
                if self.manager.status["Q001"] == Status.COMPLETED
                else f"COMPLETED: {shown[-1]}"
            )
            self.notice_until = self.time + 5

    def inspect(self, pilot, access, space):
        """Record an intentional inspection after discovery and a fresh sight-line check."""
        if (
            pilot.grounded
            and "discover" in self.facts
            and access.identity == JEEP
            and JEEP_PROFILE.visible(pilot, access, space, inspect=True)
        ):
            self.inspection_until = self.time + 7
            return self._observe("inspect", pilot.position)
        return False

    def update(self, game, dt, previous_access, previous_drive_position):
        self.time += max(dt, 0)
        control = game.player_control
        current = control.active
        occupied_changed = previous_access is not current
        position = control.pilot.position if current is None else current.vehicle.position
        jeep = control.registry[JEEP]
        command = game.input
        if command.journal_pressed:
            self.journal_open = not self.journal_open
        if command.journal_next_pressed and self.journal_open:
            self.journal_page = (self.journal_page + 1) % len(self._journal_entries())
        if command.hint_pressed and self.manager.status["Q001"] == Status.ACTIVE:
            self.hint_index = (self.hint_index + 1) % len(self.manager.definitions["Q001"].hints)
            self.notice = self.manager.definitions["Q001"].hints[self.hint_index]
            self.notice_until = self.time + 7
        changed = False
        hangar = game.world.scene.instances["hangar_01"].position
        if np.linalg.norm((position - hangar)[[0, 2]]) <= 100:
            changed |= self._observe("airbase", position)
        self.can_inspect = False
        if control.pilot is not None and control.pilot.grounded:
            for fact, identity in (("hangar", "hangar_01"), ("service", "service_garage")):
                if fact not in self.facts and self._inside(game, identity, position):
                    changed |= self._observe(fact, position)
            if "discover" not in self.facts and JEEP_PROFILE.visible(
                control.pilot, jeep, control.space
            ):
                changed |= self._observe("discover", position)
            self.can_inspect = "discover" in self.facts and JEEP_PROFILE.visible(
                control.pilot, jeep, control.space, inspect=True
            )
            if command.inspect_pressed:
                changed |= self.inspect(control.pilot, jeep, control.space)
        if occupied_changed:
            if previous_access is not None:
                self.manager.accept(
                    Event(
                        "VEHICLE_EXITED",
                        self.owner,
                        previous_access.identity,
                        tuple(previous_access.vehicle.position),
                        self.time,
                    )
                )
            if current is jeep:
                changed |= self._observe("board", position)
        elif (
            current is jeep
            and self.driven < self.drive_distance
            and dt > 0
            and (
                command.pilot_advance
                and not jeep.vehicle.parking_brake
                and not command.fire_gun
                and abs(jeep.vehicle.speed) > 0.1
                and jeep.vehicle.contacts.any()
            )
        ):
            # A static seat, a pose change, or a stalled wheel does not count as driving.
            distance = np.linalg.norm((jeep.vehicle.position - previous_drive_position)[[0, 2]])
            self.driven = min(self.drive_distance, self.driven + float(distance))
            if self.driven == self.drive_distance:
                changed |= self._observe("drive", position)
        if changed:
            self._advance()
        if changed or occupied_changed or self.time - self.last_save >= 10:
            self.save(game)

    def save(self, game):
        write_save(self.path, self.payload(game))
        self.last_save = self.time

    def payload(self, game):
        control = game.player_control
        pilot = control._pilot
        return {
            "schema_version": 2,
            "campaign": self.definition.identity,
            "elapsed": self.time,
            "facts": self.facts,
            "driven": self.driven,
            "pilot": {"position": pilot.position.tolist(), "forward": pilot.forward.tolist()},
            "occupied": control.active.identity if control.active else None,
            "jeeps": {
                v.identity: {
                    "position": v.position.tolist(),
                    "heading": float(v.heading),
                    "brake": v.parking_brake,
                    "speed": float(v.speed),
                    "steering": float(v.steering),
                }
                for v in game.world.ground_vehicles
            },
            "aircraft": {
                "position": game.player_vehicle.position.tolist(),
                "damage": self.damage,
            },
            "quests": self.manager.snapshot(),
        }

    def _journal_entries(self):
        templates = dict(self.manager.definitions["Q001"].journal)
        keys = ("initial", *self.facts)
        if self.manager.status["Q001"] == Status.COMPLETED:
            keys = (*keys, "complete")
        return tuple(templates[key] for key in keys if key in templates)

    @property
    def objective_label(self):
        if self.manager.status["Q001"] == Status.COMPLETED:
            return "Q002 / THE MEANS OF RECOVERY - AWAITING FUTURE CONTENT"
        objective = self.manager.ready("Q001")[0]
        if objective.id == "drive":
            return f"{objective.title}: {self.driven:.0f}/{self.drive_distance:.0f} M"
        return objective.title

    def hud_lines(self):
        quest = self.manager.definitions["Q001"]
        if self.journal_open:
            objectives = tuple(
                f"{'DONE' if self.manager.progress['Q001'][o.id] else 'OPEN'} / {o.title}"
                for o in quest.objectives
                if o.id != "drive" or self.manager.progress["Q001"]["board"]
            )
            entries = self._journal_entries()
            page = self.journal_page % len(entries)
            return (
                f"JOURNAL / {quest.title} / I CLOSE / N NEXT ENTRY",
                *objectives,
                self.objective_label,
                f"ENTRY {page + 1}/{len(entries)}",
                *wrap(entries[page], width=66),
            )
        lines = [f"Q001 / {quest.title}", self.objective_label]
        if self.can_inspect:
            lines.append("T INSPECT UTILITY JEEP")
        if self.inspection_until > self.time and self.can_inspect:
            lines.extend(
                (
                    JEEP_PROFILE.title,
                    JEEP_PROFILE.description,
                    JEEP_PROFILE.condition,
                    JEEP_PROFILE.actions,
                )
            )
        if self.notice_until > self.time:
            lines.extend(wrap(self.notice, width=66))
        if self.time < 12:
            lines.extend(OPENING)
        lines.append("I JOURNAL / U HINT")
        return tuple(lines)
