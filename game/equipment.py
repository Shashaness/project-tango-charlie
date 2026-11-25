"""Equipment power, exclusive physical cargo slots, and scoped access credentials."""

import json
from dataclasses import dataclass
from enum import Enum

import numpy as np

from game.campaign import number
from game.interactive import InteractiveScene
from game.player_control import UP
from game.quests.manager import Event
from game.world_manifest import object_fields, require, vector


class Condition(str, Enum):
    SERVICEABLE = "serviceable"
    DETERIORATED = "deteriorated"
    FAILED = "failed"
    UNKNOWN = "unknown"


class Slot(str, Enum):
    WORLD = "world"
    RACK = "rack"
    CARRIED = "carried"
    VEHICLE = "vehicle"
    INSTALLED = "installed"


@dataclass(frozen=True)
class Battery:
    interface: str
    condition: Condition
    charge: float

    def supplies(self, interface, minimum):
        return (
            self.interface == interface
            and self.condition is Condition.SERVICEABLE
            and self.charge >= minimum
        )

    @property
    def charge_label(self):
        return "DEPLETED" if self.charge == 0 else f"{self.charge:.0%} CHARGE"


@dataclass(frozen=True)
class PowerRequirement:
    interface: str
    minimum_charge: float

    def satisfied_by(self, battery):
        return battery.supplies(self.interface, self.minimum_charge)


@dataclass(frozen=True)
class Credential:
    level: str
    facilities: tuple
    equipment: tuple
    vehicles: tuple

    def permits(self, kind, target):
        return target in getattr(self, kind)


@dataclass
class Cargo:
    slot: Slot
    position: tuple
    transport_m: float = 0.0


class RecoveryScene(InteractiveScene):
    """Attach cargo to runtime transforms while retaining the existing T/O interaction API."""

    def __init__(self, objects, recovery):
        config = json.loads(recovery.read_text())
        additions = config["objects"]
        authored = json.loads(objects.read_text())["objects"]
        self.legacy_object_ids = set(authored)
        shift = authored["trolley"]["position"][2] - 4470
        for spec in additions.values():
            spec["position"][2] += shift
        super().__init__(objects, additions)
        self.settings = config["cargo"]
        requirement = config["power"]["trolley"]
        self.trolley_power = PowerRequirement(
            requirement["interface"], number(requirement["minimum_charge"], "minimum charge", 0, 1)
        )
        self.credentials = {
            key: Credential(
                entry["level"], *(tuple(entry[k]) for k in ("facilities", "equipment", "vehicles"))
            )
            for key, entry in config["credentials"].items()
        }
        batteries = self.settings["batteries"]
        self.batteries = {
            key: Battery(
                spec["interface"],
                Condition(spec["condition"]),
                number(spec["charge"], key, 0, 1),
            )
            for key, spec in batteries.items()
        }
        self.cargo = {
            key: Cargo(
                Slot(spec["slot"]),
                self.objects[key].spec["position"],
            )
            for key, spec in batteries.items()
        }
        self.objects["office_key"].spec.update(
            name="MAINTENANCE ACCESS CARD",
            description="ELECTRONIC CREDENTIAL FOR HANGAR B.",
            size=(0.14, 0.026, 0.09),
        )
        self.objects["crane_station"].spec["name"] = "CRN-329 MOBILE CRANE"
        self.objects["trolley"].spec["name"] = "ATAT-773 RECOVERY TROLLEY"
        self.jeep = None
        self.pilot = None
        self.actor = None
        self.distance = 0.0
        self.last_feet = None
        self.last_visual = None

    def bind(self, jeep, pilot, actor):
        self.jeep, self.pilot, self.actor = jeep, pilot, actor
        self.last_feet = pilot.position.copy()

    def occupying(self, slot):
        return next((key for key, item in self.cargo.items() if item.slot is slot), None)

    def authorized(self, actor, kind, target):
        return actor is not None and any(
            self.credentials[key].permits(kind, target)
            for key in self.inventory(actor) & self.credentials.keys()
        )

    def unlock_allowed(self, obj, actor):
        if obj.identity == "personnel_door":
            return self.authorized(actor, "facilities", "hangar_b")
        return super().unlock_allowed(obj, actor)

    def compatible(self, key):
        return self.trolley_power.satisfied_by(self.batteries[key])

    @property
    def electrically_operational(self):
        installed = self.occupying(Slot.INSTALLED)
        return installed is not None and self.compatible(installed)

    @property
    def readiness(self):
        return {
            "towable": True,
            "powered": self.electrically_operational,
            "stabilized": False,
            "securing_available": self.electrically_operational,
            "ready_to_secure_aircraft": False,
            "ready_for_loaded_transport": False,
        }

    @property
    def movement_scale(self):
        return self.settings["carry_speed"] if self.occupying(Slot.CARRIED) else 1.0

    def transform(self, key):
        cargo = self.cargo[key]
        matrix = np.eye(4)
        if cargo.slot is Slot.VEHICLE and self.jeep is not None:
            matrix = self.jeep.model_matrix()
            matrix[:3, 3] = (matrix @ (*self.settings["jeep_load"], 1))[:3]
        elif cargo.slot is Slot.CARRIED and self.pilot is not None:
            forward = self.pilot.forward
            matrix[:3, :3] = np.column_stack((np.cross(UP, forward), UP, forward))
            matrix[:3, 3] = self.pilot.position + 0.77 * forward + 1.05 * UP
        else:
            point = (
                self.objects["battery_original"].spec["position"]
                if cargo.slot is Slot.INSTALLED
                else cargo.position
            )
            matrix[:3, 3] = point
        return matrix

    def _point(self, obj):
        if obj.identity in self.cargo:
            return self.transform(obj.identity)[:3, 3]
        if obj.identity == "jeep_cargo_bay" and self.jeep is not None:
            return (self.jeep.model_matrix() @ (*self.settings["jeep_access"], 1))[:3]
        return super()._point(obj)

    def render_rows(self):
        for obj in self.objects.values():
            if (
                obj.identity not in self.cargo
                and self.visible(obj)
                and "visual" not in obj.spec
                and not obj.spec.get("marker")
            ):
                yield (*self._bounds(obj), obj.spec["color"])

    def oriented_boxes(self):
        if self.jeep is not None:
            basis = self.jeep.model_matrix()
            for part in self.settings["jeep_carrier"]:
                matrix = basis.copy()
                matrix[:3, 3] = (basis @ (*part["center"], 1))[:3]
                half = np.asarray(part["size"]) / 2
                yield -half, half, part["color"], matrix
        for key in self.cargo:
            obj = self.objects[key]
            half = np.asarray(obj.spec["size"]) / 2
            yield -half, half, obj.spec["color"], self.transform(key)

    def in_reach(self, obj, pilot, space):
        cargo = self.cargo.get(obj.identity)
        if cargo and cargo.slot in (Slot.VEHICLE, Slot.INSTALLED):
            return False
        return super().in_reach(obj, pilot, space)

    def solid(self, obj):
        cargo = self.cargo.get(obj.identity)
        if cargo:
            return cargo.slot in (Slot.WORLD, Slot.RACK, Slot.INSTALLED)
        return super().solid(obj)

    def action(self, obj):
        cargo = self.cargo.get(obj.identity)
        if cargo:
            return "SET DOWN" if cargo.slot is Slot.CARRIED else "LIFT"
        if obj.identity == "jeep_cargo_bay":
            if self.occupying(Slot.VEHICLE):
                return "UNLOAD BATTERY"
            return "LOAD BATTERY" if self.occupying(Slot.CARRIED) else "INSPECT"
        if obj.identity == "trolley_battery_bay":
            if self.occupying(Slot.INSTALLED):
                return "INSPECT" if self.electrically_operational else "REMOVE BATTERY"
            return "INSTALL BATTERY" if self.occupying(Slot.CARRIED) else "INSPECT"
        return super().action(obj)

    def describe(self, obj):
        key = obj.identity
        if key in self.cargo:
            battery = self.batteries[key]
            compatible = (
                "ATAT-773 COMPATIBLE"
                if battery.interface == self.trolley_power.interface
                else ("INCOMPATIBLE WITH ATAT-773")
            )
            return f"{battery.condition.value.upper()} / {battery.charge_label} / {compatible}"
        if key == "trolley":
            installed = self.occupying(Slot.INSTALLED)
            label = self.batteries[installed].charge_label if installed else "MISSING"
            state = "ONLINE" if self.electrically_operational else "OFFLINE"
            ending = (
                "RECOVERY CONTROLS AVAILABLE."
                if self.electrically_operational
                else ("REPLACEMENT BATTERY REQUIRED.")
            )
            return (
                f"AIRCRAFT RECOVERY PLATFORM. MECHANICAL TOWING AVAILABLE. BATTERY {label}. "
                f"STABILIZATION AND CARGO SECURING {state}. {ending}"
            )
        if key == "trolley_battery_bay":
            installed = self.occupying(Slot.INSTALLED)
            return self.describe(self.objects[installed]) if installed else "BATTERY SLOT EMPTY."
        if key == "jeep_cargo_bay":
            loaded = self.occupying(Slot.VEHICLE)
            return f"LOADED: {self.objects[loaded].spec['name']}" if loaded else "CARGO AREA EMPTY."
        if key == "crane_station":
            available = self.authorized(self.actor, "vehicles", "crn-329")
            return "MOBILE RECOVERY CRANE. VEHICLE AUTHORIZATION REQUIRED. " + (
                "COMPATIBLE CARD PRESENT." if available else "COMPATIBLE ACCESS CARD NOT PRESENT."
            )
        return super().describe(obj)

    def _drop(self, key, pilot, space):
        directions = (
            pilot.forward,
            -pilot.forward,
            np.cross(pilot.forward, UP),
            np.cross(UP, pilot.forward),
        )
        half = np.asarray(self.objects[key].spec["size"]) / 2
        for direction in directions:
            feet = pilot.position + direction * 1.05
            feet[1] = space.terrain(feet)
            point = feet + UP * half[1]
            overlap = any(
                other != key
                and np.all(
                    np.abs(self.transform(other)[:3, 3] - point)
                    < half + np.asarray(self.objects[other].spec["size"]) / 2
                )
                for other in self.cargo
            )
            if not overlap and not space.blocked(
                feet, radius=max(half[[0, 2]]), height=2 * half[1]
            ):
                self.cargo[key].position = tuple(point)
                self._transfer(key, Slot.WORLD)
                return True
        return False

    def _transfer(self, key, slot):
        cargo = self.cargo[key]
        cargo.slot = slot
        if slot is Slot.VEHICLE:
            cargo.transport_m = 0.0
        self.distance = 0.0
        if self.pilot is not None:
            self.last_feet = self.pilot.position.copy()
        self.revision += 1

    def tick_cargo(self, control, previous_drive_position):
        if self.pilot is None:
            return False
        released = False
        carried = self.occupying(Slot.CARRIED)
        if carried is not None:
            feet = self.pilot.position
            self.distance += np.linalg.norm((feet - self.last_feet)[[0, 2]])
            if self.distance >= self.settings["handle_distance"]:
                released = self._drop(carried, self.pilot, control.space)
            self.last_feet = feet.copy()
        loaded = self.occupying(Slot.VEHICLE)
        if loaded and (
            previous_drive_position is not None
            and control.active is not None
            and control.active.vehicle is self.jeep
        ):
            delta = self.jeep.position - previous_drive_position
            self.cargo[loaded].transport_m += float(np.linalg.norm(delta[[0, 2]]))
        moving = (
            tuple(self.jeep.model_matrix().flat),
            *(
                tuple(self.transform(key).flat)
                for key, item in self.cargo.items()
                if item.slot in (Slot.CARRIED, Slot.VEHICLE)
            ),
        )
        if moving != self.last_visual:
            self.revision += 1
            self.last_visual = moving
        return released

    def interact(self, identity, actor, pilot, space, now, occupants, *, action=None):
        obj = self.objects.get(identity)
        if obj is None or not self.in_reach(obj, pilot, space) or action == "INSPECT":
            return super().interact(identity, actor, pilot, space, now, occupants, action=action)
        if actor not in obj.spec.get("actors", (actor,)):
            return (), "ACCESS DENIED"
        expected = self.action(obj)
        if action is not None and action != expected:
            return (), "ACTION UNAVAILABLE"
        if expected == "INSPECT" or (
            identity not in self.cargo and identity not in ("jeep_cargo_bay", "trolley_battery_bay")
        ):
            return super().interact(identity, actor, pilot, space, now, occupants, action=action)
        carried = self.occupying(Slot.CARRIED)
        loaded = self.occupying(Slot.VEHICLE)
        installed = self.occupying(Slot.INSTALLED)

        def result(kind, key, message):
            return (
                Event(kind, actor, identity, tuple(pilot.position), now, data={"battery": key}),
            ), message

        if expected == "SET DOWN":
            if self._drop(identity, pilot, space):
                return result("CARGO_RELEASED", identity, "BATTERY SET DOWN")
            return (), "NO CLEAR SPACE TO SET DOWN THE BATTERY"
        if expected == "LIFT":
            if carried:
                return (), "SET DOWN THE BATTERY YOU ARE HOLDING FIRST"
            self._transfer(identity, Slot.CARRIED)
            return result("CARGO_LIFTED", identity, "BATTERY LIFTED")
        if identity == "jeep_cargo_bay":
            if self.jeep is None or max(abs(self.jeep.speed), abs(self.jeep.vertical_speed)) > 0.05:
                return (), "STOP THE JEEP BEFORE HANDLING CARGO"
            if loaded:
                if carried:
                    return (), "SET DOWN THE BATTERY YOU ARE HOLDING FIRST"
                half = np.asarray(self.objects[loaded].spec["size"]) / 2
                front = pilot.position + pilot.forward * 0.77 + UP * (1.05 - half[1])
                if space.blocked(front, radius=half[2], height=2 * half[1]):
                    return (), "MAKE ROOM TO UNLOAD BEHIND THE JEEP"
                self._transfer(loaded, Slot.CARRIED)
                return result("CARGO_UNLOADED", loaded, "BATTERY UNLOADED")
            size = self.objects[carried].spec["size"]
            if self.batteries[carried].interface not in self.settings["jeep_interfaces"]:
                return (), "CARGO INTERFACE INCOMPATIBLE WITH THE JEEP BAY"
            if any(a > b for a, b in zip(size, self.settings["bay_capacity"], strict=True)):
                return (), "BATTERY DOES NOT FIT THE JEEP CARGO AREA"
            self._transfer(carried, Slot.VEHICLE)
            return result("CARGO_LOADED", carried, "BATTERY SECURED IN JEEP CARGO AREA")
        if installed:
            if carried:
                return (), "SET DOWN THE BATTERY YOU ARE HOLDING FIRST"
            self._transfer(installed, Slot.CARRIED)
            return result("COMPONENT_REMOVED", installed, "BATTERY REMOVED FROM ATAT-773")
        if not self.compatible(carried):
            return (), "ATAT-773 REQUIRES A COMPATIBLE SERVICEABLE CHARGED BATTERY"
        self._transfer(carried, Slot.INSTALLED)
        return result(
            "COMPONENT_INSTALLED", carried, "POWER RESTORED: ATAT-773 RECOVERY CONTROLS AVAILABLE"
        )

    def cargo_snapshot(self):
        return {
            "distance": self.distance,
            "items": {
                key: {
                    "charge": self.batteries[key].charge,
                    "condition": self.batteries[key].condition.value,
                    "slot": item.slot.value,
                    "position": list(item.position),
                    "transport_m": item.transport_m,
                }
                for key, item in self.cargo.items()
            },
        }

    def restore_cargo(self, raw):
        object_fields(raw, "cargo", {"distance", "items"})
        object_fields(raw["items"], "cargo identities", set(self.cargo))
        replacement, batteries = {}, {}
        for key, state in raw["items"].items():
            object_fields(state, key, {"charge", "condition", "slot", "position", "transport_m"})
            slot = Slot(state["slot"])
            require(
                slot is not Slot.RACK or key != "battery_original",
                "Original trolley battery cannot occupy the service rack",
            )
            batteries[key] = Battery(
                self.batteries[key].interface,
                Condition(state["condition"]),
                number(state["charge"], key, 0, 1),
            )
            replacement[key] = Cargo(
                slot, vector(state["position"], key), number(state["transport_m"], key, 0, 1e9)
            )
        for slot in (Slot.CARRIED, Slot.VEHICLE, Slot.INSTALLED):
            require(
                sum(item.slot is slot for item in replacement.values()) <= 1,
                f"Duplicate cargo in {slot.value}",
            )
        distance = number(raw["distance"], "handling distance", 0, 1e6)
        require(
            not distance or any(item.slot is Slot.CARRIED for item in replacement.values()),
            "Only carried cargo has handling distance",
        )
        self.cargo, self.batteries, self.distance = replacement, batteries, distance
        self.revision += 1
