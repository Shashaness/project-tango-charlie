"""GLB-only TC167 configuration, transition and locomotion orchestration."""

import math

import numpy as np

from engine.gltf_animation import PATH_COMPONENT
from engine.model import trs
from engine.quaternion import slerp
from game.flight_state import Environment, FlightStatus, VehicleMode

CONFIGURATIONS = {
    VehicleMode.FIGHTER: "TC_Pose_Fighter",
    VehicleMode.VTOL: "TC_Pose_VTOL",
    VehicleMode.BATTLEDROID: "TC_Pose_Battledroid",
}
EDGES = {
    (VehicleMode.FIGHTER, VehicleMode.VTOL): "TC_FighterToVTOL",
    (VehicleMode.VTOL, VehicleMode.FIGHTER): "TC_VTOLToFighter",
    (VehicleMode.VTOL, VehicleMode.BATTLEDROID): "TC_VTOLToBattledroid",
    (VehicleMode.BATTLEDROID, VehicleMode.VTOL): "TC_BattledroidToVTOL",
}
REQUIRED_ACTIONS = (*CONFIGURATIONS.values(), *EDGES.values(), "BD_Idle", "BD_Walk", "BD_Run")


class AuthoredTransformationController:
    """Sole visual writer. Full local poses reset absent paths to configuration/rest.

    Reversal freezes the current pose, bridges for 0.15 seconds to the independently
    authored reverse Action's first frame, then plays that entire Action forward.
    The bridge clock is separate from clip duration. Physics switches at completion.
    """

    authority = "GLB"
    bridge_seconds = 0.15

    def __init__(self, vehicle=None, model=None):
        if model is None:
            raise ValueError("Authored TC167 requires a GLB model")
        self.model, self.vehicle = model, vehicle
        missing = set(REQUIRED_ACTIONS) - model.animations.keys()
        if missing:
            raise ValueError(
                "Missing authored Actions: "
                + ", ".join(sorted(missing))
                + "; select --legacy-animation for compatibility"
            )
        from game.reference_anchors import validate_anchors

        self.errors = validate_anchors(model)
        for name in REQUIRED_ACTIONS:
            if model.animations[name].duration <= 0:
                self.errors.append(f"{name}: required Action must have a positive duration")
        if self.errors:
            raise ValueError("Invalid authored references: " + "; ".join(self.errors))
        for clip in model.animations.values():
            for channel in clip.channels:
                if channel.node.name == "TC_ROOT":
                    raise ValueError(f"{clip.name}: TC_ROOT must remain simulation-owned")
        self.rest = {
            n: tuple(np.asarray(v, dtype=float) for v in n.base_trs)
            for n in model.nodes
            if n.base_trs is not None
        }
        self.paused = False
        self._bridge = None
        self._bridge_time = 0.0
        self._coordinate = 0.0
        self._direction = 1
        self._edge = None
        self.pose = {}
        self.reset()

    def _sample(self, name, time, reference=None):
        pose = {
            n: [v.copy() for v in values]
            for n, values in (self.rest if reference is None else reference).items()
        }
        for (node, path), value in self.model.animations[name].sample(time).items():
            pose[node][PATH_COMPONENT[path]] = value
        return pose

    def configuration_pose(self, mode):
        return self._sample(CONFIGURATIONS[mode], 0)

    def _apply(self, pose):
        self.pose = pose
        for node in self.model.nodes:
            node.reset_pose()
            if node in pose:
                node.set_animation_pose(trs(*pose[node]))
        self.model.world_matrices()

    @staticmethod
    def blend(left, right, weight):
        return {
            n: [
                (slerp(a, b, weight) if i == 1 else (1 - weight) * a + weight * b)
                for i, (a, b) in enumerate(zip(left[n], right[n]))
            ]
            for n in right
        }

    def reset(self, mode=VehicleMode.FIGHTER):
        self.configuration = self.source = self.target = mode
        self.active = False
        self.queue = []
        self.progress = 1.0
        self.duration = self.time = 0.0
        self._edge = None
        self._bridge = None
        self.paused = False
        self.action = CONFIGURATIONS[mode]
        self._locomotion = None
        self._apply(self.configuration_pose(mode))

    def request(self, target):
        if target not in CONFIGURATIONS:
            raise ValueError("Unknown configuration")
        # Active requests append a path from the pending destination.
        origin = (
            self.queue[-1] if self.queue else (self.target if self.active else self.configuration)
        )
        if origin is target:
            return
        path = [target] if (origin, target) in EDGES else [VehicleMode.VTOL, target]
        self.queue.extend(path)
        if not self.active:
            self._start_next(continuity=self._locomotion is not None)

    def _start_next(self, continuity=False):
        self.source = self.configuration
        self.target = self.queue.pop(0)
        self._start(self.source, self.target, continuity)

    def _start(self, source, target, continuity=False):
        frozen = self.pose
        self.source, self.target = source, target
        self.action = EDGES[source, target]
        self.duration = self.model.animations[self.action].duration
        if self.duration <= 0:
            raise ValueError(f"{self.action}: transformation duration must be positive")
        self._edge = tuple(sorted((source, target), key=lambda m: list(CONFIGURATIONS).index(m)))
        self.time = self.progress = self._coordinate = 0.0
        self._direction = 1
        self.active = True
        self._locomotion = None
        self._bridge = frozen if continuity else None
        self._bridge_time = 0.0
        if not continuity:
            self._apply(self._sample(self.action, 0, self.configuration_pose(source)))

    def reverse(self):
        if self.active:
            self.queue = []
            self._start(self.target, self.source, True)

    def interrupt(self, target):
        """Redirect continuously through the reverse edge, then queue a valid path."""
        if self.active:
            if target is self.target:
                self.queue = []
                return
            self.reverse()
            if target is not self.target:
                self.request(target)
        else:
            self.request(target)

    def cycle(self):
        if self.active:
            self.reverse()
        else:
            order = list(CONFIGURATIONS)
            self.request(order[(order.index(self.configuration) + 1) % len(order)])

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def restart(self):
        if self.active:
            self._start(self.source, self.target, True)
        else:
            self.reset(self.configuration)

    def update(self, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Animation delta must be finite and nonnegative")
        if self.paused:
            return
        while self.active:
            reference = self.configuration_pose(self.source)
            if self._bridge is not None:
                elapsed = min(dt, self.bridge_seconds - self._bridge_time)
                self._bridge_time += elapsed
                dt -= elapsed
                self._apply(
                    self.blend(
                        self._bridge,
                        self._sample(self.action, 0, reference),
                        self._bridge_time / self.bridge_seconds,
                    )
                )
                if self._bridge_time < self.bridge_seconds:
                    return
                self._bridge = None
            elapsed = min(dt, self.duration - self.time)
            self.time += elapsed
            dt -= elapsed
            self.progress = self._coordinate = self.time / self.duration
            self._apply(self._sample(self.action, self.time, reference))
            if self.time + 1e-10 < self.duration:
                return
            self.configuration = self.target
            if self.vehicle is not None:
                was_grounded = self.vehicle.is_grounded
                self.vehicle.flight_state.mode = self.target
                if self.target is VehicleMode.FIGHTER and was_grounded:
                    self.vehicle.flight_state.status = FlightStatus.FLYING
            self.active = False
            self.progress = 1.0
            self.action = CONFIGURATIONS[self.configuration]
            self.time = 0.0
            self._apply(self.configuration_pose(self.configuration))
            if self.queue:
                self._start_next()
            else:
                return

    def locomotion(self, dt, vehicle, enabled=True):
        """Absolute exported gait poses over authored BD configuration; no attachments/deltas."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Animation delta must be finite and nonnegative")
        if self.paused:
            return
        if self.active or self.configuration is not VehicleMode.BATTLEDROID or not enabled:
            if not self.active:
                self.action = CONFIGURATIONS[self.configuration]
                self.time = 0.0
                self._apply(self.configuration_pose(self.configuration))
            self._locomotion = None
            return
        grounded = (
            vehicle.flight_state.environment is Environment.ATMOSPHERE
            and vehicle.flight_state.status is FlightStatus.GROUNDED
        )
        speed = float(np.linalg.norm(vehicle.velocity[[0, 2]])) if grounded else 0.0
        name = "BD_Idle" if speed < 0.3 else "BD_Walk" if speed < 4.5 else "BD_Run"
        if self._locomotion != name:
            self._gait_source = self.pose
            self._gait_fade = self._gait_time = 0.0
            self._locomotion = name
        if not self.paused:
            self._gait_fade = min(0.12, self._gait_fade + dt)
            rate = 1 if name == "BD_Idle" else speed / (2 if name == "BD_Walk" else 6)
            self._gait_time = (self._gait_time + dt * rate) % self.model.animations[name].duration
        self.action = name
        pose = self._sample(name, self._gait_time, self.configuration_pose(VehicleMode.BATTLEDROID))
        self._apply(self.blend(self._gait_source, pose, self._gait_fade / 0.12))

    @property
    def label(self):
        return (
            f"{self.source.name} > {self.target.name} {self.progress:.0%}"
            if self.active
            else self.configuration.name
        )

    @property
    def diagnostics(self):
        time = self._gait_time if self._locomotion else self.time
        return (
            f"GLB {self.action} {time:.3f}/{self.model.animations[self.action].duration:.3f}s "
            f"{self.progress:.0%}" + (" PAUSED" if self.paused else "")
        )
