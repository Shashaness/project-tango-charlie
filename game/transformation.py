"""Small reversible staged pose controller; never owns vehicle momentum."""

import math

import numpy as np

from engine.gltf_loader import load_glb
from engine.quaternion import multiply, slerp
from game.flight_state import FlightStatus, VehicleMode
from game.prototype_model import PROTOTYPE_PATH
from game.tc167_poses import (
    BATTLEDROID_TIMINGS,
    DURATIONS,
    ENGINE_OFFSETS,
    ENGINE_ROTATIONS,
    PHYSICS_SWITCH,
    POSES,
    TIMINGS,
    PoseTransform,
)


def phase(t, start=0, end=1):
    """Map a timing window to a clamped cubic smoothstep.

    Args:
        t: Dimensionless interpolation fraction in [0, 1].
        start: Normalized beginning of the interpolation window.
        end: Normalized end of the interpolation window.

    Returns:
        Dimensionless eased progress in [0, 1].
    """
    if not 0 <= start < end <= 1:
        raise ValueError("Timing window must satisfy 0 <= start < end <= 1")
    value = float(np.clip((t - start) / (end - start), 0, 1))
    return value * value * (3 - 2 * value)


class TransformationController:
    """Evaluate reversible staged poses and switch configuration without owning momentum."""

    authority = "Legacy"

    def __init__(self, vehicle=None, model=None):
        """Load or retain the model, cache rest/attachment data, and select fighter pose.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            model: CPU model hierarchy and mesh data.
        """
        self.vehicle = vehicle
        self.model = load_glb(PROTOTYPE_PATH) if model is None else model
        self.rest = {n.name: PoseTransform(*n.base_trs) for n in self.model.nodes}
        self.pose_nodes = set().union(*(p.keys() for p in POSES.values()))
        for name in self.pose_nodes:
            self.model.find_node(name)
        self.engine_offsets = {}
        for side in ("Left", "Right"):
            self.model.world_matrices()
            foot = self.model.find_node(side + "Foot").world_matrix
            engine = self.model.find_node(side + "Engine").world_matrix
            self.engine_offsets[side] = np.linalg.inv(foot) @ engine
        self.configuration = VehicleMode.FIGHTER
        self.source = self.target = self.configuration
        self.progress = 1.0
        self.duration = 0.0
        self.active = False
        self.queue = []
        self._edge = None
        self._coordinate = 0.0
        self._direction = 1
        self.current_pose = {}
        self.reset(self.configuration)

    def _pose(self, mode, name):
        """Look up a configuration pose with an imported-rest fallback.

        Args:
            mode: Vehicle configuration to sample or activate.
            name: Model node or state name.

        Returns:
            PoseTransform for the named model node.
        """
        return POSES[mode].get(name, self.rest[name])

    def apply_curve(self, first, last, t):
        """Evaluate the canonical staged edge and update model poses and cached matrices.

        Args:
            first: Source vehicle configuration.
            last: Target vehicle configuration.
            t: Dimensionless interpolation fraction in [0, 1].
        """
        windows = BATTLEDROID_TIMINGS if last is VehicleMode.BATTLEDROID else TIMINGS
        self.model.reset_pose()
        self.current_pose = {}
        for name in self.pose_nodes:
            a = self._pose(first, name)
            b = self._pose(last, name)
            part = next((key for key in windows if key in name), (None))
            u = phase(t, *windows.get(part, (0, 1)))
            if t <= 0:
                value = a
            elif t >= 1:
                value = b
            else:
                value = PoseTransform(
                    tuple((1 - u) * np.array(a.translation) + u * np.array(b.translation)),
                    tuple(slerp(a.rotation, b.rotation, u)),
                    tuple((1 - u) * np.array(a.scale) + u * np.array(b.scale)),
                )
            self.current_pose[name] = value
            node = self.model.find_node(name)
            # Exact imported matrices for every untouched rest endpoint.
            if value != self.rest[name]:
                node.set_pose(value.translation, value.rotation, value.scale)
        self.model.world_matrices()
        engine_u = phase(t, *windows["Engine"])
        adjustment = slerp(ENGINE_ROTATIONS[first], ENGINE_ROTATIONS[last], engine_u)
        for side in ("Left", "Right"):
            # Recover the foot attachment in the engine parent’s local space.
            engine = self.model.find_node(side + "Engine")
            # Engine local pose must exclude its complete parent transform,
            # including the visual root datum. Otherwise root lift is applied twice.
            foot = (
                np.linalg.inv(engine.parent.world_matrix)
                @ self.model.find_node(side + "Foot").world_matrix
            )
            attached = foot @ self.engine_offsets[side]
            # Rotation composition from actual joint chain, no matrix-to-Euler conversion.
            q = (0, 0, 0, 1)
            chain = []
            node = self.model.find_node(side + "Foot")
            while node is not None:
                chain.append(node)
                node = node.parent
            for node in reversed(chain):
                q = multiply(q, self.current_pose.get(node.name, self.rest[node.name]).rotation)
            q = multiply(q, adjustment)
            if t <= 0 and first is VehicleMode.FIGHTER:
                engine.reset_pose()
            else:
                offset = (1 - engine_u) * np.array(ENGINE_OFFSETS[first]) + engine_u * np.array(
                    ENGINE_OFFSETS[last]
                )
                engine.set_pose(attached[:3, 3] + offset, q)
        # Engine overrides were applied after the ankle traversal. Refresh their
        # descendants too: exhaust consumers must never read the previous pose.
        self.model.world_matrices()

    def reset(self, mode=VehicleMode.FIGHTER):
        """Development reset only; leaves physical state and inventories alone.

        Args:
            mode: Vehicle configuration to sample or activate.
        """
        self.active = False
        self.queue = []
        self.configuration = mode
        self.source = self.target = mode
        self.progress = 1.0
        self._edge = None
        if mode is VehicleMode.FIGHTER:
            self.model.reset_pose()
            self.current_pose = {}
            self.model.world_matrices()
        elif mode is VehicleMode.VTOL:
            self.apply_curve(VehicleMode.FIGHTER, mode, 1)
        else:
            self.apply_curve(VehicleMode.VTOL, mode, 1)

    def request(self, target):
        """Queue a configuration change, inserting VTOL between fighter and battledroid.

        Args:
            target: Target entity, desired value, or configuration.
        """
        if target not in POSES:
            raise ValueError("Unknown transformation configuration")
        if self.active:
            raise ValueError("Use reverse() for an active transformation")
        if target is self.configuration:
            return
        if {target, self.configuration} == {VehicleMode.FIGHTER, VehicleMode.BATTLEDROID}:
            self.queue = [VehicleMode.VTOL, target]
        else:
            self.queue = [target]
        self._start_next()

    def cycle(self):
        """Request the next configuration or reverse an active transition in place."""
        if self.active:
            self.reverse()
            return
        order = (VehicleMode.FIGHTER, VehicleMode.VTOL, VehicleMode.BATTLEDROID)
        self.request(order[(order.index(self.configuration) + 1) % 3])

    def _start_next(self):
        """Consume the next queued edge and initialize its canonical curve coordinate."""
        self.source = self.configuration
        self.target = self.queue.pop(0)
        order = (VehicleMode.FIGHTER, VehicleMode.VTOL, VehicleMode.BATTLEDROID)
        self._edge = tuple(sorted((self.source, self.target), key=order.index))
        self._direction = 1 if self.source is self._edge[0] else -1
        self._coordinate = 0.0 if self._direction == 1 else 1.0
        self.duration = DURATIONS[self._edge]
        self.progress = 0.0
        self.active = True
        self.apply_curve(*self._edge, self._coordinate)

    def reverse(self):
        """Reverse the current curve direction without moving nodes at the reversal instant."""
        if not self.active:
            return
        self.source, self.target = self.target, self.source
        self._direction *= -1
        self.queue = []
        self.progress = 1 - self.progress
        # Same curve coordinate: not a single node moves when direction changes.

    def _switch_physics(self, mode):
        """Change the vehicle mode and release grounded state when required.

        Args:
            mode: Vehicle configuration to sample or activate.
        """
        if self.vehicle is None:
            return
        self.vehicle.flight_state.mode = mode
        if (
            mode not in (VehicleMode.VTOL, VehicleMode.BATTLEDROID)
            and self.vehicle.flight_state.status is FlightStatus.GROUNDED
        ):
            self.vehicle.flight_state.status = FlightStatus.FLYING

    def update(self, dt):
        """Advance staged poses and configuration hysteresis, carrying time across queued edges.

        Args:
            dt: Elapsed simulation time in seconds.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Transformation delta time must be finite and nonnegative")
        remaining = dt
        while self.active:
            # Consume only time up to this edge's endpoint. Carry any remainder into
            # the next queued edge so a long frame follows the same canonical path.
            distance = (1 - self._coordinate) if self._direction == 1 else self._coordinate
            elapsed = min(remaining, distance * self.duration)
            self._coordinate = float(
                np.clip(self._coordinate + self._direction * elapsed / self.duration, 0, 1)
            )
            completed = distance * self.duration - elapsed <= 1e-10
            if completed:
                self._coordinate = 1.0 if self._direction == 1 else 0.0
            self.progress = self._coordinate if self._direction == 1 else 1 - self._coordinate
            self.apply_curve(*self._edge, self._coordinate)
            # Directional 60% hysteresis prevents immediate flips on reversal.
            if self.progress + 1e-12 >= PHYSICS_SWITCH:
                self._switch_physics(self.target)
            remaining = max(0.0, remaining - elapsed)
            if completed:
                self._switch_physics(self.target)
                self.configuration = self.target
                self.active = False
                self.progress = 1.0
                if self.queue:
                    self._start_next()
                else:
                    break
            if remaining <= 1e-12:
                break

    @property
    def label(self):
        if self.active:
            return f"{self.source.name} > {self.target.name} {self.progress * 100:.0f}%"
        return self.configuration.name
