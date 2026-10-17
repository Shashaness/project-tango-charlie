"""State-driven battledroid visuals with optional native glTF gait and procedural fallback."""

import math

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_animation import AnimationPlayer
from game.animation_clips import AnimationClip, JointOffset, blend_pose
from game.battledroid_locomotion import (
    IDLE_SPEED_THRESHOLD,
    RUN_SPEED_THRESHOLD,
    TURN_RATE_THRESHOLD,
)
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.tc167_poses import rotation

STATES = ("AIRBORNE", "LANDING", "RECOVERY", "IDLE", "WALKING", "RUNNING", "TURNING", "STOPPING")
RUN_EXIT_SPEED = RUN_SPEED_THRESHOLD - 0.5


class BattledroidAnimationController:
    """Layer contact/idle clips over gait while preserving engine-to-foot attachments."""

    def __init__(self, locomotion, native_enabled=True):
        """Load contact clips and bind the procedural gait/model collaborators.

        Args:
            locomotion: Procedural gait controller whose model receives the contact layer.
            native_enabled: Use imported BD_Idle/BD_Walk/BD_Run clips when available.
        """
        self.locomotion = locomotion
        self.transformation = locomotion.transformation
        self.model = locomotion.model
        self.native_enabled = native_enabled
        self.native_player = AnimationPlayer(self.model)
        self.native_clip = None
        self._native_attachments = set()
        self.clips = {
            name: AnimationClip.load(asset_path("animations/" + name + ".json"))
            for name in ("landing", "recovery", "idle", "stopping")
        }
        names = set().union(*(clip.tracks.keys() for clip in self.clips.values()))
        self.nodes = {name: self.model.find_node(name) for name in names}
        self.state = "IDLE"
        self.time = 0.0
        self.total_time = 0.0
        self.severity = 0.0
        self.landing_duration = 0.22
        self.recovery_duration = 0.45
        self.last_touchdown = 0
        self.landing_count = 0
        self.running = False
        self.previous_speed = 0.0
        self.pose = {}
        self.fade_from = {}
        self.fade_time = 0.12
        self.weight = 0.0
        self._was_transforming = False
        self._enabled = False

    def clear_layer(self):
        # Called BEFORE canonical transformation and procedural gait evaluation.
        self.native_player.clear()
        for node in self._native_attachments:
            node.set_animation_pose()
        self._native_attachments.clear()
        for node in self.nodes.values():
            node.set_animation_delta()

    def _enter(self, state):
        """Start an animation state and capture the outgoing pose for a timed fade.

        Args:
            state: Animation state name to enter.
        """
        if state == self.state:
            return
        self.fade_from = dict(self.pose)
        self.fade_time = 0.0
        self.state = state
        self.time = 0.0

    def prepare(self, dt, vehicle, pilot):
        """Select contact/gait state and advance timers from read-only vehicle observations.

        Args:
            dt: Elapsed simulation time in seconds.
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            pilot: Pilot input sampled for this update; signed axes are normalized.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Invalid animation dt")
        c = self.transformation
        completed = self._was_transforming and not c.active
        self._was_transforming = c.active
        self._enabled = (
            self.locomotion.enabled
            and not c.active
            and not completed
            and c.configuration is VehicleMode.BATTLEDROID
            and vehicle.flight_state.mode is VehicleMode.BATTLEDROID
            and vehicle.flight_state.environment is Environment.ATMOSPHERE
            and vehicle.flight_state.status is not FlightStatus.CRASHED
        )
        ground = vehicle.ground_contact
        event = ground.touchdown_id
        # Sample actual tangent velocity; the current collision surface is static.
        normal = np.asarray(ground.normal)
        normal = normal / max(np.linalg.norm(normal), 1e-8)
        velocity = vehicle.velocity - np.dot(vehicle.velocity, normal) * normal
        speed = float(np.linalg.norm(velocity))
        self.total_time += dt
        self.time += dt
        self.fade_time += dt
        if not self._enabled:
            self.last_touchdown = event
            self.running = False
        elif vehicle.flight_state.status is not FlightStatus.GROUNDED:
            self._enter("AIRBORNE")
            self.last_touchdown = event
        elif event != self.last_touchdown:
            self.last_touchdown = event
            self.landing_count += 1
            self.severity = float(np.clip(ground.touchdown_impact_speed / 14.0, 0.15, 1.0))
            self.landing_duration = 0.1 + 0.12 * self.severity
            self.recovery_duration = 0.18 + 0.65 * self.severity
            self._enter("LANDING")
        else:
            # Carry overshoot across both timed states, independent of dt partition.
            if self.state == "LANDING" and self.time >= self.landing_duration:
                remaining = self.time - self.landing_duration
                self._enter("RECOVERY")
                self.time = remaining
                self.fade_time = 0.12
            if self.state == "RECOVERY" and self.time >= self.recovery_duration:
                self._enter("IDLE")
            if self.state not in ("LANDING", "RECOVERY"):
                self.running = speed > (RUN_EXIT_SPEED if self.running else RUN_SPEED_THRESHOLD)
                decelerating = speed < self.previous_speed - 1e-5
                commanded = abs(getattr(pilot, "pitch", 0)) + abs(getattr(pilot, "roll", 0)) > 1e-8
                if speed >= IDLE_SPEED_THRESHOLD:
                    state = (
                        "STOPPING"
                        if decelerating or not commanded
                        else ("RUNNING" if self.running else "WALKING")
                    )
                elif self.state == "STOPPING" and self.time < self.clips["stopping"].duration:
                    state = "STOPPING"
                elif abs(vehicle.yaw_rate) > TURN_RATE_THRESHOLD:
                    state = "TURNING"
                else:
                    state = "IDLE"
                self._enter(state)
        self.previous_speed = speed
        commanded = abs(getattr(pilot, "pitch", 0)) + abs(getattr(pilot, "roll", 0)) > 1e-8
        self.locomotion.gait_allowed = self._enabled and (
            self.state in ("WALKING", "RUNNING", "TURNING")
            or (self.state == "STOPPING" and commanded and speed >= IDLE_SPEED_THRESHOLD)
        )
        self.weight = (
            min(1.0, self.weight + 5 * dt) if self._enabled else max(0.0, self.weight - 5 * dt)
        )
        if (
            completed
            or (not c.active and c.configuration is not VehicleMode.BATTLEDROID)
            or not self.locomotion.enabled
        ):
            self.weight = 0.0
            self.pose = {}
            self.fade_from = {}
        self._prepare_native(dt, speed)

    def _native_reference(self):
        """Sample canonical local TRS without decomposing matrices or reading physics."""
        return {
            node: node.configuration_trs
            for node in self.model.nodes
            if node.configuration_trs is not None
        }

    def _prepare_native(self, dt, speed):
        """Choose optional clips from the existing state machine and advance their visual clocks.

        Args:
            dt: Elapsed simulation time in seconds.
            speed: Actual surface-tangent movement speed in m/s.
        """
        name = {"IDLE": "BD_Idle", "WALKING": "BD_Walk", "RUNNING": "BD_Run"}.get(self.state)
        available = self.native_enabled and self._enabled and name in self.model.animations
        self.locomotion.native_active = bool(available)
        self.native_clip = name if available else None
        if not available:
            self.native_player.stop()
            for node in self._native_attachments:
                node.set_animation_pose()
            self._native_attachments.clear()
            return
        if self.native_player.clip is None or self.native_player.clip.name != name:
            self.native_player.play(name, transition=0.12, reference=self._native_reference())
        # Clip authors use nominal 2 m/s walk and 6 m/s run cycles; only visual
        # playback is scaled. No motion, throttle, or contact values are written.
        nominal = 2.0 if name == "BD_Walk" else 6.0
        self.native_player.speed = 1.0 if name == "BD_Idle" else speed / nominal
        self.native_player.update(dt)
        self.pose = {}
        self.fade_from = {}

    @staticmethod
    def _relative_matrix(node, ancestor):
        """Compose locals below an ancestor without inverting animated root scale.

        Args:
            node: Descendant ModelNode whose relative transform is needed.
            ancestor: Shared engine parent in the preserved TC167 hierarchy.

        Returns:
            Affine local-to-ancestor 4x4 matrix, excluding the ancestor itself.
        """
        matrix = np.eye(4)
        while node is not ancestor:
            if node is None:
                raise ValueError("Native foot attachment requires the existing TC167 hierarchy")
            matrix = node.local_matrix @ matrix
            node = node.parent
        return matrix

    def _apply_native(self):
        """Apply absolute clip TRS once and keep unauthored engines attached to their feet."""
        self.model.world_matrices()
        feet = {side: self.model.find_node(side + "Foot") for side in ("Left", "Right")}
        engines = {side: self.model.find_node(side + "Engine") for side in feet}
        rest_feet = {
            side: self._relative_matrix(feet[side], engine.parent)
            for side, engine in engines.items()
        }
        rest_engines = {side: engine.configuration_matrix for side, engine in engines.items()}
        targeted = self.native_player.apply(self._native_reference())
        for side, engine in engines.items():
            if engine in targeted:
                continue  # Explicit engine channels own their local pose and attachment.
            desired = (
                self._relative_matrix(feet[side], engine.parent)
                @ np.linalg.inv(rest_feet[side])
                @ rest_engines[side]
            )
            # Cancel the common ancestor algebraically: visual root motion/scale
            # reaches feet and engines once, even with zero animated root scale.
            engine.set_animation_pose(desired)
            self._native_attachments.add(engine)
        self.model.world_matrices()

    def update(self):
        """Compose the prepared local animation layer and refresh model world matrices."""
        if self.locomotion.native_active:
            self._apply_native()
            return
        if self._enabled:
            if self.state == "AIRBORNE":
                target = {
                    s + "LowerLeg": JointOffset(rotation=rotation(6)) for s in ("Left", "Right")
                }
            elif self.state in ("LANDING", "RECOVERY"):
                clip = self.clips[self.state.lower()]
                duration = (
                    self.landing_duration if self.state == "LANDING" else self.recovery_duration
                )
                time = self.time / duration * clip.duration
                target = blend_pose({}, clip.sample(time), self.severity)
            elif self.state in ("IDLE", "STOPPING"):
                target = self.clips[self.state.lower()].sample(
                    self.total_time if self.state == "IDLE" else self.time
                )
            else:
                target = {}
            u = min(1.0, self.fade_time / 0.12)
            u = u * u * (3 - 2 * u)
            self.pose = blend_pose(self.fade_from, target, u)
        layer = blend_pose({}, self.pose, self.weight)
        if not layer:
            return
        self.model.world_matrices()
        feet = {s: self.model.find_node(s + "Foot").world_matrix.copy() for s in ("Left", "Right")}
        engines = {s: self.model.find_node(s + "Engine").world_matrix.copy() for s in feet}
        # Existing delta is this frame's procedural gait, never last frame's output.
        for name, offset in layer.items():
            # Preserve M17 articulation bounds while the last gait fades under
            # contact clips. Quaternion angular distance bounds the additive turn.
            part = next(
                (
                    part
                    for part in ("UpperLeg", "LowerLeg", "Foot", "UpperArm")
                    if name.endswith(part)
                ),
                None,
            )
            limits = {"UpperLeg": 28.0, "LowerLeg": 38.0, "Foot": 6.0, "UpperArm": 18.0}
            if part is not None:
                gait = self.locomotion.joint_angles.get(name, (0.0, 0.0, 0.0))
                available = max(0.0, limits[part] - sum(abs(value) for value in gait))
                angle = math.degrees(2 * math.acos(float(np.clip(abs(offset.rotation[3]), 0, 1))))
                if angle > available:
                    q = np.asarray(offset.rotation)
                    if q[3] < 0:
                        q = -q
                    axis = q[:3] / max(np.linalg.norm(q[:3]), 1e-12)
                    half = math.radians(available) / 2
                    offset = JointOffset(
                        offset.translation, tuple(np.r_[axis * math.sin(half), math.cos(half)])
                    )
            node = self.nodes[name]
            node.compose_animation_delta(offset.matrix())
        self.model.world_matrices()
        for side in feet:
            engine = self.model.find_node(side + "Engine")
            desired = (
                np.linalg.inv(engine.parent.world_matrix)
                @ self.model.find_node(side + "Foot").world_matrix
                @ np.linalg.inv(feet[side])
                @ engines[side]
            )
            # Desired above includes root motion exactly once through animated feet.
            canonical = engine.configuration_matrix
            engine.set_animation_delta(np.linalg.inv(canonical) @ desired)
        self.model.world_matrices()
