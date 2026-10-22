"""CPU-only rigid-node glTF clips, TRS sampling, and nonaccumulating visual playback."""

import math
from dataclasses import dataclass

import numpy as np

from engine.model import trs
from engine.quaternion import normalize, slerp

PATH_COMPONENT = {"translation": 0, "rotation": 1, "scale": 2}


@dataclass(frozen=True)
class AnimationSampler:
    """Immutable keyframes: seconds, local meters/quaternions/unitless scales."""

    times: np.ndarray
    values: np.ndarray
    interpolation: str = "LINEAR"

    def __post_init__(self):
        times = np.array(self.times, dtype=np.float64, copy=True)
        values = np.array(self.values, dtype=np.float64, copy=True)
        if (
            times.ndim != 1
            or not len(times)
            or not np.isfinite(times).all()
            or times[0] < 0
            or np.any(np.diff(times) <= 0)
            or values.ndim != 2
            or len(values) != len(times) * (3 if self.interpolation == "CUBICSPLINE" else 1)
            or values.shape[1] not in (3, 4)
            or not np.isfinite(values).all()
        ):
            raise ValueError("Invalid animation keyframe times or values")
        if self.interpolation not in ("LINEAR", "STEP", "CUBICSPLINE"):
            raise ValueError(f"Unsupported animation interpolation: {self.interpolation}")
        times.setflags(write=False)
        values.setflags(write=False)
        object.__setattr__(self, "times", times)
        object.__setattr__(self, "values", values)

    def sample(self, time, rotation=False):
        """Sample seconds with endpoint clamping and shortest-arc quaternion LINEAR.

        Args:
            time: Finite nonnegative clip time in seconds.
            rotation: Whether these values are [x, y, z, w] quaternions.

        Returns:
            Independent local TRS component array.
        """
        if not math.isfinite(time) or time < 0:
            raise ValueError("Animation sample time must be finite and nonnegative")
        if self.interpolation == "CUBICSPLINE":
            keys = self.values.reshape(len(self.times), 3, -1)
            i = int(np.searchsorted(self.times, time, side="right") - 1)
            if i < 0:
                value = keys[0, 1].copy()
            elif i >= len(self.times) - 1:
                value = keys[-1, 1].copy()
            else:
                span = self.times[i + 1] - self.times[i]
                u = (time - self.times[i]) / span
                value = (
                    (2 * u**3 - 3 * u**2 + 1) * keys[i, 1]
                    + (u**3 - 2 * u**2 + u) * span * keys[i, 2]
                    + (-2 * u**3 + 3 * u**2) * keys[i + 1, 1]
                    + (u**3 - u**2) * span * keys[i + 1, 0]
                )
            return normalize(value) if rotation else value
        index = int(np.searchsorted(self.times, time, side="right") - 1)
        if index < 0:
            return self.values[0].copy()
        if index >= len(self.times) - 1 or self.interpolation == "STEP":
            return self.values[index].copy()
        weight = (time - self.times[index]) / (self.times[index + 1] - self.times[index])
        left, right = self.values[index : index + 2]
        return (
            slerp(left, right, float(weight)) if rotation else (1 - weight) * left + weight * right
        )


@dataclass(frozen=True)
class AnimationChannel:
    """One sampler bound to an original glTF node index and its CPU node object."""

    node_index: int
    node: object
    path: str
    sampler: AnimationSampler

    def __post_init__(self):
        if self.path not in PATH_COMPONENT or self.node.base_trs is None:
            raise ValueError("Animation requires a translation/rotation/scale target with rest TRS")
        expected = 4 if self.path == "rotation" else 3
        if self.sampler.values.shape[1] != expected:
            raise ValueError("Animation output width does not match target path")
        if self.path == "rotation" and self.sampler.interpolation == "CUBICSPLINE":
            # Validate values only. Tangents are derivatives, not unit quaternions.
            for value in self.sampler.values[1::3]:
                normalize(value)
        if self.path == "rotation" and self.sampler.interpolation != "CUBICSPLINE":
            values = np.array([normalize(value) for value in self.sampler.values])
            object.__setattr__(
                self,
                "sampler",
                AnimationSampler(self.sampler.times, values, self.sampler.interpolation),
            )


@dataclass(frozen=True)
class AnimationClip:
    """Named glTF animation with absolute local TRS channels and a seconds timeline."""

    name: str
    channels: tuple

    def __post_init__(self):
        channels = tuple(self.channels)
        targets = [(channel.node, channel.path) for channel in channels]
        if not isinstance(self.name, str) or not self.name or not channels:
            raise ValueError("Animation clips require a nonempty name and channels")
        if len(set(targets)) != len(targets):
            raise ValueError("Duplicate/conflicting animation channels for the same node and path")
        object.__setattr__(self, "channels", channels)

    @property
    def duration(self):
        """Final key time in seconds; the clip timeline begins at zero."""
        return max(float(channel.sampler.times[-1]) for channel in self.channels)

    def sample(self, time):
        """Sample absolute local components without writing model or physics state.

        Args:
            time: Finite nonnegative time in seconds, clamped by each channel's endpoints.

        Returns:
            Mapping of (ModelNode, TRS path) to independent local component arrays.
        """
        return {
            (channel.node, channel.path): channel.sampler.sample(time, channel.path == "rotation")
            for channel in self.channels
        }


class AnimationPlayer:
    """Play named clips and crossfade component values; own only visual animation state.

    Crossfades freeze the outgoing evaluated pose, including interrupted fades, and
    advance the incoming clip. Missing components blend against the supplied reference
    pose (imported rest by default). Fade seconds are independent of playback speed.
    """

    def __init__(self, model):
        """Bind model clip metadata and initialize stopped playback without allocating GPU data.

        Args:
            model: CPU Model whose animations mapping supplies the named clips.
        """
        self.model = model
        self.clips = model.animations
        self.clip = None
        self.time = 0.0
        self.speed = 1.0
        self.loop = True
        self.playing = False
        self.finished = False
        self._source = {}
        self._fade_time = self._fade_duration = 0.0
        self._owned = set()

    def play(self, name, *, loop=True, speed=1.0, transition=0.0, reference=None):
        """Start a named clip, optionally blending from the current pose.

        Args:
            name: Exact imported clip name; unknown names raise KeyError.
            loop: Whether to wrap at the final key time.
            speed: Finite nonnegative playback multiplier; zero holds clip time.
            transition: Blend duration in real seconds; zero switches immediately.
            reference: Optional node-to-TRS reference for missing crossfade components.
        """
        clip = self.clips[name]
        if not math.isfinite(speed) or speed < 0:
            raise ValueError("Playback speed must be finite and nonnegative")
        if not math.isfinite(transition) or transition < 0:
            raise ValueError("Transition duration must be finite and nonnegative")
        self._source = self.sample(reference) if transition > 0 and self.clip is not None else {}
        self.clip = clip
        self.time = 0.0
        self.speed = speed
        self.loop = bool(loop)
        self.playing = True
        self.finished = False
        self._fade_time = 0.0
        self._fade_duration = transition if self._source else 0.0

    def pause(self):
        """Hold clip time and transition progress while retaining the sampled pose."""
        self.playing = False

    def resume(self):
        """Resume a paused clip or its unfinished crossfade."""
        if self.clip is not None and (not self.finished or self._fade_time < self._fade_duration):
            self.playing = True

    def stop(self):
        """Clear owned visual overrides and return to the current configuration/rest pose."""
        self.clear()
        self.clip = None
        self.time = 0.0
        self.playing = False
        self.finished = False
        self._source = {}
        self._fade_time = self._fade_duration = 0.0

    def update(self, dt):
        """Advance clip and blend clocks by finite nonnegative elapsed seconds."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Animation delta time must be finite and nonnegative")
        if not math.isfinite(self.speed) or self.speed < 0:
            raise ValueError("Playback speed must be finite and nonnegative")
        if self.clip is None or not self.playing:
            return
        self._fade_time = min(self._fade_duration, self._fade_time + dt)
        time = self.time + dt * self.speed
        duration = self.clip.duration
        if self.loop and duration > 0:
            self.time = time % duration
            # Avoid a frame partition leaving a mathematically exact loop at its end.
            if math.isclose(self.time, duration, rel_tol=0, abs_tol=1e-12):
                self.time = 0.0
        else:
            self.time = min(time, duration)
            if time >= duration:
                self.finished = True
                # Continue fading a short one-shot until the visual transition completes.
                self.playing = self._fade_time < self._fade_duration
        if self._fade_time >= self._fade_duration:
            self._source = {}

    @staticmethod
    def _reference(node, path, reference):
        pose = node.base_trs if reference is None else reference.get(node, node.base_trs)
        return np.asarray(pose[PATH_COMPONENT[path]], dtype=float)

    def sample(self, reference=None):
        """Sample targeted components, blending missing paths against rest/configuration TRS.

        Args:
            reference: Optional node-to-local-TRS mapping; None uses imported rest poses.

        Returns:
            Mapping of (ModelNode, TRS path) to local component arrays.
        """
        if self.clip is None:
            return {}
        target = self.clip.sample(self.time)
        if not self._source:
            return target
        weight = min(1.0, self._fade_time / self._fade_duration)
        result = {}
        for key in self._source.keys() | target.keys():
            node, path = key
            rest = self._reference(node, path, reference)
            left, right = self._source.get(key, rest), target.get(key, rest)
            result[key] = (
                slerp(left, right, weight)
                if path == "rotation"
                else (1 - weight) * left + weight * right
            )
        return result

    def clear(self):
        """Clear only this player's absolute visual overrides; retain configuration state."""
        for node in self._owned:
            node.set_animation_pose()
        self._owned.clear()

    def apply(self, reference=None):
        """Rebuild local visual matrices from the sample, never the previous frame.

        Args:
            reference: Optional node-to-(translation, rotation, scale) reference pose.
                Omitted channels use this pose; None uses immutable imported rest.

        Returns:
            Set of nodes receiving absolute visual poses. Physics is never accessed.
        """
        components = self.sample(reference)
        self.clear()
        poses = {}
        for (node, path), value in components.items():
            if node not in poses:
                poses[node] = [self._reference(node, p, reference).copy() for p in PATH_COMPONENT]
            poses[node][PATH_COMPONENT[path]] = value
        for node, pose in poses.items():
            node.set_animation_pose(trs(*pose))
        self._owned = set(poses)
        self.model.world_matrices()
        return self._owned.copy()
