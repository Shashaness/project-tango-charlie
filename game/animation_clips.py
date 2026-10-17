"""Original rigid-node offset clips. No world motion, skinning or GL dependency."""

import json
import math
from dataclasses import dataclass

import numpy as np

from engine.model import trs
from engine.quaternion import multiply, normalize, slerp

IDENTITY = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class JointOffset:
    """Immutable local translation in meters and quaternion rotation in [x, y, z, w] order."""

    translation: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = IDENTITY

    def matrix(self):
        return trs(self.translation, self.rotation)


def blend_pose(a, b, weight):
    """Blend local offsets, treating absent joints as identity.

    Args:
        a: First node-to-JointOffset pose mapping.
        b: Second node-to-JointOffset pose mapping.
        weight: Dimensionless blend weight.

    Returns:
        Dictionary of node names to interpolated JointOffset values.
    """
    result = {}
    for name in a.keys() | b.keys():
        left, right = a.get(name, JointOffset()), b.get(name, JointOffset())
        result[name] = JointOffset(
            tuple((1 - weight) * np.array(left.translation) + weight * np.array(right.translation)),
            tuple(slerp(left.rotation, right.rotation, weight)),
        )
    return result


def additive_pose(base, layer, weight=1.0):
    """Compose weighted local offsets in order without mutating the input poses.

    Args:
        base: Base pose mapping node names to local offsets.
        layer: Additive pose mapping node names to local offsets.
        weight: Dimensionless blend weight.

    Returns:
        Dictionary of composed JointOffset values.
    """
    weighted = blend_pose({}, layer, weight)
    result = dict(base)
    for name, value in weighted.items():
        old = base.get(name, JointOffset())
        # Translation of the next local layer rotates with the previous layer.
        translation = np.array(old.translation) + old.matrix()[:3, :3] @ value.translation
        result[name] = JointOffset(
            tuple(translation), tuple(multiply(old.rotation, value.rotation))
        )
    return result


class AnimationClip:
    """Validate and sample rigid-node keyframes using seconds and shortest-arc rotations."""

    def __init__(self, data):
        """Validate rigid-node tracks and retain ordered local-space keyframes.

        Args:
            data: Decoded clip data with target channels and keyframe times in seconds.
        """
        self.name = data["name"]
        self.duration = float(data["duration"])
        self.loop = data.get("loop", False)
        if (
            not math.isfinite(self.duration)
            or self.duration <= 0
            or not isinstance(self.loop, bool)
        ):
            raise ValueError("Clip duration must be positive; loop must be boolean")
        self.tracks = {}
        for name, channels in data["targets"].items():
            if not name or set(channels) - {"translation", "rotation"}:
                raise ValueError("Invalid target or channel")
            tracks = {}
            for channel, keys in channels.items():
                parsed = []
                for key in keys:
                    time = float(key["time"])
                    value = np.asarray(key["value"], dtype=float)
                    size = 4 if channel == "rotation" else 3
                    if (
                        not math.isfinite(time)
                        or not 0 <= time <= self.duration
                        or value.shape != (size,)
                        or not np.isfinite(value).all()
                        or (parsed and time <= parsed[-1][0])
                    ):
                        raise ValueError("Invalid animation keyframe")
                    if channel == "rotation":
                        value = normalize(value)
                    parsed.append((time, value))
                if not parsed:
                    raise ValueError("Empty animation channel")
                tracks[channel] = parsed
            self.tracks[name] = tracks

    @classmethod
    def load(cls, path):
        """Read and validate a JSON rigid-node animation clip.

        Args:
            path: Filesystem path to the input or output asset.

        Returns:
            Validated AnimationClip instance.
        """
        with open(path, encoding="utf-8") as stream:
            return cls(json.load(stream))

    def sample(self, time):
        """Sample a looping or endpoint-clamped clip with linear translation and quaternion
        SLERP.

        Args:
            time: Clip sample time in seconds.

        Returns:
            Dictionary mapping node names to local JointOffset values.
        """
        if not math.isfinite(time) or time < 0:
            raise ValueError("Invalid clip time")
        time = time % self.duration if self.loop else min(time, self.duration)
        pose = {}
        for name, channels in self.tracks.items():
            values = {}
            for channel, keys in channels.items():
                value = keys[-1][1]
                if time <= keys[0][0]:
                    value = keys[0][1]
                else:
                    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
                        if time <= t1:
                            u = (time - t0) / (t1 - t0)
                            # Translation is linear in local space; rotation follows
                            # the shortest quaternion arc rather than Euler angles.
                            value = (
                                slerp(v0, v1, u) if channel == "rotation" else (1 - u) * v0 + u * v1
                            )
                            break
                values[channel] = tuple(value)
            pose[name] = JointOffset(**values)
        return pose


class ClipPlayer:
    """Track playback time in seconds independently of vehicle physics."""

    def __init__(self, clip):
        """Retain the clip and initialize playback time to zero seconds.

        Args:
            clip: Validated animation clip.
        """
        self.clip = clip
        self.time = 0.0

    @property
    def finished(self):
        return not self.clip.loop and self.time >= self.clip.duration

    def update(self, dt):
        """Advance playback time and sample the clip.

        Args:
            dt: Elapsed simulation time in seconds.

        Returns:
            Local joint-offset pose at the updated playback time.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Invalid playback dt")
        self.time += dt
        return self.clip.sample(self.time)
