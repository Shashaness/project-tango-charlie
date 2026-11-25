"""Sample authored road centerlines for drawing, surface queries, and map export."""

import math

import numpy as np


class AccessRoad:
    def __init__(self, identity, width, controls):
        self.identity = identity
        self.width = width
        controls = np.asarray(controls, dtype=float)
        points, tangents = [], []
        for index, (start, end) in enumerate(zip(controls, controls[1:])):
            before = controls[max(index - 1, 0)]
            after = controls[min(index + 2, len(controls) - 1)]
            first, last = (end - before) / 2, (after - start) / 2
            count = max(2, math.ceil(np.linalg.norm(end - start) / 3))
            for t in np.linspace(0, 1, count, endpoint=False):
                points.append(
                    (2 * t**3 - 3 * t**2 + 1) * start
                    + (t**3 - 2 * t**2 + t) * first
                    + (-2 * t**3 + 3 * t**2) * end
                    + (t**3 - t**2) * last
                )
                tangents.append(
                    (6 * t**2 - 6 * t) * start
                    + (3 * t**2 - 4 * t + 1) * first
                    + (-6 * t**2 + 6 * t) * end
                    + (3 * t**2 - 2 * t) * last
                )
        self.points = np.asarray([*points, controls[-1]])
        tangents = np.asarray([*tangents, (controls[-1] - controls[-2]) / 2])
        lengths = np.linalg.norm(tangents, axis=1)
        if np.any(lengths < 1e-6):
            raise ValueError(f"Road {identity} must not reverse at a control point")
        self.normals = tangents[:, ::-1] * np.asarray((-1, 1)) / lengths[:, None]
        self.segments = np.diff(self.points, axis=0)
        self.lengths_squared = np.sum(self.segments**2, axis=1)
        if np.any(self.lengths_squared < 1e-10):
            raise ValueError(f"Road {identity} has degenerate segments")
        self.minimum = self.points.min(axis=0) - self.width / 2
        self.maximum = self.points.max(axis=0) + self.width / 2

    @property
    def length(self):
        return float(np.sqrt(self.lengths_squared).sum())

    def contains(self, x, z):
        point = np.array((x, z))
        if np.any(point < self.minimum) or np.any(point > self.maximum):
            return False
        offset = point - self.points[:-1]
        fraction = np.clip(np.sum(offset * self.segments, axis=1) / self.lengths_squared, 0, 1)
        return bool(
            np.any(
                np.sum((offset - self.segments * fraction[:, None]) ** 2, axis=1)
                <= (self.width / 2) ** 2
            )
        )

    def vertices(self, height, color):
        strips = (
            self.points[:, None, :]
            + self.normals[:, None, :]
            * np.linspace(-self.width / 2, self.width / 2, 7)[None, :, None]
        )
        rows = [[(x, height((x, 0, z)) + 0.035, z, *color) for x, z in strip] for strip in strips]
        for a, b in zip(rows, rows[1:]):
            for index in range(len(a) - 1):
                yield from (a[index], b[index], b[index + 1], a[index], b[index + 1], a[index + 1])
