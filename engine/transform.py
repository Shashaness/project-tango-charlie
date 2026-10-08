"""Small rigid transform: +X right, +Y up, -Z forward; no world-up correction."""

import numpy as np


class Transform:
    def __init__(self, position=(0, 0, 3)):
        self.position = np.array(position, dtype=np.float64)
        self.forward = np.array((0, 0, -1), dtype=np.float64)
        self.right = np.array((1, 0, 0), dtype=np.float64)
        self.up = np.array((0, 1, 0), dtype=np.float64)

    @property
    def orientation(self):
        """Local-to-world rotation matrix (columns: right, up, backward)."""
        return np.column_stack((self.right, self.up, -self.forward))

    def model_matrix(self):
        model = np.eye(4, dtype=np.float32)
        model[:3, :3] = self.orientation
        model[:3, 3] = self.position
        return model

    def rotate(self, pitch=0.0, yaw=0.0, roll=0.0):
        """Apply local angular increments in radians.

        Positive pitch looks up, yaw looks left, roll banks right.
        Combined input is one axis-angle rotation, avoiding Euler angle limits.
        """
        angular = pitch * self.right + yaw * self.up + roll * self.forward
        angle = np.linalg.norm(angular)
        if angle == 0:
            return
        axis = angular / angle
        c, s = np.cos(angle), np.sin(angle)

        def rotated(vector):
            # Rodrigues' formula, about the transform's current local axes.
            return vector * c + np.cross(axis, vector) * s + axis * np.dot(axis, vector) * (1 - c)

        self.forward = rotated(self.forward)
        self.right = rotated(self.right)
        # Remove floating-point drift using only the rotated local basis.
        self.forward /= np.linalg.norm(self.forward)
        self.right -= self.forward * np.dot(self.right, self.forward)
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.forward)

