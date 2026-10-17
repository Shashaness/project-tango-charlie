"""Small rigid transform: +X right, +Y up, -Z forward; no world-up correction."""

import numpy as np


class Transform:
    """Store a meter-space position and orthonormal right/up/forward basis."""

    def __init__(self, position=(0, 0, 3)):
        """Initialize a right-handed pose looking along world -Z.

        Args:
            position: XYZ position in meters.
        """
        self.position = np.array(position, dtype=np.float64)
        self.forward = np.array((0, 0, -1), dtype=np.float64)
        self.right = np.array((1, 0, 0), dtype=np.float64)
        self.up = np.array((0, 1, 0), dtype=np.float64)

    @property
    def orientation(self):
        """Local-to-world rotation matrix (columns: right, up, backward)."""
        return np.column_stack((self.right, self.up, -self.forward))

    def model_matrix(self):
        """Combine local-to-world orientation and translation.

        Returns:
            A float32 affine 4x4 matrix.
        """
        model = np.eye(4, dtype=np.float32)
        model[:3, :3] = self.orientation
        model[:3, 3] = self.position
        return model

    def rotate(self, pitch=0.0, yaw=0.0, roll=0.0):
        """Apply local angular increments in radians.

        Positive pitch looks up, yaw looks left, roll banks right.
        Combined input is one axis-angle rotation, avoiding Euler angle limits.

        Args:
            pitch: Local pitch increment in radians.
            yaw: Local yaw increment in radians.
            roll: Local roll increment in radians.
        """
        angular = pitch * self.right + yaw * self.up + roll * self.forward
        angle = np.linalg.norm(angular)
        if angle == 0:
            return
        axis = angular / angle
        c, s = np.cos(angle), np.sin(angle)

        def rotated(vector):
            # Rodrigues' formula, about the transform's current local axes.
            """Rotate a basis vector with the current Rodrigues axis-angle increment.

            Args:
                vector: VTOL thrust-vector fraction; None uses the vehicle value.

            Returns:
                Rotated three-component vector.
            """
            return vector * c + np.cross(axis, vector) * s + axis * np.dot(axis, vector) * (1 - c)

        self.forward = rotated(self.forward)
        self.right = rotated(self.right)
        # Remove floating-point drift using only the rotated local basis.
        self.forward /= np.linalg.norm(self.forward)
        self.right -= self.forward * np.dot(self.right, self.forward)
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.forward)
