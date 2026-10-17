"""CPU hierarchy/rest data and context-owned GPU resources, separate from physics."""

from dataclasses import dataclass

import numpy as np

from engine.quaternion import matrix as quaternion_matrix

GLTF_TO_VEHICLE = np.diag((-1.0, 1.0, -1.0, 1.0))  # proper 180-degree Y rotation, not reflection


def checked_matrix(value):
    """Copy and validate a finite affine transform.

    Args:
        value: Input value to validate or convert.

    Returns:
        Independent float64 4x4 array.
    """
    value = np.array(value, dtype=np.float64, copy=True)
    if (
        value.shape != (4, 4)
        or not np.isfinite(value).all()
        or not np.allclose(value[3], (0, 0, 0, 1))
    ):
        raise ValueError("Transform must be a finite affine 4x4 matrix.")
    return value


def trs(translation=(0, 0, 0), rotation=(0, 0, 0, 1), scale=(1, 1, 1)):
    """Compose translation, quaternion rotation, and scale for column vectors.

    Args:
        translation: Local translation vector in meters.
        rotation: Quaternion rotation in glTF [x, y, z, w] order.
        scale: Dimensionless local scale or uniform model-size multiplier.

    Returns:
        A float64 affine 4x4 matrix.
    """
    t = np.asarray(translation, dtype=float)
    s = np.asarray(scale, dtype=float)
    if t.shape != (3,) or s.shape != (3,) or not np.isfinite(t).all() or not np.isfinite(s).all():
        raise ValueError("Translation/scale must have three finite components.")
    result = quaternion_matrix(rotation)
    result[:3, :3] = result[:3, :3] @ np.diag(s)
    result[:3, 3] = t
    return result


@dataclass(frozen=True)
class Material:
    """Immutable glTF material factors used by the untextured renderer."""

    name: str = "Default"
    base_color: tuple = (1.0, 1.0, 1.0, 1.0)
    metallic: float = 1.0
    roughness: float = 1.0
    double_sided: bool = False


@dataclass(frozen=True)
class MeshPrimitive:
    """Immutable CPU mesh attributes, indices, and material reference."""

    positions: np.ndarray
    normals: np.ndarray | None
    texcoords: np.ndarray | None
    indices: np.ndarray | None
    material: Material
    name: str


class ModelNode:
    """Retain imported rest transforms and independent runtime pose/animation layers."""

    def __init__(self, name, base_matrix=None, meshes=(), base_trs=None):
        """Retain imported rest data and initialize hierarchy and runtime pose layers.

        Args:
            name: Model node or state name.
            base_matrix: Imported affine 4x4 rest matrix; None uses identity.
            meshes: Primitive indices attached to this node.
            base_trs: Optional imported translation/rotation/scale components for pose
                interpolation.
        """
        self.name = name
        self.parent = None
        self.children = []
        self.meshes = tuple(meshes)
        # Preserve imported TRS components for later pose interpolation; matrix
        # nodes keep their exact matrix and expose no invented decomposition.
        self.base_trs = (
            None if base_trs is None else tuple(tuple(float(x) for x in v) for v in base_trs)
        )
        self._base = checked_matrix(np.eye(4) if base_matrix is None else base_matrix)
        self._base.setflags(write=False)
        self._override = None
        self._override_trs = None
        self._animation_pose = None
        self._delta = np.eye(4)
        self.world_matrix = np.eye(4)

    @property
    def base_matrix(self):
        """Copy the immutable imported local transform.

        Returns:
            Independent affine 4x4 matrix.
        """
        return self._base.copy()

    @property
    def local_matrix(self):
        """Evaluate the configuration transform followed by the animation delta.

        Returns:
            Effective local affine 4x4 matrix.
        """
        pose = self._animation_pose
        if pose is None:
            pose = self._base if self._override is None else self._override
        return pose @ self._delta

    def set_animation_pose(self, matrix=None):
        """Set an absolute local visual TRS matrix; None reveals the configuration pose.

        This independent layer never modifies imported rest or configuration overrides.
        Native players clear/rebuild it each frame before adding any explicit visual delta.
        """
        self._animation_pose = None if matrix is None else checked_matrix(matrix)

    def set_pose(self, translation=(0, 0, 0), rotation=(0, 0, 0, 1), scale=(1, 1, 1)):
        """Absolute runtime TRS override, leaving the imported matrix intact.

        Args:
            translation: Local translation vector in meters.
            rotation: Quaternion rotation in glTF [x, y, z, w] order.
            scale: Dimensionless local scale or uniform model-size multiplier.
        """
        self._override = trs(translation, rotation, scale)
        self._override_trs = tuple(
            tuple(float(x) for x in value) for value in (translation, rotation, scale)
        )
        self._delta = np.eye(4)

    def set_delta(self, translation=(0, 0, 0), rotation=(0, 0, 0, 1), scale=(1, 1, 1)):
        """Rest-relative local-space articulation: effective = base * delta.

        Args:
            translation: Local translation vector in meters.
            rotation: Quaternion rotation in glTF [x, y, z, w] order.
            scale: Dimensionless local scale or uniform model-size multiplier.
        """
        self._override = None
        self._override_trs = None
        self._delta = trs(translation, rotation, scale)

    @property
    def configuration_trs(self):
        """Current absolute configuration TRS, excluding visual animation; None for matrix rest."""
        return self.base_trs if self._override is None else self._override_trs

    def set_animation_delta(self, matrix=None):
        """Local visual layer after the current configuration pose; no rest writes.

        Args:
            matrix: Affine 4x4 transform in mathematical column-vector convention.
        """
        self._delta = np.eye(4) if matrix is None else checked_matrix(matrix)

    def compose_animation_delta(self, matrix):
        """Append a local visual layer to the freshly evaluated animation pose.

        Args:
            matrix: Affine 4x4 transform in mathematical column-vector convention.
        """
        self._delta = self._delta @ checked_matrix(matrix)

    @property
    def configuration_matrix(self):
        """Copy the rest transform or active absolute pose override.

        Returns:
            Affine 4x4 matrix excluding the animation layer.
        """
        return (self._base if self._override is None else self._override).copy()

    def reset_pose(self):
        self._override = None
        self._override_trs = None
        self._animation_pose = None
        self._delta = np.eye(4)


class Model:
    """Own CPU scene nodes, roots, geometry, and explicit root coordinate conversion."""

    def __init__(
        self, nodes, roots, primitives, scale=1.0, convert_coordinates=True, animations=None
    ):
        """Retain CPU scene data and construct the root coordinate conversion.

        Args:
            nodes: CPU model nodes in the selected scene.
            roots: Root nodes of the selected scene.
            primitives: CPU mesh primitive collection indexed by node mesh references.
            scale: Dimensionless local scale or uniform model-size multiplier.
            convert_coordinates: Whether to rotate glTF coordinates into vehicle coordinates.
            animations: Optional name-to-native-clip mapping; absent for static models.
        """
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Model scale must be positive and finite.")
        self.nodes = list(nodes)
        self.roots = list(roots)
        self.primitives = tuple(primitives)
        self.animations = {} if animations is None else dict(animations)
        self.root_conversion = GLTF_TO_VEHICLE.copy() if convert_coordinates else np.eye(4)
        self.root_conversion[:3, :3] *= scale

    def find_node(self, name):
        """Resolve a unique named node, rejecting missing or ambiguous names.

        Args:
            name: Model node or state name.

        Returns:
            The matching ModelNode.
        """
        matches = [n for n in self.nodes if n.name == name]
        if len(matches) == 1:
            return matches[0]
        if matches:
            raise ValueError(f"Model node {name!r} is ambiguous ({len(matches)} matches).")
        raise KeyError(
            f"Model node {name!r} not found; available: " + ", ".join(n.name for n in self.nodes)
        )

    def reset_pose(self):
        for node in self.nodes:
            node.reset_pose()

    def world_matrices(self, world=None):
        """Traverse the hierarchy, updating cached world matrices and draw transforms.

        Args:
            world: Optional affine world transform; None uses identity.

        Returns:
            List of (primitive index, copied world matrix) pairs.
        """
        parent = checked_matrix(np.eye(4) if world is None else world) @ self.root_conversion
        draws = []

        def visit(node, parent):
            """Traverse descendants, composing parent transforms into cached world matrices.

            Args:
                node: ModelNode being traversed.
                parent: Parent affine 4x4 world transform.
            """
            node.world_matrix = parent @ node.local_matrix
            if not np.isfinite(node.world_matrix).all():
                raise ValueError(f"Nonfinite world transform for {node.name!r}.")
            for mesh in node.meshes:
                draws.append((mesh, node.world_matrix.copy()))
            for child in node.children:
                visit(child, node.world_matrix)

        for root in self.roots:
            visit(root, parent)
        return draws

    def bounds(self, world=None):
        """Transform scene vertices to compute axis-aligned bounds.

        Args:
            world: Optional affine world transform; None uses identity.

        Returns:
            Minimum and maximum XYZ vectors in transformed model units.
        """
        points = []
        for index, matrix in self.world_matrices(world):
            p = self.primitives[index].positions
            points.append((matrix @ np.column_stack((p, np.ones(len(p)))).T).T[:, :3])
        if not points:
            raise ValueError("Selected scene contains no geometry.")
        points = np.concatenate(points)
        if not np.isfinite(points).all():
            raise ValueError("Model bounds contain nonfinite vertices.")
        return points.min(axis=0), points.max(axis=0)

    def hierarchy(self):
        """Describe node nesting and mesh names without GPU access.

        Returns:
            Indented multiline hierarchy string.
        """
        lines = []

        def visit(node, depth):
            """Append node and mesh labels at the current hierarchy indentation.

            Args:
                node: ModelNode being traversed.
                depth: Nonnegative hierarchy nesting depth.
            """
            mesh_names = ", ".join(self.primitives[i].name for i in node.meshes)
            lines.append("  " * depth + node.name + (f" [{mesh_names}]" if mesh_names else ""))
            for child in node.children:
                visit(child, depth + 1)

        for root in self.roots:
            visit(root, 0)
        return "\n".join(lines)

    def print_hierarchy(self):
        """Print the CPU scene hierarchy to standard output."""
        print(self.hierarchy())


class ModelResources:
    """Unique primitive uploads reused across nodes/models; renderer owns close().

    Construct/draw/close only on the render thread with a current GL context.
    No GL work happens while loading Model CPU data.
    """

    def __init__(self, model):
        """Upload each CPU primitive once, releasing partial uploads on failure.

        Args:
            model: CPU model hierarchy and mesh data.
        """
        from engine.mesh import Mesh

        self.meshes = []
        self._closed = False
        try:
            for primitive in model.primitives:
                colors = np.tile(primitive.material.base_color[:3], (len(primitive.positions), 1))
                self.meshes.append(
                    Mesh(np.column_stack((primitive.positions, colors)), primitive.indices)
                )
        except Exception:
            self.close()
            raise

    def draw(self, model, shader, world=None):
        if self._closed:
            raise RuntimeError("Model GPU resources have been closed.")
        for index, matrix in model.world_matrices(world):
            shader.set_matrix("model", matrix)
            self.meshes[index].draw()

    def close(self):
        """Release owned resources."""
        if self._closed:
            return
        for mesh in self.meshes:
            mesh.close()
        self.meshes.clear()
        self._closed = True
