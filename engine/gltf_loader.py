"""Import CPU geometry, materials, rigid-node hierarchy, and LINEAR/STEP glTF animations."""

import json
import struct
from pathlib import Path

import numpy as np

from engine.gltf_animation import AnimationChannel, AnimationClip, AnimationSampler
from engine.model import Material, MeshPrimitive, Model, ModelNode, checked_matrix, trs
from engine.quaternion import normalize


class AssetError(ValueError):
    """Report invalid files or features outside the supported CPU-only glTF subset."""

    pass


def integer(value, label, minimum=0):
    """Validate an integer field without accepting boolean values.

    Args:
        value: Input value to validate or convert.
        label: Field name included in validation errors.
        minimum: Inclusive integer lower bound.

    Returns:
        Validated integer.
    """
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise AssetError(f"{label} must be an integer >= {minimum}.")
    return value


def item(items, index, label):
    """Resolve a checked nonnegative index into a glTF collection.

    Args:
        items: Collection to index.
        index: Zero-based index into the associated collection.
        label: Field name included in validation errors.

    Returns:
        Referenced collection item.
    """
    index = integer(index, label)
    if index >= len(items):
        raise AssetError(f"{label} index {index} is out of range.")
    return items[index]


def read_glb(path):
    """Read and validate GLB headers, chunk layout, and embedded JSON.

    Args:
        path: Filesystem path to the input or output asset.

    Returns:
        Pair of decoded JSON document and embedded binary bytes.
    """
    data = Path(path).read_bytes()
    if len(data) < 20:
        raise AssetError("Truncated GLB header.")
    magic, version, length = struct.unpack_from("<4sII", data)
    if magic != b"glTF" or version != 2 or length != len(data):
        raise AssetError("Invalid GLB magic/version/declared length.")
    chunks = []
    offset = 12
    while offset < len(data):
        if offset + 8 > len(data):
            raise AssetError("Truncated GLB chunk header.")
        size, kind = struct.unpack_from("<II", data, offset)
        offset += 8
        if size % 4 or offset + size > len(data):
            raise AssetError("Invalid GLB chunk length/alignment.")
        chunks.append((kind, data[offset : offset + size]))
        offset += size
    if not chunks or chunks[0][0] != 0x4E4F534A:
        raise AssetError("GLB must start with JSON chunk.")
    if len(chunks) > 2 or (len(chunks) == 2 and chunks[1][0] != 0x004E4942):
        raise AssetError("Unsupported GLB chunk layout.")
    document = json.loads(chunks[0][1].decode("utf8"))
    return document, chunks[1][1] if len(chunks) == 2 else b""


class Accessors:
    """Decode bounded, aligned attribute/index arrays from one embedded GLB buffer."""

    def __init__(self, document, binary):
        """Validate the single embedded buffer and retain its document and bytes.

        Args:
            document: Decoded glTF JSON object.
            binary: Embedded GLB binary buffer bytes.
        """
        self.document = document
        self.binary = binary
        buffers = document.get("buffers", [])
        if len(buffers) != 1 or "uri" in buffers[0]:
            raise AssetError(
                "Only one embedded GLB buffer is supported; external/data URIs are unsupported."
            )
        self.length = integer(buffers[0]["byteLength"], "buffer.byteLength", 1)
        if self.length > len(binary) or len(binary) - self.length > 3:
            raise AssetError("Embedded buffer length does not match BIN chunk.")

    def decode(self, index, expected, components):
        """Validate accessor layout and copy packed or strided binary attributes.

        Args:
            index: Zero-based index into the associated collection.
            expected: Expected glTF accessor type.
            components: Allowed glTF component-type codes.

        Returns:
            Finite NumPy array; scalar accessors return a one-dimensional array.
        """
        a = item(self.document.get("accessors", []), index, "accessor")
        if "sparse" in a:
            raise AssetError(f"accessor {index}: sparse accessors unsupported.")
        if a.get("type") != expected:
            raise AssetError(f"accessor {index}: expected {expected}.")
        component = a.get("componentType")
        types = {
            5121: np.dtype("u1"),
            5123: np.dtype("<u2"),
            5125: np.dtype("<u4"),
            5126: np.dtype("<f4"),
        }
        if component not in components:
            raise AssetError(f"accessor {index}: unsupported componentType {component}.")
        dtype = types[component]
        width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[expected]
        count = integer(a.get("count"), f"accessor {index}.count", 1)
        view = item(self.document.get("bufferViews", []), a.get("bufferView"), "bufferView")
        if view.get("buffer") != 0:
            raise AssetError("bufferView must reference embedded buffer 0.")
        view_offset = integer(view.get("byteOffset", 0), "bufferView.byteOffset")
        view_length = integer(view.get("byteLength"), "bufferView.byteLength", 1)
        offset = integer(a.get("byteOffset", 0), "accessor.byteOffset")
        packed = dtype.itemsize * width
        stride = integer(view.get("byteStride", packed), "bufferView.byteStride", packed)
        if "byteStride" in view and (expected == "SCALAR" or stride > 252 or stride % 4):
            raise AssetError("Invalid/unsupported accessor byteStride.")
        if offset % dtype.itemsize or (view_offset + offset) % dtype.itemsize:
            raise AssetError("Accessor component alignment is invalid.")
        if expected != "SCALAR" and ((view_offset + offset) % 4 or (count > 1 and stride % 4)):
            raise AssetError("Vertex attribute alignment must be four bytes.")
        if (
            stride % dtype.itemsize
            or offset + (count - 1) * stride + packed > view_length
            or view_offset + view_length > self.length
        ):
            raise AssetError(
                f"accessor {index}: data exceeds bufferView/buffer bounds or has invalid stride."
            )
        result = np.ndarray(
            (count, width),
            dtype=dtype,
            buffer=self.binary,
            offset=view_offset + offset,
            strides=(stride, dtype.itemsize),
        ).copy()
        if a.get("normalized", False):
            if expected != "VEC2" or component not in (5121, 5123):
                raise AssetError("Only unsigned normalized TEXCOORD_0 is supported.")
            result = result.astype(np.float32) / np.iinfo(dtype).max
        if not np.isfinite(result).all():
            raise AssetError(f"accessor {index}: nonfinite attribute values.")
        return result[:, 0] if expected == "SCALAR" else result


def reject_features(document):
    """Reject unsupported glTF features, including nested extensions.

    Args:
        document: Decoded glTF JSON object.
    """
    for key in (
        "skins",
        "cameras",
        "textures",
        "images",
        "samplers",
        "extensionsUsed",
        "extensionsRequired",
    ):
        if document.get(key):
            raise AssetError(f"Unsupported glTF feature: {key}.")

    def visit(value, path):
        """Walk nested document containers and reject any extension dictionaries.

        Args:
            value: Nested JSON object, list, or scalar being visited.
            path: Logical JSON field path included in validation errors.
        """
        if isinstance(value, dict):
            if value.get("extensions"):
                raise AssetError(f"Unsupported extensions at {path}.")
            for key, child in value.items():
                visit(child, path + "." + key)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                visit(child, f"{path}[{index}]")

    visit(document, "glTF")
    for a in document.get("accessors", []):
        if "sparse" in a:
            raise AssetError("Unsupported sparse accessor.")


def material(value):
    """Validate supported opaque material factors.

    Args:
        value: Input value to validate or convert.

    Returns:
        Immutable Material with normalized color and PBR factors.
    """
    if value.get("alphaMode", "OPAQUE") != "OPAQUE":
        raise AssetError("Only OPAQUE materials are supported; MASK/BLEND unsupported.")
    pbr = value.get("pbrMetallicRoughness", {})
    for key in ("normalTexture", "occlusionTexture", "emissiveTexture"):
        if key in value:
            raise AssetError(f"Unsupported material feature: {key}.")
    if "baseColorTexture" in pbr or "metallicRoughnessTexture" in pbr:
        raise AssetError("Texture materials are unsupported.")
    if any(value.get("emissiveFactor", (0, 0, 0))):
        raise AssetError("Emissive materials are unsupported.")
    color = np.asarray(pbr.get("baseColorFactor", (1, 1, 1, 1)), dtype=float)
    metal = float(pbr.get("metallicFactor", 1))
    rough = float(pbr.get("roughnessFactor", 1))
    if (
        color.shape != (4,)
        or not np.isfinite(color).all()
        or np.any((color < 0) | (color > 1))
        or not 0 <= metal <= 1
        or not 0 <= rough <= 1
    ):
        raise AssetError("Material factors must be finite and in [0,1].")
    return Material(
        value.get("name", "Material"),
        tuple(color),
        metal,
        rough,
        bool(value.get("doubleSided", False)),
    )


def load_animations(document, accessors, nodes):
    """Decode named rigid-node TRS clips without changing geometry or rest poses.

    Args:
        document: glTF document containing animations and accessor metadata.
        accessors: Validated embedded-buffer decoder.
        nodes: All nodes in original document-index order, including inactive scenes.

    Returns:
        Dictionary of clip names to CPU AnimationClip objects.

    Raises:
        AssetError: Invalid target, sampler, keyframes, name collision, or unsupported feature.
    """
    clips = {}
    try:
        animations = document.get("animations", [])
        if not isinstance(animations, list):
            raise AssetError("animations must be an array")
        for index, animation in enumerate(animations):
            name = animation.get("name", f"Animation_{index}")
            if not isinstance(name, str) or not name or name in clips:
                raise AssetError("Animation names must be nonempty and unique")
            raw_samplers = animation.get("samplers", [])
            raw_channels = animation.get("channels", [])
            if not isinstance(raw_samplers, list) or not raw_samplers:
                raise AssetError(f"animation {name}: samplers must be a nonempty array")
            if not isinstance(raw_channels, list) or not raw_channels:
                raise AssetError(f"animation {name}: channels must be a nonempty array")
            samplers = []
            for sampler in raw_samplers:
                input_index = sampler.get("input")
                output_index = sampler.get("output")
                times = accessors.decode(input_index, "SCALAR", (5126,))
                output = item(document.get("accessors", []), output_index, "animation output")
                kind = output.get("type")
                if kind not in ("VEC3", "VEC4"):
                    raise AssetError(f"animation {name}: output must be VEC3 or VEC4")
                values = accessors.decode(output_index, kind, (5126,))
                samplers.append(
                    AnimationSampler(times, values, sampler.get("interpolation", "LINEAR"))
                )
            channels = []
            for channel in raw_channels:
                target = channel.get("target", {})
                node_index = integer(target.get("node"), "animation target node")
                node = item(nodes, node_index, "animation target node")
                if node.base_trs is None:
                    raise AssetError(
                        f"animation {name}: matrix-authored targets cannot be animated"
                    )
                sampler = item(samplers, channel.get("sampler"), "animation sampler")
                channels.append(AnimationChannel(node_index, node, target.get("path"), sampler))
            clips[name] = AnimationClip(name, tuple(channels))
    except (ValueError, KeyError, TypeError, IndexError, AttributeError) as error:
        raise AssetError(f"Invalid glTF animation: {error}") from error
    return clips


def build_model(document, binary, scale=1.0, convert_coordinates=True):
    """Validate geometry and hierarchy before constructing CPU model data.

    Args:
        document: Decoded glTF JSON object.
        binary: Embedded GLB binary buffer bytes.
        scale: Dimensionless local scale or uniform model-size multiplier.
        convert_coordinates: Whether to rotate glTF coordinates into vehicle coordinates.

    Returns:
        Model containing the selected scene, with validated finite bounds.
    """
    if not isinstance(document, dict):
        raise AssetError("GLB JSON root must be an object.")
    if document.get("asset", {}).get("version") != "2.0":
        raise AssetError("Only glTF asset.version 2.0 supported.")
    if document.get("asset", {}).get("minVersion", "2.0") != "2.0":
        raise AssetError("Unsupported glTF minimum version.")
    reject_features(document)
    accessors = Accessors(document, binary)
    materials = [material(m) for m in document.get("materials", [])]
    primitives = []
    mesh_map = []
    for mesh_index, mesh in enumerate(document.get("meshes", [])):
        if mesh.get("weights"):
            raise AssetError("Morph weights unsupported.")
        refs = []
        for number, p in enumerate(mesh.get("primitives", [])):
            if p.get("mode", 4) != 4:
                raise AssetError("Only TRIANGLES mesh primitives are supported.")
            if p.get("targets"):
                raise AssetError("Morph targets unsupported.")
            attrs = p.get("attributes", {})
            if "POSITION" not in attrs:
                raise AssetError(f"mesh {mesh_index} primitive {number}: required POSITION absent.")
            unknown = set(attrs) - {"POSITION", "NORMAL", "TEXCOORD_0"}
            if unknown:
                raise AssetError(f"Unsupported vertex attributes: {sorted(unknown)}.")
            positions = accessors.decode(attrs["POSITION"], "VEC3", (5126,))
            normals = (
                accessors.decode(attrs["NORMAL"], "VEC3", (5126,)) if "NORMAL" in attrs else None
            )
            uv = (
                accessors.decode(attrs["TEXCOORD_0"], "VEC2", (5126, 5121, 5123))
                if "TEXCOORD_0" in attrs
                else None
            )
            if uv is not None and uv.dtype.kind != "f":
                raise AssetError("Integer UV accessors must be normalized.")
            if any(x is not None and len(x) != len(positions) for x in (normals, uv)):
                raise AssetError("Attribute counts do not match POSITION.")
            indices = (
                accessors.decode(p["indices"], "SCALAR", (5121, 5123, 5125))
                if "indices" in p
                else None
            )
            if indices is not None and np.any(indices >= len(positions)):
                raise AssetError("Mesh index exceeds vertex count.")
            if (len(positions) if indices is None else len(indices)) % 3:
                raise AssetError("TRIANGLES vertex/index count must be a multiple of three.")
            mat = item(materials, p["material"], "material") if "material" in p else Material()
            for data in (positions, normals, uv, indices):
                if data is not None:
                    data.setflags(write=False)
            refs.append(len(primitives))
            primitives.append(
                MeshPrimitive(
                    positions,
                    normals,
                    uv,
                    indices,
                    mat,
                    f"{mesh.get('name', f'Mesh{mesh_index}')}/{number}",
                )
            )
        if not refs:
            raise AssetError(f"mesh {mesh_index} has no primitives.")
        mesh_map.append(refs)
    nodes = []
    source_nodes = document.get("nodes", [])
    for index, n in enumerate(source_nodes):
        if "skin" in n or "weights" in n or "camera" in n:
            raise AssetError(f"node {index}: skin/weights/camera unsupported.")
        base_trs = None
        if not isinstance(n.get("name", f"Node{index}"), str):
            raise AssetError(f"node {index}: name must be a string.")
        if "matrix" in n:
            if any(key in n for key in ("translation", "rotation", "scale")):
                raise AssetError("Node must use matrix OR TRS, not both.")
            if len(n["matrix"]) != 16:
                raise AssetError("Node matrix must have 16 column-major values.")
            base = checked_matrix(np.asarray(n["matrix"]).reshape((4, 4), order="F"))
        else:
            base_trs = (
                n.get("translation", (0, 0, 0)),
                normalize(n.get("rotation", (0, 0, 0, 1))),
                n.get("scale", (1, 1, 1)),
            )
            base = trs(*base_trs)
        refs = item(mesh_map, n["mesh"], "mesh") if "mesh" in n else ()
        nodes.append(ModelNode(n.get("name", f"Node{index}"), base, refs, base_trs))
    for index, n in enumerate(source_nodes):
        for child_index in n.get("children", []):
            child = item(nodes, child_index, "child node")
            if child.parent is not None:
                raise AssetError("Node hierarchy has duplicate/multiple parents.")
            child.parent = nodes[index]
            nodes[index].children.append(child)
    visiting = set()
    finished = set()

    def validate(node):
        """Check the node graph for cycles using active and completed traversal sets.

        Args:
            node: Model node being traversed or optional named articulation target.
        """
        if node in visiting:
            raise AssetError("Node hierarchy contains a cycle.")
        if node in finished:
            return
        visiting.add(node)
        for child in node.children:
            validate(child)
        visiting.remove(node)
        finished.add(node)

    for node in nodes:
        validate(node)
    scene = item(document.get("scenes", []), document.get("scene", 0), "scene")
    roots = [item(nodes, i, "scene root") for i in scene.get("nodes", [])]
    if len(set(roots)) != len(roots) or any(n.parent is not None for n in roots):
        raise AssetError("Invalid duplicate/non-root scene nodes.")
    active = []

    def gather(node):
        """Append reachable nodes to the selected scene in hierarchy order.

        Args:
            node: Model node being traversed or optional named articulation target.
        """
        active.append(node)
        for child in node.children:
            gather(child)

    for root in roots:
        gather(root)
    model = Model(active, roots, primitives, scale, convert_coordinates)
    model.animations = load_animations(document, accessors, nodes)
    model.bounds()  # require visible geometry and finite transforms before uploading
    return model


def load_glb(path, scale=1.0, convert_coordinates=True):
    """Load the supported GLB subset and attach the path to asset errors.

    Args:
        path: Filesystem path to the input or output asset.
        scale: Dimensionless local scale or uniform model-size multiplier.
        convert_coordinates: Whether to rotate glTF coordinates into vehicle coordinates.

    Returns:
        Validated CPU Model; no OpenGL resources are allocated.
    """
    path = Path(path)
    try:
        document, binary = read_glb(path)
        return build_model(document, binary, scale, convert_coordinates)
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        struct.error,
        OverflowError,
        RecursionError,
        AttributeError,
    ) as error:
        raise AssetError(f"{path}: {error}") from error
