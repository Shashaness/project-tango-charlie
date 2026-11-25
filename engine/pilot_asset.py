"""CPU skinning for a standalone one-skin humanoid GLB."""

import json
import math
from copy import deepcopy

import numpy as np

from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import (
    Accessors,
    AssetError,
    build_model,
    decode_base_colors,
    item,
    read_glb,
)

CLIPS = ("Idle", "Walk", "Run", "Turn")
SEMANTICS = (
    "Hips",
    "Spine",
    "Chest",
    "Neck",
    "Head",
    *(
        f"{bone}.{side}"
        for side in ("L", "R")
        for bone in (
            "Clavicle",
            "UpperArm",
            "Forearm",
            "Hand",
            "Thigh",
            "Shin",
            "Foot",
        )
    ),
)


class PilotAsset:
    """Sample the pilot rig and deform its small render mesh on the CPU."""

    def __init__(self, path):
        document, binary = read_glb(path)
        skins = document.get("skins", [])
        nodes = document.get("nodes", [])
        mesh_nodes = [index for index, node in enumerate(nodes) if "mesh" in node]
        if len(skins) != 1 or len(mesh_nodes) != 1 or nodes[mesh_nodes[0]].get("skin") != 0:
            raise AssetError("Pilot GLB needs one skinned mesh node and one skin.")
        mesh_index = nodes[mesh_nodes[0]]["mesh"]
        extras = nodes[mesh_nodes[0]].get("extras", {})
        if not isinstance(extras, dict):
            raise AssetError("Character mesh extras must be an object.")
        version = extras.get("ptc_character_version")
        if version is not None and (type(version) is not int or version != 1):
            raise AssetError("Unsupported PTC character contract version.")
        offset = extras.get("ptc_seat_offset", [0, 0, 0])
        if (
            not isinstance(offset, list)
            or len(offset) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in offset)
        ):
            raise AssetError("Character seat offset needs three finite coordinates.")
        self.seat_offset = np.asarray(offset, dtype=float)
        skin = skins[0]
        accessors = Accessors(document, binary)
        rigid = deepcopy(document)
        rigid.pop("skins")
        textures = decode_base_colors(document, binary, rigid)
        for index, node in enumerate(rigid["nodes"]):
            node.pop("skin", None)
            node["name"] = str(index)
        for mesh in rigid["meshes"]:
            for primitive in mesh["primitives"]:
                for attribute in ("JOINTS_0", "WEIGHTS_0"):
                    primitive["attributes"].pop(attribute, None)
        model = build_model(rigid, binary, convert_coordinates=False)
        mesh_node = model.find_node(str(mesh_nodes[0]))
        if not np.allclose(mesh_node.world_matrix, np.eye(4)):
            raise AssetError("Apply the skinned mesh node transform before export.")
        self.joints = [model.find_node(str(index)) for index in skin["joints"]]
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise AssetError("The character skin needs distinct joints.")
        if version is not None:
            try:
                mapping = json.loads(extras["ptc_semantics"])
                if (
                    set(mapping) != set(SEMANTICS)
                    or any(not isinstance(name, str) for name in mapping.values())
                    or len(set(mapping.values())) != len(mapping)
                ):
                    raise ValueError
            except (ValueError, KeyError, TypeError, AttributeError) as error:
                raise AssetError("Invalid PTC semantic mapping.") from error
        else:
            mapping = {role: role for role in SEMANTICS}
        named = {}
        for index, joint in zip(skin["joints"], self.joints, strict=True):
            named.setdefault(item(nodes, index, "skin joint").get("name"), []).append(joint)
        self.semantic_joints = {
            role: named[name][0] for role, name in mapping.items() if len(named.get(name, [])) == 1
        }
        if "Hips" not in self.semantic_joints or (
            version is not None and self.semantic_joints.keys() != mapping.keys()
        ):
            raise AssetError("Character semantic mapping must resolve to distinct skin joints.")
        index = skin.get("inverseBindMatrices")
        self.inverse = (
            accessors.decode(index, "MAT4", (5126,)).reshape(-1, 4, 4).transpose(0, 2, 1)
            if index is not None
            else np.broadcast_to(np.eye(4), (len(self.joints), 4, 4))
        )
        if len(self.inverse) != len(self.joints):
            raise AssetError("Each joint needs one inverse bind matrix.")
        if any(
            name not in model.animations or model.animations[name].duration <= 0
            for name in (*CLIPS, "Seated")
            if version is not None or name != "Seated"
        ):
            raise AssetError("Character GLB is missing a required positive-duration animation.")

        self.parts = []
        self.base_colors = []
        raw = item(document["meshes"], mesh_index, "skin mesh")["primitives"]
        for original, index in zip(raw, mesh_node.meshes, strict=True):
            primitive = model.primitives[index]
            attributes = original["attributes"]
            joints = accessors.decode(attributes["JOINTS_0"], "VEC4", (5121, 5123))
            weights = accessors.decode(attributes["WEIGHTS_0"], "VEC4", (5126, 5121, 5123))
            counts = len(primitive.positions)
            if (
                len(joints) != counts
                or len(weights) != counts
                or np.any(joints >= len(self.joints))
                or weights.dtype.kind != "f"
                or np.any(weights < 0)
                or not np.allclose(weights.sum(axis=1), 1, atol=1e-3)
            ):
                raise AssetError("Invalid pilot joint indices or skin weights.")
            positions = np.column_stack((primitive.positions, np.ones(counts)))
            color = np.broadcast_to(primitive.material.base_color[:3], (counts, 3))
            texture = textures[original["material"]] if "material" in original else None
            if texture is not None:
                if primitive.texcoords is None:
                    raise AssetError("Textured characters require UV coordinates.")
                color = np.column_stack((primitive.texcoords, np.zeros(counts)))
            self.base_colors.append((texture, primitive.material.base_color[:3]))
            weights = weights / weights.sum(axis=1, keepdims=True)
            self.parts.append((positions, joints, weights, color, primitive.indices))
        for node in model.nodes:
            source = nodes[int(node.name)]
            node.name = source.get("name", f"Node{node.name}")
        self.model = model
        # Convert the character's lateral axis into each anatomical bind basis.
        self.hinge_axes = {joint.name: joint.world_matrix[0, :3].copy() for joint in self.joints}
        self.player = AnimationPlayer(model)
        self._seated_palette, self.seat_hips = self._build_seated_pose()

    def reset_locomotion(self):
        self.model.reset_pose()
        self.model.world_matrices()
        self.player = AnimationPlayer(self.model)

    def update(self, clip, dt):
        if self.player.clip is None or self.player.clip.name != clip:
            self.player.play(clip, transition=0.12)
        self.player.update(max(0, dt))
        self.player.apply()

    def _build_seated_pose(self):
        """Bake seating once, then restore the independent locomotion pose."""
        self.model.reset_pose()
        if "Seated" in self.model.animations:
            player = AnimationPlayer(self.model)
            player.play("Seated", loop=False)
            player.apply()
            hips = self.semantic_joints["Hips"].world_matrix[:3, 3].copy()
            palette = self._palette()
            player.clear()
            self.model.world_matrices()
            return palette, hips
        for side in ("L", "R"):
            for bone, angle in (
                ("Thigh", -79),
                ("Shin", 101),
                ("UpperArm", -48),
                ("Forearm", -37),
            ):
                joint = self.semantic_joints.get(f"{bone}.{side}")
                if joint is None:
                    continue
                half = math.radians(angle) / 2
                axis = self.hinge_axes[joint.name]
                joint.set_delta(rotation=(*(axis * math.sin(half)), math.cos(half)))
        self.model.world_matrices()
        hips = self.semantic_joints["Hips"].world_matrix[:3, 3].copy()
        seated_palette = self._palette()
        self.model.reset_pose()
        self.model.world_matrices()
        return seated_palette, hips

    def _palette(self):
        return np.asarray([joint.world_matrix for joint in self.joints]) @ self.inverse

    def vertices(self, seated=False):
        """Skin one pose using the same mesh and weights for both representations."""
        palette = self._seated_palette if seated else self._palette()
        for positions, joints, weights, color, indices in self.parts:
            skinned = np.einsum("vjik,vk,vj->vi", palette[joints], positions, weights)
            yield np.column_stack((skinned[:, :3], color)), indices
