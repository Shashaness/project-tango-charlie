"""Bind the Jeep's rigid GLB nodes to suspension, steering, and tire motion."""

import math
from copy import deepcopy
from functools import lru_cache

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_loader import AssetError, build_model, decode_base_colors, read_glb
from game.ground_vehicle import DriveParameters

WHEEL_IDS = ("FL", "FR", "RL", "RR")
ANCHOR_ROLES = {
    "DriverSeat": "driver_seat",
    "Entry.L": "entry",
    "Entry.R": "entry",
    "Exit.L": "exit",
    "Exit.R": "exit",
}


def load_jeep(path=None):
    path = path or asset_path("models/vehicles/jeep.glb")
    try:
        document, binary = read_glb(path)
        tagged = any(
            isinstance(extras, dict) and "ptc_vehicle_version" in extras
            for extras in (node.get("extras") for node in document["nodes"])
        )
        if not tagged or not document.get("textures"):
            return build_model(document, binary, convert_coordinates=False)
        rigid = deepcopy(document)
        colors = decode_base_colors(document, binary, rigid)
        model = build_model(rigid, binary, convert_coordinates=False)
        model.base_color_textures = tuple(
            colors[primitive["material"]] if "material" in primitive else None
            for mesh in document["meshes"]
            for primitive in mesh["primitives"]
        )
        if any(
            texture is not None and primitive.texcoords is None
            for texture, primitive in zip(model.base_color_textures, model.primitives, strict=True)
        ):
            raise AssetError("Textured vehicles require TEXCOORD_0.")
        return model
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError) as error:
        raise AssetError(f"{path}: {error}") from error


def contract_components(model):
    """Resolve v1 Jeep components by semantic ID, or defer to legacy names."""
    roots = [node for node in model.nodes if "ptc_vehicle_version" in node.extras]
    if not roots:
        return None
    root = roots[0]
    if (
        roots != model.roots
        or len(roots) != 1
        or root.meshes
        or root.extras["ptc_vehicle_version"] != 1
        or root.extras.get("ptc_controller") != "jeep"
        or root.extras.get("ptc_role") != "root"
        or not np.allclose(root.base_matrix, np.eye(4), atol=1e-5)
    ):
        raise ValueError("Expected a single identity PTC v1 Jeep mechanical root")
    components = {}
    for node in model.nodes:
        identity = node.extras.get("ptc_id")
        if not isinstance(identity, str) or not identity.strip() or identity in components:
            raise ValueError("Missing or duplicate vehicle semantic ID")
        components[identity] = node

    def component(identity, role, parent):
        node = components.get(identity)
        if not node or node.parent is not parent or node.extras.get("ptc_role") != role:
            raise ValueError(f"Expected {role} component {identity} under {parent.name}")
        if node.meshes:
            raise ValueError(f"Mechanical component {identity} must be an empty node")
        return node

    anchors = {name: component(name, role, root) for name, role in ANCHOR_ROLES.items()}
    hubs = [component(name, "wheel_hub", root) for name in WHEEL_IDS]
    wheels = [
        component(f"{name}.spin", "wheel_spin", hub)
        for name, hub in zip(WHEEL_IDS, hubs, strict=True)
    ]
    if sum(n.extras.get("ptc_role") == "wheel_hub" for n in model.nodes) != len(WHEEL_IDS):
        raise ValueError("The Jeep controller requires four wheel hubs")
    params = DriveParameters()
    for index, (hub, spin) in enumerate(zip(hubs, wheels, strict=True)):
        expected = (
            params.track / 2 * (1 if index % 2 == 0 else -1),
            params.wheelbase / 2 * (1 if index < 2 else -1),
        )
        if (
            not np.allclose(hub.base_matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.allclose(hub.base_matrix[[0, 2], 3], expected, atol=1e-5)
            or not np.allclose(spin.base_matrix, np.eye(4), atol=1e-5)
            or bool(hub.extras.get("ptc_steers")) != (index < 2)
            or hub.extras.get("ptc_rotation_axis") != [1, 0, 0]
            or hub.extras.get("ptc_steering_axis") != [0, 0, 1]
            or hub.extras.get("ptc_suspension_axis") != [0, 0, 1]
        ):
            raise ValueError(f"{hub.name} differs from the Jeep controller's mechanical layout")
        if not any(n.extras.get("ptc_role") == "wheel_visual" and n.meshes for n in spin.children):
            raise ValueError(f"{spin.name} needs wheel geometry")
    return anchors, hubs, wheels


@lru_cache(maxsize=16)
def jeep_access_points(path=None):
    model = load_jeep(path)
    contract = contract_components(model)
    anchors = contract[0] if contract else {name: model.find_node(name) for name in ANCHOR_ROLES}
    return {name: tuple(node.world_matrix[:3, 3]) for name, node in anchors.items()}


class JeepVisual:
    def __init__(self, path=None):
        self.model = load_jeep(path)
        contract = contract_components(self.model)
        if contract:
            _, self.hubs, self.wheels = contract
        else:
            self.hubs = [self.model.find_node("Hub." + name) for name in WHEEL_IDS]
            self.wheels = [self.model.find_node("Wheel." + name) for name in WHEEL_IDS]

    def pose(self, vehicle):
        vertical = vehicle.model_matrix()[1, :3]
        for index, ((x, z), hub, wheel) in enumerate(
            zip(
                vehicle.axles,
                self.hubs,
                self.wheels,
                strict=True,
            )
        ):
            height = (vehicle.wheel_heights[index] - x * vertical[0] - z * vertical[2]) / vertical[
                1
            ]
            heading = vehicle.steering / 2 if index < 2 else 0
            hub.set_pose((x, height, z), (0, math.sin(heading), 0, math.cos(heading)))
            spin = vehicle.wheel_spin[index] / 2
            wheel.set_pose(rotation=(math.sin(spin), 0, 0, math.cos(spin)))
        return np.asarray(vehicle.model_matrix())
