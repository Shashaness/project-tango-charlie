"""Installable Blender authoring tools for PTC vehicle contract v1."""

import json
import math
import struct
import tempfile
from contextlib import ExitStack, contextmanager
from pathlib import Path

import bpy
from bpy.app.handlers import persistent
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    PointerProperty,
    StringProperty,
)
from mathutils import Euler, Matrix, Quaternion, Vector

bl_info = {
    "name": "PTC Vehicle",
    "author": "Project Tango Charlie",
    "version": (1, 0, 1),
    "blender": (5, 0, 0),
    "location": "3D View > Sidebar > PTC Vehicle",
    "category": "Rigging",
}

VERSION = 1
ROLE = "ptc_role"
IDENTITY = "ptc_id"
PREVIEW = "ptc_vehicle_preview"
VISUAL_ROLES = {"body", "interior", "wheel_visual", "mechanical_visual"}
ANCHOR_ROLES = (
    "driver_seat",
    "passenger_seat",
    "entry",
    "exit",
    "camera",
    "operator",
    "attachment",
    "collision",
    "joint",
)
LEGACY_ANCHORS = {
    "DriverSeat": "driver_seat",
    "Entry.L": "entry",
    "Entry.R": "entry",
    "Exit.L": "exit",
    "Exit.R": "exit",
}
JEEP_WHEELS = ("FL", "FR", "RL", "RR")
UP = Vector((0, 0, 1))


def in_source(path):
    parts = Path(path).expanduser().resolve().parts
    return ("assets", "source") in zip(parts, parts[1:])


def require_derived_file():
    if bpy.data.filepath and in_source(bpy.data.filepath):
        raise ValueError("Save a derived Blender file outside assets/source before authoring")


def tag(obj, role, identity):
    obj[ROLE] = role
    obj[IDENTITY] = identity
    return obj


def empty(context, name, parent=None, position=(0, 0, 0), shape="PLAIN_AXES"):
    obj = bpy.data.objects.new(name, None)
    context.collection.objects.link(obj)
    obj.parent = parent
    obj.location = position
    obj.empty_display_type = shape
    obj.empty_display_size = 0.22
    return obj


def is_rigid(matrix):
    basis = matrix.to_3x3()
    gram = basis.transposed() @ basis
    return abs(basis.determinant() - 1) < 1e-5 and all(
        abs(gram[r][c] - (r == c)) < 1e-5 for r in range(3) for c in range(3)
    )


def ancestors(obj):
    seen = {obj}
    parent = obj.parent
    while parent is not None:
        if parent in seen:
            raise ValueError(f"{obj.name}: cyclic object hierarchy; clear the invalid parent")
        seen.add(parent)
        yield parent
        parent = parent.parent


def hierarchy_depth(obj):
    return sum(1 for _ in ancestors(obj))


def is_vehicle_root(obj):
    return (
        obj is not None
        and obj.type == "EMPTY"
        and not obj.parent
        and not obj.library
        and obj.get(ROLE) == "root"
        and obj.get("ptc_vehicle_version") == VERSION
        and all(abs(obj.matrix_world[r][c] - (r == c)) < 1e-5 for r in range(4) for c in range(4))
    )


def owning_root(obj):
    if obj is None:
        return None
    if is_vehicle_root(obj):
        return obj
    return next((parent for parent in ancestors(obj) if is_vehicle_root(parent)), None)


def root_in_scene(scene, obj):
    return is_vehicle_root(obj) and scene.objects.get(obj.name) == obj


def root_poll(settings, obj):
    return root_in_scene(settings.id_data, obj)


def assign_root(context, obj, identity):
    require_derived_file()
    if not obj or obj.type != "EMPTY" or obj.parent or obj.library:
        raise ValueError("Select a local, unparented Empty to use as the vehicle root")
    if obj.get(ROLE) not in (None, "root") or obj.get("ptc_vehicle_version", VERSION) != VERSION:
        raise ValueError("The selected Empty already has an incompatible component role or version")
    if not identity.strip():
        raise ValueError("Enter a vehicle ID")
    if any(abs(obj.matrix_world[r][c] - (r == c)) > 1e-5 for r in range(4) for c in range(4)):
        raise ValueError("The vehicle root must be at the origin with identity rotation and scale")
    if not obj.get("ptc_vehicle_version"):
        tag(obj, "root", identity.strip())
        obj["ptc_vehicle_version"] = VERSION
        obj["ptc_controller"] = "none"
        obj["ptc_dimensions"] = [1.0, 1.0, 1.0]
        obj["ptc_mass_kg"] = 0.0
        obj.id_properties_ui("ptc_dimensions").update(
            min=0.001, description="Reference width, length, height in Blender X, Y, Z"
        )
        obj.id_properties_ui("ptc_mass_kg").update(
            min=0, description="Zero leaves mass unspecified"
        )
    context.scene.ptc_vehicle.root = obj
    return obj


def reparent(obj, parent):
    world = obj.matrix_world.copy()
    obj.parent = parent
    obj.matrix_parent_inverse.identity()
    obj.matrix_world = world


def components(root):
    return (root, *(obj for obj in root.children_recursive if obj.get(ROLE) != "archived"))


def require_new_id(root, identity):
    identity = identity.strip()
    if not identity or any(obj.get(IDENTITY) == identity for obj in components(root)):
        raise ValueError("Use a nonempty semantic ID unique within the vehicle")
    return identity


def wheel_hub(context, root, identity, position=(0, 0, 0), hub=None):
    require_derived_file()
    identity = require_new_id(root, identity)
    if hub is None:
        hub = empty(context, f"Hub.{identity}", root, position, "SPHERE")
    tag(hub, "wheel_hub", identity)
    settings = {
        "ptc_rotation_axis": [1.0, 0.0, 0.0],
        "ptc_steering_axis": [0.0, 0.0, 1.0],
        "ptc_suspension_axis": [0.0, 0.0, 1.0],
        "ptc_radius": 0.43,
        "ptc_width": 0.41,
        "ptc_steers": False,
        "ptc_steering_min": -34.0,
        "ptc_steering_max": 34.0,
        "ptc_driven": True,
        "ptc_brakes": True,
        "ptc_suspension_travel": 0.28,
    }
    for key, value in settings.items():
        hub[key] = value
    for key in ("ptc_radius", "ptc_width", "ptc_suspension_travel"):
        hub.id_properties_ui(key).update(min=0)
    tag(empty(context, f"Spin.{identity}", hub), "wheel_spin", f"{identity}.spin")
    return hub


def spin_pivot(hub):
    if hub is None or hub.get(ROLE) != "wheel_hub":
        raise ValueError("Choose a wheel hub from this vehicle")
    pivots = [obj for obj in hub.children if obj.get(ROLE) == "wheel_spin"]
    if len(pivots) != 1:
        raise ValueError(f"{hub.name} needs exactly one spin pivot")
    return pivots[0]


def source_mesh(obj):
    return obj is not None and obj.type == "MESH" and not obj.library and not obj.children


def replace_visual(context, mesh, parent, role):
    require_derived_file()
    if not source_mesh(mesh) or mesh.get(ROLE) not in (None, "archived", role):
        raise ValueError("Select a local leaf mesh or a previously archived visual")
    identity = f"{parent[IDENTITY]}.{role}"
    ancestor = parent
    while ancestor.parent:
        ancestor = ancestor.parent
    if mesh.get(ROLE) == role and mesh.parent != parent:
        raise ValueError("This mesh already serves another visual slot")
    if mesh.get(ROLE) == "archived" and (
        mesh.parent != parent or mesh.get("ptc_archived_slot") != role
    ):
        raise ValueError("This archived mesh belongs to another visual slot")
    if any(obj.get(IDENTITY) == identity and obj.parent != parent for obj in components(ancestor)):
        raise ValueError(f"Duplicate visual ID {identity}")
    for previous in parent.children:
        if previous != mesh and previous.get(ROLE) == role:
            previous["ptc_archived_slot"] = role
            previous["ptc_archived_render"] = previous.hide_render
            previous["ptc_archived_view"] = previous.hide_get()
            previous[ROLE] = "archived"
            previous.hide_set(True)
            previous.hide_render = True
    if mesh.get(ROLE) == "archived":
        mesh.hide_set(mesh.pop("ptc_archived_view", False))
        mesh.hide_render = mesh.pop("ptc_archived_render", False)
        del mesh["ptc_archived_slot"]
    if mesh.parent != parent:
        reparent(mesh, parent)
    tag(mesh, role, identity)
    context.view_layer.update()
    return mesh


def migrate_jeep(context, body, identity):
    require_derived_file()
    if not body or body.type != "MESH" or body.parent or body.library or body.get(ROLE):
        raise ValueError("Select the original, local Jeep chassis mesh")
    if identity.strip() in (*LEGACY_ANCHORS, *JEEP_WHEELS, "chassis_collision"):
        raise ValueError("Choose a vehicle ID distinct from the Jeep's component IDs")
    direct = {obj.name: obj for obj in body.children}
    anchors = {name: direct.get(name) for name in LEGACY_ANCHORS}
    wheels = []
    for suffix in JEEP_WHEELS:
        hub = direct.get(f"Hub.{suffix}")
        tire = bpy.data.objects.get(f"Wheel.{suffix}")
        if not hub or hub.type != "EMPTY" or not source_mesh(tire) or tire.parent != hub:
            raise ValueError(f"Legacy Jeep is missing the hub or wheel mesh for {suffix}")
        wheels.append((suffix, hub, tire))
    if any(obj is None or obj.type != "EMPTY" for obj in anchors.values()):
        raise ValueError("Legacy Jeep is missing a required seat or access Empty")
    if any(obj.library or obj.get(ROLE) for _, hub, tire in wheels for obj in (hub, tire)):
        raise ValueError("Legacy wheel objects must be local and untagged")
    if any(obj.library or obj.get(ROLE) for obj in anchors.values()):
        raise ValueError("Legacy anchor objects must be local and untagged")
    root = assign_root(context, empty(context, "Vehicle Root"), identity)
    root["ptc_controller"] = "jeep"
    root["ptc_dimensions"] = [2.08, 4.22, 1.2]
    root["ptc_mass_kg"] = 1450.0
    for obj in direct.values():
        reparent(obj, root)
    replace_visual(context, body, root, "body")
    for name, obj in anchors.items():
        tag(obj, LEGACY_ANCHORS[name], name)
    for suffix, hub, tire in wheels:
        wheel_hub(context, root, suffix, hub=hub)
        hub["ptc_steers"] = suffix.startswith("F")
        context.view_layer.update()
        replace_visual(context, tire, spin_pivot(hub), "wheel_visual")
    # The legacy controller uses an analytic chassis box independent of visible meshes.
    collider = tag(
        empty(context, "Chassis collision", root, (0, 0, 0.42), "CUBE"),
        "collision",
        "chassis_collision",
    )
    collider.empty_display_size = 1
    collider.scale = (1.04, 2.11, 0.6)
    return root


def append_geometry(context, filepath, object_name=""):
    require_derived_file()
    source = Path(bpy.path.abspath(filepath)).expanduser().resolve()
    if not source.is_file():
        raise ValueError("Choose an existing .blend, .glb or .gltf file")
    if bpy.data.filepath and source == Path(bpy.data.filepath).resolve():
        raise ValueError("Choose an external geometry file")
    if source.suffix.lower() == ".blend":
        with bpy.data.libraries.load(str(source), link=False) as (available, target):
            if object_name and object_name not in available.objects:
                raise ValueError(f"No object named {object_name} in {source.name}")
            target.objects = [object_name] if object_name else list(available.objects)
        imported = [obj for obj in target.objects if obj is not None]
        if object_name and (len(imported) != 1 or imported[0].type != "MESH"):
            raise ValueError(f"{object_name} is not a mesh")
        # Evaluate copied parent transforms before flattening imported visual geometry.
        for obj in imported:
            context.collection.objects.link(obj)
    elif source.suffix.lower() in {".gltf", ".glb"}:
        before = set(bpy.data.objects)
        if bpy.ops.import_scene.gltf(filepath=str(source)) != {"FINISHED"}:
            raise ValueError(f"Could not import {source.name}")
        imported = [obj for obj in context.scene.objects if obj not in before]
    else:
        raise ValueError("Choose an existing .blend, .glb or .gltf file")
    context.view_layer.update()
    meshes = [obj for obj in imported if obj.type == "MESH"]
    if not meshes:
        raise ValueError(f"No geometry found in {source.name}")
    worlds = [(obj, obj.matrix_world.copy()) for obj in meshes]
    for obj, world in worlds:
        obj.parent = None
        obj.matrix_parent_inverse.identity()
        obj.matrix_world = world
        for key in tuple(obj.keys()):
            if key.startswith("ptc_"):
                del obj[key]
    for obj in imported:
        if obj.type != "MESH":
            for collection in list(obj.users_collection):
                collection.objects.unlink(obj)
    context.view_layer.update()
    return meshes


def correct_geometry(context, mesh, offset, rotation, scale, origin):
    require_derived_file()
    if not source_mesh(mesh) or scale <= 0:
        raise ValueError("Select a local leaf mesh and use a positive correction scale")
    # Offset and Euler correction are in the parent frame. Origin is in mesh-local meters.
    correction = (
        Matrix.Translation(offset) @ Euler(rotation).to_matrix().to_4x4() @ Matrix.Scale(scale, 4)
    )
    local = mesh.matrix_local.copy()
    if any(origin):
        mesh.data = mesh.data.copy()
        mesh.data.transform(Matrix.Translation(-Vector(origin)), shape_keys=True)
        local @= Matrix.Translation(origin)
    parent_world = mesh.parent.matrix_world if mesh.parent else Matrix.Identity(4)
    mesh.matrix_world = parent_world @ correction @ local
    context.view_layer.update()


def author_anchor(context, root, parent, role, identity, obj=None):
    require_derived_file()
    identity = require_new_id(root, identity)
    if role not in ANCHOR_ROLES or parent not in components(root) or parent.type != "EMPTY":
        raise ValueError("Choose a mechanical parent on the active vehicle")
    if obj:
        if (
            obj.type != "EMPTY"
            or obj.library
            or obj.get(ROLE)
            or obj == parent
            or obj in ancestors(parent)
        ):
            raise ValueError("Select a local, unassigned Empty")
        reparent(obj, parent)
    else:
        obj = empty(context, identity, parent, shape="CUBE" if role == "collision" else "ARROWS")
        if role == "collision":
            obj.empty_display_size = 1
    return tag(obj, role, identity)


def wheel_axis(hub, prop):
    vector = Vector(hub[prop])
    if (
        len(vector) != 3
        or not all(math.isfinite(value) for value in vector)
        or vector.length < 1e-6
    ):
        raise ValueError(f"{hub.name}: {prop} must be a finite, nonzero axis")
    return vector.normalized()


def clear_preview(scene=None, root=None):
    objects = bpy.data.objects if scene is None else scene.objects
    for obj in list(objects):
        if PREVIEW in obj and (root is None or obj.get(PREVIEW) == root.name):
            bpy.data.objects.remove(obj, do_unlink=True)


@persistent
def before_save(_):
    clear_preview()


def preview(context, root, spin, steer, suspension, *, show_geometry=True):
    require_derived_file()
    wheels = [obj for obj in components(root) if obj.get(ROLE) == "wheel_hub"]
    for hub in wheels:
        for key in ("ptc_rotation_axis", "ptc_suspension_axis", "ptc_steering_axis"):
            wheel_axis(hub, key)
        spin_pivot(hub)
    clear_preview(context.scene, root)
    for hub in wheels:
        pivot = spin_pivot(hub)
        axis = wheel_axis(hub, "ptc_rotation_axis")
        suspension_axis = wheel_axis(hub, "ptc_suspension_axis")
        travel = hub["ptc_suspension_travel"]
        for title, vector, length in (
            ("Spin axis", axis, hub["ptc_width"]),
            ("Suspension travel", suspension_axis, travel),
        ):
            guide = empty(context, title, shape="SINGLE_ARROW")
            guide[PREVIEW] = root.name
            guide.matrix_world = (
                hub.matrix_world @ UP.rotation_difference(vector).to_matrix().to_4x4()
            )
            guide.empty_display_size = max(0.03, length)
            guide.show_in_front = True
        if not show_geometry:
            continue
        low, high = map(math.radians, (hub["ptc_steering_min"], hub["ptc_steering_max"]))
        steering = min(max(steer, low), high) if hub["ptc_steers"] else 0
        pose = (
            hub.matrix_world
            @ Matrix.Translation(suspension_axis * travel * suspension)
            @ Quaternion(wheel_axis(hub, "ptc_steering_axis"), steering).to_matrix().to_4x4()
            @ pivot.matrix_local
            @ Quaternion(axis, spin).to_matrix().to_4x4()
        )
        for visual in pivot.children:
            if visual.get(ROLE) == "wheel_visual":
                ghost = bpy.data.objects.new(f"Preview · {visual.name}", visual.data)
                context.collection.objects.link(ghost)
                ghost[PREVIEW] = root.name
                ghost.matrix_world = pose @ visual.matrix_local
                ghost.display_type = "WIRE"
                ghost.show_in_front = True
                ghost.hide_render = True


def validate(root):
    if not is_vehicle_root(root):
        raise ValueError("Choose a tagged contract-v1 root Empty at the origin with unit scale")
    dimensions = root.get("ptc_dimensions", ())
    if len(dimensions) != 3 or any(not math.isfinite(v) or v <= 0 for v in dimensions):
        raise ValueError("Vehicle dimensions must be three positive, finite values")
    if not math.isfinite(root["ptc_mass_kg"]) or root["ptc_mass_kg"] < 0:
        raise ValueError("Mass must be nonnegative and finite")
    nodes = [obj for obj in components(root) if obj.get(ROLE)]
    if not any(obj.type == "MESH" and obj.get(ROLE) in VISUAL_ROLES for obj in nodes):
        raise ValueError("Assign at least one visual mesh before exporting")
    ids = set()
    for obj in nodes:
        role, identity = obj.get(ROLE), obj.get(IDENTITY)
        if not isinstance(identity, str) or not identity.strip() or identity in ids:
            raise export_issue(obj, "Missing or duplicate semantic ID", "Assign a unique ID")
        ids.add(identity)
        if obj.parent and (obj.parent not in nodes or obj.parent.get(ROLE) in VISUAL_ROLES):
            raise export_issue(obj, "Invalid parent", "Parent it to a tagged mechanical Empty")
        if role in VISUAL_ROLES:
            expected = {"wheel_visual": "wheel_spin", "mechanical_visual": "joint"}.get(
                role, "root"
            )
            if obj.type != "MESH" or obj.children or obj.parent.get(ROLE) != expected:
                raise export_issue(
                    obj, "Invalid visual slot or children", f"Use a leaf mesh under {expected}"
                )
        elif obj.type != "EMPTY" or role not in (*ANCHOR_ROLES, "root", "wheel_spin", "wheel_hub"):
            raise export_issue(
                obj, "Invalid mechanical type or role", "Use a tagged mechanical Empty"
            )
        if role not in ("collision", *VISUAL_ROLES) and not is_rigid(obj.matrix_local):
            raise export_issue(obj, "Mechanical transform has scale or shear", "Restore unit scale")
        if role == "wheel_hub":
            pivot = spin_pivot(obj)
            if not any(child.get(ROLE) == "wheel_visual" for child in pivot.children):
                raise export_issue(obj, "Missing wheel visual", "Assign a mesh to its spin pivot")
            for axis in ("ptc_rotation_axis", "ptc_steering_axis", "ptc_suspension_axis"):
                wheel_axis(obj, axis)
            radius, width, travel, low, high = (
                obj[key]
                for key in (
                    "ptc_radius",
                    "ptc_width",
                    "ptc_suspension_travel",
                    "ptc_steering_min",
                    "ptc_steering_max",
                )
            )
            if not all(math.isfinite(v) for v in (radius, width, travel, low, high)) or (
                radius <= 0 or width <= 0 or travel < 0 or not -180 <= low <= 0 <= high <= 180
            ):
                raise ValueError(f"{obj.name}: invalid wheel dimensions, travel or steering limits")
        if role == "wheel_spin" and obj.parent.get(ROLE) != "wheel_hub":
            raise export_issue(obj, "Invalid spin pivot parent", "Parent it to its wheel hub")
    return nodes


def layer_paths(layer, parents=()):
    path = (*parents, layer)
    yield path
    for child in layer.children:
        yield from layer_paths(child, path)


def export_issue(obj, reason, remedy):
    collections = ", ".join(collection.name for collection in obj.users_collection) or "none"
    return ValueError(
        f"{obj.name} [{obj.get(ROLE)} / {obj.get(IDENTITY)}], "
        f"collections: {collections}. {reason}. {remedy}."
    )


def visibility_plan(context, nodes):
    paths = {}
    for path in layer_paths(context.view_layer.layer_collection):
        paths.setdefault(path[-1].collection, []).append(path)

    def restrictions(path):
        return sum(
            layer.exclude
            + layer.hide_viewport
            + layer.collection.hide_viewport
            + layer.collection.hide_select
            for layer in path
        )

    layers = {}
    for obj in nodes:
        choices = [
            path for collection in obj.users_collection for path in paths.get(collection, ())
        ]
        if not choices:
            raise export_issue(
                obj,
                "No collection path exists in the current scene",
                "Link the component to a collection in this scene",
            )
        for layer in min(choices, key=restrictions):
            layers.setdefault(layer, obj)
    return layers


def set_temporary(stack, owner, attribute, value):
    old = getattr(owner, attribute)
    if old != value:
        setattr(owner, attribute, value)
        stack.callback(setattr, owner, attribute, old)


def restore_layer_flags(flags):
    # Blender can cascade exclusion changes from a parent to its descendants.
    # Restore parents first, then overwrite any cascaded child flags.
    for layer, excluded in flags:
        if layer.exclude != excluded:
            layer.exclude = excluded


@contextmanager
def prepared_components(context, nodes, *, keep_visible=False):
    view = context.view_layer
    show = visibility_plan(context, nodes)
    flags = [(path[-1], path[-1].exclude) for path in layer_paths(view.layer_collection)]
    active = view.objects.active
    existing = {obj: obj.select_get(view_layer=view) for obj in view.objects}
    with ExitStack() as stack:
        stack.callback(view.update)
        stack.callback(restore_layer_flags, flags)
        for layer, _ in flags:
            if layer in show:
                try:
                    layer.exclude = False
                    set_temporary(stack, layer, "hide_viewport", False)
                    set_temporary(stack, layer.collection, "hide_viewport", False)
                    set_temporary(stack, layer.collection, "hide_select", False)
                except (RuntimeError, AttributeError, TypeError) as error:
                    raise export_issue(
                        show[layer],
                        f"Could not open collection {layer.name}: {error}",
                        "Use an editable collection or link the component through another path",
                    ) from error
        restore_layer_flags([(layer, excluded) for layer, excluded in flags if layer not in show])
        view.update()
        for obj in nodes:
            try:
                set_temporary(stack, obj, "hide_viewport", False)
                set_temporary(stack, obj, "hide_select", False)
            except (RuntimeError, AttributeError, TypeError) as error:
                raise export_issue(
                    obj, f"Could not reveal the component: {error}", "Use an editable local object"
                ) from error
        view.update()

        def restore_selection():
            for obj, selected in selection.items():
                if view.objects.get(obj.name) == obj:
                    obj.select_set(selected, view_layer=view)
            view.objects.active = active

        selection = {obj: obj.select_get(view_layer=view) for obj in view.objects}
        selection.update(existing)
        stack.callback(restore_selection)
        for obj in nodes:
            if view.objects.get(obj.name) != obj:
                raise export_issue(
                    obj,
                    "Component is absent after opening its collection path",
                    "Check its collection links and library overrides",
                )
            hidden = obj.hide_get(view_layer=view)
            if hidden:
                stack.callback(obj.hide_set, True, view_layer=view)
                obj.hide_set(False, view_layer=view)
        view.update()
        for obj in nodes:
            if not obj.visible_get(view_layer=view):
                raise export_issue(
                    obj,
                    "Blender still marks it invisible",
                    "Check collection visibility and whether the object can be edited locally",
                )
        yield
        if keep_visible:
            stack.pop_all()


def export_destination(filepath):
    output = Path(bpy.path.abspath(filepath)).expanduser().resolve()
    if output.suffix.lower() != ".glb" or in_source(output):
        raise ValueError("Choose a .glb output outside assets/source")
    parent = next((path for path in output.parents if path.exists()), None)
    if output.is_dir() or (parent is not None and not parent.is_dir()):
        raise ValueError(f"{output}: choose an output file under a directory")
    return output


def preflight(context, root, filepath):
    require_derived_file()
    if context.mode != "OBJECT":
        raise ValueError("Switch Blender to Object Mode before validating or exporting")
    output = export_destination(filepath)
    context.view_layer.update()
    nodes = validate(root)
    visibility_plan(context, nodes)
    return output, nodes


def verify_export(candidate, nodes):
    with candidate.open("rb") as stream:
        magic, version, _ = struct.unpack("<4sII", stream.read(12))
        size, chunk = struct.unpack("<I4s", stream.read(8))
        doc = json.loads(stream.read(size))
    if (magic, version, chunk) != (b"glTF", 2, b"JSON"):
        raise ValueError("Blender did not produce a glTF 2.0 GLB")
    exported = doc["nodes"]
    extras = [node.get("extras", {}) for node in exported]
    expected = {obj[IDENTITY]: obj for obj in nodes}
    actual = {}
    parents = {
        child: index for index, node in enumerate(exported) for child in node.get("children", ())
    }
    for index, data in enumerate(extras):
        identity = data.get(IDENTITY)
        if identity not in expected or identity in actual:
            raise ValueError("GLB contains an unrelated, untagged or duplicate component")
        obj = expected[identity]
        actual[identity] = index
        parent = extras[parents[index]].get(IDENTITY) if index in parents else None
        if data.get(ROLE) != obj[ROLE] or parent != (obj.parent[IDENTITY] if obj.parent else None):
            raise export_issue(obj, "The GLB changed its role or parent", "Check the hierarchy")
    for identity in expected.keys() - actual.keys():
        raise export_issue(
            expected[identity],
            "Blender omitted this active component",
            "Check its object type and collection membership",
        )


def select_export_nodes(context, root, nodes):
    view = context.view_layer
    for obj in list(context.selected_objects):
        obj.select_set(False, view_layer=view)
    for obj in nodes:
        obj.select_set(True, view_layer=view)
        if not obj.select_get(view_layer=view):
            raise export_issue(obj, "It cannot be selected for export", "Check selection locks")
    view.objects.active = root


def export_vehicle(context, root, filepath, *, animations=False, exporter=None):
    output, nodes = preflight(context, root, filepath)
    exporter = exporter or bpy.ops.export_scene.gltf
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="ptc-vehicle-", dir=output.parent) as folder:
        candidate = Path(folder) / output.name
        with prepared_components(context, nodes):
            select_export_nodes(context, root, nodes)
            result = exporter(
                filepath=str(candidate),
                export_format="GLB",
                use_selection=True,
                export_extras=True,
                export_yup=True,
                export_animations=animations,
                export_cameras=False,
                export_lights=False,
            )
            if result != {"FINISHED"} or not candidate.is_file():
                raise ValueError(f"{output}: Blender did not finish the GLB export")
            verify_export(candidate, nodes)
        candidate.replace(output)
    return output


WHEEL_FIELDS = (
    ("ptc_rotation_axis", "Rotation axis"),
    ("ptc_steering_axis", "Steering axis"),
    ("ptc_suspension_axis", "Suspension axis"),
    ("ptc_radius", "Radius (m)"),
    ("ptc_width", "Width (m)"),
    ("ptc_steers", "Steers"),
    ("ptc_steering_min", "Minimum steering (°)"),
    ("ptc_steering_max", "Maximum steering (°)"),
    ("ptc_driven", "Driven"),
    ("ptc_brakes", "Brakes"),
    ("ptc_suspension_travel", "Suspension travel (m)"),
)


class PTCVehicleSettings(bpy.types.PropertyGroup):
    root: PointerProperty(name="Configured vehicle root", type=bpy.types.Object, poll=root_poll)
    visual: PointerProperty(name="Mesh", type=bpy.types.Object)
    wheel: PointerProperty(name="Wheel hub", type=bpy.types.Object)
    mechanical_parent: PointerProperty(name="Parent", type=bpy.types.Object)
    identity: StringProperty(name="Semantic ID")
    anchor_role: EnumProperty(
        name="Type", items=[(role, role.replace("_", " ").title(), "") for role in ANCHOR_ROLES]
    )
    source: StringProperty(name="Source", subtype="FILE_PATH")
    source_object: StringProperty(
        name="Source object", description="Leave blank to import all meshes"
    )
    output: StringProperty(name="GLB Output", subtype="FILE_PATH", default="//vehicle.glb")
    animations: BoolProperty(name="Include Animation Clips", default=False)
    offset: FloatVectorProperty(name="Translation", subtype="TRANSLATION", size=3)
    rotation: FloatVectorProperty(name="Euler correction", subtype="EULER", size=3)
    scale: FloatProperty(name="Scale factor", min=0.0001, default=1)
    origin: FloatVectorProperty(name="Local origin offset", subtype="TRANSLATION", size=3)
    spin: FloatProperty(name="Spin", subtype="ANGLE", min=-math.tau, max=math.tau)
    steer: FloatProperty(name="Steering", subtype="ANGLE", min=-math.pi, max=math.pi)
    suspension: FloatProperty(name="Suspension fraction", min=0, max=1)


OPERATIONS = {
    "INITIALIZE": "Initialize Vehicle",
    "ROOT": "Assign Active Empty as Root",
    "RESOLVE": "Use Selected Object's Vehicle",
    "LOCATE": "Locate Component",
    "MIGRATE": "Migrate Legacy Jeep",
    "IMPORT": "Import Source Geometry",
    "CORRECT": "Apply Geometry Corrections",
    "BODY": "Replace Body Mesh",
    "INTERIOR": "Assign Interior Mesh",
    "JOINT_MESH": "Assign Joint Mesh",
    "WHEEL": "Create Wheel at 3D Cursor",
    "TIRE": "Replace Wheel Mesh",
    "ANCHOR": "Create Anchor or Joint",
    "ASSIGN": "Assign Active Empty as Component",
    "GUIDES": "Show Wheel Guides",
    "PREVIEW": "Preview Wheel Motion",
    "CLEAR": "Clear Wheel Preview",
    "CHECK": "Validate Vehicle",
    "EXPORT": "Export Vehicle GLB",
}


class PTCVEHICLE_OT_author(bpy.types.Operator):
    bl_idname = "ptc_vehicle.author"
    bl_label = "PTC Vehicle Authoring"
    bl_options = {"REGISTER", "UNDO"}

    operation: EnumProperty(items=[(key, value, "") for key, value in OPERATIONS.items()])
    component_name: StringProperty(options={"HIDDEN", "SKIP_SAVE"})

    def execute(self, context):
        settings = context.scene.ptc_vehicle
        root, active = settings.root, context.active_object
        identity = settings.identity
        op = self.operation
        try:
            require_derived_file()
            if op == "INITIALIZE":
                assign_root(context, empty(context, "Vehicle Root"), identity or "vehicle")
            elif op == "ROOT":
                assign_root(context, active, identity or (active.name if active else "vehicle"))
            elif op == "RESOLVE":
                owner = owning_root(active)
                if not root_in_scene(context.scene, owner):
                    raise ValueError("Select a vehicle root or a descendant of an authored vehicle")
                settings.root = owner
            elif op == "MIGRATE":
                migrate_jeep(context, active, identity or "jeep")
            elif op == "IMPORT":
                settings.visual = append_geometry(context, settings.source, settings.source_object)[
                    0
                ]
            elif op == "CORRECT":
                correct_geometry(
                    context,
                    settings.visual or active,
                    settings.offset,
                    settings.rotation,
                    settings.scale,
                    settings.origin,
                )
            else:
                if not root_in_scene(context.scene, root):
                    raise ValueError("Initialize, choose, or resolve a valid vehicle root first")
                self.edit_vehicle(context, root, active, settings)
        except (ValueError, RuntimeError, OSError, KeyError, TypeError, struct.error) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}

    def edit_vehicle(self, context, root, active, settings):
        op = self.operation
        mesh = settings.visual or active
        identity = settings.identity
        if op in {"BODY", "INTERIOR"}:
            replace_visual(context, mesh, root, op.lower())
        elif op == "JOINT_MESH":
            joint = settings.mechanical_parent
            if joint not in components(root) or joint.get(ROLE) != "joint":
                raise ValueError("Choose a joint on the active vehicle")
            replace_visual(context, mesh, joint, "mechanical_visual")
        elif op == "WHEEL":
            settings.wheel = wheel_hub(context, root, identity, context.scene.cursor.location)
        elif op == "TIRE":
            hub = selected_hub(root, settings.wheel, active)
            if hub not in components(root):
                raise ValueError("Choose a wheel hub on the active vehicle")
            replace_visual(context, mesh, spin_pivot(hub), "wheel_visual")
        elif op in {"ANCHOR", "ASSIGN"}:
            if op == "ASSIGN" and not active:
                raise ValueError("Select an Empty to assign")
            parent = settings.mechanical_parent or root
            obj = author_anchor(
                context,
                root,
                parent,
                settings.anchor_role,
                identity,
                active if op == "ASSIGN" else None,
            )
            if op == "ANCHOR":
                obj.location = parent.matrix_world.inverted() @ context.scene.cursor.location
        elif op in {"GUIDES", "PREVIEW"}:
            preview(
                context,
                root,
                settings.spin,
                settings.steer,
                settings.suspension,
                show_geometry=op == "PREVIEW",
            )
        elif op == "CLEAR":
            clear_preview(context.scene, root)
        elif op == "LOCATE":
            obj = context.scene.objects.get(self.component_name)
            if obj not in components(root) or obj.type != "EMPTY":
                raise ValueError("Choose a mechanical component on the configured vehicle")
            with prepared_components(context, (obj,), keep_visible=True):
                for selected in context.selected_objects:
                    selected.select_set(False)
                obj.select_set(True)
                context.view_layer.objects.active = obj
        elif op == "CHECK":
            _, nodes = preflight(context, root, settings.output)
            with prepared_components(context, nodes):
                select_export_nodes(context, root, nodes)
            self.report({"INFO"}, f"Validated {len(nodes)} vehicle components for export")
        elif op == "EXPORT":
            result = export_vehicle(context, root, settings.output, animations=settings.animations)
            self.report({"INFO"}, f"Exported {result.name}")


def selected_hub(root, configured, active):
    if active is not None and owning_root(active) == root:
        for obj in (active, *ancestors(active)):
            if obj.get(ROLE) == "wheel_hub":
                return obj
    if configured is not None and configured.get(ROLE) == "wheel_hub":
        if owning_root(configured) == root:
            return configured
    return None


class PTCVEHICLE_PT_sidebar(bpy.types.Panel):
    bl_label = "PTC Vehicle"
    bl_idname = "PTCVEHICLE_PT_sidebar"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "PTC Vehicle"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.ptc_vehicle
        root = settings.root
        layout.prop(settings, "root")
        active = context.active_object
        layout.label(text=f"Selected object: {active.name if active else 'None'}")
        layout.prop(settings, "identity")
        self.buttons(layout, "INITIALIZE", "ROOT", "RESOLVE", "MIGRATE")
        source = layout.box()
        source.label(text="Visual geometry")
        for field in ("source", "source_object"):
            source.prop(settings, field)
        self.buttons(source, "IMPORT")
        source.prop(settings, "visual")
        for field in ("offset", "rotation", "scale", "origin"):
            source.prop(settings, field)
        self.buttons(source, "CORRECT")
        if not root_in_scene(context.scene, root):
            layout.label(text="Choose a vehicle root or resolve it from a selected component.")
            return
        self.buttons(source, "BODY", "INTERIOR")
        layout.prop(root, '["ptc_dimensions"]', text="Width / Length / Height (m)")
        layout.prop(root, '["ptc_mass_kg"]', text="Mass (kg)")
        layout.label(text="Blender +Z up, −Y forward")
        mechanics = layout.box()
        mechanics.label(text="Mechanical hierarchy")
        for obj in components(root):
            role = obj.get(ROLE)
            if role and role not in VISUAL_ROLES:
                button = mechanics.operator(
                    PTCVEHICLE_OT_author.bl_idname,
                    text=f"{'    ' * hierarchy_depth(obj)}{obj.get(IDENTITY)}  [{role}]",
                )
                button.operation = "LOCATE"
                button.component_name = obj.name
        wheels = layout.box()
        wheels.label(text="Wheels")
        self.buttons(wheels, "WHEEL")
        wheels.prop(settings, "wheel")
        hub = selected_hub(root, settings.wheel, active)
        if hub and hub.get(ROLE) == "wheel_hub":
            wheels.prop(hub, "location", text="Hub position")
            for field, label in WHEEL_FIELDS:
                wheels.prop(hub, f'["{field}"]', text=label)
            self.buttons(wheels, "TIRE")
        else:
            wheels.label(text="Select a hub or wheel visual on this vehicle.")
        for field in ("spin", "steer", "suspension"):
            wheels.prop(settings, field)
        self.buttons(wheels, "GUIDES", "PREVIEW", "CLEAR")
        anchors = layout.box()
        anchors.label(text="Anchors and joints")
        anchors.prop(settings, "anchor_role")
        anchors.prop(settings, "mechanical_parent")
        self.buttons(anchors, "ANCHOR", "ASSIGN", "JOINT_MESH")
        layout.prop(settings, "output")
        layout.prop(settings, "animations")
        self.buttons(layout, "CHECK", "EXPORT")

    @staticmethod
    def buttons(layout, *operations):
        for operation in operations:
            button = layout.operator(PTCVEHICLE_OT_author.bl_idname, text=OPERATIONS[operation])
            button.operation = operation


CLASSES = (PTCVehicleSettings, PTCVEHICLE_OT_author, PTCVEHICLE_PT_sidebar)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ptc_vehicle = PointerProperty(type=PTCVehicleSettings)
    bpy.app.handlers.save_pre.append(before_save)


def unregister():
    clear_preview()
    bpy.app.handlers.save_pre.remove(before_save)
    del bpy.types.Scene.ptc_vehicle
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
