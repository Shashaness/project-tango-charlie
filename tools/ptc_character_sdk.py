"""Blender 5 character authoring tools and standalone sidebar add-on."""

import json
import math
import tempfile
from contextlib import contextmanager
from pathlib import Path

import addon_utils
import bpy
from bpy.props import EnumProperty, FloatProperty, PointerProperty, StringProperty
from mathutils import Matrix, Vector

bl_info = {
    "name": "PTC Character",
    "author": "Project Tango Charlie",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "3D View > Sidebar > PTC Character",
    "category": "Rigging",
}

CLIPS = ("Idle", "Walk", "Run", "Turn", "Seated")
PARENTS = dict(Hips=None, Spine="Hips", Chest="Spine", Neck="Chest", Head="Neck")
for side in ("L", "R"):
    for part, parent in (
        ("Clavicle", "Chest"),
        ("UpperArm", f"Clavicle.{side}"),
        ("Forearm", f"UpperArm.{side}"),
        ("Hand", f"Forearm.{side}"),
        ("Thigh", "Hips"),
        ("Shin", f"Thigh.{side}"),
        ("Foot", f"Shin.{side}"),
    ):
        PARENTS[f"{part}.{side}"] = parent

# Landmarks in meters on the source at its original 1.892417 m height.
TORSO = (
    (0, 0.02, 0.94),
    (0, 0.01, 1.08),
    (0, -0.011, 1.28),
    (0, -0.025, 1.54),
    (0, -0.037, 1.64),
    (0, -0.035, 1.85),
)
ARM = (
    (0.065, -0.01, 1.47),
    (0.18, 0.007, 1.465),
    (0.30, 0.013, 1.24),
    (0.411, -0.009, 1.046),
    (0.447, -0.028, 0.934),
)
LEG = ((0.104, 0.022, 0.924), (0.17, 0.047, 0.55), (0.231, 0.088, 0.112), (0.266, -0.103, 0.040))
RIGIFY = dict(
    zip(
        tuple(PARENTS)[:5],
        ("spine", "spine.001", "spine.002", "spine.004", "spine.006"),
        strict=True,
    )
)
for side in ("L", "R"):
    for part, target in zip(
        ("Clavicle", "UpperArm", "Forearm", "Hand", "Thigh", "Shin", "Foot"),
        ("shoulder", "upper_arm", "forearm", "hand", "thigh", "shin", "foot"),
        strict=True,
    ):
        RIGIFY[f"{part}.{side}"] = f"{target}.{side}"


def activate(obj):
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")
    obj.hide_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def initialize(mesh, height, yaw=0):
    """Apply imported object transforms and ground the mesh without changing UVs."""
    activate(mesh)
    mesh.data = mesh.data.copy() if mesh.data.users > 1 else mesh.data
    source = Matrix.Rotation(yaw, 4, "Z") @ mesh.matrix_world
    points = [source @ v.co for v in mesh.data.vertices]
    lo = Vector(min(v[i] for v in points) for i in range(3))
    hi = Vector(max(v[i] for v in points) for i in range(3))
    if hi.z - lo.z < 1e-5:
        raise ValueError("Imported mesh must have vertical extent.")
    scale = height / (hi.z - lo.z)
    offset = Vector(((hi.x + lo.x) / 2, (hi.y + lo.y) / 2, lo.z))
    mesh.parent = None
    mesh.matrix_world = Matrix.Identity(4)
    mesh.data.transform(Matrix.Scale(scale, 4) @ Matrix.Translation(-offset) @ source)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1
    return height


def create_rig(mesh, height):
    """Create editable anatomical landmarks as a starting point for fitting."""
    rig = bpy.data.objects.new("Character Deform", bpy.data.armatures.new("Character Skeleton"))
    mesh.users_collection[0].objects.link(rig)
    rig.show_in_front = True
    activate(rig)
    bpy.ops.object.mode_set(mode="EDIT")
    points = dict(zip(tuple(PARENTS)[:5], zip(TORSO, TORSO[1:])))
    for side, sign in (("L", 1), ("R", -1)):
        for landmarks, names in (
            (ARM, ("Clavicle", "UpperArm", "Forearm", "Hand")),
            (LEG, ("Thigh", "Shin", "Foot")),
        ):
            positions = [(sign * x, y, z) for x, y, z in landmarks]
            for part, head, tail in zip(names, positions, positions[1:]):
                points[f"{part}.{side}"] = head, tail
    for role, parent in PARENTS.items():
        head, tail = points[role]
        bone = rig.data.edit_bones.new(role)
        bone.head = Vector(head) * (height / 1.892417)
        bone.tail = Vector(tail) * (height / 1.892417)
        bone.align_roll(Vector((0, -1, 0)))
        if parent:
            bone.parent = rig.data.edit_bones[parent]
            bone.use_connect = (bone.parent.tail - bone.head).length < 1e-6
    bpy.ops.object.mode_set(mode="OBJECT")
    return rig


def mapping(mesh, rig):
    roles = json.loads(mesh.get("ptc_semantics", json.dumps({role: role for role in PARENTS})))
    if (
        not isinstance(roles, dict)
        or set(roles) != set(PARENTS)
        or any(not isinstance(name, str) for name in roles.values())
        or len(set(roles.values())) != len(roles)
        or any(
            name not in rig.data.bones or not rig.data.bones[name].use_deform
            for name in roles.values()
        )
    ):
        raise ValueError("Map every anatomical role to a distinct deform bone.")
    for role, parent in PARENTS.items():
        bone = rig.data.bones[roles[role]]
        if bone.length <= 1e-5 or (
            parent and roles[parent] not in {p.name for p in bone.parent_recursive}
        ):
            raise ValueError(f"Check the length and ancestry of {role}.")
    return roles


def bind(mesh, rig):
    if mesh.vertex_groups or deform_for(mesh):
        raise ValueError("Remove existing weights and armature modifiers before a new bind.")
    roles = mapping(mesh, rig)
    bones = [bone for bone in rig.data.bones if bone.use_deform]
    if any(bone.length <= 1e-5 for bone in bones):
        raise ValueError("Every deform bone needs a nonzero length.")
    groups = {b.name: mesh.vertex_groups.new(name=b.name) for b in bones}
    for vertex in mesh.data.vertices:
        point = vertex.co
        distances = []
        for bone in bones:
            end = bone.tail_local - bone.head_local
            position = (point - bone.head_local).dot(end) / end.length_squared
            nearest = bone.head_local + end * max(0.07, min(0.93, position))
            distances.append(((point - nearest).length, bone.name))
        distances.sort()
        best = distances[0][0]
        influences = [
            (math.exp(-(((distance - best) / 0.034) ** 2)), name)
            for distance, name in distances[:4]
            if distance <= best + 0.065
        ]
        total = sum(weight for weight, _ in influences)
        for weight, name in influences:
            groups[name].add([vertex.index], weight / total, "REPLACE")
    mesh.parent = rig
    modifier = mesh.modifiers.new("Character Skin", "ARMATURE")
    modifier.object = rig
    mesh["ptc_character_version"] = 1
    mesh["ptc_semantics"] = json.dumps(roles, sort_keys=True)


def inspect(mesh):
    mesh.data.calc_loop_triangles()
    images = sorted(
        {
            node.image.name
            for material in mesh.data.materials
            if material and material.node_tree
            for node in material.node_tree.nodes
            if node.type == "TEX_IMAGE" and node.image
        }
    )
    return (
        f"{len(mesh.data.vertices):,} vertices, {len(mesh.data.loop_triangles):,} triangles. "
        f"Materials: {', '.join(mesh.data.materials.keys()) or 'none'}. "
        f"Images: {', '.join(images) or 'none'}."
    )


def validate(mesh, rig, *, actions=True):
    roles = mapping(mesh, rig)
    if any(
        max(abs(matrix[r][c] - (r == c)) for r in range(4) for c in range(4)) > 1e-5
        for matrix in (mesh.matrix_world, rig.matrix_world)
    ):
        raise ValueError("Apply mesh and armature object transforms before export.")
    if sum(m.type == "ARMATURE" and m.object == rig for m in mesh.modifiers) != 1:
        raise ValueError("Mesh must be bound to the selected deform armature.")
    if len(mesh.data.uv_layers) > 1:
        raise ValueError("PTC supports one UV map.")
    deform = {bone.name for bone in rig.data.bones if bone.use_deform}
    unweighted, invalid = [], []
    for vertex in mesh.data.vertices:
        weights = [entry for entry in vertex.groups if entry.weight > 0]
        if not weights:
            unweighted.append(vertex.index)
        elif (
            len(weights) > 4
            or abs(sum(entry.weight for entry in weights) - 1) > 1e-4
            or any(mesh.vertex_groups[w.group].name not in deform for w in weights)
        ):
            invalid.append(vertex.index)
    for vertex in mesh.data.vertices:
        vertex.select = False
    for index in (*unweighted, *invalid):
        mesh.data.vertices[index].select = True
    if unweighted or invalid:
        activate(mesh)
        raise ValueError(
            f"{len(unweighted)} unweighted and {len(invalid)} invalid vertices selected. "
            "Enter Edit Mode to inspect them."
        )
    if actions:
        tracks = action_tracks(rig)
        for clip in CLIPS:
            if (
                clip not in tracks
                or tracks[clip][1].frame_range[1] <= tracks[clip][1].frame_range[0]
            ):
                raise ValueError(f"Missing positive-duration Action {clip}.")
    mesh["ptc_character_version"] = 1
    mesh["ptc_semantics"] = json.dumps(roles, sort_keys=True)
    rig["ptc_semantics"] = mesh["ptc_semantics"]


def action_tracks(rig):
    if not rig.animation_data:
        return {}
    return {
        track.name: (track.strips[0], track.strips[0].action)
        for track in rig.animation_data.nla_tracks
        if len(track.strips) == 1 and track.strips[0].action
    }


def assign_action(rig, action, slot=None):
    data = rig.animation_data_create()
    data.action = action
    if action and slot:
        data.action_slot = slot


def add_track(rig, name, action, slot=None):
    data = rig.animation_data_create()
    old = data.nla_tracks.get(name)
    if old:
        data.nla_tracks.remove(old)
    track = data.nla_tracks.new()
    track.name = name
    strip = track.strips.new(name, 0, action)
    strip.action_slot = slot or action.slots[0]
    track.mute = True


def new_action(rig, name):
    action = bpy.data.actions.new(name)
    action.use_fake_user = True
    assign_action(rig, action)
    return action


def activate_preview(rig, clip):
    strip, action = action_tracks(rig)[clip]
    for track in rig.animation_data.nla_tracks:
        track.mute = True
    set_controls(rig, False)
    assign_action(rig, action, strip.action_slot)
    rig.data.pose_position = "POSE"
    start, end = action.frame_range
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = math.floor(start), math.ceil(end)
    scene.frame_set(math.floor(start), subframe=start % 1)


def key_pose(rig, frame):
    for bone in rig.pose.bones:
        bone.keyframe_insert("rotation_quaternion", frame=frame, group=bone.name)
        bone.keyframe_insert("location", frame=frame, group=bone.name)
        bone.keyframe_insert("scale", frame=frame, group=bone.name)


def pose_rotations(rig, rotations):
    """Set armature-space orientations and derive each joint's parent-local basis."""
    matrices = {}
    for bone in rig.pose.bones:
        rest = bone.bone.matrix_local
        parent = bone.parent
        parent_rest = parent.bone.matrix_local if parent else Matrix.Identity(4)
        parent_pose = matrices[parent.name] if parent else Matrix.Identity(4)
        natural = parent_pose @ parent_rest.inverted() @ rest
        rotation = rotations.get(bone.name)
        desired = (
            Matrix.Translation(natural.translation) @ rotation.to_matrix().to_4x4()
            if rotation
            else natural
        )
        bone.rotation_mode = "QUATERNION"
        bone.matrix_basis = bone.bone.convert_local_to_pose(
            desired,
            rest,
            parent_matrix=parent_pose,
            parent_matrix_local=parent_rest,
            invert=True,
        )
        matrices[bone.name] = desired
    return matrices


def retarget(mesh, rig, source, source_fps=24):
    """Transfer source armature-space motion onto a fitted standing reference."""
    map_target = mapping(mesh, rig)
    source_map = json.loads(
        source.get("ptc_semantics", json.dumps({role: role for role in PARENTS}))
    )
    if rig == source:
        raise ValueError("Use a separate reference armature.")
    source_data = source.animation_data
    source_tracks = action_tracks(source)
    references = {}
    for role, name in map_target.items():
        original = source.data.bones[source_map[role]]
        target = rig.data.bones[name]
        align = (target.tail_local - target.head_local).rotation_difference(
            original.tail_local - original.head_local
        )
        references[name] = align @ target.matrix_local.to_quaternion()
    scene = bpy.context.scene
    set_controls(rig, False)
    rig.data.pose_position = "POSE"
    target_fps = scene.render.fps / scene.render.fps_base
    if any(clip not in source_tracks for clip in CLIPS[:-1]):
        raise ValueError("Reference rig needs four named locomotion tracks.")
    previous_action, previous_slot = source_data.action, source_data.action_slot
    mutes = [track.mute for track in source_data.nla_tracks]
    try:
        for track in source_data.nla_tracks:
            track.mute = True
        for clip in CLIPS[:-1]:
            strip, original = source_tracks[clip]
            assign_action(source, original, strip.action_slot)
            start, end = original.frame_range
            frames = max(2, round((end - start) * target_fps / source_fps))
            action = new_action(rig, clip)
            for frame in range(frames + 1):
                sample = start + (end - start) * frame / frames
                scene.frame_set(math.floor(sample), subframe=sample % 1)
                rotations = {}
                for role, name in map_target.items():
                    source_bone = source.pose.bones[source_map[role]]
                    delta = (
                        source_bone.matrix.to_quaternion()
                        @ source_bone.bone.matrix_local.to_quaternion().inverted()
                    )
                    rotations[name] = delta @ references[name]
                pose_rotations(rig, rotations)
                bpy.context.view_layer.update()
                evaluated = mesh.evaluated_get(bpy.context.evaluated_depsgraph_get())
                sole = min(vertex.co.z for vertex in evaluated.data.vertices)
                hips = rig.pose.bones[map_target["Hips"]]
                hips.location += hips.bone.matrix_local.to_3x3().inverted() @ Vector((0, 0, -sole))
                key_pose(rig, frame)
            assign_action(rig, None)
            add_track(rig, clip, action)
    finally:
        assign_action(source, previous_action, previous_slot)
        for track, mute in zip(source_data.nla_tracks, mutes, strict=True):
            track.mute = mute


def aim(bone, direction):
    return (bone.tail_local - bone.head_local).rotation_difference(Vector(direction)) @ (
        bone.matrix_local.to_quaternion()
    )


def seat(mesh, rig):
    """Generate the reference Jeep pose, with editable held keys."""
    roles = mapping(mesh, rig)
    set_controls(rig, False)
    action = new_action(rig, "Seated")
    rotations = {}
    for side, sign in (("L", 1), ("R", -1)):
        for part, direction in (
            ("Thigh", (sign * 0.07, -0.75, 0.66)),
            ("Shin", (-sign * 0.07, -0.74, -0.67)),
            ("Foot", (sign * 0.12, -0.92, -0.36)),
            ("UpperArm", (sign * 0.11, -0.61, -0.82)),
            ("Forearm", (-sign * 0.18, -1, 0.46)),
            ("Hand", (-sign * 0.28, -0.77, 0.4)),
        ):
            name = roles[f"{part}.{side}"]
            rotations[name] = aim(rig.data.bones[name], direction)
    pose_rotations(rig, rotations)
    key_pose(rig, 0)
    key_pose(rig, 15)
    assign_action(rig, None)
    add_track(rig, "Seated", action)


def generate_controls(mesh, rig):
    """Fit a Rigify Basic Human metarig and bridge its ORG bones to the deform rig."""
    roles = mapping(mesh, rig)
    if rig.get("ptc_controls"):
        raise ValueError("A Rigify bridge already exists for this deform armature.")
    addon_utils.enable("rigify", default_set=True)
    bpy.ops.object.armature_basic_human_metarig_add()
    meta = bpy.context.object
    meta.name = "Character Metarig"
    bpy.ops.object.mode_set(mode="EDIT")
    edit = meta.data.edit_bones
    for role, target in RIGIFY.items():
        bone = rig.data.bones[roles[role]]
        edited = edit[target]
        edited.use_connect = False
        edited.head, edited.tail = bone.head_local, bone.tail_local
        edited.align_roll(bone.matrix_local.to_3x3().col[2])
    for main, extra in (("spine.002", "spine.003"), ("spine.004", "spine.005")):
        start, end = edit[main], edit[extra]
        end.use_connect = False
        end.tail = start.tail
        end.head = start.head.lerp(start.tail, 0.5)
        start.tail = end.head
        end.roll = start.roll
        end.use_connect = True
    for side in ("L", "R"):
        foot = edit[f"foot.{side}"]
        toe = edit[f"toe.{side}"]
        toe.use_connect = False
        toe.head, toe.tail = foot.tail, foot.tail + Vector((0, -0.07, 0))
        heel = edit[f"heel.02.{side}"]
        center = Vector((foot.head.x, foot.head.y + 0.04, 0))
        heel.head, heel.tail = center - Vector((0.05, 0, 0)), center + Vector((0.05, 0, 0))
        toe.use_connect = True
        for part in ("forearm", "hand", "shin", "foot"):
            edit[f"{part}.{side}"].use_connect = True
    for part in ("spine.001", "spine.002", "spine.003", "spine.005", "spine.006"):
        edit[part].use_connect = True
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.pose.rigify_generate()
    controls = meta.data.rigify_target_rig
    controls.name = "Character Rigify Controls"
    activate(controls)
    bpy.ops.object.mode_set(mode="EDIT")
    for role, target in RIGIFY.items():
        bone = rig.data.bones[roles[role]]
        bridge = controls.data.edit_bones.new(f"PTC-{role}")
        bridge.head, bridge.tail = bone.head_local, bone.tail_local
        bridge.align_roll(bone.matrix_local.to_3x3().col[2])
        bridge.parent = controls.data.edit_bones[f"ORG-{target}"]
        bridge.use_deform = False
    bpy.ops.object.mode_set(mode="OBJECT")
    helpers = controls.data.collections.new("PTC Bridge")
    for role in RIGIFY:
        helper = controls.data.bones[f"PTC-{role}"]
        for collection in list(helper.collections):
            collection.unassign(helper)
        helpers.assign(helper)
        constraint = rig.pose.bones[roles[role]].constraints.new("COPY_TRANSFORMS")
        constraint.name = "PTC Rigify Preview"
        constraint.target = controls
        constraint.subtarget = helper.name
        constraint.mute = True
    helpers.is_visible = False
    rig["ptc_controls"] = controls
    meta.hide_set(True)
    controls.hide_set(True)
    activate(rig)
    return controls


def set_controls(rig, enabled):
    controls = rig.get("ptc_controls")
    if enabled and not controls:
        raise ValueError("Generate a Rigify rig first.")
    for bone in rig.pose.bones:
        constraint = bone.constraints.get("PTC Rigify Preview")
        if constraint:
            constraint.mute = not enabled
    if controls:
        controls.hide_set(not enabled)
    if enabled:
        assign_action(rig, None)
        rig.data.pose_position = "POSE"
        activate(controls)


def bake_controls(rig, clip):
    """Sample the evaluated Rigify bridge over the scene playback range."""
    set_controls(rig, True)
    scene = bpy.context.scene
    samples = []
    for frame in range(scene.frame_start, scene.frame_end + 1):
        scene.frame_set(frame)
        evaluated = rig.evaluated_get(bpy.context.evaluated_depsgraph_get())
        samples.append({p.name: p.matrix.copy() for p in evaluated.pose.bones})
    if len(samples) < 2:
        raise ValueError("Set a playback range of at least two frames.")
    set_controls(rig, False)
    action = new_action(rig, clip)
    for frame, matrices in enumerate(samples):
        for p in rig.pose.bones:
            p.rotation_mode = "QUATERNION"
            kwargs = (
                dict(
                    parent_matrix=matrices[p.parent.name],
                    parent_matrix_local=p.parent.bone.matrix_local,
                )
                if p.parent
                else {}
            )
            p.matrix_basis = p.bone.convert_local_to_pose(
                matrices[p.name], p.bone.matrix_local, invert=True, **kwargs
            )
        key_pose(rig, frame)
    assign_action(rig, None)
    add_track(rig, clip, action)
    activate_preview(rig, clip)


def validate_glb(path):
    """Check the character structure, Action bindings, and embedding before publishing."""
    import struct

    with Path(path).open("rb") as stream:
        magic, version, length = struct.unpack("<4sII", stream.read(12))
        size, kind = struct.unpack("<I4s", stream.read(8))
        if (magic, version, kind, length) != (b"glTF", 2, b"JSON", Path(path).stat().st_size):
            raise ValueError("Invalid GLB container.")
        document = json.loads(stream.read(size))
        bin_size, bin_kind = struct.unpack("<I4s", stream.read(8))
        if bin_kind != b"BIN\0" or stream.tell() + bin_size != length:
            raise ValueError("GLB must contain a valid embedded BIN chunk.")
    skins = document.get("skins", [])
    meshes = [node for node in document["nodes"] if "mesh" in node]
    if len(skins) != 1 or len(meshes) != 1 or meshes[0].get("skin") != 0:
        raise ValueError("Export requires one skinned mesh node and one skin.")
    extras = meshes[0].get("extras", {})
    if extras.get("ptc_character_version") != 1:
        raise ValueError("Exported character contract version is missing.")
    roles = json.loads(extras.get("ptc_semantics", "{}"))
    joints = [document["nodes"][index]["name"] for index in skins[0]["joints"]]
    if set(roles) != set(PARENTS) or any(joints.count(name) != 1 for name in roles.values()):
        raise ValueError("Exported semantic mapping does not resolve uniquely.")
    clips = {clip["name"]: clip for clip in document.get("animations", [])}
    if set(clips) != set(CLIPS) or any(not clips[name]["channels"] for name in CLIPS):
        raise ValueError(f"Expected the five standard clips. Found {', '.join(clips)}.")
    if any("uri" in image or "bufferView" not in image for image in document.get("images", [])):
        raise ValueError("GLB textures must be embedded.")
    return document


def export_character(mesh, rig, path):
    """Validate a staged export before replacing the explicitly selected destination."""
    validate(mesh, rig)
    path = Path(path).resolve()
    if path.suffix.lower() != ".glb":
        raise ValueError("Select a .glb destination.")
    if path.parts[-4:] == ("assets", "models", "characters", "pilot.glb"):
        raise ValueError("Select a destination distinct from the original pilot.")
    path.parent.mkdir(parents=True, exist_ok=True)
    context = bpy.context
    scene = context.scene
    state = (scene.frame_current, scene.frame_subframe, rig.data.pose_position)
    data = rig.animation_data
    action, slot = data.action, data.action_slot
    constraints = [c for bone in rig.pose.bones for c in bone.constraints if not c.mute]
    tracks = [(t, t.mute, t.is_solo) for t in data.nla_tracks]
    if any(track.name not in CLIPS for track, _, _ in tracks):
        raise ValueError("Keep only standard named Action tracks on the runtime armature.")
    previous = context.view_layer.objects.active
    selected = list(context.selected_objects)
    try:
        for constraint in constraints:
            constraint.mute = True
        assign_action(rig, None)
        for track, _, _ in tracks:
            track.mute = False
            track.is_solo = False
        activate(rig)
        mesh.select_set(True)
        rig.data.pose_position = "POSE"
        with tempfile.TemporaryDirectory(dir=path.parent, prefix=".ptc-export-") as temporary:
            candidate = Path(temporary) / path.name
            result = bpy.ops.export_scene.gltf(
                filepath=str(candidate),
                export_format="GLB",
                use_selection=True,
                export_yup=True,
                export_extras=True,
                export_animations=True,
                export_animation_mode="NLA_TRACKS",
                export_force_sampling=True,
                export_def_bones=True,
                export_armature_object_remove=True,
                export_all_influences=False,
                export_morph=False,
                export_cameras=False,
                export_lights=False,
                export_tangents=False,
                export_optimize_animation_size=False,
            )
            if result != {"FINISHED"}:
                raise ValueError("Blender GLB export failed.")
            document = validate_glb(candidate)
            candidate.replace(path)
        return document
    finally:
        for constraint in constraints:
            constraint.mute = False
        for track, mute, solo in tracks:
            track.mute, track.is_solo = mute, solo
        assign_action(rig, action, slot)
        rig.data.pose_position = state[2]
        scene.frame_set(state[0], subframe=state[1])
        bpy.ops.object.select_all(action="DESELECT")
        for obj in selected:
            obj.select_set(True)
        context.view_layer.objects.active = previous


def data_ids():
    return {
        block
        for field in bpy.data.bl_rna.properties
        if field.type == "COLLECTION"
        for block in getattr(bpy.data, field.identifier)
        if isinstance(block, bpy.types.ID)
    }


@contextmanager
def reference_armature(path, rig_name="Pilot Deform"):
    before = data_ids()
    with bpy.data.libraries.load(str(path)) as (available, loaded):
        if rig_name not in available.objects:
            raise ValueError(f"Reference file does not contain {rig_name}.")
        loaded.objects = [rig_name]
    imported = data_ids() - before
    try:
        source = loaded.objects[0]
        if source is None or source.type != "ARMATURE":
            raise ValueError("The selected reference object must be an armature.")
        bpy.context.scene.collection.objects.link(source)
        source.hide_set(True)
        yield source
    finally:
        bpy.data.batch_remove(ids=imported)


def set_role(mesh, role, name):
    roles = json.loads(mesh.get("ptc_semantics", json.dumps({role: role for role in PARENTS})))
    roles[role] = name
    mesh["ptc_semantics"] = json.dumps(roles, sort_keys=True)


def deform_for(mesh):
    return next((m.object for m in mesh.modifiers if m.type == "ARMATURE" and m.object), None)


def mesh_object(_self, obj):
    return obj.type == "MESH"


def armature_object(_self, obj):
    return obj.type == "ARMATURE"


class PTCCharacterSettings(bpy.types.PropertyGroup):
    mesh: PointerProperty(type=bpy.types.Object, poll=mesh_object)
    rig: PointerProperty(type=bpy.types.Object, poll=armature_object)
    height: FloatProperty(name="Height (m)", min=0.1, default=1.80)
    yaw: FloatProperty(name="Yaw correction", subtype="ANGLE", default=0)
    clip: EnumProperty(name="Action", items=[(name, name, "") for name in CLIPS])
    role: EnumProperty(name="Semantic role", items=[(name, name, "") for name in PARENTS])
    bone: StringProperty(name="Deform bone")
    reference: StringProperty(
        name="Reference .blend", subtype="FILE_PATH", options={"PATH_SUPPORTS_BLEND_RELATIVE"}
    )
    reference_rig: StringProperty(name="Reference armature", default="Pilot Deform")
    reference_fps: FloatProperty(name="Reference FPS", default=24, min=1)
    output: StringProperty(
        name="Runtime GLB", subtype="FILE_PATH", options={"PATH_SUPPORTS_BLEND_RELATIVE"}
    )


class PTCCharacterCommand(bpy.types.Operator):
    bl_idname = "ptc.character_command"
    bl_label = "PTC Character Operation"
    bl_options = {"UNDO"}

    command: StringProperty()

    def execute(self, context):
        settings = context.scene.ptc_character
        mesh, rig = settings.mesh, settings.rig
        try:
            if self.command == "assign":
                if context.object is None or context.object.type not in {"MESH", "ARMATURE"}:
                    raise ValueError("Select a character mesh or deform armature.")
                if context.object.type == "MESH":
                    settings.mesh = context.object
                    settings.rig = deform_for(settings.mesh) or rig
                else:
                    settings.rig = context.object
                return {"FINISHED"}
            if self.command == "validate_glb":
                validate_glb(bpy.path.abspath(settings.output))
                self.report({"INFO"}, "Exported GLB structure and Action bindings validated.")
                return {"FINISHED"}
            if not mesh:
                raise ValueError("Assign a character mesh first.")
            if self.command == "inspect":
                self.report({"INFO"}, inspect(mesh))
                return {"FINISHED"}
            if self.command == "initialize":
                if deform_for(mesh) or mesh.vertex_groups:
                    raise ValueError("Initialize orientation and scale before binding.")
                initialize(mesh, settings.height, settings.yaw)
                return {"FINISHED"}
            if self.command == "create":
                if rig:
                    raise ValueError("Clear the deform armature field before creating another rig.")
                settings.rig = create_rig(mesh, settings.height)
                return {"FINISHED"}
            if not rig:
                raise ValueError("Create or assign the deform armature.")
            if context.object and context.object.mode != "OBJECT":
                bpy.ops.object.mode_set(mode="OBJECT")
            if self.command == "map":
                if settings.bone not in rig.data.bones:
                    raise ValueError("Select an existing deform bone.")
                set_role(mesh, settings.role, settings.bone)
            elif self.command == "bind":
                bind(mesh, rig)
            elif self.command == "rigify":
                generate_controls(mesh, rig)
            elif self.command == "controls":
                set_controls(rig, True)
            elif self.command in {"rest", "pose"}:
                rig.data.pose_position = self.command.upper()
            elif self.command in {"validate_skin", "validate"}:
                validate(mesh, rig, actions=self.command == "validate")
                self.report({"INFO"}, inspect(mesh))
            elif self.command == "retarget":
                if not settings.reference:
                    raise ValueError(
                        "Choose a reference .blend containing the standard locomotion."
                    )
                with reference_armature(
                    bpy.path.abspath(settings.reference), settings.reference_rig
                ) as source:
                    retarget(mesh, rig, source, settings.reference_fps)
                for name, (_, action) in action_tracks(rig).items():
                    if name in CLIPS[:-1]:
                        action.name = name
                activate_preview(rig, "Idle")
            elif self.command == "seat":
                seat(mesh, rig)
                activate_preview(rig, "Seated")
            elif self.command == "preview":
                activate_preview(rig, settings.clip)
            elif self.command == "bake":
                bake_controls(rig, settings.clip)
            elif self.command == "store":
                data = rig.animation_data
                if not data or not data.action or not data.action_slot:
                    raise ValueError(
                        "Assign a keyed Action to the deform rig in the Action Editor."
                    )
                add_track(rig, settings.clip, data.action, data.action_slot)
                activate_preview(rig, settings.clip)
            elif self.command == "pack":
                bpy.ops.file.pack_all()
            elif self.command in {"export", "scratch"}:
                if self.command == "export" and not settings.output:
                    raise ValueError("Choose the runtime GLB destination.")
                path = (
                    Path(tempfile.mkdtemp(prefix="ptc-character-")) / "scratch.glb"
                    if self.command == "scratch"
                    else Path(bpy.path.abspath(settings.output))
                )
                export_character(mesh, rig, path)
                self.report({"INFO"}, f"Export validated at {path}")
            else:
                raise ValueError(f"Unknown operation {self.command}")
            return {"FINISHED"}
        except (ValueError, KeyError, RuntimeError, OSError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}


class PTCCharacterPanel(bpy.types.Panel):
    bl_label = "PTC Character"
    bl_idname = "PTC_PT_character"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "PTC Character"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.ptc_character

        def button(row, command, text):
            row.operator(PTCCharacterCommand.bl_idname, text=text).command = command

        box = layout.box()
        box.prop(settings, "mesh")
        box.prop(settings, "rig")
        button(box, "assign", "Use Selected Object")
        box.prop(settings, "height")
        box.prop(settings, "yaw")
        button(box, "initialize", "Apply Scale, Yaw and Ground Origin")
        box = layout.box()
        button(box, "create", "Create A-pose Skeleton")
        box.label(text="Fit bone heads and tails in Edit Mode.")
        box.prop(settings, "role")
        if settings.rig:
            box.prop_search(settings, "bone", settings.rig.data, "bones")
        button(box, "map", "Assign Semantic Bone")
        row = box.row(align=True)
        button(row, "bind", "Bind")
        button(row, "validate_skin", "Check Weights")
        row = box.row(align=True)
        button(row, "rest", "Rest Position")
        button(row, "pose", "Pose Position")
        button(box, "rigify", "Generate Fitted Rigify Controls")
        button(box, "controls", "Use IK/FK Controls")
        box = layout.box()
        box.prop(settings, "reference")
        box.prop(settings, "reference_rig")
        box.prop(settings, "reference_fps")
        button(box, "retarget", "Retarget Locomotion")
        box.label(text="Check soles and refine planted steps.")
        button(box, "seat", "Create Jeep Seat Draft")
        box.prop(settings, "clip")
        row = box.row(align=True)
        button(row, "preview", "Preview")
        button(row, "store", "Store Active Action")
        button(box, "bake", "Bake Controls Over Scene Range")
        box = layout.box()
        button(box, "inspect", "Mesh and Material Summary")
        button(box, "pack", "Pack Resources")
        button(box, "validate", "Validate Character")
        box.prop(settings, "output")
        row = box.row(align=True)
        button(row, "export", "Export GLB")
        button(row, "scratch", "Scratch")
        button(box, "validate_glb", "Validate Exported GLB")


CLASSES = (PTCCharacterSettings, PTCCharacterCommand, PTCCharacterPanel)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ptc_character = PointerProperty(type=PTCCharacterSettings)


def unregister():
    del bpy.types.Scene.ptc_character
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
