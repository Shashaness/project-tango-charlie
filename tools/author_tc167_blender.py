"""Create editable Blender 5 rigid-node BD Actions and export them over pristine TC167 rest data.

Run with Blender --background --python tools/author_tc167_blender.py -- [options].
Blender supplies bpy/mathutils/NumPy; the game engine is not modified by this tool.
"""

import argparse
import copy
import hashlib
import json
import math
import struct
import sys
import tempfile
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

PROJECT = Path(__file__).resolve().parents[1]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from engine.gltf_loader import Accessors, load_glb, read_glb  # noqa: E402 (Blender needs the project path bootstrap)
from game.flight_state import VehicleMode  # noqa: E402 (Blender needs the project path bootstrap)
from game.tc167_poses import BATTLEDROID_POSE, rotation  # noqa: E402 (Blender needs the project path bootstrap)
from game.transformation import TransformationController  # noqa: E402 (Blender needs the project path bootstrap)

SOURCE_GLB = PROJECT / "assets/models/vehicles/tc167.glb"
SOURCE_BLEND = PROJECT / "assets/blender/TC167.blend"
CLIPS = {"BD_Idle": 49, "BD_Walk": 25, "BD_Run": 19}
# glTF (X,Y,Z) -> Blender (X,-Z,Y); quaternion components also change order.
CONVERSION = Matrix(((1, 0, 0, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))


def fingerprint(objects):
    """Hash mesh topology/coordinates and retain hierarchy/pivots/material assignments."""
    result = {}
    for obj in objects:
        geometry = None
        if obj.type == "MESH":
            data = {
                "vertices": [list(v.co) for v in obj.data.vertices],
                "faces": [list(p.vertices) for p in obj.data.polygons],
                "material_indices": [p.material_index for p in obj.data.polygons],
                "materials": [m.name if m else None for m in obj.data.materials],
                "uvs": [[list(v.uv) for v in layer.data] for layer in obj.data.uv_layers],
            }
            geometry = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
        result[obj.name] = {
            "type": obj.type,
            "parent": obj.parent.name if obj.parent else None,
            "parent_inverse": [list(row) for row in obj.matrix_parent_inverse],
            "geometry": geometry,
        }
    return result


def select_action(name, names):
    """Assign each joint's slot in one shared Action without blending archived NLA tracks."""
    action = bpy.data.actions[name]
    for name in names:
        obj = bpy.data.objects[name]
        obj.animation_data_create()
        slots = [s for s in action.slots if s.identifier in (name, "OB" + name)]
        obj.animation_data.action = action if slots else None
        if slots:
            obj.animation_data.action_slot = slots[0]
        obj.animation_data.use_nla = False
    bpy.context.scene.frame_start = int(action.frame_start)
    bpy.context.scene.frame_end = int(action.frame_end)
    bpy.context.scene.frame_set(int(action.frame_start))


def local_to_basis(obj, local):
    """Compensate the original parent inverse; do not change pivots or apply mesh transforms."""
    blender_local = CONVERSION @ Matrix(local) @ CONVERSION.inverted()
    return obj.matrix_parent_inverse.inverted() @ blender_local


def gait_pose(controller, name, progress):
    """Seed a heavy in-place gait as absolute glTF-local poses; keys become independent data."""
    controller.reset(VehicleMode.BATTLEDROID)
    model = controller.model
    if name == "BD_Idle":
        wave = math.sin(math.tau * progress)
        model.find_node("TorsoCore").set_pose((0, 0, 0.012 * wave), rotation(z=0.45 * wave))
        model.find_node("Head").set_pose((0, -0.05, 2.6), rotation(z=-0.35 * wave, x=0.2 * wave))
        for side, sign in (("Left", 1), ("Right", -1)):
            base = BATTLEDROID_POSE[side + "Shoulder"]
            model.find_node(side + "Shoulder").set_pose(
                base.translation, rotation(z=sign * 0.3 * wave), base.scale
            )
    else:
        run = name == "BD_Run"
        duty, stride, lift, compression = (
            (0.42, 1.89, 0.40, 0.28) if run else (0.60, 1.20, 0.23, 0.13)
        )
        lean = 6.0 if run else 2.0
        model.find_node("Fuselage").set_pose((0, 1.3, 0), rotation(x=-90 + lean))
        counter = (2.0 if run else 1.0) * math.sin(math.tau * progress)
        model.find_node("TorsoCore").set_pose((0, 0, 0), rotation(z=counter))
        model.find_node("Head").set_pose((0, -0.05, 2.6), rotation(x=-lean, z=-counter * 0.35))
        for side, offset, sign in (("Left", 0, 1), ("Right", 0.5, -1)):
            phase = (progress + offset) % 1.0
            if phase < duty:
                forward = stride * (0.5 - phase / duty)
                height = 0.0
            else:
                u = (phase - duty) / (1 - duty)
                smooth = u * u * (3 - 2 * u)
                forward = stride * (smooth - 0.5)
                height = lift * math.sin(math.pi * u) ** 2
            # Two 2.6 m links: solve hip/knee in their local Y/Z plane, with
            # a small suspension compression providing stance reach clearance.
            y = -5.2 * math.cos(math.radians(4)) + compression + height
            cosine = float(np.clip((y * y + forward * forward - 2 * 2.6**2) / (2 * 2.6**2), -1, 1))
            knee = math.acos(cosine)
            hip = math.atan2(y, -forward) - knee / 2
            ankle = -math.pi - hip - knee
            model.find_node(side + "UpperLeg").set_pose(
                (0, -compression, 0), rotation(x=math.degrees(hip))
            )
            model.find_node(side + "LowerLeg").set_pose(
                (0, 0, -2.6), rotation(x=math.degrees(knee))
            )
            model.find_node(side + "Foot").set_pose((0, 0, -2.6), rotation(x=math.degrees(ankle)))
            arm = (22 if run else 12) * (forward / (stride / 2))
            base = BATTLEDROID_POSE[side + "UpperArm"]
            model.find_node(side + "UpperArm").set_pose(
                base.translation, rotation(x=-90 + arm, y=-sign * 12), base.scale
            )
            base = BATTLEDROID_POSE[side + "Forearm"]
            model.find_node(side + "Forearm").set_pose(
                base.translation, rotation(x=-20 - (5 if run else 2) * abs(arm) / 22), base.scale
            )
    model.world_matrices()
    # Engines are siblings of the leg joints. Bake their foot-relative attachment
    # into real object keys so Blender and the native player show the same machine.
    for side in ("Left", "Right"):
        engine = model.find_node(side + "Engine")
        desired = (
            np.linalg.inv(engine.parent.world_matrix)
            @ model.find_node(side + "Foot").world_matrix
            @ controller.authoring_engine_offsets[side]
        )
        location, quaternion, scale = Matrix(desired.tolist()).decompose()
        engine.set_pose(
            tuple(location), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), tuple(scale)
        )
    return {node.name: node.configuration_matrix for node in model.nodes}


def key_basis(obj, basis, frame):
    """Insert real local TRS keys without changing the original parent inverse."""
    location, quaternion, scale = basis.decompose()
    rebuilt = Matrix.LocRotScale(location, quaternion, scale)
    if np.max(np.abs(np.asarray(basis) - np.asarray(rebuilt))) > 2e-5:
        raise ValueError(f"{obj.name}: parent-inverse compensation introduced unsupported shear")
    obj.rotation_mode = "QUATERNION"
    # Keep quaternion signs continuous for usable curves in Blender's Graph Editor.
    if obj.rotation_quaternion.dot(quaternion) < 0:
        quaternion.negate()
    obj.location, obj.rotation_quaternion, obj.scale = location, quaternion, scale
    for path in ("location", "rotation_quaternion", "scale"):
        obj.keyframe_insert(data_path=path, frame=frame, group="Local TRS")


def make_action(name, end, names):
    """Create one Blender 5 slotted Action shared by the preserved rigid objects."""
    if name in bpy.data.actions:
        raise ValueError(f"Refusing to replace existing artist Action {name}")
    action = bpy.data.actions.new(name)
    action.use_fake_user = True
    action.use_frame_range = True
    action.frame_start = 0 if name == "FIGHTER_Rest" else 1
    action.frame_end = end
    for name in names:
        obj = bpy.data.objects[name]
        obj.animation_data_create()
        obj.animation_data.use_nla = False
        obj.animation_data.action = action
        obj.animation_data.action_slot = action.slots.new(id_type="OBJECT", name=name)
    return action


def install_controls(names):
    """Embed a saved-file helper panel for switching every object slot together."""
    source = """import bpy
NAMES = __NAMES__

def choose(name):
    action = bpy.data.actions[name]
    for name in NAMES:
        obj = bpy.data.objects[name]
        obj.animation_data_create()
        slots = [s for s in action.slots if s.identifier in (name, 'OB' + name)]
        obj.animation_data.action = action if slots else None
        if slots: obj.animation_data.action_slot = slots[0]
        obj.animation_data.use_nla = False
    bpy.context.scene.frame_start = int(action.frame_start)
    bpy.context.scene.frame_end = int(action.frame_end)
    bpy.context.scene.frame_set(int(action.frame_start))

class TC167_OT_clip(bpy.types.Operator):
    bl_idname = 'tc167.clip'
    bl_label = 'Select TC167 Action'
    action_name: bpy.props.StringProperty()
    def execute(self, context):
        choose(self.action_name)
        return {'FINISHED'}

class TC167_PT_actions(bpy.types.Panel):
    bl_label = 'TC167 Actions'
    bl_idname = 'TC167_PT_actions'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'TC167'
    def draw(self, context):
        for name in ('BD_Idle', 'BD_Walk', 'BD_Run', 'FIGHTER_Rest'):
            self.layout.operator('tc167.clip', text=name).action_name = name

for cls in (TC167_OT_clip, TC167_PT_actions):
    old = getattr(bpy.types, cls.__name__, None)
    if old: bpy.utils.unregister_class(old)
    bpy.utils.register_class(cls)
""".replace("__NAMES__", repr(names))
    bpy.data.texts.new("TC167_Action_Controls.py").write(source)
    bpy.data.texts.new("TC167_README").write(
        "Open in BATTLEDROID at BD_Idle frame 1. 24 FPS.\n"
        "BD_Idle: 1-49 (2 s); BD_Walk: 1-25 (1 s); BD_Run: 1-19 (0.75 s).\n"
        "Run TC167_Action_Controls.py in the Text Editor once to add N-sidebar TC167 buttons.\n"
        "Each button assigns every rigid object slot to the selected shared Action.\n"
        "FIGHTER_Rest restores the production local transforms at frame 0.\n"
        "Edit real Location / Quaternion / Scale keys in Action Editor / Graph Editor.\n"
        "Keep node names, parents, origins, parent inverses, mesh data, and materials.\n"
        "No armature or root displacement. Ground is Z=-3.2 m in this COM-based authoring frame.\n"
        "Save edits, then use the documented --export-only command for a rest-safe game GLB.\n"
    )


def create_actions(controller):
    """Build and archive idle/walk/run and a separate real FIGHTER restoration Action."""
    # Blender drops unreferenced datablocks when saving. Retain even unused source
    # meshes/materials in the authoring copy; no shader or geometry data is changed.
    for datablock in list(bpy.data.materials) + list(bpy.data.meshes):
        datablock.use_fake_user = True
    names = sorted(set(BATTLEDROID_POSE) | {"LeftEngine", "RightEngine"})
    model_names = [n.name for n in controller.model.nodes]
    basis = {name: bpy.data.objects[name].matrix_basis.copy() for name in model_names}
    bpy.data.texts.new("TC167_FIGHTER_BASIS.json").write(
        json.dumps({n: [list(row) for row in m] for n, m in basis.items()})
    )
    make_action("FIGHTER_Rest", 1, model_names)
    for name in model_names:
        key_basis(bpy.data.objects[name], basis[name], 0)
    for obj in (bpy.data.objects[n] for n in model_names):
        obj.animation_data.action = None
    for name, end in CLIPS.items():
        action = make_action(name, end, names)
        for frame in range(1, end + 1):
            progress = 0.0 if frame == end else (frame - 1) / (end - 1)
            poses = gait_pose(controller, name, progress)
            for node in names:
                key_basis(
                    bpy.data.objects[node],
                    local_to_basis(bpy.data.objects[node], poses[node].tolist()),
                    frame,
                )
        for layer in action.layers:
            for strip in layer.strips:
                for bag in strip.channelbags:
                    for curve in bag.fcurves:
                        for point in curve.keyframe_points:
                            point.interpolation = "LINEAR"
                        curve.modifiers.new("CYCLES")
        for node in names:
            obj = bpy.data.objects[node]
            track = obj.animation_data.nla_tracks.new()
            track.name = name
            track.mute = True
            strip = track.strips.new(name, 1, action)
            strip.action_slot = obj.animation_data.action_slot
            obj.animation_data.action = None
    install_controls(model_names)
    bpy.context.scene.render.fps = 24
    bpy.context.scene.render.fps_base = 1.0
    select_action("BD_Idle", model_names)
    bpy.ops.object.select_all(action="DESELECT")
    obj = bpy.data.objects["TorsoCore"]
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D":
                area.spaces.active.region_3d.view_location = Vector((0, 0, 0.5))
                area.spaces.active.region_3d.view_distance = 14
                area.spaces.active.region_3d.view_rotation = Vector((8, -12, 6)).to_track_quat(
                    "Z", "Y"
                )
                area.spaces.active.region_3d.view_perspective = "ORTHO"
            elif area.type == "DOPESHEET_EDITOR":
                area.ui_type = "DOPESHEET"
                area.spaces.active.mode = "ACTION"
    return model_names


def write_glb(path, document, binary):
    """Write padded JSON/BIN chunks without altering the original geometry buffer."""
    document["buffers"][0]["byteLength"] = len(binary)
    encoded = json.dumps(document, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    binary += b"\0" * (-len(binary) % 4)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 28 + len(encoded) + len(binary))
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
        + struct.pack("<II", len(binary), 0x004E4942)
        + binary
    )


def export_actions(output):
    """Export actual Blender curves, copying only animation buffers onto the pristine GLB."""
    original, binary = read_glb(SOURCE_GLB)
    baseline = json.loads(bpy.data.texts["TC167_GEOMETRY_BASELINE.json"].as_string())
    current = fingerprint([bpy.data.objects[name] for name in baseline])
    if current != baseline:
        raise ValueError(
            "Mesh data, materials, hierarchy, or parent inverses changed; "
            "export only animation edits"
        )
    indices = {n["name"]: i for i, n in enumerate(original["nodes"])}
    bases = json.loads(bpy.data.texts["TC167_FIGHTER_BASIS.json"].as_string())
    saved = {
        n: (
            bpy.data.objects[n].animation_data.action,
            bpy.data.objects[n].animation_data.action_slot,
        )
        for n in indices
    }
    # The Blender exporter gets the genuine FIGHTER bind pose even though the saved
    # authoring file opens with an active BATTLEDROID Action.
    for name, basis in bases.items():
        obj = bpy.data.objects[name]
        obj.animation_data.action = None
        obj.animation_data.use_nla = False
        obj.matrix_basis = Matrix(basis)
    bpy.context.scene.frame_set(0)
    bpy.context.view_layer.update()
    bpy.ops.object.select_all(action="DESELECT")
    for name in indices:
        bpy.data.objects[name].select_set(True)
    with tempfile.TemporaryDirectory(prefix="tc167-animation-export-") as directory:
        raw = Path(directory) / "blender.glb"
        bpy.ops.export_scene.gltf(
            filepath=str(raw),
            export_format="GLB",
            use_selection=True,
            export_animations=True,
            export_animation_mode="ACTIONS",
            export_merge_animation="ACTION",
            export_force_sampling=True,
            export_frame_step=1,
            export_frame_range=False,
            export_anim_slide_to_zero=True,
            export_current_frame=False,
            export_optimize_animation_keep_anim_object=True,
            export_skins=False,
            export_morph=False,
            export_cameras=False,
            export_lights=False,
            export_hierarchy_flatten_objs=False,
            export_yup=True,
            export_materials="EXPORT",
        )
        exported, data = read_glb(raw)
        decoder = Accessors(exported, data)
        original["animations"] = []
        for clip in exported.get("animations", []):
            if clip.get("name") not in CLIPS:
                continue
            animation = copy.deepcopy(clip)
            for channel in animation["channels"]:
                name = exported["nodes"][channel["target"]["node"]]["name"]
                channel["target"]["node"] = indices[name]
            for sampler in animation["samplers"]:
                for field in ("input", "output"):
                    accessor = exported["accessors"][sampler[field]]
                    kind = accessor["type"]
                    values = np.asarray(decoder.decode(sampler[field], kind, (5126,)), dtype="<f4")
                    binary += b"\0" * (-len(binary) % 4)
                    view = len(original["bufferViews"])
                    original["bufferViews"].append(
                        {"buffer": 0, "byteOffset": len(binary), "byteLength": values.nbytes}
                    )
                    index = len(original["accessors"])
                    original["accessors"].append(
                        {
                            "bufferView": view,
                            "componentType": 5126,
                            "count": len(values),
                            "type": kind,
                        }
                    )
                    binary += values.tobytes()
                    sampler[field] = index
            original["animations"].append(animation)
        if {a["name"] for a in original["animations"]} != set(CLIPS):
            raise ValueError("Blender export did not contain all three BD Actions")
        write_glb(output, original, binary)
    for name, (action, slot) in saved.items():
        obj = bpy.data.objects[name]
        obj.animation_data.action = action
        if action:
            obj.animation_data.action_slot = slot
    select_action("BD_Idle", list(indices))
    return load_glb(output)


def main():
    """Create a protected authoring copy, or export saved artist edits without regenerating keys."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blend-input", type=Path, default=SOURCE_BLEND)
    parser.add_argument(
        "--blend-output", type=Path, default=PROJECT / "assets/blender/TC167_Animated.blend"
    )
    parser.add_argument(
        "--glb-output", type=Path, default=PROJECT / "assets/models/vehicles/tc167_animated.glb"
    )
    parser.add_argument("--export-only", action="store_true")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    if args.glb_output.resolve() == SOURCE_GLB.resolve() or (
        not args.export_only
        and args.blend_output.resolve() in (SOURCE_BLEND.resolve(), args.blend_input.resolve())
    ):
        parser.error("Original model files must not be overwritten")
    if not args.export_only and args.blend_output.exists():
        parser.error("Authoring file already exists; use --export-only or choose a new output")
    bpy.ops.wm.open_mainfile(filepath=str(args.blend_input.resolve()))
    before = fingerprint(list(bpy.data.objects))
    model = load_glb(SOURCE_GLB)
    controller = TransformationController(model=model)
    for node in model.nodes:
        obj = bpy.data.objects[node.name]
        parent = obj.parent.name if obj.parent else None
        if parent != (node.parent.name if node.parent else None):
            raise ValueError(f"{node.name}: hierarchy differs from the production GLB")
        if not args.export_only:
            expected = CONVERSION @ Matrix(node.base_matrix.tolist()) @ CONVERSION.inverted()
            if np.max(np.abs(np.asarray(obj.matrix_local) - np.asarray(expected))) > 2e-5:
                raise ValueError(f"{node.name}: FIGHTER local matrix differs from production")
    if not args.export_only:
        bpy.data.texts.new("TC167_GEOMETRY_BASELINE.json").write(json.dumps(before))
        controller.reset(VehicleMode.BATTLEDROID)
        controller.authoring_engine_offsets = {
            side: np.linalg.inv(model.find_node(side + "Foot").world_matrix)
            @ model.find_node(side + "Engine").world_matrix
            for side in ("Left", "Right")
        }
        create_actions(controller)
        if fingerprint(list(bpy.data.objects)) != before:
            raise ValueError("Authoring altered geometry, hierarchy, parent inverses, or materials")
    args.glb_output.parent.mkdir(parents=True, exist_ok=True)
    animated = export_actions(args.glb_output)
    if not args.export_only:
        args.blend_output.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.blend_output.resolve()))
    print(
        "TC167 M21.1:", {n: (c.duration, len(c.channels)) for n, c in animated.animations.items()}
    )
    print(
        "Preserved geometry/hierarchy fingerprint:", before == fingerprint(list(bpy.data.objects))
    )


if __name__ == "__main__":
    main()
