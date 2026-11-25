"""Export authoritative character, vehicle, and environmental assets with Blender 5.0.1."""

import argparse
import sys
import tempfile
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.validate_environment_assets import (  # noqa: E402
    ENVIRONMENT_COLLECTIONS,
    ENVIRONMENT_RUNTIME,
    REQUIRED,
    validate_environment,
)
from tools.validate_ground_assets import ANCHORS, CLIPS, PARENTS, RUNTIME, WHEELS, validate  # noqa: E402

COLLECTIONS = {"pilot": ("Pilot Asset",), "jeep": ("Jeep Asset",), **ENVIRONMENT_COLLECTIONS}
OUTPUTS = {**RUNTIME, **ENVIRONMENT_RUNTIME}


def check_import(kind, path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(path), disable_bone_shape=True)
    if kind == "pilot":
        rigs = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
        if len(rigs) != 1 or set(rigs[0].data.bones.keys()) != PARENTS.keys():
            raise ValueError("Reimported pilot is missing its expected deform armature")
        bindings = {
            strip.action.name
            for track in rigs[0].animation_data.nla_tracks
            for strip in track.strips
            if strip.action and strip.action_slot
        }
        if set(bpy.data.actions.keys()) != CLIPS or bindings != CLIPS:
            raise ValueError("Reimported pilot has missing, unexpected, or unbound Actions")
        if not any(
            obj.type == "MESH"
            and any(mod.type == "ARMATURE" and mod.object == rigs[0] for mod in obj.modifiers)
            for obj in bpy.context.scene.objects
        ):
            raise ValueError("Reimported pilot mesh lost its armature modifier")
        for name, parent in PARENTS.items():
            bone = rigs[0].data.bones[name]
            if (bone.parent.name if bone.parent else None) != parent:
                raise ValueError(f"Reimported pilot changed the parent of {name}")
    elif kind in ENVIRONMENT_RUNTIME:
        for name in REQUIRED[kind]:
            obj = bpy.context.scene.objects.get(name)
            expected = "EMPTY" if name.startswith(("Anchor.", "Collision.")) else "MESH"
            if obj is None or obj.type != expected:
                raise ValueError(f"Reimported {kind}: {name} must be a {expected}")
        for node in validate_environment(kind, path).nodes:
            if node.name.startswith(("Anchor.", "Collision.")):
                obj = bpy.context.scene.objects[node.name]
                # glTF X/Y/Z maps to Blender X/Z/-Y.
                from mathutils import Matrix

                axes = Matrix(((1, 0, 0, 0), (0, 0, -1, 0), (0, 1, 0, 0), (0, 0, 0, 1)))
                restored = axes.inverted() @ obj.matrix_world @ axes
                if any(
                    abs(restored[r][c] - node.world_matrix[r, c]) > 1e-5
                    for r in range(4)
                    for c in range(4)
                ):
                    raise ValueError(f"Reimported {kind} changed the transform of {node.name}")
    else:
        for name, parent in [(name, "Jeep") for name in ANCHORS] + [
            pair
            for suffix in WHEELS
            for pair in ((f"Hub.{suffix}", "Jeep"), (f"Wheel.{suffix}", f"Hub.{suffix}"))
        ]:
            obj = bpy.data.objects.get(name)
            if not obj or not obj.parent or obj.parent.name != parent:
                raise ValueError(f"Reimported Jeep must parent {name} to {parent}")


def export(kind, output=None, check=False, blend_input=None):
    base = ROOT / "assets/blender"
    if kind in ENVIRONMENT_RUNTIME:
        base /= "environment"
    source = blend_input or base / f"{kind}.blend"
    bpy.ops.wm.open_mainfile(filepath=str(source.resolve()))
    output = (output or OUTPUTS[kind]).resolve()
    assets = ROOT / "assets"
    if output.suffix.lower() != ".glb" or (
        output.is_relative_to(assets) and output != OUTPUTS[kind]
    ):
        raise ValueError(f"Output must be {OUTPUTS[kind]} or a GLB outside the repository assets")
    bpy.ops.object.select_all(action="DESELECT")
    export_objects = set()
    allowed = {"MESH", "EMPTY"} if kind in ENVIRONMENT_RUNTIME else {"MESH", "ARMATURE", "EMPTY"}
    for name in COLLECTIONS[kind]:
        collection = bpy.data.collections.get(name)
        if collection is None:
            raise ValueError(f"Missing export collection {name}")
        export_objects.update(
            obj
            for obj in collection.all_objects
            if obj.type in allowed
            and not (obj.type == "EMPTY" and obj.empty_display_type == "IMAGE")
        )
    for obj in export_objects:
        if obj.parent is not None and obj.parent not in export_objects:
            raise ValueError(f"{obj.name} has a parent outside {COLLECTIONS[kind]}")
        obj.select_set(True)
    if kind == "pilot":
        rig = bpy.data.objects.get("Pilot Deform")
        if rig is None or rig not in export_objects:
            raise ValueError("Pilot Asset must contain Pilot Deform")
        if rig.get("rigify_preview", 0) != 0:
            raise ValueError("Bake Rigify motion onto Pilot Deform and turn off rigify_preview")
        for bone in rig.pose.bones:
            for constraint in list(bone.constraints):
                if constraint.name == "Rigify limb preview":
                    bone.constraints.remove(constraint)
        for obj in list(bpy.context.scene.objects):
            if obj.type == "ARMATURE" and obj not in export_objects:
                bpy.data.objects.remove(obj, do_unlink=True)
        bpy.context.view_layer.update()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{kind}-export-", dir=output.parent) as folder:
        candidate = Path(folder) / output.name
        result = bpy.ops.export_scene.gltf(
            filepath=str(candidate),
            export_format="GLB",
            use_selection=True,
            export_yup=True,
            export_animations=kind == "pilot",
            export_animation_mode="ACTIONS",
            export_force_sampling=False,
            export_armature_object_remove=True,
            export_all_influences=False,
            export_morph=False,
            export_cameras=False,
            export_lights=False,
        )
        if result != {"FINISHED"}:
            raise ValueError(f"Blender did not finish exporting {kind}")
        if kind in ENVIRONMENT_RUNTIME:
            validate_environment(kind, candidate)
            check_import(kind, candidate)
        else:
            validate(kind, candidate)
        if check:
            if kind not in ENVIRONMENT_RUNTIME:
                check_import(kind, candidate)
            if output.read_bytes() != candidate.read_bytes():
                raise ValueError(f"Fresh {kind} export differs from {output}")
            print(f"{kind}: runtime validation, Blender reimport and byte comparison passed")
        else:
            candidate.replace(output)
            print(f"{kind}: validated and exported to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", choices=OUTPUTS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--blend-input", type=Path, help="Export a working copy of the source")
    parser.add_argument(
        "--check", action="store_true", help="Reimport and compare without overwriting"
    )
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    export(args.asset, args.output, args.check, args.blend_input)
