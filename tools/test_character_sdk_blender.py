"""Exercise the blank-file SDK, production Rigify bridge, and reproducible GLB export."""

import argparse
import hashlib
import sys
import tempfile
from pathlib import Path

import addon_utils
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from author_character import PRODUCTION, RUNTIME, SOURCE  # noqa: E402
from ptc_character_sdk import (  # noqa: E402
    CLIPS,
    action_tracks,
    activate,
    activate_preview,
    bake_controls,
    deform_for,
    export_character,
    set_controls,
    validate,
    validate_glb,
)


def fingerprint(mesh):
    """Compare geometry, per-corner UVs, and all user-facing material inputs."""
    material = mesh.data.materials[0]
    images = {
        node.image.name: hashlib.sha256(node.image.packed_file.data).hexdigest()
        for node in material.node_tree.nodes
        if node.type == "TEX_IMAGE" and node.image
    }
    return (
        len(mesh.data.vertices),
        tuple(tuple(face.vertices) for face in mesh.data.polygons),
        tuple(tuple(loop.uv) for loop in mesh.data.uv_layers.active.data),
        images,
        tuple(
            sorted(
                (link.from_node.name, link.from_socket.name, link.to_socket.name)
                for link in material.node_tree.links
            )
        ),
        tuple(face.material_index for face in mesh.data.polygons),
    )


def check_blank_file():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import ptc_character_sdk as sdk

    sdk.register()
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 1))
    mesh = bpy.context.object
    settings = bpy.context.scene.ptc_character
    execute = bpy.ops.ptc.character_command
    assert execute(command="assign") == {"FINISHED"}
    assert settings.mesh is mesh
    assert execute(command="initialize") == {"FINISHED"}
    assert min(v.co.z for v in mesh.data.vertices) >= -1e-6
    assert abs(mesh.dimensions.z - 1.8) < 1e-6
    assert execute(command="create") == {"FINISHED"}
    rig = settings.rig
    assert rig and len(rig.data.bones) == 19
    assert execute(command="bind") == {"FINISHED"}
    assert execute(command="validate_skin") == {"FINISHED"}
    assert execute(command="rest") == {"FINISHED"} and rig.data.pose_position == "REST"
    assert execute(command="pose") == {"FINISHED"} and rig.data.pose_position == "POSE"
    rig_group = mesh.vertex_groups[0]
    rig_group.add([0], 0.1, "REPLACE")
    try:
        validate(mesh, rig, actions=False)
    except ValueError as error:
        assert "invalid vertices selected" in str(error) and mesh.data.vertices[0].select
    else:
        raise AssertionError("Invalid weight passed Blender validation")
    sdk.unregister()
    print("Blank-file sidebar assignment, scale, setup, bind, validation and Rest/Pose passed")


def check_bridge(mesh, rig):
    controls = rig["ptc_controls"]

    def positions():
        bpy.context.view_layer.update()
        return {name: rig.pose.bones[name].head.copy() for name in ("Hand.L", "Hand.R", "Foot.R")}

    activate_preview(rig, "Idle")
    rig.data.pose_position = "REST"
    rest = positions()
    set_controls(rig, True)
    neutral = positions()
    for name in rest:
        assert (neutral[name] - rest[name]).length < 0.003, (name, neutral[name], rest[name])

    for name, offset in (("hand_ik.L", (0.03, -0.17, 0.17)), ("foot_ik.R", (0.01, -0.13, 0.14))):
        bone = controls.pose.bones[name]
        matrix = bone.matrix.copy()
        matrix.translation += Vector(offset)
        bone.matrix = matrix
    controls.update_tag()
    moved = positions()
    assert (moved["Hand.L"] - rest["Hand.L"]).length > 0.10
    assert (moved["Foot.R"] - rest["Foot.R"]).length > 0.10

    controls.pose.bones["upper_arm_parent.R"]["IK_FK"] = 1.0
    fk = controls.pose.bones["upper_arm_fk.R"]
    fk.rotation_mode = "XYZ"
    fk.rotation_euler.x = 0.5
    controls.update_tag()
    assert (positions()["Hand.R"] - rest["Hand.R"]).length > 0.1
    evaluated = mesh.evaluated_get(bpy.context.evaluated_depsgraph_get())
    assert (
        max((a.co - b.co).length for a, b in zip(mesh.data.vertices, evaluated.data.vertices)) > 0.1
    )

    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 0, 2
    bake_controls(rig, "Turn")
    assert action_tracks(rig)["Turn"][1].frame_range[1] > 0
    baked = positions()
    for name in moved:
        if name != "Hand.R":
            assert (baked[name] - moved[name]).length < 0.003
    rig.data.pose_position = "REST"
    assert all((positions()[name] - rest[name]).length < 0.003 for name in rest)
    print("Female Rigify arm and leg IK, arm FK, deform bake, and neutral restore passed")


def render_reference(mesh, rig, path):
    activate_preview(rig, "Idle")
    scene = bpy.context.scene
    bpy.ops.object.camera_add(location=(1.1, -3.1, 1.6))
    camera = bpy.context.object
    camera.rotation_euler = (
        (Vector((0, 0, 0.91)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    )
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 2.16
    scene.camera = camera
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "TEXTURE"
    scene.display.shading.light = "FLAT"
    scene.display.shading.show_shadows = False
    scene.display.shading.show_cavity = False
    scene.view_settings.view_transform = "Standard"
    scene.render.resolution_x = 640
    scene.render.resolution_y = 640
    scene.render.resolution_percentage = 100
    scene.render.filepath = str(path.resolve())
    material = mesh.data.materials[0]
    material.node_tree.nodes.active = next(
        link.from_node for link in material.node_tree.links if link.to_socket.name == "Base Color"
    )
    activate(mesh)
    bpy.ops.render.render(write_still=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    check_blank_file()
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
    source = fingerprint(bpy.data.objects["Mesh0"])
    addon_utils.enable("rigify", default_set=True)
    bpy.ops.wm.open_mainfile(filepath=str(PRODUCTION))
    mesh = next(obj for obj in bpy.context.scene.objects if obj.get("ptc_character_version") == 1)
    rig = deform_for(mesh)
    assert fingerprint(mesh) == source
    validate(mesh, rig)
    assert {action.name for _, action in action_tracks(rig).values()} == set(CLIPS)
    assert abs(min(v.co.z for v in mesh.data.vertices)) < 1e-6
    assert max(v.co.z for v in mesh.data.vertices) < 1.80001

    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / RUNTIME.name
        document = export_character(mesh, rig, path)
        assert path.read_bytes() == RUNTIME.read_bytes(), "Production export bytes differ"
        assert len(document.get("images", [])) == 3  # Base, normal, combined metallic/roughness.
        assert all("PTC-" not in node.get("name", "") for node in document["nodes"])
        assert validate_glb(path)["skins"] == document["skins"]
    print("Source polygons, UVs, packed textures and links preserved. GLB export is reproducible.")
    if args.render:
        render_reference(mesh, rig, args.render)
    check_bridge(mesh, rig)


if __name__ == "__main__":
    main()
