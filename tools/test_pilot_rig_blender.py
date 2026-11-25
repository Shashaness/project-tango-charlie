"""Exercise the pilot's actual Rigify-to-deform link in Blender 5.0.1."""

import argparse
import sys
from pathlib import Path

import addon_utils
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]


def main(render_path=None):
    addon_utils.enable("rigify", default_set=True)
    assert addon_utils.check("rigify")[1], "Enable the installed Rigify add-on"
    bpy.ops.wm.open_mainfile(filepath=str(ROOT / "assets/blender/pilot.blend"))
    meta = bpy.data.objects["Pilot Metarig"]
    controls = bpy.data.objects["Pilot Rigify Controls"]
    deform = bpy.data.objects["Pilot Deform"]
    mesh = bpy.data.objects["PilotMesh"]
    assert meta.pose.bones["upper_arm.L"].rigify_type == "limbs.arm"
    assert meta.pose.bones["thigh.R"].rigify_type == "limbs.leg"
    assert len(deform.data.bones) == 19 < len(controls.data.bones)
    for end, start in (("Forearm.L", "UpperArm.L"), ("Shin.R", "Thigh.R")):
        assert (
            deform.data.bones[end].head_local - deform.data.bones[start].tail_local
        ).length < 1e-6
    controls.hide_set(False)
    names = ("Hand.L", "Hand.R", "Foot.R", "Forearm.L", "Shin.R")

    def positions():
        bpy.context.view_layer.update()
        return {name: deform.pose.bones[name].head.copy() for name in names}

    rest = positions()
    deform["rigify_preview"] = 1.0
    deform.update_tag()
    neutral = positions()
    assert all((neutral[name] - rest[name]).length < 1e-4 for name in names)
    for name, offset in (("hand_ik.L", (0.02, -0.15, 0.20)), ("foot_ik.R", (0.01, -0.14, 0.14))):
        bone = controls.pose.bones[name]
        matrix = bone.matrix.copy()
        matrix.translation += Vector(offset)
        bone.matrix = matrix
        controls.update_tag()
        positions()
    ik = positions()
    assert (ik["Hand.L"] - rest["Hand.L"]).length > 0.15
    assert (ik["Foot.R"] - rest["Foot.R"]).length > 0.10
    p = controls.pose.bones["upper_arm_parent.R"]
    p["IK_FK"] = 1.0
    fk = controls.pose.bones["upper_arm_fk.R"]
    fk.rotation_mode = "XYZ"
    fk.rotation_euler.x = 0.5
    controls.update_tag()
    posed = positions()
    assert (posed["Hand.R"] - rest["Hand.R"]).length > 0.10
    for end, start in (("Hand.L", "Forearm.L"), ("Foot.R", "Shin.R")):
        expected = (rest[end] - rest[start]).length
        assert abs((posed[end] - posed[start]).length - expected) < 1e-5
    evaluated = mesh.evaluated_get(bpy.context.evaluated_depsgraph_get())
    assert len(evaluated.data.vertices) == len(mesh.data.vertices)
    assert (
        max((a.co - b.co).length for a, b in zip(mesh.data.vertices, evaluated.data.vertices))
        > 0.15
    )

    if render_path:
        bpy.ops.object.camera_add(location=(2.7, -4.1, 1.9))
        camera = bpy.context.object
        camera.rotation_euler = (
            (Vector((0, 0, 0.93)) - camera.location).to_track_quat("-Z", "Y").to_euler()
        )
        scene = bpy.context.scene
        scene.camera = camera
        scene.render.engine = "BLENDER_WORKBENCH"
        scene.display.shading.color_type = "MATERIAL"
        scene.render.resolution_x = scene.render.resolution_y = 700
        scene.render.resolution_percentage = 100
        scene.render.filepath = str(render_path.resolve())
        bpy.ops.render.render(write_still=True)

    deform["rigify_preview"] = 0.0
    deform.update_tag()
    restored = positions()
    assert all((rest[name] - restored[name]).length < 1e-5 for name in names)
    print("Rigify hand/foot IK, arm FK, skinned deformation and neutral restore passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    main(args.render)
