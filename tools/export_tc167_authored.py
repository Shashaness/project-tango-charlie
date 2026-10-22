"""Export saved Blender Actions and complete evolving geometry; never regenerate motion.

Run in Blender: --background --python tools/export_tc167_authored.py -- --install
Install embeds the sidebar and creates only missing original-vehicle references.
Normal exports validate and export saved edits without changing the .blend.
"""

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

import bpy
from mathutils import Matrix

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))


def export(blend, output, install=False):
    protected = {
        PROJECT / "assets/models/vehicles" / name
        for name in ("tc167.glb", "tc167_animated.glb", "tc167_complete.glb")
    }
    if output.resolve() in protected:
        raise ValueError("Previous milestone and production GLBs are protected")
    bpy.ops.wm.open_mainfile(filepath=str(blend.resolve()))
    if install:
        backup = PROJECT / "backups/TC167_Authoring_pre_M21_4.blend"
        if not backup.exists():
            shutil.copy2(blend, backup)
        text = bpy.data.texts["TC167_Action_Controls.py"]
        text.clear()
        text.write((PROJECT / "tools/tc167_authoring_controls.py").read_text())
    controls = {}
    exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), controls)
    if install:
        controls["choose"]("TC_Pose_Fighter")
        bpy.context.view_layer.update()
        controls["create_references"]()
        bpy.ops.wm.save_as_mainfile(filepath=str(blend.resolve()))
    report = controls["validate"]()
    print("M21.4 VALIDATION", json.dumps(report))
    if report["errors"]:
        raise ValueError("Validate for Game failed; inspect reported Action/reference errors")
    # Set neutral local reference transforms before exporter discovers all slotted Actions.
    # The saved file is not altered by ordinary export.
    for obj in bpy.context.scene.objects:
        if obj.animation_data:
            obj.animation_data.action = None
            obj.animation_data.use_nla = False
        basis = controls["manifest"]()["rest"].get(obj.name)
        if basis is not None:
            obj.matrix_basis = Matrix(basis)
        obj.hide_set(False)
    bpy.context.scene.frame_set(0)
    bpy.context.view_layer.update()
    bpy.ops.object.select_all(action="SELECT")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tc167-export-", dir=output.parent) as directory:
        candidate = Path(directory) / "candidate.glb"
        bpy.ops.export_scene.gltf(
            filepath=str(candidate.resolve()),
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
        from engine.gltf_loader import load_glb
        from game.authored_transformation import AuthoredTransformationController

        model = load_glb(candidate)
        AuthoredTransformationController(model=model)
        candidate.replace(output)
    print(
        "M21.4 EXPORTED", [(c.name, c.duration, len(c.channels)) for c in model.animations.values()]
    )
    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true")
    parser.add_argument(
        "--blend-input", type=Path, default=PROJECT / "assets/blender/TC167_Authoring.blend"
    )
    parser.add_argument(
        "--glb-output", type=Path, default=PROJECT / "assets/models/vehicles/tc167_authoring.glb"
    )
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    export(args.blend_input, args.glb_output, args.install)
