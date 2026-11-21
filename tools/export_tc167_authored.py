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
        controls["install_identity_baseline"]()
        controls["choose"]("TC_Pose_Fighter", editing=False)
        bpy.context.view_layer.update()
        controls["create_references"]()
    report = controls["validate"]()
    print("TC167 VALIDATION", json.dumps(report))
    if report["errors"]:
        raise ValueError("Validate for Game failed; inspect reported Action/reference errors")
    if install:
        bpy.ops.wm.save_as_mainfile(filepath=str(blend.resolve()))
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tc167-export-", dir=output.parent) as directory:
        candidate = Path(directory) / "candidate.glb"
        if controls["export_scene_glb"](candidate.resolve()) != {"FINISHED"}:
            raise ValueError("Blender GLB export did not finish")
        from engine.gltf_loader import load_glb
        from game.authored_transformation import REQUIRED_ACTIONS, AuthoredTransformationController

        model = load_glb(candidate)
        AuthoredTransformationController(model=model)
        extra = set(model.animations) - set(REQUIRED_ACTIONS)
        if extra:
            raise ValueError("Unexpected exported Actions: " + ", ".join(sorted(extra)))
        candidate.replace(output)
    print(
        "TC167 EXPORTED", [(c.name, c.duration, len(c.channels)) for c in model.animations.values()]
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
