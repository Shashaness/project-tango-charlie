"""Build the female pilot from Meshy source, or export an existing PTC character."""

import argparse
import sys
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import ptc_character_sdk as sdk  # noqa: E402

SOURCE = ROOT / "assets/source/Female_Fighter_Pilot_Navy_10K_adaptive.blend"
PRODUCTION = ROOT / "assets/blender/characters/female_pilot.blend"
RUNTIME = ROOT / "assets/models/characters/female_pilot.glb"


def build():
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
    mesh = next(obj for obj in bpy.context.scene.objects if obj.type == "MESH")
    mesh.name = "Female Fighter Pilot"
    sdk.initialize(mesh, 1.80)
    rig = sdk.create_rig(mesh, 1.80)
    sdk.bind(mesh, rig)
    sdk.generate_controls(mesh, rig)
    bpy.context.scene.render.fps = 30
    with sdk.reference_armature(ROOT / "assets/blender/pilot.blend") as source:
        sdk.retarget(mesh, rig, source)
    sdk.seat(mesh, rig)
    for name, (_, action) in sdk.action_tracks(rig).items():
        action.name = name
    mesh["ptc_seat_offset"] = [0.0, 0.18, -0.02]
    sdk.validate(mesh, rig)
    sdk.activate_preview(rig, "Idle")
    sdk.register()
    settings = bpy.context.scene.ptc_character
    settings.mesh, settings.rig = mesh, rig
    settings.output = "//../../models/characters/female_pilot.glb"
    settings.reference = "//../pilot.blend"
    sdk.activate(rig)
    bpy.ops.file.pack_all()
    bpy.context.preferences.filepaths.save_version = 0
    PRODUCTION.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(PRODUCTION), relative_remap=False)
    return mesh, rig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("build", "export", "validate"))
    parser.add_argument("--blend", type=Path, default=PRODUCTION)
    parser.add_argument("--output", type=Path, default=RUNTIME)
    parser.add_argument("--mesh", help="Mesh object name when the file has multiple characters")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    output = args.output.resolve()
    if args.command == "validate":
        sdk.validate_glb(output)
        print(f"Validated {output}")
        return
    if args.command == "build":
        mesh, rig = build()
    else:
        bpy.ops.wm.open_mainfile(filepath=str(args.blend.resolve()))
        candidates = [
            obj
            for obj in bpy.context.scene.objects
            if obj.type == "MESH" and obj.get("ptc_character_version") == 1
        ]
        if args.mesh:
            mesh = bpy.context.scene.objects[args.mesh]
        elif len(candidates) == 1:
            mesh = candidates[0]
        else:
            raise ValueError("Select the character mesh with --mesh.")
        rig = sdk.deform_for(mesh)
        if not rig:
            raise ValueError("Character mesh lacks its deform armature modifier.")
    print(sdk.inspect(mesh))
    sdk.export_character(mesh, rig, output)
    print(f"Validated {output}")


if __name__ == "__main__":
    main()
