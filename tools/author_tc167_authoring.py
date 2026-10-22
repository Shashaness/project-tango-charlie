"""Create M21.3 from the preserved M21.2 file, or export explicitly saved artist edits."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import author_tc167_blender as base
from tools import author_tc167_complete as complete

PROJECT = base.PROJECT
BLEND = PROJECT / "assets/blender/TC167_Authoring.blend"
GLB = PROJECT / "assets/models/vehicles/tc167_authoring.glb"


def embed():
    rest = json.loads(bpy.data.texts["TC167_FIGHTER_BASIS.json"].as_string())
    structure = base.fingerprint(list(bpy.data.objects))
    editable = {n for n in rest if bpy.data.objects[n].type != "MESH"}
    # Existing shell proportion/configuration channels are intentional articulation;
    # unchanged decorative mesh companions are never captured or newly animated.
    for action_name in complete.POSES:
        action = bpy.data.actions[action_name]
        for slot in action.slots:
            node = slot.identifier.removeprefix("OB")
            for layer in action.layers:
                for strip in layer.strips:
                    for bag in strip.channelbags:
                        if bag.slot_handle != slot.handle:
                            continue
                        from mathutils import Matrix

                        reference = Matrix(rest[node]).decompose()
                        defaults = dict(
                            zip(("location", "rotation_quaternion", "scale"), reference)
                        )
                        if any(
                            abs(
                                c.evaluate(action.frame_start)
                                - defaults[c.data_path][c.array_index]
                            )
                            > 2e-5
                            for c in bag.fcurves
                        ):
                            editable.add(node)
    metadata = {
        "version": 1,
        "rest": rest,
        "editable": sorted(editable),
        "structure": structure,
        "metadata_hashes": {
            name: hashlib.sha256(bpy.data.texts[name].as_string().encode()).hexdigest()
            for name in ("TC167_FIGHTER_BASIS.json", "TC167_GEOMETRY_BASELINE.json")
        },
        "mesh_hashes": {
            o.name: hashlib.sha256(
                repr(
                    (
                        [list(v.co) for v in o.data.vertices],
                        [list(p.vertices) for p in o.data.polygons],
                    )
                ).encode()
            ).hexdigest()
            for o in bpy.data.objects
            if o.type == "MESH"
        },
    }
    bpy.data.texts.new("TC167_AUTHORING_MANIFEST.json").write(json.dumps(metadata))
    text = bpy.data.texts["TC167_Action_Controls.py"]
    text.clear()
    text.write((PROJECT / "tools/tc167_authoring_controls.py").read_text())
    namespace = {}
    exec(text.as_string(), namespace)
    namespace["choose"]("TC_Pose_Fighter")
    return namespace


def export(output):
    base.CLIPS.update({n: 1 + round(v[3] * complete.FPS) for n, v in complete.EDGES.items()})
    base.CLIPS.update({n: 2 for n in complete.POSES})
    base.export_actions(output)
    complete.continuous_export(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--blend-input", type=Path)
    parser.add_argument("--blend-output", type=Path, default=BLEND)
    parser.add_argument("--glb-output", type=Path, default=GLB)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    protected = [
        PROJECT / "assets/blender" / n
        for n in ("TC167.blend", "TC167_Animated.blend", "TC167_Complete.blend")
    ]
    protected += [
        PROJECT / "assets/models/vehicles" / n
        for n in ("tc167.glb", "tc167_animated.glb", "tc167_complete.glb")
    ]
    if any(p.resolve() in protected for p in (args.blend_output, args.glb_output)):
        parser.error("Previous milestone and production assets are protected")
    if not args.export_only and args.blend_output.exists():
        parser.error("Authoring output exists; choose another filename")
    source = args.blend_input or (
        BLEND if args.export_only else PROJECT / "assets/blender/TC167_Complete.blend"
    )
    bpy.ops.wm.open_mainfile(filepath=str(source.resolve()))
    if args.export_only:
        namespace = {}
        exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), namespace)
    else:
        namespace = embed()
    report = namespace["validate"]()
    print("CONSISTENCY:", json.dumps(report["errors"], indent=2))
    if report["errors"]:
        raise ValueError(
            "Configuration validation failed; preview and synchronize explicitly before export"
        )
    export(args.glb_output)
    if not args.export_only:
        # Preserve exact initial native gaits. Export-only always uses saved artist edits.
        complete.preserve_native_gaits(args.glb_output)
        namespace["choose"]("TC_Pose_Fighter")
        bpy.ops.wm.save_as_mainfile(filepath=str(args.blend_output.resolve()))
    print("M21.3 authoring export:", args.glb_output)


if __name__ == "__main__":
    main()
