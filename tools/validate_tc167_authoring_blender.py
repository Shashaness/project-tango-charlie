"""Validate M21.3 saved authoring data, runtime export and protected assets in Blender."""

import hashlib
import json
import sys
from pathlib import Path

import bpy

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from tools.author_tc167_blender import fingerprint  # noqa: E402
from tools.tc167_runtime_pose_bridge import build  # noqa: E402


def main():
    bpy.ops.wm.open_mainfile(filepath=str(PROJECT / "assets/blender/TC167_Authoring.blend"))
    namespace = {}
    exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), namespace)
    assert (
        bpy.data.texts["TC167_Action_Controls.py"].as_string()
        == (PROJECT / "tools/tc167_authoring_controls.py").read_text()
    )
    before = fingerprint(list(bpy.data.objects))
    baseline = json.loads(bpy.data.texts["TC167_GEOMETRY_BASELINE.json"].as_string())
    assert before == baseline
    consistency = namespace["validate"]()
    assert not consistency["errors"], consistency["errors"]
    assert fingerprint(list(bpy.data.objects)) == before
    inventory = {}
    for action in bpy.data.actions:
        inventory[action.name] = {
            "frames": [action.frame_start, action.frame_end],
            "slots": len(action.slots),
            "curves": sum(
                len(b.fcurves)
                for layer in action.layers
                for s in layer.strips
                for b in s.channelbags
            ),
            "keys": sum(
                len(c.keyframe_points)
                for layer in action.layers
                for s in layer.strips
                for b in s.channelbags
                for c in b.fcurves
            ),
        }
    for name in (
        list(namespace["CONFIGURATIONS"].values())
        + list(namespace["RELATIONSHIPS"])
        + ["BD_Idle", "BD_Walk", "BD_Run"]
    ):
        namespace["choose"](name)
        action = bpy.data.actions[name]
        for frame in (
            action.frame_start,
            (action.frame_start + action.frame_end) / 2,
            action.frame_end,
        ):
            bpy.context.scene.frame_set(int(frame), subframe=float(frame % 1))
        for node in namespace["manifest"]()["rest"]:
            obj = bpy.data.objects[node]
            slot = next((s for s in action.slots if s.identifier == "OB" + node), None)
            assert obj.animation_data.action == (action if slot else None)
            if slot:
                assert obj.animation_data.action_slot == slot
    candidate, runtime = build(PROJECT / "assets/models/vehicles/tc167_authoring.glb")
    protected = [
        PROJECT / "assets/blender" / n
        for n in ("TC167.blend", "TC167_Animated.blend", "TC167_Complete.blend")
    ]
    protected += [
        PROJECT / "assets/models/vehicles" / n
        for n in ("tc167.glb", "tc167_animated.glb", "tc167_complete.glb")
    ]
    report = {
        "blender": bpy.app.version_string,
        "consistency": consistency,
        "inventory": inventory,
        "editable_nodes": namespace["manifest"]()["editable"],
        "runtime_pose_bridge": runtime,
        "protected_sha256": {
            str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in protected
        },
        "limitations": [
            "Background Blender evaluation; interactive visual review remains manual.",
            "Overlap warnings are conservative AABBs on five major shells at three clip times.",
            "Runtime configuration ownership remains in Python; "
            "candidate generation does not install it.",
        ],
    }
    tests = Path("/tmp/tc167_m213_blender_tests.json")
    if tests.exists():
        report["blender_integration_tests"] = json.loads(tests.read_text())
    (PROJECT / "docs/TC167_AUTHORING_WORKFLOW_VALIDATION.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(
        "Validated",
        len(inventory),
        "Actions;",
        len(consistency["warnings"]),
        "motion/overlap warnings",
    )


if __name__ == "__main__":
    main()
