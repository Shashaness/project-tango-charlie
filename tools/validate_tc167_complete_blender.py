"""Reopen M21.2 in Blender, exercise the embedded controls, and compare every local matrix."""

import hashlib
import json
import sys
from pathlib import Path

import bpy
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import author_tc167_blender as base
from tools import author_tc167_complete as complete


def main():
    path = complete.PROJECT / "assets/blender/TC167_Complete.blend"
    bpy.ops.wm.open_mainfile(filepath=str(path))
    namespace = {}
    exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), namespace)
    choose = namespace["choose"]
    controller = complete.TransformationController()
    baseline = json.loads(bpy.data.texts["TC167_GEOMETRY_BASELINE.json"].as_string())
    assert base.fingerprint(list(bpy.data.objects)) == baseline
    assert not any(o.type == "ARMATURE" for o in bpy.data.objects)
    result = {
        "blender": bpy.app.version_string,
        "geometry_hierarchy_pivots_preserved": True,
        "clips": {},
        "poses": {},
    }
    inverse = base.CONVERSION.inverted()

    def error():
        bpy.context.view_layer.update()
        return max(
            float(
                np.max(
                    np.abs(
                        np.asarray(
                            inverse @ bpy.data.objects[n.name].matrix_local @ base.CONVERSION
                        )
                        - n.configuration_matrix
                    )
                )
            )
            for n in controller.model.nodes
        )

    for name, mode in complete.POSES.items():
        choose(name)
        controller.reset(mode)
        value = error()
        assert value < 2e-5, (name, value)
        result["poses"][name] = value
    for name, (first, last, reverse, duration) in complete.EDGES.items():
        choose(name)
        action = bpy.data.actions[name]
        assert len(action.slots) == 68
        maximum = 0
        endpoints = []
        for t in np.linspace(0, 1, 241):
            frame = 1 + t * duration * complete.FPS
            bpy.context.scene.frame_set(int(frame), subframe=float(frame % 1))
            controller.apply_curve(first, last, 1 - t if reverse else t)
            value = error()
            maximum = max(maximum, value)
            if t in (0, 1):
                endpoints.append(value)
        assert max(endpoints) < 2e-5, (name, endpoints)
        assert maximum < 0.004, (name, maximum)
        result["clips"][name] = {
            "duration": duration,
            "nodes": 68,
            "maximum_local_matrix_error": maximum,
            "endpoint_errors": endpoints,
            "keyframe_points": sum(len(c.keyframe_points) for c in complete.curves(action)),
            "maximum_keys_per_curve": max(len(c.keyframe_points) for c in complete.curves(action)),
        }
    for name in ("BD_Idle", "BD_Walk", "BD_Run"):
        choose(name)
        for frame in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
            bpy.context.scene.frame_set(frame)
            assert all(np.isfinite(np.asarray(o.matrix_local)).all() for o in bpy.data.objects)
    from engine.gltf_animation import AnimationPlayer

    model = base.load_glb(complete.PROJECT / "assets/models/vehicles/tc167_complete.glb")
    player = AnimationPlayer(model)
    result["exported_clips"] = {}
    for name, clip in model.animations.items():
        item = {
            "duration": clip.duration,
            "nodes": len({c.node.name for c in clip.channels}),
            "channels": len(clip.channels),
        }
        if name in complete.EDGES:
            first, last, reverse, duration = complete.EDGES[name]
            player.play(name, loop=False)
            maximum = 0.0
            endpoints = []
            for t in np.linspace(0, 1, 121):
                controller.apply_curve(first, last, 1 - t if reverse else t)
                player.time = float(t * clip.duration)
                player.apply()
                value = max(
                    float(np.max(np.abs(a.local_matrix - b.configuration_matrix)))
                    for a, b in zip(model.nodes, controller.model.nodes)
                )
                maximum = max(maximum, value)
                if t in (0, 1):
                    endpoints.append(value)
            assert maximum < 0.004 and max(endpoints) < 2e-5, (name, maximum, endpoints)
            item.update(maximum_local_matrix_error=maximum, endpoint_errors=endpoints)
        result["exported_clips"][name] = item
    result["protected_sha256"] = {
        str(p.relative_to(complete.PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (
            base.SOURCE_BLEND,
            complete.PROJECT / "assets/blender/TC167_Animated.blend",
            base.SOURCE_GLB,
            complete.PROJECT / "assets/models/vehicles/tc167_animated.glb",
        )
    }
    (complete.PROJECT / "docs/TC167_COMPLETE_VALIDATION.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
