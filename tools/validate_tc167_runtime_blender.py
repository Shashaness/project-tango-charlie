"""Blender/GLB evaluated hierarchy and intermediate-edit round-trip validation.

Runs only on temporary copies, leaving the saved authoring file unchanged.
"""

import hashlib
import json
import sys
import tempfile
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from engine.gltf_loader import load_glb  # noqa: E402
from engine.model import GLTF_TO_VEHICLE  # noqa: E402
from game.authored_transformation import CONFIGURATIONS, EDGES, AuthoredTransformationController  # noqa: E402
from tools.export_tc167_authored import export  # noqa: E402

BLEND = PROJECT / "assets/blender/TC167_Authoring.blend"
GLB = PROJECT / "assets/models/vehicles/tc167_authoring.glb"


def controls():
    ns = {}
    exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), ns)
    return ns


def main():
    original_hash = hashlib.sha256(BLEND.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(BLEND))
    ns = controls()
    assert not ns["validate"]()["errors"]
    m = load_glb(GLB)
    c = AuthoredTransformationController(model=m)
    # Blender Z-up -> glTF Y-up; model adds the explicit vehicle heading conversion.
    r = np.array(Matrix.Rotation(-np.pi / 2, 4, "X"), dtype=float)
    maxima = {}
    for action_name in (*CONFIGURATIONS.values(), *EDGES.values(), "BD_Idle", "BD_Walk", "BD_Run"):
        ns["choose"](action_name)
        action = bpy.data.actions[action_name]
        clip = m.animations[action_name]
        for fraction in (0, 0.5, 1):
            frame = action.frame_start + fraction * (action.frame_end - action.frame_start)
            bpy.context.scene.frame_set(int(frame), subframe=frame - int(frame))
            bpy.context.view_layer.update()
            if action_name.startswith("BD_"):
                from game.flight_state import VehicleMode

                ref = c.configuration_pose(VehicleMode.BATTLEDROID)
            else:
                ref = c.rest
            c._apply(c._sample(action_name, fraction * clip.duration, ref))
            for node in m.nodes:
                obj = bpy.data.objects[node.name]
                expected = GLTF_TO_VEHICLE @ r @ np.array(obj.matrix_world) @ np.linalg.inv(r)
                error = float(np.max(np.abs(expected - node.world_matrix)))
                maxima[action_name] = max(maxima.get(action_name, 0), error)
                if error > 2e-4:
                    raise AssertionError((action_name, fraction, node.name, error))
    # Simulate a newly separated fuselage segment, register slots and add an artistic
    # intermediate rotation. Export/import must preserve the edit, not a pose table.
    ns["choose"]("TC_Pose_Fighter")
    original = bpy.data.objects["FuselageSurface1"]
    mesh = original.data.copy()
    node = bpy.data.objects.new("M21_TestScorpionSegment", mesh)
    bpy.context.scene.collection.objects.link(node)
    node.parent = original.parent
    node.matrix_basis = original.matrix_basis.copy()
    ns["register_articulated_objects"]()
    ns["choose"]("TC_FighterToVTOL")
    action = bpy.data.actions["TC_FighterToVTOL"]
    middle = round((action.frame_start + action.frame_end) / 2)
    for i, value in enumerate((np.cos(0.45), 0, np.sin(0.45), 0)):
        curve = ns["ensure_curve"](action, node.name, "rotation_quaternion", i)
        ns["write_key"](curve, middle, float(value))
        curve.update()

    # Endpoint sync must not alter intermediate key coordinates/handles/interpolation.
    def fingerprint():
        return [
            (
                curve.data_path,
                curve.array_index,
                tuple(k.co),
                tuple(k.handle_left),
                tuple(k.handle_right),
                k.interpolation,
            )
            for curve in ns["channels"](action, node.name).values()
            for k in curve.keyframe_points
            if action.frame_start < k.co.x < action.frame_end
        ]

    before = fingerprint()
    for config in CONFIGURATIONS:
        ns["apply_changes"](ns["preview"](config.name))
    assert fingerprint() == before
    with tempfile.TemporaryDirectory(prefix="tc167-m21-4-") as directory:
        blend = Path(directory) / "roundtrip.blend"
        glb = Path(directory) / "roundtrip.glb"
        bpy.ops.wm.save_as_mainfile(filepath=str(blend))
        time = (middle - action.frame_start) / bpy.context.scene.render.fps
        exported = export(blend, glb)
        runtime = AuthoredTransformationController(model=exported)
        from game.flight_state import VehicleMode

        runtime.request(VehicleMode.VTOL)
        runtime.update(time)
        result = runtime.pose[exported.find_node("M21_TestScorpionSegment")][1]
        assert abs(result[1]) > 0.3 or abs(result[2]) > 0.3
        assert runtime.model.find_node("M21_TestScorpionSegment").parent is not None
    assert hashlib.sha256(BLEND.read_bytes()).hexdigest() == original_hash
    report = {
        "blender_glb_world_matrix_errors": maxima,
        "new_articulated_mesh_roundtrip": True,
        "endpoint_sync_preserves_interiors": True,
        "saved_authoring_asset_unchanged_by_validation": True,
    }
    (PROJECT / "docs/M21_4_BLENDER_VALIDATION.json").write_text(json.dumps(report, indent=2) + "\n")
    print("M21.4 ROUNDTRIP PASS", json.dumps(report))


if __name__ == "__main__":
    main()
