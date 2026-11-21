"""Real Blender integration tests; run Blender --background --python this_file."""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import bmesh
import bpy
from mathutils import Matrix, Quaternion

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))


def snapshot():
    return {
        a.name: (
            tuple(a.frame_range),
            [
                (
                    s.identifier,
                    [
                        (
                            c.data_path,
                            c.array_index,
                            [
                                (
                                    tuple(k.co),
                                    tuple(k.handle_left),
                                    tuple(k.handle_right),
                                    k.interpolation,
                                    k.handle_left_type,
                                    k.handle_right_type,
                                )
                                for k in c.keyframe_points
                            ],
                        )
                        for layer in a.layers
                        for strip in layer.strips
                        for bag in strip.channelbags
                        if bag.slot_handle == s.handle
                        for c in bag.fcurves
                    ],
                )
                for s in a.slots
            ],
        )
        for a in bpy.data.actions
    }


def interior_snapshot():
    return {
        a.name: [
            (
                bag.slot_handle,
                c.data_path,
                c.array_index,
                [
                    (
                        tuple(k.co),
                        tuple(k.handle_left),
                        tuple(k.handle_right),
                        k.interpolation,
                        k.handle_left_type,
                        k.handle_right_type,
                    )
                    for k in c.keyframe_points
                    if a.frame_start < k.co.x < a.frame_end
                ],
            )
            for layer in a.layers
            for strip in layer.strips
            for bag in strip.channelbags
            for c in bag.fcurves
        ]
        for a in bpy.data.actions
        if a.name.startswith("TC_") and "To" in a.name
    }


class AuthoringTests(unittest.TestCase):
    def setUp(self):
        bpy.ops.wm.open_mainfile(
            filepath=os.environ.get(
                "TC167_TEST_BLEND", str(PROJECT / "assets/blender/TC167_Authoring.blend")
            )
        )
        self.h = {}
        exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), self.h)

    def modify(self, config="VTOL"):
        self.h["choose"](self.h["CONFIGURATIONS"][config])
        obj = bpy.data.objects["LeftWing"]
        obj.location.x += 0.125
        obj.rotation_quaternion.rotate(Quaternion((0, 0, 1), 0.04))
        bpy.context.view_layer.update()

    def test_save_configuration_and_all_related_endpoints(self):
        for config, expected in (("VTOL", 4), ("FIGHTER", 2), ("BATTLEDROID", 2)):
            self.modify(config)
            before = snapshot()
            changes = self.h["save_configuration"](config)
            self.assertTrue(
                any(c["node"] == "LeftWing" and c["channel"] == "location" for c in changes)
            )
            after = snapshot()
            pose_name = self.h["CONFIGURATIONS"][config]
            for name in before:
                if name != pose_name:
                    self.assertEqual(before[name], after[name], name)
            plan = self.h["preview"](config)
            self.assertEqual(len({(c["action"], c["endpoint"]) for c in plan}), expected)
            interior = interior_snapshot()
            before_sync = snapshot()
            self.h["apply_changes"](plan)
            self.assertEqual(interior, interior_snapshot())
            self.assertEqual(self.h["preview"](config), [])
            synced = snapshot()
            self.h["apply_changes"](self.h["preview"](config))
            self.assertEqual(synced, snapshot())
            for name in before_sync:
                if name not in self.h["RELATIONSHIPS"]:
                    self.assertEqual(before_sync[name], snapshot()[name])
            self.assertFalse(self.h["validate"]()["errors"])

    def test_cancel_unconfirmed_and_stale_previews(self):
        self.modify()
        self.h["save_configuration"]("VTOL")
        before = snapshot()
        bpy.ops.tc167.preview_endpoints()
        self.assertEqual(before, snapshot())
        self.assertEqual(bpy.ops.tc167.sync_endpoints(), {"CANCELLED"})
        self.assertEqual(before, snapshot())
        operator = SimpleNamespace(confirmed=True, report=lambda *args: None)
        fake_context = SimpleNamespace(
            window_manager=SimpleNamespace(invoke_props_dialog=lambda *a, **k: {"RUNNING_MODAL"})
        )
        self.h["TC167_OT_sync"].invoke(operator, fake_context, None)
        # Cancel/unchecked confirmation does not apply keys.
        self.h["TC167_OT_sync"].execute(operator, bpy.context)
        self.assertEqual(before, snapshot())
        operator.confirmed = True
        self.h["TC167_OT_sync"].execute(operator, bpy.context)
        self.assertEqual(self.h["preview"]("VTOL"), [])
        # The previous preview is now stale; cannot apply it a second time.
        self.assertEqual(self.h["TC167_OT_sync"].execute(operator, bpy.context), {"CANCELLED"})

    def test_missing_channel_slot_and_mismatch_detection(self):
        action = bpy.data.actions["TC_FighterToVTOL"]
        node = "LeftWing"
        curve = self.h["channels"](action, node)["location", 0]
        self.h["bags"](action, node)[0].fcurves.remove(curve)
        self.assertTrue(any(e.get("channel") == "location" for e in self.h["validate"]()["errors"]))
        plan = self.h["preview"]("VTOL")
        self.assertTrue(any(c["added_components"] for c in plan))
        self.h["apply_changes"](plan)
        self.assertFalse(self.h["validate"]()["errors"])
        slot = next(s for s in action.slots if s.identifier == "OBLeftWing")
        action.slots.remove(slot)
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.h["choose"](action.name)
        self.assertEqual(bpy.data.objects[node].animation_data.action_slot.identifier, "OBLeftWing")
        # A removed slot loses all of its previous keys. Sync of the other
        # configuration is explicit because one sync must preserve its opposite.
        self.h["apply_changes"](self.h["preview"]("FIGHTER"))
        self.assertFalse(self.h["validate"]()["errors"])

    def test_validation_is_read_only_and_detects_structural_errors(self):
        before, frame = snapshot(), bpy.context.scene.frame_current
        result = self.h["validate"]()
        self.assertFalse(result["errors"])
        self.assertEqual(before, snapshot())
        self.assertEqual(frame, bpy.context.scene.frame_current)
        bpy.data.objects["LeftWing"].matrix_parent_inverse.translation.x += 0.1
        self.assertTrue(any("pivot" in e.get("error", "") for e in self.h["validate"]()["errors"]))

    def test_undo_saved_configuration_and_sync(self):
        bpy.context.preferences.edit.use_global_undo = True
        self.modify()
        initial = snapshot()
        bpy.ops.ed.undo_push(message="Before configuration save")
        self.assertEqual(bpy.ops.tc167.save_configuration(), {"FINISHED"})
        saved = snapshot()
        self.assertNotEqual(initial, saved)
        bpy.ops.ed.undo_push(message="Saved configuration")
        self.assertEqual(bpy.ops.ed.undo(), {"FINISHED"})
        self.assertEqual(initial, snapshot())
        self.assertEqual(bpy.ops.ed.redo(), {"FINISHED"})
        self.assertEqual(saved, snapshot())
        # Real Blender undo restores sync edits, including all slotted Actions.
        self.modify()
        self.h["save_configuration"]("VTOL")
        before = snapshot()
        bpy.ops.ed.undo_push(message="Before endpoint synchronization")
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.assertNotEqual(before, snapshot())
        bpy.ops.ed.undo_push(message="Synchronized endpoints")
        self.assertEqual(bpy.ops.ed.undo(), {"FINISHED"})
        self.assertEqual(before, snapshot())
        self.assertEqual(bpy.ops.ed.redo(), {"FINISHED"})
        self.assertEqual(self.h["preview"]("VTOL"), [])

    def test_action_selection_never_saves_and_invalid_slot_cannot_save(self):
        before = snapshot()
        self.modify()
        self.h["choose"]("BD_Idle")
        self.assertEqual(before, snapshot())
        with self.assertRaisesRegex(RuntimeError, "Select the configuration"):
            bpy.ops.tc167.save_configuration()
        self.assertEqual(before, snapshot())

    def test_discontinuity_warnings_do_not_smooth_keys(self):
        self.h["choose"]("TC_Pose_VTOL")
        bpy.data.objects["LeftWing"].location.x += 3
        self.h["save_configuration"]("VTOL")
        self.assertTrue(self.h["validate"]()["errors"])
        interior = interior_snapshot()
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.assertEqual(interior, interior_snapshot())
        self.assertTrue(
            any(
                w.get("node") == "LeftWing" and w.get("channel") == "location"
                for w in self.h["motion_warnings"]()
            )
        )
        self.assertFalse(self.h["validate"]()["errors"])

    def test_insert_missing_endpoint_preserves_existing_bezier_handles(self):
        action = bpy.data.actions["TC_FighterToVTOL"]
        curve = self.h["channels"](action, "LeftWing")["location", 0]
        value = curve.evaluate(1)
        middle = curve.keyframe_points.insert(30, value + 0.5)
        middle.interpolation = "BEZIER"
        middle.handle_left_type = "FREE"
        middle.handle_right_type = "FREE"
        middle.handle_left = (25, value + 0.4)
        middle.handle_right = (35, value + 0.7)
        first = next(k for k in curve.keyframe_points if k.co.x == 1)
        curve.keyframe_points.remove(first, fast=True)
        interior = interior_snapshot()
        self.h["apply_changes"](self.h["preview"]("FIGHTER"))
        self.assertEqual(interior, interior_snapshot())
        self.assertEqual(self.h["preview"]("FIGHTER"), [])

    def test_missing_component_resets_to_rest_and_preview_never_binds(self):
        choose = self.h["choose"]
        fighter = bpy.data.actions["TC_Pose_Fighter"]
        node = "LeftWing"
        obj = bpy.data.objects[node]
        curve = self.h["channels"](fighter, node)["location", 0]
        self.h["bags"](fighter, node)[0].fcurves.remove(curve)
        before = snapshot()
        self.modify()
        for editing in (False, True):
            choose(fighter.name, editing=editing)
            self.assertAlmostEqual(
                obj.location.x, self.h["reference_component"](node, "location", 0)
            )
            self.assertEqual(obj.animation_data.action, fighter if editing else None)
        choose("TC_FighterToVTOL", editing=False)
        bpy.context.scene.frame_set(91)
        self.assertIsNone(obj.animation_data.action)
        self.assertEqual(before, snapshot())

    def test_rename_reparent_and_foreign_slot_are_reported(self):
        choose = self.h["choose"]
        choose("TC_Pose_VTOL")
        obj = bpy.data.objects["LeftWing"]
        obj.name = "Renamed Left Wing"
        self.assertTrue(
            any(
                "Renamed Left Wing" in e.get("identity_matches", [])
                for e in self.h["hierarchy_errors"]()
            )
        )
        obj.name = "LeftWing"
        obj.parent = bpy.data.objects["TC_ROOT"]
        self.assertTrue(any(e["node"] == "LeftWing" for e in self.h["hierarchy_errors"]()))
        obj.parent = bpy.data.objects[self.h["manifest"]()["structure"]["LeftWing"]["parent"]]
        obj.animation_data.action_slot = self.h["slot_for"](
            bpy.data.actions["TC_Pose_VTOL"], "RightWing"
        )
        before = snapshot()
        with self.assertRaisesRegex(ValueError, "incorrect Action or slot"):
            self.h["save_configuration"]("VTOL")
        self.assertEqual(before, snapshot())

    def test_initialize_preserves_existing_keys_and_supports_undo(self):
        self.h["choose"]("TC_Pose_Fighter", editing=False)
        part = bpy.data.objects.new("TestPreviouslyAnimatedPart", None)
        bpy.context.scene.collection.objects.link(part)
        part.parent = bpy.data.objects["Fuselage"]
        action = bpy.data.actions["TC_FighterToVTOL"]
        curve = self.h["ensure_curve"](action, part.name, "location", 0)
        for frame, value in ((1, 0.0), (91, 0.25), (181, 0.0)):
            self.h["write_key"](curve, frame, value)
        curve.update()
        before_action = snapshot()
        before = list(self.h["manifest"]()["editable"])
        bpy.ops.ed.undo_push(message="Before part initialization")
        self.assertEqual(
            bpy.ops.tc167.parts(operation="INITIALIZE", copy_current=True), {"FINISHED"}
        )
        after_action = snapshot()
        after = list(self.h["manifest"]()["editable"])
        self.assertIn(part.name, after)
        self.assertEqual(
            [tuple(key.co) for key in curve.keyframe_points], [(1, 0.0), (91, 0.25), (181, 0.0)]
        )
        self.assertFalse(self.h["validate"]()["errors"])
        bpy.ops.ed.undo_push(message="After part initialization")
        self.assertEqual(bpy.ops.ed.undo(), {"FINISHED"})
        self.assertEqual(self.h["manifest"]()["editable"], before)
        self.assertEqual(snapshot(), before_action)
        self.assertEqual(bpy.ops.ed.redo(), {"FINISHED"})
        self.assertEqual(self.h["manifest"]()["editable"], after)
        self.assertEqual(snapshot(), after_action)

    def test_scorpion_tail_separation_isolation_export_and_reopen(self):
        import numpy as np

        from engine.gltf_loader import load_glb
        from engine.model import GLTF_TO_VEHICLE
        from game.authored_transformation import AuthoredTransformationController
        from game.flight_state import VehicleMode
        from tools.export_tc167_authored import export

        choose = self.h["choose"]
        choose("TC_Pose_Fighter", editing=False)
        scene = bpy.context.scene
        source = bpy.data.objects["FuselageSurface1"]
        if source.data.users > 1:
            source.data = source.data.copy()
        names = []
        for i in range(3):
            bpy.ops.object.select_all(action="DESELECT")
            source.select_set(True)
            bpy.context.view_layer.objects.active = source
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_mode(type="FACE")
            bpy.ops.mesh.select_all(action="DESELECT")
            bm = bmesh.from_edit_mesh(source.data)
            next(iter(bm.faces)).select_set(True)
            bmesh.update_edit_mesh(source.data)
            before_names = set(scene.objects.keys())
            bpy.ops.mesh.separate(type="SELECTED")
            bpy.ops.object.mode_set(mode="OBJECT")
            (name,) = set(scene.objects.keys()) - before_names
            part = scene.objects[name]
            part.name = f"TestTail_{i}"
            names.append(part.name)
            if i:
                part.parent = scene.objects[names[i - 1]]
                part.matrix_parent_inverse = Matrix.Identity(4)
            part.rotation_mode = "QUATERNION"
            part.rotation_quaternion = Quaternion()
            part.location = (0, -0.3, 0)

        self.assertEqual({o.name for o in self.h["unregistered"]()}, set(names))
        self.assertTrue(
            any(
                "geometry" in warning
                for item in self.h["geometry_warnings"]()
                for warning in item["warnings"]
            )
        )
        choose("TC_Pose_VTOL")
        scene.objects[names[0]].rotation_quaternion = Quaternion((1, 0, 0), 0.4)
        before = snapshot()
        with self.assertRaisesRegex(ValueError, "explicitly confirm"):
            self.h["register_articulated_objects"]()
        with self.assertRaisesRegex(ValueError, "unregistered"):
            self.h["save_configuration"]("VTOL")
        self.assertEqual(snapshot(), before)
        choose("TC_Pose_Fighter", editing=False)
        scene.objects[names[0]].rotation_quaternion = Quaternion()
        original_geometry = {n: self.h["mesh_digest"](scene.objects[n]) for n in names}
        initialized = self.h["register_articulated_objects"]()
        self.assertEqual({part["node"] for part in initialized}, set(names))
        self.assertEqual(
            original_geometry, {n: self.h["mesh_digest"](scene.objects[n]) for n in names}
        )

        def samples(config):
            choose(self.h["CONFIGURATIONS"][config])
            return {n: scene.objects[n].matrix_world.copy() for n in names}

        fighter = samples("FIGHTER")
        battledroid = samples("BATTLEDROID")
        samples("VTOL")
        for i, name in enumerate(names):
            obj = scene.objects[name]
            obj.rotation_quaternion = Quaternion((1, 0, 0), 0.4 + 0.15 * i)
            obj.location.z += 0.12 * (i + 1)
        bpy.context.view_layer.update()
        raised = {n: scene.objects[n].matrix_world.copy() for n in names}
        self.assertTrue(self.h["pending_transforms"]())
        before = snapshot()
        self.h["save_configuration"]("VTOL")
        after = snapshot()
        self.assertEqual(
            {n: a for n, a in before.items() if n != "TC_Pose_VTOL"},
            {n: a for n, a in after.items() if n != "TC_Pose_VTOL"},
        )

        def compare(actual, expected):
            for n in names:
                np.testing.assert_allclose(np.array(actual[n]), np.array(expected[n]), atol=2e-5)

        compare(samples("FIGHTER"), fighter)
        compare(samples("BATTLEDROID"), battledroid)
        compare(samples("VTOL"), raised)
        self.assertIn(
            "rotation_quaternion", self.h["comparison"]()["differences"]["FIGHTER / VTOL"][names[0]]
        )
        action = bpy.data.actions["TC_FighterToVTOL"]
        curve = self.h["channels"](action, names[0])["location", 2]
        self.h["write_key"](curve, 91, 0.35)
        interior = interior_snapshot()

        def endpoints(other=False):
            result = {}
            for name, (source_config, _) in self.h["RELATIONSHIPS"].items():
                choose(name)
                action = bpy.data.actions[name]
                at_first = (source_config == "VTOL") != other
                scene.frame_set(int(action.frame_start if at_first else action.frame_end))
                result[name] = {n: scene.objects[n].matrix_world.copy() for n in names}
            return result

        opposite = endpoints(other=True)
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.assertEqual(interior_snapshot(), interior)
        for matrices in endpoints().values():
            compare(matrices, raised)
        for name, matrices in endpoints(other=True).items():
            compare(matrices, opposite[name])
        before = snapshot()
        self.assertEqual(self.h["preview"]("VTOL"), [])
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.assertEqual(snapshot(), before)

        with tempfile.TemporaryDirectory(prefix="tc167-scorpion-") as directory:
            blend = Path(directory) / "scorpion.blend"
            glb = Path(directory) / "scorpion.glb"
            bpy.ops.wm.save_as_mainfile(filepath=str(blend))
            export(blend, glb)  # Reopens the saved .blend.
            self.assertEqual(snapshot(), before)
            model = load_glb(glb)
            for gait in ("BD_Idle", "BD_Walk", "BD_Run"):
                self.assertTrue(
                    all(
                        channel.node.name not in names
                        for channel in model.animations[gait].channels
                    )
                )
            controller = AuthoredTransformationController(model=model)
            conversion = np.array(Matrix.Rotation(-np.pi / 2, 4, "X"))

            def check_runtime(expected):
                for n in names:
                    node = model.find_node(n)
                    world = (
                        GLTF_TO_VEHICLE
                        @ conversion
                        @ np.array(expected[n])
                        @ np.linalg.inv(conversion)
                    )
                    np.testing.assert_allclose(node.world_matrix, world, atol=2e-4)

            for mode, expected in (
                (VehicleMode.FIGHTER, fighter),
                (VehicleMode.VTOL, raised),
                (VehicleMode.BATTLEDROID, battledroid),
            ):
                controller._apply(controller.configuration_pose(mode))
                check_runtime(expected)
            for clip, (left, right) in self.h["RELATIONSHIPS"].items():
                for fraction, config in ((0, left), (1, right)):
                    time = model.animations[clip].duration * fraction
                    controller._apply(controller._sample(clip, time, controller.rest))
                    check_runtime(raised if config == "VTOL" else opposite[clip])
            ns = self.h
            mid_clip = "TC_FighterToVTOL"
            choose(mid_clip)
            bpy.context.scene.frame_set(91)
            midpoint = {n: bpy.data.objects[n].matrix_world.copy() for n in names}
            bpy.ops.wm.read_factory_settings(use_empty=True)
            bpy.ops.import_scene.gltf(filepath=str(glb))
            self.assertEqual(bpy.data.objects[names[1]].parent.name, names[0])
            self.assertEqual(bpy.data.objects[names[2]].parent.name, names[1])

            def imported(clip, fraction, expected):
                action = bpy.data.actions[clip]
                for obj in bpy.context.scene.objects:
                    slot = ns["slot_for"](action, obj.name)
                    if slot:
                        data = obj.animation_data_create()
                        data.use_nla = False
                        data.action = action
                        data.action_slot = slot
                    elif obj.animation_data:
                        obj.animation_data.action = None
                start, end = action.frame_range
                frame = start + (end - start) * fraction
                bpy.context.scene.frame_set(int(frame), subframe=frame % 1)
                compare({n: bpy.data.objects[n].matrix_world for n in names}, expected)

            for config, expected in (
                ("FIGHTER", fighter),
                ("VTOL", raised),
                ("BATTLEDROID", battledroid),
            ):
                imported(ns["CONFIGURATIONS"][config], 0, expected)
            for clip, (left, right) in ns["RELATIONSHIPS"].items():
                imported(clip, 0, raised if left == "VTOL" else opposite[clip])
                imported(clip, 1, raised if right == "VTOL" else opposite[clip])
            imported(mid_clip, 0.5, midpoint)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(AuthoringTests)
    )
    Path("/tmp/tc167_m215_blender_tests.json").write_text(
        json.dumps(
            {
                "tests": result.testsRun,
                "failures": len(result.failures),
                "errors": len(result.errors),
                "details": [detail for test, detail in result.failures + result.errors],
            },
            indent=2,
        )
    )
    if not result.wasSuccessful():
        raise RuntimeError("Blender authoring integration tests failed")
