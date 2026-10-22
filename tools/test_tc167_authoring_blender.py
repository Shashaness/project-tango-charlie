"""Real Blender integration tests; run Blender --background --python this_file."""

import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import bpy

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
        bpy.ops.wm.open_mainfile(filepath=str(PROJECT / "assets/blender/TC167_Authoring.blend"))
        self.h = {}
        exec(bpy.data.texts["TC167_Action_Controls.py"].as_string(), self.h)

    def modify(self, config="VTOL"):
        self.h["choose"](self.h["CONFIGURATIONS"][config])
        obj = bpy.data.objects["LeftWing"]
        obj.location.x += 0.125
        obj.rotation_quaternion.rotate(__import__("mathutils").Quaternion((0, 0, 1), 0.04))
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

    def test_edited_export_and_runtime_roundtrip(self):
        from tools.author_tc167_authoring import export
        from tools.tc167_runtime_pose_bridge import build

        self.modify()
        self.h["save_configuration"]("VTOL")
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        self.assertFalse(self.h["validate"]()["errors"])
        # Exercise GLB discovery for an actually missing multi-object slot.
        action = bpy.data.actions["TC_FighterToVTOL"]
        slot = next(s for s in action.slots if s.identifier == "OBLeftWing")
        action.slots.remove(slot)
        self.h["apply_changes"](self.h["preview"]("VTOL"))
        output = Path("/tmp/tc167_m213_edited.glb")
        export(output)
        candidate, report = build(output)
        self.assertLess(max(report["endpoint_errors"].values()), 2e-5)
        self.assertLess(max(report["roundtrip_errors"].values()), 2e-5)
        self.assertIn("LeftWing", report["python_configuration_errors"]["VTOL"])
        self.assertEqual(report["python_configuration_errors"]["FIGHTER"], {})
        self.assertEqual(report["python_configuration_errors"]["BATTLEDROID"], {})


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(AuthoringTests)
    )
    Path("/tmp/tc167_m213_blender_tests.json").write_text(
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
