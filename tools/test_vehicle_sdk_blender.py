"""Run SDK regression fixtures in Blender 5.0.1. Pass -- --ui in graphical Blender."""

import sys
import tempfile
from pathlib import Path

import bpy

options = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
addon_dir = (
    bpy.utils.user_resource("SCRIPTS", path="addons")
    if "--installed" in options
    else str(Path(__file__).resolve().parent)
)
sys.path.insert(0, addon_dir)
import ptc_vehicle_sdk as sdk  # noqa: E402


def mesh(name):
    data = bpy.data.meshes.new(name)
    data.from_pydata([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [], [(0, 1, 2)])
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    return obj


def fixture(count):
    ctx = bpy.context
    root = sdk.assign_root(ctx, sdk.empty(ctx, f"Root {count}"), f"vehicle-{count}")
    original = sdk.replace_visual(ctx, mesh("Old body"), root, "body")
    sdk.replace_visual(ctx, mesh("Body"), root, "body")
    joint = sdk.author_anchor(ctx, root, root, "joint", "base_joint")
    elbow = sdk.author_anchor(ctx, root, joint, "joint", "nested_joint")
    anchor = sdk.author_anchor(ctx, root, elbow, "operator", "station")
    assert sdk.hierarchy_depth(anchor) == 3 and sdk.owning_root(anchor) == root
    unassigned = sdk.empty(ctx, "Unassigned")
    root.parent = unassigned
    try:
        sdk.author_anchor(ctx, root, elbow, "attachment", "bad", unassigned)
    except ValueError:
        pass
    else:
        raise AssertionError("An ancestor was accepted as a child")
    finally:
        root.parent = None
    hubs = []
    for index in range(count):
        hub = sdk.wheel_hub(ctx, root, f"wheel-{index}", (index, 0, 0))
        sdk.replace_visual(ctx, mesh("Tire"), sdk.spin_pivot(hub), "wheel_visual")
        hubs.append(hub)
    return root, original, anchor, hubs


def select(obj):
    view = bpy.context.view_layer
    for previous in bpy.context.selected_objects:
        previous.select_set(False)
    view.objects.active = obj
    if obj:
        obj.select_set(True)


def export_checks():
    ctx = bpy.context
    scene = ctx.scene
    root, archived, anchor, hubs = fixture(4)
    fixtures = [fixture(count) for count in (6, 8)]
    other = mesh("Unrelated")
    select(hubs[0])
    configured = scene.ptc_vehicle
    configured.root = fixtures[-1][0]
    assert configured.root != hubs[0]
    assert bpy.ops.ptc_vehicle.author(operation="RESOLVE") == {"FINISHED"}
    assert configured.root == root
    select(other)
    ctx.view_layer.update()
    assert sdk.owning_root(other) is None and configured.root == root
    assert not sdk.root_poll(configured, other) and not sdk.root_poll(configured, hubs[0])
    assert bpy.ops.ptc_vehicle.author(operation="LOCATE", component_name=anchor.name) == {
        "FINISHED"
    }
    assert ctx.view_layer.objects.active == anchor
    select(other)
    parent = bpy.data.collections.new("Mechanics")
    nested = bpy.data.collections.new("Inside mechanics")
    sibling = bpy.data.collections.new("Unrelated excluded")
    scene.collection.children.link(parent)
    parent.children.link(nested)
    parent.children.link(sibling)
    for collection in list(anchor.users_collection):
        collection.objects.unlink(anchor)
    nested.objects.link(anchor)
    outer = ctx.view_layer.layer_collection.children[parent.name]
    inner = outer.children[nested.name]
    sibling_layer = outer.children[sibling.name]
    outer.hide_viewport = True
    inner.exclude = True
    sibling_layer.exclude = True
    nested.hide_viewport = True
    hubs[0].hide_set(True)
    hubs[1].hide_viewport = True

    def state():
        return (
            outer.exclude,
            inner.exclude,
            outer.hide_viewport,
            sibling_layer.exclude,
            nested.hide_viewport,
            hubs[0].hide_get(),
            hubs[1].hide_viewport,
            archived.hide_get(),
            archived.hide_render,
            tuple(obj.name for obj in ctx.selected_objects),
            ctx.view_layer.objects.active,
        )

    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "vehicle.glb"
        expected = state()
        sdk.export_vehicle(ctx, root, str(path))
        assert state() == expected
        good = path.read_bytes()
        called = False

        def fail(**kwargs):
            nonlocal called
            called = True
            assert anchor.visible_get() and hubs[0].visible_get()
            assert not archived.select_get() and not other.select_get()
            assert sibling_layer.exclude
            Path(kwargs["filepath"]).write_bytes(b"partial")
            raise RuntimeError("injected export failure")

        try:
            sdk.export_vehicle(ctx, root, str(path), exporter=fail)
        except RuntimeError as error:
            assert str(error) == "injected export failure"
        else:
            raise AssertionError("Injected export error was ignored")
        assert called and state() == expected and path.read_bytes() == good
        for other_root, _, _, _ in fixtures:
            sdk.export_vehicle(ctx, other_root, str(path))
        assert state() == expected
        outer.exclude = True
        expected = state()
        sdk.export_vehicle(ctx, root, str(path))
        assert state() == expected
    print("PTC Vehicle export regression passed", flush=True)


def graphical_checks(drawn):
    ctx = bpy.context
    root, archived, anchor, hubs = fixture(8)
    region = next(r for r in ctx.area.regions if r.type == "UI") if ctx.area else None
    area = next(a for a in ctx.window.screen.areas if a.type == "VIEW_3D")
    area.spaces.active.show_region_ui = True
    region = region or next(r for r in area.regions if r.type == "UI")
    cases = [
        (None, None),
        (root, root),
        (hubs[0], hubs[0]),
        (root, mesh("Untagged")),
        (root, hubs[0]),
        (root, sdk.spin_pivot(hubs[0]).children[0]),
        (root, archived),
        (root, anchor),
    ]
    archived.hide_set(False)
    pending = iter(cases)
    current = None

    def advance():
        nonlocal current
        if current is not None and current not in drawn:
            raise RuntimeError(f"Sidebar did not complete draw for {current}")
        try:
            configured, selected = next(pending)
        except StopIteration:
            print("PTC Vehicle graphical sidebar regression passed", flush=True)
            bpy.ops.wm.quit_blender()
            return None
        ctx.scene.ptc_vehicle.root = configured
        select(selected)
        current = (configured.name if configured else None, selected.name if selected else None)
        region.active_panel_category = "PTC Vehicle"
        area.tag_redraw()
        return 0.6

    bpy.app.timers.register(advance, first_interval=0.6)


if "--ui" in options:
    drawn = set()
    original_draw = sdk.PTCVEHICLE_PT_sidebar.draw

    def draw(panel, context):
        original_draw(panel, context)
        root, obj = context.scene.ptc_vehicle.root, context.active_object
        drawn.add((root.name if root else None, obj.name if obj else None))

    sdk.PTCVEHICLE_PT_sidebar.draw = draw
    sdk.register()
    graphical_checks(drawn)
else:
    sdk.register()
    export_checks()
