"""Self-contained Blender helper embedded into TC167_Authoring.blend (no game imports)."""

import hashlib
import itertools
import json
import math
import uuid
from functools import lru_cache

import bpy
from mathutils import Matrix, Quaternion, Vector

RELATIONSHIPS = {
    "TC_FighterToVTOL": ("FIGHTER", "VTOL"),
    "TC_VTOLToFighter": ("VTOL", "FIGHTER"),
    "TC_VTOLToBattledroid": ("VTOL", "BATTLEDROID"),
    "TC_BattledroidToVTOL": ("BATTLEDROID", "VTOL"),
}
CONFIGURATIONS = {
    "FIGHTER": "TC_Pose_Fighter",
    "VTOL": "TC_Pose_VTOL",
    "BATTLEDROID": "TC_Pose_Battledroid",
}
PATHS = {"location": 3, "rotation_quaternion": 4, "scale": 3}
TOLERANCE = 2e-5


@lru_cache(maxsize=4)
def _decode_manifest(source):
    return json.loads(source)


def manifest():
    return _decode_manifest(bpy.data.texts["TC167_AUTHORING_MANIFEST.json"].as_string())


def bags(action, node):
    slot = slot_for(action, node)
    if slot is None:
        return []
    return [
        bag
        for layer in action.layers
        for strip in layer.strips
        if (bag := strip.channelbag(slot)) is not None
    ]


def channels(action, node):
    return {(c.data_path, c.array_index): c for bag in bags(action, node) for c in bag.fcurves}


def reference_component(node, path, index):
    return Matrix(manifest()["rest"][node]).decompose()[list(PATHS).index(path)][index]


def pose_at(action, node, frame, reference=None):
    curves = channels(action, node)
    if reference is None:
        reference = dict(zip(PATHS, Matrix(manifest()["rest"][node]).decompose()))
    return {
        path: [
            curves[path, i].evaluate(frame) if (path, i) in curves else fallback
            for i, fallback in enumerate(reference[path])
        ]
        for path in PATHS
    }


def values(action, node, path, frame):
    return pose_at(action, node, frame)[path]


def difference(path, left, right):
    if not all(math.isfinite(v) for v in (*left, *right)):
        return float("inf")
    if path == "rotation_quaternion":
        a, b = Quaternion(left), Quaternion(right)
        if a.magnitude < 1e-10 or b.magnitude < 1e-10:
            return float("inf")
        return min(
            max(abs(x - y) for x, y in zip(a.normalized(), b.normalized())),
            max(abs(x + y) for x, y in zip(a.normalized(), b.normalized())),
        )
    return max(abs(x - y) for x, y in zip(left, right))


def ensure_curve(action, node, path, index):
    existing = channels(action, node)
    if (path, index) in existing:
        return existing[path, index]
    slot = next((s for s in action.slots if s.identifier == "OB" + node), None)
    if slot is None:
        slot = action.slots.new(id_type="OBJECT", name=node)
        ensure_export_binding(bpy.data.objects[node], action, slot)
    layer = action.layers[0] if action.layers else action.layers.new("Configuration endpoints")
    strip = layer.strips[0] if layer.strips else layer.strips.new(type="KEYFRAME")
    bag = strip.channelbag(slot, ensure=True)
    return bag.fcurves.new(path, index=index)


def write_key(curve, frame, value):
    key = next((k for k in curve.keyframe_points if abs(k.co.x - frame) < 1e-5), None)
    if key:
        # Keep interpolation, handle types/positions and all interior keys unchanged.
        key.co.y = value
    else:
        key = curve.keyframe_points.insert(frame, value, options={"FAST"})
        key.interpolation = "LINEAR"
        # FAST insertion is sorted and deliberately keeps neighboring handles intact.


def slot_for(action, node):
    return next((slot for slot in action.slots if slot.identifier == "OB" + node), None)


def basis_from_pose(pose):
    return Matrix.LocRotScale(
        Vector(pose["location"]), Quaternion(pose["rotation_quaternion"]), Vector(pose["scale"])
    )


def reference_action(name):
    return bpy.data.actions[CONFIGURATIONS["BATTLEDROID"]] if name.startswith("BD_") else None


def choose(name, editing=True):
    """Reset all managed components before evaluating the selected Action."""
    if bpy.context.mode != "OBJECT":
        raise ValueError("Return to Object Mode before switching Actions")
    action = bpy.data.actions[name]
    scene = bpy.context.scene
    scene.tool_settings.use_keyframe_insert_auto = False
    configuration = next((cfg for cfg, clip in CONFIGURATIONS.items() if clip == name), None)
    reference = reference_action(name)
    for node, rest in manifest()["rest"].items():
        obj = scene.objects.get(node)
        if obj is None:
            continue
        slot = slot_for(action, node)
        animation = obj.animation_data
        if slot or animation:
            animation = obj.animation_data_create()
            animation.action = None
            animation.use_nla = False
        obj.rotation_mode = "QUATERNION"
        fallback = (
            basis_from_pose(pose_at(reference, node, reference.frame_start))
            if reference
            else Matrix(rest)
        )
        obj.matrix_basis = fallback
        if editing and slot:
            animation.action = action
            animation.action_slot = slot
    if configuration:
        scene["tc167_configuration"] = configuration
    scene["tc167_action"] = name
    scene["tc167_editing"] = editing
    scene["tc167_dirty"] = False
    scene.frame_start, scene.frame_end = int(action.frame_start), int(action.frame_end)
    scene.frame_set(int(action.frame_start))


def display_preview(scene, _depsgraph=None):
    """Evaluate a read-only preview without assigning editable Actions."""
    if scene.get("tc167_editing", True):
        return
    action = bpy.data.actions.get(scene.get("tc167_action", ""))
    if action is None:
        return
    frame = (
        action.frame_start if action.name in CONFIGURATIONS.values() else scene.frame_current_final
    )
    for node in manifest()["rest"]:
        if obj := scene.objects.get(node):
            basis = sampled_basis(action, node, frame)
            for path, value in basis.items():
                setattr(obj, path, value)


def sampled_basis(action, node, frame):
    reference = reference_action(action.name)
    defaults = pose_at(reference, node, reference.frame_start) if reference else None
    return pose_at(action, node, frame, defaults)


def pending_transforms(update=True):
    scene = bpy.context.scene
    action = bpy.data.actions.get(scene.get("tc167_action", ""))
    if not scene.get("tc167_editing") or not action:
        return []
    if update:
        bpy.context.view_layer.update()
    result = []
    for node in manifest()["editable"]:
        obj = scene.objects.get(node)
        if obj and any(
            difference(path, getattr(obj, path), expected) > TOLERANCE
            for path, expected in sampled_basis(action, node, scene.frame_current_final).items()
        ):
            result.append(node)
    return result


def current_configuration():
    return bpy.context.scene.get("tc167_configuration", "FIGHTER")


def configuration_snapshot(config):
    """Evaluate fixed hierarchy and canonical curves without rebinding scene Actions."""
    action = bpy.data.actions[CONFIGURATIONS[config]]
    objects = bpy.context.scene.objects
    world = {}

    def matrix(obj):
        if obj.name not in world:
            basis = (
                basis_from_pose(pose_at(action, obj.name, action.frame_start))
                if obj.name in manifest()["rest"]
                else obj.matrix_basis.copy()
            )
            parent = (
                matrix(obj.parent) @ obj.matrix_parent_inverse if obj.parent else Matrix.Identity(4)
            )
            world[obj.name] = parent @ basis
        return world[obj.name]

    return {
        node: {
            **pose_at(action, node, action.frame_start),
            "parent": objects[node].parent.name if objects[node].parent else None,
            "world": [list(row) for row in matrix(objects[node])],
        }
        for node in manifest()["editable"]
    }


def matrix_difference(left, right):
    return max(abs(x - y) for a, b in zip(left, right) for x, y in zip(a, b))


def comparison():
    snapshots = {cfg: configuration_snapshot(cfg) for cfg in CONFIGURATIONS}
    differences = {}
    for left, right in itertools.combinations(snapshots, 2):
        pairs = {}
        for node, a in snapshots[left].items():
            b = snapshots[right][node]
            fields = [path for path in PATHS if difference(path, a[path], b[path]) > TOLERANCE]
            if a["parent"] != b["parent"]:
                fields.append("parent")
            if matrix_difference(a["world"], b["world"]) > TOLERANCE:
                fields.append("world")
            pairs[node] = fields
        differences[f"{left} / {right}"] = pairs
    return {"poses": snapshots, "differences": differences}


def check_assignment(action, node):
    obj = bpy.context.scene.objects.get(node)
    if not obj or not obj.animation_data or obj.animation_data.action != action:
        return False
    slot = slot_for(action, node)
    return bool(
        slot
        and obj.animation_data.action_slot == slot
        and not obj.animation_data.use_nla
        and obj.rotation_mode == "QUATERNION"
    )


def save_configuration(config):
    """Capture evaluated transforms and transact only the selected canonical Action."""
    action = bpy.data.actions[CONFIGURATIONS[config]]
    scene = bpy.context.scene
    if (
        bpy.context.mode != "OBJECT"
        or scene.get("tc167_action") != action.name
        or not scene.get("tc167_editing")
        or scene.frame_current_final != action.frame_start
    ):
        raise ValueError("Select the configuration and enter Edit Pose at its first frame")
    errors = hierarchy_errors()
    if errors:
        raise ValueError(str(errors[0]))
    if unregistered():
        raise ValueError(
            "Initialize unregistered articulated objects before saving: "
            + ", ".join(obj.name for obj in unregistered())
        )
    for node in manifest()["editable"]:
        if not check_assignment(action, node):
            raise ValueError("Select the configuration again; incorrect Action or slot on " + node)
    for name in CONFIGURATIONS.values():
        validate_channels(bpy.data.actions[name], require_complete=True)
    before = {cfg: configuration_snapshot(cfg) for cfg in CONFIGURATIONS}
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    captured = {}
    for node in manifest()["editable"]:
        obj = bpy.data.objects[node]
        evaluated = obj.evaluated_get(depsgraph)
        pose = {path: list(getattr(evaluated, path)) for path in PATHS}
        if not all(math.isfinite(v) for components in pose.values() for v in components):
            raise ValueError(f"{node}: non-finite evaluated transform")
        quaternion = Quaternion(pose["rotation_quaternion"])
        if quaternion.magnitude < 1e-10:
            raise ValueError(f"{node}: zero rotation quaternion")
        quaternion.normalize()
        reference = Quaternion(before[config][node]["rotation_quaternion"])
        if reference.dot(quaternion) < 0:
            quaternion.negate()
        pose["rotation_quaternion"] = list(quaternion)
        captured[node] = pose
    changes = []
    for node, pose in captured.items():
        for path, target in pose.items():
            old = before[config][node][path]
            if difference(path, old, target) <= TOLERANCE:
                continue
            changes.append(
                {
                    "configuration": config,
                    "node": node,
                    "channel": path,
                    "before": old,
                    "after": target,
                }
            )
    edits = []
    for change in changes:
        curves = channels(action, change["node"])
        for i, value in enumerate(change["after"]):
            edits.extend(
                (key, key.co.y, value)
                for key in curves[change["channel"], i].keyframe_points
                if abs(key.co.y - value) > TOLERANCE
            )
    try:
        for key, old, value in edits:
            key.co.y = value
        after = {cfg: configuration_snapshot(cfg) for cfg in CONFIGURATIONS}
        if any(after[cfg] != before[cfg] for cfg in CONFIGURATIONS if cfg != config):
            raise ValueError("Unrelated configuration evaluation changed; save rolled back")
        for node, pose in captured.items():
            if any(
                difference(path, pose[path], after[config][node][path]) > TOLERANCE
                for path in PATHS
            ):
                raise ValueError(f"{node}: saved pose did not match the captured evaluation")
    except Exception:
        for key, old, value in edits:
            key.co.y = old
        raise
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    scene["tc167_dirty"] = False
    return changes


def validate_channels(action, require_complete=False, nodes=None):
    if len(action.layers) > 1 or any(len(layer.strips) > 1 for layer in action.layers):
        raise ValueError(f"{action.name}: use one keyframe strip for rigid-object authoring")
    for node in manifest()["editable"] if nodes is None else nodes:
        node_bags = bags(action, node)
        present = channels(action, node)
        if sum(len(bag.fcurves) for bag in node_bags) != len(present):
            raise ValueError(f"{action.name}: duplicate animation channels for {node}")
        for (path, index), curve in present.items():
            if (
                path not in PATHS
                or index >= PATHS[path]
                or curve.mute
                or (curve.group is not None and curve.group.mute)
                or curve.modifiers
                or not curve.keyframe_points
                or any(not math.isfinite(v) for key in curve.keyframe_points for v in key.co)
            ):
                raise ValueError(f"{action.name}: unsupported channel {node} {path}[{index}]")
        if require_complete and len(present) != sum(PATHS.values()):
            raise ValueError(f"{action.name}: incomplete TRS for {node}; inspect migration report")


def preview(config):
    """Pure read: full proposed changes; no assignments, frame changes or key writes."""
    errors = hierarchy_errors()
    if errors:
        raise ValueError(str(errors[0]))
    if unregistered():
        raise ValueError("Initialize unregistered articulated objects before synchronizing")
    changes = []
    pose = bpy.data.actions[CONFIGURATIONS[config]]
    validate_channels(pose, require_complete=True)
    for name, ends in RELATIONSHIPS.items():
        action = bpy.data.actions[name]
        if config not in ends:
            continue
        validate_channels(action)
        for endpoint, configuration, frame in (
            ("first", ends[0], action.frame_start),
            ("last", ends[1], action.frame_end),
        ):
            if configuration != config:
                continue
            for node in manifest()["editable"]:
                present = channels(action, node)
                for path, width in PATHS.items():
                    before = values(action, node, path, frame)
                    target = values(pose, node, path, pose.frame_start)
                    if path == "rotation_quaternion":
                        target = list(Quaternion(target).normalized())
                        # Align to the nearest surviving interior key's sign.
                        interior = sorted(
                            {
                                k.co.x
                                for (p, i), c in present.items()
                                if p == path
                                for k in c.keyframe_points
                                if action.frame_start < k.co.x < action.frame_end
                            }
                        )
                        neighbor = interior[0 if endpoint == "first" else -1] if interior else frame
                        if (
                            Quaternion(values(action, node, path, neighbor)).dot(Quaternion(target))
                            < 0
                        ):
                            target = [-v for v in target]
                    missing = [i for i in range(width) if (path, i) not in present]
                    delta = difference(path, before, target)
                    absent_key = any(
                        not any(abs(k.co.x - frame) < 1e-5 for k in curve.keyframe_points)
                        for (p, i), curve in present.items()
                        if p == path
                    )
                    sign_change = (
                        path == "rotation_quaternion"
                        and Quaternion(before).dot(Quaternion(target)) < 0
                    )
                    if missing or absent_key or sign_change or delta > TOLERANCE:
                        changes.append(
                            {
                                "configuration": config,
                                "action": name,
                                "node": node,
                                "channel": path,
                                "endpoint": endpoint,
                                "frame": frame,
                                "before": before,
                                "after": target,
                                "difference": delta,
                                "added_components": missing,
                            }
                        )
    return changes


def apply_changes(changes):
    """Apply an already reviewed plan. UI operator checks freshness before calling."""
    for change in changes:
        action = bpy.data.actions[change["action"]]
        for i, value in enumerate(change["after"]):
            curve = ensure_curve(action, change["node"], change["channel"], i)
            if i in change["added_components"]:
                # A new curve needs both endpoints; preserve the other configuration.
                other = "last" if change["endpoint"] == "first" else "first"
                other_frame = action.frame_start if other == "first" else action.frame_end
                write_key(
                    curve,
                    other_frame,
                    reference_component(change["node"], change["channel"], i),
                )
            write_key(curve, change["frame"], value)
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)


def motion_warnings():
    result = []
    for name in RELATIONSHIPS:
        action = bpy.data.actions[name]
        for node in manifest()["editable"]:
            curves = channels(action, node)
            for path in PATHS:
                times = sorted(
                    {k.co.x for (p, i), c in curves.items() if p == path for k in c.keyframe_points}
                )
                for first, last in zip(times, times[1:]):
                    a, b = values(action, node, path, first), values(action, node, path, last)
                    if path == "rotation_quaternion":
                        qa, qb = Quaternion(a), Quaternion(b)
                        if qa.magnitude < 1e-10 or qb.magnitude < 1e-10:
                            continue
                        angle = math.degrees(qa.rotation_difference(qb).angle)
                        angle = min(angle, 360 - angle)
                        reason = "quaternion sign discontinuity" if qa.dot(qb) < 0 else ""
                        if angle > 45:
                            reason += f" rotation step {angle:.3f} degrees"
                    else:
                        delta = max(abs(x - y) for x, y in zip(a, b))
                        reason = (
                            f"{path} step {delta:.5g}"
                            if delta > (0.5 if path == "location" else 0.25)
                            else ""
                        )
                        if path == "scale" and any(x * y <= 0 for x, y in zip(a, b)):
                            reason += " scale zero crossing"
                    if reason:
                        result.append(
                            {
                                "action": name,
                                "node": node,
                                "channel": path,
                                "frames": [first, last],
                                "warning": reason.strip(),
                            }
                        )
    return result


def intersection_warnings():
    """Conservative AABB broad phase on major shells at clip ends/midpoint; no scene mutation."""
    shells = [
        "FuselageSurface1",
        "LeftWingSurface6",
        "RightWingSurface19",
        "LeftUpperLegSurface11",
        "RightUpperLegSurface11.001",
    ]
    result = []
    data = manifest()
    for name in RELATIONSHIPS:
        action = bpy.data.actions[name]
        for frame in (
            action.frame_start,
            (action.frame_start + action.frame_end) / 2,
            action.frame_end,
        ):
            worlds = {}

            def world(node):
                if node not in worlds:
                    obj = bpy.data.objects[node]
                    local = obj.matrix_parent_inverse @ Matrix.LocRotScale(
                        Vector(values(action, node, "location", frame)),
                        Quaternion(values(action, node, "rotation_quaternion", frame)),
                        Vector(values(action, node, "scale", frame)),
                    )
                    worlds[node] = (
                        world(obj.parent.name) @ local
                        if obj.parent and obj.parent.name in data["rest"]
                        else local
                    )
                return worlds[node]

            bounds = {}
            for node in shells:
                if node not in bpy.data.objects:
                    continue
                points = [world(node) @ Vector(p) for p in bpy.data.objects[node].bound_box]
                bounds[node] = (
                    [min(p[i] for p in points) for i in range(3)],
                    [max(p[i] for p in points) for i in range(3)],
                )
            for a, b in itertools.combinations(bounds, 2):
                if all(
                    min(bounds[a][1][i], bounds[b][1][i]) - max(bounds[a][0][i], bounds[b][0][i])
                    > 0.01
                    for i in range(3)
                ):
                    result.append(
                        {
                            "action": name,
                            "frame": frame,
                            "nodes": [a, b],
                            "warning": "major-shell AABB overlap; inspect visually "
                            "(not exact collision)",
                        }
                    )
    return result


def validate_legacy():
    """Read-only channel, pose and structural validation against saved baseline."""
    errors = []
    data = manifest()
    for obj in bpy.data.objects:
        if obj.name not in data["structure"]:
            errors.append({"node": obj.name, "error": "unexpected object/rig in hierarchy"})
    for text, digest in data["metadata_hashes"].items():
        if hashlib.sha256(bpy.data.texts[text].as_string().encode()).hexdigest() != digest:
            errors.append({"metadata": text, "error": "rest/baseline metadata changed"})
    for node, baseline in data["structure"].items():
        obj = bpy.data.objects.get(node)
        if not obj:
            errors.append({"node": node, "error": "missing object"})
            continue
        if (obj.parent.name if obj.parent else None) != baseline["parent"] or [
            list(r) for r in obj.matrix_parent_inverse
        ] != baseline["parent_inverse"]:
            errors.append({"node": node, "error": "hierarchy/pivot parent inverse changed"})
        if (
            obj.type == "ARMATURE"
            or obj.constraints
            or (obj.animation_data and obj.animation_data.drivers)
        ):
            errors.append({"node": node, "error": "unsupported rig/constraint/driver"})
        if (
            list(obj.delta_location) != [0, 0, 0]
            or list(obj.delta_scale) != [1, 1, 1]
            or any(obj.delta_rotation_euler)
        ):
            errors.append({"node": node, "error": "delta transform changes pivot contract"})
    # Mesh-origin moves and applied transforms change local geometry, caught here.
    for node, expected in data["mesh_hashes"].items():
        obj = bpy.data.objects.get(node)
        if obj and obj.type == "MESH":
            actual = hashlib.sha256(
                repr(
                    (
                        [list(v.co) for v in obj.data.vertices],
                        [list(p.vertices) for p in obj.data.polygons],
                    )
                ).encode()
            ).hexdigest()
            if actual != expected:
                errors.append({"node": node, "error": "mesh geometry/origin changed"})
    for name in list(RELATIONSHIPS) + list(CONFIGURATIONS.values()):
        action = bpy.data.actions.get(name)
        if action is None:
            errors.append({"action": name, "error": "missing Action"})
            continue
        for node in data["editable"]:
            present = channels(action, node)
            for path, width in PATHS.items():
                missing = [i for i in range(width) if (path, i) not in present]
                if missing:
                    errors.append(
                        {
                            "action": name,
                            "node": node,
                            "channel": path,
                            "error": "missing channel components",
                            "components": missing,
                        }
                    )
        for slot in action.slots:
            node = slot.identifier.removeprefix("OB")
            if node not in data["rest"]:
                errors.append({"action": name, "node": node, "error": "unexpected slot target"})
                continue
            present = channels(action, node)
            for (path, index), curve in present.items():
                if path not in PATHS or index >= PATHS.get(path, 0):
                    errors.append(
                        {
                            "action": name,
                            "node": node,
                            "channel": path,
                            "error": "unexpected channel",
                        }
                    )
                for key in curve.keyframe_points:
                    if not all(
                        math.isfinite(v) for v in (*key.co, *key.handle_left, *key.handle_right)
                    ):
                        errors.append(
                            {
                                "action": name,
                                "node": node,
                                "channel": path,
                                "frame": key.co.x,
                                "error": "non-finite key/handle",
                            }
                        )
                if (
                    node not in data["editable"]
                    and max(
                        (
                            abs(k.co.y - curve.keyframe_points[0].co.y)
                            for k in curve.keyframe_points
                        ),
                        default=0,
                    )
                    > TOLERANCE
                ):
                    errors.append(
                        {
                            "action": name,
                            "node": node,
                            "channel": path,
                            "error": "unexpected decorative animation",
                        }
                    )
            times = {k.co.x for c in present.values() for k in c.keyframe_points}
            for frame in times:
                q = values(action, node, "rotation_quaternion", frame)
                if not all(math.isfinite(v) for v in q) or abs(Quaternion(q).magnitude - 1) > 0.002:
                    errors.append(
                        {
                            "action": name,
                            "node": node,
                            "channel": "rotation_quaternion",
                            "frame": frame,
                            "error": "quaternion not normalized",
                        }
                    )
        if name in RELATIONSHIPS:
            for endpoint, config, frame in (
                ("first", RELATIONSHIPS[name][0], action.frame_start),
                ("last", RELATIONSHIPS[name][1], action.frame_end),
            ):
                pose = bpy.data.actions.get(CONFIGURATIONS[config])
                if pose is None:
                    continue
                for node in data["rest"]:
                    for path in PATHS:
                        delta = difference(
                            path,
                            values(action, node, path, frame),
                            values(pose, node, path, pose.frame_start),
                        )
                        if delta > TOLERANCE:
                            errors.append(
                                {
                                    "action": name,
                                    "node": node,
                                    "channel": path,
                                    "frame": frame,
                                    "endpoint": endpoint,
                                    "difference": delta,
                                    "error": "configuration endpoint mismatch",
                                }
                            )
    try:
        warnings = motion_warnings() + intersection_warnings()
    except (ValueError, KeyError, ZeroDivisionError) as error:
        warnings = [{"warning": "Motion analysis incomplete: " + str(error)}]
    return {"errors": errors, "warnings": warnings}


def publish(title, payload):
    text = bpy.data.texts.get("TC167_Last_Report.json") or bpy.data.texts.new(
        "TC167_Last_Report.json"
    )
    text.clear()
    text.write(json.dumps({"operation": title, "result": payload}, indent=2))


class TC167_OT_clip(bpy.types.Operator):
    bl_idname = "tc167.clip"
    bl_label = "Select TC167 Action"
    bl_options = {"UNDO"}
    action_name: bpy.props.StringProperty()
    editing: bpy.props.BoolProperty(default=False)

    def invoke(self, context, event):
        if context.mode == "OBJECT":
            if pending_transforms():
                return context.window_manager.invoke_confirm(
                    self,
                    event,
                    title="Discard unsaved transforms?",
                    message="Cancel to save the pose or key the animation first.",
                )
            if unregistered() and self.action_name != context.scene.get("tc167_action"):
                return context.window_manager.invoke_confirm(
                    self,
                    event,
                    title="Switch with unregistered parts?",
                    message="Unregistered parts retain their current layout until initialized.",
                )
        return self.execute(context)

    def execute(self, context):
        try:
            choose(self.action_name, editing=self.editing)
        except (ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class TC167_OT_save_configuration(bpy.types.Operator):
    bl_idname = "tc167.save_configuration"
    bl_label = "Save Configuration Pose"
    bl_options = {"REGISTER", "UNDO"}

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(
            self,
            event,
            title=f"Overwrite {current_configuration()} pose?",
            message="Save current Object Mode transforms to the selected configuration Action?",
        )

    def execute(self, context):
        try:
            changes = save_configuration(current_configuration())
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        updated = sorted({change["node"] for change in changes})
        publish("Saved configuration channels", {"objects": updated, "changes": changes})
        self.report({"INFO"}, f"Saved {len(updated)} objects. See TC167_Last_Report.json")
        return {"FINISHED"}


class TC167_OT_preview(bpy.types.Operator):
    bl_idname = "tc167.preview_endpoints"
    bl_label = "Preview Endpoint Changes"

    def execute(self, context):
        try:
            changes = preview(current_configuration())
        except (ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        publish("Endpoint preview", changes)
        self.report({"INFO"}, f"{len(changes)} proposed changes in TC167_Last_Report.json")
        return {"FINISHED"}


class TC167_OT_sync(bpy.types.Operator):
    bl_idname = "tc167.sync_endpoints"
    bl_label = "Sync Transformation Endpoints"
    bl_options = {"REGISTER", "UNDO"}
    confirmed: bpy.props.BoolProperty(default=False, options={"HIDDEN", "SKIP_SAVE"})

    def invoke(self, context, event):
        self._config = current_configuration()
        try:
            self._plan = preview(self._config)
        except (ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self._signature = json.dumps(self._plan, sort_keys=True)
        publish("Confirm endpoint synchronization", self._plan)
        self.confirmed = False
        return context.window_manager.invoke_props_dialog(self, width=780)

    def draw(self, context):
        layout = self.layout
        layout.label(text=f"Configuration: {self._config} — {len(self._plan)} node/channel changes")
        layout.label(text="Full numerical preview: Text Editor → TC167_Last_Report.json")
        for change in self._plan[:14]:
            layout.label(
                text=f"{change['action']} | {change['endpoint']} {change['frame']:g} | "
                f"{change['node']} {change['channel']} Δ={change['difference']:.5g}"
            )
        layout.label(text="Curves adjoining a changed endpoint will evaluate differently.")
        layout.prop(
            self, "confirmed", text="Apply these endpoint changes (interior keys remain unchanged)"
        )

    def execute(self, context):
        if not self.confirmed or not hasattr(self, "_plan"):
            self.report({"WARNING"}, "Use the confirmation dialog; no keys changed")
            return {"CANCELLED"}
        try:
            fresh = preview(self._config)
        except (ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        if (
            current_configuration() != self._config
            or json.dumps(fresh, sort_keys=True) != self._signature
        ):
            self.report({"ERROR"}, "Preview is stale; preview again")
            return {"CANCELLED"}
        apply_changes(self._plan)
        publish(
            "Applied endpoint synchronization",
            {
                "changes": self._plan,
                "warnings": [
                    "Changing an endpoint changes interpolation on the adjoining segment. "
                    "Existing interior keys and stored handles were preserved.",
                    *motion_warnings(),
                ],
            },
        )
        self.report(
            {"INFO"}, f"Synchronized {len(self._plan)} groups; inspect motion warnings in report"
        )
        return {"FINISHED"}


class TC167_OT_validate(bpy.types.Operator):
    bl_idname = "tc167.validate_configuration"
    bl_label = "Validate Configuration"
    endpoints: bpy.props.BoolProperty(default=False)

    def execute(self, context):
        if self.endpoints:
            result = validate()
        else:
            result = {
                "errors": hierarchy_errors(),
                "warnings": geometry_warnings(),
                "unregistered": [obj.name for obj in unregistered()],
            }
            for name in CONFIGURATIONS.values():
                try:
                    validate_channels(bpy.data.actions[name], require_complete=True)
                except (ValueError, KeyError) as error:
                    result["errors"].append({"action": name, "error": str(error)})
            if not result["errors"]:
                result.update(comparison())
        publish("Endpoint validation" if self.endpoints else "Configuration comparison", result)
        self.report(
            {"WARNING"} if result["errors"] else {"INFO"},
            f"{len(result['errors'])} errors, {len(result['warnings'])} warnings; "
            "TC167_Last_Report.json",
        )
        return {"FINISHED"}


# M21.4: references and evolving hierarchies. This remains self-contained in Blender.
def reference_objects():
    return [
        o
        for o in bpy.data.objects
        if o.name == "TC_ROOT"
        or o.name.startswith(
            (
                "TC_GROUND_",
                "TC_CONTACT_",
                "TC_THRUST_",
                "TC_BODY_",
                "TC_HULL_",
                "TC_MUZZLE_",
                "TC_HARDPOINT_",
                "TC_CAMERA",
                "TC_COCKPIT",
            )
        )
    ]


def create_references():
    """Seed the original asset once. Existing artist-positioned references are untouched."""
    root = bpy.data.objects.get("TC_ROOT")
    if root is None:
        root = bpy.data.objects.new("TC_ROOT", None)
        bpy.context.scene.collection.objects.link(root)
        for obj in list(bpy.context.scene.objects):
            if obj is not root and obj.parent is None:
                obj.parent = root

    def marker(name, parent, position):
        if name in bpy.data.objects:
            return bpy.data.objects[name]
        obj = bpy.data.objects.new(name, None)
        bpy.context.scene.collection.objects.link(obj)
        obj.parent = parent
        obj.location = position
        obj.empty_display_type = "SPHERE"
        obj.empty_display_size = 0.14
        obj.show_in_front = True
        return obj

    # Blender Z-up coordinates. These are explicit original-vehicle physical references,
    # not a geometry-inference policy for arbitrary future redesigns.
    for suffix, p in [("LEFT", (2, 4, -3)), ("RIGHT", (-2, 4, -3)), ("NOSE", (0, -6, -3))]:
        marker("TC_CONTACT_FIGHTER_" + suffix, root, p)
    initial_config = current_configuration()
    for mode in ("VTOL", "BATTLEDROID"):
        choose(CONFIGURATIONS[mode])
        bpy.context.view_layer.update()
        for side in ("Left", "Right"):
            foot = bpy.data.objects.get(side + "Foot")
            if foot:
                points = [
                    child.matrix_world @ v.co
                    for child in foot.children_recursive
                    if child.type == "MESH"
                    for v in child.data.vertices
                ]
                if points:
                    z = min(v.z for v in points)
                    sole = [v for v in points if abs(v.z - z) < 1e-4]
                    center = sum(sole, Vector()) / len(sole)
                    local = foot.matrix_world.inverted() @ center
                    marker("TC_CONTACT_" + mode + "_" + side.upper(), foot, local)
    choose(CONFIGURATIONS[initial_config])
    for side in ("Left", "Right"):
        engine = bpy.data.objects.get(side + "Engine")
        if engine:
            marker("TC_THRUST_" + side.upper(), engine, (0, 0, 0))
    for mode in CONFIGURATIONS:
        # Explicit conservative body underside corners; artists must review for redesigns.
        for i, (x, y) in enumerate(((-2, -4), (-2, 4), (2, -4), (2, 4))):
            marker(f"TC_BODY_{mode}_{i}", root, (x, y, -1.1))
        choose(CONFIGURATIONS[mode])
        bpy.context.view_layer.update()
        supports = [
            o.matrix_world.translation.z
            for o in reference_objects()
            if o.name.startswith("TC_CONTACT_" + mode + "_")
        ]
        marker("TC_GROUND_" + mode, root, (0, 0, min(supports)))
    # Explicit original-vehicle obstacle envelopes; edit these after a redesign.
    hulls = {
        "FIGHTER": ((-6, -10.5, -1.1), (6, 10.5, 3.3)),
        "VTOL": ((-6, -10.5, -6.0), (6, 10.5, 3.3)),
        "BATTLEDROID": ((-5, -4.5, -3.21), (5, 4.5, 7.6)),
    }
    for mode, (low, high) in hulls.items():
        for i, corner in enumerate(itertools.product(*zip(low, high))):
            marker(f"TC_HULL_{mode}_{i}", root, corner)
    choose(CONFIGURATIONS[initial_config])
    return root


def write_manifest(data):
    text = bpy.data.texts["TC167_AUTHORING_MANIFEST.json"]
    text.clear()
    text.write(json.dumps(data))
    _decode_manifest.cache_clear()


def capture_structure(obj):
    return {
        "parent": obj.parent.name if obj.parent else None,
        "parent_inverse": [list(row) for row in obj.matrix_parent_inverse],
        "type": obj.type,
        "identity": uuid.uuid4().hex,
    }


def install_identity_baseline():
    """Record identity only. Preserve all Actions, pivots, and mesh datablocks."""
    data = json.loads(json.dumps(manifest()))
    for node in data["rest"]:
        obj = bpy.context.scene.objects.get(node)
        if obj is not None and "identity" not in data["structure"][node]:
            identity = uuid.uuid4().hex
            data["structure"][node]["identity"] = identity
            obj["tc167_identity"] = identity
    root = bpy.context.scene.objects.get("TC167Root")
    baseline = data["structure"].get("TC167Root")
    if (
        root
        and root.parent
        and root.parent.name == "TC_ROOT"
        and baseline
        and baseline["parent"] is None
        and baseline["parent_inverse"] == [list(row) for row in root.matrix_parent_inverse]
    ):
        # M21.4 introduced TC_ROOT after the legacy manifest had been generated.
        baseline["parent"] = "TC_ROOT"
    data["version"] = 2
    write_manifest(data)


def unregistered():
    known = manifest()["rest"]
    references = set(reference_objects())
    return [
        obj
        for obj in bpy.context.scene.objects
        if obj.name not in known
        and obj not in references
        and obj.type in {"MESH", "EMPTY"}
        and not (obj.type == "EMPTY" and obj.empty_display_type == "IMAGE")
    ]


def hierarchy_errors():
    objects = bpy.context.scene.objects
    errors = []
    for node in manifest()["rest"]:
        baseline = manifest()["structure"][node]
        obj = objects.get(node)
        identity = baseline.get("identity")
        if not obj or (identity and obj.get("tc167_identity") != identity):
            matches = [o.name for o in objects if identity and o.get("tc167_identity") == identity]
            errors.append(
                {
                    "node": node,
                    "error": "Missing or renamed registered object",
                    "identity_matches": matches,
                }
            )
            continue
        parent = obj.parent.name if obj.parent else None
        if (
            parent != baseline["parent"]
            or [list(row) for row in obj.matrix_parent_inverse] != baseline["parent_inverse"]
        ):
            errors.append(
                {
                    "node": node,
                    "error": "Hierarchy or pivot changed",
                    "saved_parent": baseline["parent"],
                    "current_parent": parent,
                }
            )
        if (
            obj.constraints
            or (obj.animation_data and obj.animation_data.drivers)
            or obj.parent_type != "OBJECT"
            or any(obj.delta_location)
            or any(obj.delta_rotation_euler)
            or any(obj.delta_rotation_quaternion[1:])
            or tuple(obj.delta_scale) != (1, 1, 1)
        ):
            errors.append(
                {"node": node, "error": "Unsupported constraint, driver, parenting or delta TRS"}
            )
    return errors


def mesh_digest(obj):
    return hashlib.sha256(
        repr(
            (
                [list(v.co) for v in obj.data.vertices],
                [list(p.vertices) for p in obj.data.polygons],
            )
        ).encode()
    ).hexdigest()


def geometry_warnings():
    warnings = []
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue
        issues = []
        if obj.data.users > 1:
            issues.append("Shared mesh datablock; Edit Mode affects other instances")
        if obj.data.shape_keys:
            issues.append("Shape keys present; TC167 rigid GLB export omits morph animation")
        if obj.modifiers:
            issues.append("Modifiers may change evaluated geometry; inspect the export")
        if obj.mode == "EDIT":
            issues.append("Edit Mode changes mesh geometry in every configuration")
        expected = manifest()["mesh_hashes"].get(obj.name)
        if expected and expected != mesh_digest(obj):
            issues.append("Mesh geometry or origin changed across all configurations")
        if issues:
            warnings.append({"node": obj.name, "warnings": issues})
    return warnings


def ensure_export_binding(obj, action, slot):
    animation = obj.animation_data_create()
    if not any(
        strip.action == action and strip.action_slot == slot
        for track in animation.nla_tracks
        for strip in track.strips
    ):
        track = animation.nla_tracks.new()
        track.name = action.name + " export binding"
        track.mute = True
        strip = track.strips.new(action.name, int(action.frame_start), action)
        strip.action_slot = slot
    animation.use_nla = False


def register_articulated_objects(objects=None, allow_nonfighter=False):
    """Initialize parts from the explicitly reviewed current local pose."""
    if bpy.context.mode != "OBJECT":
        raise ValueError("Finish separation and return to Object Mode")
    scene = bpy.context.scene
    config = current_configuration()
    if scene.get("tc167_action") != CONFIGURATIONS[config]:
        raise ValueError("Select a configuration before initializing new parts")
    if config != "FIGHTER" and not allow_nonfighter:
        raise ValueError(
            "Use FIGHTER or explicitly confirm copying the current pose to all configurations"
        )
    errors = hierarchy_errors()
    if errors:
        raise ValueError(str(errors[0]))
    candidates = set(unregistered())
    parts = sorted(
        candidates if objects is None else candidates.intersection(objects),
        key=lambda obj: obj.name,
    )
    data = json.loads(json.dumps(manifest()))
    actions = [bpy.data.actions[name] for name in (*CONFIGURATIONS.values(), *RELATIONSHIPS)]
    planned = []
    for obj in parts:
        if (
            obj.parent is None
            or obj.parent.name not in data["rest"]
            and obj.parent not in parts
            and obj.parent.name != "TC_ROOT"
        ):
            raise ValueError(
                f"{obj.name}: parent it to a registered or simultaneously initialized part"
            )
        if (
            obj.constraints
            or obj.parent_type != "OBJECT"
            or any(obj.delta_location)
            or any(obj.delta_rotation_euler)
            or any(obj.delta_rotation_quaternion[1:])
            or tuple(obj.delta_scale) != (1, 1, 1)
            or (obj.animation_data and obj.animation_data.drivers)
        ):
            raise ValueError(f"{obj.name}: unsupported constraint, driver, parenting or delta TRS")
        if len(obj.name.encode()) > 63:
            raise ValueError(f"{obj.name}: name too long for an Action slot")
        basis = obj.matrix_basis.copy()
        location, rotation, scale = basis.decompose()
        pose = dict(zip(PATHS, (location, rotation, scale)))
        if (
            not all(math.isfinite(v) for row in basis for v in row)
            or max(
                abs(x - y) for ar, br in zip(basis, basis_from_pose(pose)) for x, y in zip(ar, br)
            )
            > TOLERANCE
        ):
            raise ValueError(f"{obj.name}: unsupported local transform")
        for action in actions:
            validate_channels(action, nodes=[obj.name])
        data["rest"][obj.name] = [list(row) for row in basis]
        data["structure"][obj.name] = capture_structure(obj)
        data["editable"].append(obj.name)
        if obj.type == "MESH":
            data["mesh_hashes"][obj.name] = mesh_digest(obj)
        planned.append((obj, basis, pose))
    if not planned:
        return []
    write_manifest(data)
    source_names = {
        entry["identity"]: node for node, entry in data["structure"].items() if "identity" in entry
    }
    report = []
    for obj, basis, pose in planned:
        source = source_names.get(obj.get("tc167_identity"))
        obj["tc167_identity"] = data["structure"][obj.name]["identity"]
        obj.rotation_mode = "QUATERNION"
        obj.matrix_basis = basis
        animation = obj.animation_data_create()
        previous_action, previous_slot = animation.action, animation.action_slot
        if (
            previous_action
            and previous_action not in actions
            and previous_slot
            and previous_slot.identifier != "OB" + (source or "")
        ):
            previous_action.use_fake_user = True
        animation.action = None
        # Separate can inherit the source object's NLA bindings. Remove those
        # bindings from this new object so export discovers only its own slots.
        removed_bindings = []
        for track in list(animation.nla_tracks):
            locked = track.lock
            for strip in list(track.strips):
                inherited = (
                    source and strip.action_slot and strip.action_slot.identifier == "OB" + source
                )
                wrong_slot = strip.action in actions and strip.action_slot != slot_for(
                    strip.action, obj.name
                )
                if inherited or wrong_slot:
                    removed_bindings.append(strip.action.name)
                    track.lock = False
                    track.strips.remove(strip)
            if track.strips:
                track.lock = locked
            else:
                animation.nla_tracks.remove(track)
        initialized = {}
        for action in actions:
            present = channels(action, obj.name)
            created = []
            for path, width in PATHS.items():
                for i in range(width):
                    if (path, i) in present:
                        continue
                    curve = ensure_curve(action, obj.name, path, i)
                    for frame in sorted({action.frame_start, action.frame_end}):
                        write_key(curve, frame, pose[path][i])
                    created.append(f"{path}[{i}]")
            ensure_export_binding(obj, action, slot_for(action, obj.name))
            initialized[action.name] = created
        report.append(
            {
                "node": obj.name,
                "parent": obj.parent.name,
                "source": config,
                "initialized_channels": initialized,
                "removed_inherited_bindings": removed_bindings,
            }
        )
    # Preserve pending transforms on already-registered objects.
    action = bpy.data.actions[CONFIGURATIONS[config]]
    if scene.get("tc167_editing"):
        for obj, _, _ in planned:
            obj.animation_data.action = action
            obj.animation_data.action_slot = slot_for(action, obj.name)
    return report


def validate_anchors():
    errors = []
    for name in ["TC_ROOT", *("TC_GROUND_" + m for m in CONFIGURATIONS)]:
        if name not in bpy.data.objects:
            errors.append({"anchor": name, "error": "Create missing reference anchors"})
    for mode in CONFIGURATIONS:
        for prefix in ("TC_CONTACT_", "TC_BODY_", "TC_HULL_"):
            if not any(o.name.startswith(prefix + mode + "_") for o in reference_objects()):
                errors.append(
                    {"anchor": prefix + mode + "_<id>", "error": "Missing physical references"}
                )
    if not any(o.name.startswith("TC_THRUST_") for o in reference_objects()):
        errors.append({"error": "Missing TC_THRUST_<id>"})
    for obj in reference_objects():
        if obj.type != "EMPTY" or (obj.name != "TC_ROOT" and obj.parent is None):
            errors.append({"anchor": obj.name, "error": "Use a parented Empty reference"})
    return errors


def validate():
    """Validate export contracts; geometry and decorative nodes may evolve freely."""
    errors = validate_anchors() + hierarchy_errors()
    errors.extend(
        {"node": obj.name, "error": "Unregistered articulated object"} for obj in unregistered()
    )
    if bpy.context.mode != "OBJECT":
        errors.append({"error": "Return to Object Mode before validating for export"})
    for name in (*CONFIGURATIONS.values(), *RELATIONSHIPS, "BD_Idle", "BD_Walk", "BD_Run"):
        action = bpy.data.actions.get(name)
        if not action or not action.slots:
            errors.append({"action": name, "error": "Missing Action or Action slots"})
            continue
        for slot in action.slots:
            node = slot.identifier.removeprefix("OB")
            if node not in bpy.data.objects:
                errors.append({"action": name, "node": node, "error": "Missing slot target"})
            if node == "TC_ROOT":
                errors.append({"action": name, "error": "TC_ROOT must not be animated"})
            for (path, index), curve in channels(action, node).items():
                if path not in PATHS or index >= PATHS.get(path, 0):
                    errors.append(
                        {"action": name, "node": node, "error": "Unsupported export channel"}
                    )
                if any(not math.isfinite(v) for k in curve.keyframe_points for v in k.co):
                    errors.append({"action": name, "node": node, "error": "Non-finite key"})
    for config in CONFIGURATIONS:
        try:
            for change in preview(config):
                errors.append(
                    {
                        "configuration": config,
                        "error": "Unsynchronized endpoint",
                        "channel": change["channel"],
                        "change": change,
                    }
                )
        except (ValueError, KeyError) as error:
            errors.append({"configuration": config, "error": str(error)})
    return {
        "errors": errors,
        "warnings": geometry_warnings()
        + ["Review TC_BODY collision envelope and contacts after geometry edits."],
    }


class TC167_OT_references(bpy.types.Operator):
    bl_idname = "tc167.references"
    bl_label = "Physical References"
    bl_options = {"REGISTER", "UNDO"}
    operation: bpy.props.StringProperty()

    def execute(self, context):
        op = self.operation
        if op == "CREATE":
            create_references()
        elif op in ("VALIDATE", "GAME"):
            report = validate() if op == "GAME" else {"errors": validate_anchors()}
            publish("Validate for Game" if op == "GAME" else "Validate Anchors", report)
            self.report(
                {"ERROR"} if report["errors"] else {"INFO"},
                f"{len(report['errors'])} errors; see TC167_Last_Report.json",
            )
        elif op == "DISPLAY":
            refs = reference_objects()
            hide = not any(o.hide_get() for o in refs)
            for obj in refs:
                obj.hide_set(hide)
        else:
            bpy.ops.object.select_all(action="DESELECT")
            prefix = {
                "GROUND": "TC_GROUND_" + current_configuration(),
                "CONTACT": "TC_CONTACT_" + current_configuration() + "_",
                "THRUST": "TC_THRUST_",
            }[op]
            for obj in reference_objects():
                if obj.name.startswith(prefix):
                    obj.hide_set(False)
                    obj.select_set(True)
                    context.view_layer.objects.active = obj
        return {"FINISHED"}


class TC167_OT_parts(bpy.types.Operator):
    bl_idname = "tc167.parts"
    bl_label = "Articulated Objects"
    bl_options = {"REGISTER", "UNDO"}
    operation: bpy.props.StringProperty()
    copy_current: bpy.props.BoolProperty(
        name="Use this layout as the initial pose in all three configurations", default=False
    )

    def invoke(self, context, event):
        if self.operation in {"SELECTED", "INITIALIZE"}:
            self.copy_current = False
            return context.window_manager.invoke_props_dialog(self, width=580)
        return self.execute(context)

    def draw(self, context):
        self.layout.label(
            text=f"Initial local transforms come from the current {current_configuration()} layout."
        )
        self.layout.label(text="For a straight fuselage, select FIGHTER and align the parts first.")
        self.layout.prop(self, "copy_current")

    def execute(self, context):
        candidates = unregistered()
        if self.operation in {"SELECTED", "INITIALIZE"}:
            if not self.copy_current:
                self.report({"WARNING"}, "Confirm the initial layout before writing keys")
                return {"CANCELLED"}
            try:
                result = register_articulated_objects(
                    context.selected_objects if self.operation == "SELECTED" else None,
                    allow_nonfighter=True,
                )
            except (ValueError, KeyError) as error:
                self.report({"ERROR"}, str(error))
                return {"CANCELLED"}
        elif self.operation == "HIERARCHY":
            result = hierarchy_errors()
        else:
            result = [
                {"node": obj.name, "parent": obj.parent.name if obj.parent else None}
                for obj in candidates
            ]
            if self.operation == "SHOW" and context.mode == "OBJECT":
                bpy.ops.object.select_all(action="DESELECT")
                for obj in candidates:
                    if obj.name in context.view_layer.objects:
                        obj.hide_set(False)
                        obj.select_set(True)
        publish("Articulated objects " + self.operation, result)
        self.report({"INFO"}, f"{len(result)} entries in TC167_Last_Report.json")
        return {"FINISHED"}


def export_scene_glb(filepath):
    """Export the neutral rigid-object hierarchy and all authored clips."""
    scene = bpy.context.scene
    state = {
        obj.name: (
            obj.matrix_basis.copy(),
            obj.hide_get(),
            obj.select_get(),
            obj.animation_data.action if obj.animation_data else None,
            obj.animation_data.action_slot if obj.animation_data else None,
            obj.animation_data.use_nla if obj.animation_data else False,
        )
        for obj in bpy.context.view_layer.objects
    }
    active = bpy.context.view_layer.objects.active
    frame, subframe = scene.frame_current, scene.frame_subframe
    editing = scene.get("tc167_editing")
    scene["tc167_editing"] = "EXPORT"
    try:
        for obj in bpy.context.view_layer.objects:
            if obj.animation_data:
                obj.animation_data.action = None
                obj.animation_data.use_nla = False
            basis = manifest()["rest"].get(obj.name)
            if basis is not None:
                obj.matrix_basis = Matrix(basis)
            obj.hide_set(False)
        scene.frame_set(0)
        bpy.context.view_layer.update()
        bpy.ops.object.select_all(action="SELECT")
        return bpy.ops.export_scene.gltf(
            filepath=str(filepath),
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
    finally:
        for name, (_, hidden, selected, action, slot, nla) in state.items():
            obj = bpy.context.view_layer.objects[name]
            if obj.animation_data:
                obj.animation_data.action = action
                if slot:
                    obj.animation_data.action_slot = slot
                obj.animation_data.use_nla = nla
            obj.hide_set(hidden)
            obj.select_set(selected)
        bpy.context.view_layer.objects.active = active
        scene.frame_set(frame, subframe=subframe)
        for name, (basis, *_) in state.items():
            bpy.context.view_layer.objects[name].matrix_basis = basis
        if editing is None:
            del scene["tc167_editing"]
        else:
            scene["tc167_editing"] = editing
        bpy.context.view_layer.update()


class TC167_OT_export(bpy.types.Operator):
    bl_idname = "tc167.export_glb"
    bl_label = "Export GLB"
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.glb", options={"HIDDEN"})
    check_existing: bpy.props.BoolProperty(default=True, options={"HIDDEN"})

    def invoke(self, context, event):
        self.filepath = bpy.path.abspath("//tc167_authoring.glb")
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        result = validate()
        if result["errors"] or pending_transforms():
            publish("Export validation", result)
            self.report(
                {"ERROR"},
                "Resolve validation errors and save or discard pending transforms before export",
            )
            return {"CANCELLED"}
        try:
            return export_scene_glb(bpy.path.ensure_ext(self.filepath, ".glb"))
        except (RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}


class TC167_PT_actions(bpy.types.Panel):
    bl_label = "TC167 Authoring"
    bl_idname = "TC167_PT_actions"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "TC167"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        config = current_configuration()
        action_name = scene.get("tc167_action", "")
        editing = scene.get("tc167_editing", False)
        layout.label(text=f"Configuration · {config}")
        layout.label(
            text=("Editing · " if editing else "Viewing · ") + (action_name or "Choose an Action")
        )
        obj = context.active_object
        assigned = obj.animation_data.action if obj and obj.animation_data else None
        layout.label(text="Active object Action · " + (assigned.name if assigned else "None"))
        if editing and any(
            not check_assignment(bpy.data.actions[action_name], node)
            for node in manifest()["editable"]
            if action_name in bpy.data.actions and slot_for(bpy.data.actions[action_name], node)
        ):
            layout.label(text="Action or slot changed outside TC167", icon="ERROR")
        if action_name and scene.tool_settings.use_keyframe_insert_auto:
            layout.label(text="Auto Key is on", icon="ERROR")
        if editing and scene.get("tc167_dirty"):
            layout.label(text="Unsaved Object Mode transforms", icon="ERROR")
        if action_name:
            action = layout.operator(
                "tc167.clip",
                text=(
                    "Return to Preview"
                    if editing
                    else "Edit Pose"
                    if action_name in CONFIGURATIONS.values()
                    else "Edit Animation"
                ),
            )
            action.action_name, action.editing = action_name, not editing

        box = layout.box()
        box.label(text="Configurations")
        row = box.row(align=True)
        for label, name in CONFIGURATIONS.items():
            row.operator("tc167.clip", text=label, depress=config == label).action_name = name
        save = box.row()
        save.enabled = editing and action_name == CONFIGURATIONS[config]
        save.operator("tc167.save_configuration")
        box.operator("tc167.validate_configuration")

        box = layout.box()
        box.label(text="Articulated Objects")
        if count := len(unregistered()):
            box.label(text=f"{count} unregistered parts · pose preview incomplete", icon="ERROR")
        for label, op in (
            ("Detect New Objects", "DETECT"),
            ("Register Selected Objects", "SELECTED"),
            ("Initialize New Parts Across Configurations", "INITIALIZE"),
            ("Show Unregistered Objects", "SHOW"),
            ("Validate Parent Hierarchy", "HIERARCHY"),
        ):
            box.operator("tc167.parts", text=label).operation = op
        if context.mode == "EDIT_MESH":
            box.label(text="Geometry edits affect all configurations.", icon="INFO")

        box = layout.box()
        box.label(text="Transformations")
        for name, ends in RELATIONSHIPS.items():
            box.operator("tc167.clip", text=" → ".join(ends)).action_name = name
        for name in ("BD_Idle", "BD_Walk", "BD_Run"):
            box.operator("tc167.clip", text=name).action_name = name

        box = layout.box()
        box.label(text="Synchronization")
        box.operator("tc167.preview_endpoints")
        box.operator("tc167.sync_endpoints")
        box.operator("tc167.validate_configuration", text="Validate All Endpoints").endpoints = True

        box = layout.box()
        box.label(text="Physical References")
        for label, op in (
            ("Show / Hide Anchors", "DISPLAY"),
            ("Select Ground References", "GROUND"),
            ("Select Contact Points", "CONTACT"),
            ("Select Thrust Anchors", "THRUST"),
            ("Create Missing References", "CREATE"),
            ("Validate Anchors", "VALIDATE"),
        ):
            box.operator("tc167.references", text=label).operation = op

        box = layout.box()
        box.label(text="Playback and Export")
        box.operator("screen.animation_play", text="Play / Pause")
        box.operator("tc167.clip", text="Reset to Configuration").action_name = CONFIGURATIONS[
            config
        ]
        box.operator("tc167.references", text="Validate for Game").operation = "GAME"
        box.operator("tc167.export_glb")


for cls in (
    TC167_OT_clip,
    TC167_OT_save_configuration,
    TC167_OT_preview,
    TC167_OT_sync,
    TC167_OT_validate,
    TC167_OT_references,
    TC167_OT_parts,
    TC167_OT_export,
    TC167_PT_actions,
):
    old = getattr(bpy.types, cls.__name__, None)
    if old:
        bpy.utils.unregister_class(old)
    bpy.utils.register_class(cls)


def track_dirty(scene, depsgraph):
    if (
        bpy.context.mode != "OBJECT"
        or not scene.get("tc167_editing")
        or scene.get("tc167_editing") == "EXPORT"
    ):
        return
    if not any(update.is_updated_transform for update in depsgraph.updates):
        return
    dirty = bool(pending_transforms(update=False))
    if bool(scene.get("tc167_dirty")) != dirty:
        scene["tc167_dirty"] = dirty


for handlers, callback in (
    (bpy.app.handlers.frame_change_post, display_preview),
    (bpy.app.handlers.depsgraph_update_post, track_dirty),
):
    for old in list(handlers):
        if old.__name__ == callback.__name__:
            handlers.remove(old)
    handlers.append(callback)
