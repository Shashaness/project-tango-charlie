"""Self-contained Blender helper embedded into TC167_Authoring.blend (no game imports)."""

import hashlib
import itertools
import json
import math
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
    slots = [s for s in action.slots if s.identifier == "OB" + node]
    return [
        b
        for layer in action.layers
        for strip in layer.strips
        for b in strip.channelbags
        if any(s.handle == b.slot_handle for s in slots)
    ]


def channels(action, node):
    return {(c.data_path, c.array_index): c for bag in bags(action, node) for c in bag.fcurves}


def values(action, node, path, frame):
    basis = Matrix(manifest()["rest"][node])
    loc, quat, scale = basis.decompose()
    fallback = {"location": loc, "rotation_quaternion": quat, "scale": scale}[path]
    curves = channels(action, node)
    return [
        curves[path, i].evaluate(frame) if (path, i) in curves else fallback[i]
        for i in range(PATHS[path])
    ]


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
        # Preserve multi-object exporter discovery for a newly repaired slot.
        obj = bpy.data.objects[node]
        obj.animation_data_create()
        repaired = False
        for track in obj.animation_data.nla_tracks:
            for nla_strip in track.strips:
                if nla_strip.action == action and nla_strip.action_slot is None:
                    nla_strip.action_slot = slot
                    repaired = True
        if not repaired:
            track = obj.animation_data.nla_tracks.new()
            track.name = action.name + " repaired slot"
            track.mute = True
            nla_strip = track.strips.new(action.name, int(action.frame_start), action)
            nla_strip.action_slot = slot
        obj.animation_data.use_nla = False
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


def choose(name):
    action = bpy.data.actions[name]
    for obj in bpy.context.scene.objects:
        node = obj.name
        obj.animation_data_create()
        slot = next((s for s in action.slots if s.identifier == "OB" + node), None)
        obj.animation_data.action = action if slot else None
        if slot:
            obj.animation_data.action_slot = slot
        obj.animation_data.use_nla = False
        if not slot and node in manifest()["rest"]:
            if name.startswith("BD_"):
                pose = bpy.data.actions[CONFIGURATIONS["BATTLEDROID"]]
                obj.matrix_basis = Matrix.LocRotScale(
                    Vector(values(pose, node, "location", pose.frame_start)),
                    Quaternion(values(pose, node, "rotation_quaternion", pose.frame_start)),
                    Vector(values(pose, node, "scale", pose.frame_start)),
                )
            else:
                obj.matrix_basis = Matrix(manifest()["rest"][node])
    for config, pose in CONFIGURATIONS.items():
        if name == pose:
            bpy.context.scene["tc167_configuration"] = config
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = int(action.frame_start), int(action.frame_end)
    scene.frame_set(int(action.frame_start))


def current_configuration():
    return bpy.context.scene.get("tc167_configuration", "FIGHTER")


def save_configuration(config):
    """Capture all managed evaluated bases before touching any Action data."""
    action = bpy.data.actions[CONFIGURATIONS[config]]
    for node in manifest()["editable"]:
        obj = bpy.data.objects[node]
        if (
            not obj.animation_data
            or obj.animation_data.action != action
            or not obj.animation_data.action_slot
            or obj.animation_data.action_slot.identifier != "OB" + node
        ):
            raise ValueError("Select the configuration button before saving its pose")
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    captured = {}
    for node in manifest()["editable"]:
        obj = bpy.data.objects[node]
        basis = obj.matrix_parent_inverse.inverted() @ obj.evaluated_get(depsgraph).matrix_local
        if not all(math.isfinite(v) for row in basis for v in row):
            raise ValueError(node + ": non-finite transform; no configuration was saved")
        loc, quat, scale = basis.decompose()
        reference = Quaternion(values(action, node, "rotation_quaternion", action.frame_start))
        if reference.dot(quat) < 0:
            quat.negate()
        rebuilt = Matrix.LocRotScale(loc, quat, scale)
        if max(abs(basis[i][j] - rebuilt[i][j]) for i in range(4) for j in range(4)) > TOLERANCE:
            raise ValueError(node + ": unsupported shear; no configuration was saved")
        captured[node] = {
            "location": list(loc),
            "rotation_quaternion": list(quat),
            "scale": list(scale),
        }
    changes = []
    for node, pose in captured.items():
        for path, target in pose.items():
            old = values(action, node, path, action.frame_start)
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
    for change in changes:
        for i, value in enumerate(change["after"]):
            curve = ensure_curve(action, change["node"], change["channel"], i)
            times = {action.frame_start, action.frame_end} | {k.co.x for k in curve.keyframe_points}
            for frame in sorted(times):
                write_key(curve, frame, value)
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    return changes


def preview(config):
    """Pure read: full proposed changes; no assignments, frame changes or key writes."""
    changes = []
    pose = bpy.data.actions[CONFIGURATIONS[config]]
    for name, ends in RELATIONSHIPS.items():
        action = bpy.data.actions[name]
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
                    if missing or delta > TOLERANCE:
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
                config = RELATIONSHIPS[action.name][0 if other == "first" else 1]
                pose = bpy.data.actions[CONFIGURATIONS[config]]
                other_frame = action.frame_start if other == "first" else action.frame_end
                write_key(
                    curve,
                    other_frame,
                    values(pose, change["node"], change["channel"], pose.frame_start)[i],
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
    action_name: bpy.props.StringProperty()

    def execute(self, context):
        choose(self.action_name)
        return {"FINISHED"}


class TC167_OT_save_configuration(bpy.types.Operator):
    bl_idname = "tc167.save_configuration"
    bl_label = "Save Configuration Pose"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            changes = save_configuration(current_configuration())
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        publish("Saved configuration channels", changes)
        self.report(
            {"INFO"}, f"Saved {len(changes)} node/channel groups; details: TC167_Last_Report.json"
        )
        return {"FINISHED"}


class TC167_OT_preview(bpy.types.Operator):
    bl_idname = "tc167.preview_endpoints"
    bl_label = "Preview Endpoint Changes"

    def execute(self, context):
        changes = preview(current_configuration())
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
        self._plan = preview(self._config)
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
        layout.prop(
            self, "confirmed", text="Apply these endpoint changes (interior keys remain unchanged)"
        )

    def execute(self, context):
        if not self.confirmed or not hasattr(self, "_plan"):
            self.report({"WARNING"}, "Use the confirmation dialog; no keys changed")
            return {"CANCELLED"}
        if (
            current_configuration() != self._config
            or json.dumps(preview(self._config), sort_keys=True) != self._signature
        ):
            self.report({"ERROR"}, "Preview is stale; preview again")
            return {"CANCELLED"}
        apply_changes(self._plan)
        publish(
            "Applied endpoint synchronization",
            {"changes": self._plan, "warnings": motion_warnings()},
        )
        self.report(
            {"INFO"}, f"Synchronized {len(self._plan)} groups; inspect motion warnings in report"
        )
        return {"FINISHED"}


class TC167_OT_validate(bpy.types.Operator):
    bl_idname = "tc167.validate_configuration"
    bl_label = "Validate Configuration Consistency"

    def execute(self, context):
        result = validate()
        publish("Configuration consistency", result)
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


def register_articulated_objects():
    """Explicitly adopt new objects without renaming or replacing artist animation."""
    data = manifest()
    count = 0
    missing = set(data["rest"]) - set(bpy.data.objects.keys())
    for node in missing:
        data["rest"].pop(node)
    data["editable"] = [n for n in data["editable"] if n not in missing]
    for action in bpy.data.actions:
        for slot in list(action.slots):
            if slot.identifier.removeprefix("OB") in missing:
                action.slots.remove(slot)
    for obj in bpy.context.scene.objects:
        if obj in reference_objects() or obj.name in data["rest"]:
            continue
        basis = obj.matrix_basis.copy()
        obj.rotation_mode = "QUATERNION"
        obj.matrix_basis = basis
        data["rest"][obj.name] = [list(r) for r in basis]
        data["editable"].append(obj.name)
        count += 1
    bpy.data.texts["TC167_AUTHORING_MANIFEST.json"].clear()
    bpy.data.texts["TC167_AUTHORING_MANIFEST.json"].write(json.dumps(data))
    _decode_manifest.cache_clear()
    # Add only missing endpoint channels; keep all existing interior keys and handles.
    for name in (*CONFIGURATIONS.values(), *RELATIONSHIPS):
        action = bpy.data.actions[name]
        for node in manifest()["editable"]:
            for path, width in PATHS.items():
                present = channels(action, node)
                for i in range(width):
                    if (path, i) in present:
                        continue
                    frames = (action.frame_start, action.frame_end)
                    fallback = [values(action, node, path, frame)[i] for frame in frames]
                    curve = ensure_curve(action, node, path, i)
                    for frame, value in zip(frames, fallback):
                        write_key(curve, frame, value)
                    curve.update()
    return count


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
    errors = validate_anchors()
    for obj in bpy.context.scene.objects:
        if (
            obj.type == "ARMATURE"
            or obj.constraints
            or (obj.animation_data and obj.animation_data.drivers)
        ):
            errors.append(
                {"node": obj.name, "error": "Bake constraints/drivers to rigid object TRS"}
            )
        if (
            obj.name not in manifest()["rest"]
            and obj not in reference_objects()
            and obj.animation_data
            and obj.animation_data.action
        ):
            errors.append(
                {
                    "node": obj.name,
                    "error": "Register new articulated objects before saving configurations",
                }
            )
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
        for change in preview(config):
            errors.append(
                {"configuration": config, "error": "Unsynchronized endpoint", "change": change}
            )
    return {
        "errors": errors,
        "warnings": [
            "Review TC_BODY collision envelope and contacts after geometry edits; "
            "references do not validate arbitrary visual collision geometry."
        ],
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
        elif op == "REGISTER":
            register_articulated_objects()
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


class TC167_PT_actions(bpy.types.Panel):
    bl_label = "TC167 Authoring"
    bl_idname = "TC167_PT_actions"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "TC167"

    def draw(self, context):
        layout = self.layout
        layout.label(text="Configurations: " + current_configuration())
        for config, name in CONFIGURATIONS.items():
            layout.operator("tc167.clip", text=config).action_name = name
        layout.operator("tc167.save_configuration")
        layout.operator("tc167.validate_configuration", text="Validate Configuration")
        layout.label(text="Transformations")
        for name, ends in RELATIONSHIPS.items():
            layout.operator("tc167.clip", text=" → ".join(ends)).action_name = name
        layout.label(text="Configuration Synchronization")
        layout.operator("tc167.preview_endpoints")
        layout.operator("tc167.sync_endpoints")
        layout.operator("tc167.validate_configuration", text="Validate Endpoint Consistency")
        layout.label(text="BATTLEDROID locomotion")
        for name in ("BD_Idle", "BD_Walk", "BD_Run"):
            layout.operator("tc167.clip", text=name).action_name = name
        layout.label(text="Physical Reference Anchors")
        for text, op in (
            ("Create Missing References", "CREATE"),
            ("Display / Hide Reference Anchors", "DISPLAY"),
            ("Select Ground Reference", "GROUND"),
            ("Select Landing Contacts", "CONTACT"),
            ("Select Thrust Anchors", "THRUST"),
            ("Validate Anchors", "VALIDATE"),
            ("Register New Articulated Objects", "REGISTER"),
            ("Validate for Game", "GAME"),
        ):
            layout.operator("tc167.references", text=text).operation = op
        layout.label(text="Playback")
        layout.operator("screen.animation_play", text="Play / Pause")
        layout.operator("screen.frame_jump", text="First Frame").end = False
        layout.operator("tc167.clip", text="Reset to Configuration").action_name = CONFIGURATIONS[
            current_configuration()
        ]


for cls in (
    TC167_OT_clip,
    TC167_OT_save_configuration,
    TC167_OT_preview,
    TC167_OT_sync,
    TC167_OT_validate,
    TC167_OT_references,
    TC167_PT_actions,
):
    old = getattr(bpy.types, cls.__name__, None)
    if old:
        bpy.utils.unregister_class(old)
    bpy.utils.register_class(cls)
