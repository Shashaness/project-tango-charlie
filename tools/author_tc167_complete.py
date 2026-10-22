"""M21.2: extend saved M21.1 Actions; run with Blender --python, or --export-only."""

import argparse
import json
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Quaternion, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from game.flight_state import VehicleMode as Mode
from game.transformation import TransformationController
from tools import author_tc167_blender as base

PROJECT = base.PROJECT
EDGES = {
    "TC_FighterToVTOL": (Mode.FIGHTER, Mode.VTOL, False, 1.5),
    "TC_VTOLToFighter": (Mode.FIGHTER, Mode.VTOL, True, 1.5),
    "TC_VTOLToBattledroid": (Mode.VTOL, Mode.BATTLEDROID, False, 1.9),
    "TC_BattledroidToVTOL": (Mode.VTOL, Mode.BATTLEDROID, True, 1.9),
}
POSES = {
    "TC_Pose_Fighter": Mode.FIGHTER,
    "TC_Pose_VTOL": Mode.VTOL,
    "TC_Pose_Battledroid": Mode.BATTLEDROID,
}
FPS = 120  # Exact 1.9 s endpoints, and exact original 24 FPS gait timing.


def curves(action):
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                yield from bag.fcurves


def controls(names):
    """Extend the existing embedded helper; switching still assigns every object slot."""
    text = bpy.data.texts["TC167_Action_Controls.py"]
    source = text.as_string()
    start = source.index("    def draw(self, context):")
    end = source.index("\nfor cls in", start)
    source = (
        source[:start]
        + """    def draw(self, context):
        for heading, items in (
            ('Configurations', [('FIGHTER', 'TC_Pose_Fighter'),
                ('VTOL', 'TC_Pose_VTOL'), ('BATTLEDROID', 'TC_Pose_Battledroid')]),
            ('Transformations', [('FIGHTER → VTOL', 'TC_FighterToVTOL'),
                ('VTOL → FIGHTER', 'TC_VTOLToFighter'),
                ('VTOL → BATTLEDROID', 'TC_VTOLToBattledroid'),
                ('BATTLEDROID → VTOL', 'TC_BattledroidToVTOL')]),
            ('BATTLEDROID locomotion', [(n, n) for n in ('BD_Idle', 'BD_Walk', 'BD_Run')])):
            self.layout.label(text=heading)
            for label, name in items:
                self.layout.operator('tc167.clip', text=label).action_name = name
        self.layout.label(text='Playback')
        self.layout.operator('screen.animation_play', text='Play / Pause')
        self.layout.operator('screen.frame_jump', text='Return to first frame').end = False
        button = self.layout.operator('tc167.clip', text='Reset to reference pose')
        button.action_name = context.scene.get('tc167_reference', 'TC_Pose_Fighter')
"""
        + source[end:]
    )
    source = source.replace(
        "    action = bpy.data.actions[name]",
        "    if name.startswith('TC_Pose_'): bpy.context.scene['tc167_reference'] = name\n"
        "    action = bpy.data.actions[name]",
    )
    text.clear()
    text.write(source)
    bpy.data.texts["TC167_README"].clear()
    bpy.data.texts["TC167_README"].write(
        "M21.2 complete vehicle. Run TC167_Action_Controls.py, then N → TC167.\n"
        "All Actions are standard multi-object slotted data. 120 FPS preserves exact seconds.\n"
        "See docs/TC167_ANIMATION_AUTHORING.md. Never apply transforms or edit pivots.\n"
    )


def prune_constant_channels():
    """Keep only endpoints for unchanged components; retain full multi-object slots."""
    for name in EDGES:
        for curve in curves(bpy.data.actions[name]):
            keys = curve.keyframe_points
            if len(keys) > 2 and max(k.co.y for k in keys) - min(k.co.y for k in keys) < 1e-7:
                for index in range(len(keys) - 2, 0, -1):
                    keys.remove(keys[index], fast=True)
                curve.update()


def create():
    names = list(json.loads(bpy.data.texts["TC167_FIGHTER_BASIS.json"].as_string()))
    before = base.fingerprint(list(bpy.data.objects))
    # Preserve existing gait shapes exactly while expressing their time at 120 FPS.
    for action in bpy.data.actions:
        action.frame_end = 1 + (action.frame_end - 1) * 5
        for curve in curves(action):
            for key in curve.keyframe_points:
                for point in (key.co, key.handle_left, key.handle_right):
                    point.x = 1 + (point.x - 1) * 5
            curve.update()
    controller = TransformationController(model=base.load_glb(base.SOURCE_GLB))
    for pose_name, mode in POSES.items():
        controller.reset(mode)
        base.make_action(pose_name, 2, names)
        for node in controller.model.nodes:
            obj = bpy.data.objects[node.name]
            basis = base.local_to_basis(obj, node.configuration_matrix.tolist())
            for frame in (1, 2):
                base.key_basis(obj, basis, frame)
    for clip, (first, last, reverse, duration) in EDGES.items():
        end = 1 + round(duration * FPS)
        action = base.make_action(clip, end, names)
        # Sample the source at 120 Hz, then reduce each object's TRS independently.
        samples = {n: [] for n in names}
        for i in range(end):
            t = i / (end - 1)
            controller.apply_curve(first, last, 1 - t if reverse else t)
            for node in controller.model.nodes:
                basis = base.local_to_basis(
                    bpy.data.objects[node.name], node.configuration_matrix.tolist()
                )
                loc, q, scale = basis.decompose()
                # Equivalent signed-scale TRS branches must stay continuous when
                # mirrored shell scales cross zero in the authoritative controller.
                if samples[node.name]:
                    previous = Quaternion(samples[node.name][-1][3:7])
                    candidates = []
                    for signs, turn in (
                        ((1, 1, 1), Quaternion()),
                        ((1, -1, -1), Quaternion((1, 0, 0), np.pi)),
                        ((-1, 1, -1), Quaternion((0, 1, 0), np.pi)),
                        ((-1, -1, 1), Quaternion((0, 0, 1), np.pi)),
                    ):
                        candidate = q @ turn
                        candidates.append(
                            (
                                abs(previous.dot(candidate)),
                                candidate,
                                Vector([scale[j] * signs[j] for j in range(3)]),
                            )
                        )
                    _, q, scale = max(candidates, key=lambda item: item[0])
                values = np.array([*loc, *q, *scale])
                if samples[node.name] and np.dot(values[3:7], samples[node.name][-1][3:7]) < 0:
                    values[3:7] *= -1
                samples[node.name].append(values)
        for name, values in samples.items():
            values = np.array(values)
            keep = {0, end - 1}

            def reduce(a, b):
                if b - a < 2:
                    return
                weights = np.arange(1, b - a)[:, None] / (b - a)
                predicted = values[a] * (1 - weights) + values[b] * weights
                predicted[:, 3:7] /= np.linalg.norm(predicted[:, 3:7], axis=1)[:, None]
                errors = np.max(np.abs(predicted - values[a + 1 : b]), axis=1)
                index = int(errors.argmax()) + a + 1
                if errors.max() > 0.0005:
                    keep.add(index)
                    reduce(a, index)
                    reduce(index, b)

            reduce(0, end - 1)
            for i in sorted(keep):
                v = values[i]
                obj = bpy.data.objects[name]
                obj.rotation_mode = "QUATERNION"
                obj.location = Vector(v[:3])
                obj.rotation_quaternion = Quaternion(v[3:7])
                obj.scale = Vector(v[7:])
                for path in ("location", "rotation_quaternion", "scale"):
                    obj.keyframe_insert(data_path=path, frame=i + 1, group="Local TRS")
        for curve in curves(action):
            for key in curve.keyframe_points:
                key.interpolation = "LINEAR"
    # Exporter discovers multi-object slots through muted archival strips.
    for action_name in list(POSES) + list(EDGES):
        base.select_action(action_name, names)
        for name in names:
            obj = bpy.data.objects[name]
            track = obj.animation_data.nla_tracks.new()
            track.name = action_name
            track.mute = True
            strip = track.strips.new(action_name, 1, bpy.data.actions[action_name])
            strip.action_slot = obj.animation_data.action_slot
    prune_constant_channels()
    controls(names)
    bpy.context.scene.render.fps = FPS
    base.select_action("TC_Pose_Fighter", names)
    if base.fingerprint(list(bpy.data.objects)) != before:
        raise ValueError("Geometry or pivots changed")


def continuous_export(output):
    """Resolve glTF exporter's equivalent signed-scale branches before interpolation."""
    model = base.load_glb(output)
    document, binary = base.read_glb(output)

    def append(values, kind):
        nonlocal binary
        values = np.asarray(values, dtype="<f4")
        binary += b"\0" * (-len(binary) % 4)
        view = len(document["bufferViews"])
        document["bufferViews"].append(
            {"buffer": 0, "byteOffset": len(binary), "byteLength": values.nbytes}
        )
        index = len(document["accessors"])
        document["accessors"].append(
            {"bufferView": view, "componentType": 5126, "count": len(values), "type": kind}
        )
        binary += values.tobytes()
        return index

    for animation in document["animations"]:
        if animation["name"] not in EDGES:
            continue
        clip = model.animations[animation["name"]]
        for node in model.nodes:
            channels = {c.path: c for c in clip.channels if c.node is node}
            r, s = channels["rotation"].sampler, channels["scale"].sampler
            times = sorted(set(r.times) | set(s.times))
            rotations, scales = [], []
            previous = None
            for time in times:
                xyzw = r.sample(float(time), True)
                q = Quaternion((xyzw[3], *xyzw[:3]))
                scale = s.sample(float(time))
                candidates = []
                for signs, turn in (
                    ((1, 1, 1), Quaternion()),
                    ((1, -1, -1), Quaternion((1, 0, 0), np.pi)),
                    ((-1, 1, -1), Quaternion((0, 1, 0), np.pi)),
                    ((-1, -1, 1), Quaternion((0, 0, 1), np.pi)),
                ):
                    candidate = q @ turn
                    candidates.append(
                        (
                            abs(previous.dot(candidate))
                            if previous
                            else (1 if signs == (1, 1, 1) else 0),
                            candidate,
                            scale * signs,
                        )
                    )
                _, q, scale = max(candidates, key=lambda item: item[0])
                if previous and previous.dot(q) < 0:
                    q.negate()
                previous = q
                rotations.append((q.x, q.y, q.z, q.w))
                scales.append(scale)
            input_index = append(times, "SCALAR")
            for channel in animation["channels"]:
                if document["nodes"][channel["target"]["node"]]["name"] != node.name:
                    continue
                path = channel["target"]["path"]
                if path in ("rotation", "scale"):
                    sampler = animation["samplers"][channel["sampler"]]
                    sampler["input"] = input_index
                    sampler["output"] = append(
                        rotations if path == "rotation" else scales,
                        "VEC4" if path == "rotation" else "VEC3",
                    )
    base.write_glb(output, document, binary)


def preserve_native_gaits(output):
    """Retain the exact production gait samplers rather than resampling their slerps."""
    import copy

    document, binary = base.read_glb(output)
    old, data = base.read_glb(PROJECT / "assets/models/vehicles/tc167_animated.glb")
    decoder = base.Accessors(old, data)
    document["animations"] = [
        a for a in document["animations"] if a["name"] not in ("BD_Idle", "BD_Walk", "BD_Run")
    ]
    for source in old["animations"]:
        animation = copy.deepcopy(source)
        for sampler in animation["samplers"]:
            for field in ("input", "output"):
                accessor = old["accessors"][sampler[field]]
                values = np.asarray(
                    decoder.decode(sampler[field], accessor["type"], (5126,)), dtype="<f4"
                )
                binary += b"\0" * (-len(binary) % 4)
                view = len(document["bufferViews"])
                document["bufferViews"].append(
                    {"buffer": 0, "byteOffset": len(binary), "byteLength": values.nbytes}
                )
                sampler[field] = len(document["accessors"])
                document["accessors"].append(
                    {
                        "bufferView": view,
                        "componentType": 5126,
                        "count": len(values),
                        "type": accessor["type"],
                    }
                )
                binary += values.tobytes()
        document["animations"].append(animation)
    base.write_glb(output, document, binary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--blend-input", type=Path)
    parser.add_argument(
        "--blend-output", type=Path, default=PROJECT / "assets/blender/TC167_Complete.blend"
    )
    parser.add_argument(
        "--glb-output", type=Path, default=PROJECT / "assets/models/vehicles/tc167_complete.glb"
    )
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    protected = [PROJECT / "assets/blender" / n for n in ("TC167.blend", "TC167_Animated.blend")]
    protected += [
        PROJECT / "assets/models/vehicles" / n for n in ("tc167.glb", "tc167_animated.glb")
    ]
    if any(p.resolve() in protected for p in (args.blend_output, args.glb_output)):
        parser.error("Production and M21.1 assets are protected")
    if not args.export_only and args.blend_output.exists():
        parser.error("Choose a new authoring output; file already exists")
    source = args.blend_input or (args.blend_output if args.export_only else protected[1])
    bpy.ops.wm.open_mainfile(filepath=str(source.resolve()))
    if not args.export_only:
        create()
    base.CLIPS.update({n: 1 + round(v[3] * FPS) for n, v in EDGES.items()})
    base.CLIPS.update({n: 2 for n in POSES})
    base.export_actions(args.glb_output)
    continuous_export(args.glb_output)
    if not args.export_only:
        preserve_native_gaits(args.glb_output)
    base.select_action(
        "TC_Pose_Fighter", list(json.loads(bpy.data.texts["TC167_FIGHTER_BASIS.json"].as_string()))
    )
    if not args.export_only:
        bpy.ops.wm.save_as_mainfile(filepath=str(args.blend_output.resolve()))
    print("M21.2 export complete:", args.glb_output)


if __name__ == "__main__":
    main()
