"""Validate a GLB and list native clip names, durations, channels, and target nodes."""

import argparse

from engine.gltf_loader import AssetError, load_glb


def inspect(path, require_battledroid=False):
    """Validate a file through the runtime loader and print its rigid-node clip inventory.

    Args:
        path: Input GLB path; the file is read without allocating OpenGL resources.
        require_battledroid: Whether to reject a missing BD_Idle, BD_Walk, or BD_Run.

    Returns:
        Validated CPU Model. Invalid input raises AssetError with a file diagnostic.
    """
    model = load_glb(path)
    print(f"{path}: {len(model.animations)} animation clip(s)")
    for clip in model.animations.values():
        print(f"{clip.name}: {clip.duration:.6g} s, {len(clip.channels)} channel(s)")
        for channel in clip.channels:
            sampler = channel.sampler
            print(
                f"  node[{channel.node_index}] {channel.node.name}: {channel.path}, "
                f"{sampler.interpolation}, {len(sampler.times)} keys, "
                f"{sampler.times[0]:.6g}–{sampler.times[-1]:.6g} s"
            )
    if require_battledroid:
        missing = {"BD_Idle", "BD_Walk", "BD_Run"} - model.animations.keys()
        if missing:
            raise AssetError(f"{path}: missing required clips: {', '.join(sorted(missing))}")
    return model


def main():
    """Run headless inspection; invalid assets exit with a path-qualified diagnostic."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="GLB file to validate")
    parser.add_argument(
        "--require-bd-clips", action="store_true", help="require BD_Idle, BD_Walk, and BD_Run"
    )
    args = parser.parse_args()
    try:
        inspect(args.path, require_battledroid=args.require_bd_clips)
    except AssetError as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
