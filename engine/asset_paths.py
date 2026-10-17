"""Project-relative asset paths, independent of the process working directory."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ASSET_ROOT = PROJECT_ROOT / "assets"


def asset_path(relative):
    """Resolve an asset relative to the repository asset directory.

    Args:
        relative: Asset path relative to ASSET_ROOT.

    Returns:
        Path beneath ASSET_ROOT; does not read the file.
    """
    return ASSET_ROOT / relative
