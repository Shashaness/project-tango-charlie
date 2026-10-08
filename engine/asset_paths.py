"""Project-relative asset paths, independent of the process working directory."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ASSET_ROOT = PROJECT_ROOT / 'assets'


def asset_path(relative):
    return ASSET_ROOT / relative
