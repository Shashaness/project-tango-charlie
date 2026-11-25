"""Local UTF-8 campaign saves with atomic replacement."""

import json
import os
import tempfile
from pathlib import Path

from game.world_manifest import pairs


def read_save(path):
    return json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=pairs)


def write_save(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=".campaign-", delete=False, mode="w", encoding="utf-8"
        ) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
