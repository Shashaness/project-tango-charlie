# Contributing to Project Tango Charlie

Project Tango Charlie uses a small Python/OpenGL engine. Keep contributions focused,
readable, and easy to review. Discuss architectural changes before introducing a new
framework or changing module responsibilities.

## Development setup

Create a branch from your checkout, then install the runtime dependencies and the
pinned development formatter:

```sh
git checkout -b feature/my-change
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python main.py
```

On Windows, activate with `.venv\Scripts\activate`. The code uses Python 3.10+
syntax; dependency support also depends on your interpreter and platform. Running
the game requires a display and an OpenGL 3.3 Core driver. Most automated tests run
without a display. Runtime-only installations can use `requirements.txt`; Ruff is
a development dependency and is never imported by the game.

Read [README.md](README.md) for controls, startup options, and architecture.

## Formatting and documentation

Use PEP 8, four spaces, and lines no longer than 100 characters. Ruff settings in
`pyproject.toml` define formatting and import organization for `engine`, `game`,
`main.py`, `scripts`, `tools`, and `tests`. Asset data, backups, and virtual
environments are outside this source pass.

```sh
python -m ruff format .
python -m ruff check --select I --fix .
python -m ruff format --check .
python -m ruff check --select E501,I .
python -m ruff check .
```

The full check includes existing findings documented in
[the readability report](docs/CODE_READABILITY_PASS.md). Compare those findings
with your baseline and introduce no new ones. Ruff is configured to fix import
organization only. Review import changes for initialization-order effects; retain
intentional local imports used to avoid cycles or defer expensive loading. Do not
remove imports, rename APIs, rewrite lambdas, or alter algorithms just to satisfy a
lint diagnostic during a documentation pass.

Use `snake_case` for new functions/variables, `PascalCase` for classes, and
`UPPER_CASE` for constants. Preserve established public names. Give modules a
responsibility docstring and classes/nontrivial functions Google-style docstrings:

```python
def sample_height(position):
    """Query ground elevation without changing vehicle motion.

    Args:
        position: World XYZ position in meters.

    Returns:
        Ground elevation in meters.
    """
```

Describe arguments, meaningful return values, units, and side effects. Use
`Raises` or `Yields` where they clarify the contract. Explain why complex physics,
matrix transformations, collision tests, terrain blending, or animation steps
work; avoid narrating obvious assignments.

Coordinates are right-handed: +X right, +Y up, and -Z forward. One world unit is
one meter. Physics uses SI units and radians/radians per second unless a field
explicitly uses degrees. glTF quaternions use `[x, y, z, w]`; model transforms act
on column vectors. See existing docstrings before changing a coordinate convention.

## Verification and contribution workflow

1. Establish a baseline before editing:
   `python -m unittest discover -s tests` and `python -m ruff check .`.
2. Make small groups of changes. Preserve unrelated working-tree edits and assets.
3. Run relevant tests and static checks after each group, then run the complete
   suite before submitting. Do not alter assertions to conceal an existing failure.
4. For documentation/formatting changes, inspect the diff for unchanged constants,
   key mappings, public signatures, control flow, and asset paths. Compare parsed
   syntax trees with docstrings removed when practical; review import ordering
   separately because it can affect module initialization.
5. Describe the problem, resulting behavior, verification, and known limitations
   in your pull request. Report existing defects separately from the proposed change.

The test suite uses the standard-library `unittest` runner; pytest is not required.
The readability-pass baseline contains two existing pose/transformation failures,
documented in the report. Behavioral fixes belong in separate changes with focused
regression coverage.

For changes affecting gameplay, manually check fighter flight in SPACE/ATMOSPHERE,
VTOL hover and landings, battledroid locomotion, configuration transitions, camera
CHASE/DOLLY behavior, and weapon/defense state. Preserve shared vehicle momentum
through transformations, force-based flight, and the separation between physical
motion and observational rendering/animation.

For model contributions, preserve required node names, hierarchy, pivots, local
axes, and meter units. Inspect all three configurations after exporting. Artwork,
models, textures, and audio must be original or legally redistributable under the
project's MIT license; third-party dependencies retain their own licenses.

For bug reports, include platform/interpreter versions, reproduction steps,
expected and actual behavior, and relevant logs. Explain any checks you could not
run and why.
