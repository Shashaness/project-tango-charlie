# Contributing to Project Tango Charlie

Thank you for your interest in contributing to **Project Tango Charlie**!

Project Tango Charlie is an open-source, Python-based 3D game featuring TC167, a transformable combat vehicle capable of operating in three configurations:

- **FIGHTER** — High-speed atmospheric and space flight
- **VTOL** — Vertical flight, hovering, and ground operations
- **BATTLEDROID** — Ground locomotion and articulated combat

Our goal is to develop a fun, technically interesting game while keeping the underlying engine understandable, maintainable, and accessible to contributors.

## 1. Getting Started

1. Fork the repository on GitHub.
2. Clone your fork to your local machine.
3. Create a feature branch from `main`.
4. Install the dependencies described in `README.md`.
5. Make your changes and run the relevant tests.
6. Submit a pull request describing your changes.

For example:

```bash
git clone git@github.com:YOUR_USERNAME/project-tango-charlie.git
cd project-tango-charlie

git checkout -b feature/my-improvement
```

## 2. Development Philosophy

Project Tango Charlie uses a lightweight, custom game engine built with Python, GLFW, PyOpenGL, and NumPy.

We favor:

- Readable, well-documented code.
- Small, focused changes.
- Modular systems with clear responsibilities.
- Physically consistent vehicle behavior.
- Deterministic, testable simulation logic.
- Minimal external dependencies.

Please discuss major architectural changes before implementing them.

Do not introduce a new game engine, physics framework, or large dependency without prior discussion.

## 3. Vehicle Physics and Transformation

TC167 uses a shared physical state across all three configurations.

Contributions must preserve the following principles:

- Transformation must not arbitrarily reset position, velocity, or orientation.
- Flight behavior must remain force-based.
- Vehicle orientation and world-space velocity remain independent.
- Ground locomotion and procedural animation must remain logically separate.
- SPACE and ATMOSPHERE environments must retain their distinct physics.
- Flight stabilization and hover assistance must not remove manual control.

Changes affecting physics, transformations, or flight controls should include appropriate regression tests.

## 4. 3D Models and Assets

TC167 uses a hierarchical GLB/glTF model with named components and transformation pivots.

When modifying models:

- Preserve required node names and hierarchy.
- Maintain compatible pivot locations and local axes.
- Use consistent units: **1 unit = 1 meter**.
- Avoid changing component origins without coordinating corresponding animation changes.
- Test all three vehicle configurations after exporting.

New artwork, models, textures, audio, and other assets must be original or legally redistributable under the project's MIT License.

Do not submit copyrighted franchise assets or material copied from commercial games without appropriate authorization.

## 5. Coding Standards

- Follow existing Python naming conventions and module organization.
- Use descriptive function and variable names.
- Document non-obvious mathematical operations.
- Avoid unnecessary global state.
- Keep rendering, physics, controls, and gameplay responsibilities separated.
- Add or update tests for behavioral changes.

Prefer straightforward implementations over unnecessary abstraction.

## 6. Testing

Before submitting a pull request, run the project's automated test suite:

```bash
python -m pytest
```

Where applicable, also perform manual testing of:

- FIGHTER flight in SPACE and ATMOSPHERE.
- VTOL hovering, landing, and takeoff.
- BATTLEDROID ground movement and procedural animation.
- Transformations between configurations.
- Camera CHASE/DOLLY behavior.
- Weapons, targeting, and defensive systems.

If a test fails or cannot be run, explain why in your pull request.

## 7. Issues and Feature Requests

GitHub Issues may be used to report bugs, propose features, and discuss improvements.

For bug reports, please include:

- Operating system and Python version.
- Steps to reproduce the issue.
- Expected and actual behavior.
- Relevant error messages or logs.

For larger feature proposals, describe the intended behavior and any impact on existing systems.

## 8. Pull Requests

Please keep pull requests focused on one feature or fix whenever practical.

Each pull request should include:

- A summary of the changes.
- The reason