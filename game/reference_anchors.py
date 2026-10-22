"""Explicit physical references evaluated through the current animated hierarchy."""

import numpy as np

from game.flight_state import VehicleMode


def authored(vehicle):
    controller = getattr(vehicle, "transformation", None)
    return controller if getattr(controller, "authority", None) == "GLB" else None


def validate_anchors(model):
    names = [n.name for n in model.nodes]
    errors = []
    for name in ["TC_ROOT", *(f"TC_GROUND_{m.name}" for m in VehicleMode)]:
        if names.count(name) != 1:
            errors.append(f"Required unique anchor {name}")
    for mode in VehicleMode:
        for prefix in ("TC_CONTACT_", "TC_BODY_", "TC_HULL_"):
            if not any(n.startswith(prefix + mode.name + "_") for n in names):
                errors.append(f"Missing {prefix}{mode.name}_<id> physical references")
    if not any(n.startswith("TC_THRUST_") for n in names):
        errors.append("Missing TC_THRUST_<id> propulsion references")
    for node in model.nodes:
        if node.name.startswith(
            ("TC_CONTACT_", "TC_BODY_", "TC_GROUND_", "TC_THRUST_", "TC_HULL_")
        ):
            if node.meshes:
                errors.append(f"{node.name}: references must be non-rendered nodes")
            if node.parent is None:
                errors.append(f"{node.name}: parent reference to vehicle/component")
    model.world_matrices()
    for mode in VehicleMode:
        hull = [
            n.world_matrix[:3, 3]
            for n in model.nodes
            if n.name.startswith("TC_HULL_" + mode.name + "_")
        ]
        if len(hull) < 4 or np.any(np.ptp(hull, axis=0) < 1e-4):
            errors.append(f"TC_HULL_{mode.name}_<id>: define a nondegenerate 3D collision envelope")
    return errors


def positions(vehicle, prefix=None):
    c = authored(vehicle)
    if c is None:
        return {}
    c.model.world_matrices()
    return {
        n.name: n.world_matrix[:3, 3].copy()
        for n in c.model.nodes
        if n.name.startswith(
            prefix or ("TC_GROUND_", "TC_CONTACT_", "TC_THRUST_", "TC_BODY_", "TC_HULL_")
        )
    }


def support_points(vehicle):
    c = authored(vehicle)
    if c is None:
        return None
    modes = {c.source, c.target} if c.active else {c.configuration}
    prefixes = tuple(
        prefix + mode.name + ("_" if prefix != "TC_GROUND_" else "")
        for mode in modes
        for prefix in (
            ("TC_CONTACT_", "TC_BODY_") if c.active else ("TC_CONTACT_", "TC_BODY_", "TC_GROUND_")
        )
    )
    points = list(positions(vehicle, prefixes).values())
    if not points:
        raise ValueError("Authored model has no evaluated ground/collision references")
    return np.asarray(points)


def clearance(vehicle):
    points = support_points(vehicle)
    return (
        None if points is None else max(0.0, -float((points @ vehicle.orientation.T)[:, 1].min()))
    )


def floor(vehicle, height):
    """Sample terrain at every rotated support/body reference, retaining world physics."""
    points = support_points(vehicle)
    if points is None:
        return None
    offsets = points @ vehicle.orientation.T
    values = [height(vehicle.position + p) for p in offsets]
    valid = [float(h) - p[1] for p, h in zip(offsets, values) if h is not None]
    return max(valid) if valid else None
