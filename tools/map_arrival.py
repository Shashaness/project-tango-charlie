"""Generate the Arrival overhead SVG from the runtime manifest and road tessellation."""

from html import escape
from pathlib import Path
from xml.etree import ElementTree

import numpy as np

from engine.asset_paths import PROJECT_ROOT
from game.access_road import AccessRoad
from game.arrival import WORLD
from game.urban_layout import CITY_CENTER, CITY_SIZE
from game.world_loader import SceneRegistry, scene_model
from game.world_manifest import load_manifest

STRUCTURES = {
    "hangar_01": "Hangar A",
    "hangar_b": "Hangar B",
    "service_garage": "Jeep garage",
    "service_shed": "Equipment shed",
    "service_store": "Storage",
    "service_workshop": "Workshop",
}
LABELS = {
    "hangar_01": (300, 4360),
    "hangar_b": (300, 4490),
    "service_garage": (27, 4720),
    "service_shed": (147, 4811),
    "service_store": (-4, 4775),
    "service_workshop": (128, 4730),
}


def generate():
    manifest = load_manifest(WORLD)
    scene = SceneRegistry(manifest)
    airport = manifest.airport
    road = AccessRoad("compound_access", *airport["access_roads"]["compound_access"])
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="1110" '
        'viewBox="0 0 1080 1110">',
        '<rect width="1080" height="1110" fill="#101e25"/>',
        "<style>text{font:14px system-ui;fill:#f0eadb}.title{font:bold 31px system-ui}"
        ".caption{fill:#b9c7ca}.label{font:bold 13px system-ui;paint-order:stroke;"
        "stroke:#172b32;stroke-width:4;stroke-linejoin:round}.small{font:12px system-ui}</style>",
        '<text x="39" y="54" class="title">Arrival world layout</text>',
        '<text x="40" y="80" class="caption">'
        "Project Tango Charlie · Q1.2C · World-local meters</text>",
        '<defs><clipPath id="local">'
        '<rect x="38" y="110" width="533" height="864" rx="10"/></clipPath>'
        '<marker id="arrow" markerWidth="9" markerHeight="8" refX="7" refY="4" orient="auto">'
        '<path d="M0 0L8 4L0 8Z" fill="#f4ca78"/></marker></defs>',
    ]
    X, Y, SCALE = 38, 110, 1.0

    def xy(x, z):
        return X + (x + 60) * SCALE, Y + (z - 4050) * SCALE

    def text(x, y, label, style=""):
        svg.append(f'<text x="{x:.1f}" y="{y:.1f}" class="{style}">{escape(label)}</text>')

    def rect(center, size, fill):
        w, d = size
        x, y = xy(center[0] - w / 2, center[1] - d / 2)
        svg.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w * SCALE:.1f}" '
            f'height="{d * SCALE:.1f}" fill="{fill}"/>'
        )

    def path(points, color, width, extra=""):
        coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in (xy(*p) for p in points))
        svg.append(
            f'<polyline points="{coords}" fill="none" stroke="{color}" '
            f'stroke-width="{width}" stroke-linejoin="round" {extra}/>'
        )

    svg.append('<g clip-path="url(#local)">')
    svg.append('<rect x="38" y="110" width="533" height="864" fill="#263b41"/>')
    for x in range(-50, 401, 50):
        path(((x, 4050), (x, 4900)), "#344a50", 1)
    for z in range(4100, 4900, 50):
        path(((-60, z), (410, z)), "#344a50", 1)
    runway = airport["runway"]
    r = runway["center"]
    rect(r[::2], (runway["width"], runway["length"]), "#809398")
    path(
        ((r[0], r[2] - runway["length"] / 2), (r[0], r[2] + runway["length"] / 2)),
        "#dde2df",
        1,
        'stroke-dasharray="9 9"',
    )
    for center, size in (*airport["taxiways"], airport["apron_link"]):
        rect(center, size, "#647b83")
    for center, size in (airport["apron"], *airport["service_yards"]):
        rect(center, size, "#546a72")
    for center, size in airport["side_hangars"]:
        rect(center[::2], size[::2], "#56605f")
    path(road.points, "#93b2b5", road.width * SCALE)
    path(road.points, "#3b5155", 1.2, 'stroke-dasharray="4 5"')

    for key, label in STRUCTURES.items():
        placement = scene.instances[key]
        lo, hi = scene_model(placement.definition.asset.path).bounds()
        local = [
            (x, 0, z, 1)
            for x, z in (
                (lo[0], lo[2]),
                (hi[0], lo[2]),
                (hi[0], hi[2]),
                (lo[0], hi[2]),
            )
        ]
        world = np.asarray(local) @ np.asarray(placement.matrix).T
        poly = " ".join(f"{x:.2f},{y:.2f}" for x, y in (xy(*p[::2][:2]) for p in world))
        svg.append(f'<polygon points="{poly}" fill="#a69b76" stroke="#292a24" stroke-width="2"/>')
        if key in {"service_garage", "service_store", "service_workshop"}:
            path((tuple(placement.position[::2]), LABELS[key]), "#d0c5a9", 0.9)
        text(*xy(*LABELS[key]), label, "label")

    path(
        (
            (271, 4475),
            (228, 4475),
            *road.points[::-1],
            (87, 4812),
            (57, 4826),
            (40, 4834),
            tuple(scene.instances["tc167"].position[::2]),
        ),
        "#f4ca78",
        2,
        'stroke-dasharray="6 5" marker-end="url(#arrow)"',
    )
    for key, label, dx, dy in (
        ("tc167", "TC167 crash", -70, 19),
        ("pilot_01", "Kara spawn", 9, -5),
        ("hangar-jeep", "Jeep", 19, -39),
    ):
        x, y = xy(*scene.instances[key].position[::2])
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="#f4ca78"/>')
        if key == "hangar-jeep":
            svg.append(f'<path d="M{x:.1f} {y:.1f}l{dx} {dy}" stroke="#f4ca78"/>')
        text(x + dx, y + dy, label, "label")
    text(*xy(110, 4280), "Maintenance apron", "label")
    text(*xy(-20, 4200), "Runway", "label")
    text(*xy(4, 4428), "Taxiway", "label")
    text(*xy(228, 4620), "Curved access road", "label")
    text(*xy(144, 4785), "Outlying service compound", "label")
    path((airport["service_yards"][0][0], (121, 4788)), "#d0c5a9", 0.9)
    text(*xy(121, 4788), "Yard", "label")
    svg.append("</g>")

    text(617, 132, "Orientation", "label")
    svg.append('<path d="M655 216V154" stroke="#f4ca78" fill="none" marker-end="url(#arrow)"/>')
    for axis in ("M655 216H725", "M655 216V258"):
        svg.append(f'<path d="{axis}" stroke="#f4ca78" fill="none" marker-end="url(#arrow)"/>')
    text(670, 165, "North (−Z) toward city")
    text(735, 221, "+X east")
    text(671, 258, "+Z south")
    svg.append('<path d="M620 300h100m-100 -5v10m50 -10v10m50 -10v10" stroke="#eee9dd"/>')
    text(619, 325, "0")
    text(660, 325, "50")
    text(706, 325, "100 m")

    def distance(a, b):
        p, q = (np.asarray(scene.instances[key].position)[::2] for key in (a, b))
        return np.linalg.norm(p - q)

    yard_distance = np.linalg.norm(
        np.asarray(scene.instances["tc167"].position)[::2] - airport["service_yards"][0][0]
    )
    text(617, 382, "Measured separations", "label")
    for i, line in enumerate(
        (
            f"Crash to courtyard  {yard_distance:.1f} m",
            f"Crash to garage  {distance('tc167', 'service_garage'):.1f} m",
            f"Crash to Hangar A  {distance('tc167', 'hangar_01'):.1f} m",
            f"Garage to Hangar A  {distance('service_garage', 'hangar_01'):.1f} m",
            f"Access road centerline  {road.length:.1f} m",
        )
    ):
        text(617, 410 + 25 * i, line)

    text(617, 553, "Wider view", "label")
    svg.append('<rect x="617" y="568" width="390" height="380" rx="10" fill="#263b41"/>')

    def overview(x, z):
        return 775 + x * 0.054, 650 + (z + 300) * 0.054

    cx, cz = CITY_CENTER
    x, y = overview(cx - CITY_SIZE / 2, cz - CITY_SIZE / 2)
    svg.append(
        f'<rect x="{x}" y="{y}" width="{CITY_SIZE * 0.054}" height="{CITY_SIZE * 0.054}" '
        'fill="#a69b76"/>'
    )
    text(833, 622, "Ruined city")
    p0, p1 = (
        overview(r[0], r[2] - runway["length"] / 2),
        overview(r[0], r[2] + runway["length"] / 2),
    )
    svg.append(f'<path d="M{p0[0]} {p0[1]}L{p1[0]} {p1[1]}" stroke="#809398" stroke-width="4"/>')
    x, y = overview(*scene.instances["hangar_01"].position[::2])
    svg.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#f4ca78"/>')
    text(833, 892, "Arrival local area")
    text(647, 923, "City edge is about 4.4 km from the crash.", "small")
    svg.append('<path d="M42 991h54" stroke="#f4ca78" stroke-width="2" stroke-dasharray="6 5"/>')
    text(109, 997, "Future recovery route (proposed)")
    text(
        40,
        1040,
        "Local map drawn from manifest placements and sampled access-road geometry.",
        "caption",
    )
    svg.append("</svg>")
    result = "\n".join(svg) + "\n"
    ElementTree.fromstring(result)
    return result


if __name__ == "__main__":
    target = Path(PROJECT_ROOT) / "docs/images/arrival_world_layout.svg"
    target.write_text(generate(), encoding="utf-8")
    print(target)
