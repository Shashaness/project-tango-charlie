"""Deterministic, desolate M20.2 scenery; CPU generation needs no OpenGL."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass, replace

import numpy as np

from game.atmospheric_physics import PARAMETERS
from game.urban_layout import (
    CITY_CENTER,
    CITY_SIZE,
    SEED,
    center_size,
    downtown_distance,
    generate_layout,
    generate_parcels,
    inset,
    subtract_rectangle,
)


@dataclass(frozen=True)
class Runway:
    """Runway center/extent geometry in local meters with orientation-aware containment."""

    center: tuple = (0.0, 0.0, 2500.0)
    length: float = 2400.0
    width: float = 45.0
    forward: tuple = (0.0, 0.0, -1.0)
    designation: str = "36"

    @property
    def spawn_xz(self):
        return tuple(
            (np.asarray(self.center) - (self.length / 2 - 120) * np.asarray(self.forward))[[0, 2]]
        )

    def coordinates(self, points):
        """Project world points into runway lateral and longitudinal coordinates.

        Args:
            points: Query point array in the coordinate system used by the operation.

        Returns:
            Pair of lateral and longitudinal coordinate arrays in meters.
        """
        delta = np.asarray(points) - np.asarray(self.center)
        forward = np.asarray(self.forward, float)
        right = np.cross(forward, (0.0, 1.0, 0.0))
        return delta @ right, delta @ forward

    def contains(self, points):
        """Test points against the rotated runway rectangle.

        Args:
            points: Query point array in the coordinate system used by the operation.

        Returns:
            Boolean mask for the query point shape.
        """
        lateral, longitudinal = self.coordinates(points)
        return (np.abs(lateral) <= self.width / 2) & (np.abs(longitudinal) <= self.length / 2)


RUNWAY = Runway()
HANGAR_SPAWN = (310.0, 0.0, 3000.0)
HANGAR_FORWARD = (-1.0, 0.0, 0.0)
GROUND_ELEVATION = PARAMETERS.ground_altitude
TERRAIN_BOUNDS = (
    -9000.0,
    9000.0,
    -8000.0,
    10000.0,
)  # Legacy composition canvas; never rendered uncropped.
BUILDING_PALETTE = (
    (0.66, 0.65, 0.62),  # weathered light concrete
    (0.51, 0.51, 0.49),  # medium concrete
    (0.36, 0.37, 0.37),  # dark concrete
    (0.24, 0.25, 0.26),  # charcoal
    (0.13, 0.14, 0.15),  # near-black
    (0.43, 0.47, 0.49),  # desaturated steel
    (0.47, 0.42, 0.36),  # faded industrial brown
)
PANEL_COLOR = (0.12, 0.14, 0.15)
GROUND_PALETTE = {
    "desert": (0.57, 0.48, 0.36),
    "street": (0.22, 0.22, 0.21),
    "sidewalk": (0.48, 0.46, 0.41),
    "lot": (0.42, 0.36, 0.28),
    "runway": (0.15, 0.16, 0.16),
    "taxiway": (0.20, 0.21, 0.20),
    "apron": (0.34, 0.33, 0.30),
    "perimeter": (0.27, 0.26, 0.23),
    "marking": (0.85, 0.83, 0.74),
    "park": (0.44, 0.43, 0.32),
    "plaza": (0.48, 0.47, 0.43),
    "path": (0.60, 0.56, 0.47),
}


@dataclass(frozen=True)
class Building:
    """Static building footprint, dimensions in meters, and facade/render metadata."""

    center: tuple
    size: tuple
    color: tuple
    block_id: int = -1
    parcel: tuple = ()
    style: str = "slab"
    landmark: bool = False

    @property
    def footprint(self):
        x, _, z = self.center
        w, _, d = self.size
        return (x - w / 2, x + w / 2, z - d / 2, z + d / 2)


@dataclass(frozen=True)
class GroundSurface:
    """One tile in a disjoint partition: no hidden base or offset overlays."""

    bounds: tuple
    kind: str
    elevation: float = GROUND_ELEVATION

    @property
    def color(self):
        return GROUND_PALETTE[self.kind]

    def subtract(self, bounds):
        """Keep only this rectangle's area outside an overlaid rectangle.

        Args:
            bounds: Horizontal (minimum X, maximum X, minimum Z, maximum Z) rectangle in
                meters.

        Returns:
            Tuple of retained GroundSurface rectangles with the same kind and elevation.
        """
        return tuple(replace(self, bounds=p) for p in subtract_rectangle(self.bounds, bounds))


def building_height(x, z, rng):
    # Smooth district anchors: fringe -> industrial -> inner city -> core.
    """Choose a seeded skyline height using downtown distance and district variation.

    Args:
        x: Local east-axis coordinate(s) in meters.
        z: Local south-axis coordinate(s) in meters.
        rng: Seeded NumPy random generator.

    Returns:
        Building height in meters.
    """
    distance = downtown_distance(x, z)
    anchors = (0.0, 0.30, 0.58, 0.85, 1.08, 1.5)
    lows = (210.0, 140.0, 60.0, 20.0, 8.0, 8.0)
    highs = (320.0, 270.0, 160.0, 65.0, 30.0, 30.0)
    band = min(len(anchors) - 2, max(0, int(np.searchsorted(anchors, distance)) - 1))
    u = float(np.clip((distance - anchors[band]) / (anchors[band + 1] - anchors[band]), 0.0, 1.0))
    u = u * u * (3.0 - 2.0 * u)
    low = (1.0 - u) * lows[band] + u * lows[band + 1]
    high = (1.0 - u) * highs[band] + u * highs[band + 1]
    return float(rng.uniform(low, high))


class WorldEnvironment:
    """Generate deterministic urban/airfield scenery, protected footprints, and obstacle boxes."""

    def __init__(self, seed=SEED):
        """Generate seeded city/airport scenery and build protected infrastructure footprints.

        Args:
            seed: Integer seed for deterministic variation.
        """
        self.runway = RUNWAY
        self.blocks = []
        self.buildings = []
        self.solid_boxes = []
        self.vertices = []
        self.infrastructure_footprints = {"city": [], "airport": []}
        self.ground_surfaces = [GroundSurface(TERRAIN_BOUNDS, "desert")]
        rng = np.random.default_rng(seed)
        self.layout = generate_layout(seed)
        self.blocks = self.layout.blocks
        self.roads = self.layout.roads
        self.parks = self.layout.parks
        self.parcels = []
        for block in self.blocks:
            self.surface(block.center, block.size, "sidewalk")
            self.surface(*center_size(inset(block.bounds, 4.0)), "lot")
        for road in self.roads:
            self.surface(*center_size(road.bounds), "street")
        # Landmark sites come from buildable downtown blocks, never an independent grid.
        sites = []
        for target in ((0.0, -300.0), (170.0, -410.0), (-70.0, -70.0)):
            site = min(
                (
                    b
                    for b in self.blocks
                    if b.district == "downtown"
                    and b.kind != "open_space"
                    and min(b.size) >= 55
                    and b not in sites
                ),
                key=lambda b: np.hypot(b.center[0] - target[0], b.center[1] - target[1]),
            )
            sites.append(site)
        for block in self.blocks:
            if block.kind == "open_space" or block in sites:
                continue
            parcels = generate_parcels(block, rng)
            self.parcels.extend(parcels)
            for parcel in parcels:
                if parcel.vacant:
                    continue
                (px, pz), (pw, pd) = center_size(inset(parcel.bounds, 2.5))
                factor = (0.84, 0.98) if block.district == "downtown" else (0.62, 0.90)
                w = pw * float(rng.uniform(*factor))
                d = pd * float(rng.uniform(*factor))
                x = px + float(rng.uniform(-1, 1)) * (pw - w) / 2
                z = pz + float(rng.uniform(-1, 1)) * (pd - d) / 2
                height = building_height(x, z, rng)
                if block.district == "outer":
                    style = "warehouse" if rng.random() < 0.75 else "slab"
                    if style == "warehouse":
                        height = float(rng.uniform(8.0, 24.0))
                else:
                    style = str(
                        rng.choice(
                            ("slab", "setback", "sections"),
                            p=(0.25, 0.50, 0.25)
                            if block.district == "downtown"
                            else (0.4, 0.25, 0.35),
                        )
                    )
                color = BUILDING_PALETTE[int(rng.integers(len(BUILDING_PALETTE)))]
                b = Building(
                    (x, GROUND_ELEVATION + height / 2, z),
                    (w, height, d),
                    color,
                    block.id,
                    parcel.bounds,
                    style,
                )
                self.buildings.append(b)
                self.render_building(b, rng)
        for index, (site, total_height) in enumerate(zip(sites, (420.0, 385.0, 350.0))):
            x, z = site.center
            body_height = total_height - 40.0
            color = BUILDING_PALETTE[(2, 5, 3)[index]]
            b = Building(
                (x, GROUND_ELEVATION + total_height / 2, z),
                (32.0, total_height, 32.0),
                color,
                site.id,
                inset(site.bounds, 7.0),
                "landmark",
                True,
            )
            self.buildings.append(b)
            self.surface((x, z), (44.0, 44.0), "plaza")
            self.box(
                (x, GROUND_ELEVATION + body_height / 2, z),
                (32, body_height, 32),
                color,
                facade=True,
            )
            self.box((x, GROUND_ELEVATION + body_height + 12, z), (20, 24, 20), PANEL_COLOR)
            self.box((x, GROUND_ELEVATION + body_height + 32, z), (6, 16, 6), color)
        for park in self.parks:
            self.surface(park.center, park.size, "path")
            self.surface(*center_size(inset(park.bounds, 4.0)), park.kind)
            x, z = park.center
            w, d = park.size
            self.surface((x, z), (5.0, d), "path")
            self.surface((x, z), (w, 4.0), "path")
            if park is self.parks[0]:
                self.surface((x, z - 155), (55.0, 38.0), "plaza")
                self.surface((x + 75, z + 160), (42.0, 42.0), "plaza")
            else:
                self.surface((x, z + d * 0.22), (min(35.0, w * 0.5), min(30.0, d * 0.3)), "plaza")
        r = self.runway
        rx, _, rz = r.center
        self.surface((rx, rz), (r.width, r.length), "runway")
        self.surface((100, 2500), (20, 2400), "taxiway")
        for z in (1450, 2500, 3550):
            self.surface((50, z), (100, 20), "taxiway")
        # Reapply pavement at taxiway junctions before cutting the markings into it.
        self.surface((rx, rz), (r.width, r.length), "runway")
        self.surface((230, 3000), (250, 500), "apron")
        self.surface((55, 3000), (110, 24), "taxiway")
        for z in np.arange(rz - r.length / 2 + 150, rz + r.length / 2 - 149, 60):
            self.surface((rx, z), (1.2, 30), "marking")
        for z in (rz - r.length / 2 + 30, rz + r.length / 2 - 30):
            for x in (-17, -12, -7, 7, 12, 17):
                self.surface((rx + x, z), (3, 40), "marking")
        self.number(r.designation, rx, rz + r.length / 2 - 95)
        self.number("18", rx, rz - r.length / 2 + 95, reverse=True)
        for z in (2830, 3170):
            self.box(
                (300, GROUND_ELEVATION + 12, z),
                (90, 24, 110),
                BUILDING_PALETTE[1],
                region="airport",
            )
        # Middle hangar opens west onto the apron; no invisible front collision box.
        for center, size in (
            ((344, 12, 3000), (2, 24, 110)),
            ((300, 12, 2946), (88, 24, 2)),
            ((300, 12, 3054), (88, 24, 2)),
            ((300, 23.5, 3000), (90, 1, 110)),
        ):
            self.box(center, size, BUILDING_PALETTE[1], region="airport", bottom=size[1] == 1)
        self.box(
            (180, GROUND_ELEVATION + 18, 2700), (14, 36, 14), BUILDING_PALETTE[0], region="airport"
        )
        self.box(
            (180, GROUND_ELEVATION + 39, 2700), (28, 8, 28), BUILDING_PALETTE[5], region="airport"
        )
        for x in (-70, 390):
            self.surface((x, 2500), (12, 2600), "perimeter")
        for z in (1200, 3800):
            self.surface((160, z), (460, 12), "perimeter")

        # Derive separate envelopes from actual emitted infrastructure, including
        # airport structures. Keep ground only within their compact union.
        def envelope(rectangles):
            return (
                min(r[0] for r in rectangles),
                max(r[1] for r in rectangles),
                min(r[2] for r in rectangles),
                max(r[3] for r in rectangles),
            )

        self.city_footprint = envelope(self.infrastructure_footprints["city"])
        self.airport_footprint = envelope(self.infrastructure_footprints["airport"])
        self.protected_footprints = (self.city_footprint, self.airport_footprint)
        compact = []
        for tile in self.ground_surfaces:
            for a, b, c, d in self.protected_footprints:
                e, f, g, h = tile.bounds
                bounds = (max(a, e), min(b, f), max(c, g), min(d, h))
                if bounds[0] < bounds[1] and bounds[2] < bounds[3]:
                    compact.append(replace(tile, bounds=bounds))
        self.ground_surfaces = compact
        # Store the planar prefix for structural tests and GL validation.
        buildings = self.vertices
        self.vertices = []
        # Split shared edges at every neighbor endpoint. Without this, a short
        # edge meeting a long edge (T junction) can leave isolated sky-colored
        # pixels after projection even though the rectangles cover the plane.
        horizontal = {}
        vertical = {}
        for tile in self.ground_surfaces:
            x0, x1, z0, z1 = tile.bounds
            for z in (z0, z1):
                horizontal.setdefault(z, set()).update((x0, x1))
            for x in (x0, x1):
                vertical.setdefault(x, set()).update((z0, z1))
        horizontal = {z: sorted(xs) for z, xs in horizontal.items()}
        vertical = {x: sorted(zs) for x, zs in vertical.items()}

        def interval(points, start, end):
            return points[bisect_left(points, start) : bisect_right(points, end)]

        for tile in self.ground_surfaces:
            x0, x1, z0, z1 = tile.bounds
            y = tile.elevation
            # Counterclockwise viewed from +Y, matching all other scene faces.
            boundary = [(x0, y, z) for z in interval(vertical[x0], z0, z1)[:-1]]
            boundary += [(x, y, z1) for x in interval(horizontal[z1], x0, x1)[:-1]]
            boundary += [(x1, y, z) for z in reversed(interval(vertical[x1], z0, z1)[1:])]
            boundary += [(x, y, z0) for x in reversed(interval(horizontal[z0], x0, x1)[1:])]
            center = ((x0 + x1) / 2, y, (z0 + z1) / 2)
            for index, point in enumerate(boundary):
                following = boundary[(index + 1) % len(boundary)]
                self.vertices.extend((*p, *tile.color) for p in (center, point, following))
        self.ground_vertex_count = len(self.vertices)
        self.vertices.extend(buildings)
        self.vertices = np.asarray(self.vertices, dtype=np.float32)
        self.vertices.setflags(write=False)
        self.ground_surfaces = tuple(self.ground_surfaces)

    def render_building(self, building, rng):
        """Append building geometry, facade details, and optional roof features.

        Args:
            building: Building geometry and facade metadata.
            rng: Seeded NumPy random generator.
        """
        x, _, z = building.center
        w, h, d = building.size
        color = building.color
        y = GROUND_ELEVATION
        if building.style == "setback":
            self.box((x, y + h * 0.16, z), (w, h * 0.32, d), color, facade=True)
            self.box(
                (x + w * 0.05, y + h * 0.575, z), (w * 0.78, h * 0.51, d * 0.80), color, facade=True
            )
            self.box(
                (x - w * 0.06, y + h * 0.915, z - d * 0.04), (w * 0.52, h * 0.17, d * 0.56), color
            )
        elif building.style == "sections":
            self.box((x - w * 0.22, y + h / 2, z), (w * 0.56, h, d), color, facade=h > 30)
            self.box(
                (x + w * 0.28, y + h * 0.30, z + d * 0.08), (w * 0.44, h * 0.60, d * 0.84), color
            )
        else:
            self.box(building.center, building.size, color, facade=h > 30)
        if building.style == "setback":
            x -= w * 0.06
            z -= d * 0.04
        elif building.style == "sections":
            x -= w * 0.22
        roof = int(rng.integers(3))
        if roof == 0:
            self.box((x, y + h + 1.5, z), (w * 0.35, 3.0, d * 0.38), PANEL_COLOR)
        elif roof == 1:
            self.box((x, y + h + 3.0, z), (w * 0.40, 6.0, d * 0.44), color)
            self.box((x, y + h + 7.0, z), (w * 0.22, 2.0, d * 0.24), PANEL_COLOR)

    def surface(self, center, size, kind):
        """Append a ground surface and its colored pavement geometry.

        Args:
            center: XYZ center position in meters.
            size: XYZ box dimensions in meters.
            kind: Countermeasure classification: flare or chaff.
        """
        x, z = center
        w, d = size
        bounds = tuple(round(float(v), 2) for v in (x - w / 2, x + w / 2, z - d / 2, z + d / 2))
        region = (
            "airport" if kind in ("runway", "taxiway", "apron", "perimeter", "marking") else "city"
        )
        self.infrastructure_footprints[region].append(bounds)
        self.ground_surfaces = [
            piece for tile in self.ground_surfaces for piece in tile.subtract(bounds)
        ]
        self.ground_surfaces.append(GroundSurface(bounds, kind))

    def box(self, center, size, color, facade=False, region=None, bottom=False):
        """Append box faces and optional facade details to the scenery vertex buffer.

        Args:
            center: XYZ center position in meters.
            size: XYZ box dimensions in meters.
            color: RGB display color.
            facade: Whether to append facade detail geometry.
            region: Optional facade region metadata.
            bottom: Whether to include the bottom box face.
        """
        if region == "airport":
            self.solid_boxes.append((tuple(center), tuple(size)))
        if region is not None:
            x, _, z = center
            w, _, d = size
            self.infrastructure_footprints[region].append(
                (x - w / 2, x + w / 2, z - d / 2, z + d / 2)
            )
        corners = np.array(
            [
                np.asarray(center) + np.asarray(size) * np.array((x, y, z)) / 2
                for x in (-1, 1)
                for y in (-1, 1)
                for z in (-1, 1)
            ]
        )
        # Outward winding. Omit hidden undersides: no coplanar base/roof faces.
        faces = (
            ((0, 1, 3, 2), 0.68),
            ((4, 6, 7, 5), 0.85),
            ((2, 3, 7, 6), 1.0),
            ((0, 2, 6, 4), 0.72),
            ((1, 5, 7, 3), 0.90),
        )
        for face, light in faces + ((((0, 4, 5, 1), 0.55),) if bottom else ()):
            if facade and face == (1, 5, 7, 3):
                # One broad dark glass panel replaces a wall patch; five adjoining
                # quads instead of a coplanar overlay or individual window meshes.
                x0, y0, z0 = corners[1]
                x1, y1, _ = corners[7]
                inset_x = (x1 - x0) * 0.18
                inset_y = (y1 - y0) * 0.12
                patches = (
                    (x0, x0 + inset_x, y0, y1, color),
                    (x1 - inset_x, x1, y0, y1, color),
                    (x0 + inset_x, x1 - inset_x, y0, y0 + inset_y, color),
                    (x0 + inset_x, x1 - inset_x, y1 - inset_y, y1, color),
                    (x0 + inset_x, x1 - inset_x, y0 + inset_y, y1 - inset_y, PANEL_COLOR),
                )
                for left, right, bottom, top, material in patches:
                    points = (
                        (left, bottom, z0),
                        (right, bottom, z0),
                        (right, top, z0),
                        (left, top, z0),
                    )
                    shaded = np.asarray(material) * light
                    self.vertices.extend((*points[i], *shaded) for i in (0, 1, 2, 0, 2, 3))
                continue
            shaded = np.asarray(color) * light
            self.vertices.extend((*corners[face[i]], *shaded) for i in (0, 1, 2, 0, 2, 3))

    def number(self, text, x, z, reverse=False):
        """Append runway designation glyph geometry on the ground plane.

        Args:
            text: Runway designation text.
            x: Local east-axis coordinate(s) in meters.
            z: Local south-axis coordinate(s) in meters.
            reverse: Whether to orient designation glyphs toward the opposite runway end.
        """
        glyphs = {
            "3": ("111", "001", "111", "001", "111"),
            "6": ("111", "100", "111", "101", "111"),
            "1": ("010", "110", "010", "010", "111"),
            "8": ("111", "101", "111", "101", "111"),
        }
        sign = -1 if reverse else 1
        for digit, char in enumerate(text):
            for row, line in enumerate(glyphs[char]):
                for col, bit in enumerate(line):
                    if bit == "1":
                        self.surface(
                            (x + sign * ((digit * 4 + col) - 3) * 2, z + sign * (row - 2) * 3),
                            (1.8, 2.8),
                            "marking",
                        )
