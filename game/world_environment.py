"""Deterministic, desolate M20.2 scenery; CPU generation needs no OpenGL."""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, replace
import numpy as np
from game.atmospheric_physics import PARAMETERS
from game.urban_layout import (CITY_CENTER, CITY_SIZE, SEED, center_size, generate_layout,
                               generate_parcels, inset, subtract_rectangle, downtown_distance)


@dataclass(frozen=True)
class Runway:
    center: tuple = (0., 0., 2500.)
    length: float = 2400.
    width: float = 45.
    forward: tuple = (0., 0., -1.)
    designation: str = '36'

    @property
    def spawn_xz(self):
        return (self.center[0], self.center[2]+self.length/2-120.)


RUNWAY = Runway()
GROUND_ELEVATION = PARAMETERS.ground_altitude
TERRAIN_BOUNDS = (-9000., 9000., -8000., 10000.)  # xmin, xmax, zmin, zmax
BUILDING_PALETTE = (
    (.66, .65, .62),  # weathered light concrete
    (.51, .51, .49),  # medium concrete
    (.36, .37, .37),  # dark concrete
    (.24, .25, .26),  # charcoal
    (.13, .14, .15),  # near-black
    (.43, .47, .49),  # desaturated steel
    (.47, .42, .36),  # faded industrial brown
)
PANEL_COLOR = (.12, .14, .15)
GROUND_PALETTE = {
    'desert': (.57, .48, .36),
    'street': (.22, .22, .21),
    'sidewalk': (.48, .46, .41),
    'lot': (.42, .36, .28),
    'runway': (.15, .16, .16),
    'taxiway': (.20, .21, .20),
    'apron': (.34, .33, .30),
    'perimeter': (.27, .26, .23),
    'marking': (.85, .83, .74),
    'park': (.44, .43, .32),
    'plaza': (.48, .47, .43),
    'path': (.60, .56, .47),
}


@dataclass(frozen=True)
class Building:
    center: tuple
    size: tuple
    color: tuple
    block_id: int = -1
    parcel: tuple = ()
    style: str = 'slab'
    landmark: bool = False

    @property
    def footprint(self):
        x,_,z = self.center;w,_,d = self.size
        return (x-w/2,x+w/2,z-d/2,z+d/2)


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
        """Keep only this rectangle's area outside an overlaid rectangle."""
        return tuple(replace(self,bounds=p) for p in subtract_rectangle(self.bounds,bounds))


def building_height(x, z, rng):
    # Smooth district anchors: fringe -> industrial -> inner city -> core.
    distance = downtown_distance(x, z)
    anchors = (0., .30, .58, .85, 1.08, 1.5)
    lows = (210., 140., 60., 20., 8., 8.)
    highs = (320., 270., 160., 65., 30., 30.)
    band = min(len(anchors)-2, max(0, int(np.searchsorted(anchors, distance))-1))
    u = float(np.clip((distance-anchors[band])/(anchors[band+1]-anchors[band]), 0., 1.))
    u = u*u*(3.-2.*u)
    low = (1.-u)*lows[band]+u*lows[band+1]
    high = (1.-u)*highs[band]+u*highs[band+1]
    return float(rng.uniform(low, high))


class WorldEnvironment:
    def __init__(self, seed=SEED):
        self.runway = RUNWAY
        self.blocks = []
        self.buildings = []
        self.vertices = []
        self.ground_surfaces = [GroundSurface(TERRAIN_BOUNDS, 'desert')]
        rng = np.random.default_rng(seed)
        self.layout = generate_layout(seed)
        self.blocks = self.layout.blocks
        self.roads = self.layout.roads
        self.parks = self.layout.parks
        self.parcels = []
        for block in self.blocks:
            self.surface(block.center,block.size,'sidewalk')
            self.surface(*center_size(inset(block.bounds,4.)),'lot')
        for road in self.roads:
            self.surface(*center_size(road.bounds),'street')
        # Landmark sites come from buildable downtown blocks, never an independent grid.
        sites = []
        for target in ((0.,-300.),(170.,-410.),(-70.,-70.)):
            site = min((b for b in self.blocks if b.district=='downtown' and
                        b.kind!='open_space' and min(b.size)>=55 and b not in sites),
                       key=lambda b:np.hypot(b.center[0]-target[0],b.center[1]-target[1]))
            sites.append(site)
        for block in self.blocks:
            if block.kind=='open_space' or block in sites:continue
            parcels = generate_parcels(block,rng)
            self.parcels.extend(parcels)
            for parcel in parcels:
                if parcel.vacant:continue
                (px,pz),(pw,pd) = center_size(inset(parcel.bounds,2.5))
                factor = (.84,.98) if block.district=='downtown' else (.62,.90)
                w = pw*float(rng.uniform(*factor));d = pd*float(rng.uniform(*factor))
                x = px+float(rng.uniform(-1,1))*(pw-w)/2
                z = pz+float(rng.uniform(-1,1))*(pd-d)/2
                height = building_height(x,z,rng)
                if block.district=='outer':
                    style = 'warehouse' if rng.random()<.75 else 'slab'
                    if style=='warehouse':height = float(rng.uniform(8.,24.))
                else:style = str(rng.choice(('slab','setback','sections'),p=(.25,.50,.25) if block.district=='downtown' else (.4,.25,.35)))
                color = BUILDING_PALETTE[int(rng.integers(len(BUILDING_PALETTE)))]
                b = Building((x,GROUND_ELEVATION+height/2,z),(w,height,d),color,
                             block.id,parcel.bounds,style)
                self.buildings.append(b);self.render_building(b,rng)
        for index,(site,total_height) in enumerate(zip(sites,(420.,385.,350.))):
            x,z = site.center;body_height = total_height-40.
            color = BUILDING_PALETTE[(2,5,3)[index]]
            b = Building((x,GROUND_ELEVATION+total_height/2,z),(32.,total_height,32.),
                         color,site.id,inset(site.bounds,7.),'landmark',True)
            self.buildings.append(b)
            self.surface((x,z),(44.,44.),'plaza')
            self.box((x,GROUND_ELEVATION+body_height/2,z),(32,body_height,32),color,facade=True)
            self.box((x,GROUND_ELEVATION+body_height+12,z),(20,24,20),PANEL_COLOR)
            self.box((x,GROUND_ELEVATION+body_height+32,z),(6,16,6),color)
        for park in self.parks:
            self.surface(park.center,park.size,'path')
            self.surface(*center_size(inset(park.bounds,4.)),park.kind)
            x,z = park.center;w,d = park.size
            self.surface((x,z),(5.,d),'path')
            self.surface((x,z),(w,4.),'path')
            if park is self.parks[0]:
                self.surface((x,z-155),(55.,38.),'plaza')
                self.surface((x+75,z+160),(42.,42.),'plaza')
            else:self.surface((x,z+d*.22),(min(35.,w*.5),min(30.,d*.3)),'plaza')
        r = self.runway
        rx, _, rz = r.center
        self.surface((rx, rz), (r.width, r.length), 'runway')
        self.surface((100, 2500), (20, 2400), 'taxiway')
        for z in (1450, 2500, 3550):
            self.surface((50, z), (100, 20), 'taxiway')
        # Reapply pavement at taxiway junctions before cutting the markings into it.
        self.surface((rx, rz), (r.width, r.length), 'runway')
        self.surface((230, 3000), (250, 500), 'apron')
        for z in np.arange(rz-r.length/2+150, rz+r.length/2-149, 60):
            self.surface((rx, z), (1.2, 30), 'marking')
        for z in (rz-r.length/2+30, rz+r.length/2-30):
            for x in (-17, -12, -7, 7, 12, 17):
                self.surface((rx+x, z), (3, 40), 'marking')
        self.number(r.designation, rx, rz+r.length/2-95)
        self.number('18', rx, rz-r.length/2+95, reverse=True)
        for z in (2830, 3000, 3170):
            self.box((300, GROUND_ELEVATION+12, z), (90, 24, 110), BUILDING_PALETTE[1])
        self.box((180, GROUND_ELEVATION+18, 2700), (14, 36, 14), BUILDING_PALETTE[0])
        self.box((180, GROUND_ELEVATION+39, 2700), (28, 8, 28), BUILDING_PALETTE[5])
        for x in (-70, 390):
            self.surface((x, 2500), (12, 2600), 'perimeter')
        for z in (1200, 3800):
            self.surface((160, z), (460, 12), 'perimeter')
        # Store the planar prefix for structural tests and GL validation.
        buildings = self.vertices
        self.vertices = []
        # Split shared edges at every neighbor endpoint. Without this, a short
        # edge meeting a long edge (T junction) can leave isolated sky-colored
        # pixels after projection even though the rectangles cover the plane.
        horizontal = {}; vertical = {}
        for tile in self.ground_surfaces:
            x0, x1, z0, z1 = tile.bounds
            for z in (z0, z1):horizontal.setdefault(z, set()).update((x0, x1))
            for x in (x0, x1):vertical.setdefault(x, set()).update((z0, z1))
        horizontal = {z: sorted(xs) for z, xs in horizontal.items()}
        vertical = {x: sorted(zs) for x, zs in vertical.items()}
        def interval(points, start, end):
            return points[bisect_left(points, start):bisect_right(points, end)]
        for tile in self.ground_surfaces:
            x0, x1, z0, z1 = tile.bounds
            y = tile.elevation
            # Counterclockwise viewed from +Y, matching all other scene faces.
            boundary = [(x0, y, z) for z in interval(vertical[x0], z0, z1)[:-1]]
            boundary += [(x, y, z1) for x in interval(horizontal[z1], x0, x1)[:-1]]
            boundary += [(x1, y, z) for z in reversed(interval(vertical[x1], z0, z1)[1:])]
            boundary += [(x, y, z0) for x in reversed(interval(horizontal[z0], x0, x1)[1:])]
            center = ((x0+x1)/2, y, (z0+z1)/2)
            for index, point in enumerate(boundary):
                following = boundary[(index+1)%len(boundary)]
                self.vertices.extend((*p, *tile.color) for p in (center, point, following))
        self.ground_vertex_count = len(self.vertices)
        self.vertices.extend(buildings)
        self.vertices = np.asarray(self.vertices, dtype=np.float32)
        self.vertices.setflags(write=False)
        self.ground_surfaces = tuple(self.ground_surfaces)

    def render_building(self, building, rng):
        x,_,z = building.center;w,h,d = building.size;color = building.color
        y = GROUND_ELEVATION
        if building.style=='setback':
            self.box((x,y+h*.16,z),(w,h*.32,d),color,facade=True)
            self.box((x+w*.05,y+h*.575,z),(w*.78,h*.51,d*.80),color,facade=True)
            self.box((x-w*.06,y+h*.915,z-d*.04),(w*.52,h*.17,d*.56),color)
        elif building.style=='sections':
            self.box((x-w*.22,y+h/2,z),(w*.56,h,d),color,facade=h>30)
            self.box((x+w*.28,y+h*.30,z+d*.08),(w*.44,h*.60,d*.84),color)
        else:self.box(building.center,building.size,color,facade=h>30)
        if building.style=='setback':x -= w*.06;z -= d*.04
        elif building.style=='sections':x -= w*.22
        roof = int(rng.integers(3))
        if roof==0:
            self.box((x,y+h+1.5,z),(w*.35,3.,d*.38),PANEL_COLOR)
        elif roof==1:
            self.box((x,y+h+3.,z),(w*.40,6.,d*.44),color)
            self.box((x,y+h+7.,z),(w*.22,2.,d*.24),PANEL_COLOR)

    def surface(self, center, size, kind):
        """Compose colors on the CPU by cutting tiles, never by layering planes."""
        x, z = center; w, d = size
        bounds = tuple(round(float(v),2) for v in (x-w/2,x+w/2,z-d/2,z+d/2))
        self.ground_surfaces = [piece for tile in self.ground_surfaces
                                for piece in tile.subtract(bounds)]
        self.ground_surfaces.append(GroundSurface(bounds, kind))

    def box(self, center, size, color, facade=False):
        corners = np.array([np.asarray(center)+np.asarray(size)*np.array((x, y, z))/2
                            for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        # Outward winding. Omit hidden undersides: no coplanar base/roof faces.
        for face, light in (((0, 1, 3, 2), .68), ((4, 6, 7, 5), .85),
                            ((2, 3, 7, 6), 1.), ((0, 2, 6, 4), .72), ((1, 5, 7, 3), .90)):
            if facade and face == (1, 5, 7, 3):
                # One broad dark glass panel replaces a wall patch; five adjoining
                # quads instead of a coplanar overlay or individual window meshes.
                x0, y0, z0 = corners[1]
                x1, y1, _ = corners[7]
                inset_x = (x1-x0)*.18; inset_y = (y1-y0)*.12
                patches = ((x0, x0+inset_x, y0, y1, color),
                           (x1-inset_x, x1, y0, y1, color),
                           (x0+inset_x, x1-inset_x, y0, y0+inset_y, color),
                           (x0+inset_x, x1-inset_x, y1-inset_y, y1, color),
                           (x0+inset_x, x1-inset_x, y0+inset_y, y1-inset_y, PANEL_COLOR))
                for left, right, bottom, top, material in patches:
                    points = ((left, bottom, z0), (right, bottom, z0),
                              (right, top, z0), (left, top, z0))
                    shaded = np.asarray(material)*light
                    self.vertices.extend((*points[i], *shaded) for i in (0, 1, 2, 0, 2, 3))
                continue
            shaded = np.asarray(color)*light
            self.vertices.extend((*corners[face[i]], *shaded) for i in (0, 1, 2, 0, 2, 3))

    def number(self, text, x, z, reverse=False):
        glyphs = {'3': ('111', '001', '111', '001', '111'),
                  '6': ('111', '100', '111', '101', '111'),
                  '1': ('010', '110', '010', '010', '111'),
                  '8': ('111', '101', '111', '101', '111')}
        sign = -1 if reverse else 1
        for digit, char in enumerate(text):
            for row, line in enumerate(glyphs[char]):
                for col, bit in enumerate(line):
                    if bit == '1':
                        self.surface((x+sign*((digit*4+col)-3)*2, z+sign*(row-2)*3),
                                     (1.8, 2.8), 'marking')
