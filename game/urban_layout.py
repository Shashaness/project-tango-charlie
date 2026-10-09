"""Seeded street/block/park plan, independent of rendering and vehicle physics."""
from dataclasses import dataclass, replace
import math
import numpy as np

CITY_CENTER = (0., -300.)
CITY_SIZE = 1500.
SEED = 20
ROAD_WIDTHS = {'major': (28., 40.), 'secondary': (12., 20.), 'minor': (7., 12.)}


def center_size(bounds):
    x0, x1, z0, z1 = bounds
    return ((x0+x1)/2, (z0+z1)/2), (x1-x0, z1-z0)


def intersects(a, b):
    return a[0] < b[1] and a[1] > b[0] and a[2] < b[3] and a[3] > b[2]


def subtract_rectangle(bounds, cut):
    x0, x1, z0, z1 = bounds
    ix0, ix1 = max(x0, cut[0]), min(x1, cut[1])
    iz0, iz1 = max(z0, cut[2]), min(z1, cut[3])
    if ix0 >= ix1 or iz0 >= iz1:
        return (bounds,)
    pieces = ((x0, ix0, z0, z1), (ix1, x1, z0, z1),
              (ix0, ix1, z0, iz0), (ix0, ix1, iz1, z1))
    return tuple(p for p in pieces if p[0] < p[1] and p[2] < p[3])


def inset(bounds, amount):
    return (bounds[0]+amount, bounds[1]-amount, bounds[2]+amount, bounds[3]-amount)


def downtown_distance(x, z):
    dx, dz = x-CITY_CENTER[0], z-CITY_CENTER[1]
    radius = math.hypot(dx/650., dz/720.)
    angle = math.atan2(dz, dx)
    return radius*(1.+.12*math.sin(3*angle)+.07*math.cos(2*angle+.8))


def district_for(bounds):
    (x,z), _ = center_size(bounds)
    distance = downtown_distance(x,z)
    return 'downtown' if distance < .48 else 'midtown' if distance < .90 else 'outer'


@dataclass(frozen=True)
class Road:
    bounds: tuple
    kind: str
    axis: str  # x or z direction of travel
    width: float

    def __post_init__(self):
        object.__setattr__(self,'bounds',tuple(round(float(v),2) for v in self.bounds))
        object.__setattr__(self,'width',round(float(self.width),2))


@dataclass(frozen=True)
class Block:
    id: int
    bounds: tuple
    district: str
    kind: str = 'ordinary'

    def __post_init__(self):
        object.__setattr__(self,'bounds',tuple(round(float(v),2) for v in self.bounds))

    @property
    def center(self):return center_size(self.bounds)[0]

    @property
    def size(self):return center_size(self.bounds)[1]


@dataclass(frozen=True)
class Park:
    name: str
    bounds: tuple
    kind: str = 'park'

    def __post_init__(self):
        object.__setattr__(self,'bounds',tuple(round(float(v),2) for v in self.bounds))

    @property
    def center(self):return center_size(self.bounds)[0]

    @property
    def size(self):return center_size(self.bounds)[1]


@dataclass(frozen=True)
class Parcel:
    block_id: int
    bounds: tuple
    district: str
    vacant: bool


@dataclass(frozen=True)
class UrbanLayout:
    seed: int
    roads: tuple
    blocks: tuple
    parks: tuple


def axis_plan(rng, low, high, count, minimum, maximum):
    """Fit varied block spans and street widths exactly within the city boundary."""
    widths = rng.integers(12,21,count+1).astype(float)
    for index in (count//2, count//2+1):widths[index] = rng.integers(28,41)
    spans = rng.uniform(minimum,maximum,count)
    target = high-low-float(widths.sum())
    # Distribute remaining extent without exceeding either district span limit.
    for _ in range(30):
        delta = target-float(spans.sum())
        if abs(delta)<1e-8:break
        room = maximum-spans if delta>0 else spans-minimum
        spans += delta*room/room.sum()
    spans = np.round(spans,2)
    spans[-1] += target-float(spans.sum())
    blocks = []; streets = []; cursor = low
    for index, width in enumerate(widths):
        streets.append((cursor,round(cursor+width,2),'major' if width>=28 else 'secondary'))
        cursor = round(cursor+width,2)
        if index<count:
            blocks.append((cursor,round(cursor+float(spans[index]),2)))
            cursor = round(cursor+float(spans[index]),2)
    return blocks, streets


def generate_layout(seed=SEED):
    rng = np.random.default_rng(seed)
    xs, vertical = axis_plan(rng,-750.,750.,12,60.,140.)
    zs, horizontal = axis_plan(rng,-1050.,450.,9,80.,180.)
    roads = [Road((a,b,-1050.,450.),kind,'z',b-a) for a,b,kind in vertical]
    roads += [Road((-750.,750.,a,b),kind,'x',b-a) for a,b,kind in horizontal]
    cells = []
    # Occasional local mergers remove the intervening street from the same plan.
    for z0,z1 in zs:
        column = 0
        while column<len(xs):
            x0,x1 = xs[column]
            district = district_for((x0,x1,z0,z1))
            merge = column+1<len(xs) and district!='downtown' and vertical[column+1][2]!='major' and rng.random()<.11
            if merge:
                x1 = xs[column+1][1]
                cut = (x0,x1,z0,z1)
                roads = [replace(road,bounds=p) for road in roads
                         for p in subtract_rectangle(road.bounds,cut)]
                cells.append((cut,'merged'));column += 2
            else:
                cells.append(((x0,x1,z0,z1),'ordinary'));column += 1
    # Access-road doglegs subdivide selected long blocks and offset local routes.
    access = []
    planned = []
    for bounds,kind in cells:
        (x,z),(w,d) = center_size(bounds)
        if w>=90 and d>=140 and rng.random()<.20:
            width = float(rng.integers(7,13));offset = float(rng.integers(12,21))
            junction = round(x+float(rng.uniform(-.12,.12))*w,2)
            route = (Road((bounds[0],junction+width/2,z-width/2,z+width/2),'minor','x',width),
                     Road((junction-width/2,junction+width/2,z-width/2,z+offset+width/2),'minor','z',width),
                     Road((junction-width/2,bounds[1],z+offset-width/2,z+offset+width/2),'minor','x',width))
            pieces = [bounds]
            for road in route:
                pieces = [p for part in pieces for p in subtract_rectangle(part,road.bounds)]
            planned.extend((p,'subdivided') for p in pieces);access.extend(route)
        else:planned.append((bounds,kind))
    roads.extend(access)
    central = Park('Central dry park',(-370.,-120.,-525.,-75.))
    # Reserve four entire district blocks; no street grid is independently invented.
    eligible = [(bounds,kind) for bounds,kind in planned if not intersects(bounds,central.bounds)
                and min(center_size(bounds)[1])>=65
                and downtown_distance(*center_size(bounds)[0])>.48]
    selected = []
    for target in ((-520.,-750.),(420.,-560.),(-420.,160.),(480.,190.)):
        choice = min((p for p in eligible if p not in selected),
                     key=lambda p:math.hypot(center_size(p[0])[0][0]-target[0],
                                             center_size(p[0])[0][1]-target[1]))
        selected.append(choice)
    parks = [central]+[Park(f'District open space {i+1}',inset(p[0],4.),
                           'plaza' if i%2 else 'park') for i,p in enumerate(selected)]
    # Roads entering the central park stop at its perimeter, retaining full width.
    cut_roads = []
    for road in roads:
        if intersects(road.bounds,central.bounds):
            x0,x1,z0,z1 = road.bounds
            cut = ((central.bounds[0],central.bounds[1],z0,z1) if road.axis=='x'
                   else (x0,x1,central.bounds[2],central.bounds[3]))
            cut_roads.extend(replace(road,bounds=p) for p in subtract_rectangle(road.bounds,cut))
        else:cut_roads.append(road)
    blocks = []
    for bounds,kind in planned:
        if (bounds,kind) in selected:
            blocks.append(Block(len(blocks),bounds,district_for(bounds),'open_space'))
            continue
        for p in subtract_rectangle(bounds,central.bounds):
            # Narrow remnants by park/access edges stay sandy instead of tiny towers.
            if min(center_size(p)[1])<32:continue
            blocks.append(Block(len(blocks),p,district_for(p),
                                'park_edge' if p!=bounds else kind))
    return UrbanLayout(seed,tuple(cut_roads),tuple(blocks),tuple(parks))


def generate_parcels(block, rng):
    bounds = inset(block.bounds,7.)  # four-meter sidewalk plus building setback
    (_, _),(w,d) = center_size(bounds)
    if min(w,d)<18:return ()
    if block.district=='downtown':
        columns = max(1,min(3,int(w/25)));rows = max(1,min(5,int(d/28)))
        vacancy = .04
    elif block.district=='midtown':
        columns = max(1,min(3,int(w/34)));rows = max(1,min(3,int(d/40)))
        if columns>1 and rng.random()<.25:columns -= 1
        if rows>1 and rng.random()<.25:rows -= 1
        vacancy = .18
    else:
        columns = 1 if rng.random()<.65 else max(1,min(2,int(w/45)))
        rows = 1 if rng.random()<.45 else max(1,min(2,int(d/55)))
        vacancy = .38
    def breaks(start,length,count):
        weights = rng.uniform(.8,1.2,count);spans = length*weights/weights.sum()
        return np.r_[start,start+np.cumsum(spans)]
    xs = breaks(bounds[0],w,columns);zs = breaks(bounds[2],d,rows)
    return tuple(Parcel(block.id,(float(xs[i]),float(xs[i+1]),float(zs[j]),float(zs[j+1])),
                        block.district,bool(rng.random()<vacancy))
                 for i in range(columns) for j in range(rows))
