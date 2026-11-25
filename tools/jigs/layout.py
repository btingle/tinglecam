"""Shared blank layout for the glue-up jig and the op0 plate (same frame, so their holes always agree).

Glue frame: looking down on the stack, fixture-hole (track top) side up.
Rings: pieces whose end faces meet at a corner are placed in quadrant II with their ends facing the
axes, then mirrored into the other quadrants -> a closed loop of track with `web` gaps on the axes.
Rows: pieces with parallel end faces sit side by side.
"""
from dataclasses import dataclass, field

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

MIRROR_X = (-1, 0, 0, 1, 0, 0)   # x -> -x
MIRROR_Y = (1, 0, 0, -1, 0, 0)
ROT180 = (-1, 0, 0, -1, 0, 0)


def compose(m2, m1):
    """Affine applying m1 then m2 (shapely 6-tuples)."""
    a = np.array([[m1[0], m1[1], m1[4]], [m1[2], m1[3], m1[5]], [0, 0, 1]])
    b = np.array([[m2[0], m2[1], m2[4]], [m2[2], m2[3], m2[5]], [0, 0, 1]])
    c = b @ a
    return (c[0, 0], c[0, 1], c[1, 0], c[1, 1], c[0, 2], c[1, 2])


def rotation(deg, origin=(0, 0)):
    r = np.radians(deg)
    c, s = np.cos(r), np.sin(r)
    ox, oy = origin
    return (c, -s, s, c, ox - c * ox + s * oy, oy - s * ox - c * oy)


def translation(dx, dy):
    return (1, 0, 0, 1, dx, dy)


def apply(m, geom):
    return affinity.affine_transform(geom, m)


@dataclass
class Species:
    name: str
    base: str            # template whose outline defines slot A
    members: tuple       # templates made with this species' jigs
    copies: int          # rings (ring layouts) or pieces (row layouts)
    corners: tuple = ()  # ring layouts: type in each corner [base, mirror X, mirror Y, rotated 180]

    def slot_type(self, i):
        return self.corners[i % len(self.corners)] if self.corners else self.base


SPECIES = {
    "lr": Species("lr", "L", ("L", "R"), 2, ("L", "R", "R", "L")),
    "slr": Species("slr", "sL", ("sL", "sR", "Ls", "Rs"), 1, ("sL", "sR", "Ls", "Rs")),
    "sSs": Species("sSs", "sSs", ("sSs",), 4),
}


@dataclass
class Slot:
    m: tuple             # template frame -> glue frame
    kind: str            # "A" = base outline, "B" = its mirror
    ring: int
    track: str = ""      # the type glued and machined in this slot


@dataclass
class Layout:
    species: Species
    kind: str
    slots: list
    blank: object        # base blank outline (template frame, with allowance)
    holes: list          # base fixture holes (template frame)
    centres: list        # ring/row centres
    bolts: list = field(default_factory=list)
    plate: tuple = None  # (x0, y0, x1, y1)
    placements: dict = field(default_factory=dict)  # slot index -> [slot type frame -> glue frame]

    def blanks(self):
        return [apply(s.m, self.blank) for s in self.slots]

    def fixture_holes(self):
        return [apply(s.m, h) for s in self.slots for h in self.holes]

    def cutouts(self, clearance):
        return unary_union([b.buffer(clearance, join_style=2) for b in self.blanks()])


def corner_of(track):
    """Intersection of the two end-face lines, or None when they are parallel."""
    (p0, n0), (p1, n1) = track.ends
    f0, f1 = np.array([-n0[1], n0[0]]), np.array([-n1[1], n1[0]])
    a = np.column_stack([f0, -f1])
    if abs(np.linalg.det(a)) < 1e-6:
        return None
    s, _ = np.linalg.solve(a, p1 - p0)
    return p0 + s * f0


def quadrant_transform(track, blank, half_web):
    """Template frame -> quadrant II with half_web clearance to both axes (rotation only)."""
    cx, cy = corner_of(track)
    m = translation(-cx, -cy)
    for _ in range(4):
        c = apply(m, blank).centroid
        if c.x < 0 < c.y:
            break
        m = compose(rotation(90), m)
    else:
        raise ValueError(f"{track.name}: outline does not fit a quadrant")
    b = apply(m, blank).bounds
    return compose(translation(-half_web - b[2], half_web - b[1]), m)


def invert(m):
    a = np.array([[m[0], m[1], m[4]], [m[2], m[3], m[5]], [0, 0, 1]])
    i = np.linalg.inv(a)
    return (i[0, 0], i[0, 1], i[1, 0], i[1, 1], i[0, 2], i[1, 2])


def registrations(src, dst, tol=0.01):
    """Rigid placements (rotation + translation, never mirrored: a blank has a top) of src onto dst."""
    cs, cd = src.centroid, dst.centroid
    hits = []
    for deg in range(360):
        m = compose(translation(cd.x, cd.y), compose(rotation(deg), translation(-cs.x, -cs.y)))
        err = apply(m, src).symmetric_difference(dst).area / dst.area
        if err < tol:
            hits.append((deg, err, m))
    best = []
    for deg, err, m in hits:  # one per cluster of neighbouring angles
        if best and (deg - best[-1][0]) <= 2:
            if err < best[-1][1]:
                best[-1] = (deg, err, m)
        else:
            best.append((deg, err, m))
    if len(best) > 1 and best[0][0] <= 2 and best[-1][0] >= 357:
        best = [min(best[0], best[-1], key=lambda b: b[1])] + best[1:-1]
    return [m for _, _, m in best]


def place_members(layout, tracks, p):
    """Every rigid way the assigned type's blank seats in each slot (normally exactly one)."""
    placements = {}
    for i, slot in enumerate(layout.slots):
        regs = registrations(tracks[slot.track].outline.buffer(p.laser_allowance), apply(slot.m, layout.blank))
        if not regs:
            raise ValueError(f"{layout.species.name}: {slot.track} does not fit slot {i} ({slot.kind})")
        placements[i] = regs
    return placements


def _first_clear(origin, direction, obstacles, clearance, start=0.0, step=0.25, limit=400):
    d = np.asarray(direction, float)
    d /= np.linalg.norm(d)
    s = start
    while s < limit:
        pt = Point(*(np.asarray(origin) + s * d))
        if pt.distance(obstacles) >= clearance:
            return pt
        s += step
    raise ValueError("No clear bolt position found")


def _beyond(centre, direction, region):
    """Distance along a ray from centre to where it leaves region's convex hull."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    far = LineString([centre, np.asarray(centre) + d * 1000])
    hit = far.intersection(region.convex_hull.boundary)
    pts = [np.array(g.coords[0]) for g in getattr(hit, "geoms", [hit]) if not g.is_empty]
    return max((np.linalg.norm(q - centre) for q in pts), default=0.0)


def ring_bolts(centre, region, obstacles, p):
    """Centre bolt (if it fits) + one bolt per quadrant along the ring's bounding-box diagonals."""
    r = p.glue_bolt_d / 2 + p.glue_bolt_wall
    cx, cy = centre
    x0, y0, x1, y1 = region.bounds
    w, h = max(x1 - cx, cx - x0), max(y1 - cy, cy - y0)
    bolts = [Point(cx, cy)] if Point(cx, cy).distance(obstacles) >= r else []
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        d = (sx * w, sy * h)
        bolts.append(_first_clear(centre, d, obstacles, r, start=_beyond(centre, d, region)))
    return bolts


def row_bolts(centre, region, obstacles, p):
    """Four diagonal bolts + two at the middle of the long sides."""
    r = p.glue_bolt_d / 2 + p.glue_bolt_wall
    cx, cy = centre
    x0, y0, x1, y1 = region.bounds
    w, h = max(x1 - cx, cx - x0), max(y1 - cy, cy - y0)
    dirs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    dirs += [(0, 1), (0, -1)]
    dirs = [(sx * w, sy * h) for sx, sy in dirs]
    bolts = [_first_clear(centre, d, obstacles, r, start=_beyond(centre, d, region)) for d in dirs]
    return bolts


def build_layout(species, tracks, p):
    base = tracks[species.base]
    blank = base.outline.buffer(p.laser_allowance)
    holes = base.fixture_holes
    clear = p.guide_clearance

    if corner_of(base) is not None:
        q = quadrant_transform(base, blank, p.web / 2)
        ring = [Slot(q, "A", 0), Slot(compose(MIRROR_X, q), "B", 0),
                Slot(compose(MIRROR_Y, q), "B", 0), Slot(compose(ROT180, q), "A", 0)]
        ring_shape = unary_union([apply(s.m, blank) for s in ring]).buffer(clear)
        width = ring_shape.bounds[2] - ring_shape.bounds[0]
        # space rings apart until every ring's bolts clear every other ring
        pitch = width + p.web
        for _ in range(int(width) + 1):
            slots = [Slot(compose(translation(i * pitch, 0), s.m), s.kind, i)
                     for i in range(species.copies) for s in ring]
            layout = Layout(species, "ring", slots, blank, holes,
                            [(i * pitch, 0.0) for i in range(species.copies)])
            cut = layout.cutouts(clear)
            bolts = []
            for i, c in enumerate(layout.centres):
                region = unary_union([apply(s.m, blank) for s in slots if s.ring == i])
                bolts += ring_bolts(c, region, cut, p)
            if _bolts_ok(bolts, cut, p):
                break
            pitch += 1.0
        else:
            raise ValueError(f"{species.name}: no ring spacing gives clear bolts")
    else:
        b = blank.bounds
        pitch = (b[2] - b[0]) + p.web
        slots = [Slot(translation(i * pitch - (b[0] + b[2]) / 2 - (species.copies - 1) * pitch / 2,
                                  -(b[1] + b[3]) / 2), "A", 0) for i in range(species.copies)]
        layout = Layout(species, "row", slots, blank, holes, [(0.0, 0.0)])
        cut = layout.cutouts(clear)
        bolts = row_bolts((0.0, 0.0), unary_union(layout.blanks()), cut, p)
        if not _bolts_ok(bolts, cut, p):
            raise ValueError(f"{species.name}: bolts collide")

    # centre everything on the origin
    allb = unary_union(layout.blanks()).bounds
    shift = translation(-(allb[0] + allb[2]) / 2, -(allb[1] + allb[3]) / 2)
    for s in layout.slots:
        s.m = compose(shift, s.m)
    layout.centres = [apply(shift, Point(c)).coords[0] for c in layout.centres]
    layout.bolts = [apply(shift, b) for b in bolts]
    # symmetric rounded-rect plate around cutouts and bolts
    stuff = unary_union([layout.cutouts(clear)] + [b.buffer(p.glue_bolt_d / 2 + p.glue_plate_margin)
                                                   for b in layout.bolts])
    x0, y0, x1, y1 = stuff.bounds
    hx, hy = max(-x0, x1), max(-y0, y1)
    layout.plate = (-hx, -hy, hx, hy)
    for i, s in enumerate(layout.slots):
        s.track = species.slot_type(i)
    layout.placements = place_members(layout, tracks, p)
    return layout


def _bolts_ok(bolts, cut, p):
    r = p.glue_bolt_d / 2 + p.glue_bolt_wall
    if any(b.distance(cut) < r - 1e-6 for b in bolts):
        return False
    return all(a.distance(b) >= p.glue_bolt_d + 2 * p.glue_bolt_wall
               for i, a in enumerate(bolts) for b in bolts[i + 1:])
