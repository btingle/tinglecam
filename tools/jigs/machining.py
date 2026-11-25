"""3D machining jigs (OpenCascade solids).

op0  plate the blanks bolt to (M4 into the fixture holes), same layout as the glue-up.
     Mill frame: plate z in [-t, 0], blanks on top (track bottom up), countersinks underneath.
     The glue frame looks at the fixture-hole side, the mill frame at the other side: y -> -y.

op1  top-side soft jaws, per track type, as many pieces as fit the vise jaw.
     Pocket = the op0 result: finished bottom and walls, extruded straight up (piece drops in).
     A raised wall follows the track (and its elevation) up to op1_top_clearance below the
     finished top. The block is split along each piece's centreline, so one jaw holds the inner
     wall and the other the outer wall; pieces are rotated so that split is a graph y(x) and the
     jaws open along the vise axis (Y).

op2  end-face jaws, per track type. A block around the piece, both end faces standing
     op2_face_proud above the block, split across the track's thickness (a horizontal plane
     in the track frame) so the piece lifts straight out of the lower jaw. Heat-set inserts in
     jaw A + M3 bolts through jaw B hold the pair together when it is turned over. The vise
     clamps along the split normal, which stays horizontal with either end face up.
"""
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from . import occ
from .layout import apply, rotation, compose, translation


@dataclass
class Jig:
    name: str
    parts: dict                         # name -> solid (the parts to make)
    context: dict = field(default_factory=dict)   # name -> solid (pieces, for previews/checks)
    info: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------------------------- op0
def build_op0(layout, tracks, p):
    base = tracks[layout.species.base]
    t = p.op0_thickness
    blanks = [apply((1, 0, 0, -1, 0, 0), b) for b in layout.blanks()]
    holes = [apply((1, 0, 0, -1, 0, 0), h) for h in layout.fixture_holes()]
    x0, y0, x1, y1 = unary_union(blanks).bounds
    hx, hy = max(-x0, x1) + p.op0_margin, max(-y0, y1) + p.op0_margin
    plate = occ.rounded_rect(-hx, -hy, hx, hy, p.op0_corner_r, -t, 0)
    csk_depth = (p.op0_csk_d - p.op0_bolt_d) / 2
    tools = []
    for h in holes:
        tools.append(occ.cylinder(h.x, h.y, -t - 1, 1, p.op0_bolt_d))
        tools.append(occ.cone(h.x, h.y, -t - 0.01, -t + csk_depth, p.op0_csk_d + 0.02, p.op0_bolt_d))
    plate = occ.cut(plate, *tools)
    bz = base.sheets * p.sheet
    context = {f"blank{i}": occ.prism(b, 0, bz) for i, b in enumerate(blanks)}
    return Jig(f"{layout.species.name}_op0_plate", {"plate": plate}, context,
               {"size": (2 * hx, 2 * hy, t), "holes": holes, "blanks": blanks})


# ---------------------------------------------------------------------------------------------- op1
def _place(shape, m2):
    return occ.transform(shape, occ.affine2d(m2))


def build_op1(track, p):
    h = track.headings
    mid = (h.min() + h.max()) / 2
    if np.degrees(np.abs(h - mid).max()) > 80:
        raise ValueError(f"{track.name}: turns too far for a single straight-opening op1 split")
    rot = rotation(-np.degrees(mid))
    clear, wall = p.op1_pocket_clearance, p.op1_wall
    px0, py0, px1, py1 = track.profile_poly.bounds
    blank_top = track.blank_z[1]
    pocket_poly = track.filled_profile(extend_up=blank_top - py0 + 5).buffer(clear, join_style=2)
    wall_poly = box(px0 - clear - wall, py0 - track.p.elevation - 3, px1 + clear + wall, py1 - p.op1_top_clearance)

    # how many fit along the vise jaw
    footprint = apply(rot, track.outline.buffer(clear + wall, join_style=2))
    fw = footprint.bounds[2] - footprint.bounds[0]
    W = p.vise_jaw_width
    n = max(1, int((W - p.op1_piece_gap) // (fw + p.op1_piece_gap)))
    xs = (np.arange(n) - (n - 1) / 2) * (fw + p.op1_piece_gap)
    fcx, fcy = (footprint.bounds[0] + footprint.bounds[2]) / 2, (footprint.bounds[1] + footprint.bounds[3]) / 2
    slots = [compose(translation(x - fcx, -fcy), rot) for x in xs]

    pocket = track.sweep_solid(pocket_poly)
    walls = track.sweep_solid(wall_poly)
    zlo, zhi = track.z_range
    z_floor = zlo - clear - p.op1_floor
    top = track.envelope[1] - p.op1_top_clearance
    outlines = [apply(m, track.outline.buffer(clear + wall, join_style=2)) for m in slots]
    ys = unary_union(outlines).bounds
    y0, y1 = ys[1] - p.op1_margin, ys[3] + p.op1_margin
    block = occ.box(-W / 2, y0, z_floor, W / 2, y1, top)
    jaw = occ.fuse(block, *[_place(walls, m) for m in slots])
    jaw = occ.common(jaw, occ.box(-W / 2, y0, z_floor, W / 2, y1, zhi + 50))  # flat bottom on the vise
    jaw = occ.cut(jaw, *[_place(pocket, m) for m in slots])

    # split: left edge -> each centreline -> right edge, as one x-monotone polyline
    path = []
    for m in slots:
        c = np.array(apply(m, track.centreline).coords)
        path.extend(c if c[-1, 0] > c[0, 0] else c[::-1])
    path = [(-W / 2 - 1, path[0][1])] + [tuple(q) for q in path] + [(W / 2 + 1, path[-1][1])]
    split = LineString(path)
    gap = split.buffer(p.split_gap / 2, cap_style=2, join_style=2)
    jaws = occ.solids(occ.cut(jaw, occ.prism(gap, z_floor - 1, zhi + 50)))
    jaws.sort(key=lambda s: occ.centroid(s)[1])
    pieces = {f"piece{i}": _place(track.solid, m) for i, m in enumerate(slots)}
    return Jig(f"{track.name}_op1_jaws", {f"jaw_{k}": s for k, s in zip("AB", jaws)} if len(jaws) == 2
               else {f"jaw_{i}": s for i, s in enumerate(jaws)}, pieces,
               {"count": n, "size": (W, y1 - y0, top - z_floor), "split": split, "rotation": -np.degrees(mid),
                "outlines": outlines, "pocket": pocket, "track_solid": track.solid})


# ---------------------------------------------------------------------------------------------- op2
def _halfplane(point, normal, offset, size=1000):
    """Points q with (q - point) . normal <= -offset."""
    n = np.asarray(normal) / np.linalg.norm(normal)
    t = np.array([-n[1], n[0]])
    o = np.asarray(point) - n * offset
    return Polygon([o + t * size, o - t * size, o - t * size - n * size, o + t * size - n * size])


def orientation_matrix(normal):
    """3x4 rotation taking a horizontal outward normal to +Z (piece rests on the opposite face)."""
    n = np.array([normal[0], normal[1], 0.0])
    n /= np.linalg.norm(n)
    side = np.array([-n[1], n[0], 0.0])        # stays horizontal: becomes mill X
    z = np.array([0, 0, 1.0])                  # track Z (split normal) becomes mill Y
    r = np.vstack([side, z, n])                # rows: new x, y, z expressed in track frame
    return np.hstack([r, np.zeros((3, 1))]).tolist()


def build_op2(track, p):
    lo_env, hi_env = track.envelope
    if lo_env >= hi_env:
        raise ValueError(f"{track.name}: no horizontal plane cuts every section; op2 split needs another rule")
    c = (lo_env + hi_env) / 2
    clear, wall, g = p.op2_pocket_clearance, p.op2_wall, p.split_gap / 2
    zlo, zhi = track.z_range
    z0, z1 = zlo - wall, zhi + wall
    x0, y0, x1, y1 = track.outline.bounds
    keep_out = track.outline.buffer(clear + p.op2_insert_d / 2 + p.op2_insert_wall)
    # widen the jaws (not past the end faces) until at least two bolts fit beside the pocket
    for wall in np.arange(wall, wall + 20.5, 0.5):
        foot = box(x0 - wall, y0 - wall, x1 + wall, y1 + wall)
        for pt, n in track.ends:
            foot = foot.intersection(_halfplane(pt, n, p.op2_face_proud))
        room = foot.buffer(-(p.op2_bolt_head_d / 2 + 1.5)).difference(keep_out)
        pts = np.array([q for poly in getattr(room, "geoms", [room]) if not poly.is_empty
                        for q in poly.exterior.coords]).reshape(-1, 2)
        bolts = []
        for d in ((1, 1), (-1, 1), (-1, -1), (1, -1), (1, 0), (-1, 0), (0, 1), (0, -1)):
            if len(pts) and len(bolts) < 4:  # extreme points: diagonals first, then sides
                q = Point(pts[np.argmax(pts @ np.array(d))])
                if all(q.distance(b) > p.op2_bolt_head_d + 2 for b in bolts):
                    bolts.append(q)
        if len(bolts) >= 2:
            break
    else:
        raise ValueError(f"{track.name}: no room for op2 bolts")
    pocket = track.sweep_solid(track.filled_profile().buffer(clear, join_style=2))
    body = occ.cut(occ.prism(foot, z0, z1), pocket)
    lower = occ.common(body, occ.box(-500, -500, z0 - 1, 500, 500, c - g))
    upper = occ.common(body, occ.box(-500, -500, c + g, 500, 500, z1 + 1))
    lower = occ.cut(lower, *[occ.cylinder(b.x, b.y, c - g - p.op2_insert_depth, c, p.op2_insert_d) for b in bolts])
    upper = occ.cut(upper, *[occ.cylinder(b.x, b.y, c, z1 + 1, p.op2_bolt_d) for b in bolts],
                    *[occ.cylinder(b.x, b.y, z1 - p.op2_bolt_head_depth, z1 + 1, p.op2_bolt_head_d) for b in bolts])
    bolt_len = (z1 - p.op2_bolt_head_depth) - (c - g) + p.op2_insert_depth * 0.8
    fx0, fy0, fx1, fy1 = foot.bounds
    return Jig(f"{track.name}_op2_jaws", {"jaw_A": lower, "jaw_B": upper}, {"piece": track.solid},
               {"split_z": c, "size": (fx1 - fx0, fy1 - fy0, z1 - z0), "bolts": bolts,
                "bolt_length_max": bolt_len, "footprint": foot, "pocket": pocket, "track_solid": track.solid,
                "orientations": [orientation_matrix(n) for _, n in track.ends]})
