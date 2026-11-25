"""Self-review: geometric assertions over everything the pipeline produced."""
import numpy as np
from shapely.geometry import Point

from . import occ
from .layout import apply, invert, MIRROR_X, MIRROR_Y


class Report:
    def __init__(self):
        self.rows = []

    def check(self, group, ok, what, detail=""):
        self.rows.append((group, bool(ok), what, detail))

    @property
    def failures(self):
        return [r for r in self.rows if not r[1]]

    def markdown(self):
        lines = ["| | group | check | detail |", "|---|---|---|---|"]
        lines += [f"| {'ok' if ok else '**FAIL**'} | {g} | {w} | {d} |" for g, ok, w, d in self.rows]
        return "\n".join(lines)


def _sym_set(points, m, tol=0.05):
    pts = [np.array(q.coords[0]) for q in points]
    return all(min(np.linalg.norm(np.array(apply(m, Point(q)).coords[0]) - r) for r in pts) < tol for q in pts)


def check_layout(rep, layout, tracks, p):
    g = f"{layout.species.name} glue-up"
    blanks = layout.blanks()
    webs = [a.distance(b) for i, a in enumerate(blanks) for b in blanks[i + 1:]]
    rep.check(g, min(webs) >= p.web - 0.01, "blank-to-blank web", f"min {min(webs):.2f} (want {p.web})")
    cut = layout.cutouts(p.guide_clearance)
    d = min(b.distance(cut) for b in layout.bolts) - p.glue_bolt_d / 2
    rep.check(g, d >= p.glue_bolt_wall - 0.01, "bolt hole to cutout wall", f"min {d:.2f}")
    x0, y0, x1, y1 = layout.plate
    edge = min(min(b.x - x0, x1 - b.x, b.y - y0, y1 - b.y) for b in layout.bolts) - p.glue_bolt_d / 2
    rep.check(g, edge >= p.glue_plate_margin - 0.01, "bolt hole to plate edge", f"min {edge:.2f}")
    if layout.kind == "ring":
        sym = _sym_set(layout.bolts, MIRROR_X) and _sym_set(layout.bolts, MIRROR_Y)
        rep.check(g, sym, "bolt pattern mirror-symmetric (X and Y)")
        centred = all(any(b.distance(Point(c)) < 0.01 for b in layout.bolts) for c in layout.centres)
        rep.check(g, centred, "bolt in the middle of every ring")
    base = tracks[layout.species.base]
    stack = base.sheets * p.sheet - base.sheets * p.acrylic
    rep.check(g, stack > 0, "wood stands proud of the guide stack", f"{stack:.2f} mm")
    rep.check(g, base.sheets * p.sheet >= base.height, "sheets cover track height",
              f"{base.sheets} x {p.sheet} >= {base.height:.2f}")
    for name in layout.species.members:
        t = tracks[name]
        rep.check(g, t.sheets == base.sheets, f"{name} needs the same sheet count", str(t.sheets))


def check_fixture_holes(rep, layout, tracks, p):
    """Every fixture hole must lie in material the finished track does not keep."""
    g = f"{layout.species.name} blanks"
    seen = set()
    for i, regs in layout.placements.items():
        slot = layout.slots[i]
        if (slot.track, slot.kind) in seen:
            continue
        seen.add((slot.track, slot.kind))
        t = tracks[slot.track]
        lo, hi = t.blank_z
        slot_holes = [apply(slot.m, h) for h in layout.holes]
        rep.check(g, len(regs) == 1, f"{slot.track} seats one way in its slot", f"{len(regs)} ways")
        for m in regs:
            holes = [apply(invert(m), h) for h in slot_holes]  # slot holes in this type's own frame
            worst = max(occ.volume(occ.common(occ.cylinder(h.x, h.y, hi - p.sheet, hi + 1, p.fixture_hole_d),
                                              t.solid)) for h in holes)
            rep.check(g, worst < 1e-3, f"{slot.track}: fixture holes vanish with the waste",
                      f"overlap {worst:.4f} mm^3")
            inner = t.outline.buffer(p.laser_allowance).buffer(-p.fixture_hole_d / 2 - 1)
            rep.check(g, all(inner.contains(h) for h in holes), f"{slot.track}: holes inside blank")


def check_op0(rep, jig, layout, p):
    g = jig.name
    plate = jig.parts["plate"]
    rep.check(g, occ.valid(plate) and len(occ.solids(plate)) == 1, "plate is one valid solid")
    n = len(layout.fixture_holes())
    rep.check(g, len(jig.info["holes"]) == n, "one countersunk hole per fixture hole", str(n))
    glue = sorted((round(h.x, 3), round(-h.y, 3)) for h in jig.info["holes"])
    ref = sorted((round(h.x, 3), round(h.y, 3)) for h in layout.fixture_holes())
    rep.check(g, glue == ref, "holes match the glue-up layout (mirrored to the mill frame)")
    for k, b in jig.context.items():
        if occ.volume(occ.common(b, plate)) > 1e-3:
            rep.check(g, False, f"{k} overlaps the plate")


def _moved(shape, axis, d):
    v = np.asarray(axis, float) * d
    return occ.transform(shape, [[1, 0, 0, v[0]], [0, 1, 0, v[1]], [0, 0, 1, v[2]]])


def _jaw_checks(rep, jig, p, clearance, axis):
    """Jaws close along the vise axis. OCC booleans are unreliable for ~0.02 mm overlaps, so every
    probe overlaps by >= 0.1; the pocket is an exact profile offset, so its clearance is uniform."""
    g = jig.name
    jaws = [jig.parts.get("jaw_A"), jig.parts.get("jaw_B")]
    ok = all(j is not None for j in jaws) and len(jig.parts) == 2 and all(occ.valid(j) for j in jaws)
    rep.check(g, ok, "split into exactly two valid jaws", f"{len(jig.parts)} bodies")
    if not ok:
        return
    rep.check(g, p.split_gap >= 2 * clearance + 0.05, "split gap > both pocket clearances (piece gripped first)",
              f"{p.split_gap} vs 2 x {clearance}")
    close = (p.split_gap - 0.1) / 2   # stop 0.1 short of the nominal gap
    touch = occ.volume(occ.common(_moved(jaws[0], axis, close), _moved(jaws[1], axis, -close)))
    rep.check(g, touch < 1e-3, f"jaws close {2 * close:.2f} without meeting", f"overlap {touch:.4f}")
    # jaw = block - pocket, so the open jaws clear the piece exactly when the pocket contains it
    spill = occ.volume(occ.cut(jig.info["track_solid"], jig.info["pocket"]))
    rep.check(g, spill < 1e-2, "pocket contains the piece (open jaws never touch it)", f"{spill:.4f} mm^3 outside")
    probe = clearance + 0.1
    k, piece = next(iter(jig.context.items()))   # every pocket in a jaw set is the same copy
    for name, j, sign in zip("AB", jaws, (1, -1)):
        grip = occ.volume(occ.common(piece, _moved(j, axis, sign * probe)))
        rep.check(g, grip > 1e-3, f"jaw {name} bears on {k} (closing {probe:.2f})", f"{grip:.2f} mm^3")


def check_op1(rep, jig, track, p):
    _jaw_checks(rep, jig, p, p.op1_pocket_clearance, (0, 1, 0))
    xs = np.array(jig.info["split"].coords)[:, 0]
    rep.check(jig.name, np.all(np.diff(xs) > 0), "split is x-monotone (jaws open along the vise axis)")
    w = jig.info["size"][0]
    rep.check(jig.name, w <= p.vise_jaw_width + 1e-6, "fits the vise jaw", f"{w:.1f} <= {p.vise_jaw_width}")
    outl = jig.info["outlines"]
    rep.check(jig.name, all(o.bounds[0] >= -w / 2 and o.bounds[2] <= w / 2 for o in outl),
              "every pocket inside the jaw width", f"{jig.info['count']} pieces")


def check_op2(rep, jig, track, p):
    _jaw_checks(rep, jig, p, p.op2_pocket_clearance, (0, 0, 1))
    rep.check(jig.name, len(jig.info["bolts"]) >= 2, "bolts holding the jaws together",
              f"{len(jig.info['bolts'])} x M3, bolt <= {jig.info['bolt_length_max']:.1f} long")
    foot = jig.info["footprint"]
    for i, (pt, n) in enumerate(track.ends):
        proud = -max((np.array(q) - pt) @ n for q in foot.exterior.coords)
        rep.check(jig.name, proud >= p.op2_face_proud - 0.01, f"end face {i} stands proud of the jaws",
                  f"{proud:.2f}")
