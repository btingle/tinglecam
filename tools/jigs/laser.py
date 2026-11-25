"""2D laser outputs: wood blank sheets and the acrylic glue-up stack (DXF, mm)."""
from pathlib import Path

import ezdxf
import numpy as np

from .layout import apply


class Dxf:
    def __init__(self):
        self.doc = ezdxf.new(units=4)
        self.doc.header["$MEASUREMENT"] = 1
        self.msp = self.doc.modelspace()

    def polygon(self, geom, layer="CUT"):
        for poly in getattr(geom, "geoms", [geom]):
            for ring in [poly.exterior, *poly.interiors]:
                self.msp.add_lwpolyline(list(ring.coords)[:-1], close=True, dxfattribs={"layer": layer})

    def circle(self, pt, d, layer="CUT"):
        self.msp.add_circle((pt.x, pt.y), d / 2, dxfattribs={"layer": layer})

    def hexagon(self, pt, across_flats, layer="CUT"):
        r = across_flats / np.sqrt(3)
        a = np.radians(np.arange(6) * 60 + 30)
        self.msp.add_lwpolyline(list(zip(pt.x + r * np.cos(a), pt.y + r * np.sin(a))), close=True,
                                dxfattribs={"layer": layer})

    def rounded_rect(self, rect, r, layer="CUT"):
        x0, y0, x1, y1 = rect
        b = np.tan(np.pi / 8)  # bulge of a 90 degree CCW arc, stored on the arc's start vertex
        pts = [(x1 - r, y0, b), (x1, y0 + r, 0), (x1, y1 - r, b), (x1 - r, y1, 0),
               (x0 + r, y1, b), (x0, y1 - r, 0), (x0, y0 + r, b), (x0 + r, y0, 0)]
        self.msp.add_lwpolyline([(x, y, 0, 0, bl) for x, y, bl in pts], format="xyseb", close=True,
                                dxfattribs={"layer": layer})

    def save(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.doc.saveas(path)
        return Path(path)


def blank_files(layout, tracks, p, out):
    """One outline per slot kind; the top (track-top) sheet carries the fixture holes.

    Sheets are drawn as seen from above in the glue frame (holes side up)."""
    files = []
    sheets = tracks[layout.species.base].sheets
    for kind in ("A", "B"):
        slot = next((s for s in layout.slots if s.kind == kind), None)
        if slot is None:
            continue
        outline = apply(slot.m, layout.blank)
        cx, cy = outline.centroid.coords[0]
        local = lambda g: apply((1, 0, 0, 1, -cx, -cy), apply(slot.m, g))
        for top in (True, False):
            d = Dxf()
            d.polygon(local(layout.blank))
            if top:
                for h in layout.holes:
                    d.circle(local(h), p.fixture_hole_d)
            name = f"{layout.species.name}_blank_{kind}_{'top' if top else 'plain'}.dxf"
            files.append(d.save(out / name))
    from collections import Counter
    counts = Counter(f"{s.track} ({s.kind})" for s in layout.slots)
    note = (f"{layout.species.name}: {sheets} sheets per blank (1 top with holes + {sheets - 1} plain). "
            f"Per glue-up: " + ", ".join(f"{n} x {k}" for k, n in counts.items()))
    return files, note


def glueup_files(layout, tracks, p, out):
    sheets = tracks[layout.species.base].sheets
    cut = layout.cutouts(p.guide_clearance)
    name = layout.species.name
    files = []

    def plate(holes=True, nuts=False, cutouts=False):
        d = Dxf()
        d.rounded_rect(layout.plate, p.glue_plate_corner_r)
        if cutouts:
            d.polygon(cut)
        for b in layout.bolts:
            if nuts:
                d.hexagon(b, p.glue_nut_af + p.glue_nut_clearance)
            elif holes:
                d.circle(b, p.glue_bolt_d)
        return d

    files.append(plate(nuts=True).save(out / f"{name}_glue_0_bottom_plate.dxf"))
    for i in range(sheets):
        files.append(plate(cutouts=True).save(out / f"{name}_glue_{i + 1}_guide.dxf"))
    files.append(plate().save(out / f"{name}_glue_{sheets + 1}_top_plate.dxf"))
    return files
