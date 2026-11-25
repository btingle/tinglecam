"""PNG previews: 2D laser layers and shaded 3D views of the solids."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from . import occ
from .layout import apply

ACRYLIC, WOOD, CUT, HOLE = "#cfe3f0", "#e8c9a0", "#2b6c99", "#c0392b"
PALETTE = ["#8fb8d8", "#e0a96d", "#9ccf9c", "#d39bd3"]


def _poly(ax, geom, **kw):
    for poly in getattr(geom, "geoms", [geom]):
        for ring in [poly.exterior, *poly.interiors]:
            ax.plot(*ring.xy, **kw)


def glueup(ax, layout, p):
    x0, y0, x1, y1 = layout.plate
    r = p.glue_plate_corner_r
    ax.add_patch(matplotlib.patches.FancyBboxPatch((x0 + r, y0 + r), x1 - x0 - 2 * r, y1 - y0 - 2 * r,
                                                   boxstyle=f"round,pad={r}", fc=ACRYLIC, ec=CUT))
    cut = layout.cutouts(p.guide_clearance)
    for poly in getattr(cut, "geoms", [cut]):
        ax.fill(*poly.exterior.xy, color="white", zorder=1)
        _poly(ax, poly, color=CUT, lw=1, zorder=2)
    for b in layout.blanks():
        _poly(ax, b, color="#a0662b", lw=0.6, ls="--", zorder=2)
    for s in layout.slots:
        c = apply(s.m, layout.blank).representative_point()
        ax.text(c.x, c.y, s.track, ha="center", va="center", fontsize=7, zorder=3)
    for h in layout.fixture_holes():
        ax.add_patch(plt.Circle((h.x, h.y), p.fixture_hole_d / 2, color=HOLE, zorder=3))
    for b in layout.bolts:
        ax.add_patch(plt.Circle((b.x, b.y), p.glue_bolt_d / 2, fill=False, color="k", zorder=3))
    ax.set_aspect("equal")
    ax.set_title(f"{layout.species.name} glue-up guide  ({x1 - x0:.0f} x {y1 - y0:.0f})", fontsize=9)
    ax.grid(alpha=0.2)


def _tris(shape, deflection):
    t = occ.triangles(shape, deflection)
    return t if len(t) else np.zeros((0, 3, 3))


def solids3d(ax, shapes, title, view=(30, -60), deflection=0.25, alpha=1.0):
    """shapes: list of (shape, colour)."""
    light = np.array([0.4, -0.5, 0.75])
    light /= np.linalg.norm(light)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for shape, colour in shapes:
        t = _tris(shape, deflection)
        if not len(t):
            continue
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(colour))
        ax.add_collection3d(Poly3DCollection(t, facecolors=np.clip(base * shade[:, None], 0, 1),
                                             edgecolors="none", alpha=alpha))
        lo, hi = np.minimum(lo, t.reshape(-1, 3).min(0)), np.maximum(hi, t.reshape(-1, 3).max(0))
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(*view)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")


def explode(shape, dz):
    return occ.transform(shape, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, dz]])
