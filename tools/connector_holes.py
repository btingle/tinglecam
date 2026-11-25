"""Connector holes in the track end faces: the deepest symmetric pair for each hole size.

Two holes at (+x, z) and (-x, z) on the profile, drilled square to each end face. Mating faces are mirror
images, so a symmetric pair always lines up; mirrored (R, sR, RL) and reversed (Ls, Rs) pieces then need no
separate check, and flat tracks are easier than their elevated counterparts. A spot's depth is the least,
over both ends of sL, L, LR and sSs, of how far a cylinder of radius r + wall stays inside the piece, less
`wall` at the bottom. Rays along the hole axis from points round that cylinder are cast against the STEP
solids with Open CASCADE (no Onshape calls). Positions are relative to the profile's bounding box centre.

    python3 tools/connector_holes.py --diameters 2 3 4 5 6 --wall 1
    python3 tools/connector_holes.py --diameters 3 --fs-out tools/onshape/connector_holes.fs
"""
import argparse
from pathlib import Path

import numpy as np

if __package__:
    from .generate_tracks import create_track_templates
    from .track_curves import as_curve_profile
    from .track_profiles import load_profile
    from .track_step import build_track_shape
else:
    from generate_tracks import create_track_templates
    from track_curves import as_curve_profile
    from track_profiles import load_profile
    from track_step import build_track_shape

CHECKED = ("sL", "L", "LR", "sSs")
RAYS = 24
EPS = 0.01


class PieceEnd:
    """One end face of one piece: maps profile points onto the face and casts rays into the piece."""

    def __init__(self, name, template, intersector, t, centre):
        self.name = f"{name} {'start' if t == 0 else 'end'}"
        self.template = template
        self.intersector = intersector
        self.t = t
        template.set_sample_pt(np.array(centre, dtype=np.float64))
        a = template.sample_path_t(t)
        b = template.sample_path_t(1e-4 if t == 0 else 1 - 1e-4)
        d = (b - a) * [1, 1, 0]
        self.inward = d / np.linalg.norm(d)

    def face_point(self, px, pz):
        self.template.set_sample_pt(np.array([px, pz], dtype=np.float64))
        return self.template.sample_path_t(self.t)

    def exit_depth(self, px, pz):
        """How far the line square to the face through (px, pz) stays inside; 0 if (px, pz) is off the face."""
        from OCP.gp import gp_Dir, gp_Lin, gp_Pnt

        start = self.face_point(px, pz) - EPS * self.inward
        self.intersector.Perform(gp_Lin(gp_Pnt(*start), gp_Dir(*self.inward)), 0, 1e4)
        hits = sorted(self.intersector.WParameter(i) for i in range(1, self.intersector.NbPnt() + 1))
        if len(hits) < 2 or abs(hits[0] - EPS) > 5e-3:
            return 0.0
        return hits[1] - EPS


def piece_ends(profile, grid_size, elevation):
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

    names, templates = create_track_templates(grid_size, elevation)
    lo, hi = as_curve_profile(profile).bounds()
    centre = (lo + hi) / 2
    ends = []
    for name in CHECKED:
        template = templates[names.index(name)]
        shape, _, _ = build_track_shape(template, profile, cap_ends=True)
        intersector = IntCurvesFace_ShapeIntersector()
        intersector.Load(shape, 1e-6)
        ends += [PieceEnd(name, template, intersector, t, centre) for t in (0.0, 1.0)]
    return ends, centre


def clearance_2d(polygon, point):
    """Signed distance from point to the closed polygon's boundary (positive inside)."""
    from matplotlib.path import Path as MplPath

    a, b = polygon, np.roll(polygon, -1, axis=0)
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", point - a, ab) / np.einsum("ij,ij->i", ab, ab), 0, 1)
    dist = np.min(np.linalg.norm(a + ab * t[:, None] - point, axis=1))
    return dist if MplPath(polygon).contains_point(point) else -dist


def fits_face(polygon, centre, x, z, radius, wall):
    """Both holes keep `wall` from the face outline and from each other."""
    return x >= radius + wall / 2 and clearance_2d(polygon, centre + [x, z]) >= radius + wall


def spot_depth(ends, centre, x, z, radius, wall, best=-np.inf):
    """Depth of the hole pair at (+-x, z) (relative to the profile centre), and the limiting end.

    Stops early once the depth drops to `best` or below (it can only fall further)."""
    rim = radius + wall
    angles = np.linspace(0, 2 * np.pi, RAYS, endpoint=False)
    offsets = [(0.0, 0.0)] + [(rim * np.cos(a), rim * np.sin(a)) for a in angles]
    depth, limit = np.inf, None
    for end in ends:
        for sx in (1, -1):
            for dx, dz in offsets:
                d = end.exit_depth(centre[0] + sx * x + dx, centre[1] + z + dz) - wall
                if d < depth:
                    depth, limit = d, end.name
                    if depth <= best:
                        return depth, limit
    return depth, limit


def best_spot(ends, centre, polygon, radius, wall, step=0.5, refine=0.1):
    """Grid search over the half face (x >= 0), then a finer grid round the best spot."""
    lo, hi = polygon.min(axis=0) - centre, polygon.max(axis=0) - centre

    def fits(x, z):
        return fits_face(polygon, centre, x, z, radius, wall)

    def search(xs, zs, best):
        grid = {}
        for x in xs:
            for z in zs:
                if not fits(x, z):
                    continue
                d, limit = spot_depth(ends, centre, x, z, radius, wall, best[0])
                grid[(x, z)] = d
                if d > best[0]:
                    best = (d, x, z, limit)
        return best, grid

    xs = np.arange(0, hi[0] + 1e-9, step)
    zs = np.arange(lo[1], hi[1] + 1e-9, step)
    best, grid = search(xs, zs, (0.0, None, None, None))
    if best[1] is None:
        return best, grid
    fx = np.arange(max(0, best[1] - step), best[1] + step + 1e-9, refine)
    fz = np.arange(best[2] - step, best[2] + step + 1e-9, refine)
    best, _ = search(fx, fz, best)
    return best, grid


def depth_map(ends, centre, polygon, radius, wall, step):
    """Full (unpruned) depth over the half face, for the plot."""
    lo, hi = polygon.min(axis=0) - centre, polygon.max(axis=0) - centre
    xs = np.arange(0, hi[0] + 1e-9, step)
    zs = np.arange(lo[1], hi[1] + 1e-9, step)
    grid = np.full((len(zs), len(xs)), np.nan)
    for j, z in enumerate(zs):
        for i, x in enumerate(xs):
            if fits_face(polygon, centre, x, z, radius, wall):
                grid[j, i] = max(spot_depth(ends, centre, x, z, radius, wall)[0], 0)
    return xs, zs, grid


def plot(results, maps, polygon, centre, wall, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(results), figsize=(4.2 * len(results), 3.2), squeeze=False)
    outline = np.vstack([polygon, polygon[:1]]) - centre
    for ax, (diameter, best), (xs, zs, grid) in zip(axes[0], results, maps):
        step = xs[1] - xs[0] if len(xs) > 1 else 1
        full = np.hstack([grid[:, :0:-1], grid])
        fx = np.r_[-xs[:0:-1], xs]
        im = ax.imshow(full, origin="lower", cmap="viridis", vmin=0,
                       extent=[fx[0] - step / 2, fx[-1] + step / 2, zs[0] - step / 2, zs[-1] + step / 2])
        ax.plot(outline[:, 0], outline[:, 1], "k-", lw=0.8)
        d, x, z, _ = best
        if x is not None:
            for sx in (1, -1):
                ax.add_patch(plt.Circle((sx * x, z), diameter / 2, fill=False, color="r"))
                ax.add_patch(plt.Circle((sx * x, z), diameter / 2 + wall, fill=False, color="r", ls=":"))
        ax.set_title(f"Ø{diameter:g}: {d:.1f} mm deep" if x is not None else f"Ø{diameter:g}: no fit")
        ax.set_aspect("equal")
        fig.colorbar(im, ax=ax, label="depth (mm)")
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)
    return path


def write_fs(path, diameter, wall, best, args):
    d, x, z, limit = best
    path = Path(path)
    path.write_text(
        f"// Connector holes, from tools/connector_holes.py (diameter {diameter:g}, wall {wall:g}, grid {args.grid_size},\n"
        f"// elevation {args.elevation_gain}, limited by {limit}). A symmetric pair square to each end face, centres\n"
        f"// relative to the profile's bounding box centre (mm). Regenerate rather than edit.\n"
        f"const CONNECTOR_D = {diameter:g};\n"
        f"const CONNECTOR_DEPTH = {np.floor(d * 10) / 10:g};\n"
        f"const CONNECTOR_X = {x:.2f};\n"
        f"const CONNECTOR_Z = {z:.2f};\n")
    return path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--diameters", type=float, nargs="+", default=[2, 3, 4, 5, 6], help="Hole diameters (mm)")
    parser.add_argument("--wall", type=float, default=1.0, help="Least material round and below a hole (mm)")
    parser.add_argument("--grid-size", type=int, default=40)
    parser.add_argument("--elevation-gain", type=int, default=6)
    parser.add_argument("--profile-path", default=str(Path(__file__).parents[1] / "models/2D/trackprofile.dxf"))
    parser.add_argument("--plot", default="output/connector_holes.png", help="Depth map image ('' to skip)")
    parser.add_argument("--fs-out", help="Write the FeatureScript constants for the (single) diameter here")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.fs_out and len(args.diameters) != 1:
        raise SystemExit("--fs-out needs exactly one diameter")
    profile = load_profile(args.profile_path)
    polygon = as_curve_profile(profile).sample(0.1)
    polygon = polygon[np.r_[True, np.linalg.norm(np.diff(polygon, axis=0), axis=1) > 1e-9]]
    if np.allclose(polygon[0], polygon[-1]):
        polygon = polygon[:-1]
    ends, centre = piece_ends(profile, args.grid_size, args.elevation_gain)
    results, maps = [], []
    print(f"wall {args.wall:g} mm; positions relative to the profile centre; checked {', '.join(CHECKED)}")
    print(f"{'diameter':>8} {'depth':>6} {'x (+-)':>7} {'z':>6}  limited by")
    for diameter in args.diameters:
        best, _ = best_spot(ends, centre, polygon, diameter / 2, args.wall)
        d, x, z, limit = best
        if x is None:
            print(f"{diameter:8g}   no spot leaves {args.wall:g} mm of wall")
        else:
            print(f"{diameter:8g} {d:6.2f} {x:7.2f} {z:6.2f}  {limit}")
        results.append((diameter, best))
        if args.plot:
            maps.append(depth_map(ends, centre, polygon, diameter / 2, args.wall, 0.25))
    if args.plot:
        print("depth maps:", plot(results, maps, polygon, centre, args.wall, args.plot))
    if args.fs_out and results[0][1][1] is not None:
        print("constants:", write_fs(args.fs_out, args.diameters[0], args.wall, results[0][1], args))


if __name__ == "__main__":
    main()
