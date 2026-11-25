"""One track type as the jigs see it: plan outline, centreline, end faces, height envelope, solids.

Everything is derived from the generate_tracks templates, so a new template gets jigs for free.
Track frame: X/Y plan, Z up, profile Y = world Z; the track's top (channel side) faces +Z.
"""
from functools import cached_property

import numpy as np
from shapely.geometry import LineString, MultiPoint, Point, Polygon, box
from shapely.ops import unary_union

from ..generate_tracks import compute_contours, create_track_templates
from ..track_profiles import load_profile
from ..track_step import build_track_shape


class Track:
    def __init__(self, name, params, samples=129):
        names, templates = create_track_templates(int(params.grid), int(params.elevation))
        if name not in names:
            raise ValueError(f"Unknown track type {name!r}; templates: {names}")
        self.name, self.p, self.samples = name, params, samples
        self.template = dict(zip(names, templates))[name]
        self.profile = load_profile(params.profile)
        pts = self.profile.sample(0.1)
        self.profile_poly = Polygon(pts).buffer(0)
        self.profile_pts = pts

    # --- sampled sweep -------------------------------------------------------------------------------
    @cached_property
    def sweep(self):
        """(profile points, t samples, xyz) of the profile's convex hull, enough for outlines/heights."""
        hull = np.array(self.profile_poly.convex_hull.exterior.coords)[:-1]
        pts = np.ascontiguousarray(np.vstack([hull, self.profile_pts[::8]]))
        out = np.empty((len(pts), self.samples, 3))
        compute_contours(out, self.template, self.samples, pts, self.p.elevation, False)
        return out

    @cached_property
    def outline(self):
        """Plan outline of the finished track (no allowance)."""
        s = self.sweep
        parts = [MultiPoint(s[:, k:k + 2, :2].reshape(-1, 2)).convex_hull for k in range(self.samples - 1)]
        return unary_union(parts).simplify(0.005)

    @cached_property
    def z_range(self):
        return float(self.sweep[..., 2].min()), float(self.sweep[..., 2].max())

    @cached_property
    def envelope(self):
        """(max bottom z, min top z) over the path: any z between them cuts every section."""
        z = self.sweep[..., 2]
        return float(z.min(axis=0).max()), float(z.max(axis=0).min())

    @property
    def height(self):
        lo, hi = self.z_range
        return hi - lo

    @property
    def sheets(self):
        return int(np.ceil(self.height / self.p.sheet - 1e-6))

    @cached_property
    def blank_z(self):
        """(bottom, top) of the glued blank, track centred in it."""
        lo, hi = self.z_range
        spare = self.sheets * self.p.sheet - self.height
        return lo - spare / 2, hi + spare / 2

    def centre_point(self, t, z=None):
        self.template.set_sample_pt(np.array([0.0, self.profile_poly.bounds[3] if z is None else z]))
        return self.template.sample_path_t(float(t))

    @cached_property
    def centreline(self):
        return LineString([self.centre_point(t)[:2] for t in np.linspace(0, 1, self.samples)])

    @cached_property
    def ends(self):
        """[(point, outward plan normal)] for the start and end faces (faces are vertical)."""
        c = np.array(self.centreline.coords)
        d0, d1 = c[0] - c[1], c[-1] - c[-2]
        return [(c[0], d0 / np.linalg.norm(d0)), (c[-1], d1 / np.linalg.norm(d1))]

    @cached_property
    def headings(self):
        d = np.diff(np.array(self.centreline.coords), axis=0)
        return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))

    @property
    def turn(self):
        return float(np.degrees(self.headings[-1] - self.headings[0]))

    @cached_property
    def fixture_holes(self):
        """Plan positions of the blank's fixture holes (top sheet)."""
        return [Point(self.centre_point(t)[:2]) for t in self.p.fixture_hole_t]

    # --- profiles (profile frame: x width, y height) ---------------------------------------------------
    def filled_profile(self, extend_up=0.0):
        """Profile with everything above its lower boundary filled (channel closed), raised by extend_up."""
        poly, pts = self.profile_poly, self.profile_pts
        tall = poly.bounds[3] - poly.bounds[1] + extend_up + 1
        quads = [Polygon([a, b, (b[0], b[1] + tall), (a[0], a[1] + tall)])
                 for a, b in zip(pts[:-1], pts[1:]) if abs(a[0] - b[0]) > 1e-9]
        filled = unary_union([poly, *quads]).buffer(0)
        x0, y0, x1, y1 = poly.bounds
        return filled.intersection(box(x0 - 1, y0 - 1, x1 + 1, y1 + extend_up))

    # --- solids -------------------------------------------------------------------------------------------
    @cached_property
    def solid(self):
        shape, _, _ = build_track_shape(self.template, self.profile, self.p.track_tolerance, 257, True)
        return shape

    def sweep_solid(self, polygon):
        """Sweep any simple polygon (profile frame) through this track's equations, capped."""
        coords = np.array(polygon.simplify(self.p.pocket_simplify).exterior.coords)
        shape, _, _ = build_track_shape(self.template, coords, self.p.pocket_tolerance, 129, True)
        return shape
