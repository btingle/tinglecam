"""Continuous 2D profile spans; sampling never changes their surface topology."""

from dataclasses import dataclass

import numpy as np


class Segment:
    def reversed(self):
        return ReversedSegment(self)


@dataclass
class LineSegment(Segment):
    start: np.ndarray
    end: np.ndarray
    kind = "LINE"

    def points(self, parameters):
        u = np.asarray(parameters)[:, None]
        return np.asarray(self.start) * (1 - u) + np.asarray(self.end) * u

    def bounds(self):
        points = self.points([0., 1.])
        return points.min(axis=0), points.max(axis=0)


@dataclass
class EllipseArc(Segment):
    center: np.ndarray
    axis_a: np.ndarray
    axis_b: np.ndarray
    start_angle: float
    sweep: float
    kind: str = "ELLIPSE"

    def points(self, parameters):
        angles = self.start_angle + np.asarray(parameters) * self.sweep
        return (self.center + np.cos(angles)[:, None] * self.axis_a
                + np.sin(angles)[:, None] * self.axis_b)

    def bounds(self):
        parameters = [0., 1.]
        for a, b in zip(self.axis_a, self.axis_b):
            angle = np.arctan2(b, a)
            for turn in range(-4, 5):
                u = (angle + turn * np.pi - self.start_angle) / self.sweep
                if 0 < u < 1:
                    parameters.append(u)
        points = self.points(parameters)
        return points.min(axis=0), points.max(axis=0)


class SplineSegment(Segment):
    kind = "SPLINE"

    def __init__(self, control_points, knots, degree, weights=None, parameter_range=None):
        try:
            from scipy.interpolate import BSpline
        except ImportError as exc:
            raise ImportError("DXF splines require SciPy: python3 -m pip install -r tools/requirements-tracks.txt") from exc

        points = np.asarray(control_points, dtype=float)
        weights = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
                or len(weights) != len(points) or not np.isfinite(weights).all()
                or np.any(weights <= 0)):
            raise ValueError("Profile splines need finite control points and positive weights")
        self.spline = BSpline(knots, np.column_stack((points * weights[:, None], weights)), degree)
        self.low, self.high = self.spline.t[degree], self.spline.t[-degree - 1]
        if parameter_range is not None:
            self.low, self.high = parameter_range
        if self.high <= self.low:
            raise ValueError("Profile spline has an empty parameter interval")

    def points(self, parameters):
        values = self.spline(self.low + np.asarray(parameters) * (self.high - self.low))
        return values[:, :2] / values[:, 2, None]

    def bounds(self):
        # Rational extrema solve N' W - N W' = 0 within each polynomial span.
        from scipy.interpolate import PPoly
        from numpy.polynomial import polynomial as poly

        pieces = [PPoly.from_spline((self.spline.t, self.spline.c[:, i], self.spline.k))
                  for i in range(3)]
        parameters = [self.low, self.high]
        for span, (a, b) in enumerate(zip(pieces[0].x[:-1], pieces[0].x[1:])):
            if b <= a or a < self.low or b > self.high:
                continue
            parameters.extend((a, b))
            weight = pieces[2].c[:, span][::-1]
            for axis in range(2):
                numerator = pieces[axis].c[:, span][::-1]
                derivative = poly.polysub(poly.polymul(poly.polyder(numerator), weight),
                                          poly.polymul(numerator, poly.polyder(weight)))
                for root in poly.polyroots(derivative):
                    if abs(root.imag) < 1e-9 and 0 < root.real < b - a:
                        parameters.append(a + root.real)
        points = self.points((np.asarray(parameters) - self.low) / (self.high - self.low))
        return points.min(axis=0), points.max(axis=0)


@dataclass
class ReversedSegment(Segment):
    original: Segment

    @property
    def kind(self):
        return self.original.kind

    def points(self, parameters):
        return self.original.points(1 - np.asarray(parameters))

    def bounds(self):
        return self.original.bounds()


@dataclass
class JoinedSegment(Segment):
    """Close sub-tolerance DXF endpoint gaps with a tiny linear correction."""
    original: Segment
    start: np.ndarray
    end: np.ndarray

    @property
    def kind(self):
        return self.original.kind

    def points(self, parameters):
        u = np.asarray(parameters, dtype=float)
        original_ends = self.original.points([0., 1.])
        result = (self.original.points(u) + (1 - u[:, None]) * (self.start - original_ends[0])
                  + u[:, None] * (self.end - original_ends[1]))
        result[u == 0] = self.start
        result[u == 1] = self.end
        return result

    def bounds(self):
        low, high = self.original.bounds()
        offsets = np.array([self.start, self.end]) - self.original.points([0., 1.])
        return low + np.minimum(offsets.min(axis=0), 0), high + np.maximum(offsets.max(axis=0), 0)


class CurveProfile:
    def __init__(self, segments, join_tolerance=1e-5):
        if not segments or not np.isfinite(join_tolerance) or join_tolerance <= 0:
            raise ValueError("A curve profile needs spans and a positive join tolerance")
        ends = [s.points([0., 1.]) for s in segments]
        if not np.isfinite(ends).all():
            raise ValueError("Profile contains non-finite coordinates")
        for i in range(len(segments) - 1):
            if np.linalg.norm(ends[i][1] - ends[i + 1][0]) > join_tolerance:
                raise ValueError("Profile spans do not form a connected chain")
            joint = (ends[i][1] + ends[i + 1][0]) / 2
            ends[i][1] = ends[i + 1][0] = joint
        self.closed = np.linalg.norm(ends[-1][1] - ends[0][0]) <= join_tolerance
        if self.closed:
            ends[-1][1] = ends[0][0] = (ends[-1][1] + ends[0][0]) / 2
        self.segments = [JoinedSegment(s, a, b) for s, (a, b) in zip(segments, ends)]

    def bounds(self):
        bounds = [s.bounds() for s in self.segments]
        return np.min([b[0] for b in bounds], axis=0), np.max([b[1] for b in bounds], axis=0)

    def sample(self, spacing=0.25):
        if not np.isfinite(spacing) or spacing <= 0:
            raise ValueError("Profile spacing must be positive and finite")
        result = []
        for segment in self.segments:
            nodes = np.linspace(0, 1, 17)
            for _ in range(16):
                points = segment.points(nodes)
                midpoints = (nodes[:-1] + nodes[1:]) / 2
                errors = np.linalg.norm(segment.points(midpoints) - (points[:-1] + points[1:]) / 2, axis=1)
                failing = errors > min(0.001, spacing / 100)
                if not failing.any():
                    break
                nodes = np.sort(np.r_[nodes, midpoints[failing]])
            else:
                raise ValueError("Profile curve sampling did not converge")
            distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            if distances[-1] <= 1e-12:
                raise ValueError("Profile contains a zero-length span")
            parameters = np.interp(np.r_[np.arange(0, distances[-1], spacing), distances[-1]], distances, nodes)
            sampled = segment.points(parameters)
            result.extend(sampled if not result else sampled[1:])
        return np.ascontiguousarray(result)


def as_curve_profile(profile):
    if isinstance(profile, CurveProfile):
        return profile
    points = np.ascontiguousarray(profile, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2
            or not np.isfinite(points).all()
            or np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) == 0)):
        raise ValueError("Profile needs finite points without zero-length edges")
    return CurveProfile([LineSegment(a, b) for a, b in zip(points[:-1], points[1:])])
