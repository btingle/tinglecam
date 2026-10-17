"""Reconstruct an ordered OBJ profile as a compact DXF of lines/arcs/ellipses."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

if __package__:
    from .track_curves import EllipseArc, LineSegment
    from .track_dxf import write_profile_dxf
    from .track_profiles import load_profile_obj
else:
    from track_curves import EllipseArc, LineSegment
    from track_dxf import write_profile_dxf
    from track_profiles import load_profile_obj


def _line(points, tolerance):
    start, end = points[0], points[-1]
    direction = end - start
    length_squared = direction @ direction
    if length_squared < 1e-20:
        return None
    parameters = np.clip((points - start) @ direction / length_squared, 0, 1)
    error = np.linalg.norm(start + parameters[:, None] * direction - points, axis=1).max()
    return (LineSegment(start, end), float(error)) if error <= tolerance else None


def _angles(points, center, axes):
    local = np.linalg.solve(axes, (points - center).T).T
    angles = np.unwrap(np.arctan2(local[:, 1], local[:, 0]))
    differences = np.diff(angles)
    if not (np.all(differences > 0) or np.all(differences < 0)):
        return None
    if abs(angles[-1] - angles[0]) > 2 * np.pi + 1e-8:
        return None
    return angles


def _circle(points, tolerance):
    # Constrain the circle to pass through both end vertices, keeping adjacent
    # reconstructed entities connected even with rounded OBJ coordinates.
    start, end = points[0], points[-1]
    chord = end - start
    length = np.linalg.norm(chord)
    if length < 1e-10:
        return None
    normal = np.array([-chord[1], chord[0]]) / length
    middle = (start + end) / 2
    q = points[1:-1] - start
    coefficients = 2 * (q @ normal)
    if coefficients @ coefficients < 1e-18:
        return None
    rhs = (q * q).sum(axis=1) - 2 * (q @ (middle - start))
    center = middle + normal * ((coefficients @ rhs) / (coefficients @ coefficients))
    radius = np.linalg.norm(start - center)
    error = np.max(np.abs(np.linalg.norm(points - center, axis=1) - radius))
    if error > tolerance:
        return None
    axes = np.eye(2) * radius
    angles = _angles(points, center, axes)
    if angles is None:
        return None
    return EllipseArc(center, axes[:, 0], axes[:, 1], angles[0], angles[-1] - angles[0], "ARC"), float(error)


def _ellipse(points, tolerance):
    # Fit a normalized conic, then recover center and principal radii. Reject
    # parabolas, hyperbolas, and unstable near-linear fits.
    origin = points.mean(axis=0)
    scale = np.linalg.norm(points - origin, axis=1).max()
    if scale < 1e-10:
        return None
    x, y = ((points - origin) / scale).T
    design = np.column_stack((x*x, x*y, y*y, x, y))
    coefficients, _, rank, _ = np.linalg.lstsq(design, np.ones(len(points)), rcond=None)
    if rank < 5:
        return None
    a, b, c, d, e = coefficients
    quadratic = np.array([[a, b / 2], [b / 2, c]])
    values, vectors = np.linalg.eigh(quadratic)
    if values.min() <= 0:
        return None
    center_local = -0.5 * np.linalg.solve(quadratic, [d, e])
    level = 1 + center_local @ quadratic @ center_local
    if level <= 0:
        return None
    axes = vectors * (scale * np.sqrt(level / values))
    center = origin + center_local * scale
    angles = _angles(points, center, axes)
    if angles is None:
        return None
    fitted_ends = center + (axes @ np.array([np.cos(angles[[0, -1]]), np.sin(angles[[0, -1]])])).T
    chord = fitted_ends[1] - fitted_ends[0]
    if chord @ chord < 1e-20:
        return None
    # An affine correction pins both ends without replacing the ellipse by a
    # polyline. SVD in the DXF writer restores its orthogonal axis representation.
    transform = np.eye(2) + np.outer(points[-1] - points[0] - chord, chord) / (chord @ chord)
    center = points[0] + transform @ (center - fitted_ends[0])
    axes = transform @ axes
    fitted = center + (axes @ np.array([np.cos(angles), np.sin(angles)])).T
    error = np.linalg.norm(fitted - points, axis=1).max()
    if error > tolerance:
        return None
    return EllipseArc(center, axes[:, 0], axes[:, 1], angles[0], angles[-1] - angles[0]), float(error)


def reconstruct_profile(points, tolerance=1e-5):
    """Minimize span count using contiguous line/circle/ellipse candidate runs.

    The tolerance measures source-vertex residuals in mm. Arcs interpolate the
    vertices' intended curve, so they naturally depart from the OBJ's chords
    between vertices. No symmetry or benchmark-specific dimensions are assumed.
    """
    points = np.asarray(points, dtype=float)
    if (not np.isfinite(tolerance) or tolerance <= 0 or points.ndim != 2
            or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all()):
        raise ValueError("Reconstruction needs finite 2D points and a positive fit tolerance")
    if np.array_equal(points[0], points[-1]):
        # Move a cyclic seam out of a curved run when a straight chord is present.
        vertices = points[:-1]
        seam = int(np.linalg.norm(np.roll(vertices, -1, axis=0) - vertices, axis=1).argmax())
        vertices = np.roll(vertices, -seam, axis=0)
        points = np.vstack((vertices, vertices[0]))
    count = len(points)
    costs = [float("inf")] * count
    choices = [None] * count
    costs[-1] = 0.
    for start in range(count - 2, -1, -1):
        for fitter, minimum, penalty in ((_line, 2, 0.), (_circle, 5, 0.0001), (_ellipse, 8, 0.0002)):
            for end in range(start + minimum - 1, count):
                fit = fitter(points[start:end + 1], tolerance)
                if fit is None:
                    break
                segment, error = fit
                cost = 1 + penalty + costs[end]
                if cost < costs[start]:
                    costs[start] = cost
                    choices[start] = (end, segment, error)
    segments, errors, index = [], [], 0
    while index < count - 1:
        if choices[index] is None:
            raise ValueError("Profile contains duplicate vertices or could not be reconstructed")
        index, segment, error = choices[index]
        segments.append(segment)
        errors.append(error)
    return segments, {"input_vertices": count - int(np.array_equal(points[0], points[-1])),
                      "segment_count": len(segments), "entity_counts": dict(Counter(s.kind for s in segments)),
                      "fit_tolerance_mm": tolerance, "max_vertex_residual_mm": max(errors),
                      "segments": [{"type": s.kind, "start": s.points([0.])[0].tolist(),
                                    "end": s.points([1.])[0].tolist(), "vertex_residual_mm": error,
                                    **({"radii_mm": [float(np.linalg.norm(s.axis_a)), float(np.linalg.norm(s.axis_b))]}
                                       if isinstance(s, EllipseArc) else {})}
                                   for s, error in zip(segments, errors)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fit-tolerance", type=float, default=1e-5, help="Source vertex residual in mm")
    parser.add_argument("--open-path", action="store_true")
    parser.add_argument("--report", type=Path, help="Write a JSON reconstruction report")
    args = parser.parse_args()
    points = load_profile_obj(args.input, args.open_path)
    segments, report = reconstruct_profile(points, args.fit_tolerance)
    write_profile_dxf(segments, args.output)
    report.update(source=str(args.input), source_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest())
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{args.output}: {report['input_vertices']} vertices -> {len(segments)} spans "
          f"{report['entity_counts']}; maximum vertex residual {report['max_vertex_residual_mm']:.6g} mm")


if __name__ == "__main__":
    main()
