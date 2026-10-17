"""Import a single planar DXF contour without flattening its curves."""

from pathlib import Path

import numpy as np

if __package__:
    from .track_curves import CurveProfile, EllipseArc, LineSegment, SplineSegment
else:
    from track_curves import CurveProfile, EllipseArc, LineSegment, SplineSegment


def _ezdxf():
    try:
        import ezdxf
    except ImportError as exc:
        raise ImportError("DXF profiles require ezdxf: python3 -m pip install -r tools/requirements-tracks.txt") from exc
    return ezdxf


def _entity_segments(entity, scale):
    from ezdxf.edgesmith import is_pure_2d_entity

    kind = entity.dxftype()
    if kind in {"TEXT", "MTEXT", "DIMENSION", "POINT"}:
        return []
    if kind == "INSERT":
        instances = entity.multi_insert() if entity.mcount > 1 else [entity]
        return [span for instance in instances for child in instance.virtual_entities()
                for span in _entity_segments(child, scale)]
    if kind not in {"LINE", "ARC", "CIRCLE", "ELLIPSE", "SPLINE", "LWPOLYLINE", "POLYLINE"}:
        raise ValueError(f"Unsupported DXF profile entity {kind}; export a planar contour or select --profile-layer")
    if not is_pure_2d_entity(entity):
        raise ValueError(f"DXF {kind} must lie in the XY plane with +Z extrusion")
    if kind in {"LWPOLYLINE", "POLYLINE"}:
        return [span for child in entity.virtual_entities() for span in _entity_segments(child, scale)]
    if kind == "LINE":
        return [LineSegment(np.array(tuple(entity.dxf.start)[:2]) * scale,
                            np.array(tuple(entity.dxf.end)[:2]) * scale)]
    if kind in {"ARC", "CIRCLE"}:
        radius = entity.dxf.radius * scale
        start = np.deg2rad(entity.dxf.start_angle) if kind == "ARC" else 0.
        sweep = np.deg2rad((entity.dxf.end_angle - entity.dxf.start_angle) % 360) if kind == "ARC" else 2 * np.pi
        if sweep == 0 or radius <= 0:
            raise ValueError("DXF contains a zero-length circular arc")
        return [EllipseArc(np.array(tuple(entity.dxf.center)[:2]) * scale,
                           np.array([radius, 0.]), np.array([0., radius]), start, sweep, "ARC")]
    if kind == "ELLIPSE":
        ellipse = entity.construction_tool()
        return [EllipseArc(np.array(tuple(ellipse.center)[:2]) * scale,
                           np.array(tuple(ellipse.major_axis)[:2]) * scale,
                           np.array(tuple(ellipse.minor_axis)[:2]) * scale,
                           ellipse.start_param, ellipse.param_span)]
    spline = entity.construction_tool()
    weights = list(spline.weights())
    controls = np.array([tuple(p)[:2] for p in spline.control_points]) * scale
    knots = np.array(list(spline.knots()))
    low, high = knots[spline.degree], knots[-spline.degree - 1]
    values, counts = np.unique(knots, return_counts=True)
    interior = (values > low) & (values < high)
    if np.any(counts[interior] > spline.degree):
        raise ValueError("DXF spline has a discontinuity; export connected curve spans")
    # A degree-fold interior knot can be a sharp corner. Keep it as a face
    # boundary instead of fitting a smooth interpolant across it.
    splits = np.r_[low, values[interior & (counts >= spline.degree)], high]
    return [SplineSegment(controls, knots, spline.degree, weights or None, (a, b))
            for a, b in zip(splits[:-1], splits[1:])]


def _order_segments(segments, tolerance):
    """Order unordered/reversed entities; reject branches and multiple contours."""
    if not segments:
        raise ValueError("No supported profile geometry found in DXF")
    nodes, adjacency, endpoints = [], [], []
    for index, segment in enumerate(segments):
        pair = []
        for point in segment.points([0., 1.]):
            matches = [i for i, node in enumerate(nodes) if np.linalg.norm(point - node) <= tolerance]
            if len(matches) > 1:
                raise ValueError("DXF endpoints are ambiguous at the requested join tolerance")
            if matches:
                node = matches[0]
            else:
                node = len(nodes)
                nodes.append(point)
                adjacency.append([])
            adjacency[node].append(index)
            pair.append(node)
        endpoints.append(pair)
    if any(len(edges) > 2 for edges in adjacency):
        raise ValueError("DXF profile branches; select a layer containing one contour")
    tips = [i for i, edges in enumerate(adjacency) if len(edges) == 1]
    if len(tips) not in (0, 2):
        raise ValueError("DXF contains disconnected contours")
    current = min(tips or range(len(nodes)), key=lambda i: tuple(nodes[i]))
    remaining = set(range(len(segments)))
    ordered = []
    while remaining:
        candidates = [i for i in adjacency[current] if i in remaining]
        if not candidates:
            raise ValueError("DXF contains multiple contours; select --profile-layer")
        index = candidates[0]
        a, b = endpoints[index]
        ordered.append(segments[index] if a == current else segments[index].reversed())
        current = b if a == current else a
        remaining.remove(index)
    return ordered


def load_profile_dxf(path, open_path=False, layer=None, join_tolerance=1e-5):
    ezdxf = _ezdxf()
    from ezdxf import units

    if not np.isfinite(join_tolerance) or join_tolerance <= 0:
        raise ValueError("Profile join tolerance must be positive and finite")
    try:
        document = ezdxf.readfile(path)
    except ezdxf.DXFError as exc:
        raise ValueError(f"Invalid DXF profile: {path}") from exc
    unit = document.units
    # Unitless CAD drawings are conventionally interpreted as millimetres here.
    scale = units.conversion_factor(unit, units.MM) if unit else 1.
    segments = [segment for entity in document.modelspace()
                if layer is None or entity.dxf.layer == layer
                for segment in _entity_segments(entity, scale)]
    ordered = _order_segments(segments, join_tolerance)
    if not open_path:
        start, end = ordered[0].points([0.])[0], ordered[-1].points([1.])[0]
        if np.linalg.norm(start - end) > join_tolerance:
            ordered.append(LineSegment(end, start))
    return CurveProfile(ordered, join_tolerance)


def write_profile_dxf(segments, path):
    """Write reconstructed line/circle/ellipse spans as standard DXF entities."""
    ezdxf = _ezdxf()
    document = ezdxf.new("R2010")
    document.units = ezdxf.units.MM
    modelspace = document.modelspace()
    for segment in segments:
        attributes = {"layer": "PROFILE"}
        if isinstance(segment, LineSegment):
            modelspace.add_line(tuple(segment.start), tuple(segment.end), dxfattribs=attributes)
        elif isinstance(segment, EllipseArc):
            # SVD converts even slightly non-orthogonal fitted ellipse axes to
            # DXF's orthogonal major-axis/ratio representation without flattening.
            axes = np.column_stack((segment.axis_a, segment.axis_b))
            directions, lengths, rotation = np.linalg.svd(axes)
            if np.linalg.det(directions) < 0:
                directions[:, 1] *= -1
                rotation[1] *= -1
            start_vector = rotation @ [np.cos(segment.start_angle), np.sin(segment.start_angle)]
            start = np.arctan2(start_vector[1], start_vector[0])
            sweep = segment.sweep * np.linalg.det(rotation)
            end = start + sweep
            if sweep < 0:
                start, end = end, start
            if segment.kind == "ARC":
                if abs(sweep) >= 2 * np.pi - 1e-9:
                    modelspace.add_circle(tuple(segment.center), float(lengths.mean()), dxfattribs=attributes)
                else:
                    modelspace.add_arc(tuple(segment.center), float(lengths.mean()),
                                       float(np.rad2deg(start) % 360), float(np.rad2deg(end) % 360),
                                       dxfattribs=attributes)
            else:
                if abs(sweep) >= 2 * np.pi - 1e-9:
                    start, end = 0., 2 * np.pi
                else:
                    start, end = start % (2 * np.pi), end % (2 * np.pi)
                modelspace.add_ellipse(tuple(segment.center), tuple(directions[:, 0] * lengths[0]),
                                       float(lengths[1] / lengths[0]), float(start), float(end),
                                       dxfattribs=attributes)
        else:
            raise ValueError("DXF reconstruction writer accepts lines, arcs, and ellipses")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document.saveas(path)
