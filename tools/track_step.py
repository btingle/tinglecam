"""Fit the track equations to sewn B-spline surfaces and export millimetre STEP.

This module receives templates from generate_tracks, so it never duplicates the
track equations. SciPy and Open CASCADE (cadquery-ocp) are optional dependencies
loaded only when STEP output is requested. No mesh conversion is involved.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numba import njit

if __package__:
    from .track_curves import as_curve_profile
else:
    from track_curves import as_curve_profile


@dataclass
class SurfacePatch:
    profile_edge: int
    branch: int
    surface: object
    sampled_error: float


@dataclass
class StepResult:
    path: Path
    patch_count: int
    sampled_error: float
    samples_per_axis: tuple[int, int]
    capped: bool = False


def _load_dependencies():
    try:
        from scipy.interpolate import BSpline, make_interp_spline
        from OCP.Geom import Geom_BSplineSurface
    except ImportError as exc:
        raise ImportError(
            "STEP export requires SciPy and cadquery-ocp. Install with: "
            "python3 -m pip install -r tools/requirements-tracks.txt"
        ) from exc
    return BSpline, make_interp_spline, Geom_BSplineSurface


@njit
def sample_profile_points(template, profile_points, vs, branch, min_z, max_z):
    """Map evaluated profile points through the original track equations."""
    result = np.empty((len(profile_points), len(vs), 3))
    for i in range(len(profile_points)):
        pt = profile_points[i]
        template.set_sample_pt(pt)
        for j in range(len(vs)):
            t = vs[j]
            if branch != -1:
                join = template.gs / template.get_base_path_length(pt)
                if branch == 0:
                    t = t * join
                else:
                    t = join + t * (1 - join)
            sample = template.sample_path_t(t)
            sample[2] = max(min(sample[2], max_z), min_z)
            result[i, j] = sample
    return result


@njit
def sample_patch(template, start, end, us, vs, branch, min_z, max_z):
    """Compatibility sampler for a straight profile span.

    Branch -1 covers a whole track; 0 and 1 cover the straight and curved
    sections of a transition, with the join depending on each profile point.
    """
    points = np.empty((len(us), 2))
    for i in range(len(us)):
        points[i] = start * (1 - us[i]) + end * us[i]
    return sample_profile_points(template, points, vs, branch, min_z, max_z)


def _occ_knots(knots):
    from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal

    values, counts = np.unique(knots, return_counts=True)
    occ_values = TColStd_Array1OfReal(1, len(values))
    occ_counts = TColStd_Array1OfInteger(1, len(values))
    for i, (value, count) in enumerate(zip(values, counts), 1):
        occ_values.SetValue(i, float(value))
        occ_counts.SetValue(i, int(count))
    return occ_values, occ_counts


def _make_surface(poles, uknots, vknots, surface_type):
    from OCP.TColgp import TColgp_Array2OfPnt
    from OCP.gp import gp_Pnt

    points = TColgp_Array2OfPnt(1, poles.shape[0], 1, poles.shape[1])
    for i, j in np.ndindex(poles.shape[:2]):
        points.SetValue(i + 1, j + 1, gp_Pnt(*poles[i, j]))
    uk, um = _occ_knots(uknots)
    vk, vm = _occ_knots(vknots)
    return surface_type(points, uk, vk, um, vm, 3, 3)


def _check_parameters(template, profile, tolerance, max_samples):
    if not np.isfinite(tolerance) or tolerance < 1e-6:
        raise ValueError("STEP tolerance must be finite and at least 0.000001 mm")
    if max_samples < 5:
        raise ValueError("STEP max samples must be at least 5")
    if template.gs <= 0:
        raise ValueError("Grid size must be positive")
    curve = getattr(template, "curve_segment", template)
    if hasattr(curve, "left"):
        for point in profile.bounds():
            if curve.get_base_path_length(point) <= 1e-6:
                raise ValueError("Profile reaches or crosses the bend axis; increase grid size")


def fit_track_surfaces(template, profile, tolerance=0.01, max_samples=129):
    """Return patches and grid size after checking off-grid equation samples.

    Neighboring patches share knots along their common edges. Profile knots are
    shared by the branches of each profile segment; longitudinal knots are shared
    across segments. Intervals failing the deviation check are bisected. This also
    resolves the creases caused by the generator's elevation clamps. The error
    is a sampled Euclidean deviation, not a mathematical bound over the continuous
    domain. Failure to meet the requested sampled tolerance raises an error.
    """
    profile = as_curve_profile(profile)
    _check_parameters(template, profile, tolerance, max_samples)
    BSpline, interpolate, surface_type = _load_dependencies()
    branches = (0, 1) if hasattr(template, "curve_segment") else (-1,)
    lower, upper = profile.bounds()
    min_z = float(lower[1] - template.elev)
    max_z = float(upper[1] + template.elev)
    u_nodes = [np.linspace(0, 1, 5) for _ in profile.segments]
    vs = np.linspace(0, 1, 5)
    while True:
        # Include boundaries and three independent interior checks per cell.
        def check_nodes(nodes):
            return np.unique(np.r_[nodes, (nodes[:-1, None] + np.diff(nodes)[:, None]
                                          * [0.2113248654, 0.5, 0.7886751346]).ravel()])

        check_v = check_nodes(vs)
        interval_v = np.clip(np.searchsorted(vs, check_v, side="right") - 1, 0, len(vs) - 2)
        edge_errors = []
        errors_v = np.zeros(len(vs) - 1)
        patches = []
        worst_error = 0.0
        for edge, segment in enumerate(profile.segments):
            us = u_nodes[edge]
            check_u = check_nodes(us)
            interval_u = np.clip(np.searchsorted(us, check_u, side="right") - 1, 0, len(us) - 2)
            errors_u = np.zeros(len(us) - 1)
            for branch in branches:
                points = sample_profile_points(template, segment.points(us), vs, branch, min_z, max_z)
                along_u = interpolate(us, points, k=3, axis=0)
                along_v = interpolate(vs, along_u.c, k=3, axis=1)
                # SciPy moves the interpolated coefficient axis to the front.
                poles = np.moveaxis(along_v.c, 0, 1)
                basis_u = BSpline.design_matrix(check_u, along_u.t, 3)
                basis_v = BSpline.design_matrix(check_v, along_v.t, 3)
                intermediate = (basis_u @ poles.reshape(len(us), -1)).reshape(len(check_u), len(vs), 3)
                fitted = (basis_v @ intermediate.transpose(1, 0, 2).reshape(len(vs), -1))
                fitted = fitted.reshape(len(check_v), len(check_u), 3).transpose(1, 0, 2)
                reference = sample_profile_points(template, segment.points(check_u), check_v, branch, min_z, max_z)
                errors = np.linalg.norm(fitted - reference, axis=2)
                error = float(errors.max())
                if not np.isfinite(error):
                    raise ValueError("Track equations produced non-finite coordinates")
                worst_error = max(worst_error, error)
                np.maximum.at(errors_u, interval_u, errors.max(axis=1))
                np.maximum.at(errors_v, interval_v, errors.max(axis=0))
                patches.append((edge, branch, poles, along_u.t, along_v.t, error))
            edge_errors.append(errors_u)
        if worst_error <= tolerance:
            return [SurfacePatch(edge, branch, _make_surface(poles, uk, vk, surface_type), error)
                    for edge, branch, poles, uk, vk, error in patches], (max(map(len, u_nodes)), len(vs))

        def refine(nodes, errors):
            failing = np.flatnonzero(errors > tolerance)
            # When near the limit, give the largest errors priority.
            failing = failing[np.argsort(errors[failing])[::-1]][:max_samples - len(nodes)]
            return np.sort(np.r_[nodes, (nodes[failing] + nodes[failing + 1]) / 2])

        next_u = [refine(nodes, errors) for nodes, errors in zip(u_nodes, edge_errors)]
        next_v = refine(vs, errors_v)
        if all(len(a) == len(b) for a, b in zip(next_u, u_nodes)) and len(next_v) == len(vs):
            raise ValueError(
                f"STEP fit deviation {worst_error:.6g} mm exceeds {tolerance:.6g} mm "
                f"at up to {max(map(len, u_nodes))} x {len(vs)} samples; "
                "increase --step-max-samples or relax --step-tolerance"
            )
        u_nodes, vs = next_u, next_v


def _end_cap(patches, branch, end_parameter):
    """Build a planar cap from the exact fitted surface boundary curves."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire

    wire = BRepBuilderAPI_MakeWire()
    for patch in patches:
        if patch.branch == branch:
            edge = BRepBuilderAPI_MakeEdge(patch.surface.VIso(end_parameter))
            if not edge.IsDone():
                raise ValueError("Could not construct a cap boundary edge")
            wire.Add(edge.Edge())
    if not wire.IsDone():
        raise ValueError("Cap boundary does not form a connected wire")
    cap = BRepBuilderAPI_MakeFace(wire.Wire(), True)
    if not cap.IsDone():
        raise ValueError("Track end could not be capped with a planar face")
    return cap.Face()


def export_step(template, profile, path, tolerance=0.01, max_samples=129, cap_ends=False):
    """Write a sewn surface shell, or a capped solid, with explicit mm units."""
    profile = as_curve_profile(profile)
    if cap_ends and not profile.closed:
        raise ValueError("End caps require a closed profile")
    patches, samples = fit_track_surfaces(template, profile, tolerance, max_samples)
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer

    # Shared spline boundaries already coincide numerically. Sewing tolerance
    # stays much smaller than the approximation tolerance and does not hide gaps.
    sewing = BRepBuilderAPI_Sewing(min(1e-6, tolerance * 0.01))
    for patch in patches:
        face = BRepBuilderAPI_MakeFace(patch.surface, 1e-7)
        if not face.IsDone():
            raise ValueError(f"Could not construct STEP face for profile edge {patch.profile_edge}")
        sewing.Add(face.Face())
    if cap_ends:
        first_branch, last_branch = (0, 1) if hasattr(template, "curve_segment") else (-1, -1)
        sewing.Add(_end_cap(patches, first_branch, 0.))
        sewing.Add(_end_cap(patches, last_branch, 1.))
    sewing.Perform()
    shape = sewing.SewedShape()
    if shape.IsNull() or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("STEP surface shell failed Open CASCADE validity checks")
    shells = TopExp_Explorer(shape, TopAbs_SHELL)
    count = 0
    while shells.More():
        count += 1
        shells.Next()
    # A single profile segment without a transition is one face, not a shell.
    if len(patches) > 1 and count != 1:
        raise ValueError("STEP patches did not sew into one connected surface shell")

    if cap_ends:
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
        from OCP.BRepLib import BRepLib
        from OCP.TopoDS import TopoDS

        if sewing.NbFreeEdges() != 0 or sewing.NbMultipleEdges() != 0 or count != 1:
            raise ValueError("Capped STEP shell is not watertight")
        shell = TopoDS.Shell_s(TopExp_Explorer(shape, TopAbs_SHELL).Current())
        solid = BRepBuilderAPI_MakeSolid(shell).Solid()
        if not BRepLib.OrientClosedSolid_s(solid) or not BRepCheck_Analyzer(solid).IsValid():
            raise ValueError("Capped STEP model failed solid validity checks")
        shape = solid

    writer = STEPControl_Writer()
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    writer.SetTolerance(1e-7)
    if writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone:
        raise RuntimeError("Open CASCADE could not translate the surfaces to STEP")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise OSError(f"Could not write STEP file: {path}")
    return StepResult(path, len(patches), max(p.sampled_error for p in patches), samples, cap_ends)


def export_step_tracks(names, templates, profile, save_dir, tolerance=0.01, max_samples=129, cap_ends=False):
    for name, template in zip(names, templates):
        result = export_step(template, profile, Path(save_dir) / f"{name}.step", tolerance, max_samples, cap_ends)
        print(f"saved {result.path}: {result.patch_count} B-spline patches, "
              f"sampled deviation {result.sampled_error:.6g} mm "
              f"({result.samples_per_axis[0]} x {result.samples_per_axis[1]} samples)", flush=True)
