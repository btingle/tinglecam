"""Load ordered (width, height) profiles and sample them for mesh/toolpath output."""

from pathlib import Path

import numpy as np


def prepare_profile(points, open_path=False):
    """Remove consecutive duplicates and close a profile exactly once, if requested."""
    profile = np.asarray(points, dtype=np.float64)
    if profile.ndim != 2 or profile.shape[1] != 2 or not np.isfinite(profile).all():
        raise ValueError("Profile must contain finite (width, height) points")
    if len(profile):
        profile = profile[np.r_[True, np.any(np.diff(profile, axis=0) != 0, axis=1)]]
    if len(profile) < 2:
        raise ValueError("Profile needs at least two distinct points")
    if not open_path:
        if not np.array_equal(profile[-1], profile[0]):
            profile = np.vstack((profile, profile[0]))
        if len(profile) < 4:
            raise ValueError("A closed profile needs at least three distinct points")
    return np.ascontiguousarray(profile)


def load_profile_obj(path, open_path=False):
    """Read OBJ vertices in file order; OBJ X/Y become profile width/height.

    Faces and line indices are intentionally ignored, matching the generator's
    original profile convention. Normal and texture records are not vertices.
    """
    points = []
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.split("#", 1)[0].split()
        if fields and fields[0] == "v":
            if len(fields) < 3:
                raise ValueError(f"{path}:{number}: vertex needs X and Y coordinates")
            try:
                points.append([float(fields[1]), float(fields[2])])
            except ValueError as exc:
                raise ValueError(f"{path}:{number}: invalid vertex coordinates") from exc
    return prepare_profile(points, open_path)


def default_profile(open_path=False):
    return prepare_profile([[-12, -6], [-12, 6], [12, 6], [12, -6]], open_path)


def resample_profile(profile, spacing=0.25):
    """Sample by arc length, retaining original corners and both endpoints."""
    if hasattr(profile, "segments"):
        return profile.sample(spacing)
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("Profile spacing must be positive and finite")
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(profile, axis=0), axis=1))]
    samples = np.unique(np.r_[np.arange(0.0, distances[-1], spacing), distances])
    return np.ascontiguousarray(np.column_stack([
        np.interp(samples, distances, profile[:, axis]) for axis in range(2)
    ]))


def load_profile(path, open_path=False, layer=None, join_tolerance=1e-5):
    """Auto-detect an ordered OBJ or a continuous planar DXF profile."""
    suffix = Path(path).suffix.lower()
    if suffix == ".obj":
        if layer is not None:
            raise ValueError("--profile-layer applies to DXF files only")
        return load_profile_obj(path, open_path)
    if suffix == ".dxf":
        if __package__:
            from .track_dxf import load_profile_dxf
        else:
            from track_dxf import load_profile_dxf
        return load_profile_dxf(path, open_path, layer, join_tolerance)
    raise ValueError("Profile file must be .obj or .dxf")
