# reduced roster of tracks from full set
# going to have:
# s,l,r,L,R,sSs(1),sL,sR,Ls,Rs
# granted- still 10 jigs- but less than it could be
# next template would be for LR/RL- requires thick stock, more work

# convention (coordinates)
# Y
# ^
# |
# |
# |
# +--------> X

import argparse
from math import cos, pi, sin, sqrt
import numpy as np
from numba import njit, float64, int64, bool, types
from numba.experimental import jitclass

# math helpers
# simple cubic bezier curve generator for transitioning between slopes
@njit
def sample_bezier(start, start_slope, end, end_slope, t, quadratic=False):

    pl = end[0] - start[0]
    m1 = np.array(start_slope)
    m2 = np.array(end_slope)

    if quadratic:
        # convert from normalized vector into xy slope
        m1 = m1[1] / m1[0] # the ole rise over run
        m2 = m2[1] / m2[0]
        p0 = np.array(start)
        p2 = np.array(end)
        x1 = m1 * p0[0] - m2 * p2[0] + p2[1] - p0[1]
        y1 = m1 * (x1 - p0[0]) + p0[1]
        p1 = np.array([x1, y1])
        return (
            (1 - t)**2 * p0 + 
            2 * (1 - t) * t * p1 + 
            t**2 * p2
        )

    p0 = np.array(start)
    p3 = np.array(end)
    p1 = p0 + (pl * m1) / 3
    p2 = p3 - (pl * m2) / 3

    return (
        (1 - t) ** 3 * p0
        + (3 * (1 - t) ** 2 * t) * p1
        + 3 * (1 - t) * t**2 * p2
        + t**3 * p3
    )


# s, sSs
@jitclass([
    ("scurve", bool),
    ("gs", int64),
    ("elev", int64),
    ("pt", float64[::1]),
    ("pl", float64)
])
class StraightTrackTemplate:

    def __init__(self, scurve, grid_size=40, elevation_gain=6):
        self.gs = grid_size
        self.elev = elevation_gain
        self.scurve = scurve

    def get_base_path_length(self, pt):
        return self.gs

    def set_sample_pt(self, pt):
        self.pl = self.get_base_path_length(pt)
        self.pt = pt

    # samples the path traced by profile vertex pt without any elevation modulation
    def sample_base_path_t(self, t):
        return np.array([self.pt[0], t * self.gs - (self.gs/2), self.pt[1]])

    def sample_path_t(self, t):
        #base_pt = self.sample_base_path_t(t)
        if self.scurve:
            xy = sample_bezier((0, 0), (1, 0), (self.pl, self.elev), (1, 0), t)
            basepath_t = xy[0] / self.pl
            base_pt = self.sample_base_path_t(basepath_t)
            base_pt[2] += xy[1]
            return base_pt
        else:
            return self.sample_base_path_t(t)
            

# lr,L,R
@jitclass([
    ("left", bool),
    ("elevated", bool),
    ("gs", int64),
    ("gs2", int64),
    ("elev", int64),
    ("pl", float64),
    ("pt", float64[::1]),
    ("slope", types.UniTuple(float64, 2))
])
class CurveTrackTemplate:

    def __init__(self, left, elevated, grid_size=40, elevation_gain=6):
        self.left = left
        self.elevated = elevated
        self.gs = grid_size
        self.elev = elevation_gain
        self.gs2 = self.gs / 2

    def get_base_path_length(self, pt):

        # profiles guaranteed to be centered on bbox
        radius = self.gs2 - pt[0] if self.left else self.gs2 + pt[0]
        return radius * pi/2

    def get_slope(self, pt, elevated=False):
        if not (self.elevated or elevated):
            return (0, 0)
        pl = self.get_base_path_length(pt)
        mag = sqrt(pl**2 + self.elev**2)
        return (pl/mag, self.elev/mag)

    def set_sample_pt(self, pt):
        self.pl = self.get_base_path_length(pt)
        self.slope = self.get_slope(pt)
        self.pt = pt

    def sample_base_path_t(self, t):

        z = self.pt[1]
        theta = t * pi/2
        if self.left:
            radius = self.gs2 - self.pt[0]
            return np.array([self.gs2 - cos(theta) * radius, -self.gs2 + sin(theta) * radius, z], dtype=np.float64)
        else:
            radius = self.gs2 + self.pt[0]
            return np.array([-self.gs2 + cos(theta) * radius, -self.gs2 + sin(theta) * radius, z], dtype=np.float64)

    def sample_path_t(self, t):
        base_pt = self.sample_base_path_t(t)
        if self.elevated:
            base_pt[2] += t*self.elev
        return base_pt

# sL, sR, Rs, Ls
@jitclass([
    ("left", bool),
    ("rising", bool),
    ("gs", int64),
    ("elev", int64),
    ("curve_segment", CurveTrackTemplate.class_type.instance_type),
    ("straight_segment", StraightTrackTemplate.class_type.instance_type),
    ("pl", float64),
    ("pt", float64[::1]),
    ("slope_end", types.UniTuple(float64, 2))
])
class StraightCurveTransitionTrackTemplate:

    # "rising" indicates whether the straight track rises to the curve or the opposite
    def __init__(self, left, rising, grid_size=40, elevation_gain=6): 
        self.gs = grid_size
        self.elev = elevation_gain
        self.rising = rising
        self.curve_segment = CurveTrackTemplate(not (left ^ rising), False, grid_size, elevation_gain)
        self.straight_segment = StraightTrackTemplate(False, grid_size, elevation_gain)

    def get_base_path_length(self, pt):

        return self.gs + self.curve_segment.get_base_path_length(pt)

    def set_sample_pt(self, pt):

        self.pt = pt
        self.pl = self.get_base_path_length(pt)
        self.slope_end = self.curve_segment.get_slope(pt, elevated=True)
        self.curve_segment.set_sample_pt(pt)
        self.straight_segment.set_sample_pt(pt)
        if not self.rising: # clamp the slope for certain cases with very small inner curve radius (like our jig extrusions)
            self.slope_end = (self.slope_end[0], -self.slope_end[1])
        if abs(self.slope_end[1]) > 0.5:
            self.slope_end = (sqrt(3)/2, -0.5 if not self.rising else 0.5)

    def sample_base_path_t(self, t):

        t_trans = self.gs / self.pl
        if t <= t_trans:
            return self.straight_segment.sample_path_t(t/t_trans) - np.array([0, self.gs/2, 0], dtype=np.float64)
        else:
            return self.curve_segment.sample_path_t((t-t_trans)/(1-t_trans)) + np.array([0, self.gs/2, 0], dtype=np.float64)

    def sample_path_t(self, t):

        #base_pt = self.sample_base_path_t(t)
        if self.rising:
            xy = sample_bezier([0, 0], [1, 0], [self.pl, self.elev], self.slope_end, t)
        else:
            xy = sample_bezier([0, self.elev], [1, 0], [self.pl, 0], self.slope_end, t) 
        t_basepath = xy[0] / self.pl
        base_pt = self.sample_base_path_t(t_basepath)
        base_pt[2] += xy[1]

        return base_pt

def create_track_templates(grid_size, elevation_gain):
    s = StraightTrackTemplate(False, grid_size=grid_size, elevation_gain=elevation_gain)
    sSs = StraightTrackTemplate(True, grid_size=grid_size, elevation_gain=elevation_gain)
    lr = CurveTrackTemplate(False, False, grid_size=grid_size, elevation_gain=elevation_gain)
    L = CurveTrackTemplate(True, True, grid_size=grid_size, elevation_gain=elevation_gain)
    R = CurveTrackTemplate(False, True, grid_size=grid_size, elevation_gain=elevation_gain)
    sL = StraightCurveTransitionTrackTemplate(True, True, grid_size=grid_size, elevation_gain=elevation_gain)
    sR = StraightCurveTransitionTrackTemplate(False, True, grid_size=grid_size, elevation_gain=elevation_gain)
    Ls = StraightCurveTransitionTrackTemplate(True, False, grid_size=grid_size, elevation_gain=elevation_gain)
    Rs = StraightCurveTransitionTrackTemplate(False, False, grid_size=grid_size, elevation_gain=elevation_gain)
    slr = StraightCurveTransitionTrackTemplate(False, False, grid_size=grid_size, elevation_gain=0)
    srl = StraightCurveTransitionTrackTemplate(True, False, grid_size=grid_size, elevation_gain=0)

    all_template_names = ["s", "sSs", "lr", "L", "R", "sL", "sR", "Ls", "Rs", "slr", "srl"]
    all_templates = [s, sSs, lr, L, R, sL, sR, Ls, Rs, slr, srl]
    return all_template_names, all_templates


@njit
def compute_contours(result, template, resolution, profile, elevation_gain, clamp_bottom, clamp_z=None):
    min_z = min(profile[:, 1])
    max_z = max(profile[:, 1])
    if not clamp_z:
        clamp_z = min_z

    for j in range(profile.shape[0]):
        template.set_sample_pt(profile[j])
        _p0 = template.sample_path_t(0)
        _p1 = template.sample_path_t(1)
        _min_z = min(_p0[2], _p1[2])
        _max_z = max(_p0[2], _p1[2])
        if abs(profile[j][1] - min_z) < 0.01 and clamp_bottom:
            _clampmin = True
        else:
            _clampmin = False
        for k in range(resolution):
            sample = template.sample_path_t(k / (resolution - 1))
            if _clampmin:
                sample[2] = clamp_z
            else:
                sample[2] = max(min(sample[2], _max_z), _min_z)
            result[j, k] = sample


def create_all_tracks(grid_size, elevation_gain, profile, resolution=256, clamp_bottom=False, clamp_z=None):
    if resolution < 2:
        raise ValueError("Contour resolution must be at least 2")
    names, templates = create_track_templates(grid_size, elevation_gain)
    results = np.empty((len(templates), profile.shape[0], resolution, 3))
    for i, (name, template) in enumerate(zip(names, templates)):
        print(i, name)
        compute_contours(results[i], template, resolution, profile, elevation_gain, clamp_bottom, clamp_z)
    return names, results


def main(grid_size, elevation_gain, profile_obj_path, save_dir, resolution=256,
         gcode=False, open_path=False, translate=(0, 0, 0), step=False,
         step_tolerance=0.01, step_max_samples=129, cap_ends=False,
         profile_layer=None, profile_join_tolerance=1e-5, clamp_bottom=False, clamp_z=None):
    if __package__:
        from .track_profiles import default_profile, load_profile, resample_profile
        from .track_curves import as_curve_profile
    else:
        from track_profiles import default_profile, load_profile, resample_profile
        from track_curves import as_curve_profile

    if grid_size <= 0:
        raise ValueError("Grid size must be positive")
    if gcode and step:
        raise ValueError("Choose either G-code or STEP output")
    if cap_ends and gcode:
        raise ValueError("End caps apply to STEP and OBJ output, not G-code")
    profile = (load_profile(profile_obj_path, open_path, profile_layer, profile_join_tolerance) if profile_obj_path
               else default_profile(open_path))
    if cap_ends and not as_curve_profile(profile).closed:
        raise ValueError("End caps require a closed profile")
    if step:
        if __package__:
            from .track_step import export_step_tracks
        else:
            from track_step import export_step_tracks
        names, templates = create_track_templates(grid_size, elevation_gain)
        export_step_tracks(names, templates, profile, save_dir,
                           step_tolerance, step_max_samples, cap_ends)
    else:
        if __package__:
            from .track_exports import export_sampled_tracks
        else:
            from track_exports import export_sampled_tracks
        names, results = create_all_tracks(grid_size, elevation_gain,
                                           resample_profile(profile), resolution, clamp_bottom, clamp_z)
        export_sampled_tracks(names, results, save_dir, gcode, translate, cap_ends)


def trans_type(value):
    try:
        coordinates = [float(part) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Translation must be comma-separated numbers") from exc
    if not 1 <= len(coordinates) <= 3 or not np.isfinite(coordinates).all():
        raise argparse.ArgumentTypeError("Translation needs 1 to 3 finite coordinates")
    return coordinates + [0.0] * (3 - len(coordinates))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Generate marble tracks as OBJ, G-code, or STEP surfaces.")
    parser.add_argument("--grid-size", type=int, default=40, help="Grid size in mm (default: 40)")
    parser.add_argument("--elevation-gain", type=int, default=6, help="Elevation gain in mm (default: 6)")
    parser.add_argument("--profile-path", "--profile-obj-path", dest="profile_obj_path", type=str,
                        help="OBJ or DXF profile: X/Y become width/height in mm")
    parser.add_argument("--profile-layer", help="Read geometry from this DXF layer only")
    parser.add_argument("--profile-join-tolerance", type=float, default=1e-5,
                        help="DXF endpoint joining tolerance in mm (default: 0.00001)")
    parser.add_argument("--save-dir", type=str, default=".")
    parser.add_argument("--resolution", type=int, default=256, help="Longitudinal samples for OBJ/G-code only")
    formats = parser.add_mutually_exclusive_group()
    formats.add_argument("--gcode", action="store_true", help="Export G-code and OBJ toolpath previews")
    formats.add_argument("--step", action="store_true", help="Export sewn B-spline surface bodies in STEP format")
    parser.add_argument("--step-tolerance", type=float, default=0.01,
                        help="Maximum sampled surface deviation in mm (default: 0.01)")
    parser.add_argument("--step-max-samples", type=int, default=129,
                        help="Maximum adaptive samples per patch axis (default: 129)")
    parser.add_argument("--open-path", action="store_true", help="Do not close the input profile")
    parser.add_argument("--cap-ends", "--cap", action="store_true",
                        help="Cap both ends of a closed profile (STEP solid or capped OBJ)")
    parser.add_argument("--translate", type=trans_type, default=[0, 0, 0],
                        help="G-code/preview translation X,Y,Z in mm")
    parser.add_argument("--clamp-bottom", action="store_true", help="clamp the bottommost points of the profile to the minimum z of the swept track (not available for STEP mode)")
    parser.add_argument("--clamp-z", type=float, default=None, help="clamp to specific Z value")
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    try:
        main(**vars(args))
    except (ValueError, ImportError, OSError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc
