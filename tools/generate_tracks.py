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

import os, sys, argparse
from math import *
from abc import ABC, abstractmethod
import numpy as np
from numba import njit, vectorize, float64, int64, bool, types
from numba.experimental import jitclass

# math helpers
# simple cubic bezier curve generator for transitioning between slopes
@njit
def sample_bezier(start, start_slope, end, end_slope, t):

    pl = end[0] - start[0]
    b0 = start[1]
    b3 = end[1]
    b1 = b0 + (pl * start_slope[1]) / 3
    b2 = b3 - (pl * end_slope[1]) / 3

    return (
        (1 - t) ** 3 * b0
        + (3 * (1 - t) ** 2 * t) * b1
        + 3 * (1 - t) * t**2 * b2
        + t**3 * b3
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
        base_pt = self.sample_base_path_t(t)
        if self.scurve:
            base_pt[2] += sample_bezier((0, 0), (1, 0), (self.pl, self.elev), (1, 0), t)
        return base_pt
            

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
        if not self.rising:
            self.slope_end = (self.slope_end[0], -self.slope_end[1])

    def sample_base_path_t(self, t):

        t_trans = self.gs / self.pl
        if t <= t_trans:
            return self.straight_segment.sample_path_t(t/t_trans) - np.array([0, self.gs/2, 0], dtype=np.float64)
        else:
            return self.curve_segment.sample_path_t((t-t_trans)/(1-t_trans)) + np.array([0, self.gs/2, 0], dtype=np.float64)

    def sample_path_t(self, t):

        base_pt = self.sample_base_path_t(t)
        if self.rising:
            base_pt[2] += sample_bezier((0, 0), (1, 0), (self.pl, self.elev), self.slope_end, t) 
        else:
            base_pt[2] += sample_bezier((0, self.elev), (1, 0), (self.pl, 0), self.slope_end, t) 

        return base_pt

def create_all_tracks(grid_size, elevation_gain, profile, resolution=256):
    s = StraightTrackTemplate(False, grid_size=grid_size, elevation_gain=elevation_gain)
    sSs = StraightTrackTemplate(True, grid_size=grid_size, elevation_gain=elevation_gain)
    lr = CurveTrackTemplate(False, False, grid_size=grid_size, elevation_gain=elevation_gain)
    L = CurveTrackTemplate(True, False, grid_size=grid_size, elevation_gain=elevation_gain)
    R = CurveTrackTemplate(False, True, grid_size=grid_size, elevation_gain=elevation_gain)
    sL = StraightCurveTransitionTrackTemplate(True, True, grid_size=grid_size, elevation_gain=elevation_gain)
    sR = StraightCurveTransitionTrackTemplate(False, True, grid_size=grid_size, elevation_gain=elevation_gain)
    Ls = StraightCurveTransitionTrackTemplate(True, False, grid_size=grid_size, elevation_gain=elevation_gain)
    Rs = StraightCurveTransitionTrackTemplate(False, False, grid_size=grid_size, elevation_gain=elevation_gain)

    # dumb but whatever
    all_template_names = ["s", "sSs", "lr", "L", "R", "sL", "sR", "Ls", "Rs"]
    all_templates = [s, sSs, lr, L, R, sL, sR, Ls, Rs]
    all_results = np.empty((len(all_templates),profile.shape[0],resolution,3))

    @njit
    def compute_contours(result, template, resolution, profile):
        min_z = min(profile[:,1]) - elevation_gain
        max_z = max(profile[:,1]) + elevation_gain
        for j in range(profile.shape[0]):
            pt = profile[j]
            template.set_sample_pt(pt)
            for k in range(resolution):
                t = k / (resolution - 1)
                sample = template.sample_path_t(t)
                result[j,k,0] = sample[0]
                result[j,k,1] = sample[1]
                result[j,k,2] = max(min(sample[2], max_z), min_z)

    for i in range(len(all_templates)):
        template = all_templates[i]
        print(i, all_template_names[i])

        compute_contours(all_results[i], template, resolution, profile)

    return all_template_names, all_results

def main(grid_size, elevation_gain, profile_obj_path, save_dir, resolution):
    def load_profile_obj(profile_obj_path):
        profile = []
        with open(profile_obj_path, "r") as f:
            for line in f:
                if line.startswith("v"):
                    profile.append([float(x) for x in line.strip().split()[1:3]])
        profile.append(profile[0])
        return np.array(profile)

    if not profile_obj_path:
        profile = np.array([[-12, -6], [-12, 6], [12, 6], [12, -6], [-12, -6]], dtype=np.float64)
    else:
        profile = load_profile_obj(profile_obj_path)

    names, results = create_all_tracks(grid_size, elevation_gain, profile, resolution)

    def save_obj(contours):
        print(contours.shape)
        def yield_vertex_obj_data(vertex):
            def form(x):
                return f"{x:.3f}".rjust(8, " ")
            return "v" + " ".join([form(vertex[0]), form(vertex[1]), form(vertex[2])]) + "\n"

        obj_data = ''.join(np.apply_along_axis(yield_vertex_obj_data, -1, contours).flatten())

        I, J = contours.shape[:2]
        def yield_face_obj_data(index):
            i, j = index
            return f"f {i*J+j+1} {(i+1)*J+j+1} {(i+1)*J+j+2} {i*J+j+2}\n"

        obj_data += ''.join([yield_face_obj_data(i) for i in np.ndindex(I-1, J-1)])
        return obj_data

    for i in range(results.shape[0]):
        name = names[i]
        print("saving", name)
        data = save_obj(results[i])
        with open(os.path.join(save_dir, name + ".obj"), "w") as f:
            f.write(data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid-size", type=int, default=40)
    parser.add_argument("--elevation-gain", type=int, default=6)
    parser.add_argument("--profile-obj-path", type=str)
    parser.add_argument("--save-dir", type=str, default=".")
    parser.add_argument("--resolution", type=int, default=256)

    args = parser.parse_args()
    main(args.grid_size, args.elevation_gain, args.profile_obj_path, args.save_dir, args.resolution)





