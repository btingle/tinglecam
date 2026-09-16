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

def main(grid_size, elevation_gain, profile_obj_path, save_dir, resolution, gcode, open_path, translate):
    def load_profile_obj(profile_obj_path):
        profile = []
        # profile_t for evenly sampling the toolpath- meshing doesn't really care
        profile_t = []
        with open(profile_obj_path, "r") as f:
            def dist(a, b):
                xdist = abs(a[0] - b[0])
                ydist = abs(a[1] - b[1])
                return sqrt(xdist**2+ydist**2)
            pv = None
            d = 0
            for line in f:
                if line.startswith("v"):
                    v = [float(x) for x in line.strip().split()[1:3]]
                    if pv:
                        d += dist(v, pv)
                    profile_t.append(d)
                    profile.append(v)
                    pv = v
        if not open_path:
            profile.append(profile[0])
            d += dist(profile[-1], profile[-2])
            profile_t.append(d)
        return np.array(profile), np.array(profile_t)

    profile_resolution_mm = 0.25

    if not profile_obj_path:
        profile = np.array([[-12, -6], [-12, 6], [12, 6], [12, -6], [-12, -6]], dtype=np.float64)
        profile_t = np.array([0, 12, 36, 48, 72], dtype=np.float64)
    else:
        profile, profile_t = load_profile_obj(profile_obj_path)

    profile_resolution = int(profile_t[-1] / profile_resolution_mm)
    def sample_profile(i):
        cur_d = i*profile_resolution_mm
        idx = np.searchsorted(profile_t, cur_d)
        if idx <= 0:
            idx = 1
        if idx >= profile_t.size:
            idx = profile_t.size - 1
        seg_d = profile_t[idx] - profile_t[idx-1]
        t = (cur_d - profile_t[idx-1]) / seg_d
        return (profile[idx] - profile[idx-1]) * t + profile[idx-1]

    if gcode:
        new_profile = np.array([sample_profile(i) for i in range(profile_resolution)])
        profile = new_profile

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

    def save_gcode(contours, datum=1, tool=1, spindle_speed=12000, feed=500, retract=30):
        def form_gcode(pt):
            x, y, z = pt
            return f"X{x:.3f} Y{y:.3f} Z{z:.3f}\n"
        def form_obj(pt):
            x, y, z = pt
            return f"v {x:.3f} {y:.3f} {z:.3f}\n"
        
        first_pos = contours[0,0]+translate
        gcode = f"""
G21 G90 G94 G17 G{53+datum}
M6 T{tool}
G28
G0 X{first_pos[0]} Y{first_pos[1]}
Z{retract}
M3 S{spindle_speed} F{feed}
G4 P3.0
"""
        obj = ""
        I, J = contours.shape[:2]
        for i in range(0, I, 2):
            for j in range(J):
                gcode += form_gcode(contours[i,j] + translate)
                obj += form_obj(contours[i,j] + translate)
            i += 1
            if i >= I:
                break
            for j in reversed(range(J)):
                gcode += form_gcode(contours[i,j] + translate)
                obj += form_obj(contours[i,j] + translate)
        # emit ze lines
        chunksize=1000
        for chunk in range(0, I*J, chunksize):
            obj += f"l {" ".join([str(_i+1) for _i in range(chunk, min(I*J,chunk+chunksize+1001))])}\n"
        return gcode, obj


    if gcode:

        for i in range(results.shape[0]):
            name = names[i]
            print("saving", name)
            gcode_data, obj_data = save_gcode(results[i])
            with open(os.path.join(save_dir, name + ".nc"), "w") as nc_f:
                nc_f.write(gcode_data)
            with open(os.path.join(save_dir, name + '.nc.obj'), "w") as nc_obj_f:
                nc_obj_f.write(obj_data)

    else:

        for i in range(results.shape[0]):
            name = names[i]
            print("saving", name)
            data = save_obj(results[i])
            with open(os.path.join(save_dir, name + ".obj"), "w") as f:
                f.write(data)

if __name__ == "__main__":
    def trans_type(s):
        v = [0, 0, 0]
        for i, t in enumerate(s.split(',')):
            f = float(t)
            v[i] = f
        return v
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid-size", type=int, default=40)
    parser.add_argument("--elevation-gain", type=int, default=6)
    parser.add_argument("--profile-obj-path", type=str)
    parser.add_argument("--save-dir", type=str, default=".")
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--gcode", action="store_true")
    parser.add_argument("--open-path", action="store_true")
    parser.add_argument("--translate", type=trans_type, default=[0,0,0])

    args = parser.parse_args()
    main(args.grid_size, args.elevation_gain, args.profile_obj_path, args.save_dir, args.resolution, args.gcode, args.open_path, args.translate)





