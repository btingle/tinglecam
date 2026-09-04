import numpy as np
import sys, os, argparse


# deciding to represent my profile shape with gcode
# because I can't think of a better way to represent a simple shape consisting of straight lines and radii
# want to be able to sample evenly along its perimeter/radii

class ProfileTemplate:
    def __init__(self, gcode):

        lines = gcode.strip().split("\n")
        def parse_gcode_point(line, curr_pt):
            for token in line.strip().split():
                if token.startswith("X"):
                    curr_pt[0] = float(token[1:])
                if token.startswith("Y"):
                    curr_pt[1] = float(token[1:])

        self.curves = []
        length = 0 # length of path so far
        length_circle = 0 # length of just radii so far
        for line in lines:
            token_map = { tok[0] : float(tok[1:]) for tok in line.strip().split() }
            curr_pt = [0, 0]
            if line.startswith("G00"):
                curr_pt[0] = token_map['X'] or curr_pt[0]
                curr_pt[1] = token_map['Y'] or curr_pt[1]
                self.curves.append((curr_pt, length, length_circle))
            if line.startswith("G01"):
                curr_pt = parse_gcode_point(line)
                self.curves.append((curr_pt, length, length_circle))
    def sample(t, scale=[1,1]):
        pass


class TrackTemplateOne(ProfileTemplate):
    def __init__(self):
        self.gcode = """
            G00 X-9 Y-6
            G02 X-12 Y-3 J3
            G01 X-12 Y5
            G02 X-11 Y6 I1
            G01 X-9
            G02 X-8 Y5 J-1
            G03 X8 I8
            G02 X9 Y6 I1
            G01 X11 Y6
            G02 X12 Y5 J-1
            G01 Y-3
            G02 X9 Y-6 I-3
            G01 X-9 Y-6
        """


def sample_track_template_1(t):
    pass

def track_template_1_jig(args):
    pass