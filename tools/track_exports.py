"""OBJ meshes and serpentine G-code exports for sampled track contours."""

from pathlib import Path

import numpy as np


def _triangulate_end(points):
    """Ear-clip a simple planar end polygon, including concave marble channels."""
    centered = points - points.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    polygon = centered @ axes[:2].T
    area = np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1)
                  - polygon[:, 1] * np.roll(polygon[:, 0], -1))
    epsilon = max(float(np.ptp(polygon, axis=0).max()) ** 2 * 1e-12, 1e-14)
    if abs(area) < epsilon:
        raise ValueError("Cannot cap a zero-area profile")
    direction = np.sign(area)
    remaining = list(range(len(points)))
    triangles = []
    while len(remaining) > 3:
        for position, middle in enumerate(remaining):
            before, after = remaining[position - 1], remaining[(position + 1) % len(remaining)]
            a, b, c = polygon[[before, middle, after]]
            ab, bc, ca = b - a, c - b, a - c
            cross = ab[0] * (c - a)[1] - ab[1] * (c - a)[0]
            if cross * direction <= epsilon or (area - cross) * direction <= epsilon:
                continue
            others = [i for i in remaining if i not in (before, middle, after)]
            q = polygon[others]
            tests = [direction * (edge[0] * (q - origin)[:, 1] - edge[1] * (q - origin)[:, 0])
                     for edge, origin in ((ab, a), (bc, b), (ca, c))]
            if np.any(np.all(np.array(tests) >= -epsilon, axis=0)):
                continue
            triangles.append((before, middle, after))
            remaining.pop(position)
            area -= cross
            break
        else:
            raise ValueError("Cannot triangulate cap; check for self-intersections or duplicate profile points")
    triangles.append(tuple(remaining))
    return triangles


def save_obj(contours, cap_ends=False):
    """Serialize the profile-by-path grid, optionally welding and capping its ends."""
    if cap_ends:
        if len(contours) < 4 or not np.allclose(contours[0], contours[-1], atol=1e-8, rtol=0):
            raise ValueError("End caps require a closed profile")
        # Weld the repeated profile seam so capped meshes share every edge.
        contours = contours[:-1]
    lines = ["v" + " ".join(f"{value:.3f}".rjust(8) for value in vertex) + "\n"
             for vertex in contours.reshape(-1, 3)]
    rows, columns = contours.shape[:2]
    for i in range(rows if cap_ends else rows - 1):
        next_row = (i + 1) % rows
        for j in range(columns - 1):
            lines.append(f"f {i*columns+j+1} {next_row*columns+j+1} "
                         f"{next_row*columns+j+2} {i*columns+j+2}\n")
    if cap_ends:
        for end in (0, columns - 1):
            for triangle in _triangulate_end(contours[:, end]):
                if end == 0:
                    triangle = triangle[::-1]
                lines.append("f " + " ".join(str(i * columns + end + 1) for i in triangle) + "\n")
    return "".join(lines)


def save_gcode(contours, translate=(0, 0, 0), datum=1, tool=1,
               spindle_speed=12000, feed=500, retract=30):
    """Return the existing cutting program and an OBJ preview of its traversal.

    This retains the generator's machine commands and cutting order; it does not
    add a machine-specific program footer or change its retract strategy.
    """
    translate = np.asarray(translate, dtype=float)
    first_pos = contours[0, 0] + translate
    gcode = [f"""
G21 G90 G94 G17 G{53+datum}
M6 T{tool}
G28
G0 X{first_pos[0]} Y{first_pos[1]}
Z{retract}
M3 S{spindle_speed} F{feed}
G4 P3.0
"""]
    obj = []
    for i, row in enumerate(contours):
        for x, y, z in (row if i % 2 == 0 else row[::-1]) + translate:
            gcode.append(f"X{x:.3f} Y{y:.3f} Z{z:.3f}\n")
            obj.append(f"v {x:.3f} {y:.3f} {z:.3f}\n")
    # Adjacent chunks share one vertex, keeping a continuous polyline. Every
    # index refers to a vertex actually emitted above.
    count = contours.shape[0] * contours.shape[1]
    for first in range(0, count - 1, 1000):
        indices = " ".join(str(i + 1) for i in range(first, min(count, first + 1001)))
        obj.append(f"l {indices}\n")
    return "".join(gcode), "".join(obj)


def export_sampled_tracks(names, results, save_dir, gcode=False, translate=(0, 0, 0), cap_ends=False):
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    for name, contours in zip(names, results):
        print("saving", name)
        if gcode:
            program, preview = save_gcode(contours, translate=translate)
            (save_dir / f"{name}.nc").write_text(program)
            (save_dir / f"{name}.nc.obj").write_text(preview)
        else:
            (save_dir / f"{name}.obj").write_text(save_obj(contours, cap_ends))
