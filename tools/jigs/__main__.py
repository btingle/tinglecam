"""Generate every jig for one or more track species.

    python3 -m tools.jigs lr slr sSs --out output/jigs [--params overrides.json] [--no-preview]

Per species directory:
  laser/   wood blank outlines (top sheet with fixture holes) and the acrylic glue-up stack (DXF)
  op0/     bolt-down plate (STEP + STL)
  op1/     top-side soft jaws per track type (STEP assembly + one STL per jaw)
  op2/     end-face jaws per track type (STEP assembly + one STL per jaw)
  report.md   parameters, decisions, and the self-review check table
  preview_*.png
"""
import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

import matplotlib.pyplot as plt
from OCP.Message import Message, Message_Gravity

from . import checks, occ, preview
from .laser import blank_files, glueup_files
from .layout import SPECIES, build_layout
from .machining import build_op0, build_op1, build_op2, orientation_matrix
from .params import Params
from .track import Track
from ..track_step import write_step


def export(jig, out):
    out.mkdir(parents=True, exist_ok=True)
    write_step(occ.compound(list(jig.parts.values())), out / f"{jig.name}.step")
    for k, s in jig.parts.items():
        occ.write_stl(s, out / f"{jig.name}_{k}.stl")


class Stopwatch:
    def __init__(self, label):
        self.label, self.times = label, []

    def __call__(self, stage, fn, *args):
        t = time.time()
        result = fn(*args)
        self.times.append((stage, time.time() - t))
        print(f"  {self.label} {stage}: {self.times[-1][1]:.1f}s", flush=True)
        return result


def run_species(name, p, root, tracks, make_preview=True):
    sp = SPECIES[name]
    out = root / name
    sw = Stopwatch(name)
    for m in sp.members:
        tracks.setdefault(m, Track(m, p))
        sw(f"{m} solid", lambda t=tracks[m]: t.solid)
    rep = checks.Report()
    t0 = time.time()
    layout = sw("layout", build_layout, sp, tracks, p)
    blanks, blank_note = blank_files(layout, tracks, p, out / "laser")
    glue = glueup_files(layout, tracks, p, out / "laser")
    sw("layout checks", checks.check_layout, rep, layout, tracks, p)
    sw("fixture hole checks", checks.check_fixture_holes, rep, layout, tracks, p)

    op0 = sw("op0", build_op0, layout, tracks, p)
    sw("op0 export", export, op0, out / "op0")
    sw("op0 checks", checks.check_op0, rep, op0, layout, p)

    op1s, op2s = {}, {}
    for m in sp.members:
        op1s[m] = sw(f"{m} op1", build_op1, tracks[m], p)
        sw(f"{m} op1 export", export, op1s[m], out / "op1")
        sw(f"{m} op1 checks", checks.check_op1, rep, op1s[m], tracks[m], p)
        op2s[m] = sw(f"{m} op2", build_op2, tracks[m], p)
        sw(f"{m} op2 export", export, op2s[m], out / "op2")
        sw(f"{m} op2 checks", checks.check_op2, rep, op2s[m], tracks[m], p)

    lines = [f"# {name} jigs", "", f"Members: {', '.join(sp.members)} (base outline {sp.base}). "
             f"Built in {time.time() - t0:.0f}s.", "", "## Shop notes", "", f"- {blank_note}",
             f"- Glue-up plate {layout.plate[2] * 2:.1f} x {layout.plate[3] * 2:.1f}, "
             f"{len(layout.bolts)} x M5; stack: bottom plate (hex nut sockets), "
             f"{tracks[sp.base].sheets} guides, top plate (fixture-hole side).",
             f"- op0 plate {op0.info['size'][0]:.1f} x {op0.info['size'][1]:.1f} x {op0.info['size'][2]:.1f}, "
             f"{len(op0.info['holes'])} x M4 countersunk."]
    for m in sp.members:
        a, b = op1s[m].info, op2s[m].info
        lines.append(f"- {m} op1: {a['count']} per jaw set, block {a['size'][0]:.0f} x {a['size'][1]:.1f} x "
                     f"{a['size'][2]:.1f}, pieces rotated {a['rotation']:.0f} deg.")
        lines.append(f"- {m} op2: block {b['size'][0]:.0f} x {b['size'][1]:.0f} x {b['size'][2]:.0f}, split at "
                     f"z={b['split_z']:.2f}, {len(b['bolts'])} x M3 (<= {b['bolt_length_max']:.0f} mm) into inserts.")
    lines += ["", "## Self-review", "", rep.markdown(), "", "## Parameters", "", "```json",
              json.dumps(asdict(p), indent=1), "```"]
    (out / "report.md").write_text("\n".join(lines) + "\n")

    if make_preview:
        sw("previews", write_previews, out, sp, layout, op0, op1s, op2s, tracks, p)
    return rep


def write_previews(out, sp, layout, op0, op1s, op2s, tracks, p):
    fig = plt.figure(figsize=(16, 10))
    preview.glueup(fig.add_subplot(2, 2, 1), layout, p)
    ax = fig.add_subplot(2, 2, 2, projection="3d")
    preview.solids3d(ax, [(op0.parts["plate"], "#9fb4c8")] +
                     [(b, preview.WOOD) for b in op0.context.values()],
                     f"op0 plate + blanks (mill frame)", view=(35, -60))
    ax = fig.add_subplot(2, 2, 3, projection="3d")
    preview.solids3d(ax, [(op0.parts["plate"], "#9fb4c8")], "op0 underside (countersinks)", view=(-40, -60))
    fig.tight_layout()
    fig.savefig(out / "preview_layout.png", dpi=90)
    plt.close(fig)
    for m in sp.members:
        fig = plt.figure(figsize=(18, 6))
        j1 = op1s[m]
        jaws = [(s, preview.PALETTE[i]) for i, s in enumerate(j1.parts.values())]
        pieces = [(s, preview.WOOD) for s in j1.context.values()]
        preview.solids3d(fig.add_subplot(1, 3, 1, projection="3d"), jaws + pieces,
                         f"{m} op1 jaws + pieces", view=(40, -70))
        preview.solids3d(fig.add_subplot(1, 3, 2, projection="3d"), jaws, f"{m} op1 jaws (top)", view=(89, -90))
        j2 = op2s[m]
        o = orientation_matrix(tracks[m].ends[0][1])
        parts = [(occ.transform(preview.explode(j2.parts["jaw_B"], 15), o), preview.PALETTE[1]),
                 (occ.transform(j2.parts["jaw_A"], o), preview.PALETTE[0]),
                 (occ.transform(j2.context["piece"], o), preview.WOOD)]
        preview.solids3d(fig.add_subplot(1, 3, 3, projection="3d"), parts,
                         f"{m} op2 (start face up, jaw B pulled back)", view=(25, -50))
        fig.tight_layout()
        fig.savefig(out / f"preview_{m}.png", dpi=90)
        plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("species", nargs="*", default=list(SPECIES), choices=list(SPECIES))
    ap.add_argument("--out", type=Path, default=Path("output/jigs"))
    ap.add_argument("--params", type=Path, help="JSON file overriding Params fields")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args(argv)
    p = Params.load(args.params)
    printers = Message.DefaultMessenger_s().Printers()
    for i in range(1, printers.Size() + 1):  # STEP writer chatter
        printers.Value(i).SetTraceLevel(Message_Gravity.Message_Fail)
    tracks, failed = {}, 0
    for name in args.species:
        rep = run_species(name, p, args.out, tracks, not args.no_preview)
        failed += len(rep.failures)
        print(f"{name}: {len(rep.rows) - len(rep.failures)}/{len(rep.rows)} checks ok -> {args.out / name}")
        for g, _, w, d in rep.failures:
            print(f"  FAIL {g}: {w} {d}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
